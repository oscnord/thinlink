use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, sync_channel};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use serde::{Deserialize, Serialize};

use sat_teleop_control::envelope::{CHALLENGE_EVERY, ControlLimits, ToGround};
use sat_teleop_control::telemetry::TelemetryLimits;
use sat_teleop_control::vehicle::{self, VehicleConfig, VehicleSnapshot, VehicleStats};
use sat_teleop_detail::store::{Store, StoreSettings, StoreSnapshot, Tag};
use sat_teleop_detail::vehicle::{
    self as retrieval, RetrievalLimits, RetrievalSnapshot, RetrievalStats,
};
use sat_teleop_detect::{Detection, Detector};
use sat_teleop_media::hooks::sender_hooks;
use sat_teleop_media::region::{
    RegionSettings, Regional, RegionalSnapshot, validate_soften_factor,
};
use sat_teleop_media::retrieval as media_retrieval;
use sat_teleop_media::tags::TagFeed;
use sat_teleop_policy::follow::{Follower, Luma, Step};
use sat_teleop_policy::kind::Kind;
use sat_teleop_policy::region::NormRect;
use sat_teleop_policy::retrieval::ground_heard;
use sat_teleop_policy::sighting::Seen;
use sat_teleop_video::adapt::{
    self, AdaptLimits, AdaptSnapshot, AdaptStats, LadderConfig, Rungs, Setup, Uplink,
};
use sat_teleop_video::camera;
use sat_teleop_video::sender::{self, SenderConfig, SenderSnapshot, SenderStats, Snapshot};
use sat_teleop_video::{Health, HealthSnapshot, metrics, supervise, unix_ms};
use sat_teleop_video_core::budget::{Budget, wire_bytes};

#[cfg(feature = "testbed")]
mod hooks;
mod landing;
mod process;

#[derive(Parser)]
#[command(
    name = "sat-teleop-vehicle",
    about = "Vehicle companion: stamped H.264 over RTP and gated MAVLink to the relay"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run the sender and the SITL bridge, restarting them after failures, and serve GET /metrics.
    Run {
        /// Vehicle config, for example testbed/config/vehicle.toml.
        #[arg(long)]
        config: PathBuf,
        /// Operational limits, for example testbed/limits.toml.
        #[arg(long)]
        limits: PathBuf,
        /// The adaptation ladder, for example testbed/ladder.toml.
        #[arg(long)]
        ladder: PathBuf,
        /// Start at this rung (counted from the best, 0) instead of the ladder's `start`.
        #[arg(long)]
        start_rung: Option<usize>,
        /// Address for GET /metrics, for example 192.168.233.10:9100.
        #[arg(long)]
        metrics_addr: String,
        /// A file with the bearer token the metrics endpoint (and the test hooks) need. Without one
        /// the endpoint listens on 127.0.0.1 only.
        #[arg(long)]
        token_file: Option<PathBuf>,
        /// A file with the relay's job API token, which the vehicle presents to the relay. Without
        /// one the job API is not used. A token of its own: the metrics token never leaves the
        /// vehicle.
        #[arg(long)]
        job_token_file: Option<PathBuf>,
    },
    /// List the video sources `GStreamer` sees and the modes each offers, to choose the `[source]` of
    /// a vehicle config for a camera.
    ProbeCamera,
}

/// The token files of the command line.
struct Tokens<'a> {
    metrics: Option<&'a Path>,
    job: Option<&'a Path>,
}

struct Config {
    sender: SenderConfig,
    mavlink: VehicleConfig,
    /// The relay's feedback endpoint.
    feedback: SocketAddr,
    /// The test-only hooks (`hooks`) answer only when this is set; testbed builds only.
    #[cfg(feature = "testbed")]
    debug_hooks: bool,
    /// The snapshot store; required with a `[snapshot]` branch.
    store: Option<StoreSettings>,
    /// The relay's job API (`http://host:port`); without it nothing is retrieved.
    detail: Option<String>,
    /// The person detector; without it nobody is outlined.
    detector: Option<DetectorConfig>,
    /// Block size of the softened area outside the pilot's box and the people found; 1 is off.
    soften_factor: u8,
    /// The landing target switch, when it is on.
    landing: Option<landing::On>,
    /// Whether the pilot's box is followed from frame to frame.
    follow: bool,
}

/// `[region]` of the vehicle config. The limits are in `[region]` of the limits file.
#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct RegionConfig {
    detector: Option<DetectorConfig>,
    /// 1 (the default) is off: the pilot's box is not kept and nothing is softened. 2 to 4 soften.
    soften_factor: Option<u8>,
    /// Follow the pilot's box from frame to frame (off by default).
    #[serde(default)]
    follow: bool,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct DetectorConfig {
    /// The ONNX model, for example /usr/share/sat-teleop/person.onnx.
    model: PathBuf,
    /// The model's square input size in pixels, `DETECTOR_INPUT`.
    input: u32,
    /// What counts: person, bicycle, car, motorcycle, bus, truck or boat. People alone by default.
    #[serde(default = "default_kinds")]
    kinds: Vec<Kind>,
}

fn default_kinds() -> Vec<Kind> {
    vec![Kind::Person]
}

/// What a model's input may be: the capsfilter and each frame's buffer are built from it.
const DETECTOR_INPUT: std::ops::RangeInclusive<u32> = 32..=1280;

impl DetectorConfig {
    fn validate(&self) -> Result<()> {
        if !DETECTOR_INPUT.contains(&self.input) {
            anyhow::bail!(
                "the detector's input must be {}-{} pixels, not {}",
                DETECTOR_INPUT.start(),
                DETECTOR_INPUT.end(),
                self.input
            );
        }
        if self.kinds.is_empty() {
            anyhow::bail!("the detector's kinds must name at least one kind");
        }
        Ok(())
    }
}

/// Snapshots the sender may have waiting for the writer; more are dropped, never the live frames.
const WRITER_QUEUE: usize = 2;
/// What `POST /snapshots/slow` makes each write take: longer than the snapshot interval.
#[cfg(feature = "testbed")]
const SLOW_WRITE: Duration = Duration::from_secs(3);

/// The store and the writer's counters.
struct Retained {
    store: Mutex<Store>,
    write_failed: AtomicU64,
    /// Test only: each write waits this long first.
    write_delay_ms: AtomicU64,
}

#[derive(Serialize)]
struct RetainedMetrics {
    store: StoreSnapshot,
    write_failed: u64,
    write_delay_ms: u64,
}

impl Retained {
    fn metrics(&self) -> RetainedMetrics {
        RetainedMetrics {
            store: self
                .store
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .snapshot(),
            write_failed: self.write_failed.load(Ordering::Relaxed),
            write_delay_ms: self.write_delay_ms.load(Ordering::Relaxed),
        }
    }

    /// Writes what the sender hands over, one at a time, until the sender is gone; `tag` says
    /// where the vehicle is and whether the ground is heard at the moment of the write.
    fn write(&self, snapshots: &Receiver<Snapshot>, tag: impl Fn() -> Tag) {
        for snapshot in snapshots {
            let delay = self.write_delay_ms.load(Ordering::Relaxed);
            thread::sleep(Duration::from_millis(delay));
            let mut store = self.store.lock().unwrap_or_else(PoisonError::into_inner);
            let stored = store.put(
                snapshot.capture_ms,
                snapshot.frame_id,
                &snapshot.jpeg,
                tag(),
                unix_ms(),
                Instant::now(),
            );
            drop(store);
            if let Err(e) = stored {
                eprintln!("cannot store a snapshot: {e}");
                self.write_failed.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

/// What the vehicle reads from `testbed/limits.toml`; the other applications' tables are theirs.
#[derive(Deserialize)]
struct Limits {
    telemetry: TelemetryLimits,
    control: ControlLimits,
    adapt: AdaptLimits,
    retrieval: RetrievalLimits,
    region: RegionSettings,
}

#[derive(Serialize)]
struct Metrics {
    #[serde(flatten)]
    video: SenderSnapshot,
    #[serde(flatten)]
    regional: RegionalSnapshot,
    process: Option<process::ProcessSnapshot>,
    mavlink: VehicleSnapshot,
    tag: Option<landing::TagSnapshot>,
    adapt: AdaptMetrics,
    retained: Option<RetainedMetrics>,
    retrieval: RetrievalSnapshot,
}

#[derive(Serialize)]
struct AdaptMetrics {
    #[serde(flatten)]
    state: AdaptSnapshot,
    #[serde(flatten)]
    health: HealthSnapshot,
}

fn main() -> Result<()> {
    match Cli::parse().command {
        Command::Run {
            config,
            limits,
            ladder,
            start_rung,
            metrics_addr,
            token_file,
            job_token_file,
        } => run(
            &config,
            &limits,
            &ladder,
            start_rung,
            &metrics_addr,
            &Tokens {
                metrics: token_file.as_deref(),
                job: job_token_file.as_deref(),
            },
        ),
        Command::ProbeCamera => probe_camera(),
    }
}

fn probe_camera() -> Result<()> {
    sat_teleop_video::init().context("GStreamer")?;
    let devices = camera::probe().context("device monitor")?;
    if devices.is_empty() {
        println!(
            "no video source found: is the camera plugged in, and is the user in the video group?"
        );
    }
    for device in devices {
        println!("{device}");
    }
    Ok(())
}

fn run(
    config: &Path,
    limits: &Path,
    ladder: &Path,
    start_rung: Option<usize>,
    metrics_addr: &str,
    tokens: &Tokens,
) -> Result<()> {
    let Config {
        sender,
        mavlink,
        feedback,
        #[cfg(feature = "testbed")]
        debug_hooks,
        store,
        detail,
        detector,
        soften_factor,
        landing: landing_on,
        follow,
    } = load(config)?;
    let token = tokens
        .metrics
        .map(metrics::load_token)
        .transpose()
        .context("token file")?;
    let job_token = tokens
        .job
        .map(metrics::load_token)
        .transpose()
        .context("job token file")?;
    let limits = load_limits(limits)?;
    let mut rungs = load_ladder(ladder)?;
    rungs.start = start_rung.unwrap_or(rungs.start);
    #[cfg(feature = "testbed")]
    let ladder = rungs.encodings.clone();
    let allowance = limits.retrieval.allowance_bytes_per_s;
    let adapt_limits = limits.adapt;
    let (probe_every, thresholds) = limits
        .adapt
        .validate()
        .context("the limits file has invalid [adapt] limits")?;
    let start = rungs
        .start_encoding()
        .context("the ladder has no start rung")?;
    let setup = Setup {
        relay: feedback,
        probe_every,
        thresholds,
        budget: Budget {
            telemetry_percent: limits.telemetry.offered_percent,
            telemetry_floor: limits.telemetry.essential_bytes_per_s,
            telemetry_cap: limits.telemetry.bytes_per_s,
            control: control_reserve(probe_every),
        },
        rungs,
    };
    sat_teleop_video::init().context("GStreamer")?;
    let retained = store
        .map(|settings| open_store(settings, sender.snapshot.is_some()))
        .transpose()?
        .map(Arc::new);
    let tag_feed = landing_on
        .as_ref()
        .map(|on| TagFeed::new(on.fps, on.width))
        .transpose()?;
    let bridge = Arc::new(VehicleStats::new(
        &mavlink,
        landing_on.as_ref().map(|on| on.setup),
    ));
    let tag_stats = Arc::new(landing::TagStats::default());
    if let (Some(feed), Some(on)) = (&tag_feed, &landing_on) {
        let (feed, bridge, tag_stats, id) = (
            Arc::clone(feed),
            Arc::clone(&bridge),
            Arc::clone(&tag_stats),
            on.tag_id(),
        );
        thread::spawn(move || landing::find_tags(&feed, id, &bridge, &tag_stats));
    }
    let adaptation = Arc::new((AdaptStats::default(), Health::default()));
    let (snapshots, writer_queue) = sync_channel(WRITER_QUEUE);
    if let Some(retained) = &retained {
        let (retained, bridge, adaptation) = (
            Arc::clone(retained),
            Arc::clone(&bridge),
            Arc::clone(&adaptation),
        );
        thread::spawn(move || {
            retained.write(&writer_queue, || Tag {
                position: bridge.position.fresh(Instant::now()),
                outage: !heard(&adaptation, &adapt_limits, allowance),
            });
        });
    }
    // The model loads before the sender starts, so a model that cannot load leaves the detector's
    // branch out of the pipeline altogether.
    let loaded = detector
        .as_ref()
        .map(|config| (config.input, load_detector(config)));
    let input = loaded
        .as_ref()
        .and_then(|(input, loaded)| loaded.is_ok().then_some(*input));
    let regional = Regional::new(
        limits.region,
        bridge.control.region.clone(),
        input,
        soften_factor,
        follow,
    );
    if follow {
        let regional = Arc::clone(&regional);
        thread::spawn(move || follow_things(&regional));
    }
    match loaded {
        Some((_, Ok(detector))) => {
            let regional = Arc::clone(&regional);
            let (bridge, adaptation) = (Arc::clone(&bridge), Arc::clone(&adaptation));
            thread::spawn(move || {
                find_things(&regional, detector, |found, capture_ms| {
                    if !heard(&adaptation, &adapt_limits, allowance) {
                        note_sightings(&bridge, found, capture_ms);
                    }
                });
            });
        }
        Some((_, Err(cause))) => {
            eprintln!("running without the detector: {cause}");
            regional.detector_failed(&cause);
        }
        None => {}
    }
    let video = Arc::new(SenderStats::default());
    video.request(start);
    {
        let video = Arc::clone(&video);
        let hooks = sender_hooks(&regional, tag_feed.as_ref());
        thread::spawn(move || {
            supervise(&video.health, || {
                sender::run(&sender, &video, &hooks, &snapshots)
            });
        });
    }
    {
        let (bridge, adaptation) = (Arc::clone(&bridge), Arc::clone(&adaptation));
        thread::spawn(move || {
            let ground_heard = || heard(&adaptation, &adapt_limits, allowance);
            let run = vehicle::run(
                mavlink,
                limits.telemetry,
                limits.control,
                ground_heard,
                &bridge,
            );
            if let Err(e) = run {
                eprintln!("mavlink bridge stopped: {e}");
                bridge.sitl.down(&e);
            }
        });
    }
    let job = Arc::new(RetrievalStats::default());
    {
        let (video, bridge, adaptation, job) = (
            Arc::clone(&video),
            Arc::clone(&bridge),
            Arc::clone(&adaptation),
            Arc::clone(&job),
        );
        thread::spawn(move || {
            let (stats, health) = &*adaptation;
            supervise(health, || {
                adapt::run(
                    &setup,
                    &video,
                    || uplink(&bridge),
                    |cap| bridge.set_telemetry_cap(cap),
                    || job.reserved(),
                    allowance,
                    stats,
                )
            });
        });
    }
    if let (Some(retained), Some(relay)) = (&retained, detail) {
        let (retained, bridge, adaptation, job) = (
            Arc::clone(retained),
            Arc::clone(&bridge),
            Arc::clone(&adaptation),
            Arc::clone(&job),
        );
        thread::spawn(move || {
            let conditions = || {
                media_retrieval::conditions(&adaptation.0, Instant::now(), &adapt_limits, allowance)
            };
            retrieval::run(
                &relay,
                &retained.store,
                &bridge.sightings,
                allowance,
                conditions,
                &job,
                job_token.as_ref(),
            );
        });
    }
    let snapshot = || Metrics {
        video: video.snapshot(),
        regional: regional.snapshot(),
        process: process::snapshot(),
        mavlink: bridge.snapshot(),
        tag: landing_on.as_ref().map(|_| tag_stats.snapshot()),
        adapt: AdaptMetrics {
            state: adaptation.0.snapshot(),
            health: adaptation.1.snapshot(),
        },
        retained: retained.as_ref().map(|retained| retained.metrics()),
        retrieval: job.snapshot(),
    };
    #[cfg(feature = "testbed")]
    let clock = bridge.control.clock();
    #[cfg(feature = "testbed")]
    let route = |method: &str, path: &str| {
        let hooks = hooks::Hooks {
            enabled: debug_hooks,
            video: &video,
            retained: retained.as_deref(),
            clock: clock.as_deref(),
            bridge: &bridge,
            ladder: &ladder,
        };
        hooks.answer(method, path)
    };
    #[cfg(not(feature = "testbed"))]
    let route = |_: &str, _: &str| None;
    metrics::serve_with(metrics_addr, token.as_ref(), snapshot, route)
        .context("metrics endpoint stopped")
}

/// Whether the relay answers the vehicle's probes, so the ground can be heard.
fn heard(adaptation: &(AdaptStats, Health), limits: &AdaptLimits, allowance: u32) -> bool {
    let conditions = media_retrieval::conditions(&adaptation.0, Instant::now(), limits, allowance);
    ground_heard(&conditions)
}

/// Loads the detector's model; the error is the cause the metrics show.
fn load_detector(config: &DetectorConfig) -> Result<Detector, String> {
    Detector::load(&config.model, config.input, config.kinds.clone()).map_err(|e| e.to_string())
}

/// Runs the detector on the newest frame the sender offers, over and over, and publishes what it
/// finds; `seen` is also told of it, with the UNIX time the frame was captured.
fn find_things(regional: &Regional, mut detector: Detector, seen: impl Fn(&[Detection], u64)) {
    regional.detector_running();
    loop {
        let frame = regional.next_frame();
        let started = Instant::now();
        let found = detector.detect(
            &frame.rgb,
            frame.width,
            frame.height,
            regional.detector_min_score(),
        );
        let age_ms = regional.cell.now_ms().saturating_sub(frame.at_ms);
        seen(&found, unix_ms().saturating_sub(age_ms));
        let boxes: Vec<(Kind, NormRect)> = found.iter().map(|d| (d.kind, d.rect)).collect();
        regional.detected(&boxes, frame.at_ms, started.elapsed());
    }
}

/// Follows the pilot's box on the newest frame the sender offers, over and over, and tells the
/// cell where the target is or that it is lost.
fn follow_things(regional: &Regional) {
    let mut follower = Follower::new(regional.follow_settings());
    loop {
        let frame = regional.next_follow_frame();
        let Some(luma) = Luma::new(&frame.luma) else {
            continue;
        };
        let started = Instant::now();
        let order = regional.cell.follow_order();
        let detections: Vec<NormRect> = regional
            .cell
            .detected()
            .into_iter()
            .map(|(_, rect)| rect)
            .collect();
        let step = follower.step(luma, order, &detections);
        let Some(order) = order else {
            continue;
        };
        match step {
            Step::Moved(rect, score) => {
                regional
                    .cell
                    .follow_moved(order.generation, rect, frame.at_ms);
                regional.followed_frame(score, started.elapsed());
            }
            Step::Held(score) => regional.followed_frame(score, started.elapsed()),
            Step::Lost(why) => regional.cell.follow_lost(order.generation, why),
            Step::Idle => {}
        }
    }
}

/// Records `found` in the frame captured at `capture_ms` as sightings, where the drone is now.
/// Without a fresh position there is nowhere to say they were.
fn note_sightings(bridge: &VehicleStats, found: &[Detection], capture_ms: u64) {
    let Some(position) = bridge.position.fresh(Instant::now()) else {
        return;
    };
    let mut sightings = bridge
        .sightings
        .lock()
        .unwrap_or_else(PoisonError::into_inner);
    for detection in found {
        sightings.record(Seen {
            kind: detection.kind,
            capture_ms,
            position,
            rect: detection.rect,
            score: detection.score,
        });
    }
}

/// The probes and the control endpoint's challenges on the wire, per second: reserved in the uplink
/// budget before video.
fn control_reserve(probe_every: Duration) -> u32 {
    let per_second =
        |every: Duration| u32::try_from(1000 / every.as_millis().max(1)).unwrap_or(u32::MAX);
    let probes = per_second(probe_every).saturating_mul(adapt::probe_wire_bytes());
    let challenges =
        per_second(CHALLENGE_EVERY).saturating_mul(wire_bytes(largest_challenge_len()));
    probes.saturating_add(challenges)
}

/// The longest challenge the control endpoint sends: every number at its widest.
fn largest_challenge_len() -> u32 {
    let challenge = ToGround::Challenge {
        epoch: u64::MAX,
        token: u64::MAX,
        lifetime_ms: u64::MAX,
    };
    let len = serde_json::to_vec(&challenge).map_or(0, |bytes| bytes.len());
    u32::try_from(len).unwrap_or(u32::MAX)
}

/// What the vehicle sent besides video and probes: telemetry frames and every reply of the
/// control endpoint, and what SITL offered.
fn uplink(bridge: &VehicleStats) -> Uplink {
    let snapshot = bridge.snapshot();
    let telemetry = snapshot.telemetry;
    let control = snapshot.control.counts;
    Uplink {
        telemetry_offered: telemetry.bytes_in,
        telemetry_datagrams: telemetry.messages_out,
        telemetry_bytes: telemetry.bytes_out,
        replies: control.replies,
        reply_bytes: control.reply_bytes,
    }
}

/// Opens the snapshot store: scans the directory, deletes the `.tmp` files a crash left and trims
/// to the limits.
fn open_store(settings: StoreSettings, branch: bool) -> Result<Retained> {
    let config = settings
        .validate()
        .context("the [store] table is invalid")?;
    let store = Store::open(config, unix_ms(), Instant::now()).context("cannot open the store")?;
    let recovery = store.snapshot().recovery;
    eprintln!(
        "snapshot store: {} kept, {} .tmp removed, {} names ignored{}",
        recovery.kept,
        recovery.tmp_removed,
        recovery.ignored,
        if branch {
            ""
        } else {
            " (no [snapshot] branch)"
        }
    );
    Ok(Retained {
        store: Mutex::new(store),
        write_failed: AtomicU64::new(0),
        write_delay_ms: AtomicU64::new(0),
    })
}

fn load(path: &Path) -> Result<Config> {
    let text =
        fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let mut table: toml::Table =
        toml::from_str(&text).with_context(|| format!("{} is not TOML", path.display()))?;
    #[cfg(not(feature = "testbed"))]
    refuse_test_hooks(&table, path)?;
    let mavlink = table
        .remove("mavlink")
        .with_context(|| format!("{} has no [mavlink] table", path.display()))?;
    let mavlink: VehicleConfig = mavlink
        .try_into()
        .with_context(|| format!("{} has an invalid [mavlink] table", path.display()))?;
    let mavlink = mavlink
        .validate()
        .with_context(|| format!("{} has an invalid [mavlink] table", path.display()))?;
    let feedback = table
        .remove("feedback")
        .with_context(|| format!("{} has no feedback address", path.display()))?
        .try_into()
        .with_context(|| format!("{}: feedback must be a host:port", path.display()))?;
    #[cfg(feature = "testbed")]
    let debug_hooks = test_hooks(&mut table, &mavlink, path)?;
    let store = table
        .remove("store")
        .map(toml::Value::try_into)
        .transpose()
        .with_context(|| format!("{} has an invalid [store] table", path.display()))?;
    let detail = match table.remove("detail") {
        None => None,
        Some(value) => Some(value.try_into().with_context(|| {
            format!(
                "{}: detail must be the relay's http://host:port",
                path.display()
            )
        })?),
    };
    let region: RegionConfig = table
        .remove("region")
        .map(toml::Value::try_into)
        .transpose()
        .with_context(|| format!("{} has an invalid [region] table", path.display()))?
        .unwrap_or_default();
    if let Some(detector) = &region.detector {
        detector
            .validate()
            .with_context(|| format!("{} has an invalid [region] table", path.display()))?;
    }
    let soften_factor = validate_soften_factor(region.soften_factor.unwrap_or(1))
        .with_context(|| format!("{} has an invalid [region] table", path.display()))?;
    let camera: landing::CameraConfig = table
        .remove("camera")
        .map(toml::Value::try_into)
        .transpose()
        .with_context(|| format!("{} has an invalid [camera] table", path.display()))?
        .unwrap_or_default();
    let landing_config: landing::LandingConfig = table
        .remove("landing")
        .map(toml::Value::try_into)
        .transpose()
        .with_context(|| format!("{} has an invalid [landing] table", path.display()))?
        .unwrap_or_default();
    let landing = landing::resolve(&camera, &landing_config, &mavlink)
        .with_context(|| format!("{} has an invalid landing target setup", path.display()))?;
    let sender: SenderConfig = table
        .try_into()
        .with_context(|| format!("{} is not a sender config", path.display()))?;
    if sender.snapshot.is_some() && store.is_none() {
        anyhow::bail!("{}: [snapshot] needs a [store] table", path.display());
    }
    Ok(Config {
        sender,
        mavlink,
        feedback,
        #[cfg(feature = "testbed")]
        debug_hooks,
        store,
        detail,
        detector: region.detector,
        follow: region.follow,
        soften_factor,
        landing,
    })
}

/// Whether the config turns the test hooks on. Real mode takes them only together with
/// `sitl_in_real_mode` (the testbed's SITL stand-in), never on their own, and `sitl_in_real_mode`
/// needs them.
#[cfg(feature = "testbed")]
fn test_hooks(table: &mut toml::Table, mavlink: &VehicleConfig, path: &Path) -> Result<bool> {
    use sat_teleop_control::identity::Mode;

    let enabled = match table.remove("debug_hooks") {
        None => false,
        Some(value) => value
            .as_bool()
            .with_context(|| format!("{}: debug_hooks must be true or false", path.display()))?,
    };
    if mavlink.sitl_in_real_mode && !enabled {
        anyhow::bail!(
            "{}: sitl_in_real_mode exists for testing only and needs debug_hooks = true",
            path.display()
        );
    }
    if enabled && mavlink.mode == Mode::Real && !mavlink.sitl_in_real_mode {
        anyhow::bail!(
            "{}: debug_hooks in real mode would let a request switch the interlocks off; only the testbed's sitl_in_real_mode may use them there",
            path.display()
        );
    }
    Ok(enabled)
}

/// A build without the `testbed` feature has no test hooks and no SITL allowance in real mode: a
/// config that names either is refused before anything starts.
#[cfg(not(feature = "testbed"))]
fn refuse_test_hooks(table: &toml::Table, path: &Path) -> Result<()> {
    let in_mavlink = |key: &str| {
        table
            .get("mavlink")
            .and_then(toml::Value::as_table)
            .is_some_and(|mavlink| mavlink.contains_key(key))
    };
    for key in ["debug_hooks", "sitl_in_real_mode"] {
        if table.contains_key(key) || in_mavlink(key) {
            anyhow::bail!(
                "{}: this build has no test hooks, so `{key}` is not accepted: remove it from the config (only the testbed's builds have them)",
                path.display()
            );
        }
    }
    Ok(())
}

fn load_ladder(path: &Path) -> Result<Rungs> {
    let text =
        fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let ladder: LadderConfig =
        toml::from_str(&text).with_context(|| format!("{} is not a ladder", path.display()))?;
    ladder
        .validate()
        .with_context(|| format!("{} is an invalid ladder", path.display()))
}

fn load_limits(path: &Path) -> Result<Limits> {
    let text =
        fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let limits: Limits = toml::from_str(&text)
        .with_context(|| format!("{} is not a limits file", path.display()))?;
    Ok(Limits {
        telemetry: limits
            .telemetry
            .validate()
            .with_context(|| format!("{} has an invalid telemetry budget", path.display()))?,
        control: limits
            .control
            .validate()
            .with_context(|| format!("{} has invalid control limits", path.display()))?,
        adapt: limits.adapt,
        retrieval: limits
            .retrieval
            .validate()
            .with_context(|| format!("{} has invalid retrieval limits", path.display()))?,
        region: limits
            .region
            .validate()
            .with_context(|| format!("{} has invalid [region] limits", path.display()))?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn load_text(text: &str) -> Result<Config> {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let path = std::env::temp_dir().join(format!(
            "sat-teleop-vehicle-{}-{}.toml",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::write(&path, text).unwrap();
        let loaded = load(&path);
        fs::remove_file(&path).unwrap();
        loaded
    }

    fn config(name: &str) -> String {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        fs::read_to_string(dir.join(name)).unwrap()
    }

    #[test]
    fn the_deployment_config_loads() {
        load_text(&config("deploy/vehicle/vehicle.toml")).unwrap();
    }

    #[test]
    fn limits_and_ladder_load() {
        let testbed = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testbed");
        let limits = load_limits(&testbed.join("limits.toml")).unwrap();
        limits.adapt.validate().unwrap();
        load_ladder(&testbed.join("ladder.toml")).unwrap();
    }

    #[test]
    fn a_detector_input_outside_32_to_1280_is_refused() {
        let base = config("deploy/vehicle/vehicle.toml");
        for (input, accepted) in [(0, false), (31, false), (320, true), (5000, false)] {
            let text = format!(
                "{base}\n[region]\ndetector = {{ model = \"/m.onnx\", input = {input} }}\n"
            );
            assert_eq!(load_text(&text).is_ok(), accepted, "input {input}");
        }
    }

    #[test]
    fn detector_kinds_default_to_people_and_an_unknown_one_is_refused() {
        let base = config("deploy/vehicle/vehicle.toml");
        let with = |kinds: &str| {
            load_text(&format!(
                "{base}\n[region]\ndetector = {{ model = \"/m.onnx\", input = 320{kinds} }}\n"
            ))
        };
        let kinds = |text: &str| with(text).unwrap().detector.unwrap().kinds;
        assert_eq!(kinds(""), [Kind::Person]);
        assert_eq!(
            kinds(", kinds = [\"person\", \"car\"]"),
            [Kind::Person, Kind::Car]
        );
        assert!(with(", kinds = [\"tank\"]").is_err());
        assert!(with(", kinds = []").is_err());
    }

    #[test]
    fn the_landing_target_is_off_unless_the_config_names_a_camera_and_turns_it_on() {
        let base = config("deploy/vehicle/vehicle.toml");
        let real = |require: &str| {
            let mode = format!("[mavlink]\nmode = \"real\"\nrequire = [{require}]\n");
            base.replacen("[mavlink]", &mode, 1)
        };
        let precland = r#"
            { name = "PLND_ENABLED", type = "int8", value = 1 },
            { name = "PLND_TYPE", type = "int8", value = 1 },
            { name = "PLND_YAW_ALIGN", type = "real32", value = 0 },
            { name = "PLND_STRICT", type = "int8", value = @STRICT@ },
            { name = "PLND_ALT_MIN", type = "real32", value = 0 },
            { name = "PLND_ALT_MAX", type = "real32", value = 0 },"#;
        let on = "\n[landing]\ntarget = true\n";
        let down = "\n[camera]\nhfov_deg = 60\ntilt_deg = -90\n";
        let cases = [
            ("default", base.clone(), true, false),
            ("on, fixed camera", format!("{base}{on}{down}"), true, true),
            ("on without a camera", format!("{base}{on}"), false, false),
            (
                "on, gimbal",
                format!("{base}{on}\n[camera]\nhfov_deg = 60\n"),
                true,
                true,
            ),
            (
                "a tilt that is not an angle",
                format!("{base}{on}\n[camera]\nhfov_deg = 60\ntilt_deg = -120\n"),
                false,
                false,
            ),
            (
                "a fixed camera tilted off straight down",
                format!("{base}{on}\n[camera]\nhfov_deg = 60\ntilt_deg = -80\n"),
                false,
                false,
            ),
            (
                "no frames to search",
                format!("{base}{on}tag_fps = 0\n{down}"),
                false,
                false,
            ),
            (
                "real mode with retry and a straight-down landing",
                format!("{}{on}{down}", real(&precland.replace("@STRICT@", "1"))),
                true,
                true,
            ),
            (
                "real mode with a gimbal, whose frame is verified only in the simulation",
                format!(
                    "{}{on}\n[camera]\nhfov_deg = 60\n",
                    real(&precland.replace("@STRICT@", "1"))
                ),
                false,
                false,
            ),
            (
                "real mode that holds with no end",
                format!("{}{on}{down}", real(&precland.replace("@STRICT@", "2"))),
                false,
                false,
            ),
            (
                "real mode without the parameters",
                format!(
                    "{}{on}{down}",
                    real(r#"{ name = "FS_GCS_ENABLE", type = "int8", value = 5 }"#)
                ),
                false,
                false,
            ),
        ];
        for (name, text, loads, enabled) in cases {
            let loaded = load_text(&text);
            assert_eq!(loaded.is_ok(), loads, "{name}: {:?}", loaded.as_ref().err());
            if let Ok(config) = loaded {
                assert_eq!(config.landing.is_some(), enabled, "{name}");
            }
        }
    }

    #[test]
    fn soften_factor_is_off_by_default_and_refused_above_4() {
        let base = config("deploy/vehicle/vehicle.toml");
        for (table, accepted) in [
            ("", true),
            ("soften_factor = 2", true),
            ("soften_factor = 5", false),
            ("soften_factor = 0", false),
        ] {
            let text = format!("{base}\n[region]\n{table}\n");
            assert_eq!(load_text(&text).is_ok(), accepted, "{table}");
        }
        assert_eq!(load_text(&base).unwrap().soften_factor, 1);
    }

    #[test]
    fn a_missing_model_is_reported_with_its_path() {
        let config = DetectorConfig {
            model: PathBuf::from("/nonexistent/person.onnx"),
            input: 320,
            kinds: default_kinds(),
        };
        let cause = load_detector(&config).err().unwrap();
        assert!(cause.contains("/nonexistent/person.onnx"), "{cause}");
    }

    #[cfg(feature = "testbed")]
    #[test]
    fn the_testbed_configs_load() {
        for name in ["vehicle", "vehicle-real"] {
            load_text(&config(&format!("testbed/config/{name}.toml"))).unwrap();
        }
    }

    /// Real mode takes the hooks only on the testbed's SITL stand-in, and that needs the hooks.
    #[cfg(feature = "testbed")]
    #[test]
    fn real_mode_and_the_hooks_go_together_only_for_the_stand_in() {
        let real = config("testbed/config/vehicle-real.toml");
        let cases = [
            (
                real.replace("debug_hooks = true", "debug_hooks = false"),
                "sitl_in_real_mode",
            ),
            (
                real.replace("sitl_in_real_mode = true", "")
                    .replace("clock = \"shared\"", "clock = \"chrony\""),
                "debug_hooks in real mode",
            ),
        ];
        for (text, message) in cases {
            let error = load_text(&text).err().map(|e| e.to_string());
            assert!(error.is_some_and(|e| e.contains(message)), "{message}");
        }
    }

    /// The release binary refuses a config that names the hooks, with a message that says why.
    #[cfg(not(feature = "testbed"))]
    #[test]
    fn a_build_without_the_testbed_feature_refuses_the_hooks() {
        let cases = [
            ("testbed/config/vehicle.toml", "debug_hooks"),
            ("testbed/config/vehicle-real.toml", "debug_hooks"),
        ];
        for (name, key) in cases {
            let error = load_text(&config(name)).err().map(|e| e.to_string());
            assert!(error.is_some_and(|e| e.contains("no test hooks") && e.contains(key)));
        }
        let allowance = config("deploy/vehicle/vehicle.toml")
            .replace("[mavlink]", "[mavlink]\nsitl_in_real_mode = true");
        let error = load_text(&allowance).err().map(|e| e.to_string());
        assert!(error.is_some_and(|e| e.contains("sitl_in_real_mode")));
    }
}
