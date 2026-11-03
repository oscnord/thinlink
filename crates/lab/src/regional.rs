//! `lab check regional`: what regional video gains, and what it costs outside the box. The vehicle
//! plays a still test card (`card`: a landing pad and a person-sized figure inside the box, another
//! figure, thin lines and blocks outside it, 1280x720) at its top rung from a one-frame y4m clip the check
//! writes to the vehicle's volume; `checks` plays the ground's output and scores each decoded frame
//! against the card scaled to the output's size (`sat_teleop_video::card`). In three windows of the
//! same length:
//!
//! 1. a box is tracked the way `QGroundControl` asks for it (`CAMERA_TRACK_RECTANGLE` to the
//!    adapter's camera component, after `SET_MESSAGE_INTERVAL` for the tracking status), and the
//!    vehicle softens everything outside it;
//! 2. tracking is stopped: the plain picture at the same rung;
//! 3. the plain picture one rung down (the vehicle's test hook `POST /video/rung/1`).
//!
//! A fourth window, with the box tracked again, scores the detail inset the ground draws from the
//! crops it fetches for the box (`detail`): the inset's SSIM against the card's region at full
//! resolution, and the live picture's SSIM in the same region.
//!
//! The metrics are the box's SSIM with and without regional video and the gain, the SSIM outside
//! the box against the rung below, and the share of local contrast kept outside the box relative
//! to the rung below. `docs/testbed.md#regional-check` defines them.

pub(crate) mod card;
mod detail;
pub(crate) mod ssim;

use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{COMMAND_LONG_DATA, MavCmd, MavMessage};
use serde_json::{Value, json};

use sat_teleop_control::ground::LeaseStatus;
use sat_teleop_video::card::{self as reference_card, Luma};
use sat_teleop_video::receiver::Source;
use sat_teleop_video::{Health, supervise};
use sat_teleop_video_core::stamp::plausible_age;

use self::detail::Detail;
use self::ssim::{Region, Scores};
use crate::adaptation;
use crate::budget::{self, Budget, Budgets};
use crate::check::{self, Results, Verdict, percentile};
use crate::client::AgentClient;
use crate::control::take_lease;
use crate::mavlink::gcs::{CAMERA_COMPONENT, Gcs, Kind};

/// The vehicle's source and the ground output's size (`testbed/config/ground.toml`).
const SOURCE_SIZE: (u16, u16) = (1280, 720);
const OUTPUT_SIZE: (u16, u16) = (640, 360);
/// The tracked box, as `QGroundControl` sends it: fractions of the picture.
const BOX: [f32; 4] = [0.35, 0.3, 0.65, 0.7];
/// `CAMERA_TRACKING_IMAGE_STATUS`.
const TRACKING_STATUS_ID: f32 = 275.0;
/// Rows scored: the vehicle's stamp is at the top and the ground's label at the bottom.
const SCORED_ROWS: (f32, f32) = (0.2, 0.85);
/// Windows this close to the box belong to neither group (output pixels).
const EDGE_MARGIN: usize = 16;
/// Every n-th decoded frame is scored; the others cost nothing.
const SCORE_EVERY: u64 = 3;
const FIRST_FRAME_DEADLINE: Duration = Duration::from_secs(30);
const HEARTBEAT_DEADLINE: Duration = Duration::from_secs(60);
/// A GEO round trip, the adapter's status interval and the vehicle's refresh, with margin.
const TRACKING_DEADLINE: Duration = Duration::from_secs(20);
const RUNG_DEADLINE: Duration = Duration::from_secs(20);
/// From the box to the first frame with the inset: the ground asks every 4 s, and a crop crosses
/// a GEO round trip several times (request, the vehicle's pickup, the chunks, the poll).
const DETAIL_DEADLINE: Duration = Duration::from_secs(90);
/// The inset must be in this share of the detail window's frames, or the window did not measure it.
const MIN_INSET_SHARE: f64 = 0.9;
/// The inset's SSIM must be this far above the live picture's in the same region.
const MIN_DETAIL_GAIN: f64 = 0.05;
const MIN_FRAMES: usize = 10;
/// The ground's output adds its own hops to the vehicle's frame age (`qgc_status`).
const OUTPUT_ALLOWANCE_MS: f64 = 100.0;
/// The vehicle's first rung, and each rung's bitrate in `testbed/ladder.toml`, by which a rung is
/// told apart from the next (two share a width).
const TOP_RUNG: usize = 0;
const RUNG_KBIT: [u64; 5] = [1200, 500, 250, 120, 50];

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// Where the adapter sends this host its video output.
    pub output_listen: SocketAddr,
    /// Where the adapter sends this ground station its telemetry and camera messages.
    pub gcs_listen: SocketAddr,
    /// The ladder rung the first two windows run at, pinned through the vehicle's test hook (0:
    /// the top rung, as the vehicle starts); the third window is the next rung down.
    pub rung: usize,
    /// The vehicle's snapshot volume as this host mounts it: the card's clip goes here.
    pub snapshot_dir: PathBuf,
    pub warmup: Duration,
    pub duration: Duration,
    pub vehicle_metrics: String,
    pub ground_metrics: String,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "source": [SOURCE_SIZE.0, SOURCE_SIZE.1],
            "output": [OUTPUT_SIZE.0, OUTPUT_SIZE.1],
            "box": BOX,
            "warmup_s": self.warmup.as_secs_f64(),
            "duration_s": self.duration.as_secs_f64(),
            "score_every_nth_frame": SCORE_EVERY,
            "rung": self.rung,
            "detail_deadline_s": DETAIL_DEADLINE.as_secs_f64(),
        })
    }
}

/// What one scored frame showed.
#[derive(Debug, Clone, Copy)]
struct Frame {
    at: Instant,
    age_ms: Option<f64>,
    scores: Scores,
    /// SSIM of the detail inset's place in the frame and of the live picture's region, against the
    /// card's region at full resolution.
    detail: Option<(f64, f64)>,
}

/// The ground's output decoded and scored on its own threads.
struct Scorer {
    stop: Arc<AtomicBool>,
    frames: Arc<Mutex<Vec<Frame>>>,
    thread: JoinHandle<()>,
}

impl Scorer {
    fn start(source: Source, reference: Luma, region: Region, detail: Detail) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let frames = Arc::new(Mutex::new(Vec::new()));
        let thread = {
            let (stop, frames) = (Arc::clone(&stop), Arc::clone(&frames));
            thread::spawn(move || {
                let health = Health::default();
                let mut seen = 0_u64;
                supervise(&health, || {
                    reference_card::receive_luma(
                        &source,
                        OUTPUT_SIZE,
                        &stop,
                        |observation, luma| {
                            seen += 1;
                            let scored = seen
                                .is_multiple_of(SCORE_EVERY)
                                .then_some(observation.stamp)
                                .flatten();
                            let Some(stamp) = scored else {
                                return true;
                            };
                            let windows = ssim::windows(
                                &luma.pixels,
                                &reference.pixels,
                                reference.width,
                                reference.height,
                            );
                            if let (Some(scores), Ok(mut frames)) =
                                (region.score(&windows), frames.lock())
                            {
                                frames.push(Frame {
                                    at: observation.received_at,
                                    age_ms: plausible_age(observation.received_ms, stamp)
                                        .map(f64::from),
                                    scores,
                                    detail: detail.scores(&luma),
                                });
                            }
                            true
                        },
                    )
                });
            })
        };
        Self {
            stop,
            frames,
            thread,
        }
    }

    fn since(&self, from: Instant) -> Vec<Frame> {
        let frames = self.frames.lock().map(|f| f.clone()).unwrap_or_default();
        frames.into_iter().filter(|f| f.at >= from).collect()
    }

    fn any(&self) -> bool {
        self.frames.lock().is_ok_and(|f| !f.is_empty())
    }

    fn finish(self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.thread.join();
    }
}

/// The means over the frames of one window.
#[derive(Debug, Clone, Copy)]
struct Window {
    frames: usize,
    inside_ssim: f64,
    outside_ssim: f64,
    outside_contrast: f64,
    age_p95_ms: Option<f64>,
    /// Means of the inset's and the live picture's SSIM in the inset's region.
    inset_ssim: Option<f64>,
    live_ssim: Option<f64>,
}

#[expect(
    clippy::cast_precision_loss,
    reason = "frame counts stay far below 2^53"
)]
fn summarise(frames: &[Frame]) -> Option<Window> {
    if frames.is_empty() {
        return None;
    }
    let n = frames.len() as f64;
    let mean = |f: fn(&Scores) -> f64| frames.iter().map(|x| f(&x.scores)).sum::<f64>() / n;
    let received: f64 = frames.iter().map(|x| x.scores.outside_std_received).sum();
    let reference: f64 = frames.iter().map(|x| x.scores.outside_std_reference).sum();
    let details: Vec<(f64, f64)> = frames.iter().filter_map(|x| x.detail).collect();
    let detail_mean = |pick: fn(&(f64, f64)) -> f64| {
        (!details.is_empty()).then(|| details.iter().map(pick).sum::<f64>() / details.len() as f64)
    };
    let mut ages: Vec<f64> = frames.iter().filter_map(|x| x.age_ms).collect();
    ages.sort_by(f64::total_cmp);
    Some(Window {
        frames: frames.len(),
        inside_ssim: mean(|s| s.inside_ssim),
        outside_ssim: mean(|s| s.outside_ssim),
        outside_contrast: received / reference.max(f64::EPSILON),
        age_p95_ms: percentile(&ages, 95),
        inset_ssim: detail_mean(|d| d.0),
        live_ssim: detail_mean(|d| d.1),
    })
}

pub fn run(
    agent: &AgentClient,
    profile_text: &str,
    settings: &Settings,
    budgets: &Budgets,
    profile: &str,
) -> Results {
    let mut results = Results {
        thresholds: json!({
            "region_ssim_gain_above": 0.0,
            "detail_ssim_gain_at_least": MIN_DETAIL_GAIN,
            "inset_share_at_least": MIN_INSET_SHARE,
            "outside_ssim_vs_lower_rung_at_least": 0.0,
            "test_object_contrast_ratio_at_least": 1.0,
            "min_frames_per_window": MIN_FRAMES,
        }),
        settings: settings.to_json(),
        ..Results::default()
    };
    if !check::apply_profile(&mut results, agent, profile_text) {
        return results;
    }
    let vehicle = AgentClient::new(&settings.vehicle_metrics);
    let ground = AgentClient::new(&settings.ground_metrics);
    let settled = adaptation::settle_at_top(&vehicle);
    adaptation::record_settled(&mut results, &settled);
    let Some(windows) = measure(&mut results, settings, &vehicle, &ground) else {
        return results;
    };
    grade(&mut results, &windows, budgets.get("skeleton", profile));
    budget::grade(&mut results, budgets.get("regional", profile));
    results
}

/// The three windows' means and the vehicle's readings around the first.
struct Measured {
    with: Window,
    without: Window,
    lower: Window,
    /// The window with the box tracked and the inset drawn, and the share of its frames that had
    /// the inset.
    detail: Window,
    inset_share: f64,
    /// The detector ran: its amber outlines are drawn inside the box and count against the card.
    detector_on: bool,
}

fn measure(
    results: &mut Results,
    settings: &Settings,
    vehicle: &AgentClient,
    ground: &AgentClient,
) -> Option<Measured> {
    if let Err(e) = write_clip(&settings.snapshot_dir) {
        let detail = format!(
            "cannot write the card's clip to {}: {e}",
            settings.snapshot_dir.display()
        );
        results.record("card_written", Verdict::Incomplete, detail);
        return None;
    }
    let reference = match reference_card::reference(&card::render(), SOURCE_SIZE, OUTPUT_SIZE) {
        Ok(reference) => reference,
        Err(e) => {
            results.record("card_rendered", Verdict::Incomplete, e.to_string());
            return None;
        }
    };
    let card_luma = card::render();
    let source_luma = card_luma
        .get(..usize::from(SOURCE_SIZE.0) * usize::from(SOURCE_SIZE.1))
        .unwrap_or_default();
    let output = (usize::from(OUTPUT_SIZE.0), usize::from(OUTPUT_SIZE.1));
    let source = (usize::from(SOURCE_SIZE.0), usize::from(SOURCE_SIZE.1));
    let Some(detail) = Detail::new(source_luma, BOX, source, output) else {
        let detail = "the inset has no room in the output, or the box has no crop";
        results.record("inset_placed", Verdict::Incomplete, detail);
        return None;
    };
    let mut region = region(&reference);
    region.excluded = Some(detail.covered(output.0));
    let scorer = Scorer::start(
        Source::Rtp(settings.output_listen),
        reference,
        region,
        detail,
    );
    let measured = phases(results, settings, vehicle, ground, &scorer);
    scorer.finish();
    measured
}

/// Writes the card's clip where the vehicle's source reads it (`/snapshots/card.y4m` in
/// `vehicle-regional.toml`), whole or not at all: the vehicle starts over until it opens one.
fn write_clip(dir: &Path) -> std::io::Result<()> {
    let part = dir.join("card.y4m.part");
    fs::write(&part, card::y4m())?;
    fs::rename(part, dir.join("card.y4m"))
}

fn region(reference: &Luma) -> Region {
    let pixels = |fraction: f32, side: usize| {
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss,
            reason = "a fraction of a frame side, in pixels"
        )]
        let at = (fraction * side as f32).round() as usize;
        at
    };
    let [x0, y0, x1, y1] = BOX;
    Region {
        x0: pixels(x0, reference.width),
        y0: pixels(y0, reference.height),
        x1: pixels(x1, reference.width),
        y1: pixels(y1, reference.height),
        rows: (
            pixels(SCORED_ROWS.0, reference.height),
            pixels(SCORED_ROWS.1, reference.height),
        ),
        margin: EDGE_MARGIN,
        excluded: None,
    }
}

fn phases(
    results: &mut Results,
    settings: &Settings,
    vehicle: &AgentClient,
    ground: &AgentClient,
    scorer: &Scorer,
) -> Option<Measured> {
    let first = Instant::now() + FIRST_FRAME_DEADLINE;
    while !scorer.any() && Instant::now() < first {
        thread::sleep(Duration::from_millis(100));
    }
    if !scorer.any() {
        let detail = "no frame of the ground's output was decoded and scored (a stamp that reads, a box in view)";
        results.record("output_received", Verdict::Incomplete, detail);
        return None;
    }
    if take_lease(ground) != Some(LeaseStatus::Held) {
        let detail = "the ground adapter did not take the control lease, so it cannot send the box";
        results.record("lease_taken", Verdict::Incomplete, detail);
        return None;
    }
    let mut gcs = match Gcs::bind(settings.gcs_listen) {
        Ok(gcs) => gcs,
        Err(e) => {
            let detail = format!("cannot bind {}: {e}", settings.gcs_listen);
            results.record("camera_heard", Verdict::Incomplete, detail);
            return None;
        }
    };
    let heard = gcs.pump_until(Instant::now() + HEARTBEAT_DEADLINE, |seen| {
        seen.iter()
            .any(|s| s.from_camera && matches!(s.kind, Kind::Heartbeat { .. }))
    });
    if !heard {
        results.record("camera_heard", Verdict::Incomplete, "no camera heartbeat");
        return None;
    }
    let mut run = Run {
        settings,
        vehicle,
        ground,
        gcs,
        scorer,
        results,
        detector_on: false,
    };
    run.ask_for_tracking_status();
    if settings.rung != TOP_RUNG && !run.rung(settings.rung) {
        let detail = "the sender did not apply the rung asked for (is `debug_hooks` on?)";
        run.results
            .record("rung_pinned", Verdict::Incomplete, detail);
        return None;
    }
    let with = run.with_box()?;
    let without = run.without_box()?;
    let (detail, inset_share) = run.with_detail()?;
    let lower = run.lower_rung()?;
    run.rung(TOP_RUNG);
    Some(Measured {
        with,
        without,
        lower,
        detail,
        inset_share,
        detector_on: run.detector_on,
    })
}

struct Run<'a> {
    settings: &'a Settings,
    vehicle: &'a AgentClient,
    ground: &'a AgentClient,
    gcs: Gcs,
    scorer: &'a Scorer,
    results: &'a mut Results,
    detector_on: bool,
}

impl Run<'_> {
    fn camera_command(&mut self, command: MavCmd, params: [f32; 4]) {
        let [p1, p2, p3, p4] = params;
        self.gcs.send(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command,
            param1: p1,
            param2: p2,
            param3: p3,
            param4: p4,
            target_system: 1,
            target_component: CAMERA_COMPONENT,
            ..COMMAND_LONG_DATA::default()
        }));
    }

    /// What `QGroundControl` does first: asks the camera for its tracking status at the default
    /// rate (interval 0). The status must then arrive, idle, with no box tracked.
    fn ask_for_tracking_status(&mut self) {
        self.camera_command(
            MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            [TRACKING_STATUS_ID, 0.0, 0.0, 0.0],
        );
        let idle = self.wait_status(false);
        let detail = if idle {
            "SET_MESSAGE_INTERVAL started CAMERA_TRACKING_IMAGE_STATUS, idle while no box is tracked"
        } else {
            "no idle CAMERA_TRACKING_IMAGE_STATUS after SET_MESSAGE_INTERVAL"
        };
        self.results
            .record("tracking_status_requested", verdict(idle), detail);
    }

    /// Pumps until a tracking status says `active`; false at the deadline.
    fn wait_status(&mut self, active: bool) -> bool {
        self.gcs
            .pump_until(Instant::now() + TRACKING_DEADLINE, |seen| {
                seen.iter().rev().find_map(|s| match s.kind {
                    Kind::TrackingStatus { active: a, rect } if s.from_camera => {
                        Some(a == active && (!active || near_box(rect)))
                    }
                    _ => None,
                }) == Some(true)
            })
    }

    fn vehicle_metrics(&self) -> Result<Value, String> {
        check::get_json(self.vehicle, "/metrics")
    }

    /// Asks the sender for ladder rung `index` and waits for its first frame; false at the deadline.
    fn rung(&mut self, index: usize) -> bool {
        if self
            .vehicle
            .post(&format!("/video/rung/{index}"), "")
            .is_err()
        {
            return false;
        }
        let until = Instant::now() + RUNG_DEADLINE;
        let want = RUNG_KBIT.get(index).copied();
        while Instant::now() < until {
            let metrics = self.vehicle_metrics().ok();
            let applied = |pointer: &str| metrics.as_ref()?.pointer(pointer)?.as_u64();
            let landed = applied("/applied/seq") == applied("/requested/seq")
                && applied("/applied/rung/bitrate_kbit") == want;
            if landed {
                return true;
            }
            self.gcs.pump(Instant::now() + Duration::from_millis(200));
        }
        false
    }

    fn with_box(&mut self) -> Option<Window> {
        let [x0, y0, x1, y1] = BOX;
        self.camera_command(MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE, [x0, y0, x1, y1]);
        let active = self.wait_status(true);
        let detail = if active {
            "the status went active once the vehicle confirmed the box it keeps sharp"
        } else {
            "the status never went active: the vehicle did not confirm the box"
        };
        self.results
            .record("tracking_active", verdict(active), detail);
        if !active {
            return None;
        }
        let before = self.vehicle_metrics();
        self.detector_on = before
            .as_ref()
            .ok()
            .and_then(|m| {
                m.pointer("/detector/state")?
                    .as_str()
                    .map(|s| s == "running")
            })
            .unwrap_or(false);
        let window = self.window("with")?;
        let after = self.vehicle_metrics();
        self.record_vehicle(&before, &after);
        Some(window)
    }

    /// The box tracked again until the ground has drawn crops of it as an inset, then a window of
    /// frames with the inset, and the share of the window's frames that had it. Tracking is
    /// stopped after.
    fn with_detail(&mut self) -> Option<(Window, f64)> {
        let [x0, y0, x1, y1] = BOX;
        self.camera_command(MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE, [x0, y0, x1, y1]);
        let first = self.inset_frames();
        let ground = self.ground;
        let shown = self.gcs.pump_until(Instant::now() + DETAIL_DEADLINE, |_| {
            ground_counter(ground, "/video/inset_frames").is_some_and(|now| Some(now) > first)
        });
        let detail = if shown {
            "the ground fetched a crop of the box and drew it as an inset"
        } else {
            "no inset was drawn: no crop of the box reached the ground (is the job token mounted?)"
        };
        self.results.record("detail_shown", verdict(shown), detail);
        if !shown {
            return None;
        }
        let (inset_before, pushed_before) = (self.inset_frames(), self.pushed_frames());
        let window = self.window("detail")?;
        let drawn = increase(inset_before, self.inset_frames());
        let sent = increase(pushed_before, self.pushed_frames());
        self.camera_command(MavCmd::MAV_CMD_CAMERA_STOP_TRACKING, [0.0; 4]);
        let idle = self.wait_status(false);
        let share = drawn / sent.max(1.0);
        let detail = if idle {
            "the status went idle after STOP_TRACKING"
        } else {
            "the status stayed active after STOP_TRACKING"
        };
        self.results.record("detail_stopped", verdict(idle), detail);
        idle.then_some((window, share))
    }

    fn inset_frames(&self) -> Option<u64> {
        ground_counter(self.ground, "/video/inset_frames")
    }

    fn pushed_frames(&self) -> Option<u64> {
        ground_counter(self.ground, "/video/pushed")
    }

    fn without_box(&mut self) -> Option<Window> {
        self.camera_command(MavCmd::MAV_CMD_CAMERA_STOP_TRACKING, [0.0; 4]);
        let idle = self.wait_status(false);
        let detail = if idle {
            "the status went idle after STOP_TRACKING"
        } else {
            "the status stayed active after STOP_TRACKING"
        };
        self.results
            .record("tracking_stopped", verdict(idle), detail);
        if !idle {
            return None;
        }
        self.window("without")
    }

    fn lower_rung(&mut self) -> Option<Window> {
        if !self.rung(self.settings.rung + 1) {
            let detail = "the sender did not apply the lower rung (is `debug_hooks` on?)";
            self.results
                .record("lower_rung", Verdict::Incomplete, detail);
            return None;
        }
        self.window("lower")
    }

    /// Lets the picture settle, then scores the frames of one measuring window; keeps the
    /// ground station's heartbeat going meanwhile.
    fn window(&mut self, name: &'static str) -> Option<Window> {
        self.gcs.pump(Instant::now() + self.settings.warmup);
        let from = Instant::now();
        self.gcs.pump(from + self.settings.duration);
        let frames = self.scorer.since(from);
        let window = summarise(&frames).filter(|w| w.frames >= MIN_FRAMES);
        let (verdict, detail) = match window {
            Some(w) => (
                Verdict::Pass,
                format!(
                    "{} frames scored: box SSIM {:.4}, outside SSIM {:.4}, outside contrast kept {:.3}",
                    w.frames, w.inside_ssim, w.outside_ssim, w.outside_contrast
                ),
            ),
            None => (
                Verdict::Incomplete,
                format!("{} frames scored, {MIN_FRAMES} needed", frames.len()),
            ),
        };
        self.results
            .record(format!("frames_{name}"), verdict, detail);
        window
    }

    /// CPU and memory of the vehicle process over the window with the box, and what the sender
    /// and the detector spent.
    fn record_vehicle(&mut self, before: &Result<Value, String>, after: &Result<Value, String>) {
        let reading =
            |m: &Result<Value, String>, pointer: &str| m.as_ref().ok()?.pointer(pointer)?.as_f64();
        let window_ms = (self.settings.duration + self.settings.warmup).as_secs_f64() * 1000.0;
        if let (Some(start), Some(end)) = (
            reading(before, "/process/cpu_ms"),
            reading(after, "/process/cpu_ms"),
        ) {
            self.results
                .metrics
                .insert("vehicle_cpu_percent", (end - start) / window_ms * 100.0);
        }
        if let Some(rss_kb) = reading(after, "/process/rss_kb") {
            self.results
                .metrics
                .insert("vehicle_rss_mb", rss_kb / 1024.0);
        }
        if let Some(ms) = reading(after, "/region/soften/p95_ms") {
            self.results.metrics.insert("soften_ms_p95", ms);
        }
        let ran = reading(after, "/detector/runs").unwrap_or(0.0) > 0.0;
        if let (true, Some(ms)) = (ran, reading(after, "/detector/ms/p95_ms")) {
            self.results.metrics.insert("detector_ms_p95", ms);
        }
        self.results.readings = json!({
            "vehicle_before_with": before.clone().unwrap_or(Value::Null),
            "vehicle_after_with": after.clone().unwrap_or(Value::Null),
        });
    }
}

fn ground_counter(ground: &AgentClient, pointer: &str) -> Option<u64> {
    check::counter(&check::get_json(ground, "/metrics"), pointer)
}

#[expect(
    clippy::cast_precision_loss,
    reason = "frame counts stay far below 2^53"
)]
fn increase(before: Option<u64>, after: Option<u64>) -> f64 {
    after
        .zip(before)
        .map_or(0.0, |(a, b)| a.saturating_sub(b) as f64)
}

/// The status names the box that was asked for.
fn near_box(rect: [f32; 4]) -> bool {
    rect.iter().zip(BOX).all(|(a, b)| (a - b).abs() < 0.01)
}

fn verdict(holds: bool) -> Verdict {
    if holds { Verdict::Pass } else { Verdict::Fail }
}

/// Records the metrics and grades them: the box must gain, the outside must not fall below the
/// plain picture one rung down, and the age must stay within the skeleton's budget for the profile.
fn grade(results: &mut Results, m: &Measured, skeleton: Option<&Budget>) {
    let gain = m.with.inside_ssim - m.without.inside_ssim;
    let outside = m.with.outside_ssim - m.lower.outside_ssim;
    let contrast = m.with.outside_contrast / m.lower.outside_contrast.max(f64::EPSILON);
    for (name, value) in [
        ("region_ssim_with", m.with.inside_ssim),
        ("region_ssim_without", m.without.inside_ssim),
        ("region_ssim_gain", gain),
        ("outside_ssim_vs_lower_rung", outside),
        ("test_object_contrast_ratio", contrast),
    ] {
        results.metrics.insert(name, value);
    }
    if m.detector_on {
        let detail = format!(
            "{gain:+.4}, not graded: the detector's amber outlines are drawn inside the box and count against the card; run on vehicle-regional.toml (pilot's box only)"
        );
        results.record("region_ssim_gain", Verdict::Incomplete, detail);
    }
    let rules = [
        (
            "region_ssim_gain",
            gain > 0.0,
            format!("{gain:+.4} (box SSIM with regional video minus without)"),
        ),
        (
            "outside_ssim_vs_lower_rung",
            outside >= 0.0,
            format!("{outside:+.4} (outside the box, against the plain picture one rung down)"),
        ),
        (
            "test_object_contrast_ratio",
            contrast >= 1.0,
            format!("{contrast:.3} (fine detail outside the box kept, against one rung down)"),
        ),
    ];
    for (name, holds, detail) in rules {
        if name == "region_ssim_gain" && m.detector_on {
            continue;
        }
        results.record(name, verdict(holds), detail);
    }
    age(
        results,
        "frame_age_within_skeleton_budget",
        "frame_age_p95_ms",
        m.with.age_p95_ms,
        skeleton,
    );
    grade_detail(results, m, skeleton);
}

/// The inset must be drawn through the window, and be clearly closer to the card at full resolution
/// than the live picture is in the same region, without the live frames getting older.
fn grade_detail(results: &mut Results, m: &Measured, skeleton: Option<&Budget>) {
    let (Some(inset), Some(live)) = (m.detail.inset_ssim, m.detail.live_ssim) else {
        let detail = "no frame of the detail window had a score for the inset";
        results.record("detail_ssim_gain", Verdict::Incomplete, detail);
        return;
    };
    let gain = inset - live;
    for (name, value) in [
        ("detail_ssim_inset", inset),
        ("detail_ssim_live", live),
        ("detail_ssim_gain", gain),
        ("inset_share", m.inset_share),
    ] {
        results.metrics.insert(name, value);
    }
    let rules = [
        (
            "inset_drawn_throughout",
            m.inset_share >= MIN_INSET_SHARE,
            format!(
                "{:.0} % of the detail window's frames had the inset, at least {:.0} %",
                m.inset_share * 100.0,
                MIN_INSET_SHARE * 100.0
            ),
        ),
        (
            "detail_ssim_gain",
            gain >= MIN_DETAIL_GAIN,
            format!(
                "{gain:+.4}: the inset {inset:.4} against the live picture {live:.4} in the box's region, at least {MIN_DETAIL_GAIN:+.2}"
            ),
        ),
    ];
    for (name, holds, detail) in rules {
        results.record(name, verdict(holds), detail);
    }
    age(
        results,
        "detail_frame_age_within_skeleton_budget",
        "detail_frame_age_p95_ms",
        m.detail.age_p95_ms,
        skeleton,
    );
}

fn age(
    results: &mut Results,
    name: &'static str,
    metric: &'static str,
    p95: Option<f64>,
    skeleton: Option<&Budget>,
) {
    let limit = skeleton
        .and_then(|b| b.max.get("frame_age_p95_ms"))
        .copied();
    let (Some(limit), Some(with)) = (limit, p95) else {
        results.record(
            name,
            Verdict::Incomplete,
            "no frame age or no skeleton budget for this profile",
        );
        return;
    };
    results.metrics.insert(metric, with);
    let allowed = limit + OUTPUT_ALLOWANCE_MS;
    results.record(
        name,
        verdict(with <= allowed),
        format!("p95 {with:.0} ms, at most {allowed:.0} ms (skeleton {limit:.0} ms plus the output's hops)"),
    );
}
