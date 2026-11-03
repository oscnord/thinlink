//! The ground's freshness output: `MediaMTX`'s stream decoded, every frame marked with its freshness
//! (`sat_teleop_policy::freshness`) as a state stamp and overlay text, re-encoded and sent as RTP/H.264 to
//! QGC and to the lab. A new frame goes out as soon as it is decoded; the last frame (or a black
//! slate) is sent again with its state updated only when nothing went out for two output periods,
//! or earlier when the state may change then, so the picture never looks live while the source is
//! stale, frozen or gone and shows each change on time.

use std::collections::VecDeque;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_video as gst_video;
use serde::{Deserialize, Serialize};

use sat_teleop_policy::freshness::{self, LabelDeadline, Link, Observed, State, Thresholds};
use sat_teleop_policy::inset;
use sat_teleop_policy::mark as draw_mark;
use sat_teleop_policy::predict::{self, Inputs, Mark};
use sat_teleop_policy::stamp::{self, CaptureMs, FrameId, StateStamp};

use crate::inset::{DetailCell, Image};
use crate::receiver::{self, Observation, Source};
use crate::sender::Rung;
use crate::stages::{Stage, StageSnapshot};
use crate::{
    DATA_PROBE, Health, HealthSnapshot, MediaError, Running, Traffic, luma_geometry,
    plane_geometry, supervise,
};

/// Encoder threads: slices of one frame in parallel, without holding frames back.
const ENCODER_THREADS: u32 = 2;
const RTP_MTU: u32 = 1200;
const RTP_PAYLOAD_TYPE: u32 = 96;
const BLACK_Y: u8 = 16;
const BLACK_CHROMA: u8 = 128;
/// Overlay colours (ARGB): white while live, amber at a low rate, red otherwise.
const LIVE_COLOUR: u32 = 0xFFFF_FFFF;
const LOW_RATE_COLOUR: u32 = 0xFFFF_C000;
const ALERT_COLOUR: u32 = 0xFFFF_3030;
/// The readable share of stamps is taken over the frames of this window.
const READABLE_WINDOW: Duration = Duration::from_secs(1);
/// Bounds the readable window's memory whatever the source sends.
const READABLE_WINDOW_FRAMES: usize = 240;
/// New frames kept for the source's frame interval: more than a second's worth at 60 fps, the
/// sender's highest rate.
const CADENCE_FRAMES: usize = 64;
/// A frame's end offset when its state holds however late it leaves (`GST_BUFFER_OFFSET_NONE`).
const NO_DEADLINE: u64 = u64::MAX;
/// A frame's offset when it carries no inset; otherwise the inset's capture age in ms, for the
/// label drawn next to it (`GST_BUFFER_OFFSET_NONE`).
const NO_INSET: u64 = u64::MAX;
/// The source is never stopped; the process ends instead.
static NEVER: AtomicBool = AtomicBool::new(false);
/// Output periods without a new frame before the last one is sent again. Above one, so a source
/// frame arriving a little late does not meet a repeat in the encoder.
const REPEAT_PERIODS: u32 = 2;

/// `[video]` of `testbed/config/ground.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutputConfig {
    /// `MediaMTX`'s RTSP stream.
    pub source: String,
    /// Where the output goes, as `host:port`.
    pub destinations: Vec<String>,
    /// The stream URI the ground adapter's camera component announces; no camera without it.
    pub camera_uri: Option<String>,
    /// The output's encoding.
    pub encoding: Rung,
}

impl OutputConfig {
    pub fn validate(&self) -> Result<(), MediaError> {
        self.encoding.validate()?;
        let rung = self.encoding;
        stamp::check_state_size(usize::from(rung.width), usize::from(rung.height))
            .map_err(|e| MediaError::Config(format!("the state stamp does not fit: {e}")))?;
        if self.destinations.is_empty() {
            return Err(MediaError::Config("no destinations".to_owned()));
        }
        Ok(())
    }
}

/// `[camera]` of `testbed/config/ground.toml`. Without it the predictive display is off.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraConfig {
    /// The camera's horizontal field of view in degrees, the picture's full width.
    pub hfov_deg: f32,
    /// For a camera fixed to the body, how far it is mounted below the nose, in degrees: it turns
    /// the horizon on. Absent for a gimbal that holds its own angle.
    #[serde(default)]
    pub tilt_deg: Option<f32>,
}

impl CameraConfig {
    pub fn validate(self) -> Result<(), MediaError> {
        if !(self.hfov_deg.is_finite() && self.hfov_deg > 0.0 && self.hfov_deg < 180.0) {
            return Err(MediaError::Config(format!(
                "hfov_deg {} is not between 0 and 180 degrees",
                self.hfov_deg
            )));
        }
        if let Some(tilt) = self.tilt_deg.filter(|t| !(t.is_finite() && t.abs() < 90.0)) {
            return Err(MediaError::Config(format!(
                "tilt_deg {tilt} is not between -90 and 90 degrees"
            )));
        }
        Ok(())
    }
}

/// The predictive display: the camera's field of view, and where its inputs come from, read for
/// every frame drawn. `None` from `inputs` (no attitude yet, no uplink delay measured) draws nothing.
pub struct Predictive<'a> {
    pub camera: CameraConfig,
    pub inputs: &'a dyn Fn() -> Option<Inputs>,
}

/// `[freshness]` of `testbed/limits.toml`, in milliseconds.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FreshnessLimits {
    pub stale_after_ms: u64,
    pub frozen_after_ms: u64,
    pub disconnected_after_ms: u64,
    pub min_readable_percent: u8,
    pub low_rate_interval_ms: u64,
    pub low_rate_stale_after_ms: u64,
}

impl FreshnessLimits {
    pub fn validate(self) -> Result<Thresholds, MediaError> {
        let in_range = |ms: u64| (1..=60_000).contains(&ms);
        let rules = [
            (
                [
                    self.stale_after_ms,
                    self.frozen_after_ms,
                    self.disconnected_after_ms,
                    self.low_rate_interval_ms,
                    self.low_rate_stale_after_ms,
                ]
                .into_iter()
                .all(in_range),
                "thresholds must be 1-60000 ms",
            ),
            (
                self.stale_after_ms < self.disconnected_after_ms
                    && self.frozen_after_ms < self.disconnected_after_ms,
                "stale_after_ms and frozen_after_ms must be below disconnected_after_ms",
            ),
            (
                self.stale_after_ms <= self.low_rate_stale_after_ms
                    && self.low_rate_stale_after_ms < self.disconnected_after_ms,
                "low_rate_stale_after_ms must lie between stale_after_ms and disconnected_after_ms",
            ),
            (
                self.min_readable_percent <= 100,
                "min_readable_percent must be 0-100",
            ),
        ];
        if let Some((_, rule)) = rules.iter().find(|(holds, _)| !holds) {
            return Err(MediaError::Config((*rule).to_owned()));
        }
        Ok(Thresholds {
            stale_after_ms: self.stale_after_ms,
            frozen_after_ms: self.frozen_after_ms,
            disconnected_after_ms: self.disconnected_after_ms,
            min_readable_percent: self.min_readable_percent,
            low_rate_interval_ms: self.low_rate_interval_ms,
            low_rate_stale_after_ms: self.low_rate_stale_after_ms,
        })
    }
}

/// Counters shared between the pipelines and the metrics endpoint.
#[derive(Debug, Default)]
pub struct OutputStats {
    /// The pipeline decoding `MediaMTX`'s stream.
    pub source: Health,
    /// The pipeline encoding and sending the output.
    pub output: Health,
    decoded: AtomicU64,
    unreadable: AtomicU64,
    new_frames: AtomicU64,
    decode_age_ms_total: AtomicU64,
    pushed: AtomicU64,
    repeated: AtomicU64,
    late: AtomicU64,
    inset_frames: AtomicU64,
    states: StateCounts,
    state: Mutex<Option<State>>,
    rtp: Traffic,
    /// From the overlay's input to the first packet at the sink: drawing, encoding, payloading.
    reencode: Stage,
}

#[derive(Debug, Default)]
struct StateCounts {
    live: AtomicU64,
    stale: AtomicU64,
    frozen: AtomicU64,
    disconnected: AtomicU64,
    age_unknown: AtomicU64,
    low_rate: AtomicU64,
}

#[derive(Debug, Serialize)]
pub struct OutputSnapshot {
    pub source: HealthSnapshot,
    pub output: HealthSnapshot,
    /// Frames decoded from `MediaMTX`.
    pub decoded: u64,
    /// Decoded frames whose stamp did not read back or carried an implausible age.
    pub unreadable: u64,
    /// Decoded frames with a stamp newer than any before.
    pub new_frames: u64,
    /// Sum of the new frames' ages at decoding, for a mean over two readings.
    pub decode_age_ms_total: u64,
    /// Frames sent to the encoder, and how many of them repeated the last one.
    pub pushed: u64,
    pub repeated: u64,
    /// Frames dropped on their way out because the state they show had stopped being true.
    pub late: u64,
    /// Frames sent with the detail inset drawn in them.
    pub inset_frames: u64,
    /// The state of the frame sent last; `null` before the first.
    pub state: Option<&'static str>,
    /// Frames sent, by state.
    pub states: StateCountsSnapshot,
    pub rtp_packets: u64,
    pub rtp_bytes: u64,
    /// Parsing and decoding a frame (`receiver::DECODE`), over the last 1024 frames.
    pub decode: StageSnapshot,
    /// Drawing the state, re-encoding and payloading a frame, over the last 1024 frames.
    pub reencode: StageSnapshot,
}

#[derive(Debug, Serialize)]
pub struct StateCountsSnapshot {
    pub live: u64,
    pub stale: u64,
    pub frozen: u64,
    pub disconnected: u64,
    pub age_unknown: u64,
    pub low_rate: u64,
}

impl OutputStats {
    pub fn snapshot(&self) -> OutputSnapshot {
        let load = |counter: &AtomicU64| counter.load(Ordering::Relaxed);
        let rtp = self.rtp.snapshot();
        let s = &self.states;
        OutputSnapshot {
            source: self.source.snapshot(),
            output: self.output.snapshot(),
            decoded: load(&self.decoded),
            unreadable: load(&self.unreadable),
            new_frames: load(&self.new_frames),
            decode_age_ms_total: load(&self.decode_age_ms_total),
            pushed: load(&self.pushed),
            repeated: load(&self.repeated),
            late: load(&self.late),
            inset_frames: load(&self.inset_frames),
            state: self.state.lock().ok().and_then(|s| *s).map(State::name),
            states: StateCountsSnapshot {
                live: load(&s.live),
                stale: load(&s.stale),
                frozen: load(&s.frozen),
                disconnected: load(&s.disconnected),
                age_unknown: load(&s.age_unknown),
                low_rate: load(&s.low_rate),
            },
            rtp_packets: rtp.packets,
            rtp_bytes: rtp.bytes,
            decode: receiver::DECODE.snapshot(),
            reencode: self.reencode.snapshot(),
        }
    }

    fn sent(&self, state: State, repeat: bool) {
        self.pushed.fetch_add(1, Ordering::Relaxed);
        self.repeated
            .fetch_add(u64::from(repeat), Ordering::Relaxed);
        let s = &self.states;
        let counter = match state {
            State::Live => &s.live,
            State::Stale => &s.stale,
            State::Frozen => &s.frozen,
            State::Disconnected => &s.disconnected,
            State::AgeUnknown => &s.age_unknown,
            State::LowRate => &s.low_rate,
        };
        counter.fetch_add(1, Ordering::Relaxed);
        if let Ok(mut current) = self.state.lock() {
            *current = Some(state);
        }
    }
}

/// Decodes the source and sends the output until the process ends, each pipeline restarted after
/// a failure. `destinations` are `config.destinations`, resolved; `clock_bound` is read for every
/// state it computes. With `predictive` the aim mark and the horizon are drawn into current frames;
/// the newest crop in `detail` is drawn as an inset into current frames while it is young.
pub fn run(
    config: &OutputConfig,
    destinations: &[SocketAddr],
    thresholds: Thresholds,
    stats: &Arc<OutputStats>,
    clock_bound: &dyn Fn() -> Option<u64>,
    predictive: Option<&Predictive>,
    detail: &DetailCell,
) {
    let latest = Arc::new(Latest::default());
    let rung = config.encoding;
    {
        let (latest, stats) = (Arc::clone(&latest), Arc::clone(stats));
        let source = Source::RtspTcp(config.source.clone());
        thread::spawn(move || {
            supervise(&stats.source, || {
                receiver::decode(&source, (rung.width, rung.height), &NEVER, |o, sample| {
                    latest.put(o, sample);
                    true
                })
            });
        });
    }
    let output = Output {
        config,
        destinations,
        thresholds,
        latest: &latest,
        stats,
        clock_bound,
        predictive,
        detail,
    };
    supervise(&stats.output, || output.run());
}

/// The newest decoded frame, handed from the source pipeline to the output.
#[derive(Default)]
struct Latest {
    slot: Mutex<Slot>,
    arrived: Condvar,
}

#[derive(Default)]
struct Slot {
    seq: u64,
    frame: Option<(Observation, gst::Sample)>,
}

impl Latest {
    fn put(&self, observation: Observation, sample: gst::Sample) {
        let mut slot = self.lock();
        slot.seq += 1;
        slot.frame = Some((observation, sample));
        self.arrived.notify_one();
    }

    /// Waits at most `timeout` for a frame put after `seen`; returns the newest sequence number and
    /// the frame, if one is waiting that nobody took yet.
    fn next(&self, seen: u64, timeout: Duration) -> (u64, Option<(Observation, gst::Sample)>) {
        let slot = self.lock();
        let (mut slot, _) = self
            .arrived
            .wait_timeout_while(slot, timeout, |s| s.seq == seen)
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        (slot.seq, slot.frame.take())
    }

    fn lock(&self) -> MutexGuard<'_, Slot> {
        self.slot
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

struct Output<'a> {
    config: &'a OutputConfig,
    destinations: &'a [SocketAddr],
    thresholds: Thresholds,
    latest: &'a Latest,
    stats: &'a Arc<OutputStats>,
    /// How far the vehicle's clock may be from the ground's now, in ms; without a bound no age is
    /// trusted and nothing is shown as live.
    clock_bound: &'a dyn Fn() -> Option<u64>,
    predictive: Option<&'a Predictive<'a>>,
    detail: &'a DetailCell,
}

/// The output pipeline and the end the loop pushes into.
struct Pipeline {
    running: Running,
    appsrc: gst_app::AppSrc,
    video: gst_video::VideoInfo,
}

impl Output<'_> {
    /// Sends frames until the output pipeline fails. Never returns `Ok`.
    fn run(&self) -> Result<(), MediaError> {
        let pipeline = self.build()?;
        pipeline.running.set(gst::State::Playing)?;
        let slate = slate(&pipeline.video)?;
        let longest = Duration::from_secs(1) * REPEAT_PERIODS / u32::from(self.config.encoding.fps);
        let mut tracker = Tracker::default();
        let mut seen = 0;
        let mut shown: Option<gst::Buffer> = None;
        let mut last_push = Instant::now();
        let mut repeat_after = longest;
        loop {
            if let Some(error) = pipeline.running.bus_error() {
                return Err(error);
            }
            let due = last_push + repeat_after;
            let (seq, next) = self
                .latest
                .next(seen, due.saturating_duration_since(Instant::now()));
            seen = seq;
            let repeat = next.is_none();
            // Woken without a frame (one taken before a restart, or spuriously): wait until due.
            if repeat && Instant::now() < due {
                continue;
            }
            let decoded = next
                .map(|(observation, sample)| (self.observe(&mut tracker, &observation), sample));
            last_push = Instant::now();
            let (state, change_ms) = self.state(&tracker, last_push);
            if let Some((seen, sample)) = decoded {
                shown = shown_after(seen, state.state, &sample, shown);
            }
            repeat_after = change_ms.map_or(longest, |ms| longest.min(Duration::from_millis(ms)));
            let drew_inset = pipeline.push(
                shown.as_ref().unwrap_or(&slate),
                state,
                Decorations {
                    mark: self.mark(state),
                    inset: self.detail.current(last_push),
                },
                last_push,
                change_ms,
            )?;
            self.stats
                .inset_frames
                .fetch_add(u64::from(drew_inset), Ordering::Relaxed);
            self.stats.sent(state.state, repeat);
        }
    }

    /// The predictive display for a frame shown in `state`, if it is on and has its inputs.
    fn mark(&self, state: StateStamp) -> Option<Mark> {
        let predictive = self.predictive?;
        let inputs = (predictive.inputs)()?;
        let rung = self.config.encoding;
        predict::mark(
            state.state,
            state.age_ms().map(u64::from),
            &inputs,
            predictive.camera.hfov_deg,
            predictive.camera.tilt_deg,
            u32::from(rung.width),
            u32::from(rung.height),
        )
    }

    fn observe(&self, tracker: &mut Tracker, observation: &Observation) -> Seen {
        let stats = self.stats;
        stats.decoded.fetch_add(1, Ordering::Relaxed);
        let seen = tracker.observe(observation);
        match seen {
            Seen::Unreadable => {
                stats.unreadable.fetch_add(1, Ordering::Relaxed);
            }
            Seen::New { age_ms } => {
                stats.new_frames.fetch_add(1, Ordering::Relaxed);
                stats
                    .decode_age_ms_total
                    .fetch_add(u64::from(age_ms), Ordering::Relaxed);
            }
            Seen::Same | Seen::Older => {}
        }
        seen
    }

    /// The state to show now, and in how many ms it may change without a new frame.
    fn state(&self, tracker: &Tracker, now: Instant) -> (StateStamp, Option<u64>) {
        let link = if self.stats.source.running() {
            Link::Up
        } else {
            Link::Down
        };
        let clock_bound_ms = (self.clock_bound)();
        let observed = tracker.observed(now, link, clock_bound_ms);
        let state = freshness::classify(self.thresholds, observed);
        let change_ms = freshness::next_change_ms(self.thresholds, observed);
        (StateStamp::new(state, observed.age_ms), change_ms)
    }

    fn build(&self) -> Result<Pipeline, MediaError> {
        let rung = self.config.encoding;
        let video = gst_video::VideoInfo::builder(
            gst_video::VideoFormat::I420,
            u32::from(rung.width),
            u32::from(rung.height),
        )
        .fps(gst::Fraction::new(i32::from(rung.fps), 1))
        .build()?;
        let appsrc = gst_app::AppSrc::builder()
            .caps(&video.to_caps()?)
            .is_live(true)
            .format(gst::Format::Time)
            .do_timestamp(true)
            .block(false)
            .property("max-buffers", 1u64)
            .property_from_str("leaky-type", "downstream")
            .build();
        let overlay = gst::ElementFactory::make("textoverlay")
            .property_from_str("halignment", "left")
            .property_from_str("valignment", "bottom")
            .property("font-desc", "Sans Bold 20")
            .property("shaded-background", true)
            .build()?;
        let inset_label = gst::ElementFactory::make("textoverlay")
            .property_from_str("halignment", "right")
            .property_from_str("valignment", "top")
            .property("font-desc", "Sans Bold 20")
            .property("shaded-background", true)
            .property("xpad", pixels(inset::MARGIN_PX)?)
            .property("ypad", pixels(inset::label_y(usize::from(rung.width)))?)
            .property("silent", true)
            .build()?;
        let clients: Vec<String> = self.destinations.iter().map(ToString::to_string).collect();
        let sink = gst::ElementFactory::make("multiudpsink")
            .property("clients", clients.join(","))
            .property("sync", false)
            .build()?;
        let chain = [
            appsrc.clone().upcast(),
            overlay.clone(),
            inset_label.clone(),
            gst::ElementFactory::make("x264enc")
                .property_from_str("tune", "zerolatency")
                .property_from_str("speed-preset", "ultrafast")
                .property("bitrate", rung.bitrate_kbit)
                .property("key-int-max", rung.keyframe_frames())
                .property("bframes", 0u32)
                .property("threads", ENCODER_THREADS)
                .build()?,
            gst::ElementFactory::make("rtph264pay")
                .property("config-interval", 1i32)
                .property("pt", RTP_PAYLOAD_TYPE)
                .property("mtu", RTP_MTU)
                .build()?,
            sink.clone(),
        ];
        let pipeline = gst::Pipeline::new();
        pipeline.add_many(&chain)?;
        gst::Element::link_many(&chain)?;
        let in_flight = Arc::new(Mutex::new(LabelDeadline::default()));
        let entered = Arc::new(Mutex::new(None));
        label_each_frame(&overlay, &inset_label, video.clone(), &in_flight, &entered)?;
        drop_late_frames(&sink, &in_flight, &entered, self.stats)?;
        Ok(Pipeline {
            running: Running(pipeline),
            appsrc,
            video,
        })
    }
}

/// What is drawn into a frame besides its stamps.
struct Decorations {
    mark: Option<Mark>,
    /// The detail crop and its capture age in ms.
    inset: Option<(Arc<Image>, u64)>,
}

impl Pipeline {
    /// Sends `frame` with `state` drawn below its stamp (the overlay reads it from there) and
    /// the decorations: `mark` under the stamps, then the inset (if the picture is current and
    /// the crop young: `inset::place`) with its age in the buffer's offset for the inset's label,
    /// all before the overlays write their labels. The state was decided at `decided_at` and holds
    /// for `holds_ms` more; the frame carries the moment that ends, on the pipeline's clock, in its
    /// end offset. Returns whether the inset was drawn.
    fn push(
        &self,
        frame: &gst::Buffer,
        state: StateStamp,
        decorations: Decorations,
        decided_at: Instant,
        holds_ms: Option<u64>,
    ) -> Result<bool, MediaError> {
        let deadline = holds_ms
            .filter(|_| current(state.state))
            .zip(self.appsrc.current_running_time())
            .map(|(ms, now)| {
                (now + gst::ClockTime::from_mseconds(ms))
                    .saturating_sub(gst::ClockTime::from_nseconds(nanos(decided_at.elapsed())))
            });
        let placed = decorations.inset.and_then(|(image, age_ms)| {
            let picture = (self.video.width() as usize, self.video.height() as usize);
            let placement = inset::place(state.state, age_ms, image.size(), picture)?;
            Some((image, age_ms, placement))
        });
        let mut buffer = frame.copy_deep()?;
        {
            let buffer = buffer.make_mut();
            buffer.set_offset_end(deadline.map_or(NO_DEADLINE, gst::ClockTime::nseconds));
            buffer.set_offset(placed.as_ref().map_or(NO_INSET, |(_, age_ms, _)| *age_ms));
            buffer.set_pts(gst::ClockTime::NONE);
            buffer.set_dts(gst::ClockTime::NONE);
            buffer.set_duration(gst::ClockTime::NONE);
            let mut frame =
                gst_video::VideoFrameRef::from_buffer_ref_writable(buffer, &self.video)?;
            let geometry = luma_geometry(&frame)?;
            let plane = frame.plane_data_mut(0)?;
            stamp::draw_state(plane, geometry, state)?;
            if let Some(Mark { aim, horizon }) = decorations.mark {
                draw_mark::draw(plane, geometry, aim, horizon)?;
            }
            if let Some((image, _, placement)) = &placed {
                for (index, component) in (0..3_u32).enumerate() {
                    let geometry = plane_geometry(&frame, component)?;
                    image.draw(
                        index,
                        frame.plane_data_mut(component)?,
                        geometry,
                        *placement,
                    );
                }
            }
        }
        self.appsrc
            .push_buffer(buffer)
            .map_err(|e| MediaError::Config(format!("the output refused a frame: {e:?}")))?;
        Ok(placed.is_some())
    }
}

/// Drops a frame that reaches the sender after the state it shows stopped being true, and has the
/// encoder start a new keyframe so the pictures after the gap do not build on the dropped one. The
/// state is decided before the encoder and the sender, and a stall in them would otherwise let
/// them send what was Live as Live, however old it has become.
fn drop_late_frames(
    sink: &gst::Element,
    in_flight: &Arc<Mutex<LabelDeadline>>,
    entered: &Arc<Mutex<Option<Instant>>>,
    stats: &Arc<OutputStats>,
) -> Result<(), MediaError> {
    let pad = sink
        .static_pad("sink")
        .ok_or(MediaError::Missing("sink pad"))?;
    let (sink, in_flight, entered, stats) = (
        sink.clone(),
        Arc::clone(in_flight),
        Arc::clone(entered),
        Arc::clone(stats),
    );
    pad.add_probe(DATA_PROBE, move |pad, info| {
        stats.rtp.record(info.data.as_ref());
        let since = entered
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .take();
        if let Some(since) = since {
            stats.reencode.record(since.elapsed());
        }
        let Some(now) = sink.current_running_time() else {
            return gst::PadProbeReturn::Ok;
        };
        let late = in_flight
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .late(now.nseconds());
        if !late {
            return gst::PadProbeReturn::Ok;
        }
        stats.late.fetch_add(1, Ordering::Relaxed);
        pad.push_event(
            gst_video::UpstreamForceKeyUnitEvent::builder()
                .all_headers(true)
                .build(),
        );
        gst::PadProbeReturn::Drop
    });
    Ok(())
}

/// Sets the overlay's text and colour from each frame's own state stamp, in the streaming thread
/// just before the overlay draws that frame, so a label can never land on another frame. It also
/// hands the frame's deadline to `drop_late_frames`: the encoder and the sender run in this same
/// thread, one frame at a time with nothing held back (`zerolatency`). The inset's label is set
/// the same way from the age in the frame's offset, and silent for a frame without an inset.
fn label_each_frame(
    overlay: &gst::Element,
    inset_label: &gst::Element,
    video: gst_video::VideoInfo,
    in_flight: &Arc<Mutex<LabelDeadline>>,
    entered: &Arc<Mutex<Option<Instant>>>,
) -> Result<(), MediaError> {
    let (in_flight, entered) = (Arc::clone(in_flight), Arc::clone(entered));
    let inset_label = inset_label.clone();
    let pad = overlay
        .static_pad("video_sink")
        .ok_or(MediaError::Missing("overlay video sink pad"))?;
    pad.add_probe(gst::PadProbeType::BUFFER, move |pad, info| {
        let Some(gst::PadProbeData::Buffer(buffer)) = &info.data else {
            return gst::PadProbeReturn::Ok;
        };
        *entered
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner) = Some(Instant::now());
        let deadline = Some(buffer.offset_end()).filter(|end| *end != NO_DEADLINE);
        in_flight
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .arm(deadline);
        let state = gst_video::VideoFrameRef::from_buffer_ref_readable(buffer.as_ref(), &video)
            .ok()
            .and_then(|frame| {
                let geometry = luma_geometry(&frame).ok()?;
                stamp::read_state(frame.plane_data(0).ok()?, geometry)
            });
        if let Some(overlay) = pad.parent_element() {
            let (text, colour) = label(state);
            overlay.set_property("text", text);
            overlay.set_property("color", colour);
            let inset_age = Some(buffer.offset()).filter(|age| *age != NO_INSET);
            inset_label.set_property("silent", inset_age.is_none());
            if let Some(age_ms) = inset_age {
                inset_label.set_property("text", inset::label(age_ms));
                inset_label.set_property("color", colour);
            }
        }
        gst::PadProbeReturn::Ok
    });
    Ok(())
}

/// Whether the state claims a current picture.
fn current(state: State) -> bool {
    matches!(state, State::Live | State::LowRate)
}

/// The overlay text and colour for a frame's state stamp; never Live without one.
fn label(state: Option<StateStamp>) -> (String, u32) {
    let Some(state) = state else {
        return ("NO STATE".to_owned(), ALERT_COLOUR);
    };
    let (name, colour) = match state.state {
        State::Live => ("LIVE", LIVE_COLOUR),
        State::Stale => ("STALE", ALERT_COLOUR),
        State::Frozen => ("FROZEN", ALERT_COLOUR),
        State::Disconnected => ("NO VIDEO", ALERT_COLOUR),
        State::AgeUnknown => ("AGE UNKNOWN", ALERT_COLOUR),
        State::LowRate => ("LOW RATE", LOW_RATE_COLOUR),
    };
    let text = match state.age_ms() {
        Some(ms) => format!("{name} {:.2} s", f64::from(ms) / 1000.0),
        None => name.to_owned(),
    };
    (text, colour)
}

/// The picture to send from now on: the decoded `sample`, unless it is older than the newest one
/// seen, or its stamp is unreadable while `state` calls the picture current. A decoder concealing
/// a loss with an earlier or broken picture would otherwise show it under the newest frame's
/// state, an old picture labelled as current.
fn shown_after(
    seen: Seen,
    state: State,
    sample: &gst::Sample,
    shown: Option<gst::Buffer>,
) -> Option<gst::Buffer> {
    if seen == Seen::Older || (seen == Seen::Unreadable && current(state)) {
        return shown;
    }
    sample.buffer_owned().or(shown)
}

/// A black frame, shown until the first frame is decoded.
fn slate(video: &gst_video::VideoInfo) -> Result<gst::Buffer, MediaError> {
    let mut buffer = gst::Buffer::with_size(video.size())?;
    {
        let buffer = buffer.make_mut();
        let mut frame = gst_video::VideoFrameRef::from_buffer_ref_writable(buffer, video)?;
        for (plane, value) in [(0, BLACK_Y), (1, BLACK_CHROMA), (2, BLACK_CHROMA)] {
            frame.plane_data_mut(plane)?.fill(value);
        }
    }
    Ok(buffer)
}

/// What the output knows about its source: when frames arrived, the newest new one, the source's
/// frame interval and how many stamps of the last second read back.
#[derive(Default)]
struct Tracker {
    last_frame_at: Option<Instant>,
    newest: Option<Newest>,
    /// The ids and capture times of the newest new frames, oldest first, and their cadence.
    captures: VecDeque<(FrameId, CaptureMs)>,
    interval_ms: Option<u64>,
    /// When each frame of the readable window arrived and whether its stamp was readable and
    /// plausible. A window of time, not of frames: the source's frame rate changes with its rung.
    recent: VecDeque<(Instant, bool)>,
}

/// The newest new frame: its capture time, when it arrived and how old it was then. Its age later
/// is that age plus the monotonic time since, so a wall-clock step never makes it younger.
#[derive(Clone, Copy)]
struct Newest {
    capture: CaptureMs,
    at: Instant,
    age_ms: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Seen {
    Unreadable,
    /// A stamp newer than any before, `age_ms` old when decoded.
    New {
        age_ms: u32,
    },
    /// The newest stamp again: a frozen source.
    Same,
    /// A stamp older than the newest: a decoder concealing a loss with an earlier picture.
    Older,
}

impl Tracker {
    fn observe(&mut self, o: &Observation) -> Seen {
        self.last_frame_at = Some(o.received_at);
        let readable = o
            .stamp
            .and_then(|s| Some((s, stamp::plausible_age(o.received_ms, s)?)));
        while self.recent.len() >= READABLE_WINDOW_FRAMES
            || self.recent.front().is_some_and(|(at, _)| {
                o.received_at.saturating_duration_since(*at) > READABLE_WINDOW
            })
        {
            self.recent.pop_front();
        }
        self.recent.push_back((o.received_at, readable.is_some()));
        let Some((stamp, age_ms)) = readable else {
            return Seen::Unreadable;
        };
        if let Some(newest) = self.newest.filter(|n| !newer(stamp.capture_ms, n.capture)) {
            return if stamp.capture_ms == newest.capture {
                Seen::Same
            } else {
                Seen::Older
            };
        }
        if self.captures.len() >= CADENCE_FRAMES {
            self.captures.pop_front();
        }
        self.captures.push_back((stamp.frame_id, stamp.capture_ms));
        self.interval_ms = freshness::cadence_ms(self.captures.make_contiguous());
        self.newest = Some(Newest {
            capture: stamp.capture_ms,
            at: o.received_at,
            age_ms,
        });
        Seen::New { age_ms }
    }

    fn observed(&self, now: Instant, link: Link, clock_bound_ms: Option<u64>) -> Observed {
        let since = |at: Instant| millis(now.saturating_duration_since(at));
        let count = |n: usize| u32::try_from(n).unwrap_or(u32::MAX);
        let window = self
            .recent
            .iter()
            .filter(|(at, _)| now.saturating_duration_since(*at) <= READABLE_WINDOW);
        let (decoded, readable) = window.fold((0, 0), |(all, ok), (_, readable)| {
            (all + 1, ok + usize::from(*readable))
        });
        Observed {
            link,
            since_frame_ms: self.last_frame_at.map(since),
            since_new_frame_ms: self.newest.map(|n| since(n.at)),
            age_ms: self
                .newest
                .map(|n| u64::from(n.age_ms).saturating_add(since(n.at))),
            frame_interval_ms: self.interval_ms,
            readable: count(readable),
            decoded: count(decoded),
            clock_bound_ms,
        }
    }
}

/// Whether `capture` is later than `than` on the wrapping 32-bit stamp clock.
fn newer(capture: CaptureMs, than: CaptureMs) -> bool {
    let ahead = capture.0.wrapping_sub(than.0);
    ahead != 0 && ahead < 1 << 31
}

/// A pixel count as the overlay's integer property.
fn pixels(count: usize) -> Result<i32, MediaError> {
    i32::try_from(count).map_err(|_| MediaError::Config(format!("{count} px is too large")))
}

fn nanos(duration: Duration) -> u64 {
    u64::try_from(duration.as_nanos()).unwrap_or(u64::MAX)
}

fn millis(duration: Duration) -> u64 {
    u64::try_from(duration.as_millis()).unwrap_or(u64::MAX)
}
