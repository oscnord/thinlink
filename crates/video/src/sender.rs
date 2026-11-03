//! Vehicle sender: source -> convert -> tee -> scale/rate -> I420 rung -> stamp -> leaky queue ->
//! encoder -> RTP -> UDP to the relay. The encoder is x264 or, on a board that has one, the V4L2
//! hardware H.264 encoder (`camera`). The rung changes while it runs: x264's bitrate and VBV buffer
//! at the next frame, its resolution and frame rate through the rung's caps (it reopens itself).
//! The tee's second branch takes the source's full-resolution frames for retained snapshots:
//! a copy of each frame -> leaky queue -> videorate -> jpegenc -> appsink, handed to the caller's writer. The RTP packets leave through the pacer (`pacer`), which holds the video to the
//! uplink measured.

mod pacer;
mod tap;

pub use tap::{Tap, TapFormat, TapFrame, TapSpec};

use std::collections::VecDeque;
use std::net::{IpAddr, SocketAddr};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicU64, Ordering};
use std::sync::mpsc::{SyncSender, TrySendError};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_video as gst_video;
use serde::{Deserialize, Serialize};

use sat_teleop_video_core::stamp::{self, CaptureMs, FrameId, Stamp};

use crate::camera::{self, CameraFormat, CaptureClock, Encoder, EncoderWish};
use crate::ingest::{self, IngestSnapshot, IngestStats};
use crate::stages::{InEncoder, Marks, Stage, StageSnapshot};
use crate::{
    Health, HealthSnapshot, MediaError, Running, Traffic, TrafficSnapshot, failure, luma_geometry,
    unix_ms,
};

/// RTP packet size including its header. With RTP, UDP and IP headers a packet stays below the
/// 1420-byte `WireGuard` MTU, so no packet is fragmented inside the tunnel.
const RTP_MTU: u32 = 1200;
const RTP_PAYLOAD_TYPE: u32 = 96;
const MAX_WIDTH: u16 = 3840;
const MAX_HEIGHT: u16 = 2160;
const MAX_FPS: u16 = 60;
const ENCODER: &str = "encoder";
/// A camera keeps its mode whatever the rung: the test pattern is held to one rate too.
const TEST_PATTERN_FPS: i32 = 30;
const MAX_SNAPSHOT_FPS: u16 = 10;
/// Snapshot frames waiting for their identity or their JPEG.
const SNAPSHOT_IN_FLIGHT: usize = 4;
/// A mapped raw frame the hook may draw into.
pub type Frame<'a> = gst_video::VideoFrameRef<&'a mut gst::BufferRef>;

const STAMP_PROBE: gst::PadProbeType =
    gst::PadProbeType::BUFFER.union(gst::PadProbeType::EVENT_DOWNSTREAM);

/// Called with each raw frame before the stamp is drawn into it, in the streaming thread.
pub type FrameHook = Arc<dyn Fn(&mut Frame) + Send + Sync>;

/// What the caller adds to the sender: a hook on every raw frame and taps off the tee.
#[derive(Clone, Default)]
pub struct Hooks {
    pub frame: Option<FrameHook>,
    pub taps: Vec<Arc<Tap>>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SenderConfig {
    /// Where RTP goes: the relay's overlay address.
    pub destination: SocketAddr,
    pub source: Source,
    /// Retained snapshots of the source's frames; none without it.
    #[serde(default)]
    pub snapshot: Option<SnapshotConfig>,
    /// `x264` (the default) or `hardware`: the board's H.264 encoder when it has one.
    #[serde(default)]
    pub encoder: EncoderWish,
}

/// `[snapshot]` of the vehicle config.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SnapshotConfig {
    /// Snapshots per second, of the source's frames at the source's size.
    pub fps: u16,
    /// `jpegenc`'s quality, 1-100.
    pub quality: u8,
}

impl SnapshotConfig {
    fn validate(self) -> Result<(), MediaError> {
        let rules = [
            (
                (1..=MAX_SNAPSHOT_FPS).contains(&self.fps),
                "snapshot fps must be 1-10",
            ),
            (
                (1..=100).contains(&self.quality),
                "snapshot quality must be 1-100",
            ),
        ];
        match rules.iter().find(|(holds, _)| !holds) {
            Some((_, rule)) => Err(MediaError::Config((*rule).to_owned())),
            None => Ok(()),
        }
    }
}

/// One retained snapshot, as the sender hands it to the caller.
#[derive(Debug)]
pub struct Snapshot {
    /// UNIX milliseconds when the frame passed the branch.
    pub capture_ms: u64,
    /// The id the live stream's next frame carries at that moment, so the snapshot lies within a
    /// frame of the live frame of the same name.
    pub frame_id: u32,
    pub jpeg: Vec<u8>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case", deny_unknown_fields)]
pub enum Source {
    /// `videotestsrc is-live=true`, at `width` x `height` when both are given.
    TestPattern {
        #[serde(default)]
        width: Option<u16>,
        #[serde(default)]
        height: Option<u16>,
        #[serde(default)]
        pattern: TestPattern,
    },
    /// A y4m file, looped and paced to real time; with `still` its first frame, as a live
    /// picture at the file's frame rate, so there is no loop to restart.
    File {
        path: PathBuf,
        #[serde(default)]
        still: bool,
    },
    /// RTP/H.264 from a camera outside the container, decoded here before the stamp. Datagrams
    /// from any address but `from` are dropped and counted.
    Ingest {
        listen: SocketAddr,
        from: Vec<IpAddr>,
    },
    /// A USB (UVC) camera through `v4l2src`, asked for `format` at this size and rate.
    V4l2 {
        device: PathBuf,
        format: CameraFormat,
        width: u16,
        height: u16,
        fps: u16,
    },
    /// A Raspberry Pi camera module through `libcamerasrc`.
    Libcamera { width: u16, height: u16, fps: u16 },
}

/// What `videotestsrc` draws. Snow is noise, which the encoder cannot compress: it fills any rung.
/// The zone plate is a still picture of concentric rings of rising frequency: fine detail
/// everywhere, the same in every run, so `crate::card` can render what the viewer should see.
#[derive(Debug, Clone, Copy, Default, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TestPattern {
    #[default]
    Ball,
    Snow,
    ZonePlate,
}

impl TestPattern {
    fn nick(self) -> &'static str {
        match self {
            Self::Ball => "ball",
            Self::Snow => "snow",
            Self::ZonePlate => "zone-plate",
        }
    }

    /// A `videotestsrc` drawing this pattern. The zone plate's rings are set here, not left to
    /// the element's defaults, which draw a flat picture.
    pub(crate) fn source(self, live: bool) -> Result<gst::Element, MediaError> {
        let source = gst::ElementFactory::make("videotestsrc")
            .property("is-live", live)
            .property_from_str("pattern", self.nick())
            .build()?;
        if matches!(self, Self::ZonePlate) {
            source.set_property("kx2", ZONE_PLATE_RINGS);
            source.set_property("ky2", ZONE_PLATE_RINGS);
        }
        Ok(source)
    }
}

const ZONE_PLATE_RINGS: i32 = 20;

/// One encoding setting: resolution, frame rate and bitrate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Rung {
    pub width: u16,
    pub height: u16,
    pub fps: u16,
    pub bitrate_kbit: u32,
    pub keyframe_interval_ms: u32,
}

impl Rung {
    pub fn validate(&self) -> Result<(), MediaError> {
        let rules = [
            (
                self.width.is_multiple_of(2) && self.height.is_multiple_of(2),
                "width and height must be even",
            ),
            (
                self.width <= MAX_WIDTH && self.height <= MAX_HEIGHT,
                "the frame must fit in 3840x2160",
            ),
            ((1..=MAX_FPS).contains(&self.fps), "fps must be 1-60"),
            (
                (8..=50_000).contains(&self.bitrate_kbit),
                "bitrate_kbit must be 8-50000",
            ),
            (
                (100..=60_000).contains(&self.keyframe_interval_ms),
                "keyframe_interval_ms must be 100-60000",
            ),
        ];
        if let Some((_, rule)) = rules.iter().find(|(holds, _)| !holds) {
            return Err(MediaError::Config((*rule).to_owned()));
        }
        stamp::check_size(usize::from(self.width), usize::from(self.height))
            .map_err(|e| MediaError::Config(e.to_string()))
    }

    /// Frames from one keyframe to the next, at least one.
    pub fn keyframe_frames(&self) -> u32 {
        u32::from(self.fps)
            .saturating_mul(self.keyframe_interval_ms)
            .div_ceil(1000)
            .max(1)
    }

    /// x264's VBV buffer: one frame period, the smallest x264 allows, so no frame (a keyframe
    /// included) bursts more than one frame's share of the rate onto the link.
    fn vbv_ms(&self) -> u32 {
        1000_u32.div_ceil(u32::from(self.fps.max(1)))
    }

    /// The encoder's bitrate, for x264 and V4L2 alike. With a VBV buffer of B ms, any one second
    /// carries at most the bitrate times (1000 + B) / 1000, keyframes included; this keeps that at
    /// the rung's bitrate.
    fn encoder_kbit(&self) -> u32 {
        let vbv_ms = self.vbv_ms();
        let kbit = u64::from(self.bitrate_kbit) * 1000 / (1000 + u64::from(vbv_ms));
        u32::try_from(kbit).unwrap_or(u32::MAX).max(1)
    }

    fn caps(&self) -> gst::Caps {
        gst_video::VideoCapsBuilder::new()
            .format(gst_video::VideoFormat::I420)
            .width(i32::from(self.width))
            .height(i32::from(self.height))
            .framerate(gst::Fraction::new(i32::from(self.fps), 1))
            .build()
    }

    /// Whether frames of `video` have this rung's size and rate.
    fn shapes(&self, video: &gst_video::VideoInfo) -> bool {
        let fps = gst::Fraction::new(i32::from(self.fps), 1);
        (video.width(), video.height(), video.fps())
            == (u32::from(self.width), u32::from(self.height), fps)
    }
}

/// The rung asked for, the request's number (counting from 1) and when.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Requested {
    pub rung: Rung,
    pub seq: u64,
    pub unix_ms: u64,
    #[serde(skip)]
    at: Instant,
}

/// The first frame stamped after request `seq` took effect: in its caps, and at x264's next frame
/// for its bitrate; `after_request_ms` on the monotonic clock.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Applied {
    pub rung: Rung,
    pub seq: u64,
    pub frame_id: u32,
    pub after_request_ms: u64,
    pub unix_ms: u64,
}

/// The running pipeline's rung filter and encoder, and the rung they were last set to.
#[derive(Debug)]
struct Handles {
    pipeline: gst::Pipeline,
    rung_filter: gst::Element,
    kind: Encoder,
    rung: Rung,
    /// The ingest's reading thread, stopped when the run ends.
    _ingest: Option<ingest::Reader>,
}

impl Handles {
    /// Takes `rung` as set and returns the change to make, outside the encoding lock: a new caps
    /// renegotiates through the streaming thread, which takes that lock for every frame.
    fn retarget(&mut self, rung: Rung) -> Change {
        let change = Change {
            pipeline: self.pipeline.clone(),
            rung_filter: self.rung_filter.clone(),
            kind: self.kind,
            from: self.rung,
            to: rung,
        };
        self.rung = rung;
        change
    }
}

struct Change {
    pipeline: gst::Pipeline,
    rung_filter: gst::Element,
    kind: Encoder,
    from: Rung,
    to: Rung,
}

impl Change {
    /// The new bitrate and VBV buffer take effect at the encoder's next frame. A new size or frame
    /// rate goes into the rung filter's caps: x264 reopens itself for them (`sat-teleop-x264`),
    /// with a refresh period of the same duration at the new rate, starting with an IDR frame.
    fn apply(self) {
        let rung = self.to;
        set_bitrate(&self.pipeline, self.kind, rung);
        let shape = |r: Rung| (r.width, r.height, r.fps);
        if shape(rung) != shape(self.from) {
            self.rung_filter.set_property("caps", rung.caps());
        }
    }
}

/// Sets the encoder's rate for `rung`. The V4L2 encoder takes it as a control; whether it applies
/// one while playing is for the first run on a board to show (`docs/hardware.md#encoder`).
fn set_bitrate(pipeline: &gst::Pipeline, kind: Encoder, rung: Rung) {
    let Some(encoder) = pipeline.by_name(ENCODER) else {
        return;
    };
    match kind {
        Encoder::X264 => {
            encoder.set_property("vbv-buf-capacity", rung.vbv_ms());
            encoder.set_property("bitrate", rung.encoder_kbit());
        }
        Encoder::V4l2 => encoder.set_property("extra-controls", v4l2_controls(rung)),
    }
}

/// `v4l2h264enc`'s controls: the rung's bitrate scaled as for x264 (`Rung::encoder_kbit`),
/// parameter sets with every keyframe and the rung's keyframe distance in frames.
fn v4l2_controls(rung: Rung) -> gst::Structure {
    gst::Structure::builder("controls")
        .field(
            "video_bitrate",
            i32::try_from(u64::from(rung.encoder_kbit()) * 1000).unwrap_or(i32::MAX),
        )
        .field("repeat_sequence_header", 1_i32)
        .field(
            "h264_i_frame_period",
            i32::try_from(rung.keyframe_frames()).unwrap_or(i32::MAX),
        )
        .build()
}

#[derive(Debug, Default)]
struct Encoding {
    requested: Option<Requested>,
    applied: Option<Applied>,
    pipeline: Option<Handles>,
}

/// What the source does, for testing the ground's freshness output: frames go out as captured,
/// none goes out, or the first frame after freezing goes out again and again with its stamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceMode {
    Live,
    Paused,
    Frozen,
}

impl SourceMode {
    const ALL: [Self; 3] = [Self::Live, Self::Paused, Self::Frozen];
}

/// Counters shared between the pipeline and the metrics endpoint. Frame ids keep counting across
/// pipeline restarts, so the receiver never sees an id repeat within one process.
#[derive(Debug, Default)]
pub struct SenderStats {
    pub health: Health,
    /// Index into [`SourceMode::ALL`].
    mode: AtomicU8,
    /// While the uplink queue drains, frames are dropped before the encoder, and the first one
    /// after asks for a keyframe: the relay drops packets that waited too long in the queue.
    draining: AtomicBool,
    /// The pace the video is held to, in bytes per second on the wire; zero for none.
    pace: AtomicU32,
    /// The pacer dropped packets: the next frame asks for a keyframe.
    keyframe_wanted: AtomicBool,
    paced_dropped: AtomicU64,
    next_frame_id: AtomicU32,
    frames_stamped: AtomicU64,
    stamp_failures: AtomicU64,
    frames_encoded: AtomicU64,
    snapshots: SnapshotCounts,
    rtp: Traffic,
    ingest: Arc<IngestStats>,
    /// When each frame in flight was stamped, for the stages below.
    stamped_at: Marks,
    in_encoder: InEncoder,
    /// When each encoded frame in flight was stamped, by the encoder's timestamp.
    encoded_at: Marks,
    stages: Stages,
    encoding: Mutex<Encoding>,
}

/// Where a frame's time goes in the vehicle (`crate::stages`).
#[derive(Debug, Default)]
struct Stages {
    /// From the source's capture to the stamp: conversion, scaling and rate control.
    before_stamp: Stage,
    /// From the stamp to the encoder's input: the leaky queue.
    queue: Stage,
    /// From the encoder's input to its output.
    encode: Stage,
    /// From the stamp to the last packet of the frame leaving for the relay: also the payloader
    /// and the pacer's backlog.
    sent: Stage,
}

#[derive(Debug, Serialize)]
pub struct StagesSnapshot {
    pub before_stamp: StageSnapshot,
    pub queue: StageSnapshot,
    pub encode: StageSnapshot,
    pub sent: StageSnapshot,
}

/// The snapshot branch's counters.
#[derive(Debug, Default)]
struct SnapshotCounts {
    taken: AtomicU64,
    /// The writer's queue was full: the snapshot was dropped, never the live frame.
    dropped: AtomicU64,
    /// A JPEG whose frame the branch had no identity for.
    unidentified: AtomicU64,
}

#[derive(Debug, Serialize)]
pub struct SnapshotCountsSnapshot {
    pub taken: u64,
    pub dropped: u64,
    pub unidentified: u64,
}

#[derive(Debug, Serialize)]
pub struct SenderSnapshot {
    pub requested: Option<Requested>,
    pub applied: Option<Applied>,
    pub mode: SourceMode,
    pub frames_stamped: u64,
    pub stamp_failures: u64,
    pub frames_encoded: u64,
    pub snapshots: SnapshotCountsSnapshot,
    pub rtp_packets: u64,
    pub rtp_bytes: u64,
    /// The pace the video is held to (bytes per second on the wire, zero for none) and the packets
    /// the pacer dropped for waiting longer than its age limit.
    pub pace_bytes_per_s: u32,
    pub paced_dropped: u64,
    /// Zero unless the source is an ingest.
    pub ingest: IngestSnapshot,
    pub stages: StagesSnapshot,
    #[serde(flatten)]
    pub health: HealthSnapshot,
}

impl SenderStats {
    pub fn mode(&self) -> SourceMode {
        let index = usize::from(self.mode.load(Ordering::Relaxed));
        SourceMode::ALL
            .get(index)
            .copied()
            .unwrap_or(SourceMode::Live)
    }

    pub fn set_mode(&self, mode: SourceMode) {
        let index = SourceMode::ALL.iter().position(|m| *m == mode).unwrap_or(0);
        self.mode
            .store(u8::try_from(index).unwrap_or(0), Ordering::Relaxed);
    }

    pub fn set_draining(&self, draining: bool) {
        self.draining.store(draining, Ordering::Relaxed);
    }

    /// The next frame is a keyframe.
    pub fn want_keyframe(&self) {
        self.keyframe_wanted.store(true, Ordering::Relaxed);
    }

    /// Holds the video to `bytes_per_s` on the wire from now on; `None` lets it go as it comes.
    pub fn set_pace(&self, bytes_per_s: Option<u32>) {
        self.pace.store(bytes_per_s.unwrap_or(0), Ordering::Relaxed);
    }

    pub fn draining(&self) -> bool {
        self.draining.load(Ordering::Relaxed)
    }

    /// Asks for `rung`: at once in the running pipeline, and in every pipeline started later.
    pub fn request(&self, rung: Rung) {
        let change = {
            let mut encoding = self.encoding();
            let seq = encoding.requested.map_or(1, |r| r.seq.saturating_add(1));
            encoding.requested = Some(Requested {
                rung,
                seq,
                unix_ms: unix_ms(),
                at: Instant::now(),
            });
            encoding
                .pipeline
                .as_mut()
                .map(|handles| handles.retarget(rung))
        };
        if let Some(change) = change {
            change.apply();
        }
    }

    pub fn requested(&self) -> Option<Rung> {
        self.encoding().requested.map(|r| r.rung)
    }

    /// RTP packets and their bytes, and encoded frames, so far.
    pub fn sent(&self) -> (TrafficSnapshot, u64) {
        (
            self.rtp.snapshot(),
            self.frames_encoded.load(Ordering::Relaxed),
        )
    }

    pub fn snapshot(&self) -> SenderSnapshot {
        let rtp = self.rtp.snapshot();
        let (requested, applied) = {
            let encoding = self.encoding();
            (encoding.requested, encoding.applied)
        };
        SenderSnapshot {
            requested,
            applied,
            mode: self.mode(),
            frames_stamped: self.frames_stamped.load(Ordering::Relaxed),
            stamp_failures: self.stamp_failures.load(Ordering::Relaxed),
            frames_encoded: self.frames_encoded.load(Ordering::Relaxed),
            snapshots: SnapshotCountsSnapshot {
                taken: self.snapshots.taken.load(Ordering::Relaxed),
                dropped: self.snapshots.dropped.load(Ordering::Relaxed),
                unidentified: self.snapshots.unidentified.load(Ordering::Relaxed),
            },
            rtp_packets: rtp.packets,
            rtp_bytes: rtp.bytes,
            pace_bytes_per_s: self.pace.load(Ordering::Relaxed),
            paced_dropped: self.paced_dropped.load(Ordering::Relaxed),
            ingest: self.ingest.snapshot(),
            stages: StagesSnapshot {
                before_stamp: self.stages.before_stamp.snapshot(),
                queue: self.stages.queue.snapshot(),
                encode: self.stages.encode.snapshot(),
                sent: self.stages.sent.snapshot(),
            },
            health: self.health.snapshot(),
        }
    }

    fn encoding(&self) -> MutexGuard<'_, Encoding> {
        self.encoding.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Lends the running pipeline's elements to [`Self::request`], first setting them to a rung
    /// requested while the pipeline was being built.
    fn attach(&self, mut handles: Handles) -> Attached<'_> {
        let change = {
            let mut encoding = self.encoding();
            let change = encoding
                .requested
                .filter(|r| r.rung != handles.rung)
                .map(|r| handles.retarget(r.rung));
            encoding.pipeline = Some(handles);
            change
        };
        if let Some(change) = change {
            change.apply();
        }
        Attached(self)
    }

    /// Marks the newest request applied with `frame_id` once frames of `video` have its shape.
    fn stamped(&self, frame_id: FrameId, video: &gst_video::VideoInfo) {
        let mut encoding = self.encoding();
        let Some(requested) = encoding.requested else {
            return;
        };
        let applied = encoding.applied.is_some_and(|a| a.seq == requested.seq);
        if applied || !requested.rung.shapes(video) {
            return;
        }
        encoding.applied = Some(Applied {
            rung: requested.rung,
            seq: requested.seq,
            frame_id: frame_id.0,
            after_request_ms: u64::try_from(requested.at.elapsed().as_millis()).unwrap_or(u64::MAX),
            unix_ms: unix_ms(),
        });
    }
}

/// Takes the pipeline's elements back from [`SenderStats`] when its run ends.
struct Attached<'a>(&'a SenderStats);

impl Drop for Attached<'_> {
    fn drop(&mut self) {
        self.0.encoding().pipeline = None;
    }
}

/// Builds and plays the sender at the requested rung until it fails. A file source starts over at
/// its end. Snapshots go to `snapshots`, never blocking: a full queue drops the snapshot. Never
/// returns `Ok`; run it under [`crate::supervise`].
pub fn run(
    config: &SenderConfig,
    stats: &Arc<SenderStats>,
    hooks: &Hooks,
    snapshots: &SyncSender<Snapshot>,
) -> Result<(), MediaError> {
    let rung = stats
        .requested()
        .ok_or(MediaError::Missing("requested rung"))?;
    rung.validate()?;
    let (pipeline, handles) = build(config, rung, stats, hooks, snapshots)?;
    let running = Running(pipeline);
    let _attached = stats.attach(handles);
    running.set(gst::State::Playing)?;
    let bus = running.0.bus().ok_or(MediaError::Missing("bus"))?;
    let looped = matches!(config.source, Source::File { still: false, .. });
    loop {
        let watched = [gst::MessageType::Error, gst::MessageType::Eos];
        let Some(message) = bus.timed_pop_filtered(gst::ClockTime::NONE, &watched) else {
            continue;
        };
        let at_end = matches!(message.view(), gst::MessageView::Eos(_));
        if !(looped && at_end) {
            return Err(failure(&message).unwrap_or(MediaError::Eos));
        }
        // y4mdec cannot do segment seeks, so a file loops with a flushing seek at its end.
        running
            .0
            .seek_simple(gst::SeekFlags::FLUSH, gst::ClockTime::ZERO)?;
    }
}

fn build(
    config: &SenderConfig,
    rung: Rung,
    stats: &Arc<SenderStats>,
    hooks: &Hooks,
    snapshots: &SyncSender<Snapshot>,
) -> Result<(gst::Pipeline, Handles), MediaError> {
    let (mut head, ingest) = source(&config.source, &stats.ingest)?;
    let capture = match (&config.source, head.first()) {
        (Source::V4l2 { .. }, Some(camera)) => {
            let clock = Arc::new(CaptureClock::default());
            camera::watch_capture(camera, &clock)?;
            Some(clock)
        }
        _ => None,
    };
    let tee = gst::ElementFactory::make("tee")
        .property("allow-not-linked", true)
        .build()?;
    head.extend([
        gst::ElementFactory::make("videoconvert").build()?,
        tee.clone(),
    ]);
    let mut chain = vec![tee];
    let rung_filter = gst::ElementFactory::make("capsfilter")
        .property("caps", rung.caps())
        .build()?;
    let kind = camera::choose_encoder(config.encoder, camera::hardware_encoder_present());
    if config.encoder == EncoderWish::Hardware && kind == Encoder::X264 {
        eprintln!("no v4l2h264enc on this board: encoding with x264");
    }
    let encoder = make_encoder(kind, rung, stats)?;
    let sink = pacer::sink(config.destination, stats)?;
    chain.extend([
        gst::ElementFactory::make("videoscale").build()?,
        // Without drop-only, videorate holds every frame until the next one arrives, a full
        // frame interval of latency before the stamp can see it.
        gst::ElementFactory::make("videorate")
            .property("drop-only", true)
            .build()?,
        rung_filter.clone(),
        gst::ElementFactory::make("queue")
            .property_from_str("leaky", "downstream")
            .property("max-size-buffers", 2u32)
            .property("max-size-bytes", 0u32)
            .property("max-size-time", 0u64)
            .build()?,
        encoder,
        gst::ElementFactory::make("capsfilter")
            .property("caps", encoded_caps(kind))
            .build()?,
        gst::ElementFactory::make("rtph264pay")
            .property("config-interval", 1i32)
            .property("pt", RTP_PAYLOAD_TYPE)
            .property("mtu", RTP_MTU)
            .build()?,
        sink.clone().upcast(),
    ]);
    let pipeline = gst::Pipeline::new();
    pipeline.add_many(&head)?;
    pipeline.add_many(chain.iter().skip(1))?;
    gst::Element::link_many(&head)?;
    gst::Element::link_many(&chain)?;
    if let Some(snapshot) = config.snapshot {
        snapshot.validate()?;
        let mut branch = vec![head.last().cloned().ok_or(MediaError::Missing("tee"))?];
        branch.extend(snapshot_branch(snapshot, stats, snapshots)?);
        pipeline.add_many(branch.iter().skip(1))?;
        gst::Element::link_many(&branch)?;
    }
    for tap in &hooks.taps {
        let mut branch = vec![head.last().cloned().ok_or(MediaError::Missing("tee"))?];
        branch.extend(tap::branch(tap)?);
        pipeline.add_many(branch.iter().skip(1))?;
        gst::Element::link_many(&branch)?;
    }
    let keyframes = Keyframes {
        pipeline: pipeline.downgrade(),
        every: periodic_keyframe_every(kind, rung),
        last: None,
    };
    stamp_frames(&rung_filter, keyframes, stats, hooks, capture)?;
    let handles = Handles {
        pipeline: pipeline.clone(),
        rung_filter,
        kind,
        rung,
        _ingest: ingest,
    };
    Ok((pipeline, handles))
}

/// The encoder element for `kind`, counted into `stats`. x264 (`sat-teleop-x264`) is configured for
/// the lowest latency and refreshes the picture with a moving column of intra blocks over the
/// rung's keyframe interval; it gives a whole IDR frame only when asked for one.
fn make_encoder(
    kind: Encoder,
    rung: Rung,
    stats: &Arc<SenderStats>,
) -> Result<gst::Element, MediaError> {
    let encoder = match kind {
        Encoder::X264 => {
            sat_teleop_x264::register()?;
            gst::ElementFactory::make(sat_teleop_x264::ELEMENT)
                .name(ENCODER)
                .property("bitrate", rung.encoder_kbit())
                .property("vbv-buf-capacity", rung.vbv_ms())
                .property("refresh-ms", rung.keyframe_interval_ms)
                .build()?
        }
        Encoder::V4l2 => gst::ElementFactory::make("v4l2h264enc")
            .name(ENCODER)
            .property("extra-controls", v4l2_controls(rung))
            .build()?,
    };
    count_encoded(&encoder, stats)?;
    Ok(encoder)
}

/// The caps after the encoder: the V4L2 encoder reads its profile and level from them before
/// negotiation.
fn encoded_caps(kind: Encoder) -> gst::Caps {
    match kind {
        Encoder::X264 => gst::Caps::new_any(),
        Encoder::V4l2 => gst::Caps::builder("video/x-h264")
            .field("profile", "main")
            .field("level", "4")
            .build(),
    }
}

/// How often the stamp probe forces a keyframe by time. x264 refreshes the picture by itself every
/// refresh period, so a whole IDR frame only on request; the V4L2 encoder gets a keyframe every interval.
fn periodic_keyframe_every(kind: Encoder, rung: Rung) -> Duration {
    match kind {
        Encoder::X264 => Duration::MAX,
        Encoder::V4l2 => Duration::from_millis(u64::from(rung.keyframe_interval_ms)),
    }
}

/// Forces a keyframe every `every` by time, whatever the frame rate.
#[derive(Debug)]
struct Keyframes {
    pipeline: gst::glib::WeakRef<gst::Pipeline>,
    every: Duration,
    /// The last keyframe asked for, or the encoder's start (new caps start it with one); `None`
    /// when one is due.
    last: Option<Instant>,
}

impl Keyframes {
    fn restarted(&mut self) {
        self.last = Some(Instant::now());
    }

    fn due(&mut self) {
        self.last = None;
    }

    /// Before a frame goes to the encoder: asks for a keyframe when one is due.
    fn frame(&mut self) {
        let now = Instant::now();
        if self
            .last
            .is_some_and(|last| now.duration_since(last) < self.every)
        {
            return;
        }
        let Some(encoder) = self.pipeline.upgrade().and_then(|p| p.by_name(ENCODER)) else {
            return;
        };
        let event = gst_video::UpstreamForceKeyUnitEvent::builder()
            .all_headers(true)
            .build();
        encoder.send_event(event);
        self.last = Some(now);
    }
}

fn source(
    source: &Source,
    ingest: &Arc<IngestStats>,
) -> Result<(Vec<gst::Element>, Option<ingest::Reader>), MediaError> {
    match source {
        Source::Ingest { listen, from } => {
            let (elements, reader) = ingest::source(*listen, from, ingest)?;
            Ok((elements, Some(reader)))
        }
        Source::TestPattern {
            width,
            height,
            pattern,
        } => Ok((test_pattern(*width, *height, *pattern)?, None)),
        Source::File { path, still } => Ok((file_source(path, *still)?, None)),
        Source::V4l2 {
            device,
            format,
            width,
            height,
            fps,
        } => {
            check_mode(*width, *height, *fps)?;
            Ok((
                camera::v4l2_source(device, *format, *width, *height, *fps)?,
                None,
            ))
        }
        Source::Libcamera { width, height, fps } => {
            check_mode(*width, *height, *fps)?;
            Ok((camera::libcamera_source(*width, *height, *fps)?, None))
        }
    }
}

/// A camera mode within what the stamp and the sender carry.
fn check_mode(width: u16, height: u16, fps: u16) -> Result<(), MediaError> {
    let fits = (1..=MAX_WIDTH).contains(&width)
        && (1..=MAX_HEIGHT).contains(&height)
        && (1..=MAX_FPS).contains(&fps);
    if fits {
        return Ok(());
    }
    Err(MediaError::Config(format!(
        "camera mode {width}x{height} at {fps} fps is outside 3840x2160 and 60 fps"
    )))
}

fn test_pattern(
    width: Option<u16>,
    height: Option<u16>,
    pattern: TestPattern,
) -> Result<Vec<gst::Element>, MediaError> {
    let pattern = pattern.source(true)?;
    let mut size = gst::Caps::builder("video/x-raw")
        .field("framerate", gst::Fraction::new(TEST_PATTERN_FPS, 1));
    if let Some(width) = width {
        size = size.field("width", i32::from(width));
    }
    if let Some(height) = height {
        size = size.field("height", i32::from(height));
    }
    let filter = gst::ElementFactory::make("capsfilter")
        .property("caps", size.build())
        .build()?;
    Ok(vec![pattern, filter])
}

fn file_source(path: &Path, still: bool) -> Result<Vec<gst::Element>, MediaError> {
    let mut elements = vec![
        gst::ElementFactory::make("filesrc")
            .property("location", path.display().to_string())
            .build()?,
        gst::ElementFactory::make("y4mdec").build()?,
    ];
    if still {
        elements.push(
            gst::ElementFactory::make("imagefreeze")
                .property("is-live", true)
                .build()?,
        );
        return Ok(elements);
    }
    // The file is not live: pace it to real time, since the sink does not sync.
    elements.push(gst::ElementFactory::make("clocksync").build()?);
    Ok(elements)
}

fn sink_pad(element: &gst::Element) -> Result<gst::Pad, MediaError> {
    element
        .static_pad("sink")
        .ok_or(MediaError::Missing("sink pad"))
}

fn src_pad(element: &gst::Element) -> Result<gst::Pad, MediaError> {
    element
        .static_pad("src")
        .ok_or(MediaError::Missing("src pad"))
}

/// Draws the stamp into every frame leaving the rung filter, and drops or repeats frames as the
/// source mode says, or drops them while the uplink drains. The frame format is taken from the caps
/// event once, not parsed from the caps for every frame. Also asks for the keyframes `keyframes`
/// keeps time for.
fn stamp_frames(
    rung_filter: &gst::Element,
    keyframes: Keyframes,
    stats: &Arc<SenderStats>,
    hooks: &Hooks,
    capture: Option<Arc<CaptureClock>>,
) -> Result<(), MediaError> {
    let (stats, hook) = (Arc::clone(stats), hooks.frame.clone());
    let format = Mutex::new(None);
    let frozen = Mutex::new(None);
    let keyframes = Mutex::new(keyframes);
    src_pad(rung_filter)?.add_probe(STAMP_PROBE, move |pad, info| {
        let mut keyframes = keyframes.lock().unwrap_or_else(PoisonError::into_inner);
        let buffer = match &mut info.data {
            Some(gst::PadProbeData::Event(event)) => {
                if remember_format(event, &format) {
                    keyframes.restarted();
                }
                return gst::PadProbeReturn::Ok;
            }
            Some(gst::PadProbeData::Buffer(buffer)) => buffer,
            _ => return gst::PadProbeReturn::Ok,
        };
        if stats.draining.load(Ordering::Relaxed) {
            keyframes.due();
            return gst::PadProbeReturn::Drop;
        }
        let mut frozen = frozen.lock().unwrap_or_else(PoisonError::into_inner);
        match stats.mode() {
            SourceMode::Live => {
                *frozen = None;
                record_before_stamp(pad, buffer, &stats);
                stamp_next(buffer, &format, &stats, hook.as_deref(), capture.as_deref());
            }
            SourceMode::Paused => {
                *frozen = None;
                return gst::PadProbeReturn::Drop;
            }
            SourceMode::Frozen => {
                repeat_frozen(
                    buffer,
                    &mut frozen,
                    &format,
                    &stats,
                    hook.as_deref(),
                    capture.as_deref(),
                );
            }
        }
        if stats.keyframe_wanted.swap(false, Ordering::Relaxed) {
            keyframes.due();
        }
        keyframes.frame();
        gst::PadProbeReturn::Ok
    });
    Ok(())
}

/// Records how long the frame has been in the pipeline when it reaches the stamp, and remembers
/// when it was stamped.
fn record_before_stamp(pad: &gst::Pad, buffer: &gst::Buffer, stats: &SenderStats) {
    let Some(pts) = buffer.pts() else {
        return;
    };
    stats.stamped_at.mark(pts.nseconds());
    let now = pad.parent_element().and_then(|e| e.current_running_time());
    if let Some(elapsed) = now.and_then(|now| now.checked_sub(pts)) {
        stats
            .stages
            .before_stamp
            .record(Duration::from_nanos(elapsed.nseconds()));
    }
}

fn stamp_next(
    buffer: &mut gst::Buffer,
    format: &Mutex<Option<gst_video::VideoInfo>>,
    stats: &SenderStats,
    hook: Option<&(dyn Fn(&mut Frame) + Send + Sync)>,
    capture: Option<&CaptureClock>,
) {
    let frame_id = FrameId(stats.next_frame_id.fetch_add(1, Ordering::Relaxed));
    let video = format.lock().ok().and_then(|format| format.clone());
    let Some(video) = video else {
        stats.stamp_failures.fetch_add(1, Ordering::Relaxed);
        return;
    };
    let capture_ms = capture
        .and_then(CaptureClock::latest_ms)
        .unwrap_or_else(unix_ms);
    let counter = match stamp_buffer(buffer, &video, frame_id, capture_ms, hook) {
        Ok(()) => &stats.frames_stamped,
        Err(_) => &stats.stamp_failures,
    };
    counter.fetch_add(1, Ordering::Relaxed);
    stats.stamped(frame_id, &video);
}

/// Replaces `buffer` with the frame frozen first, stamp and all, on `buffer`'s timestamps.
fn repeat_frozen(
    buffer: &mut gst::Buffer,
    frozen: &mut Option<gst::Buffer>,
    format: &Mutex<Option<gst_video::VideoInfo>>,
    stats: &SenderStats,
    hook: Option<&(dyn Fn(&mut Frame) + Send + Sync)>,
    capture: Option<&CaptureClock>,
) {
    let Some(held) = frozen.as_ref() else {
        stamp_next(buffer, format, stats, hook, capture);
        *frozen = buffer.copy_deep().ok();
        return;
    };
    let mut repeat = held.copy();
    {
        let repeat = repeat.make_mut();
        repeat.set_pts(buffer.pts());
        repeat.set_dts(buffer.dts());
        repeat.set_duration(buffer.duration());
    }
    *buffer = repeat;
}

/// Keeps the format of a caps event; true when it was one.
fn remember_format(event: &gst::Event, format: &Mutex<Option<gst_video::VideoInfo>>) -> bool {
    let gst::EventView::Caps(caps) = event.view() else {
        return false;
    };
    let video = gst_video::VideoInfo::from_caps(caps.caps()).ok();
    if let Ok(mut format) = format.lock() {
        *format = video;
    }
    true
}

fn stamp_buffer(
    buffer: &mut gst::Buffer,
    video: &gst_video::VideoInfo,
    frame_id: FrameId,
    capture_ms: u64,
    hook: Option<&(dyn Fn(&mut Frame) + Send + Sync)>,
) -> Result<(), MediaError> {
    let mut frame = gst_video::VideoFrameRef::from_buffer_ref_writable(buffer.make_mut(), video)?;
    if let Some(hook) = hook {
        hook(&mut frame);
    }
    let geometry = luma_geometry(&frame)?;
    let stamp = Stamp {
        frame_id,
        capture_ms: CaptureMs::from_unix_ms(capture_ms),
    };
    stamp::draw(frame.plane_data_mut(0)?, geometry, stamp)?;
    Ok(())
}

/// The snapshot branch after the tee: a leaky queue of one buffer so it never holds the live frames
/// back, a videorate down to the snapshot rate, `jpegenc` and an appsink that drops what the
/// caller's writer cannot take. Frames are thinned to the snapshot rate and copied as they enter the branch
/// (`copy_frames`), so no element in it, the queue and the videorate's remembered frame
/// included, holds a camera buffer. Returns the elements after the tee in order.
fn snapshot_branch(
    config: SnapshotConfig,
    stats: &Arc<SenderStats>,
    snapshots: &SyncSender<Snapshot>,
) -> Result<Vec<gst::Element>, MediaError> {
    let queue = gst::ElementFactory::make("queue")
        .property_from_str("leaky", "downstream")
        .property("max-size-buffers", 1u32)
        .property("max-size-bytes", 0u32)
        .property("max-size-time", 0u64)
        .build()?;
    copy_frames(&queue, config.fps)?;
    let rate = gst::ElementFactory::make("videorate")
        .property("drop-only", true)
        .build()?;
    let at_rate = gst::ElementFactory::make("capsfilter")
        .property(
            "caps",
            gst::Caps::builder("video/x-raw")
                .field("framerate", gst::Fraction::new(i32::from(config.fps), 1))
                .build(),
        )
        .build()?;
    let encoder = gst::ElementFactory::make("jpegenc")
        .property("quality", i32::from(config.quality))
        .build()?;
    let sink = gst_app::AppSink::builder()
        .max_buffers(2)
        .drop(true)
        .sync(false)
        .build();
    let identities = Arc::new(Mutex::new(VecDeque::new()));
    identify_frames(&at_rate, stats, &identities)?;
    hand_over(&sink, stats, snapshots, identities);
    Ok(vec![queue, rate, at_rate, encoder, sink.upcast()])
}

/// Thins the frames entering `element` to about `fps` a second and replaces each one it keeps with
/// a copy, so the camera's buffer goes back to its pool (V4L2 defaults to four) at once and only
/// the kept frames cost a copy. A frame that cannot be copied goes on as it is.
fn copy_frames(element: &gst::Element, fps: u16) -> Result<(), MediaError> {
    let pad = element
        .static_pad("sink")
        .ok_or(MediaError::Missing("sink pad"))?;
    let interval = 1_000_000_000 / u64::from(fps.max(1));
    let slots = Mutex::new(None::<(u64, u64)>);
    pad.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        let Some(gst::PadProbeData::Buffer(buffer)) = &mut info.data else {
            return gst::PadProbeReturn::Ok;
        };
        let pts = buffer.pts().map_or(0, gst::ClockTime::nseconds);
        let mut slots = slots.lock().unwrap_or_else(PoisonError::into_inner);
        let Some(kept) = thin(*slots, pts, interval) else {
            return gst::PadProbeReturn::Drop;
        };
        *slots = Some(kept);
        if let Ok(copy) = buffer.copy_deep() {
            *buffer = copy;
        }
        gst::PadProbeReturn::Ok
    });
    Ok(())
}

/// Decides whether the frame at `pts` is kept, given the slot grid `(origin, due)` of the last kept
/// frame, and returns the grid after it. After a kept frame `due` is at most one interval ahead, so
/// a frame more than an interval below `due` follows a backwards jump (a flushing seek, a segment
/// reset): the grid restarts at that frame instead of dropping everything until time catches up.
fn thin(state: Option<(u64, u64)>, pts: u64, interval: u64) -> Option<(u64, u64)> {
    let (origin, due) = match state {
        Some((_, due)) if pts.saturating_add(interval) < due => (pts, pts),
        Some(state) => state,
        None => (pts, pts),
    };
    (pts >= due).then(|| (origin, next_due(origin, pts, interval)))
}

/// The presentation time from which a frame is kept for the videorate slot after `pts`. Its slots
/// are `origin + n * interval` and it passes the frame nearest each one, so the due time sits a
/// quarter interval before the slot: a kept frame is then always the nearest to its slot, and
/// because the due times stay on the slot grid no kept frame is lost to drift.
fn next_due(origin: u64, pts: u64, interval: u64) -> u64 {
    let margin = interval / 4;
    let slot = pts.saturating_sub(origin).saturating_add(margin) / interval + 1;
    origin.saturating_add(slot.saturating_mul(interval).saturating_sub(margin))
}

/// When a frame leaves the snapshot rate: its presentation time and the identity it gets there.
type Identities = Mutex<VecDeque<(Option<gst::ClockTime>, Identity)>>;

#[derive(Debug, Clone, Copy)]
struct Identity {
    capture_ms: u64,
    frame_id: u32,
}

/// Notes who each frame that the snapshot rate lets through is, before the JPEG encoder's time
/// moves the clock: the encoder keeps the presentation time, which finds the identity again.
fn identify_frames(
    at_rate: &gst::Element,
    stats: &Arc<SenderStats>,
    identities: &Arc<Identities>,
) -> Result<(), MediaError> {
    let (stats, identities) = (Arc::clone(stats), Arc::clone(identities));
    src_pad(at_rate)?.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        let Some(gst::PadProbeData::Buffer(buffer)) = &info.data else {
            return gst::PadProbeReturn::Ok;
        };
        let identity = Identity {
            capture_ms: unix_ms(),
            frame_id: stats.next_frame_id.load(Ordering::Relaxed),
        };
        let mut waiting = identities.lock().unwrap_or_else(PoisonError::into_inner);
        if waiting.len() >= SNAPSHOT_IN_FLIGHT {
            waiting.pop_front();
        }
        waiting.push_back((buffer.pts(), identity));
        gst::PadProbeReturn::Ok
    });
    Ok(())
}

/// Gives each JPEG the appsink pulls, with its identity, to the writer's queue; when that is full
/// the snapshot is dropped and counted.
fn hand_over(
    sink: &gst_app::AppSink,
    stats: &Arc<SenderStats>,
    snapshots: &SyncSender<Snapshot>,
    identities: Arc<Identities>,
) {
    let (stats, snapshots) = (Arc::clone(stats), snapshots.clone());
    let pulled = move |sink: &gst_app::AppSink| {
        let sample = sink.pull_sample().map_err(|_| gst::FlowError::Eos)?;
        let Some(buffer) = sample.buffer() else {
            return Ok(gst::FlowSuccess::Ok);
        };
        let identity = {
            let mut waiting = identities.lock().unwrap_or_else(PoisonError::into_inner);
            let at = waiting.iter().position(|(pts, _)| *pts == buffer.pts());
            at.and_then(|at| waiting.remove(at))
                .map(|(_, identity)| identity)
        };
        let Some(identity) = identity else {
            stats.snapshots.unidentified.fetch_add(1, Ordering::Relaxed);
            return Ok(gst::FlowSuccess::Ok);
        };
        let Ok(map) = buffer.map_readable() else {
            return Ok(gst::FlowSuccess::Ok);
        };
        let snapshot = Snapshot {
            capture_ms: identity.capture_ms,
            frame_id: identity.frame_id,
            jpeg: map.as_slice().to_vec(),
        };
        let counter = match snapshots.try_send(snapshot) {
            Ok(()) => &stats.snapshots.taken,
            Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) => &stats.snapshots.dropped,
        };
        counter.fetch_add(1, Ordering::Relaxed);
        Ok(gst::FlowSuccess::Ok)
    };
    sink.set_callbacks(
        gst_app::AppSinkCallbacks::builder()
            .new_sample(pulled)
            .build(),
    );
}

fn count_encoded(encoder: &gst::Element, stats: &Arc<SenderStats>) -> Result<(), MediaError> {
    let entering = Arc::clone(stats);
    sink_pad(encoder)?.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        let pts = buffer_pts(info.data.as_ref());
        let stamped = pts.and_then(|pts| entering.stamped_at.at(pts));
        if let Some(stamped) = stamped {
            entering.stages.queue.record(stamped.elapsed());
            entering.in_encoder.enter(stamped);
        }
        gst::PadProbeReturn::Ok
    });
    let stats = Arc::clone(stats);
    src_pad(encoder)?.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        stats.frames_encoded.fetch_add(1, Ordering::Relaxed);
        let left = stats.in_encoder.leave();
        if let (Some((stamped, inside)), Some(pts)) = (left, buffer_pts(info.data.as_ref())) {
            stats.stages.encode.record(inside);
            stats.encoded_at.mark_at(pts, stamped);
        }
        gst::PadProbeReturn::Ok
    });
    Ok(())
}

fn buffer_pts(data: Option<&gst::PadProbeData>) -> Option<u64> {
    match data {
        Some(gst::PadProbeData::Buffer(buffer)) => buffer.pts().map(gst::ClockTime::nseconds),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEFAULT: Rung = Rung {
        width: 640,
        height: 360,
        fps: 15,
        bitrate_kbit: 500,
        keyframe_interval_ms: 2000,
    };

    #[test]
    fn snapshot_frames_are_thinned_to_the_slot_grid_and_survive_a_backwards_jump() {
        const S: u64 = 1_000_000_000;
        let due = S - S / 4;
        // (state, pts, grid after the frame, or None if dropped); slots at 1 s, 2 s, ...
        for (state, pts, kept) in [
            (None, 0, Some((0, due))),
            (Some((0, due)), due - 1, None),
            (Some((0, due)), due, Some((0, 2 * S - S / 4))),
            (Some((0, due)), due + 33_000_000, Some((0, 2 * S - S / 4))),
            (Some((0, 5 * S)), 5 * S + 10, Some((0, 6 * S - S / 4))),
            // looped source: time restarts at 0 while the grid is waiting for 5 s
            (Some((0, 5 * S - S / 4)), 10, Some((10, S + 10 - S / 4))),
            (Some((10, S + 10 - S / 4)), 33_000_000, None),
        ] {
            assert_eq!(thin(state, pts, S), kept, "{state:?} pts {pts}");
        }
    }

    #[test]
    fn the_hook_runs_before_the_stamp_is_drawn() {
        crate::init().unwrap();
        let video = gst_video::VideoInfo::builder(gst_video::VideoFormat::I420, 640, 360)
            .build()
            .unwrap();
        let mut buffer = gst::Buffer::with_size(video.size()).unwrap();
        // A hook that paints the whole luma plane, the stamp's rows included.
        let paint = |frame: &mut Frame| frame.plane_data_mut(0).unwrap().fill(200);
        stamp_buffer(&mut buffer, &video, FrameId(5), 1_234_567, Some(&paint)).unwrap();

        let frame =
            gst_video::VideoFrameRef::from_buffer_ref_readable(buffer.as_ref(), &video).unwrap();
        let geometry = luma_geometry(&frame).unwrap();
        let luma = frame.plane_data(0).unwrap();
        assert_eq!(luma[300 * geometry.stride + 101], 200, "the hook drew");
        let stamp = stamp::read(luma, geometry).expect("the stamp survives the hook");
        assert_eq!(stamp.frame_id, FrameId(5));
        assert_eq!(stamp.capture_ms, CaptureMs::from_unix_ms(1_234_567));
    }

    #[test]
    fn rejects_rungs_the_stamp_or_encoder_cannot_carry() {
        let cases = [
            (DEFAULT, true),
            (
                Rung {
                    width: 192,
                    height: 108,
                    fps: 2,
                    bitrate_kbit: 50,
                    ..DEFAULT
                },
                true,
            ),
            (
                Rung {
                    width: 190,
                    height: 108,
                    ..DEFAULT
                },
                false,
            ),
            (
                Rung {
                    width: 641,
                    ..DEFAULT
                },
                false,
            ),
            (
                Rung {
                    width: 7680,
                    height: 4320,
                    ..DEFAULT
                },
                false,
            ),
            (Rung { fps: 0, ..DEFAULT }, false),
            (
                Rung {
                    bitrate_kbit: 0,
                    ..DEFAULT
                },
                false,
            ),
            (
                Rung {
                    keyframe_interval_ms: 0,
                    ..DEFAULT
                },
                false,
            ),
        ];
        for (rung, valid) in cases {
            assert_eq!(rung.validate().is_ok(), valid, "{rung:?}");
        }
    }

    #[test]
    fn keyframe_interval_rounds_up_to_whole_frames() {
        let cases = [(15, 2000, 30), (2, 2000, 4), (15, 100, 2), (1, 100, 1)];
        for (fps, interval_ms, frames) in cases {
            let rung = Rung {
                fps,
                keyframe_interval_ms: interval_ms,
                ..DEFAULT
            };
            assert_eq!(rung.keyframe_frames(), frames, "{rung:?}");
        }
    }
}
