//! Regional video on the vehicle: the `[region]` limits, what the sender needs to give the encoder
//! the regions to spend more on and outline the people and vehicles found, and the detector's counters. The
//! detector itself (`sat-teleop-detect`) is in a sibling crate, so the application runs it: the
//! sender offers it the newest scaled frame (`DetectorFrame`) and it publishes what it finds to the cell.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use gstreamer as gst;
use gstreamer_video as gst_video;
use gstreamer_video::prelude::*;
use serde::{Deserialize, Serialize};

use sat_teleop_policy::draw::{outline, tag};
use sat_teleop_policy::follow::{self, Lost};
use sat_teleop_policy::kind::Kind;
use sat_teleop_policy::region::{NormRect, PixelRect, RegionCell, RegionLimits};

use sat_teleop_video::sender::{FrameHook, Roi, Tap, TapFormat, TapSpec};
use sat_teleop_video::stages::{Stage, StageSnapshot};
use sat_teleop_video::{MediaError, plane_geometry};

fn wait_forever() -> ! {
    loop {
        std::thread::park();
    }
}

fn follow_side(pixels: usize) -> u32 {
    u32::try_from(pixels).unwrap_or(u32::MAX)
}

const MIN_ROI_QP_OFFSET: f32 = -51.0;
const MAX_REGIONS: usize = 16;
const MAX_DETECTOR_FPS: u16 = 10;
const MAX_TTL_MS: u64 = 60_000;
const MAX_FOLLOW_FPS: u16 = 30;
const MAX_LOST_FRAMES: u32 = 60;
/// The letter of a tag in Y, U and V: black.
const TAG_INK: [u8; 3] = [16, 128, 128];
/// An outline is this many pixels at a luma width of `OUTLINE_REFERENCE_WIDTH`, and scales with it.
const OUTLINE_PX: usize = 2;
const OUTLINE_REFERENCE_WIDTH: usize = 640;

/// The quantiser offset inside the regions from `[region]` of the vehicle config: 0 (the default) is
/// off, down to -51 is finer.
pub fn validate_roi_qp_offset(offset: f32) -> Result<f32, MediaError> {
    if (MIN_ROI_QP_OFFSET..=0.0).contains(&offset) {
        return Ok(offset);
    }
    Err(MediaError::Config(
        "roi_qp_offset must be -51 to 0".to_owned(),
    ))
}

/// `[region]` of `testbed/limits.toml`. The ground reads `pilot_refresh_ms` from the same table.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct RegionSettings {
    pub pilot_ttl_ms: u64,
    pub detection_ttl_ms: u64,
    pub max_regions: usize,
    pub max_area_percent: u8,
    pub detector_fps: u16,
    pub detector_min_score: f32,
    /// How often the tracker takes a frame.
    #[serde(default = "default_follow_fps")]
    pub follow_fps: u16,
    /// The correlation a match must reach to hold the followed box.
    #[serde(default = "default_follow_min_score")]
    pub follow_min_score: f32,
    /// Frames in a row under `follow_min_score` that end the follow as lost.
    #[serde(default = "default_follow_lost_frames")]
    pub follow_lost_frames: u32,
}

fn default_follow_fps() -> u16 {
    10
}

fn default_follow_min_score() -> f32 {
    0.5
}

fn default_follow_lost_frames() -> u32 {
    5
}

impl RegionSettings {
    pub fn validate(self) -> Result<Self, MediaError> {
        let rules = [
            (
                (1..=MAX_TTL_MS).contains(&self.pilot_ttl_ms)
                    && (1..=MAX_TTL_MS).contains(&self.detection_ttl_ms),
                "region ttls must be 1-60000 ms",
            ),
            (
                (1..=MAX_REGIONS).contains(&self.max_regions),
                "max_regions must be 1-16",
            ),
            (
                (1..=100).contains(&self.max_area_percent),
                "max_area_percent must be 1-100",
            ),
            (
                (1..=MAX_DETECTOR_FPS).contains(&self.detector_fps),
                "detector_fps must be 1-10",
            ),
            (
                self.detector_min_score > 0.0 && self.detector_min_score <= 1.0,
                "detector_min_score must be above 0 and at most 1",
            ),
            (
                (1..=MAX_FOLLOW_FPS).contains(&self.follow_fps),
                "follow_fps must be 1-30",
            ),
            (
                self.follow_min_score > 0.0 && self.follow_min_score <= 1.0,
                "follow_min_score must be above 0 and at most 1",
            ),
            (
                (1..=MAX_LOST_FRAMES).contains(&self.follow_lost_frames),
                "follow_lost_frames must be 1-60",
            ),
        ];
        match rules.iter().find(|(holds, _)| !holds) {
            Some((_, rule)) => Err(MediaError::Config((*rule).to_owned())),
            None => Ok(self),
        }
    }

    /// What `active` is given. With region offsets off the pilot's box does
    /// nothing, so it has no lifetime and is never confirmed to the ground; detections still
    /// count, to be outlined.
    fn limits(self, shaping: bool) -> RegionLimits {
        RegionLimits {
            pilot_ttl_ms: if shaping { self.pilot_ttl_ms } else { 0 },
            detection_ttl_ms: self.detection_ttl_ms,
            max_regions: self.max_regions,
            max_area_percent: self.max_area_percent,
        }
    }
}

/// A frame for the detector: RGB, 3 bytes a pixel, already the model's input size, and the cell's
/// time when the frame was taken.
#[derive(Debug)]
pub struct DetectorFrame {
    pub at_ms: u64,
    pub width: u32,
    pub height: u32,
    pub rgb: Vec<u8>,
}

/// A frame for the tracker: the luma plane at `follow::WIDTH` x `follow::HEIGHT`, and the cell's
/// time when the frame was taken.
#[derive(Debug)]
pub struct FollowFrame {
    pub at_ms: u64,
    pub luma: Vec<u8>,
}

/// What the sender needs for regional video, and its counters. Shared with the application's
/// detector thread and its metrics.
#[derive(Debug)]
pub struct Regional {
    pub cell: RegionCell,
    settings: RegionSettings,
    /// Quantiser offset the encoder applies inside the regions (negative); 0 is off.
    roi_qp_offset: f32,
    /// Whether the pilot's box is followed (the sender then feeds the tracker).
    follow: bool,
    detector_tap: Option<Arc<Tap>>,
    follow_tap: Option<Arc<Tap>>,
    active: AtomicU64,
    shape_failures: AtomicU64,
    detector: Detector,
    tracker: Tracker,
}

#[derive(Debug)]
struct Detector {
    state: Mutex<String>,
    runs: AtomicU64,
    found: AtomicU64,
    run: Stage,
}

#[derive(Debug)]
struct Tracker {
    frames: AtomicU64,
    /// The last match's correlation, as `f32` bits.
    score: AtomicU32,
    run: Stage,
}

#[derive(Debug, Serialize)]
pub struct RegionalSnapshot {
    pub region: RegionCounts,
    pub detector: DetectorSnapshot,
    pub follow: FollowSnapshot,
}

#[derive(Debug, Serialize)]
pub struct FollowSnapshot {
    /// `off`, `idle` (nothing followed) or `following`.
    pub state: &'static str,
    pub frames: u64,
    /// Correlation of the last match.
    pub score: f32,
    /// Follows that ended lost, and why the last did (`score`, `edge` or `flat`).
    pub lost: u64,
    pub last_lost: Option<&'static str>,
    /// Where the target is now: x0, y0, x1, y1 in fractions of the picture.
    pub target: Option<[f32; 4]>,
    pub ms: StageSnapshot,
}

#[derive(Debug, Serialize)]
pub struct RegionCounts {
    /// Regions given to the encoder in the last frame.
    pub active: u64,
    /// Frames that could not be shaped or outlined; they went on with their stamp.
    pub shape_failures: u64,
}

#[derive(Debug, Serialize)]
pub struct DetectorSnapshot {
    /// `off`, `running` or `failed: <cause>`.
    pub state: String,
    pub runs: u64,
    pub found: u64,
    pub ms: StageSnapshot,
    /// What is active now: kind and x0, y0, x1, y1 in fractions of the picture.
    pub boxes: Vec<(&'static str, [f32; 4])>,
}

impl Regional {
    /// Regional video with `settings` (already validated) around `cell`, which the control loop
    /// shares. `detector_input` is the model's input size when a detector runs: the sender then
    /// feeds it. `roi_qp_offset` is the encoder's quantiser offset inside the regions (0 is off). With `follow`
    /// the pilot's box starts a follow and the sender feeds the tracker.
    pub fn new(
        settings: RegionSettings,
        cell: RegionCell,
        detector_input: Option<u32>,
        roi_qp_offset: f32,
        follow: bool,
    ) -> Arc<Self> {
        cell.configure(settings.limits(roi_qp_offset < 0.0));
        if follow {
            cell.enable_follow();
        }
        let regional = Self {
            cell,
            settings,
            roi_qp_offset,
            follow,
            detector_tap: detector_input.map(|side| {
                Tap::new(TapSpec {
                    format: TapFormat::Rgb,
                    width: side,
                    height: Some(side),
                    fps: settings.detector_fps,
                })
            }),
            follow_tap: follow.then(|| {
                Tap::new(TapSpec {
                    format: TapFormat::Gray8,
                    width: follow_side(follow::WIDTH),
                    height: Some(follow_side(follow::HEIGHT)),
                    fps: settings.follow_fps,
                })
            }),
            active: AtomicU64::new(0),
            shape_failures: AtomicU64::new(0),
            detector: Detector {
                state: Mutex::new("off".to_owned()),
                runs: AtomicU64::new(0),
                found: AtomicU64::new(0),
                run: Stage::new(),
            },
            tracker: Tracker {
                frames: AtomicU64::new(0),
                score: AtomicU32::new(0),
                run: Stage::new(),
            },
        };
        Arc::new(regional)
    }

    pub fn detector_min_score(&self) -> f32 {
        self.settings.detector_min_score
    }

    /// The tracker's settings from `[region]` of the limits file.
    pub fn follow_settings(&self) -> follow::Settings {
        follow::Settings {
            min_score: self.settings.follow_min_score,
            lost_frames: self.settings.follow_lost_frames,
        }
    }

    /// The taps the sender feeds the detector and the tracker with, when they run.
    pub fn taps(&self) -> Vec<Arc<Tap>> {
        self.detector_tap
            .iter()
            .chain(&self.follow_tap)
            .cloned()
            .collect()
    }

    /// The newest frame the sender offered the detector and it has not taken yet, waiting for one
    /// if there is none: a busy detector skips to the newest frame, so what it finds is never a
    /// run old, and the live video never waits.
    pub fn next_frame(&self) -> DetectorFrame {
        let Some(tap) = &self.detector_tap else {
            wait_forever()
        };
        let frame = tap.next_frame();
        DetectorFrame {
            at_ms: self.taken_ms(frame.at),
            width: frame.width,
            height: frame.height,
            rgb: frame.data,
        }
    }

    /// The tracker's next frame, like `next_frame`. A buffer that is not exactly the picture the
    /// tracker works on (padded rows) is skipped, never read as another shape.
    pub fn next_follow_frame(&self) -> FollowFrame {
        let Some(tap) = &self.follow_tap else {
            wait_forever()
        };
        loop {
            let frame = tap.next_frame();
            if follow::Luma::new(&frame.data).is_some() {
                let at_ms = self.taken_ms(frame.at);
                return FollowFrame {
                    at_ms,
                    luma: frame.data,
                };
            }
        }
    }

    /// The cell's time at which a frame left the tee.
    fn taken_ms(&self, at: Instant) -> u64 {
        let ago = u64::try_from(at.elapsed().as_millis()).unwrap_or(u64::MAX);
        self.cell.now_ms().saturating_sub(ago)
    }

    /// Counts a frame the tracker took, with its last correlation and how long it took.
    pub fn followed_frame(&self, score: f32, took: Duration) {
        self.tracker.frames.fetch_add(1, Ordering::Relaxed);
        self.tracker.score.store(score.to_bits(), Ordering::Relaxed);
        self.tracker.run.record(took);
    }

    /// Counts a frame that could not be shaped.
    fn shape_failed(&self) {
        self.shape_failures.fetch_add(1, Ordering::Relaxed);
    }

    pub fn detector_running(&self) {
        self.set_state("running".to_owned());
    }

    pub fn detector_failed(&self, cause: &dyn std::fmt::Display) {
        self.set_state(format!("failed: {cause}"));
    }

    fn set_state(&self, state: String) {
        *self
            .detector
            .state
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = state;
    }

    /// Publishes what was found in the frame taken at `at_ms`, and how long the run took.
    pub fn detected(&self, found: &[(Kind, NormRect)], at_ms: u64, took: Duration) {
        self.cell.publish_detections(found.iter().copied(), at_ms);
        self.detector.runs.fetch_add(1, Ordering::Relaxed);
        self.detector
            .found
            .fetch_add(found.len() as u64, Ordering::Relaxed);
        self.detector.run.record(took);
    }

    pub fn snapshot(&self) -> RegionalSnapshot {
        RegionalSnapshot {
            region: RegionCounts {
                active: self.active.load(Ordering::Relaxed),
                shape_failures: self.shape_failures.load(Ordering::Relaxed),
            },
            detector: DetectorSnapshot {
                state: self
                    .detector
                    .state
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .clone(),
                runs: self.detector.runs.load(Ordering::Relaxed),
                found: self.detector.found.load(Ordering::Relaxed),
                ms: self.detector.run.snapshot(),
                boxes: self
                    .cell
                    .detected()
                    .into_iter()
                    .map(|(kind, rect)| (kind.name(), corners(rect)))
                    .collect(),
            },
            follow: self.follow_snapshot(),
        }
    }

    fn follow_snapshot(&self) -> FollowSnapshot {
        let target = self.cell.followed();
        let (lost, last_lost) = self.cell.follow_lost_count();
        let state = match (self.follow, self.cell.follow_order()) {
            (false, _) => "off",
            (true, None) => "idle",
            (true, Some(_)) => "following",
        };
        FollowSnapshot {
            state,
            frames: self.tracker.frames.load(Ordering::Relaxed),
            score: f32::from_bits(self.tracker.score.load(Ordering::Relaxed)),
            lost,
            last_lost: last_lost.map(Lost::name),
            target: target.map(corners),
            ms: self.tracker.run.snapshot(),
        }
    }

    /// What is outlined: the detections in their kind's colour, then the followed box in its own.
    fn outlines(&self) -> Vec<([u8; 3], [u8; 7], NormRect)> {
        let detected = self
            .cell
            .detected()
            .into_iter()
            .map(|(kind, rect)| (kind.yuv(), kind.glyph(), rect));
        let followed = self
            .cell
            .followed()
            .map(|rect| (follow::YUV, follow::GLYPH, rect));
        detected.chain(followed).collect()
    }

    /// The sender's hook for regional video: shapes each raw frame before its stamp is drawn. A
    /// frame that cannot be shaped still goes out, with its stamp.
    pub fn frame_hook(self: &Arc<Self>) -> FrameHook {
        let regional = Arc::clone(self);
        Arc::new(move |frame| {
            regional.shape(frame).unwrap_or_else(|_| {
                regional.shape_failed();
                Vec::new()
            })
        })
    }

    /// The encoder's quantiser offset inside the regions, for the sender's hooks.
    pub fn roi_qp_offset(&self) -> f32 {
        self.roi_qp_offset
    }

    /// Outlines what was found and returns the regions the encoder should spend more on. Runs
    /// before the stamp is drawn, so the stamp stays exact.
    pub(crate) fn shape(
        &self,
        frame: &mut gst_video::VideoFrameRef<&mut gst::BufferRef>,
    ) -> Result<Vec<Roi>, MediaError> {
        let regions = if self.roi_qp_offset < 0.0 {
            self.cell.regions()
        } else {
            Vec::new()
        };
        self.active.store(regions.len() as u64, Ordering::Relaxed);
        let width = usize::try_from(frame.width()).unwrap_or(0);
        let height = usize::try_from(frame.height()).unwrap_or(0);
        let rois = regions
            .iter()
            .map(|r| roi(r.pixels(width, height)))
            .collect();
        let outlines = self.outlines();
        if outlines.is_empty() {
            return Ok(rois);
        }
        for (plane, ink) in (0..3u32).zip(TAG_INK) {
            let geometry = plane_geometry(frame, plane)?;
            let index = plane as usize;
            let data = frame.plane_data_mut(plane)?;
            let thickness = outline_thickness(index, width);
            for (yuv, glyph, rect) in &outlines {
                let pixels = rect.pixels(geometry.width, geometry.height);
                let colour = yuv.get(index).copied().unwrap_or(ink);
                outline(data, geometry, pixels, thickness, colour);
                tag(data, geometry, pixels, *glyph, thickness, colour, ink);
            }
        }
        Ok(rois)
    }
}

fn roi(rect: PixelRect) -> Roi {
    let side = |from: usize, to: usize| u32::try_from(to - from).unwrap_or(u32::MAX);
    (
        u32::try_from(rect.x0).unwrap_or(u32::MAX),
        u32::try_from(rect.y0).unwrap_or(u32::MAX),
        side(rect.x0, rect.x1),
        side(rect.y0, rect.y1),
    )
}

fn corners(rect: NormRect) -> [f32; 4] {
    [rect.x0, rect.y0, rect.x1, rect.y1]
}

/// The outline's thickness, which is also the tag's pixel size, in plane `index` of a frame
/// `luma_width` wide: 2 px at 640 wide, scaled with the width, half of it (at least 1) on the half-size chroma planes.
fn outline_thickness(index: usize, luma_width: usize) -> usize {
    let luma = (OUTLINE_PX * luma_width / OUTLINE_REFERENCE_WIDTH).max(1);
    if index == 0 { luma } else { (luma / 2).max(1) }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SETTINGS: RegionSettings = RegionSettings {
        pilot_ttl_ms: 3000,
        detection_ttl_ms: 1000,
        max_regions: 4,
        max_area_percent: 50,
        detector_fps: 2,
        detector_min_score: 0.4,
        follow_fps: 10,
        follow_min_score: 0.5,
        follow_lost_frames: 5,
    };

    #[test]
    fn config_refuses_an_offset_outside_minus_51_to_0() {
        for offset in [0.0, -6.0, -51.0] {
            assert_eq!(validate_roi_qp_offset(offset).unwrap(), offset);
        }
        for offset in [1.0, -52.0, f32::NAN] {
            assert!(validate_roi_qp_offset(offset).is_err(), "offset {offset}");
        }
    }

    #[test]
    fn with_offsets_off_detections_count_and_the_pilot_box_does_not() {
        let cell = RegionCell::default();
        let _regional = Regional::new(SETTINGS, cell.clone(), Some(320), 0.0, false);
        let pilot = NormRect::new(0.1, 0.1, 0.3, 0.3).unwrap();
        let person = (Kind::Person, NormRect::new(0.5, 0.5, 0.6, 0.8).unwrap());
        cell.set(sat_teleop_policy::region::Timed {
            rect: pilot,
            at_ms: cell.now_ms(),
        });
        cell.publish_detections([person], cell.now_ms());
        assert_eq!(cell.pilot_in_use(), None);
        assert_eq!(cell.detected(), vec![person]);
    }

    #[test]
    fn the_hook_returns_the_pilot_box_in_pixels_when_offsets_are_on() {
        sat_teleop_video::init().unwrap();
        let cell = RegionCell::default();
        let regional = Regional::new(SETTINGS, cell.clone(), None, -6.0, false);
        // The bottom left quarter of the picture.
        let rect = NormRect::new(0.0, 0.5, 0.5, 1.0).unwrap();
        cell.set(sat_teleop_policy::region::Timed {
            rect,
            at_ms: cell.now_ms(),
        });
        let video = gst_video::VideoInfo::builder(gst_video::VideoFormat::I420, 640, 360)
            .build()
            .unwrap();
        let mut buffer = gst::Buffer::with_size(video.size()).unwrap();
        let mut frame =
            gst_video::VideoFrameRef::from_buffer_ref_writable(buffer.get_mut().unwrap(), &video)
                .unwrap();

        assert_eq!(regional.frame_hook()(&mut frame), vec![(0, 180, 320, 180)]);
        assert!(cell.pilot_in_use().is_some(), "the box is kept");
    }
}
