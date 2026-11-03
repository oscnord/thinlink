//! Regional video on the vehicle: the `[region]` limits, what the sender needs to soften the
//! picture outside the regions and outline the people and vehicles found, and the detector's counters. The
//! detector itself (`sat-teleop-detect`) is in a sibling crate, so the application runs it: the
//! sender offers it the newest scaled frame (`DetectorFrame`) and it publishes what it finds to the cell.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, PoisonError};
use std::time::{Duration, Instant};

use gstreamer as gst;
use gstreamer_video as gst_video;
use gstreamer_video::prelude::*;
use serde::{Deserialize, Serialize};

use sat_teleop_policy::kind::Kind;
use sat_teleop_policy::region::{NormRect, RegionCell, RegionLimits};
use sat_teleop_policy::soften::{outline, soften_i420_plane, tag};

use crate::stages::{Stage, StageSnapshot};
use crate::{MediaError, plane_geometry};

const MAX_SOFTEN_FACTOR: u8 = 4;
const MAX_REGIONS: usize = 16;
const MAX_DETECTOR_FPS: u16 = 10;
const MAX_TTL_MS: u64 = 60_000;
/// The letter of a tag in Y, U and V: black.
const TAG_INK: [u8; 3] = [16, 128, 128];
/// An outline is this many pixels at a luma width of `OUTLINE_REFERENCE_WIDTH`, and scales with it.
const OUTLINE_PX: usize = 2;
const OUTLINE_REFERENCE_WIDTH: usize = 640;

/// The block size of the softened area from `[region]` of the vehicle config: 1 (the default) is
/// off, 2 to 4 soften.
pub fn validate_soften_factor(factor: u8) -> Result<u8, MediaError> {
    if (1..=MAX_SOFTEN_FACTOR).contains(&factor) {
        return Ok(factor);
    }
    Err(MediaError::Config("soften_factor must be 1-4".to_owned()))
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
        ];
        match rules.iter().find(|(holds, _)| !holds) {
            Some((_, rule)) => Err(MediaError::Config((*rule).to_owned())),
            None => Ok(self),
        }
    }

    /// What `active` is given. With softening off the pilot's box does nothing, so it has no
    /// lifetime and is never confirmed to the ground; detections still count, to be outlined.
    fn limits(self, soften_factor: u8) -> RegionLimits {
        RegionLimits {
            pilot_ttl_ms: if soften_factor > 1 {
                self.pilot_ttl_ms
            } else {
                0
            },
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

/// What the sender needs for regional video, and its counters. Shared with the application's
/// detector thread and its metrics.
#[derive(Debug)]
pub struct Regional {
    pub cell: RegionCell,
    settings: RegionSettings,
    /// Block size of the softened area; 1 is off.
    soften_factor: u8,
    /// The model's input size, when a detector is configured: the sender feeds it.
    detector_input: Option<u32>,
    /// The newest frame the sender offered that the detector has not taken.
    latest: Mutex<Option<DetectorFrame>>,
    offered: Condvar,
    active: AtomicU64,
    softened_frames: AtomicU64,
    shape_failures: AtomicU64,
    soften: Stage,
    detector: Detector,
}

#[derive(Debug)]
struct Detector {
    state: Mutex<String>,
    runs: AtomicU64,
    found: AtomicU64,
    run: Stage,
}

#[derive(Debug, Serialize)]
pub struct RegionalSnapshot {
    pub region: RegionCounts,
    pub detector: DetectorSnapshot,
}

#[derive(Debug, Serialize)]
pub struct RegionCounts {
    /// Regions kept sharp in the last frame.
    pub active: u64,
    pub softened_frames: u64,
    /// Frames that could not be softened or outlined; they went on with their stamp.
    pub shape_failures: u64,
    pub soften: StageSnapshot,
}

#[derive(Debug, Serialize)]
pub struct DetectorSnapshot {
    /// `off`, `running` or `failed: <cause>`.
    pub state: String,
    pub runs: u64,
    pub found: u64,
    pub ms: StageSnapshot,
}

impl Regional {
    /// Regional video with `settings` (already validated) around `cell`, which the control loop
    /// shares. `detector_input` is the model's input size when a detector runs: the sender then
    /// feeds it. `soften_factor` is the block size of the softened area (1 is off).
    pub fn new(
        settings: RegionSettings,
        cell: RegionCell,
        detector_input: Option<u32>,
        soften_factor: u8,
    ) -> Arc<Self> {
        cell.configure(settings.limits(soften_factor));
        let regional = Self {
            cell,
            settings,
            soften_factor,
            detector_input,
            latest: Mutex::new(None),
            offered: Condvar::new(),
            active: AtomicU64::new(0),
            softened_frames: AtomicU64::new(0),
            shape_failures: AtomicU64::new(0),
            soften: Stage::new(),
            detector: Detector {
                state: Mutex::new("off".to_owned()),
                runs: AtomicU64::new(0),
                found: AtomicU64::new(0),
                run: Stage::new(),
            },
        };
        Arc::new(regional)
    }

    pub(crate) fn detector_input(&self) -> Option<u32> {
        self.detector_input
    }

    pub(crate) fn detector_fps(&self) -> u16 {
        self.settings.detector_fps
    }

    pub fn detector_min_score(&self) -> f32 {
        self.settings.detector_min_score
    }

    /// Hands a frame to the detector, replacing one it has not taken yet: a busy detector skips
    /// to the newest frame, so what it finds is never a run old, and the live video never waits.
    pub(crate) fn offer(&self, frame: DetectorFrame) {
        *self.latest.lock().unwrap_or_else(PoisonError::into_inner) = Some(frame);
        self.offered.notify_one();
    }

    /// The newest frame offered and not taken yet, waiting for one if there is none.
    pub fn next_frame(&self) -> DetectorFrame {
        let mut latest = self.latest.lock().unwrap_or_else(PoisonError::into_inner);
        loop {
            if let Some(frame) = latest.take() {
                return frame;
            }
            latest = self
                .offered
                .wait(latest)
                .unwrap_or_else(PoisonError::into_inner);
        }
    }

    /// Counts a frame that could not be shaped.
    pub(crate) fn shape_failed(&self) {
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
                softened_frames: self.softened_frames.load(Ordering::Relaxed),
                shape_failures: self.shape_failures.load(Ordering::Relaxed),
                soften: self.soften.snapshot(),
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
            },
        }
    }

    /// Softens the picture outside the regions and outlines what was found. Runs before the
    /// stamp is drawn, so the stamp stays exact.
    pub(crate) fn shape(
        &self,
        frame: &mut gst_video::VideoFrameRef<&mut gst::BufferRef>,
    ) -> Result<(), MediaError> {
        let started = Instant::now();
        let regions = self.cell.regions();
        let boxes = self.cell.detected();
        let softening = self.soften_factor > 1 && !regions.is_empty();
        self.active.store(
            if softening { regions.len() as u64 } else { 0 },
            Ordering::Relaxed,
        );
        if !softening && boxes.is_empty() {
            return Ok(());
        }
        let factor = if softening {
            usize::from(self.soften_factor)
        } else {
            1
        };
        let width = usize::try_from(frame.width()).unwrap_or(0);
        for (plane, ink) in (0..3u32).zip(TAG_INK) {
            let geometry = plane_geometry(frame, plane)?;
            let index = plane as usize;
            let data = frame.plane_data_mut(plane)?;
            soften_i420_plane(index, data, geometry, &regions, factor);
            let thickness = outline_thickness(index, width);
            for (kind, rect) in &boxes {
                let pixels = rect.pixels(geometry.width, geometry.height);
                let colour = kind.yuv().get(index).copied().unwrap_or(ink);
                outline(data, geometry, pixels, thickness, colour);
                tag(data, geometry, pixels, kind.glyph(), thickness, colour, ink);
            }
        }
        if softening {
            self.softened_frames.fetch_add(1, Ordering::Relaxed);
            self.soften.record(started.elapsed());
        }
        Ok(())
    }
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
    };

    #[test]
    fn config_refuses_a_soften_factor_over_4() {
        for factor in [1, 2, 4] {
            assert_eq!(validate_soften_factor(factor).unwrap(), factor);
        }
        for factor in [0, 5] {
            assert!(validate_soften_factor(factor).is_err(), "factor {factor}");
        }
    }

    #[test]
    fn with_softening_off_detections_count_and_the_pilot_box_does_not() {
        let cell = RegionCell::default();
        let _regional = Regional::new(SETTINGS, cell.clone(), Some(320), 1);
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
    fn a_busy_detector_gets_the_newest_frame() {
        let regional = Regional::new(SETTINGS, RegionCell::default(), Some(320), 2);
        for at_ms in [1, 2, 3] {
            regional.offer(DetectorFrame {
                at_ms,
                width: 1,
                height: 1,
                rgb: Vec::new(),
            });
        }
        assert_eq!(regional.next_frame().at_ms, 3);
    }
}
