//! The landing target switch: its config (`[landing]` and `[camera]` of the vehicle config), what it
//! asks of real mode, and the thread that looks for the marker in the tag feed's frames. Everything
//! the thread finds goes to `VehicleStats::landing`, the one path to the autopilot.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, PoisonError};
use std::time::Instant;

use anyhow::{Result, bail};
use serde::{Deserialize, Serialize};

use thinlink_control::identity::Mode;
use thinlink_control::params;
use thinlink_control::vehicle::{VehicleConfig, VehicleStats};
use thinlink_detect::TagDetector;
use thinlink_media::tags::{self, TagFeed};
use thinlink_policy::landing::{Camera, Setup, Sighting};
use thinlink_video::stages::{Stage, StageSnapshot};

/// A fixed camera for the landing target looks straight down: the angles are not corrected for a tilt.
const FIXED_TILT_DEG: f32 = -90.0;

/// `[camera]` of the vehicle config: what the landing target needs to know of the picture.
#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct CameraConfig {
    /// The picture's horizontal field of view in degrees.
    hfov_deg: Option<f32>,
    /// A fixed camera's pitch from the horizon, which the landing target needs to be exactly -90
    /// (straight down). Without it the camera is on the first mount, whose attitude the autopilot
    /// reports.
    tilt_deg: Option<f32>,
}

/// `[landing]` of the vehicle config.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LandingConfig {
    /// Off by default: with it on the vehicle may tell the autopilot where the marker is while the
    /// camera points down.
    #[serde(default)]
    target: bool,
    /// The tag36h11 tag to land on.
    #[serde(default)]
    tag_id: u16,
    /// How many frames a second are searched for the tag.
    #[serde(default = "default_tag_fps")]
    tag_fps: u16,
    /// The width those frames are scaled to.
    #[serde(default = "default_tag_width")]
    tag_width: u32,
}

impl Default for LandingConfig {
    fn default() -> Self {
        Self {
            target: false,
            tag_id: 0,
            tag_fps: default_tag_fps(),
            tag_width: default_tag_width(),
        }
    }
}

fn default_tag_fps() -> u16 {
    10
}

fn default_tag_width() -> u32 {
    640
}

/// The switch, on, and everything it needs.
pub struct On {
    pub setup: Setup,
    tag_id: u16,
    pub fps: u16,
    pub width: u32,
}

impl On {
    pub fn tag_id(&self) -> u16 {
        self.tag_id
    }
}

/// The landing target of a config that has it on, or `None`. It refuses a config that cannot be
/// trusted to point the autopilot: no field of view, a fixed camera that is not at -90 degrees, tag frames the
/// feed cannot make, and, in real mode, a `require` without the precision landing parameters.
pub fn resolve(
    camera: &CameraConfig,
    landing: &LandingConfig,
    mavlink: &VehicleConfig,
) -> Result<Option<On>> {
    if !landing.target {
        return Ok(None);
    }
    let Some(hfov_deg) = camera
        .hfov_deg
        .filter(|f| f.is_finite() && (1.0..180.0).contains(f))
    else {
        bail!("[landing] target needs [camera] hfov_deg, from 1 to 180 degrees");
    };
    let camera = match camera.tilt_deg {
        None => Camera::Gimbal,
        Some(tilt_deg) if tilt_deg == FIXED_TILT_DEG => Camera::Fixed { tilt_deg },
        Some(tilt_deg) => bail!(
            "[camera] tilt_deg must be exactly {FIXED_TILT_DEG} for the landing target, not {tilt_deg}: \
             a tilt is not corrected"
        ),
    };
    tags::validate(landing.tag_fps, landing.tag_width)?;
    if mavlink.mode == Mode::Real {
        if camera == Camera::Gimbal {
            bail!(
                "[landing] target in real mode needs a fixed camera ([camera] tilt_deg = {FIXED_TILT_DEG}): \
                 a gimbal's frame is verified only for the simulation's servo mount, which follows the airframe"
            );
        }
        params::validate_landing(&mavlink.require).map_err(anyhow::Error::msg)?;
    }
    Ok(Some(On {
        setup: Setup { camera, hfov_deg },
        tag_id: landing.tag_id,
        fps: landing.tag_fps,
        width: landing.tag_width,
    }))
}

/// The tag thread's counters.
#[derive(Default)]
pub struct TagStats {
    state: Mutex<String>,
    runs: AtomicU64,
    found: AtomicU64,
    run: Stage,
}

#[derive(Serialize)]
pub struct TagSnapshot {
    /// `running` or `failed: <cause>`.
    pub state: String,
    pub runs: u64,
    pub found: u64,
    pub ms: StageSnapshot,
}

impl TagStats {
    pub fn snapshot(&self) -> TagSnapshot {
        TagSnapshot {
            state: self
                .state
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone(),
            runs: self.runs.load(Ordering::Relaxed),
            found: self.found.load(Ordering::Relaxed),
            ms: self.run.snapshot(),
        }
    }

    fn set_state(&self, state: String) {
        *self.state.lock().unwrap_or_else(PoisonError::into_inner) = state;
    }
}

/// Looks for the tag in the newest frame the feed offers, over and over, and hands each find to the
/// bridge. A busy thread skips to the newest frame, so a find is never from an old one.
pub fn find_tags(feed: &TagFeed, id: u16, bridge: &VehicleStats, stats: &TagStats) {
    let mut detector = match TagDetector::new(id) {
        Ok(detector) => detector,
        Err(cause) => {
            eprintln!("running without the landing tag detector: {cause}");
            stats.set_state(format!("failed: {cause}"));
            return;
        }
    };
    stats.set_state("running".to_owned());
    loop {
        let frame = feed.next_frame();
        let started = Instant::now();
        let found = detector.find(&frame.luma, frame.width, frame.height);
        stats.run.record(started.elapsed());
        stats.runs.fetch_add(1, Ordering::Relaxed);
        let Some(found) = found else {
            continue;
        };
        stats.found.fetch_add(1, Ordering::Relaxed);
        bridge.landing.see(Sighting {
            at: frame.at,
            x: found.x,
            y: found.y,
            aspect: aspect(frame.width, frame.height),
        });
    }
}

#[expect(
    clippy::cast_precision_loss,
    reason = "frame sizes are a few thousand pixels"
)]
fn aspect(width: u32, height: u32) -> f32 {
    width as f32 / height.max(1) as f32
}
