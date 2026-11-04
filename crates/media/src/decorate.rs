//! What the ground draws into the output's frames besides the state: the predictive display (aim
//! ring and horizon, `thinlink_policy::predict`) and the pilot's detail inset (`inset`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

use gstreamer_video::prelude::*;
use serde::Deserialize;

use thinlink_policy::inset as inset_policy;
use thinlink_policy::mark as draw_mark;
use thinlink_policy::predict::{self, Inputs, Mark};
use thinlink_video_core::stamp::StateStamp;

use crate::inset::DetailCell;
use thinlink_video::output::Note;
use thinlink_video::sender::Frame;
use thinlink_video::{MediaError, luma_geometry, plane_geometry};

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

/// Frames sent with the detail inset drawn in them.
#[derive(Debug, Default)]
pub struct InsetCount(AtomicU64);

impl InsetCount {
    pub fn get(&self) -> u64 {
        self.0.load(Ordering::Relaxed)
    }
}

/// The output's per-frame drawing: `mark` under the stamps, then the inset (if the picture is
/// current and the crop young: `inset::place`) with its age as the note for the inset's label.
/// Never draw the inset into another state, over the stamp rows or without its label.
pub fn draw(
    predictive: Option<&Predictive>,
    detail: &DetailCell,
    drawn: &InsetCount,
    frame: &mut Frame,
    state: StateStamp,
    decided_at: Instant,
) -> Result<Option<u64>, MediaError> {
    let picture = (frame.width() as usize, frame.height() as usize);
    let mark = predictive.and_then(|predictive| {
        let inputs = (predictive.inputs)()?;
        predict::mark(
            state.state,
            state.age_ms().map(u64::from),
            &inputs,
            predictive.camera.hfov_deg,
            predictive.camera.tilt_deg,
            frame.width(),
            frame.height(),
        )
    });
    if let Some(Mark { aim, horizon }) = mark {
        let geometry = luma_geometry(frame)?;
        draw_mark::draw(frame.plane_data_mut(0)?, geometry, aim, horizon)?;
    }
    let Some((image, age_ms)) = detail.current(decided_at) else {
        return Ok(None);
    };
    let Some(placement) = inset_policy::place(state.state, age_ms, image.size(), picture) else {
        return Ok(None);
    };
    for (index, component) in (0..3_u32).enumerate() {
        let geometry = plane_geometry(frame, component)?;
        image.draw(index, frame.plane_data_mut(component)?, geometry, placement);
    }
    drawn.0.fetch_add(1, Ordering::Relaxed);
    Ok(Some(age_ms))
}

/// The inset's label, top right.
pub const NOTE: Note = Note {
    margin_px: inset_policy::MARGIN_PX,
    top_px: inset_policy::label_y,
    text: inset_policy::label,
};
