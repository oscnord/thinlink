//! A small detector of people and vehicles: NanoDet-Plus-m 320 run through `tract-onnx`. The caller hands it a
//! frame already scaled to the model's input size; it never sees the video pipeline.

mod decode;
mod tag;

use std::path::{Path, PathBuf};
use std::sync::Arc;

use tract_onnx::prelude::tract_ndarray::Array4;
use tract_onnx::prelude::{
    DatumExt, Framework, InferenceModelExt, IntoRunnable, Tensor, TypedRunnableModel, tvec,
};

use thinlink_policy::kind::Kind;
use thinlink_policy::region::NormRect;

pub use self::decode::non_max_suppression;
pub use self::tag::{TagDetector, TagFound};

/// Boxes overlapping a better one by more than this are the same object.
const NMS_IOU: f32 = 0.5;
/// What the model was trained to see: BGR, per channel `(value - mean) / std`.
const NORMALISE: [(f32, f32); 3] = [(103.53, 57.375), (116.28, 57.12), (123.675, 58.395)];

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    pub rect: NormRect,
    pub score: f32,
    pub kind: Kind,
}

#[derive(Debug, thiserror::Error)]
pub enum DetectError {
    #[error("cannot load the detector model {path}: {cause}")]
    Load { path: PathBuf, cause: String },
    #[error("cannot start the tag detector: {0}")]
    Tag(String),
}

pub struct Detector {
    model: Arc<TypedRunnableModel>,
    input: u32,
    kinds: Vec<Kind>,
}

impl Detector {
    /// Loads and optimises the model for a square RGB input of `input` pixels a side, to find
    /// `kinds`.
    pub fn load(model: &Path, input: u32, kinds: Vec<Kind>) -> Result<Self, DetectError> {
        let fail = |cause: &dyn std::fmt::Display| DetectError::Load {
            path: model.to_path_buf(),
            cause: cause.to_string(),
        };
        let side = usize::try_from(input).map_err(|e| fail(&e))?;
        let model = tract_onnx::onnx()
            .model_for_path(model)
            .and_then(|m| m.with_input_fact(0, f32::fact([1, 3, side, side]).into()))
            .and_then(InferenceModelExt::into_optimized)
            .and_then(IntoRunnable::into_runnable)
            .map_err(|e| fail(&e))?;
        Ok(Self {
            model,
            input,
            kinds,
        })
    }

    /// The configured kinds in `rgb`, a `width` x `height` frame of 3 bytes a pixel that is already
    /// the model's input size, scoring at least `min_score`. A frame of another size or a failed
    /// run finds nothing.
    pub fn detect(
        &mut self,
        rgb: &[u8],
        width: u32,
        height: u32,
        min_score: f32,
    ) -> Vec<Detection> {
        if width != self.input || height != self.input {
            return Vec::new();
        }
        let Some(input) = tensor(rgb, self.input) else {
            return Vec::new();
        };
        let Ok(outputs) = self.model.run(tvec!(input.into())) else {
            return Vec::new();
        };
        let found = outputs
            .first()
            .and_then(|out| {
                let view = out.to_plain_array_view::<f32>().ok()?;
                Some(decode::decode(
                    view.as_slice()?,
                    self.input,
                    min_score,
                    &self.kinds,
                ))
            })
            .unwrap_or_default();
        non_max_suppression(found, NMS_IOU)
    }
}

/// The model's input from `rgb`; `None` when it is not `side` x `side` pixels.
fn tensor(rgb: &[u8], side: u32) -> Option<Tensor> {
    let side = usize::try_from(side).ok()?;
    if rgb.len() != side.checked_mul(side)?.checked_mul(3)? {
        return None;
    }
    let tensor = Array4::from_shape_fn((1, 3, side, side), |(_, channel, y, x)| {
        let blue_first = 2 - channel;
        let value = rgb
            .get((y * side + x) * 3 + blue_first)
            .copied()
            .unwrap_or(0);
        let (mean, std) = NORMALISE.get(channel).copied().unwrap_or((0.0, 1.0));
        (f32::from(value) - mean) / std
    });
    Some(tensor.into())
}
