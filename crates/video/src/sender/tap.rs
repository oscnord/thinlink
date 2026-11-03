//! Leaky taps off the sender's tee: the caller asks for frames of a size, format and rate and
//! reads the newest from a latest-only slot on a thread of its own. The tap sits behind a leaky
//! queue of one buffer and an appsink that drops what it cannot take, so a slow reader skips to the
//! newest frame and never holds the live video.

use std::sync::{Arc, Condvar, Mutex, PoisonError};
use std::time::Instant;

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_video as gst_video;

use super::copy_frames;
use crate::MediaError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TapFormat {
    /// 3 bytes a pixel.
    Rgb,
    /// 1 byte a pixel.
    Gray8,
}

impl TapFormat {
    fn name(self) -> &'static str {
        match self {
            Self::Rgb => "RGB",
            Self::Gray8 => "GRAY8",
        }
    }
}

/// What a tap delivers: about `fps` frames a second, scaled to `width` x `height` (the height keeps
/// the picture's shape without one).
#[derive(Debug, Clone, Copy)]
pub struct TapSpec {
    pub format: TapFormat,
    pub width: u32,
    pub height: Option<u32>,
    pub fps: u16,
}

/// A frame as the tap received it: `stride` bytes a row, so rows may be padded.
#[derive(Debug)]
pub struct TapFrame {
    /// When the frame left the tee.
    pub at: Instant,
    pub width: u32,
    pub height: u32,
    pub stride: usize,
    pub data: Vec<u8>,
}

#[derive(Debug)]
pub struct Tap {
    spec: TapSpec,
    latest: Mutex<Option<TapFrame>>,
    offered: Condvar,
}

impl Tap {
    pub fn new(spec: TapSpec) -> Arc<Self> {
        Arc::new(Self {
            spec,
            latest: Mutex::new(None),
            offered: Condvar::new(),
        })
    }

    /// The newest frame offered and not taken yet, waiting for one if there is none.
    pub fn next_frame(&self) -> TapFrame {
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

    /// Replaces a frame the reader has not taken.
    fn offer(&self, frame: TapFrame) {
        *self.latest.lock().unwrap_or_else(PoisonError::into_inner) = Some(frame);
        self.offered.notify_one();
    }
}

/// The tap's elements after the tee: frames thinned to the tap's rate and copied as they enter a
/// leaky queue of one buffer (`copy_frames`), scaled and converted, and handed over by an appsink
/// that drops what it cannot take.
pub(super) fn branch(tap: &Arc<Tap>) -> Result<Vec<gst::Element>, MediaError> {
    let spec = tap.spec;
    let queue = gst::ElementFactory::make("queue")
        .property_from_str("leaky", "downstream")
        .property("max-size-buffers", 1u32)
        .property("max-size-bytes", 0u32)
        .property("max-size-time", 0u64)
        .build()?;
    copy_frames(&queue, spec.fps)?;
    let mut caps = gst::Caps::builder("video/x-raw")
        .field("format", spec.format.name())
        .field("width", side(spec.width)?);
    if let Some(height) = spec.height {
        caps = caps.field("height", side(height)?);
    }
    let shape = gst::ElementFactory::make("capsfilter")
        .property("caps", caps.build())
        .build()?;
    let sink = gst_app::AppSink::builder()
        .max_buffers(1)
        .drop(true)
        .sync(false)
        .build();
    let tap = Arc::clone(tap);
    sink.set_callbacks(
        gst_app::AppSinkCallbacks::builder()
            .new_sample(move |sink| {
                let sample = sink.pull_sample().map_err(|_| gst::FlowError::Eos)?;
                if let Some(frame) = frame(&sample) {
                    tap.offer(frame);
                }
                Ok(gst::FlowSuccess::Ok)
            })
            .build(),
    );
    Ok(vec![
        queue,
        gst::ElementFactory::make("videoscale").build()?,
        gst::ElementFactory::make("videoconvert").build()?,
        shape,
        sink.upcast(),
    ])
}

fn side(pixels: u32) -> Result<i32, MediaError> {
    i32::try_from(pixels).map_err(|e| MediaError::Config(e.to_string()))
}

fn frame(sample: &gst::Sample) -> Option<TapFrame> {
    let at = Instant::now();
    let info = gst_video::VideoInfo::from_caps(sample.caps()?).ok()?;
    let stride = usize::try_from(*info.stride().first()?).ok()?;
    let data = sample.buffer()?.map_readable().ok()?.as_slice().to_vec();
    Some(TapFrame {
        at,
        width: info.width(),
        height: info.height(),
        stride,
        data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_busy_reader_gets_the_newest_frame() {
        let tap = Tap::new(TapSpec {
            format: TapFormat::Gray8,
            width: 2,
            height: None,
            fps: 1,
        });
        for width in [1, 2, 3] {
            tap.offer(TapFrame {
                at: Instant::now(),
                width,
                height: 1,
                stride: 1,
                data: Vec::new(),
            });
        }
        assert_eq!(tap.next_frame().width, 3);
    }
}
