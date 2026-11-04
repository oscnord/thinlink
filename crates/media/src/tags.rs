//! Frames for the landing marker's detector: the newest greyscale frame the sender's tag tap
//! offers. The application's tag thread takes them; a thread that is busy skips to the newest, so
//! the live video never waits and a marker is never found in an old frame.

use std::sync::Arc;
use std::time::Instant;

use thinlink_video::MediaError;
use thinlink_video::sender::{Tap, TapFormat, TapFrame, TapSpec};

const WIDTHS: std::ops::RangeInclusive<u32> = 160..=1920;
const MAX_FPS: u16 = 15;

/// A greyscale frame, 1 byte a pixel and no padding between rows, and when it was taken.
#[derive(Debug)]
pub struct TagFrame {
    pub at: Instant,
    pub width: u32,
    pub height: u32,
    pub luma: Vec<u8>,
}

/// Whether a feed of `fps` frames a second and `width` pixels across can be made.
pub fn validate(fps: u16, width: u32) -> Result<(), MediaError> {
    if !(1..=MAX_FPS).contains(&fps) {
        return Err(MediaError::Config(format!(
            "tag_fps must be 1-{MAX_FPS}, not {fps}"
        )));
    }
    if !WIDTHS.contains(&width) {
        return Err(MediaError::Config(format!(
            "tag_width must be {}-{} pixels, not {width}",
            WIDTHS.start(),
            WIDTHS.end()
        )));
    }
    Ok(())
}

#[derive(Debug)]
pub struct TagFeed {
    tap: Arc<Tap>,
}

impl TagFeed {
    /// A feed of about `fps` frames a second, scaled to `width` pixels across (the height keeps the
    /// picture's shape).
    pub fn new(fps: u16, width: u32) -> Result<Arc<Self>, MediaError> {
        validate(fps, width)?;
        let tap = Tap::new(TapSpec {
            format: TapFormat::Gray8,
            width,
            height: None,
            fps,
        });
        Ok(Arc::new(Self { tap }))
    }

    /// The tap the sender feeds.
    pub fn tap(&self) -> Arc<Tap> {
        Arc::clone(&self.tap)
    }

    /// The newest frame offered and not taken yet, waiting for one if there is none. A frame
    /// whose rows cannot be read without their padding is skipped.
    pub fn next_frame(&self) -> TagFrame {
        loop {
            let frame = self.tap.next_frame();
            if let Some(frame) = unpadded(&frame) {
                return frame;
            }
        }
    }
}

/// The greyscale rows of `frame` without the padding `GStreamer` may add to each.
fn unpadded(frame: &TapFrame) -> Option<TagFrame> {
    let (width, height) = (
        usize::try_from(frame.width).ok()?,
        usize::try_from(frame.height).ok()?,
    );
    let mut luma = Vec::with_capacity(width.checked_mul(height)?);
    for row in 0..height {
        let start = row.checked_mul(frame.stride)?;
        luma.extend_from_slice(frame.data.get(start..start.checked_add(width)?)?);
    }
    Some(TagFrame {
        at: frame.at,
        width: frame.width,
        height: frame.height,
        luma,
    })
}
