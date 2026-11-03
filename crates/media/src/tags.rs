//! Frames for the landing marker's detector: the newest greyscale frame the sender's tag branch
//! offers. The application's tag thread takes them; a thread that is busy skips to the newest, so
//! the live video never waits and a marker is never found in an old frame.

use std::sync::{Arc, Condvar, Mutex, PoisonError};
use std::time::Instant;

use crate::MediaError;

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
    fps: u16,
    width: u32,
    latest: Mutex<Option<TagFrame>>,
    offered: Condvar,
}

impl TagFeed {
    /// A feed of about `fps` frames a second, scaled to `width` pixels across (the height keeps the
    /// picture's shape).
    pub fn new(fps: u16, width: u32) -> Result<Arc<Self>, MediaError> {
        validate(fps, width)?;
        Ok(Arc::new(Self {
            fps,
            width,
            latest: Mutex::new(None),
            offered: Condvar::new(),
        }))
    }

    pub(crate) fn fps(&self) -> u16 {
        self.fps
    }

    pub(crate) fn width(&self) -> u32 {
        self.width
    }

    /// Replaces a frame the tag thread has not taken.
    pub(crate) fn offer(&self, frame: TagFrame) {
        *self.latest.lock().unwrap_or_else(PoisonError::into_inner) = Some(frame);
        self.offered.notify_one();
    }

    /// The newest frame offered and not taken yet, waiting for one if there is none.
    pub fn next_frame(&self) -> TagFrame {
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
}
