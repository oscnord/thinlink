//! A `GStreamer` H.264 encoder element on libx264, `thinlinkx264enc`, for the sender.
//!
//! `x264enc` cannot give a whole IDR frame on demand while intra refresh is on, cannot change its
//! refresh period while playing and has no per-macroblock quantiser offsets. This element drives
//! libx264 directly: the refresh period follows the caps' frame rate, a forced key unit (upstream
//! or downstream) makes the next picture a whole IDR frame, and the `GstVideoRegionOfInterestMeta`
//! on a frame's buffer lowers the quantiser inside its regions (`roi-qp-offset`).

mod element;
mod encoder;
mod roi;

#[allow(
    unsafe_code,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    clippy::all,
    clippy::pedantic,
    clippy::indexing_slicing,
    reason = "bindgen output for x264.h"
)]
mod sys {
    include!(concat!(env!("OUT_DIR"), "/x264.rs"));
}

pub use element::{ELEMENT, register};
