//! What the lab compares video with: its still test card scaled as the viewer should see it, and a
//! receiver that hands back the luma plane of each decoded frame. `GStreamer` stays in this crate;
//! the lab works on plain bytes.

use std::sync::atomic::AtomicBool;

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_video as gst_video;

use crate::receiver::{self, Observation, Source};
use crate::{MediaError, Running};

/// How long the reference takes to render; it is one frame.
const RENDER_TIMEOUT: gst::ClockTime = gst::ClockTime::from_seconds(10);

/// A luma plane without row padding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Luma {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<u8>,
}

/// `i420`, one frame of `source` size, scaled to `size` the way the sender and the receiver scale
/// it, so it differs from what arrives only by what the link and the encoder did.
pub fn reference(i420: &[u8], source: (u16, u16), size: (u16, u16)) -> Result<Luma, MediaError> {
    crate::init()?;
    let caps = |(width, height): (u16, u16)| {
        gst_video::VideoCapsBuilder::new()
            .format(gst_video::VideoFormat::I420)
            .width(i32::from(width))
            .height(i32::from(height))
            .framerate(gst::Fraction::new(1, 1))
            .build()
    };
    if i420.len() != usize::from(source.0) * usize::from(source.1) * 3 / 2 {
        return Err(MediaError::Config(
            "the card is not one I420 frame".to_owned(),
        ));
    }
    let src = gst_app::AppSrc::builder()
        .caps(&caps(source))
        .format(gst::Format::Time)
        .build();
    let sink = gst_app::AppSink::builder()
        .max_buffers(1)
        .sync(false)
        .build();
    let elements = [
        src.clone().upcast(),
        gst::ElementFactory::make("videoscale").build()?,
        gst::ElementFactory::make("capsfilter")
            .property("caps", caps(size))
            .build()?,
        sink.clone().upcast(),
    ];
    let pipeline = gst::Pipeline::new();
    pipeline.add_many(&elements)?;
    gst::Element::link_many(&elements)?;
    let running = Running(pipeline);
    running.set(gst::State::Playing)?;
    src.push_buffer(gst::Buffer::from_slice(i420.to_vec()))
        .map_err(|e| MediaError::Config(e.to_string()))?;
    let sample = sink
        .try_pull_sample(RENDER_TIMEOUT)
        .ok_or(MediaError::Missing("test card frame"))?;
    luma_of(&sample).ok_or(MediaError::Missing("test card luma plane"))
}

/// Decodes `source` at `size` like [`receiver::decode`] and hands `each` the luma plane of every
/// frame, until `stop` is set, the pipeline fails or `each` returns false.
pub fn receive_luma(
    source: &Source,
    size: (u16, u16),
    stop: &AtomicBool,
    mut each: impl FnMut(Observation, Luma) -> bool,
) -> Result<(), MediaError> {
    receiver::decode(source, size, stop, |observation, sample| {
        luma_of(&sample).is_none_or(|luma| each(observation, luma))
    })
}

fn luma_of(sample: &gst::Sample) -> Option<Luma> {
    let info = gst_video::VideoInfo::from_caps(sample.caps()?).ok()?;
    let frame = gst_video::VideoFrameRef::from_buffer_ref_readable(sample.buffer()?, &info).ok()?;
    let (width, height) = (
        usize::try_from(info.width()).ok()?,
        usize::try_from(info.height()).ok()?,
    );
    let stride = usize::try_from(*info.stride().first()?).ok()?;
    let plane = frame.plane_data(0).ok()?;
    let mut pixels = Vec::with_capacity(width * height);
    for row in plane.chunks(stride).take(height) {
        pixels.extend_from_slice(row.get(..width)?);
    }
    Some(Luma {
        width,
        height,
        pixels,
    })
}
