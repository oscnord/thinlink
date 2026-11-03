//! Cameras and encoders of real hardware: the V4L2 and libcamera sources, the choice of encoder and
//! the device listing behind `sat-teleop-vehicle probe-camera`.
//!
//! Nothing here has run against hardware. The pure parts (`plan`, `choose_encoder`) are tested, the
//! elements are built from `GStreamer`'s documented properties, and the first run on a Pi is the
//! check (`docs/hardware.md#what-to-measure-first`).

use std::fmt::Write;
use std::path::Path;
use std::sync::{Arc, Mutex};

use gstreamer as gst;
use gstreamer::prelude::*;
use serde::Deserialize;

use crate::{MediaError, unix_ms};

/// `GST_BUFFER_OFFSET_NONE`: the buffer has no offset.
const OFFSET_NONE: u64 = u64::MAX;

/// What a V4L2 camera is asked to send.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CameraFormat {
    /// Compressed frames, decoded here with `jpegdec`: the stamp needs raw pixels.
    Mjpeg,
    /// Uncompressed 4:2:2, needing only a conversion.
    Yuyv,
}

/// How a camera format enters the pipeline.
#[derive(Debug, PartialEq, Eq)]
pub struct CameraPlan {
    pub media_type: &'static str,
    /// The `format` field of the raw caps; none for compressed frames.
    pub raw_format: Option<&'static str>,
    /// The element that turns the frames into raw video; none when they already are.
    pub decoder: Option<&'static str>,
}

pub fn plan(format: CameraFormat) -> CameraPlan {
    match format {
        CameraFormat::Mjpeg => CameraPlan {
            media_type: "image/jpeg",
            raw_format: None,
            decoder: Some("jpegdec"),
        },
        CameraFormat::Yuyv => CameraPlan {
            media_type: "video/x-raw",
            raw_format: Some("YUY2"),
            decoder: None,
        },
    }
}

/// The encoder the vehicle config asks for.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EncoderWish {
    #[default]
    X264,
    /// The board's H.264 encoder (`v4l2h264enc`) if it has one, else x264.
    Hardware,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Encoder {
    X264,
    V4l2,
}

/// `v4l2h264enc` exists only where the board has the encoder: `GStreamer` registers it from the
/// device nodes it finds, so `hardware_present` is whether the element factory exists.
pub fn choose_encoder(wish: EncoderWish, hardware_present: bool) -> Encoder {
    match wish {
        EncoderWish::Hardware if hardware_present => Encoder::V4l2,
        EncoderWish::Hardware | EncoderWish::X264 => Encoder::X264,
    }
}

pub fn hardware_encoder_present() -> bool {
    gst::ElementFactory::find("v4l2h264enc").is_some()
}

/// A V4L2 camera at a configured mode: `v4l2src`, caps negotiation by that mode, and a decoder for
/// compressed frames.
pub fn v4l2_source(
    device: &Path,
    format: CameraFormat,
    width: u16,
    height: u16,
    fps: u16,
) -> Result<Vec<gst::Element>, MediaError> {
    let plan = plan(format);
    let mut caps = gst::Caps::builder(plan.media_type)
        .field("width", i32::from(width))
        .field("height", i32::from(height))
        .field("framerate", gst::Fraction::new(i32::from(fps), 1));
    if let Some(raw) = plan.raw_format {
        caps = caps.field("format", raw);
    }
    let mut elements = vec![
        gst::ElementFactory::make("v4l2src")
            .property("device", device.display().to_string())
            .build()?,
        gst::ElementFactory::make("capsfilter")
            .property("caps", caps.build())
            .build()?,
    ];
    if let Some(decoder) = plan.decoder {
        elements.push(gst::ElementFactory::make(decoder).build()?);
    }
    Ok(elements)
}

/// When each frame of a V4L2 camera was captured, from the capture buffer's own timestamp and
/// sequence number instead of the time the frame reached the stamp. A camera that sends the same
/// frame again (a driver that re-queues its last buffer, a sensor that stalled) then keeps that
/// frame's old capture time, and the ground reads `Frozen` and then `Stale` instead of a fresh
/// picture. What it cannot see: a driver that gives a repeated image a new timestamp and sequence
/// number, which only comparing images would show.
#[derive(Debug, Default)]
pub struct CaptureClock {
    state: Mutex<Capture>,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Capture {
    sequence: Option<u64>,
    capture_ms: u64,
}

impl Capture {
    /// The state after a buffer with `sequence` that was `age_ms` old at `now_ms` (UNIX ms). A
    /// buffer with the sequence number of the one before is that frame again; otherwise the capture
    /// time is when the buffer was captured and never goes back.
    fn next(self, sequence: Option<u64>, age_ms: u64, now_ms: u64) -> Self {
        if sequence.is_some() && sequence == self.sequence {
            return self;
        }
        Self {
            sequence,
            capture_ms: now_ms.saturating_sub(age_ms).max(self.capture_ms),
        }
    }
}

impl CaptureClock {
    /// The capture time, in UNIX ms, of the newest buffer the source delivered; none before the
    /// first.
    pub fn latest_ms(&self) -> Option<u64> {
        let capture = self.state.lock().ok()?.capture_ms;
        (capture > 0).then_some(capture)
    }

    fn observe(&self, sequence: Option<u64>, age_ms: u64) {
        if let Ok(mut state) = self.state.lock() {
            *state = state.next(sequence, age_ms, unix_ms());
        }
    }
}

/// Reads each buffer `source` (the camera element) delivers into `clock`: its age is the pipeline's
/// running time minus the buffer's timestamp, its identity the buffer's offset, which `v4l2src` sets
/// to the driver's sequence number. The stamp, downstream in the same streaming thread, reads
/// `clock` for the buffer it is about to mark.
pub fn watch_capture(source: &gst::Element, clock: &Arc<CaptureClock>) -> Result<(), MediaError> {
    let pad = source
        .static_pad("src")
        .ok_or(MediaError::Missing("camera src pad"))?;
    let clock = Arc::clone(clock);
    pad.add_probe(gst::PadProbeType::BUFFER, move |pad, info| {
        let Some(gst::PadProbeData::Buffer(buffer)) = &info.data else {
            return gst::PadProbeReturn::Ok;
        };
        let sequence = Some(buffer.offset()).filter(|offset| *offset != OFFSET_NONE);
        let running = pad
            .parent_element()
            .and_then(|element| element.current_running_time());
        let age = running
            .zip(buffer.pts())
            .map_or(0, |(now, pts)| now.saturating_sub(pts).mseconds());
        clock.observe(sequence, age);
        gst::PadProbeReturn::Ok
    });
    Ok(())
}

/// The Raspberry Pi camera modules through libcamera (`libcamerasrc`, from
/// `gstreamer1.0-libcamera`), at a configured size and rate.
pub fn libcamera_source(
    width: u16,
    height: u16,
    fps: u16,
) -> Result<Vec<gst::Element>, MediaError> {
    let caps = gst::Caps::builder("video/x-raw")
        .field("width", i32::from(width))
        .field("height", i32::from(height))
        .field("framerate", gst::Fraction::new(i32::from(fps), 1))
        .build();
    Ok(vec![
        gst::ElementFactory::make("libcamerasrc").build()?,
        gst::ElementFactory::make("capsfilter")
            .property("caps", caps)
            .build()?,
    ])
}

/// What the camera listing prints for each video source `GStreamer`'s device monitor finds.
pub fn probe() -> Result<Vec<String>, MediaError> {
    let monitor = gst::DeviceMonitor::new();
    monitor.add_filter(Some("Video/Source"), None);
    monitor.start()?;
    let devices = monitor.devices();
    monitor.stop();
    Ok(devices.iter().map(describe).collect())
}

fn describe(device: &gst::Device) -> String {
    let path = device
        .properties()
        .and_then(|p| p.get::<String>("device.path").ok())
        .unwrap_or_else(|| "no device path".to_owned());
    let mut text = format!("{} ({path})", device.display_name());
    for structure in device.caps().iter().flat_map(|caps| caps.iter()) {
        let _ = write!(text, "\n    {structure}");
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_formats_enter_the_pipeline_as_raw_video() {
        let mjpeg = plan(CameraFormat::Mjpeg);
        let yuyv = plan(CameraFormat::Yuyv);
        assert_eq!(
            (mjpeg.media_type, mjpeg.raw_format, mjpeg.decoder),
            ("image/jpeg", None, Some("jpegdec"))
        );
        assert_eq!(
            (yuyv.media_type, yuyv.raw_format, yuyv.decoder),
            ("video/x-raw", Some("YUY2"), None)
        );
    }

    #[test]
    fn a_repeated_frame_keeps_its_old_capture_time() {
        // (sequence, age in ms, now in ms) -> the capture time after it
        let steps = [
            (Some(1), 5, 1005, 1000),
            (Some(2), 4, 1038, 1034),
            // The driver delivers frame 2 again, 33 ms later and then 66 ms later.
            (Some(2), 37, 1071, 1034),
            (Some(2), 70, 1104, 1034),
            // A repeat with a new sequence number but its old timestamp: no later capture.
            (Some(3), 103, 1137, 1034),
            // The camera recovers.
            (Some(4), 6, 1170, 1164),
            // Without sequence numbers the timestamp alone decides, and time never goes back.
            (None, 6, 1203, 1197),
            (None, 80, 1236, 1197),
        ];
        let mut capture = Capture::default();
        for (sequence, age, now, expected) in steps {
            capture = capture.next(sequence, age, now);
            assert_eq!(
                capture.capture_ms, expected,
                "sequence {sequence:?} at {now}"
            );
        }
    }

    #[test]
    fn the_hardware_encoder_is_used_only_where_the_board_has_one() {
        use Encoder::{V4l2, X264};
        let cases = [
            (EncoderWish::X264, true, X264),
            (EncoderWish::X264, false, X264),
            (EncoderWish::Hardware, true, V4l2),
            (EncoderWish::Hardware, false, X264),
        ];
        for (wish, present, chosen) in cases {
            assert_eq!(choose_encoder(wish, present), chosen, "{wish:?} {present}");
        }
    }
}
