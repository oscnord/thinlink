//! Headless receiver: a stream -> depayload -> parse -> decode -> I420 -> appsink. Each decoded
//! frame becomes an [`Observation`] with the stamp and the state stamp read back. The lab measures
//! with it; the ground adapter's freshness output decodes its source with it.

use std::collections::VecDeque;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::SyncSender;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Instant;

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_video as gst_video;

use sat_teleop_policy::stamp::{self, Stamp, StateStamp};

use crate::{MediaError, Running, luma_geometry, unix_ms};

/// RTSP jitter buffer latency, QGC's default. With a non-syncing sink it is only the longest wait
/// for a missing packet; one arriving later is dropped.
pub const LATENCY_MS: u32 = 80;
/// RTSP lower transport, as QGC requests it first.
pub const PROTOCOLS: &str = "udp";
pub const DECODER: &str = "avdec_h264";
/// How long one pull waits before the stop flag and the bus are checked again.
const PULL_TIMEOUT: gst::ClockTime = gst::ClockTime::from_mseconds(100);
/// Stamps read before scaling and not yet pulled; the appsink drops all but the newest frame.
const UNSCALED_KEPT: usize = 16;

/// Where a receiver takes its stream from.
#[derive(Debug, Clone)]
pub enum Source {
    /// RTSP as QGC plays it: UDP first, an 80 ms jitter buffer.
    Rtsp(String),
    /// RTSP over TCP with no jitter buffer wait: nothing is lost or reordered on a TCP hop, so a
    /// wait could only add latency.
    RtspTcp(String),
    /// RTP/H.264 on a UDP socket, as QGC plays a `MAVLink` camera's stream in low latency mode (no
    /// jitter buffer).
    Rtp(SocketAddr),
}

/// One decoded frame as the receiver saw it.
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    /// Monotonic receive time, for local windows and deadlines.
    pub received_at: Instant,
    /// Wall-clock receive time in UNIX milliseconds, for frame age.
    pub received_ms: u64,
    /// `None` when the stamp does not read back (CRC mismatch or a damaged frame).
    pub stamp: Option<Stamp>,
    /// The ground output's state stamp; `None` in a stream that has none.
    pub state: Option<StateStamp>,
}

/// Receives `source` until `stop` is set (`Ok`) or the pipeline fails. Observations go to the
/// bounded `observations` channel; if its consumer is gone the receiver stops.
pub fn receive(
    source: &Source,
    stop: &AtomicBool,
    observations: &SyncSender<Observation>,
) -> Result<(), MediaError> {
    pull(source, None, stop, |observation, _| {
        observations.send(observation).is_ok()
    })
}

/// Decodes `source`, scaled to `width` x `height`, and hands every frame to `each` until `stop` is
/// set, the pipeline fails or `each` returns false. The stamps are read before scaling, at the
/// size they were drawn at, so a source of any size reads back.
pub fn decode(
    source: &Source,
    (width, height): (u16, u16),
    stop: &AtomicBool,
    each: impl FnMut(Observation, gst::Sample) -> bool,
) -> Result<(), MediaError> {
    pull(source, Some((width, height)), stop, each)
}

fn pull(
    source: &Source,
    size: Option<(u16, u16)>,
    stop: &AtomicBool,
    mut each: impl FnMut(Observation, gst::Sample) -> bool,
) -> Result<(), MediaError> {
    if stop.load(Ordering::Relaxed) {
        return Ok(());
    }
    let (pipeline, appsink, unscaled) = build(source, size)?;
    let running = Running(pipeline);
    running.set(gst::State::Playing)?;
    while !stop.load(Ordering::Relaxed) {
        if let Some(error) = running.bus_error() {
            return Err(error);
        }
        let Some(sample) = appsink.try_pull_sample(PULL_TIMEOUT) else {
            continue;
        };
        let (stamp, state) = unscaled
            .as_ref()
            .and_then(|u| u.take(sample.buffer()?.pts()?))
            .unwrap_or_else(|| read_stamps(&sample));
        let observation = Observation {
            received_at: Instant::now(),
            received_ms: unix_ms(),
            stamp,
            state,
        };
        if !each(observation, sample) {
            return Ok(());
        }
    }
    Ok(())
}

/// Stamps read before scaling, by the frame's presentation time. Scaled, a stamp's blocks no
/// longer sit on the grid the reader expects: 1280-wide frames scaled to 640 have 26.5-pixel blocks
/// where the reader samples 26-pixel ones, 480-wide ones 26.7.
#[derive(Default)]
struct Unscaled {
    frames: Mutex<VecDeque<(gst::ClockTime, Stamps)>>,
}

type Stamps = (Option<Stamp>, Option<StateStamp>);

impl Unscaled {
    fn put(&self, pts: gst::ClockTime, stamps: Stamps) {
        let mut frames = self.frames.lock().unwrap_or_else(PoisonError::into_inner);
        if frames.len() >= UNSCALED_KEPT {
            frames.pop_front();
        }
        frames.push_back((pts, stamps));
    }

    /// The stamps of the frame at `pts`, forgetting it and every frame before it.
    fn take(&self, pts: gst::ClockTime) -> Option<Stamps> {
        let mut frames = self.frames.lock().unwrap_or_else(PoisonError::into_inner);
        let index = frames.iter().position(|(at, _)| *at == pts)?;
        let found = frames.get(index).map(|(_, stamps)| *stamps);
        frames.drain(..=index);
        found
    }

    /// Reads the stamps of every frame leaving `element`'s source pad, at its size.
    fn read_from(self: &Arc<Self>, element: &gst::Element) -> Result<(), MediaError> {
        let unscaled = Arc::clone(self);
        let video = Mutex::new(None);
        let probe = gst::PadProbeType::BUFFER.union(gst::PadProbeType::EVENT_DOWNSTREAM);
        element
            .static_pad("src")
            .ok_or(MediaError::Missing("converter src pad"))?
            .add_probe(probe, move |_, info| {
                match &info.data {
                    Some(gst::PadProbeData::Event(event)) => remember_format(event, &video),
                    Some(gst::PadProbeData::Buffer(buffer)) => unscaled.read(buffer, &video),
                    _ => {}
                }
                gst::PadProbeReturn::Ok
            });
        Ok(())
    }

    fn read(&self, buffer: &gst::Buffer, video: &Mutex<Option<gst_video::VideoInfo>>) {
        let Some(pts) = buffer.pts() else {
            return;
        };
        let video = video.lock().unwrap_or_else(PoisonError::into_inner);
        let stamps = video.as_ref().and_then(|v| read_frame(buffer, v));
        self.put(pts, stamps.unwrap_or((None, None)));
    }
}

fn remember_format(event: &gst::Event, video: &Mutex<Option<gst_video::VideoInfo>>) {
    let gst::EventView::Caps(caps) = event.view() else {
        return;
    };
    *video.lock().unwrap_or_else(PoisonError::into_inner) =
        gst_video::VideoInfo::from_caps(caps.caps()).ok();
}

fn build(
    source: &Source,
    size: Option<(u16, u16)>,
) -> Result<(gst::Pipeline, gst_app::AppSink, Option<Arc<Unscaled>>), MediaError> {
    let caps = gst_video::VideoCapsBuilder::new().format(gst_video::VideoFormat::I420);
    let caps = match size {
        Some((width, height)) => caps.width(i32::from(width)).height(i32::from(height)),
        None => caps,
    };
    let appsink = gst_app::AppSink::builder()
        .sync(false)
        .max_buffers(1)
        .drop(true)
        .build();
    let depay = gst::ElementFactory::make("rtph264depay").build()?;
    let convert = gst::ElementFactory::make("videoconvert").build()?;
    let mut chain = vec![
        depay.clone(),
        gst::ElementFactory::make("h264parse").build()?,
        // Frame threading holds frames back; slices only, whatever libav guesses about the source.
        gst::ElementFactory::make(DECODER)
            .property_from_str("thread-type", "slice")
            .build()?,
        convert.clone(),
    ];
    let unscaled = size.map(|_| Arc::new(Unscaled::default()));
    if let Some(unscaled) = &unscaled {
        unscaled.read_from(&convert)?;
        chain.push(gst::ElementFactory::make("videoscale").build()?);
    }
    chain.extend([
        gst::ElementFactory::make("capsfilter")
            .property("caps", caps.build())
            .build()?,
        appsink.clone().upcast(),
    ]);
    let pipeline = gst::Pipeline::new();
    pipeline.add_many(&chain)?;
    gst::Element::link_many(&chain)?;
    let input = input(source)?;
    pipeline.add(&input)?;
    if let Source::Rtp(_) = source {
        input.link(&depay)?;
        return Ok((pipeline, appsink, unscaled));
    }
    let depay_sink = depay
        .static_pad("sink")
        .ok_or(MediaError::Missing("depayloader sink pad"))?;
    // rtspsrc adds its pads once the stream is set up. A pad that fails to link surfaces as a
    // not-linked error on the bus.
    input.connect_pad_added(move |_, pad| {
        if !depay_sink.is_linked() {
            let _ = pad.link(&depay_sink);
        }
    });
    Ok((pipeline, appsink, unscaled))
}

fn input(source: &Source) -> Result<gst::Element, MediaError> {
    let element = match source {
        Source::Rtsp(url) => rtspsrc(url)
            .property("latency", LATENCY_MS)
            .property("drop-on-latency", true)
            .property_from_str("protocols", PROTOCOLS)
            .build()?,
        Source::RtspTcp(url) => rtspsrc(url)
            .property("latency", 0u32)
            .property_from_str("protocols", "tcp")
            .build()?,
        Source::Rtp(listen) => {
            let caps = gst::Caps::builder("application/x-rtp")
                .field("media", "video")
                .field("clock-rate", 90_000i32)
                .field("encoding-name", "H264")
                .build();
            gst::ElementFactory::make("udpsrc")
                .property("address", listen.ip().to_string())
                .property("port", i32::from(listen.port()))
                .property("caps", &caps)
                .build()?
        }
    };
    Ok(element)
}

fn rtspsrc(url: &str) -> gst::element_factory::ElementBuilder<'_> {
    gst::ElementFactory::make("rtspsrc").property("location", url)
}

fn read_stamps(sample: &gst::Sample) -> Stamps {
    let read = || {
        let video = gst_video::VideoInfo::from_caps(sample.caps()?).ok()?;
        read_frame(sample.buffer()?, &video)
    };
    read().unwrap_or((None, None))
}

fn read_frame(buffer: &gst::BufferRef, video: &gst_video::VideoInfo) -> Option<Stamps> {
    let frame = gst_video::VideoFrameRef::from_buffer_ref_readable(buffer, video).ok()?;
    let geometry = luma_geometry(&frame).ok()?;
    let plane = frame.plane_data(0).ok()?;
    Some((
        stamp::read(plane, geometry),
        stamp::read_state(plane, geometry),
    ))
}
