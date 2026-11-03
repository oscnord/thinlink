//! `GStreamer` pipelines of the live video path: the vehicle sender, the relay ingress, the
//! headless receiver `lab` measures with and the ground's freshness output; media feedback between
//! the vehicle and the relay and the vehicle's adaptation to it; and the JSON metrics endpoint the
//! applications serve.

use std::net::SocketAddr;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_video::prelude::*;
use serde::Serialize;

use sat_teleop_policy::stamp::{PlaneGeometry, StampError};

pub mod adapt;
pub mod camera;
pub mod card;
pub mod feedback;
pub mod ingest;
pub mod ingress;
pub mod inset;
pub mod metrics;
pub mod output;
pub mod receiver;
pub mod region;
pub mod sender;
pub mod stages;

/// Pause between a failed pipeline run and the next attempt.
const RESTART_DELAY: Duration = Duration::from_secs(1);

#[derive(Debug, thiserror::Error)]
pub enum MediaError {
    #[error("GStreamer does not initialise: {0}")]
    Init(#[from] gst::glib::Error),
    #[error("GStreamer: {0}")]
    Glib(#[from] gst::glib::BoolError),
    #[error("cannot set the pipeline to {state:?}: {reason}")]
    State { state: gst::State, reason: String },
    #[error("{element}: {message} ({debug})")]
    Stream {
        element: String,
        message: String,
        debug: String,
    },
    #[error("the stream ended")]
    Eos,
    #[error("the pipeline has no {0}")]
    Missing(&'static str),
    #[error(transparent)]
    Stamp(#[from] StampError),
    #[error("the crop cannot be shown: {0}")]
    Crop(String),
    #[error("invalid config: {0}")]
    Config(String),
    #[error("cannot serve metrics on {addr}: {reason}")]
    Metrics { addr: String, reason: String },
    #[error("UDP socket on {addr}: {reason}")]
    Socket { addr: SocketAddr, reason: String },
}

pub fn init() -> Result<(), MediaError> {
    Ok(gst::init()?)
}

/// Wall-clock UNIX milliseconds; zero if the clock reads before 1970.
pub fn unix_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_millis()).unwrap_or(u64::MAX))
}

/// Whether a pipeline is running and why it last failed. A failed pipeline never looks healthy.
#[derive(Debug, Default)]
pub struct Health {
    runs: AtomicU64,
    running: AtomicBool,
    last_error: Mutex<Option<String>>,
}

#[derive(Debug, Serialize)]
pub struct HealthSnapshot {
    /// Pipeline starts since the process began; above 1 means it failed and restarted.
    pub runs: u64,
    pub running: bool,
    pub last_error: Option<String>,
}

impl Health {
    pub fn running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    pub fn snapshot(&self) -> HealthSnapshot {
        HealthSnapshot {
            runs: self.runs.load(Ordering::Relaxed),
            running: self.running.load(Ordering::Relaxed),
            last_error: self.last_error.lock().ok().and_then(|e| e.clone()),
        }
    }

    fn record_failure(&self, error: &MediaError) {
        if let Ok(mut last) = self.last_error.lock() {
            *last = Some(error.to_string());
        }
    }
}

/// Runs `run` until it returns `Ok`, starting it again `RESTART_DELAY` after every failure.
pub fn supervise(health: &Health, mut run: impl FnMut() -> Result<(), MediaError>) {
    loop {
        health.runs.fetch_add(1, Ordering::Relaxed);
        health.running.store(true, Ordering::Relaxed);
        let result = run();
        health.running.store(false, Ordering::Relaxed);
        let Err(error) = result else { return };
        eprintln!("pipeline failed, restarting: {error}");
        health.record_failure(&error);
        thread::sleep(RESTART_DELAY);
    }
}

/// Packets and bytes through a pad.
#[derive(Debug, Default)]
pub struct Traffic {
    packets: AtomicU64,
    bytes: AtomicU64,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct TrafficSnapshot {
    pub packets: u64,
    pub bytes: u64,
}

impl Traffic {
    fn record(&self, data: Option<&gst::PadProbeData<'_>>) {
        let (packets, bytes) = match data {
            Some(gst::PadProbeData::Buffer(buffer)) => (1, buffer.size()),
            Some(gst::PadProbeData::BufferList(list)) => (list.len(), list.calculate_size()),
            _ => return,
        };
        self.packets.fetch_add(packets as u64, Ordering::Relaxed);
        self.bytes.fetch_add(bytes as u64, Ordering::Relaxed);
    }

    fn snapshot(&self) -> TrafficSnapshot {
        TrafficSnapshot {
            packets: self.packets.load(Ordering::Relaxed),
            bytes: self.bytes.load(Ordering::Relaxed),
        }
    }
}

/// Probe mask for counting what a pad carries, single buffers or lists.
const DATA_PROBE: gst::PadProbeType =
    gst::PadProbeType::BUFFER.union(gst::PadProbeType::BUFFER_LIST);

/// A started pipeline. Dropping it sets the pipeline to `Null`, so sockets are released before a
/// restart binds them again.
struct Running(gst::Pipeline);

impl Running {
    fn set(&self, state: gst::State) -> Result<(), MediaError> {
        match self.0.set_state(state) {
            Ok(_) => Ok(()),
            Err(e) => Err(self.bus_error().unwrap_or(MediaError::State {
                state,
                reason: e.to_string(),
            })),
        }
    }

    /// The first error or end of stream waiting on the bus, if any.
    fn bus_error(&self) -> Option<MediaError> {
        let bus = self.0.bus()?;
        let message = bus.pop_filtered(&[gst::MessageType::Error, gst::MessageType::Eos])?;
        failure(&message)
    }
}

impl Drop for Running {
    fn drop(&mut self) {
        let _ = self.0.set_state(gst::State::Null);
    }
}

/// An error or end-of-stream message as a typed error; `None` for any other message.
fn failure(message: &gst::Message) -> Option<MediaError> {
    match message.view() {
        gst::MessageView::Error(err) => Some(MediaError::Stream {
            element: err
                .src()
                .map(|src| src.path_string().to_string())
                .unwrap_or_default(),
            message: err.error().to_string(),
            debug: err.debug().map(|d| d.to_string()).unwrap_or_default(),
        }),
        gst::MessageView::Eos(_) => Some(MediaError::Eos),
        _ => None,
    }
}

/// Geometry of the luma plane of a mapped I420 frame.
fn luma_geometry(frame: &impl VideoFrameExt) -> Result<PlaneGeometry, MediaError> {
    plane_geometry(frame, 0)
}

/// Geometry of plane `plane` of a mapped planar frame, where each plane is one component (I420).
fn plane_geometry(frame: &impl VideoFrameExt, plane: u32) -> Result<PlaneGeometry, MediaError> {
    let stride = frame
        .plane_stride()
        .get(plane as usize)
        .copied()
        .unwrap_or(0);
    let invalid = |_| MediaError::Missing("valid plane geometry");
    Ok(PlaneGeometry {
        width: usize::try_from(frame.comp_width(plane)).map_err(invalid)?,
        height: usize::try_from(frame.comp_height(plane)).map_err(invalid)?,
        stride: usize::try_from(stride).map_err(invalid)?,
    })
}
