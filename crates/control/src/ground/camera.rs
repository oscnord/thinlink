//! The ground adapter as the vehicle's camera component (100) for the ground stations: it announces
//! the ground's freshness output as an RTP/UDP H.264 stream, which `QGroundControl` then plays, and
//! answers every message addressed to the camera itself, so none reaches the vehicle or travels in
//! an envelope. It speaks only while the vehicle is heard: `QGroundControl` takes any message from
//! the vehicle's system id as a sign of life, and a made-up one must never hide a lost link.

use std::collections::BTreeMap;
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    CAMERA_INFORMATION_DATA, COMMAND_ACK_DATA, CameraCapFlags, HEARTBEAT_DATA, MavAutopilot,
    MavCmd, MavMessage, MavModeFlag, MavResult, MavState, MavType, VIDEO_STREAM_INFORMATION_DATA,
    VIDEO_STREAM_STATUS_DATA, VideoStreamStatusFlags, VideoStreamType,
};
use mavlink::{MavHeader, Message};
use serde::Serialize;

use crate::frame::Frame;

/// `MAV_COMP_ID_CAMERA`.
const CAMERA: u8 = 100;
/// `MAV_COMP_ID_AUTOPILOT1`, whose heartbeat the camera's follows.
const AUTOPILOT: u8 = 1;
const HEARTBEAT_ID: u32 = 0;
/// The vehicle counts as heard this long after its autopilot's last message (`ArduPilot` sends a
/// burst every second). `QGroundControl` 5.1.5 declares the link lost 3.5 s after the last message
/// from the vehicle's system id, so a camera answer can delay that by at most this much.
const HEARD_FOR: Duration = Duration::from_millis(1200);
/// The one stream.
const STREAM_ID: u8 = 1;
const STREAM_NAME: &str = "overview";
const VENDOR: &str = "sat-teleop";
const MODEL: &str = "ground freshness output";

/// The stream the camera announces: the freshness output's encoding and its URI.
#[derive(Debug, Clone)]
pub struct CameraStream {
    pub width: u16,
    pub height: u16,
    pub fps: u16,
    pub bitrate_kbit: u32,
    pub uri: String,
}

#[derive(Debug, Default, Clone, Serialize)]
pub struct CameraCounts {
    /// Heartbeats sent, one after each of the vehicle's.
    pub heartbeats: u64,
    /// Requests answered with the message asked for, by message.
    pub answered: BTreeMap<&'static str, u64>,
    /// Commands answered `MAV_RESULT_UNSUPPORTED`.
    pub unsupported: u64,
    /// Messages to the camera dropped: not a command, or sent while the vehicle was not heard.
    pub dropped: u64,
}

pub struct Camera {
    stream: CameraStream,
    started: Instant,
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    /// The vehicle's system id and when its heartbeat last passed.
    heard: Option<(u8, Instant)>,
    sequence: u8,
    counts: CameraCounts,
}

/// What a camera command asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Requested {
    Information,
    StreamInformation,
    StreamStatus,
}

impl Requested {
    fn name(self) -> &'static str {
        match self {
            Self::Information => "CAMERA_INFORMATION",
            Self::StreamInformation => "VIDEO_STREAM_INFORMATION",
            Self::StreamStatus => "VIDEO_STREAM_STATUS",
        }
    }
}

/// Whether `message` is addressed to the camera component.
pub(super) fn addressed(message: &MavMessage) -> bool {
    message.target_component_id() == Some(CAMERA)
}

impl Camera {
    pub fn new(stream: CameraStream) -> Self {
        Self {
            stream,
            started: Instant::now(),
            state: Mutex::new(State::default()),
        }
    }

    pub fn counts(&self) -> CameraCounts {
        self.lock().counts.clone()
    }

    /// Notes a frame from the vehicle; returns the camera's heartbeat to follow it to the stations
    /// if it is the autopilot's heartbeat.
    pub(super) fn follow(&self, frame: &Frame) -> Option<Vec<u8>> {
        if frame.component_id() != AUTOPILOT {
            return None;
        }
        let system = u8::try_from(frame.system_id()).ok()?;
        let mut state = self.lock();
        state.heard = Some((system, Instant::now()));
        if frame.message_id() != HEARTBEAT_ID {
            return None;
        }
        state.counts.heartbeats += 1;
        state.bytes(system, &heartbeat())
    }

    /// The answers to a message addressed to the camera: `COMMAND_ACK` and, for a request it
    /// serves, the message asked for. Nothing unless the vehicle was heard within `HEARD_FOR`.
    pub(super) fn answer(&self, message: &MavMessage) -> Vec<Vec<u8>> {
        let mut state = self.lock();
        let heard = state.heard.filter(|(_, at)| at.elapsed() <= HEARD_FOR);
        let (Some((system, _)), Some((command, param1, param2))) = (heard, command(message)) else {
            state.counts.dropped += 1;
            return Vec::new();
        };
        let Some(requested) = requested(command, param1, param2) else {
            state.counts.unsupported += 1;
            return state
                .bytes(system, &ack(command, MavResult::MAV_RESULT_UNSUPPORTED))
                .into_iter()
                .collect();
        };
        *state.counts.answered.entry(requested.name()).or_default() += 1;
        let reply = self.message(requested);
        [ack(command, MavResult::MAV_RESULT_ACCEPTED), reply]
            .iter()
            .filter_map(|message| state.bytes(system, message))
            .collect()
    }

    fn message(&self, requested: Requested) -> MavMessage {
        let s = &self.stream;
        let framerate = f32::from(s.fps);
        let bitrate = s.bitrate_kbit.saturating_mul(1000);
        let running = VideoStreamStatusFlags::VIDEO_STREAM_STATUS_FLAGS_RUNNING;
        match requested {
            Requested::Information => MavMessage::CAMERA_INFORMATION(CAMERA_INFORMATION_DATA {
                time_boot_ms: u32::try_from(self.started.elapsed().as_millis()).unwrap_or(u32::MAX),
                vendor_name: name(VENDOR),
                model_name: name(MODEL),
                flags: CameraCapFlags::CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM,
                resolution_h: s.width,
                resolution_v: s.height,
                ..CAMERA_INFORMATION_DATA::default()
            }),
            Requested::StreamInformation => {
                MavMessage::VIDEO_STREAM_INFORMATION(VIDEO_STREAM_INFORMATION_DATA {
                    framerate,
                    bitrate,
                    flags: running,
                    resolution_h: s.width,
                    resolution_v: s.height,
                    stream_id: STREAM_ID,
                    count: 1,
                    mavtype: VideoStreamType::VIDEO_STREAM_TYPE_RTPUDP,
                    name: STREAM_NAME.into(),
                    uri: s.uri.as_str().into(),
                    ..VIDEO_STREAM_INFORMATION_DATA::default()
                })
            }
            Requested::StreamStatus => MavMessage::VIDEO_STREAM_STATUS(VIDEO_STREAM_STATUS_DATA {
                framerate,
                bitrate,
                flags: running,
                resolution_h: s.width,
                resolution_v: s.height,
                stream_id: STREAM_ID,
                ..VIDEO_STREAM_STATUS_DATA::default()
            }),
        }
    }

    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

impl State {
    /// `message` as a `MAVLink` 2 frame from the camera of `system`.
    fn bytes(&mut self, system: u8, message: &MavMessage) -> Option<Vec<u8>> {
        let header = MavHeader {
            system_id: u32::from(system),
            component_id: CAMERA,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        let mut bytes = Vec::new();
        mavlink::write_v2_msg(&mut bytes, header, message).ok()?;
        Some(bytes)
    }
}

/// A command's id and first two parameters.
fn command(message: &MavMessage) -> Option<(MavCmd, f32, f32)> {
    match message {
        MavMessage::COMMAND_LONG(c) => Some((c.command, c.param1, c.param2)),
        MavMessage::COMMAND_INT(c) => Some((c.command, c.param1, c.param2)),
        _ => None,
    }
}

/// What a camera command asks for: `REQUEST_MESSAGE` for the camera information or the stream's
/// information or status (stream 0, all, or 1), or the deprecated commands `QGroundControl` falls
/// back to on a retry. `None` for anything else.
#[expect(
    clippy::float_cmp,
    reason = "message and stream ids are small integers, exact in f32"
)]
#[expect(
    deprecated,
    reason = "QGroundControl 5.1.5 still sends them on a retry"
)]
fn requested(command: MavCmd, param1: f32, param2: f32) -> Option<Requested> {
    let ours = |stream: f32| stream == 0.0 || stream == f32::from(STREAM_ID);
    let wanted = match command {
        MavCmd::MAV_CMD_REQUEST_MESSAGE if param1 == 259.0 => Requested::Information,
        MavCmd::MAV_CMD_REQUEST_MESSAGE if param1 == 269.0 && ours(param2) => {
            Requested::StreamInformation
        }
        MavCmd::MAV_CMD_REQUEST_MESSAGE if param1 == 270.0 && ours(param2) => {
            Requested::StreamStatus
        }
        MavCmd::MAV_CMD_REQUEST_CAMERA_INFORMATION => Requested::Information,
        MavCmd::MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION if ours(param1) => {
            Requested::StreamInformation
        }
        MavCmd::MAV_CMD_REQUEST_VIDEO_STREAM_STATUS if ours(param1) => Requested::StreamStatus,
        _ => return None,
    };
    Some(wanted)
}

fn heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_CAMERA,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

fn ack(command: MavCmd, result: MavResult) -> MavMessage {
    MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command, result })
}

/// `text` as a NUL-padded 32-byte field.
fn name(text: &str) -> [u8; 32] {
    let mut field = [0; 32];
    for (slot, byte) in field.iter_mut().zip(text.bytes()) {
        *slot = byte;
    }
    field
}
