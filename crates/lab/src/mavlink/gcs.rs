//! `checks` as a ground station of the ground adapter: one UDP socket, a heartbeat every second,
//! optionally a message repeated at a fixed rate, and a compact record of what arrives. Also a
//! silent observer of telemetry.

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::dialects::ardupilotmega::{
    CameraCapFlags, CameraTrackingStatusFlags, HEARTBEAT_DATA, MavAutopilot, MavCmd, MavMessage,
    MavMissionResult, MavModeFlag, MavResult, MavState, MavType, VideoStreamStatusFlags,
    VideoStreamType,
};
use mavlink::types::CharArray;

use thinlink_control::frame::{MAX_DATAGRAM, datagram_frames};
use thinlink_policy::landing;
use thinlink_video::unix_ms;

/// SITL's system id and autopilot component.
const VEHICLE_SYSTEM: u32 = 1;
const VEHICLE_COMPONENT: u8 = 1;
/// The ground adapter answers as the vehicle's camera (`MAV_COMP_ID_CAMERA`).
pub(crate) const CAMERA_COMPONENT: u8 = 100;
/// `QGroundControl`'s ids, so the vehicle sees a familiar ground station.
const GCS_SYSTEM: u32 = 255;
const GCS_COMPONENT: u8 = 190;
/// Receive timeout, and so the granularity of sending.
const POLL: Duration = Duration::from_millis(20);
pub(crate) const HEARTBEAT_EVERY: Duration = Duration::from_secs(1);
/// More than an hour of SITL telemetry; later messages are counted, not kept.
const MAX_SEEN: usize = 500_000;

/// What `checks` received, as far as the checks need it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Kind {
    /// `mode` is the autopilot's custom mode.
    Heartbeat {
        armed: bool,
        mode: u32,
    },
    Ack {
        command: MavCmd,
        result: MavResult,
    },
    AutopilotVersion,
    /// `VFR_HUD`'s throttle, in percent.
    Hud {
        throttle: u16,
    },
    /// `GLOBAL_POSITION_INT`: the autopilot's boot time, altitude above home (m), latitude and
    /// longitude in degrees times 1e7.
    Position {
        boot_ms: u32,
        rel_alt_m: f32,
        lat: i32,
        lon: i32,
    },
    /// `GIMBAL_DEVICE_ATTITUDE_STATUS`: how far the camera's axis is from straight down and how far
    /// the picture's top is turned from the nose (degrees).
    Mount {
        off_nadir_deg: f32,
        turn_deg: f32,
        /// The reported quaternion, for the landing trace.
        q: [f32; 4],
    },
    /// `ATTITUDE` in radians: yaw is north 0 and clockwise positive.
    Attitude {
        roll: f32,
        pitch: f32,
        yaw: f32,
    },
    /// The autopilot asking for mission item `seq` (`MISSION_REQUEST`, which is what `ArduPilot` sends,
    /// or `MISSION_REQUEST_INT`).
    MissionRequest {
        seq: u16,
    },
    /// The end of a mission upload, or the ground adapter's refusal of one.
    MissionAck {
        result: MavMissionResult,
    },
    /// `MISSION_ITEM_REACHED`.
    MissionReached {
        seq: u16,
    },
    /// `GPS_RAW_INT`'s ground speed, in m/s.
    Gps {
        speed: f32,
    },
    /// The autopilot's `STATUSTEXT`: what it says about a failsafe or a refusal.
    StatusText {
        text: CharArray<50>,
    },
    CameraInformation {
        video_stream: bool,
    },
    VideoStreamInformation {
        rtp_udp: bool,
        thermal: bool,
    },
    VideoStreamStatus {
        thermal: bool,
    },
    /// `CAMERA_TRACKING_IMAGE_STATUS`: whether a box is tracked, and its corners (x0, y0, x1, y1).
    TrackingStatus {
        active: bool,
        rect: [f32; 4],
    },
    Other,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Seen {
    pub(crate) at: Instant,
    pub(crate) unix_ms: u64,
    /// From SITL's autopilot (or the adapter answering for it).
    pub(crate) from_vehicle: bool,
    /// From the adapter's camera component.
    pub(crate) from_camera: bool,
    pub(crate) size: u32,
    pub(crate) kind: Kind,
}

impl Seen {
    pub(crate) fn vehicle_heartbeat(&self) -> bool {
        self.from_vehicle && matches!(self.kind, Kind::Heartbeat { .. })
    }
}

/// `checks` as a ground station: one socket, heartbeats once a second to the adapter it hears.
pub(crate) struct Gcs {
    pub(crate) socket: UdpSocket,
    adapter: Option<SocketAddr>,
    sequence: u8,
    /// `None` for a silent observer.
    next_heartbeat: Option<Instant>,
    stream: Option<Stream>,
    pub(crate) seen: Vec<Seen>,
    pub(crate) dropped: u64,
    pub(crate) undecodable: u64,
}

impl Gcs {
    pub(crate) fn bind(listen: SocketAddr) -> io::Result<Self> {
        let socket = UdpSocket::bind(listen)?;
        socket.set_read_timeout(Some(POLL))?;
        Ok(Self {
            socket,
            adapter: None,
            sequence: 0,
            next_heartbeat: Some(Instant::now()),
            stream: None,
            seen: Vec::new(),
            dropped: 0,
            undecodable: 0,
        })
    }

    /// A socket that only listens and records.
    pub(crate) fn observe(listen: SocketAddr) -> io::Result<Self> {
        Ok(Self {
            next_heartbeat: None,
            ..Self::bind(listen)?
        })
    }

    /// Sends `message` every `every` while pumping, until called with `None`.
    pub(crate) fn stream(&mut self, message: Option<MavMessage>, every: Duration) {
        self.stream = message.map(|message| Stream {
            message,
            every,
            next: Instant::now(),
        });
    }

    /// Receives until `until`.
    pub(crate) fn pump(&mut self, until: Instant) {
        self.pump_until(until, |_| false);
    }

    /// Receives until `until` or until `done` holds for what arrived meanwhile; returns `done`.
    pub(crate) fn pump_until(&mut self, until: Instant, done: impl Fn(&[Seen]) -> bool) -> bool {
        let from = self.seen.len();
        loop {
            if done(self.seen.get(from..).unwrap_or_default()) {
                return true;
            }
            if Instant::now() >= until {
                return false;
            }
            self.heartbeat_if_due();
            self.stream_if_due();
            self.receive();
        }
    }

    fn heartbeat_if_due(&mut self) {
        let now = Instant::now();
        let due = self.next_heartbeat.is_some_and(|next| now >= next);
        if self.adapter.is_none() || !due {
            return;
        }
        self.next_heartbeat = Some(now + HEARTBEAT_EVERY);
        self.send(&gcs_heartbeat());
    }

    fn stream_if_due(&mut self) {
        let now = Instant::now();
        let Some(stream) = self.stream.as_mut().filter(|s| now >= s.next) else {
            return;
        };
        stream.next = now + stream.every;
        let message = stream.message.clone();
        self.send(&message);
    }

    /// Sends to the adapter; false before it was heard from or when sending fails.
    pub(crate) fn send(&mut self, message: &MavMessage) -> bool {
        let Some(adapter) = self.adapter else {
            return false;
        };
        let header = MavHeader {
            system_id: GCS_SYSTEM,
            component_id: GCS_COMPONENT,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        let mut bytes = Vec::new();
        mavlink::write_v2_msg(&mut bytes, header, message).is_ok()
            && self.socket.send_to(&bytes, adapter).is_ok()
    }

    fn receive(&mut self) {
        let mut buf = [0_u8; MAX_DATAGRAM];
        let Ok((len, from)) = self.socket.recv_from(&mut buf) else {
            return;
        };
        let (at, unix_ms) = (Instant::now(), unix_ms());
        self.adapter = Some(from);
        for frame in datagram_frames(buf.get(..len).unwrap_or_default()) {
            let Ok(message) = frame.decode() else {
                self.undecodable += 1;
                continue;
            };
            let vehicle = frame.system_id() == VEHICLE_SYSTEM;
            let seen = Seen {
                at,
                unix_ms,
                from_vehicle: vehicle && frame.component_id() == VEHICLE_COMPONENT,
                from_camera: vehicle && frame.component_id() == CAMERA_COMPONENT,
                size: frame.size(),
                kind: kind(&message),
            };
            self.keep(seen);
        }
    }

    fn keep(&mut self, seen: Seen) {
        if self.seen.len() < MAX_SEEN {
            self.seen.push(seen);
        } else {
            self.dropped += 1;
        }
    }
}

/// A message sent at a fixed rate, as a joystick does.
struct Stream {
    message: MavMessage,
    every: Duration,
    next: Instant,
}

fn kind(message: &MavMessage) -> Kind {
    match message {
        MavMessage::HEARTBEAT(h) => Kind::Heartbeat {
            armed: h
                .base_mode
                .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),
            mode: h.custom_mode,
        },
        MavMessage::VFR_HUD(h) => Kind::Hud {
            throttle: h.throttle,
        },
        MavMessage::GPS_RAW_INT(g) if g.vel != u16::MAX => Kind::Gps {
            speed: f32::from(g.vel) / 100.0,
        },
        #[expect(clippy::cast_precision_loss, reason = "millimetres of altitude")]
        MavMessage::GLOBAL_POSITION_INT(p) => Kind::Position {
            boot_ms: p.time_boot_ms,
            rel_alt_m: p.relative_alt as f32 / 1000.0,
            lat: p.lat,
            lon: p.lon,
        },
        MavMessage::ATTITUDE(a) => Kind::Attitude {
            roll: a.roll,
            pitch: a.pitch,
            yaw: a.yaw,
        },
        MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(g) => landing::off_nadir_deg(g.q)
            .zip(landing::picture_turn_deg(g.q))
            .map_or(Kind::Other, |(off_nadir_deg, turn_deg)| Kind::Mount {
                off_nadir_deg,
                turn_deg,
                q: g.q,
            }),
        MavMessage::MISSION_REQUEST_INT(r) => Kind::MissionRequest { seq: r.seq },
        #[expect(
            deprecated,
            reason = "ArduPilot asks for the items of an upload with MISSION_REQUEST"
        )]
        MavMessage::MISSION_REQUEST(r) => Kind::MissionRequest { seq: r.seq },
        MavMessage::MISSION_ACK(a) => Kind::MissionAck { result: a.mavtype },
        MavMessage::MISSION_ITEM_REACHED(r) => Kind::MissionReached { seq: r.seq },
        MavMessage::COMMAND_ACK(a) => Kind::Ack {
            command: a.command,
            result: a.result,
        },
        MavMessage::AUTOPILOT_VERSION(_) => Kind::AutopilotVersion,
        MavMessage::STATUSTEXT(t) => Kind::StatusText { text: t.text },
        MavMessage::CAMERA_INFORMATION(c) => Kind::CameraInformation {
            video_stream: c
                .flags
                .contains(CameraCapFlags::CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM),
        },
        MavMessage::VIDEO_STREAM_INFORMATION(v) => Kind::VideoStreamInformation {
            rtp_udp: v.mavtype == VideoStreamType::VIDEO_STREAM_TYPE_RTPUDP,
            thermal: v.flags.contains(THERMAL),
        },
        MavMessage::VIDEO_STREAM_STATUS(v) => Kind::VideoStreamStatus {
            thermal: v.flags.contains(THERMAL),
        },
        MavMessage::CAMERA_TRACKING_IMAGE_STATUS(t) => Kind::TrackingStatus {
            active: t
                .tracking_status
                .contains(CameraTrackingStatusFlags::CAMERA_TRACKING_STATUS_FLAGS_ACTIVE),
            rect: [t.rec_top_x, t.rec_top_y, t.rec_bottom_x, t.rec_bottom_y],
        },
        _ => Kind::Other,
    }
}

const THERMAL: VideoStreamStatusFlags = VideoStreamStatusFlags::VIDEO_STREAM_STATUS_FLAGS_THERMAL;

fn gcs_heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
        ..HEARTBEAT_DATA::default()
    })
}
