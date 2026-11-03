//! The ground adapter as the vehicle's camera component (100) for the ground stations: it announces
//! the ground's freshness output as an RTP/UDP H.264 stream, which `QGroundControl` then plays, and
//! answers every message addressed to the camera itself, so none reaches the vehicle or travels in
//! an envelope. It speaks only while the vehicle is heard: `QGroundControl` takes any message from
//! the vehicle's system id as a sign of life, and a made-up one must never hide a lost link.

use std::collections::BTreeMap;
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    CAMERA_INFORMATION_DATA, CAMERA_TRACKING_IMAGE_STATUS_DATA, COMMAND_ACK_DATA, CameraCapFlags,
    CameraTrackingMode, CameraTrackingStatusFlags, HEARTBEAT_DATA, MavAutopilot, MavCmd,
    MavMessage, MavModeFlag, MavResult, MavState, MavType, VIDEO_STREAM_INFORMATION_DATA,
    VIDEO_STREAM_STATUS_DATA, VideoStreamStatusFlags, VideoStreamType,
};
use mavlink::{MavHeader, Message};
use serde::Serialize;

use sat_teleop_policy::region::NormRect;

use crate::envelope::TrackingLimits;
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
/// `CAMERA_TRACKING_IMAGE_STATUS`.
const TRACKING_STATUS_ID: f32 = 275.0;
/// How often the tracking status goes out when a station asks for the default rate (interval 0).
const DEFAULT_STATUS_EVERY: Duration = Duration::from_millis(500);
const MIN_STATUS_EVERY: Duration = Duration::from_millis(50);
/// A crop for the box counts as the box being served for this long after it arrived.
const DETAIL_TTL: Duration = Duration::from_secs(30);

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
    /// Tracking commands refused: a box too small to keep sharp.
    pub tracking_denied: u64,
}

pub struct Camera {
    stream: CameraStream,
    started: Instant,
    /// How long a box the vehicle confirmed counts as applied.
    pilot_ttl: Duration,
    refresh_every: Duration,
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    /// The vehicle's system id and when its heartbeat last passed.
    heard: Option<(u8, Instant)>,
    sequence: u8,
    counts: CameraCounts,
    /// The box a station asked for.
    tracking: Option<NormRect>,
    /// The sequence number of the first region envelope that carried `tracking`; none until sent.
    box_from: Option<u64>,
    /// The box the vehicle last said it applies, and when it said so.
    applied: Option<(NormRect, Instant)>,
    /// The box a retained crop was last fetched for, and when it arrived.
    detail: Option<(NormRect, Instant)>,
    /// How often a station asked for the tracking status, and when it last went out.
    status_every: Option<Duration>,
    status_sent: Option<Instant>,
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
    pub fn new(stream: CameraStream, tracking: TrackingLimits) -> Self {
        Self {
            stream,
            started: Instant::now(),
            pilot_ttl: Duration::from_millis(tracking.pilot_ttl_ms),
            refresh_every: Duration::from_millis(tracking.pilot_refresh_ms),
            state: Mutex::new(State::default()),
        }
    }

    /// The box a station asked for: what the ground keeps sending to the vehicle.
    pub fn tracking(&self) -> Option<NormRect> {
        self.lock().tracking
    }

    /// How often the box is sent while tracking.
    pub fn refresh_every(&self) -> Duration {
        self.refresh_every
    }

    /// Where the vehicle reports the tracked box now, or the box the station drew while it has
    /// not: the part of the picture a crop is asked for.
    pub fn target(&self, now: Instant) -> Option<NormRect> {
        let state = self.lock();
        let drawn = state.tracking?;
        let reported = state
            .applied
            .filter(|(_, at)| now.saturating_duration_since(*at) <= self.pilot_ttl);
        Some(reported.map_or(drawn, |(rect, _)| rect))
    }

    /// Region envelope `seq` carrying `sent` went to the vehicle: the first to carry the box asked
    /// for, if none did. Ignored when the pilot has drawn another box since `sent` was read.
    pub fn box_sent(&self, seq: u64, sent: Option<NormRect>) {
        let mut state = self.lock();
        if state.tracking == sent {
            state.box_from.get_or_insert(seq);
        }
    }

    /// The vehicle lost the box it followed, as of its region envelope `region_seq`: if that is the
    /// box still tracked, nothing is tracked any more, so the station's tracking status goes idle
    /// and nothing is sent to the vehicle until a new box is drawn. A message for an older box, or
    /// one that reaches a box not yet sent, changes nothing.
    pub fn lost(&self, region_seq: u64) {
        let mut state = self.lock();
        if !lost_is_for_box(state.box_from, region_seq) {
            return;
        }
        state.tracking = None;
        state.box_from = None;
        state.applied = None;
        state.detail = None;
    }

    /// What the vehicle reports it applies, as of `now`.
    pub fn applied(&self, region: Option<NormRect>, now: Instant) {
        self.lock().applied = region.map(|rect| (rect, now));
    }

    /// Notes that a crop of `rect` arrived at `now`. Ignored unless `rect` is the box being
    /// tracked: a crop fetched for a box the pilot has since replaced or cleared says nothing.
    pub fn detail_received(&self, rect: NormRect, now: Instant) {
        let mut state = self.lock();
        if state.tracking == Some(rect) {
            state.detail = Some((rect, now));
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
        let (Some((system, _)), Some((command, params))) = (heard, command(message)) else {
            state.counts.dropped += 1;
            return Vec::new();
        };
        let Some(requested) = requested(command, params) else {
            let result = self
                .change(&mut state, command, params)
                .unwrap_or(MavResult::MAV_RESULT_UNSUPPORTED);
            state.counts.unsupported += u64::from(result == MavResult::MAV_RESULT_UNSUPPORTED);
            return state
                .bytes(system, &ack(command, result))
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

    /// Runs a command that changes the camera's tracking or what it streams; `None` for any other.
    fn change(&self, state: &mut State, command: MavCmd, params: Params) -> Option<MavResult> {
        let wanted = match command {
            MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE => {
                let [x0, y0, x1, y1] = params;
                NormRect::new(x0, y0, x1, y1)
            }
            MavCmd::MAV_CMD_CAMERA_TRACK_POINT => {
                let [x, y, radius, _] = params;
                NormRect::around(x, y, radius, self.aspect())
            }
            MavCmd::MAV_CMD_CAMERA_STOP_TRACKING => {
                state.tracking = None;
                state.box_from = None;
                state.detail = None;
                return Some(MavResult::MAV_RESULT_ACCEPTED);
            }
            MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL if is_tracking_status(params[0]) => {
                state.status_every = status_interval(params[1]);
                return Some(MavResult::MAV_RESULT_ACCEPTED);
            }
            _ => return None,
        };
        let Some(rect) = wanted else {
            state.counts.tracking_denied += 1;
            return Some(MavResult::MAV_RESULT_DENIED);
        };
        if state.tracking != Some(rect) {
            state.detail = None;
            state.box_from = None;
        }
        state.tracking = Some(rect);
        Some(MavResult::MAV_RESULT_ACCEPTED)
    }

    /// The stream's width over its height.
    fn aspect(&self) -> f32 {
        f32::from(self.stream.width) / f32::from(self.stream.height.max(1))
    }

    /// The tracking status for a station that asked for it, once per interval it asked for, while
    /// the vehicle is heard.
    pub(super) fn status_frame(&self, now: Instant) -> Option<Vec<u8>> {
        let mut state = self.lock();
        let every = state.status_every?;
        let (system, heard_at) = state.heard?;
        let heard = now.saturating_duration_since(heard_at) <= HEARD_FOR;
        let due = state
            .status_sent
            .is_none_or(|sent| now.saturating_duration_since(sent) >= every);
        if !(heard && due) {
            return None;
        }
        state.status_sent = Some(now);
        let served = served_box(state.applied, self.pilot_ttl, state.detail, now);
        let status = tracking_status(served);
        state.bytes(system, &status)
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
                flags: CameraCapFlags::CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
                    | CameraCapFlags::CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE
                    | CameraCapFlags::CAMERA_CAP_FLAGS_HAS_TRACKING_POINT,
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

/// A command's first four parameters.
type Params = [f32; 4];

/// A command's id and first four parameters.
fn command(message: &MavMessage) -> Option<(MavCmd, Params)> {
    match message {
        MavMessage::COMMAND_LONG(c) => Some((c.command, [c.param1, c.param2, c.param3, c.param4])),
        MavMessage::COMMAND_INT(c) => Some((c.command, [c.param1, c.param2, c.param3, c.param4])),
        _ => None,
    }
}

/// Whether a lost message naming region envelope `region_seq` is for the box tracked, which first
/// went out in `box_from`: a box sent after that envelope, or not sent yet, is never the one lost.
fn lost_is_for_box(box_from: Option<u64>, region_seq: u64) -> bool {
    box_from.is_some_and(|from| region_seq >= from)
}

fn is_tracking_status(message_id: f32) -> bool {
    (message_id - TRACKING_STATUS_ID).abs() < 0.5
}

/// The interval a station asked for in microseconds: none to stop (-1), the default for 0 or
/// anything unreadable, never faster than 50 ms.
fn status_interval(micros: f32) -> Option<Duration> {
    if micros < 0.0 {
        return None;
    }
    let asked = Duration::try_from_secs_f32(micros / 1_000_000.0).ok();
    match asked {
        Some(every) if !every.is_zero() => Some(every.max(MIN_STATUS_EVERY)),
        _ => Some(DEFAULT_STATUS_EVERY),
    }
}

/// The box the tracking status reports: the one the vehicle said it applies within `ttl` of
/// `now`, or else the one a crop arrived for within `DETAIL_TTL`. `None` is idle, so a lost link
/// or a stalled retrieval takes the pilot's red box away.
fn served_box(
    applied: Option<(NormRect, Instant)>,
    ttl: Duration,
    detail: Option<(NormRect, Instant)>,
    now: Instant,
) -> Option<NormRect> {
    let within = |ttl: Duration| {
        move |(_, at): &(NormRect, Instant)| now.saturating_duration_since(*at) <= ttl
    };
    applied
        .filter(within(ttl))
        .or_else(|| detail.filter(within(DETAIL_TTL)))
        .map(|(rect, _)| rect)
}

/// `CAMERA_TRACKING_IMAGE_STATUS`: active with `served` box, idle without one.
fn tracking_status(served: Option<NormRect>) -> MavMessage {
    let Some(rect) = served else {
        return MavMessage::CAMERA_TRACKING_IMAGE_STATUS(CAMERA_TRACKING_IMAGE_STATUS_DATA {
            tracking_status: CameraTrackingStatusFlags::CAMERA_TRACKING_STATUS_FLAGS_IDLE,
            tracking_mode: CameraTrackingMode::CAMERA_TRACKING_MODE_NONE,
            ..CAMERA_TRACKING_IMAGE_STATUS_DATA::default()
        });
    };
    MavMessage::CAMERA_TRACKING_IMAGE_STATUS(CAMERA_TRACKING_IMAGE_STATUS_DATA {
        tracking_status: CameraTrackingStatusFlags::CAMERA_TRACKING_STATUS_FLAGS_ACTIVE,
        tracking_mode: CameraTrackingMode::CAMERA_TRACKING_MODE_RECTANGLE,
        rec_top_x: rect.x0,
        rec_top_y: rect.y0,
        rec_bottom_x: rect.x1,
        rec_bottom_y: rect.y1,
        ..CAMERA_TRACKING_IMAGE_STATUS_DATA::default()
    })
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
fn requested(command: MavCmd, [param1, param2, ..]: Params) -> Option<Requested> {
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
    MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
        command,
        result,
        ..COMMAND_ACK_DATA::default()
    })
}

/// `text` as a NUL-padded 32-byte field.
fn name(text: &str) -> [u8; 32] {
    let mut field = [0; 32];
    for (slot, byte) in field.iter_mut().zip(text.bytes()) {
        *slot = byte;
    }
    field
}

#[cfg(test)]
mod tests {
    use mavlink::dialects::ardupilotmega::COMMAND_LONG_DATA;

    use super::*;
    use crate::frame::encoded;

    const TTL: Duration = Duration::from_millis(3000);

    fn camera() -> Camera {
        let stream = CameraStream {
            width: 1280,
            height: 720,
            fps: 15,
            bitrate_kbit: 400,
            uri: "rtp://127.0.0.1:5600".to_owned(),
        };
        let tracking = TrackingLimits {
            pilot_refresh_ms: 500,
            pilot_ttl_ms: 3000,
        };
        let camera = Camera::new(stream, tracking);
        camera.follow(&encoded(&MavMessage::HEARTBEAT(HEARTBEAT_DATA::default())));
        camera
    }

    fn command_long(command: MavCmd, params: Params) -> MavMessage {
        let [param1, param2, param3, param4] = params;
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command,
            param1,
            param2,
            param3,
            param4,
            target_component: CAMERA,
            ..COMMAND_LONG_DATA::default()
        })
    }

    fn decoded(bytes: &[u8]) -> MavMessage {
        crate::frame::datagram_frames(bytes)
            .next()
            .unwrap()
            .decode()
            .unwrap()
    }

    /// The result the camera acked the command with.
    fn result(camera: &Camera, command: MavCmd, params: Params) -> MavResult {
        let answers = camera.answer(&command_long(command, params));
        match answers.first().map(|bytes| decoded(bytes)) {
            Some(MavMessage::COMMAND_ACK(ack)) => ack.result,
            other => panic!("no ack: {other:?}"),
        }
    }

    fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> NormRect {
        NormRect { x0, y0, x1, y1 }
    }

    #[test]
    fn information_advertises_tracking() {
        let camera = camera();
        let message = camera.message(Requested::Information);
        let MavMessage::CAMERA_INFORMATION(info) = message else {
            panic!("not camera information");
        };
        let wanted = CameraCapFlags::CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
            | CameraCapFlags::CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE
            | CameraCapFlags::CAMERA_CAP_FLAGS_HAS_TRACKING_POINT;
        assert!(info.flags.contains(wanted));
    }

    #[test]
    fn tracking_commands_set_and_clear_the_box() {
        let camera = camera();
        let accepted = MavResult::MAV_RESULT_ACCEPTED;
        let rectangle = MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE;
        // QGroundControl 5.1.5 sends top-left x, y and bottom-right x, y; either corner order works.
        assert_eq!(result(&camera, rectangle, [0.6, 0.7, 0.2, 0.1]), accepted);
        assert_eq!(camera.tracking(), Some(rect(0.2, 0.1, 0.6, 0.7)));

        let point = MavCmd::MAV_CMD_CAMERA_TRACK_POINT;
        assert_eq!(result(&camera, point, [0.5, 0.5, 0.1, 0.0]), accepted);
        assert_eq!(
            camera.tracking(),
            NormRect::around(0.5, 0.5, 0.1, 1280.0 / 720.0)
        );

        let stop = MavCmd::MAV_CMD_CAMERA_STOP_TRACKING;
        assert_eq!(result(&camera, stop, [0.0; 4]), accepted);
        assert_eq!(camera.tracking(), None);
    }

    #[test]
    fn degenerate_rectangle_keeps_the_box() {
        let camera = camera();
        let rectangle = MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE;
        result(&camera, rectangle, [0.2, 0.1, 0.6, 0.7]);
        let denied = result(&camera, rectangle, [0.5, 0.5, 0.5, 0.9]);
        assert_eq!(denied, MavResult::MAV_RESULT_DENIED);
        assert_eq!(camera.tracking(), Some(rect(0.2, 0.1, 0.6, 0.7)));
    }

    #[test]
    fn status_reports_what_the_vehicle_applies() {
        let camera = camera();
        let (_, now) = camera.lock().heard.expect("the vehicle was heard");
        let interval = MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL;
        let set = |micros| result(&camera, interval, [TRACKING_STATUS_ID, micros, 0.0, 0.0]);
        let status = |at| match decoded(&camera.status_frame(at).expect("a status is due")) {
            MavMessage::CAMERA_TRACKING_IMAGE_STATUS(status) => status,
            other => panic!("not a tracking status: {other:?}"),
        };
        assert_eq!(set(500_000.0), MavResult::MAV_RESULT_ACCEPTED);

        // A box asked for but not confirmed is not reported.
        result(
            &camera,
            MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE,
            [0.2, 0.1, 0.6, 0.7],
        );
        let idle = status(now);
        assert_eq!(
            idle.tracking_status,
            CameraTrackingStatusFlags::CAMERA_TRACKING_STATUS_FLAGS_IDLE
        );

        let applied = rect(0.2, 0.1, 0.6, 0.7);
        camera.applied(Some(applied), now);
        let active = status(now + Duration::from_millis(600));
        assert_eq!(
            active.tracking_status,
            CameraTrackingStatusFlags::CAMERA_TRACKING_STATUS_FLAGS_ACTIVE
        );
        assert_eq!(
            active.tracking_mode,
            CameraTrackingMode::CAMERA_TRACKING_MODE_RECTANGLE
        );
        assert_eq!(
            (
                active.rec_top_x,
                active.rec_top_y,
                active.rec_bottom_x,
                active.rec_bottom_y
            ),
            (0.2, 0.1, 0.6, 0.7)
        );

        camera.applied(None, now);
        let cleared = status(now + Duration::from_millis(1100));
        assert_eq!(
            cleared.tracking_status,
            CameraTrackingStatusFlags::CAMERA_TRACKING_STATUS_FLAGS_IDLE
        );

        // -1 stops the stream.
        set(-1.0);
        assert!(camera.status_frame(now + Duration::from_secs(5)).is_none());
    }

    #[test]
    fn status_is_served_by_the_vehicle_or_by_a_crop() {
        let now = Instant::now();
        let box_ = rect(0.2, 0.1, 0.6, 0.7);
        let at = |ms: u64| now.checked_sub(Duration::from_millis(ms)).unwrap();
        let served = |applied: Option<u64>, detail: Option<u64>| {
            served_box(
                applied.map(|ms| (box_, at(ms))),
                TTL,
                detail.map(|ms| (box_, at(ms))),
                now,
            )
            .is_some()
        };
        let cases = [
            ("nothing", None, None, false),
            ("confirmed", Some(3000), None, true),
            ("confirmation expired", Some(3001), None, false),
            ("crop alone", None, Some(29_000), true),
            ("crop expired", None, Some(30_001), false),
            ("old crop, fresh confirmation", Some(0), Some(60_000), true),
            (
                "expired confirmation, fresh crop",
                Some(5000),
                Some(1000),
                true,
            ),
        ];
        for (name, applied, detail, expected) in cases {
            assert_eq!(served(applied, detail), expected, "{name}");
        }
    }

    #[test]
    fn a_crop_counts_only_for_the_box_being_tracked() {
        let camera = camera();
        let (_, now) = camera.lock().heard.expect("the vehicle was heard");
        let rectangle = MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE;
        let served = |at: Instant| {
            let state = camera.lock();
            served_box(state.applied, TTL, state.detail, at)
        };
        result(&camera, rectangle, [0.2, 0.1, 0.6, 0.7]);
        camera.detail_received(rect(0.5, 0.5, 0.9, 0.9), now);
        assert_eq!(served(now), None, "a crop of another box");
        camera.detail_received(rect(0.2, 0.1, 0.6, 0.7), now);
        assert_eq!(served(now), Some(rect(0.2, 0.1, 0.6, 0.7)));

        result(&camera, rectangle, [0.3, 0.1, 0.6, 0.7]);
        assert_eq!(served(now), None, "a new box starts without a crop");

        camera.detail_received(rect(0.3, 0.1, 0.6, 0.7), now);
        result(&camera, MavCmd::MAV_CMD_CAMERA_STOP_TRACKING, [0.0; 4]);
        assert_eq!(served(now), None, "stopped");
    }

    #[test]
    fn a_lost_message_drops_only_the_box_it_was_for() {
        // (first envelope of the tracked box, envelope the lost message names, dropped)
        for (box_from, region_seq, dropped) in [
            (Some(3), 3, true),
            (Some(3), 5, true),
            (Some(6), 5, false),
            (None, 5, false),
        ] {
            assert_eq!(lost_is_for_box(box_from, region_seq), dropped);
        }
    }

    #[test]
    fn crops_follow_the_reported_box_and_a_lost_one_stops_the_tracking() {
        let camera = camera();
        let (_, now) = camera.lock().heard.expect("the vehicle was heard");
        let (drawn, now_at) = (rect(0.2, 0.1, 0.6, 0.7), rect(0.4, 0.1, 0.8, 0.7));
        result(
            &camera,
            MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE,
            [0.2, 0.1, 0.6, 0.7],
        );
        assert_eq!(camera.target(now), Some(drawn), "nothing reported yet");
        camera.applied(Some(now_at), now);
        assert_eq!(camera.target(now), Some(now_at));
        assert_eq!(
            camera.target(now + TTL + Duration::from_millis(1)),
            Some(drawn)
        );

        camera.detail_received(drawn, now);
        camera.box_sent(1, Some(drawn));
        camera.lost(1);
        assert_eq!(camera.target(now), None);
        let state = camera.lock();
        assert_eq!(served_box(state.applied, TTL, state.detail, now), None);
    }
}
