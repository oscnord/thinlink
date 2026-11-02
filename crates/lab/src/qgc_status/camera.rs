//! The adapter's camera component as QGC 5.1.5 finds it: `checks`, a ground station of the adapter,
//! waits for the camera's heartbeat and sends the requests QGC sends it. The adapter must answer
//! them itself, as the camera, and none may reach the vehicle.

use std::net::SocketAddr;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{COMMAND_LONG_DATA, MavCmd, MavMessage, MavResult};
use serde_json::{Value, json};

use crate::check::{self, Results, Verdict, increase};
use crate::client::AgentClient;
use crate::mavlink::gcs::{CAMERA_COMPONENT, Gcs, Kind, Seen};

/// SITL boots under qemu after `lab-up`, and the camera speaks only after the vehicle.
const HEARTBEAT_DEADLINE: Duration = Duration::from_secs(60);
const ANSWER_DEADLINE: Duration = Duration::from_secs(5);
/// The camera answers from the ground at once, but a request that leaked raw would reach the
/// vehicle only one downlink later (300 ms under `geo`): watch three times that.
const LEAK_WATCH: Duration = Duration::from_secs(1);
/// `REQUEST_MESSAGE` as QGC 5.1.5 asks the camera: `CAMERA_INFORMATION`,
/// `VIDEO_STREAM_INFORMATION` for all streams (0) and `VIDEO_STREAM_STATUS` of stream 1.
const REQUESTS: [(f32, f32); 3] = [(259.0, 0.0), (269.0, 0.0), (270.0, 1.0)];

/// What came back from the camera.
#[derive(Debug, Default)]
struct Answers {
    accepted: usize,
    other_acks: usize,
    video_stream: Option<bool>,
    /// RTP/UDP, thermal.
    information: Option<(bool, bool)>,
    /// Thermal.
    status: Option<bool>,
}

impl Answers {
    fn of(seen: &[Seen]) -> Self {
        let mut answers = Self::default();
        for s in seen.iter().filter(|s| s.from_camera) {
            match s.kind {
                Kind::Ack {
                    command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
                    result: MavResult::MAV_RESULT_ACCEPTED,
                } => answers.accepted += 1,
                Kind::Ack { .. } => answers.other_acks += 1,
                Kind::CameraInformation { video_stream } => {
                    answers.video_stream = Some(video_stream);
                }
                Kind::VideoStreamInformation { rtp_udp, thermal } => {
                    answers.information = Some((rtp_udp, thermal));
                }
                Kind::VideoStreamStatus { thermal } => answers.status = Some(thermal),
                _ => {}
            }
        }
        answers
    }

    fn complete(&self) -> bool {
        self.accepted == REQUESTS.len()
            && self.video_stream.is_some()
            && self.information.is_some()
            && self.status.is_some()
    }

    /// The stream announced is RTP/UDP with video, and nothing is flagged thermal.
    fn as_qgc_needs(&self) -> bool {
        self.complete()
            && self.other_acks == 0
            && self.video_stream == Some(true)
            && self.information == Some((true, false))
            && self.status == Some(false)
    }
}

/// Records `camera_answered`; returns the adapter's camera counts before and after the requests.
pub(super) fn check(
    results: &mut Results,
    listen: SocketAddr,
    vehicle: &AgentClient,
    ground: &AgentClient,
) -> Value {
    let name = "camera_answered";
    let mut gcs = match Gcs::bind(listen) {
        Ok(gcs) => gcs,
        Err(e) => {
            results.record(
                name,
                Verdict::Incomplete,
                format!("cannot bind {listen}: {e}"),
            );
            return Value::Null;
        }
    };
    let heard = gcs.pump_until(Instant::now() + HEARTBEAT_DEADLINE, |seen| {
        seen.iter()
            .any(|s| s.from_camera && matches!(s.kind, Kind::Heartbeat { .. }))
    });
    if !heard {
        let detail = format!(
            "no camera heartbeat within {} s (the camera speaks only while the vehicle is heard)",
            HEARTBEAT_DEADLINE.as_secs()
        );
        results.record(name, Verdict::Incomplete, detail);
        return Value::Null;
    }
    let vehicle_before = check::get_json(vehicle, "/metrics");
    let ground_before = check::get_json(ground, "/metrics");
    for (message, stream) in REQUESTS {
        gcs.send(&request(message, stream));
    }
    gcs.pump_until(Instant::now() + ANSWER_DEADLINE, |seen| {
        Answers::of(seen).complete()
    });
    gcs.pump(Instant::now() + LEAK_WATCH);
    let answers = Answers::of(&gcs.seen);
    let vehicle_after = check::get_json(vehicle, "/metrics");
    let reached_vehicle = increase(&vehicle_before, &vehicle_after, "/mavlink/gcs/messages_in");
    let ground_after = check::get_json(ground, "/metrics");
    let camera = |r: &Result<Value, String>| match r {
        Ok(v) => v.get("camera").cloned().unwrap_or(Value::Null),
        Err(e) => json!({ "error": e }),
    };
    let detail = format!(
        "{} requests to component {CAMERA_COMPONENT}: {} accepted, {} other acks, CAMERA_INFORMATION with a video stream {:?}, VIDEO_STREAM_INFORMATION (RTP/UDP, thermal) {:?}, VIDEO_STREAM_STATUS thermal {:?}; messages that reached the vehicle meanwhile: {reached_vehicle:?}",
        REQUESTS.len(),
        answers.accepted,
        answers.other_acks,
        answers.video_stream,
        answers.information,
        answers.status,
    );
    let verdict = match reached_vehicle {
        None => Verdict::Incomplete,
        Some(0) if answers.as_qgc_needs() => Verdict::Pass,
        Some(_) => Verdict::Fail,
    };
    results.record(name, verdict, detail);
    json!({ "before": camera(&ground_before), "after": camera(&ground_after) })
}

/// `REQUEST_MESSAGE` to the vehicle's camera component.
fn request(message: f32, stream: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
        param1: message,
        param2: stream,
        target_system: 1,
        target_component: CAMERA_COMPONENT,
        ..COMMAND_LONG_DATA::default()
    })
}
