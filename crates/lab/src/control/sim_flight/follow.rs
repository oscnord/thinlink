//! `lab check follow`: the vehicle follows the person the pilot picked while the simulated copter
//! yaws (`docs/drone.md#regional-video`). Needs `just regional=1 sim-up`. `checks` is a
//! ground station: it takes the copter up to a low hover, picks the right-most person the
//! detector reports the way `QGroundControl` does (`CAMERA_TRACK_RECTANGLE`), and yaws the copter in
//! short turns, holding still between them so the detector's person and the followed box are the
//! same moment. At each hold the followed box must sit on that person, and the status the station
//! gets must be active. The turns go on until the person leaves the picture: the follow must end as
//! lost, the status go idle and no box be outlined.

use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    COMMAND_LONG_DATA, MANUAL_CONTROL_DATA, MavCmd, MavMessage,
};
use serde_json::Value;

use super::{
    ARM_DEADLINE, ARM_EVERY, GUIDED, LAND, LOITER, SETPOINT_EVERY, Settings, THROTTLE_MIDDLE,
    WATCH_INTERVAL_US, WATCHED, altitude, arm, parts,
};
use crate::check::{self, Results, Verdict};
use crate::client::AgentClient;
use crate::control::stop::accepted;
use crate::control::{Parts, round_trip, set_mode, stop, wait_for_telemetry};
use crate::mavlink::command;
use crate::mavlink::gcs::Kind;

/// The component of the ground adapter's camera.
const CAMERA_COMPONENT: u8 = 100;
const TRACKING_STATUS_ID: f32 = 275.0;
/// A low hover keeps the people in the camera's view.
const HOVER_M: f32 = 3.0;
const AIRBORNE_M: f32 = 2.5;
const CLIMB_DEADLINE: Duration = Duration::from_secs(60);
const SETTLE: Duration = Duration::from_secs(3);
const DETECTION_DEADLINE: Duration = Duration::from_secs(60);
const STATUS_DEADLINE: Duration = Duration::from_secs(20);
/// A turn: yaw stick (1000 is full) for this long, then still for the hold.
const YAW_STICK: i16 = 200;
const TURN: Duration = Duration::from_millis(500);
const HOLD: Duration = Duration::from_secs(2);
/// The follow is given this many turns to lose the person, which leaves the picture on a turn of
/// a few tens of degrees.
const MAX_TURNS: usize = 40;
const LAND_DEADLINE: Duration = Duration::from_secs(60);
/// The followed box's centre may differ from the person's by this much of the picture.
const MAX_OFF_X: f32 = 0.08;
const MAX_OFF_Y: f32 = 0.10;
/// The person must have moved at least this much of the picture's width across the holds, or the
/// run proved nothing about following through a turn.
const MIN_TRAVEL: f32 = 0.2;
const MIN_HOLDS: usize = 3;

type Corners = [f32; 4];

/// What the vehicle said at one hold: the followed box and the detector's people.
#[derive(Debug, Clone, PartialEq)]
struct Hold {
    following: bool,
    target: Option<Corners>,
    people: Vec<Corners>,
    last_lost: Option<String>,
    /// The last match's correlation.
    score: f64,
}

fn centre([x0, y0, x1, y1]: Corners) -> (f32, f32) {
    (f32::midpoint(x0, x1), f32::midpoint(y0, y1))
}

/// The person the pilot picked, and the one to compare with while it is in view: the right-most.
fn right_most(people: &[Corners]) -> Option<Corners> {
    people
        .iter()
        .copied()
        .max_by(|a, b| centre(*a).0.total_cmp(&centre(*b).0))
}

/// How far the followed box's centre is from the person's, as fractions of the picture.
fn off(target: Corners, person: Corners) -> (f32, f32) {
    let ((tx, ty), (px, py)) = (centre(target), centre(person));
    ((tx - px).abs(), (ty - py).abs())
}

fn corners(value: &Value) -> Option<Corners> {
    let values = value.as_array()?;
    let mut numbers = values.iter().filter_map(Value::as_f64);
    #[expect(clippy::cast_possible_truncation, reason = "fractions of the picture")]
    let mut next = || numbers.next().map(|v| v as f32);
    Some([next()?, next()?, next()?, next()?])
}

fn read_hold(vehicle: &AgentClient) -> Option<Hold> {
    let metrics = check::get_json(vehicle, "/metrics").ok()?;
    let people = metrics
        .pointer("/detector/boxes")?
        .as_array()?
        .iter()
        .filter_map(|b| {
            let [kind, rect] = b.as_array()?.as_slice() else {
                return None;
            };
            (kind.as_str()? == "person")
                .then_some(rect)
                .and_then(corners)
        })
        .collect();
    Some(Hold {
        following: metrics.pointer("/follow/state")?.as_str()? == "following",
        target: metrics.pointer("/follow/target").and_then(corners),
        people,
        last_lost: metrics
            .pointer("/follow/last_lost")
            .and_then(Value::as_str)
            .map(str::to_owned),
        score: metrics
            .pointer("/follow/score")
            .and_then(Value::as_f64)
            .unwrap_or(0.0),
    })
}

/// The grade of the holds in which the follow was on and the person in view: every one within
/// the limits, enough of them, and across enough of the picture.
fn holds_verdict(held: &[(Corners, Corners)]) -> (Verdict, String) {
    if held.len() < MIN_HOLDS {
        let detail = format!(
            "only {} holds with the box on and the person in view",
            held.len()
        );
        return (Verdict::Incomplete, detail);
    }
    let worst = held
        .iter()
        .fold((0.0_f32, 0.0_f32), |(wx, wy), (target, person)| {
            let (x, y) = off(*target, *person);
            (wx.max(x), wy.max(y))
        });
    let across = |(_, person): &(Corners, Corners)| centre(*person).0;
    let travel = held
        .first()
        .zip(held.last())
        .map_or(0.0, |(first, last)| (across(first) - across(last)).abs());
    let detail = format!(
        "{} holds, the box at most {:.3} across and {:.3} down from the person (limits {MAX_OFF_X} and {MAX_OFF_Y}), the person moved {travel:.2} of the width (at least {MIN_TRAVEL})",
        held.len(),
        worst.0,
        worst.1
    );
    let holds = worst.0 <= MAX_OFF_X && worst.1 <= MAX_OFF_Y && travel >= MIN_TRAVEL;
    (if holds { Verdict::Pass } else { Verdict::Fail }, detail)
}

fn manual_yaw(r: i16) -> MavMessage {
    MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA {
        target: 1,
        z: THROTTLE_MIDDLE,
        r,
        ..MANUAL_CONTROL_DATA::default()
    })
}

fn camera_command(command: MavCmd, params: [f32; 4]) -> MavMessage {
    let [p1, p2, p3, p4] = params;
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command,
        param1: p1,
        param2: p2,
        param3: p3,
        param4: p4,
        target_system: 1,
        target_component: CAMERA_COMPONENT,
        ..COMMAND_LONG_DATA::default()
    })
}

pub fn run(agent: &AgentClient, profile_text: &str, settings: &Settings) -> Results {
    let mut results = Results {
        settings: settings.to_json(),
        ..Results::default()
    };
    if !check::apply_profile(&mut results, agent, profile_text) {
        return results;
    }
    let mut parts = match parts(settings) {
        Ok(parts) => parts,
        Err(e) => {
            results.record("telemetry_received", Verdict::Incomplete, e.to_string());
            return results;
        }
    };
    if wait_for_telemetry(&mut results, &mut parts) && hover(&mut results, &mut parts) {
        follow(&mut results, &mut parts);
        land(&mut parts);
    }
    stop::clean_up(&mut parts);
    results
}

/// Holds the yaw stick at `yaw` with throttle at the middle for `duration`.
fn hold_stick(parts: &mut Parts, yaw: i16, duration: Duration) {
    parts.station.stream(Some(manual_yaw(yaw)), SETPOINT_EVERY);
    parts.station.pump(Instant::now() + duration);
}

fn command_accepted(parts: &mut Parts, message: &MavMessage) -> bool {
    accepted(&round_trip(&mut parts.station, message))
}

/// The lease, the telemetry, Guided, arm (retried while the pre-arm checks refuse), the takeoff to
/// `HOVER_M` and Loiter. False when it could not be done.
fn hover(results: &mut Results, parts: &mut Parts) -> bool {
    let _ = parts.ground.post("/lease", "");
    for id in WATCHED {
        let interval = command(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, id, WATCH_INTERVAL_US);
        round_trip(&mut parts.station, &interval);
    }
    hold_stick(parts, 0, Duration::from_secs(1));
    let guided = command_accepted(parts, &set_mode(GUIDED));
    let deadline = Instant::now() + ARM_DEADLINE;
    let mut armed = false;
    while guided && !armed && Instant::now() < deadline {
        armed = command_accepted(parts, &arm());
        if !armed {
            hold_stick(parts, 0, ARM_EVERY);
        }
    }
    if !armed {
        let detail = format!(
            "no arm within {} s (Guided accepted: {guided})",
            ARM_DEADLINE.as_secs()
        );
        results.record("armed", Verdict::Incomplete, detail);
        return false;
    }
    let from = Instant::now();
    command_accepted(parts, &takeoff());
    while from.elapsed() < CLIMB_DEADLINE && !airborne(parts, from) {
        hold_stick(parts, 0, Duration::from_millis(100));
    }
    let up = airborne(parts, from);
    let verdict = if up {
        Verdict::Pass
    } else {
        Verdict::Incomplete
    };
    results.record(
        "hovering",
        verdict,
        format!("{HOVER_M} m asked for, reached: {up}"),
    );
    command_accepted(parts, &set_mode(LOITER));
    hold_stick(parts, 0, SETTLE);
    up
}

fn takeoff() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        param7: HOVER_M,
        target_system: 1,
        target_component: 1,
        ..COMMAND_LONG_DATA::default()
    })
}

fn airborne(parts: &Parts, from: Instant) -> bool {
    parts
        .observer
        .since(from)
        .iter()
        .rev()
        .find_map(altitude)
        .is_some_and(|alt| alt >= AIRBORNE_M)
}

/// Whether the station's newest tracking status from the camera says `active`.
fn status_is(parts: &mut Parts, active: bool, within: Duration) -> bool {
    parts.station.pump_until(Instant::now() + within, |seen| {
        seen.iter().rev().find_map(|s| match s.kind {
            Kind::TrackingStatus { active: a, .. } if s.from_camera => Some(a == active),
            _ => None,
        }) == Some(true)
    })
}

/// The right-most person in view, waiting for the detector to report one.
fn pick(parts: &mut Parts) -> Option<Corners> {
    let deadline = Instant::now() + DETECTION_DEADLINE;
    while Instant::now() < deadline {
        parts
            .station
            .pump(Instant::now() + Duration::from_millis(500));
        let people = read_hold(&parts.vehicle)
            .map(|h| h.people)
            .unwrap_or_default();
        if let Some(person) = right_most(&people) {
            return Some(person);
        }
    }
    None
}

fn follow(results: &mut Results, parts: &mut Parts) {
    parts.station.send(&camera_command(
        MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        [TRACKING_STATUS_ID, 0.0, 0.0, 0.0],
    ));
    let Some(person) = pick(parts) else {
        results.record(
            "person_picked",
            Verdict::Incomplete,
            "the detector reported no person in view",
        );
        return;
    };
    parts.station.send(&camera_command(
        MavCmd::MAV_CMD_CAMERA_TRACK_RECTANGLE,
        person,
    ));
    let active = status_is(parts, true, STATUS_DEADLINE);
    let detail =
        format!("picked the right-most person, box {person:?}; the status went active: {active}");
    results.record(
        "person_picked",
        if active { Verdict::Pass } else { Verdict::Fail },
        detail,
    );
    if !active {
        return;
    }
    let mut held = Vec::new();
    let mut ended = None;
    for _ in 0..MAX_TURNS {
        hold_stick(parts, YAW_STICK, TURN);
        hold_stick(parts, 0, HOLD);
        let Some(hold) = read_hold(&parts.vehicle) else {
            continue;
        };
        if !hold.following {
            ended = Some(hold);
            break;
        }
        let compared = hold.target.zip(right_most(&hold.people));
        held.extend(compared);
    }
    let (verdict, detail) = holds_verdict(&held);
    results.record("box_stays_on_the_person", verdict, detail);
    lost(results, parts, ended);
}

/// The follow ended as lost, the station's status went idle and no box is outlined.
fn lost(results: &mut Results, parts: &mut Parts, ended: Option<Hold>) {
    let Some(ended) = ended else {
        let detail = format!("still following after {MAX_TURNS} turns");
        results.record("lost_when_it_leaves", Verdict::Fail, detail);
        return;
    };
    let reason = ended.last_lost.as_deref();
    let verdict = if reason.is_some() {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    let detail = format!(
        "the follow ended, lost for {reason:?} at a correlation of {:.2}, no box outlined: {}",
        ended.score,
        ended.target.is_none()
    );
    results.record("lost_when_it_leaves", verdict, detail);
    let idle = status_is(parts, false, STATUS_DEADLINE);
    let verdict = if idle { Verdict::Pass } else { Verdict::Fail };
    results.record(
        "status_goes_idle",
        verdict,
        "the station's tracking status went idle",
    );
}

/// Lands and waits for the copter to be down, so the next run starts on the ground.
fn land(parts: &mut Parts) {
    command_accepted(parts, &set_mode(LAND));
    let from = Instant::now();
    while from.elapsed() < LAND_DEADLINE {
        hold_stick(parts, 0, Duration::from_millis(500));
        let down = parts
            .observer
            .since(from)
            .iter()
            .rev()
            .find_map(altitude)
            .is_some_and(|alt| alt <= 0.5);
        if down {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PERSON: Corners = [0.60, 0.40, 0.70, 0.80];

    fn moved(dx: f32) -> Corners {
        [PERSON[0] + dx, PERSON[1], PERSON[2] + dx, PERSON[3]]
    }

    #[test]
    fn the_picked_person_is_the_right_most_one() {
        let left = [0.1, 0.4, 0.2, 0.8];
        assert_eq!(right_most(&[left, PERSON]), Some(PERSON));
        assert_eq!(right_most(&[]), None);
    }

    #[test]
    fn the_holds_pass_only_with_the_box_on_the_person_across_the_picture() {
        // (name, (box, person) pairs, expected)
        let on = |dx: f32, shift: f32| (moved(shift + dx), moved(shift));
        let cases = [
            (
                "on the person all the way",
                vec![on(0.0, 0.0), on(0.01, -0.1), on(-0.02, -0.2), on(0.0, -0.3)],
                Verdict::Pass,
            ),
            (
                "slid off the person",
                vec![on(0.0, 0.0), on(0.01, -0.1), on(0.2, -0.2), on(0.0, -0.3)],
                Verdict::Fail,
            ),
            (
                "the person never moved",
                vec![on(0.0, 0.0), on(0.0, -0.01), on(0.0, 0.0)],
                Verdict::Fail,
            ),
            ("too few holds", vec![on(0.0, 0.0)], Verdict::Incomplete),
        ];
        for (name, held, expected) in cases {
            assert_eq!(holds_verdict(&held).0, expected, "{name}");
        }
    }
}
