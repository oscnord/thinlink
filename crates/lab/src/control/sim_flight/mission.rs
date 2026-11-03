//! `lab check sim-mission`: a mission flown out of reach (`docs/drone.md#missions`).
//! `checks` is a ground station that behaves as `QGroundControl` 5.1.5's Plan view does: it
//! uploads a takeoff, three waypoints and a landing, starts the mission and, once the copter has
//! reached the first waypoint, has the ground adapter killed. With `allow_missions` on, the copter
//! must fly the rest in Auto by the autopilot's own failsafe and land; with it off, the same upload,
//! the start and Auto must all be refused. The vehicle's metrics say which switch the run is.

use std::thread;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA, MavCmd, MavFrame, MavMessage, MavMissionResult,
    MavMissionType, MavResult,
};
use serde_json::Value;

use super::{
    ARM_DEADLINE, ARM_EVERY, GUIDED, KILL_REQUEST, ON_GROUND_M, SETPOINT_EVERY, Settings,
    WATCH_INTERVAL_US, WATCHED, altitude, arm, manual, parts, state,
};
use crate::check::{self, Results, Verdict, float};
use crate::client::AgentClient;
use crate::control::stop::accepted;
use crate::control::{
    Parts, await_no_lease, force_disarm, round_trip, set_mode, stop, wait_for_telemetry,
};
use crate::mavlink::command;
use crate::mavlink::gcs::{Gcs, Kind, Seen};

/// `ArduCopter`'s Auto mode (`Mode::Number::AUTO`).
const AUTO: u32 = 3;
/// `PlanManager::_ackTimeoutMilliseconds` and `_maxRetryCount` of `QGroundControl` 5.1.5: how long
/// the Plan view waits for the autopilot's next request, and how often it sends `MISSION_COUNT`
/// again before the first request.
const QGC_ACK_TIMEOUT: Duration = Duration::from_millis(1500);
const QGC_MAX_RETRIES: u32 = 5;
/// The mission: item 0 is the home placeholder `QGroundControl` sends for `ArduPilot`, 1 the takeoff,
/// 2 to 4 the waypoints, 5 the landing.
const TAKEOFF_M: f32 = 10.0;
const WAYPOINT_STEP_E7: i32 = 1800;
const FIRST_WAYPOINT: u16 = 2;
const LAST_WAYPOINT: u16 = 4;
const LAND_ITEM: u16 = 5;
/// The items of the mission below.
const MISSION_ITEMS: u16 = LAND_ITEM + 1;
/// The mission needs a takeoff and 20 m to the first waypoint.
const REACH_DEADLINE: Duration = Duration::from_secs(120);
/// From the cut: the rest of the legs and the landing.
const FINISH_WITHIN: Duration = Duration::from_secs(180);
const START_DEADLINE: Duration = Duration::from_secs(15);
const LEASE_GONE_WITHIN: Duration = Duration::from_secs(15);
const ANSWER_WITHIN: Duration = Duration::from_secs(5);
const POLL: Duration = Duration::from_millis(100);

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
    if wait_for_telemetry(&mut results, &mut parts) {
        let allowed = missions_allowed(&parts.vehicle);
        begin(&mut parts);
        match allowed {
            Some(true) => flown(&mut results, &mut parts),
            Some(false) => refused(&mut results, &mut parts),
            None => {
                let detail = "the vehicle's metrics do not say whether allow_missions is on";
                results.record("switch_known", Verdict::Incomplete, detail);
            }
        }
    }
    stop::clean_up(&mut parts);
    round_trip(&mut parts.station, &force_disarm());
    results
}

fn missions_allowed(vehicle: &AgentClient) -> Option<bool> {
    let metrics = check::get_json(vehicle, "/metrics").ok()?;
    metrics
        .pointer("/mavlink/control/allow_missions")?
        .as_bool()
}

/// The lease and the telemetry the check reads.
fn begin(parts: &mut Parts) {
    let _ = parts.ground.post("/lease", "");
    for id in WATCHED {
        let interval = command(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, id, WATCH_INTERVAL_US);
        round_trip(&mut parts.station, &interval);
    }
}

/// What the vehicle counted refused for `reason` (`mavlink.control.rejected`).
fn rejected(vehicle: &AgentClient, reason: &str) -> Option<u64> {
    let metrics = check::get_json(vehicle, "/metrics").ok()?;
    let counts = metrics.pointer("/mavlink/control/rejected")?;
    Some(counts.get(reason).and_then(Value::as_u64).unwrap_or(0))
}

/// The switch is off: the upload, the start and Auto are each answered denied, nothing is asked
/// of the autopilot, and the vehicle counts the refusals by reason.
fn refused(results: &mut Results, parts: &mut Parts) {
    let before = (
        rejected(&parts.vehicle, "mission_not_allowed"),
        rejected(&parts.vehicle, "mode_not_allowed"),
    );
    let from = parts.station.seen.len();
    parts.station.send(&mission_count());
    parts
        .station
        .pump_until(Instant::now() + ANSWER_WITHIN, |seen| {
            seen.iter().any(|s| mission_reply(s).is_some())
        });
    let replies: Vec<Reply> = parts
        .station
        .seen
        .get(from..)
        .unwrap_or_default()
        .iter()
        .filter_map(mission_reply)
        .collect();
    let upload = match replies.as_slice() {
        [Reply::Ack(MavMissionResult::MAV_MISSION_DENIED)] => {
            (Verdict::Pass, "answered MISSION_ACK denied")
        }
        [] => (Verdict::Incomplete, "no answer to MISSION_COUNT"),
        _ => (Verdict::Fail, "the upload was not simply denied"),
    };
    results.record("upload_refused", upload.0, upload.1);

    let start = round_trip(
        &mut parts.station,
        &command(MavCmd::MAV_CMD_MISSION_START, 0.0, 0.0),
    );
    let auto = round_trip(&mut parts.station, &set_mode(AUTO));
    let denied =
        |a: &crate::control::Ack| matches!(a.answer, Some((_, MavResult::MAV_RESULT_DENIED)));
    let after = (
        rejected(&parts.vehicle, "mission_not_allowed"),
        rejected(&parts.vehicle, "mode_not_allowed"),
    );
    let counted = after.0.zip(before.0).map(|(a, b)| a - b) == Some(2)
        && after.1.zip(before.1).map(|(a, b)| a - b) == Some(1);
    let detail = format!(
        "start denied {}, Auto denied {}; the vehicle counted mission_not_allowed {:?} to {:?}, mode_not_allowed {:?} to {:?}",
        denied(&start),
        denied(&auto),
        before.0,
        after.0,
        before.1,
        after.1
    );
    let verdict = if denied(&start) && denied(&auto) && counted {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record("start_and_auto_refused", verdict, detail);
}

/// The switch is on: upload, start, cut the link at the first waypoint, watch the copter finish.
fn flown(results: &mut Results, parts: &mut Parts) {
    let Some(home) = position(&parts.station) else {
        results.record(
            "mission_uploaded",
            Verdict::Incomplete,
            "no position report",
        );
        return;
    };
    if !upload(results, &mut parts.station, &mission(home)) || !armed_and_started(results, parts) {
        return;
    }
    let Some(cut) = first_waypoint_then_cut(results, parts) else {
        return;
    };
    finished(results, parts, cut);
}

fn position(station: &Gcs) -> Option<(i32, i32)> {
    station.seen.iter().rev().find_map(|s| match s.kind {
        Kind::Position { lat, lon, .. } if s.from_vehicle => Some((lat, lon)),
        _ => None,
    })
}

fn mission(home: (i32, i32)) -> Vec<MISSION_ITEM_INT_DATA> {
    let item = |seq: u16, command, (x, y), z| MISSION_ITEM_INT_DATA {
        seq,
        command,
        frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        current: u8::from(seq == 0),
        autocontinue: 1,
        x,
        y,
        z,
        target_system: 1,
        target_component: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        ..MISSION_ITEM_INT_DATA::default()
    };
    let north = |n: i32| (home.0 + n * WAYPOINT_STEP_E7, home.1);
    let waypoint = MavCmd::MAV_CMD_NAV_WAYPOINT;
    vec![
        item(0, waypoint, home, 0.0),
        item(1, MavCmd::MAV_CMD_NAV_TAKEOFF, (0, 0), TAKEOFF_M),
        item(FIRST_WAYPOINT, waypoint, north(1), TAKEOFF_M),
        item(3, waypoint, north(2), TAKEOFF_M),
        item(LAST_WAYPOINT, waypoint, north(3), TAKEOFF_M),
        item(LAND_ITEM, MavCmd::MAV_CMD_NAV_LAND, (0, 0), 0.0),
    ]
}

fn mission_count() -> MavMessage {
    MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
        count: MISSION_ITEMS,
        target_system: 1,
        target_component: 1,
        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        ..MISSION_COUNT_DATA::default()
    })
}

/// What the autopilot (or the adapter for it) said in a mission upload.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Reply {
    Request(u16),
    Ack(MavMissionResult),
}

fn mission_reply(s: &Seen) -> Option<Reply> {
    match s.kind {
        Kind::MissionRequest { seq } if s.from_vehicle => Some(Reply::Request(seq)),
        Kind::MissionAck { result } if s.from_vehicle => Some(Reply::Ack(result)),
        _ => None,
    }
}

/// The upload as `PlanManager` does it: `MISSION_COUNT`, then each item the autopilot requests,
/// waiting `QGC_ACK_TIMEOUT` for each next request and sending `MISSION_COUNT` again, up to
/// `QGC_MAX_RETRIES` times, only while no item has been requested. Records how long it took and
/// how many `MISSION_COUNT` it needed.
fn upload(results: &mut Results, station: &mut Gcs, items: &[MISSION_ITEM_INT_DATA]) -> bool {
    let started = Instant::now();
    let first = station.seen.len();
    let (mut counts, mut sent_items, mut requested) = (1_u32, 0_u32, false);
    station.send(&mission_count());
    let outcome = loop {
        let from = station.seen.len();
        let deadline = Instant::now() + QGC_ACK_TIMEOUT;
        station.pump_until(deadline, |seen| {
            seen.iter().any(|s| mission_reply(s).is_some())
        });
        let reply = station
            .seen
            .get(from..)
            .unwrap_or_default()
            .iter()
            .find_map(mission_reply);
        match reply {
            Some(Reply::Ack(result)) => break Some(result),
            Some(Reply::Request(seq)) => {
                requested = true;
                let item = items.iter().find(|i| i.seq == seq);
                let sent =
                    item.is_some_and(|i| station.send(&MavMessage::MISSION_ITEM_INT(i.clone())));
                sent_items += u32::from(sent);
            }
            None if !requested && counts <= QGC_MAX_RETRIES => {
                counts += 1;
                station.send(&mission_count());
            }
            None => break None,
        }
    };
    let upload_ms = started.elapsed().as_secs_f64() * 1000.0;
    results.metrics.insert("upload_ms", upload_ms);
    results
        .metrics
        .insert("upload_mission_counts", f64::from(counts));
    results
        .metrics
        .insert("upload_items_sent", f64::from(sent_items));
    let ok = outcome == Some(MavMissionResult::MAV_MISSION_ACCEPTED);
    let detail = format!(
        "{} items in {upload_ms:.0} ms with {counts} MISSION_COUNT and {sent_items} items sent, the autopilot's last word {outcome:?}",
        items.len()
    );
    let said: Vec<String> = station
        .seen
        .get(first..)
        .unwrap_or_default()
        .iter()
        .filter_map(|s| match s.kind {
            Kind::StatusText { text } if s.from_vehicle => text.to_str().ok().map(str::to_owned),
            _ => None,
        })
        .collect();
    let replies: Vec<Reply> = station
        .seen
        .get(first..)
        .unwrap_or_default()
        .iter()
        .filter_map(mission_reply)
        .collect();
    let detail = if ok {
        detail
    } else {
        format!("{detail}; replies {replies:?}; the autopilot said {said:?}")
    };
    let verdict = if ok { Verdict::Pass } else { Verdict::Fail };
    results.record("mission_uploaded", verdict, detail);
    ok
}

/// Guided, arm (retried while the pre-arm checks refuse), then `MAV_CMD_MISSION_START` as
/// `QGroundControl` sends it to a copter on the ground; the copter must report Auto.
fn armed_and_started(results: &mut Results, parts: &mut Parts) -> bool {
    parts.station.stream(Some(manual(0)), SETPOINT_EVERY);
    parts.station.pump(Instant::now() + Duration::from_secs(1));
    let guided = accepted(&round_trip(&mut parts.station, &set_mode(GUIDED)));
    let armed = guided && arm_when_ready(&mut parts.station);
    if !armed {
        let detail = format!(
            "no arm within {} s (Guided accepted: {guided})",
            ARM_DEADLINE.as_secs()
        );
        results.record("armed", Verdict::Incomplete, detail);
        return false;
    }
    let from = Instant::now();
    let start = command(MavCmd::MAV_CMD_MISSION_START, 0.0, 0.0);
    let ack = accepted(&round_trip(&mut parts.station, &start));
    let until = Instant::now() + START_DEADLINE;
    let mut auto = false;
    while !auto && Instant::now() < until {
        parts.station.pump(Instant::now() + POLL);
        auto = parts
            .observer
            .since(from)
            .iter()
            .any(|s| state(s).is_some_and(|(mode, _)| mode == AUTO));
    }
    let verdict = if ack && auto {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record(
        "mission_started",
        verdict,
        format!("MISSION_START accepted {ack}, copter in Auto {auto}"),
    );
    ack && auto
}

fn arm_when_ready(station: &mut Gcs) -> bool {
    let deadline = Instant::now() + ARM_DEADLINE;
    while Instant::now() < deadline {
        if accepted(&round_trip(station, &arm())) {
            return true;
        }
        station.pump(Instant::now() + ARM_EVERY);
    }
    false
}

/// The item numbers the copter reported reached from `from` on.
fn reached_since(parts: &Parts, from: Instant) -> Vec<u16> {
    parts
        .observer
        .since(from)
        .iter()
        .filter_map(|s| match s.kind {
            Kind::MissionReached { seq } if s.from_vehicle => Some(seq),
            _ => None,
        })
        .collect()
}

/// Waits for the first waypoint, then has the adapter killed (`just lab-check` does it on
/// request, as in `sim-flight`) and waits for the vehicle to lose its lease. The moment of the cut.
fn first_waypoint_then_cut(results: &mut Results, parts: &mut Parts) -> Option<Instant> {
    let from = Instant::now();
    while !reached_since(parts, from).contains(&FIRST_WAYPOINT) {
        if from.elapsed() >= REACH_DEADLINE {
            let detail = format!(
                "item {FIRST_WAYPOINT} not reached within {} s",
                REACH_DEADLINE.as_secs()
            );
            results.record("first_waypoint_reached", Verdict::Incomplete, detail);
            return None;
        }
        parts.station.pump(Instant::now() + POLL);
    }
    let seconds = from.elapsed().as_secs_f64();
    results.metrics.insert("first_waypoint_s", seconds);
    results.record(
        "first_waypoint_reached",
        Verdict::Pass,
        format!("item {FIRST_WAYPOINT} reached {seconds:.1} s after the start"),
    );
    parts.station.stream(None, SETPOINT_EVERY);
    let cut = Instant::now();
    println!("{KILL_REQUEST}");
    let gone = await_no_lease(&parts.vehicle, cut + LEASE_GONE_WITHIN);
    let detail = if gone {
        "the vehicle lost the lease: nothing reaches the copter"
    } else {
        "the vehicle still holds the lease: the adapter was not killed (run through `just lab-check`)"
    };
    let verdict = if gone {
        Verdict::Pass
    } else {
        Verdict::Incomplete
    };
    results.record("link_cut", verdict, detail);
    gone.then_some(cut)
}

/// The copter must report the rest of the waypoints reached, stay in Auto and be on the ground and
/// disarmed, with nothing from the ground after the cut.
fn finished(results: &mut Results, parts: &Parts, cut: Instant) {
    let landed = |parts: &Parts| {
        let seen = parts.observer.since(cut);
        let down = seen
            .iter()
            .rev()
            .find_map(altitude)
            .is_some_and(|alt| alt <= ON_GROUND_M);
        down && seen
            .iter()
            .rev()
            .find_map(state)
            .is_some_and(|(_, armed)| !armed)
    };
    while !landed(parts) && cut.elapsed() < FINISH_WITHIN {
        thread::sleep(POLL);
    }
    let seen = parts.observer.since(cut);
    let reached = reached_since(parts, cut);
    let modes: Vec<u32> = seen
        .iter()
        .filter_map(|s| state(s).map(|(mode, _)| mode))
        .fold(Vec::new(), |mut all, mode| {
            if !all.contains(&mode) {
                all.push(mode);
            }
            all
        });
    let said: Vec<String> = seen
        .iter()
        .filter_map(|s| match s.kind {
            Kind::StatusText { text } if s.from_vehicle => text.to_str().ok().map(str::to_owned),
            _ => None,
        })
        .collect();
    let on_ground = landed(parts);
    let seconds = cut.elapsed().as_secs_f64();
    let remaining = [FIRST_WAYPOINT + 1, LAST_WAYPOINT];
    let all_reached = remaining.iter().all(|seq| reached.contains(seq));
    let only_auto = modes.iter().all(|mode| *mode == AUTO);
    if on_ground {
        results.metrics.insert("finish_s", seconds);
    }
    results.metrics.insert(
        "waypoints_after_cut",
        float(u64::try_from(reached.len()).unwrap_or(u64::MAX)),
    );
    let detail = format!(
        "after the cut: items reached {reached:?}, modes {modes:?} (Auto is {AUTO}), on the ground and disarmed {on_ground} after {seconds:.0} s; the autopilot said {said:?}"
    );
    let verdict = if all_reached && only_auto && on_ground {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record("mission_finished_out_of_reach", verdict, detail);
}
