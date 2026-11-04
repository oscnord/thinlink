//! `lab check sim-landing`: landing on a marker (`docs/drone.md#landing-on-a-marker`). The
//! world has a tag on the ground a few metres from the take-off point. `checks` is the ground station:
//! it takes the copter off, flies it to about 2 m from above the tag on `MANUAL_CONTROL`, lets it
//! hover and switches it to Land. What the run must show depends on what the vehicle reports of
//! itself: with the landing target switch on and the camera pointing down the copter must land on
//! the tag; with the switch off, or the camera not pointing down, it must land where it hovered, not on
//! the tag (and the pilot must have been told in the second case). Without a target the autopilot
//! pauses before it descends (`PLND_STRICT` 1), which the run records as `descent_begins_s`. The expectation is derived from
//! those two readings and recorded, so a copter that lands on the tag when it should not, or not when it should, fails either way.

use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::MavMessage;
use serde_json::Value;

use thinlink_policy::landing::{MAX_OFF_NADIR_DEG, MAX_TURN_DEG};

use super::{
    ARM_DEADLINE, ARM_EVERY, CLIMB_DEADLINE, GUIDED, LAND, LOITER, ON_GROUND_M, SETPOINT_EVERY,
    Settings, TAKEOFF_M, WATCH_INTERVAL_US, WATCHED, altitude, arm, parts, state, takeoff,
};
use crate::check::{self, Results, Sample, Verdict};
use crate::control::stop::accepted;
use crate::control::{Parts, force_disarm, round_trip, set_mode, stop, wait_for_telemetry};
use crate::mavlink::command;
use crate::mavlink::gcs::Kind;

use mavlink::dialects::ardupilotmega::{MANUAL_CONTROL_DATA, MavCmd};

/// The tag's centre east and north of the take-off point (m): the world's `landing_tag`.
const TAG_EAST_N: (f64, f64) = (6.0, -3.0);
/// The copter hovers this far from above the tag (m) before Land: east, north. About 2.1 m away, so
/// a landing where it hovers is far outside `LAND_WITHIN_M`.
const HOVER_OFFSET: (f64, f64) = (-1.5, 1.5);
/// Where a camera that does not point down hovers from the tag (m): east, north. It looks north and
/// 20 degrees down with a 114.6 by 82.4 degree view, so from 9 m it sees the ground from about 5 m
/// ahead; 7 m south of the tag puts the tag about 52 degrees below the horizon, in the lower part of
/// the picture, where the vehicle sees it and must withhold it.
const FORWARD_HOVER_OFFSET: (f64, f64) = (0.0, -7.0);
/// What counts as having landed on the tag: the copter's distance from its centre (m). The tag is
/// 1.2 m across; a copter that came down where it hovered is 2.1 m away.
const LAND_WITHIN_M: f64 = 1.0;
/// The angles the vehicle sent may differ from the geometry by this much (degrees).
const AIM_WITHIN_DEG: f64 = 3.0;
/// The last target sent must be no older than this at the end of the window (ms).
const AIM_MAX_AGE_MS: u64 = 500;
/// Hover is reached within this distance of the hover point (m) and held this long.
const HOVER_REACHED_M: f64 = 0.6;
const HOVER_HOLD: Duration = Duration::from_secs(5);
const HOVER_DEADLINE: Duration = Duration::from_secs(90);
/// Land, until the copter is on the ground and disarmed; a copter that is not lets the run end here.
const LAND_DEADLINE: Duration = Duration::from_secs(90);
/// A stick at this share of its range (of 1000) per metre of distance, at most `MAX_STICK`.
const STICK_PER_M: f64 = 120.0;
const MAX_STICK: f64 = 350.0;
/// The copter has begun to descend once it is this far below its hover (m).
const DESCENT_BEGUN_M: f32 = 0.5;
/// Seconds into Land after which the copter should be closing on the tag, not swinging.
const SETTLE_S: f64 = 10.0;
const POLL: Duration = Duration::from_millis(100);
const METRES_PER_DEGREE: f64 = 111_320.0;
const THROTTLE_MIDDLE: i16 = 500;

pub fn run(agent: &crate::client::AgentClient, profile_text: &str, settings: &Settings) -> Results {
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
        flown(&mut results, &mut parts);
    }
    stop::clean_up(&mut parts);
    round_trip(&mut parts.station, &force_disarm());
    results
}

/// Where the copter is, in metres east and north of `home` (latitude and longitude in 1e7 degrees).
fn offset_m(home: (i32, i32), here: (i32, i32)) -> (f64, f64) {
    let north = f64::from(here.0 - home.0) * 1e-7 * METRES_PER_DEGREE;
    let east = f64::from(here.1 - home.1)
        * 1e-7
        * METRES_PER_DEGREE
        * (f64::from(home.0) * 1e-7).to_radians().cos();
    (east, north)
}

fn latest_position(parts: &Parts) -> Option<(i32, i32)> {
    parts
        .observer
        .since(ago(Duration::from_secs(2)))
        .iter()
        .rev()
        .find_map(|s| match s.kind {
            Kind::Position { lat, lon, .. } if s.from_vehicle => Some((lat, lon)),
            _ => None,
        })
}

/// `back` before now, or now when the clock has not run that long.
fn ago(back: Duration) -> Instant {
    let now = Instant::now();
    now.checked_sub(back).unwrap_or(now)
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// `MANUAL_CONTROL`: pitch forward is north (the copter flies with its nose north), roll is east.
#[expect(
    clippy::cast_possible_truncation,
    reason = "clamped to MAX_STICK, far inside i16"
)]
fn sticks(east: f64, north: f64) -> MavMessage {
    let stick = |m: f64| (m * STICK_PER_M).clamp(-MAX_STICK, MAX_STICK).round() as i16;
    MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA {
        target: 1,
        x: stick(north),
        y: stick(east),
        z: THROTTLE_MIDDLE,
        ..MANUAL_CONTROL_DATA::default()
    })
}

fn flown(results: &mut Results, parts: &mut Parts) {
    let _ = parts.ground.post("/lease", "");
    for id in WATCHED {
        let interval = command(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, id, WATCH_INTERVAL_US);
        round_trip(&mut parts.station, &interval);
    }
    parts.station.stream(Some(sticks(0.0, 0.0)), SETPOINT_EVERY);
    parts.station.pump(Instant::now() + Duration::from_secs(1));
    let Some(home) = latest_position(parts) else {
        results.record("armed", Verdict::Incomplete, "no position report");
        return;
    };
    if !armed_and_airborne(results, parts) {
        return;
    }
    let tag = TAG_EAST_N;
    let facts = Facts::read(parts, &parts.vehicle);
    let offset = if facts.pointing_down() {
        HOVER_OFFSET
    } else {
        FORWARD_HOVER_OFFSET
    };
    let hover = (tag.0 + offset.0, tag.1 + offset.1);
    if !hovering(results, parts, home, hover) {
        return;
    }
    results
        .metrics
        .insert("camera_off_nadir_deg", f64::from(facts.off_nadir_deg));
    results
        .metrics
        .insert("camera_turn_deg", f64::from(facts.turn_deg));
    let before = check::get_json(&parts.vehicle, "/metrics");
    let from = Instant::now();
    // Let the vehicle's messages and the pilot's notice show before the switch to Land.
    parts.station.pump(from + Duration::from_secs(4));
    let during = check::get_json(&parts.vehicle, "/metrics");
    report_sending(results, &facts, &before, &during, parts);
    if facts.expect_on_tag() {
        aim_matches_geometry(results, parts, home, tag, &during);
    }
    land(results, parts, home, tag, &facts);
}

/// What decides the run: the vehicle's switch and where its camera points.
struct Facts {
    switch_on: bool,
    off_nadir_deg: f32,
    turn_deg: f32,
}

impl Facts {
    fn read(parts: &Parts, vehicle: &crate::client::AgentClient) -> Self {
        let metrics = check::get_json(vehicle, "/metrics").ok();
        let switch_on = metrics
            .as_ref()
            .and_then(|m| m.pointer("/mavlink/landing/enabled")?.as_bool())
            .unwrap_or(false);
        let (off_nadir_deg, turn_deg) = parts
            .observer
            .since(ago(Duration::from_secs(5)))
            .iter()
            .rev()
            .find_map(|s| match s.kind {
                Kind::Mount {
                    off_nadir_deg,
                    turn_deg,
                    ..
                } if s.from_vehicle => Some((off_nadir_deg, turn_deg)),
                _ => None,
            })
            .unwrap_or((f32::NAN, f32::NAN));
        Self {
            switch_on,
            off_nadir_deg,
            turn_deg,
        }
    }

    fn pointing_down(&self) -> bool {
        self.off_nadir_deg <= MAX_OFF_NADIR_DEG && self.turn_deg.abs() <= MAX_TURN_DEG
    }

    /// Whether the copter is expected to come down on the tag.
    fn expect_on_tag(&self) -> bool {
        self.switch_on && self.pointing_down()
    }
}

fn armed_and_airborne(results: &mut Results, parts: &mut Parts) -> bool {
    let guided = accepted(&round_trip(&mut parts.station, &set_mode(GUIDED)));
    let deadline = Instant::now() + ARM_DEADLINE;
    let mut armed = false;
    while guided && !armed && Instant::now() < deadline {
        armed = accepted(&round_trip(&mut parts.station, &arm()));
        if !armed {
            parts.station.pump(Instant::now() + ARM_EVERY);
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
    results.record("armed", Verdict::Pass, "armed in Guided under the lease");
    let from = Instant::now();
    let ack = accepted(&round_trip(&mut parts.station, &takeoff(TAKEOFF_M)));
    let mut alt = 0.0;
    while from.elapsed() < CLIMB_DEADLINE && alt < TAKEOFF_M - 1.0 {
        parts.station.pump(Instant::now() + POLL);
        alt = parts
            .observer
            .since(from)
            .iter()
            .rev()
            .find_map(altitude)
            .unwrap_or(alt);
    }
    let up = alt >= TAKEOFF_M - 1.0;
    let verdict = if up { Verdict::Pass } else { Verdict::Fail };
    results.record(
        "takeoff",
        verdict,
        format!("{alt:.1} m (takeoff accepted: {ack})"),
    );
    up
}

/// Loiter, then steer with the sticks to the hover point and hold it.
fn hovering(results: &mut Results, parts: &mut Parts, home: (i32, i32), hover: (f64, f64)) -> bool {
    accepted(&round_trip(&mut parts.station, &set_mode(LOITER)));
    let from = Instant::now();
    let mut held_since: Option<Instant> = None;
    while from.elapsed() < HOVER_DEADLINE {
        let Some(here) = latest_position(parts).map(|p| offset_m(home, p)) else {
            parts.station.pump(Instant::now() + POLL);
            continue;
        };
        let error = (hover.0 - here.0, hover.1 - here.1);
        parts
            .station
            .stream(Some(sticks(error.0, error.1)), SETPOINT_EVERY);
        parts.station.pump(Instant::now() + POLL);
        let near = distance(here, hover) <= HOVER_REACHED_M;
        held_since = if near {
            held_since.or(Some(Instant::now()))
        } else {
            None
        };
        if held_since.is_some_and(|at| at.elapsed() >= HOVER_HOLD) {
            parts.station.stream(Some(sticks(0.0, 0.0)), SETPOINT_EVERY);
            results.record(
                "hover_reached",
                Verdict::Pass,
                format!(
                    "held within {HOVER_REACHED_M} m of the hover point, {:.1} m from the tag",
                    distance(here, TAG_EAST_N)
                ),
            );
            return true;
        }
    }
    results.record(
        "hover_reached",
        Verdict::Incomplete,
        format!(
            "not at the hover point within {} s",
            HOVER_DEADLINE.as_secs()
        ),
    );
    false
}

/// The vehicle's counters over the hover, and what the pilot was told.
fn report_sending(
    results: &mut Results,
    facts: &Facts,
    before: &Result<Value, String>,
    after: &Result<Value, String>,
    parts: &Parts,
) {
    let sent = check::increase(before, after, "/mavlink/landing/sent").unwrap_or(0);
    // A reason's counter appears with its first sighting, so a missing start is zero.
    let withheld = |metrics: &Result<Value, String>, reason: &str| {
        metrics
            .as_ref()
            .ok()
            .and_then(|m| {
                m.pointer(&format!("/mavlink/landing/withheld/{reason}"))?
                    .as_u64()
            })
            .unwrap_or(0)
    };
    let not_down: u64 = ["not_down", "turned"]
        .into_iter()
        .map(|reason| withheld(after, reason).saturating_sub(withheld(before, reason)))
        .sum();
    results.metrics.insert(
        "target_messages_in_4s",
        f64::from(u32::try_from(sent).unwrap_or(u32::MAX)),
    );
    let said: Vec<String> = parts
        .observer
        .since(ago(Duration::from_secs(60)))
        .iter()
        .filter_map(|s| match s.kind {
            Kind::StatusText { text } if s.from_vehicle => text.to_str().ok().map(str::to_owned),
            _ => None,
        })
        .filter(|t| t.starts_with("Landing target"))
        .collect();
    let (verdict, detail) = if facts.expect_on_tag() {
        let v = if sent > 0 {
            Verdict::Pass
        } else {
            Verdict::Fail
        };
        (
            v,
            format!(
                "{sent} LANDING_TARGET messages in 4 s with the switch on, the camera {:.1} degrees from down and its picture turned {:.1} degrees",
                facts.off_nadir_deg, facts.turn_deg
            ),
        )
    } else if facts.switch_on {
        let told = said.iter().any(|t| t.contains("Landing target off"));
        let v = if sent == 0 && told && not_down > 0 {
            Verdict::Pass
        } else {
            Verdict::Fail
        };
        (
            v,
            format!(
                "camera {:.1} degrees from down, picture turned {:.1}: {sent} messages sent, {not_down} sightings withheld, the pilot was told {said:?}",
                facts.off_nadir_deg, facts.turn_deg
            ),
        )
    } else {
        let v = if sent == 0 {
            Verdict::Pass
        } else {
            Verdict::Fail
        };
        (v, format!("switch off: {sent} messages sent"))
    };
    results.record("target_sent_as_expected", verdict, detail);
}

/// What the target should be for a copter at `copter` (east, north of home) `height_m` above home,
/// heading `yaw` (radians, north 0, clockwise) with the marker at `tag` on the ground: the angles in
/// degrees of a picture whose top is the nose, `x` right of its centre and `y` below it (towards the
/// tail), and the line-of-sight distance.
fn expected_aim(copter: (f64, f64), tag: (f64, f64), height_m: f64, yaw: f64) -> (f64, f64, f64) {
    let (east, north) = (tag.0 - copter.0, tag.1 - copter.1);
    let forward = north * yaw.cos() + east * yaw.sin();
    let right = east * yaw.cos() - north * yaw.sin();
    (
        (right / height_m).atan().to_degrees(),
        (-forward / height_m).atan().to_degrees(),
        (forward * forward + right * right + height_m * height_m).sqrt(),
    )
}

/// The last target the vehicle sent against the angles the marker should have from where the
/// copter is, at the end of the hover window. The truth comes from the copter's own position,
/// height and heading and the world's tag, not from how `ArduPilot` flies, so it pins the axis
/// conventions on their own.
fn aim_matches_geometry(
    results: &mut Results,
    parts: &Parts,
    home: (i32, i32),
    tag: (f64, f64),
    metrics: &Result<Value, String>,
) {
    let recent = parts.observer.since(ago(Duration::from_secs(2)));
    let yaw = recent.iter().rev().find_map(|s| match s.kind {
        Kind::Attitude { yaw, .. } if s.from_vehicle => Some(f64::from(yaw)),
        _ => None,
    });
    let height = recent.iter().rev().find_map(altitude).map(f64::from);
    let copter = latest_position(parts).map(|p| offset_m(home, p));
    let sent = |key: &str| {
        metrics
            .as_ref()
            .ok()
            .and_then(|m| m.pointer(&format!("/mavlink/landing/last/{key}"))?.as_f64())
    };
    let (Some(yaw), Some(height), Some(copter)) = (yaw, height, copter) else {
        results.record(
            "aim_matches_geometry",
            Verdict::Incomplete,
            "no heading, height or position report to compute the truth from",
        );
        return;
    };
    let (Some(x), Some(y), Some(turn), Some(age)) = (
        sent("angle_x_deg"),
        sent("angle_y_deg"),
        sent("turn_deg"),
        sent("age_ms"),
    ) else {
        results.record(
            "aim_matches_geometry",
            Verdict::Fail,
            "the vehicle reports no last target",
        );
        return;
    };
    let (want_x, want_y, want_distance) = expected_aim(copter, tag, height, yaw);
    let gap = (x - want_x).abs().max((y - want_y).abs());
    let fresh = age <= f64::from(u32::try_from(AIM_MAX_AGE_MS).unwrap_or(u32::MAX));
    results.metrics.insert("aim_error_deg", gap);
    results.metrics.insert("aim_turn_deg", turn);
    let verdict = if gap <= AIM_WITHIN_DEG && fresh {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record(
        "aim_matches_geometry",
        verdict,
        format!(
            "sent angle_x {x:.1}, angle_y {y:.1} degrees ({age:.0} ms old, picture turned {turn:.1}); geometry says angle_x {want_x:.1}, angle_y {want_y:.1}, distance {want_distance:.1} m (copter {:.1} m east {:.1} m north of home, {height:.1} m up, heading {:.1} degrees); within {AIM_WITHIN_DEG} degrees counts",
            copter.0,
            copter.1,
            yaw.to_degrees()
        ),
    );
}

/// Land, and judge where the copter ends up against the tag.
fn land(
    results: &mut Results,
    parts: &mut Parts,
    home: (i32, i32),
    tag: (f64, f64),
    facts: &Facts,
) {
    let hover_alt = parts
        .observer
        .since(ago(Duration::from_secs(2)))
        .iter()
        .rev()
        .find_map(altitude);
    let from = Instant::now();
    let asked = accepted(&round_trip(&mut parts.station, &set_mode(LAND)));
    let mut landed = false;
    let mut sent = Vec::new();
    while from.elapsed() < LAND_DEADLINE && !landed {
        parts.station.pump(Instant::now() + POLL);
        note_sent_target(&mut sent, &check::get_json(&parts.vehicle, "/metrics"));
        let seen = parts.observer.since(from);
        let down = seen
            .iter()
            .rev()
            .find_map(altitude)
            .is_some_and(|alt| alt <= ON_GROUND_M);
        landed = down
            && seen
                .iter()
                .rev()
                .find_map(state)
                .is_some_and(|(_, armed)| !armed);
    }
    let here = latest_position(parts).map_or((f64::NAN, f64::NAN), |p| offset_m(home, p));
    let alt = parts
        .observer
        .since(from)
        .iter()
        .rev()
        .find_map(altitude)
        .unwrap_or(f32::NAN);
    let error = distance(here, tag);
    results.metrics.insert("landing_error_m", error);
    results.metrics.insert("final_altitude_m", f64::from(alt));
    if landed {
        results
            .metrics
            .insert("land_s", from.elapsed().as_secs_f64());
    }
    // The pause before the descent begins: with no tag in view ArduPilot holds first (docs/real-vehicle.md).
    let started = hover_alt.and_then(|hover| {
        let sunk = parts
            .observer
            .since(from)
            .iter()
            .find(|s| altitude(s).is_some_and(|a| a < hover - DESCENT_BEGUN_M))
            .map(|s| s.at);
        sunk.map(|at| at.saturating_duration_since(from).as_secs_f64())
    });
    if let Some(seconds) = started {
        results.metrics.insert("descent_begins_s", seconds);
    }
    let distances = tag_distance_samples(parts, from, home, tag);
    if let Some(worst) = worst_after_settling(&distances) {
        results
            .metrics
            .insert("tag_distance_max_after_10s_m", worst);
    }
    results.samples.extend(distances);
    results
        .samples
        .extend(trace_samples(parts, from, home, &sent));
    let said = autopilot_said(parts, from);
    let outcome = format!(
        "the autopilot said {said:?}; Land accepted {asked}; after {:.0} s the copter is {error:.2} m from the tag centre at {alt:.1} m, landed {landed}, descent began after {started:?} s",
        from.elapsed().as_secs_f64()
    );
    let (name, ok) = if facts.expect_on_tag() {
        ("landed_on_tag", landed && error <= LAND_WITHIN_M)
    } else {
        ("landed_away_from_tag", landed && error > LAND_WITHIN_M)
    };
    let verdict = if ok { Verdict::Pass } else { Verdict::Fail };
    results.record(
        name,
        verdict,
        format!("{outcome} (within {LAND_WITHIN_M} m counts as on the tag)"),
    );
}

/// What the autopilot's `STATUSTEXT` said from `from` on (its precision landing speaks here).
fn autopilot_said(parts: &Parts, from: Instant) -> Vec<String> {
    parts
        .observer
        .since(from)
        .iter()
        .filter_map(|s| match s.kind {
            Kind::StatusText { text } if s.from_vehicle => text.to_str().ok().map(str::to_owned),
            _ => None,
        })
        .filter(|t| !t.starts_with("Landing target"))
        .collect()
}

/// The largest distance from the tag more than `SETTLE_S` after the first sample: a copter that
/// swings through the tag shows it here long after Land began.
fn worst_after_settling(distances: &[Sample]) -> Option<f64> {
    let first = distances.first()?.unix_s;
    distances
        .iter()
        .filter(|s| s.unix_s - first > SETTLE_S)
        .map(|s| s.value)
        .reduce(f64::max)
}

/// A target the vehicle sent: its angles (degrees), distance (m), sighting time (UNIX ms) and the
/// time from the sighting to the send (ms).
type SentTarget = (f64, f64, f64, u64, f64);

/// Keeps the vehicle's last target when it is a sighting not seen before.
fn note_sent_target(sent: &mut Vec<SentTarget>, metrics: &Result<Value, String>) {
    let last = |key: &str| {
        metrics
            .as_ref()
            .ok()
            .and_then(|m| m.pointer(&format!("/mavlink/landing/last/{key}"))?.as_f64())
    };
    let (Some(x), Some(y), Some(distance), Some(delay), Some(at)) = (
        last("angle_x_deg"),
        last("angle_y_deg"),
        last("distance_m"),
        last("to_send_ms"),
        metrics
            .as_ref()
            .ok()
            .and_then(|m| m.pointer("/mavlink/landing/last/sighted_unix_ms")?.as_u64()),
    ) else {
        return;
    };
    if sent.last().is_none_or(|(_, _, _, seen, _)| *seen != at) {
        sent.push((x, y, distance, at, delay));
    }
}

/// Everything the swing analysis aligns, as arrived from `from` on: each target sent (stamped with
/// its sighting), the copter's position and height, its attitude (degrees) and the mount's
/// quaternion.
fn trace_samples(
    parts: &Parts,
    from: Instant,
    home: (i32, i32),
    sent: &[SentTarget],
) -> Vec<Sample> {
    let mut rows: Vec<(u64, &'static str, f64)> = Vec::new();
    for (x, y, distance, at, delay) in sent {
        rows.push((*at, "sent_angle_x_deg", *x));
        rows.push((*at, "sent_angle_y_deg", *y));
        rows.push((*at, "sent_distance_m", *distance));
        rows.push((*at, "sent_delay_ms", *delay));
    }
    for s in parts.observer.since(from).iter().filter(|s| s.from_vehicle) {
        let at = s.unix_ms;
        match s.kind {
            Kind::Position {
                lat,
                lon,
                rel_alt_m,
                ..
            } => {
                let (east, north) = offset_m(home, (lat, lon));
                rows.push((at, "pos_east_m", east));
                rows.push((at, "pos_north_m", north));
                rows.push((at, "height_m", f64::from(rel_alt_m)));
            }
            Kind::Attitude { roll, pitch, yaw } => {
                rows.push((at, "roll_deg", f64::from(roll.to_degrees())));
                rows.push((at, "pitch_deg", f64::from(pitch.to_degrees())));
                rows.push((at, "yaw_deg", f64::from(yaw.to_degrees())));
            }
            Kind::Mount {
                q: [w, x, y, z], ..
            } => {
                rows.push((at, "mount_qw", f64::from(w)));
                rows.push((at, "mount_qx", f64::from(x)));
                rows.push((at, "mount_qy", f64::from(y)));
                rows.push((at, "mount_qz", f64::from(z)));
            }
            _ => {}
        }
    }
    let mut seqs: std::collections::BTreeMap<&'static str, u32> = std::collections::BTreeMap::new();
    rows.into_iter()
        .map(|(unix_ms, series, value)| {
            let seq = seqs.entry(series).or_default();
            *seq += 1;
            Sample {
                unix_s: crate::check::float(unix_ms) / 1000.0,
                series,
                seq: *seq,
                value,
            }
        })
        .collect()
}

/// The copter's distance from the tag (m) at each position report from `from` on.
fn tag_distance_samples(
    parts: &Parts,
    from: Instant,
    home: (i32, i32),
    tag: (f64, f64),
) -> Vec<Sample> {
    parts
        .observer
        .since(from)
        .iter()
        .filter_map(|s| match s.kind {
            Kind::Position { lat, lon, .. } if s.from_vehicle => Some((s.unix_ms, lat, lon)),
            _ => None,
        })
        .zip(0_u32..)
        .map(|((unix_ms, lat, lon), seq)| Sample {
            unix_s: crate::check::float(unix_ms) / 1000.0,
            series: "tag_distance_m",
            seq,
            value: distance(offset_m(home, (lat, lon)), tag),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Seen from a copter heading east, a marker north of it is to the left (negative x) and one
    /// east of it is straight ahead, which is above the picture's centre (negative y).
    #[test]
    fn the_expected_angles_follow_the_heading() {
        let east = std::f64::consts::FRAC_PI_2;
        let (x, y, d) = expected_aim((0.0, 0.0), (0.0, 10.0), 10.0, east);
        assert!((x + 45.0).abs() < 1e-9 && y.abs() < 1e-9, "{x} {y}");
        assert!((d - 200.0_f64.sqrt()).abs() < 1e-9);
        let (x, y, _) = expected_aim((0.0, 0.0), (10.0, 0.0), 10.0, east);
        assert!(x.abs() < 1e-9 && (y + 45.0).abs() < 1e-9, "{x} {y}");
        // Heading north, a marker behind the copter is below the centre.
        let (_, y, _) = expected_aim((0.0, 5.0), (0.0, 0.0), 5.0, 0.0);
        assert!((y - 45.0).abs() < 1e-9);
    }

    #[test]
    fn east_and_north_follow_the_degrees() {
        let home = (-353_632_620, 1_491_652_370);
        let (east, north) = offset_m(home, (home.0 + 1_000, home.1));
        assert!(
            east.abs() < 1e-9 && (north - 11.132).abs() < 1e-6,
            "{east} {north}"
        );
        let (east, north) = offset_m(home, (home.0, home.1 + 1_000));
        assert!(north.abs() < 1e-9 && east > 0.0 && east < 11.132);
    }
}
