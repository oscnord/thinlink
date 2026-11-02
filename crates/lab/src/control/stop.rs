//! Stopping: a stop command amid a setpoint stream must run; when the ground side goes away, the
//! adapter killed while it streams `MANUAL_CONTROL` (SITL must stop driving within the deadline,
//! and the restarted adapter holds no lease), or a total blackout (nothing sent into it may run
//! afterwards, and control needs the lease again).

use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::MavResult;
use serde_json::Value;

use sat_teleop_control::envelope::{ControlLimits, TICK};

use sat_teleop_control::ground::LeaseStatus;

use super::{
    Ack, MANUAL, Parts, Untouched, applied, armed, disarm, force_disarm, intruder, lease_status,
    round_trip, set_mode, take_lease, throttle, untouched_verdict, watch,
};
use crate::check::{self, Results, Sample, Verdict, float, percentile};
use crate::client::AgentClient;
use crate::mavlink::gcs::{Kind, Seen};

/// `MANUAL_CONTROL` throttle (z, -1000 to 1000; 200 is 20 % ahead, about 3.5 m/s in SITL) and
/// rate: `QGroundControl` sends a joystick's axes at 25 Hz by default.
const THROTTLE: i16 = 200;
const SETPOINT_EVERY: Duration = Duration::from_millis(40);
/// How often the vehicle's metrics are read while driving.
const SAMPLE_EVERY: Duration = Duration::from_millis(200);
/// Drive at least this long, and give up on moving after the deadline.
const DRIVE_FOR: Duration = Duration::from_secs(5);
const DRIVE_DEADLINE: Duration = Duration::from_secs(15);
/// Ground speeds above `MOVING` count as driving, at or below `STOPPED` as stopped (m/s).
const MOVING: f32 = 1.0;
const STOPPED: f32 = 0.2;
/// What `just lab-check` waits for on stdout to kill the adapter.
const KILL_REQUEST: &str = "lab: kill ground";
/// For the vehicle to stop getting setpoints after asking for the kill.
const KILL_DEADLINE: Duration = Duration::from_secs(15);
/// No setpoint applied for this long, while `checks` still sends them: the adapter is down.
const STREAM_GONE: Duration = Duration::from_secs(1);
/// After the vehicle stops getting setpoints, how long SITL is watched for zero throttle, Hold
/// and standstill.
const STOP_WATCH: Duration = Duration::from_secs(12);
/// For the restarted adapter to answer.
const RESTART_DEADLINE: Duration = Duration::from_secs(30);
/// `ArduRover`'s hold mode, as `custom_mode`.
const HOLD: u32 = 4;
/// After the restore, how long commands keep going and the vehicle is watched.
const OUTAGE_WATCH: Duration = Duration::from_secs(6);
/// How long setpoints stream before the stop command goes out amid them.
const STREAM_BEFORE_STOP: Duration = Duration::from_secs(2);

/// One applied setpoint, from the vehicle's metrics (UNIX ms).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Timing {
    sent: u64,
    received: u64,
    applied: u64,
}

/// Manual mode, arm and stream throttle through the adapter, then send `DO_SET_MODE` Hold amid the
/// stream, as an operator stopping would: SITL must accept it. Each kind of envelope counts its own
/// sequence numbers, so a setpoint that overtakes the command cannot get it refused. Leaves SITL
/// disarmed.
pub(super) fn stop_while_streaming(results: &mut Results, parts: &mut Parts) {
    let mode = round_trip(&mut parts.station, &set_mode(MANUAL));
    let arm = round_trip(&mut parts.station, &intruder::arm());
    parts
        .station
        .stream(Some(throttle(THROTTLE)), SETPOINT_EVERY);
    parts.station.pump(Instant::now() + STREAM_BEFORE_STOP);
    let before = check::get_json(&parts.vehicle, "/metrics");
    let hold = round_trip(&mut parts.station, &set_mode(HOLD));
    parts.station.pump(Instant::now() + SAMPLE_EVERY);
    let after = check::get_json(&parts.vehicle, "/metrics");
    parts.station.stream(None, SETPOINT_EVERY);
    round_trip(&mut parts.station, &force_disarm());
    let streamed = check::increase(&before, &after, "/mavlink/control/applied/continuous");
    if let Some((ms, _)) = hold.answer {
        results.metrics.insert("stop_command_ack_ms", ms);
    }
    let name = "stop_command_applied";
    let (verdict, detail) = if !accepted(&mode) || !accepted(&arm) {
        let detail = format!("could not set manual mode and arm: {mode:?}, {arm:?}");
        (Verdict::Incomplete, detail)
    } else if streamed.is_none_or(|n| n == 0) {
        let detail = format!("no setpoint applied around the stop command ({streamed:?})");
        (Verdict::Incomplete, detail)
    } else if accepted(&hold) {
        let detail = format!(
            "Hold accepted amid the setpoint stream ({} setpoints applied around it)",
            streamed.unwrap_or_default()
        );
        (Verdict::Pass, detail)
    } else {
        (
            Verdict::Fail,
            format!("Hold amid the stream got {:?}", hold.answer),
        )
    };
    results.record(name, verdict, detail);
}

/// Manual mode, arm, drive, then kill the adapter mid-stream.
pub(super) fn drive_and_kill(results: &mut Results, parts: &mut Parts, limits: ControlLimits) {
    let mode = round_trip(&mut parts.station, &set_mode(MANUAL));
    let arm = round_trip(&mut parts.station, &intruder::arm());
    if !accepted(&mode) || !accepted(&arm) {
        let detail = format!("could not set manual mode and arm: {mode:?}, {arm:?}");
        for name in [
            "setpoints_applied",
            "refresh_bounded",
            "stopped_after_kill",
            "restarted_without_lease",
        ] {
            results.record(name, Verdict::Incomplete, detail.clone());
        }
        return;
    }
    let (timings, top_speed) = drive(parts);
    let stop = kill(parts, limits, top_speed);
    record_setpoints(results, &timings, top_speed);
    record_stop(results, &stop);
    restarted_without_lease(results, parts, limits, &stop);
}

pub(super) fn accepted(ack: &Ack) -> bool {
    matches!(ack.answer, Some((_, MavResult::MAV_RESULT_ACCEPTED)))
}

/// Streams throttle through the adapter until SITL drives, sampling the setpoints the vehicle
/// applied; also the fastest ground speed meanwhile (m/s).
fn drive(parts: &mut Parts) -> (Vec<Timing>, f32) {
    parts
        .station
        .stream(Some(throttle(THROTTLE)), SETPOINT_EVERY);
    let started = Instant::now();
    let mut timings = Vec::new();
    loop {
        let now = Instant::now();
        let top = top_speed(&parts.observer.since(started));
        if (top > MOVING && now >= started + DRIVE_FOR) || now >= started + DRIVE_DEADLINE {
            return (timings, top);
        }
        parts.station.pump(now + SAMPLE_EVERY);
        let latest = last_setpoint(&parts.vehicle);
        if latest.is_some() && latest != timings.last().copied() {
            timings.extend(latest);
        }
    }
}

/// The adapter killed while `checks` keeps streaming, and what SITL did after.
#[derive(Debug, Clone)]
struct Stop {
    /// Fastest ground speed before the kill (m/s).
    top_speed: f32,
    /// The vehicle stopped getting setpoints while `checks` still sent them.
    adapter_down: bool,
    /// When the vehicle wrote its last setpoint from an envelope and its last refresh, and after
    /// that SITL's first zero throttle, first heartbeat in Hold and first speed at or below
    /// `STOPPED` (UNIX ms).
    last_setpoint: Option<u64>,
    last_refresh: Option<u64>,
    released: Option<u64>,
    hold: Option<u64>,
    stopped: Option<u64>,
    limits: ControlLimits,
    /// SITL's ground speed and throttle from the kill on.
    trace: Vec<Seen>,
}

impl Stop {
    /// Milliseconds from the last setpoint to `at`; zero for anything before it.
    fn after_last(&self, at: Option<u64>) -> Option<u64> {
        Some(at?.saturating_sub(self.last_setpoint?))
    }
}

fn kill(parts: &mut Parts, limits: ControlLimits, top_speed: f32) -> Stop {
    let asked = Instant::now();
    println!("{KILL_REQUEST}");
    let mut last = last_setpoint(&parts.vehicle);
    let mut changed = Instant::now();
    let mut adapter_down = false;
    while Instant::now() < asked + KILL_DEADLINE {
        parts.station.pump(Instant::now() + SAMPLE_EVERY / 2);
        let latest = last_setpoint(&parts.vehicle);
        if latest != last {
            (last, changed) = (latest, Instant::now());
        } else if changed.elapsed() >= STREAM_GONE {
            adapter_down = true;
            break;
        }
    }
    let last_setpoint = last.map(|t| t.applied);
    let first = |seen: &[Seen], kind: fn(Kind) -> bool| first_after(seen, last_setpoint, kind);
    let held: fn(Kind) -> bool = |kind| matches!(kind, Kind::Heartbeat { mode: HOLD, .. });
    let still: fn(Kind) -> bool = |kind| matches!(kind, Kind::Gps { speed } if speed <= STOPPED);
    let watch_until = Instant::now() + STOP_WATCH;
    while Instant::now() < watch_until {
        parts.station.pump(Instant::now() + SAMPLE_EVERY / 2);
        let seen = parts.observer.since(asked);
        if first(&seen, held).is_some() && first(&seen, still).is_some() {
            break;
        }
    }
    parts.station.stream(None, SETPOINT_EVERY);
    let seen = parts.observer.since(asked);
    let metrics = check::get_json(&parts.vehicle, "/metrics");
    Stop {
        top_speed,
        adapter_down,
        last_setpoint,
        last_refresh: metrics
            .ok()
            .and_then(|m| m.pointer("/mavlink/control/last_refresh_unix_ms")?.as_u64()),
        released: first(&seen, |kind| matches!(kind, Kind::Hud { throttle: 0 })),
        hold: first(&seen, held),
        stopped: first(&seen, still),
        limits,
        trace: seen
            .into_iter()
            .filter(|s| s.from_vehicle && matches!(s.kind, Kind::Gps { .. } | Kind::Hud { .. }))
            .collect(),
    }
}

/// SITL drove before the kill and the adapter went down; then SITL's throttle went to zero within
/// the deadline after the last setpoint and its GCS failsafe switched to Hold. How long the rover
/// then coasts is SITL's physics: recorded, not graded. No drive or no kill is incomplete.
fn stop_verdict(e: &Stop) -> (Verdict, String) {
    if !e.adapter_down {
        let detail =
            "the adapter kept sending setpoints: it was not killed (run through `just lab-check`)";
        return (Verdict::Incomplete, detail.to_owned());
    }
    if e.top_speed <= MOVING {
        let detail = format!(
            "SITL reached only {:.2} m/s, so stopping proves nothing",
            e.top_speed
        );
        return (Verdict::Incomplete, detail);
    }
    if e.last_setpoint.is_none() {
        let detail = "no setpoint time from the vehicle";
        return (Verdict::Incomplete, detail.to_owned());
    }
    let deadline = e.limits.throttle_release_deadline_ms;
    let watched = STOP_WATCH.as_secs();
    let Some(released) = e.after_last(e.released) else {
        let detail =
            format!("SITL's throttle was not zero within {watched} s of the last setpoint");
        return (Verdict::Fail, detail);
    };
    if released > deadline {
        let detail = format!(
            "SITL's throttle zero {released} ms after the last setpoint, deadline {deadline} ms"
        );
        return (Verdict::Fail, detail);
    }
    let Some(hold) = e.after_last(e.hold) else {
        let detail = format!("no Hold within {watched} s of the last setpoint");
        return (Verdict::Fail, detail);
    };
    let speed = e.after_last(e.stopped).map_or_else(
        || format!("not within {watched} s"),
        |ms| format!("after {ms} ms"),
    );
    let detail = format!(
        "throttle zero {released} ms after the last setpoint (deadline {deadline} ms), Hold after {hold} ms; from {:.1} m/s to at most {STOPPED} m/s {speed} (SITL coasting, not graded)",
        e.top_speed
    );
    (Verdict::Pass, detail)
}

/// The vehicle refreshes a setpoint at most for its lifetime: its last refresh comes no later than
/// the setpoint lifetime plus one tick after the last setpoint.
fn refresh_verdict(e: &Stop) -> (Verdict, String) {
    let Some(refreshed) = e.after_last(e.last_refresh) else {
        let detail = "no refresh or setpoint time from the vehicle";
        return (Verdict::Incomplete, detail.to_owned());
    };
    let bound = e.limits.setpoint_lifetime_ms + duration_ms(TICK);
    let verdict = if refreshed <= bound {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    let detail = format!(
        "last refresh {refreshed} ms after the last setpoint, at most {bound} ms (setpoint lifetime and one tick)"
    );
    (verdict, detail)
}

fn record_stop(results: &mut Results, stop: &Stop) {
    let metrics = [
        ("top_speed_m_s", Some(f64::from(stop.top_speed))),
        (
            "refresh_last_ms",
            stop.after_last(stop.last_refresh).map(float),
        ),
        (
            "throttle_released_ms",
            stop.after_last(stop.released).map(float),
        ),
        ("hold_ms", stop.after_last(stop.hold).map(float)),
        ("speed_zero_ms", stop.after_last(stop.stopped).map(float)),
    ];
    results.metrics.extend(
        metrics
            .into_iter()
            .filter_map(|(name, value)| Some((name, value?))),
    );
    let samples = stop.trace.iter().zip(0_u32..).filter_map(|(s, seq)| {
        let (series, value) = match s.kind {
            Kind::Gps { speed } => ("ground_speed", f64::from(speed)),
            Kind::Hud { throttle } => ("throttle", f64::from(throttle)),
            _ => return None,
        };
        Some(Sample {
            unix_s: float(s.unix_ms) / 1000.0,
            series,
            seq,
            value,
        })
    });
    results.samples.extend(samples);
    let (verdict, detail) = refresh_verdict(stop);
    results.record("refresh_bounded", verdict, detail);
    let (verdict, detail) = stop_verdict(stop);
    results.record("stopped_after_kill", verdict, detail);
}

fn duration_ms(duration: Duration) -> u64 {
    u64::try_from(duration.as_millis()).unwrap_or(u64::MAX)
}

fn record_setpoints(results: &mut Results, timings: &[Timing], top_speed: f32) {
    let mut delays: Vec<f64> = timings
        .iter()
        .map(|t| float(t.applied.saturating_sub(t.sent)))
        .collect();
    delays.sort_by(f64::total_cmp);
    let apply_max = timings
        .iter()
        .map(|t| t.applied.saturating_sub(t.received))
        .max();
    let metrics = [
        ("setpoint_delay_p50_ms", percentile(&delays, 50)),
        ("setpoint_delay_max_ms", delays.last().copied()),
        ("setpoint_apply_max_ms", apply_max.map(float)),
    ];
    results.metrics.extend(
        metrics
            .into_iter()
            .filter_map(|(name, value)| Some((name, value?))),
    );
    results.sample_counts.insert("setpoints", timings.len());
    results
        .samples
        .extend(timings.iter().zip(0_u32..).map(|(t, seq)| Sample {
            unix_s: float(t.sent) / 1000.0,
            series: "setpoint_delay",
            seq,
            value: float(t.applied.saturating_sub(t.sent)),
        }));
    let (verdict, detail) = if timings.is_empty() || top_speed <= MOVING {
        let detail = format!(
            "{} setpoints sampled, SITL at most {top_speed:.2} m/s",
            timings.len()
        );
        (Verdict::Incomplete, detail)
    } else {
        let detail = format!(
            "{} setpoints sampled through the adapter, SITL up to {top_speed:.1} m/s",
            timings.len()
        );
        (Verdict::Pass, detail)
    };
    results.record("setpoints_applied", verdict, detail);
}

/// The restarted adapter answers without a lease, and the vehicle has ended the old one.
fn restarted_without_lease(
    results: &mut Results,
    parts: &mut Parts,
    limits: ControlLimits,
    stop: &Stop,
) {
    let name = "restarted_without_lease";
    if !stop.adapter_down {
        results.record(name, Verdict::Incomplete, "the adapter was not killed");
        return;
    }
    let deadline = Instant::now() + RESTART_DEADLINE;
    let lease = loop {
        let lease = lease_status(&parts.ground);
        if lease.is_some() || Instant::now() >= deadline {
            break lease;
        }
        parts.station.pump(Instant::now() + SAMPLE_EVERY);
    };
    let lease_ends = Instant::now() + Duration::from_millis(limits.lease_lifetime_ms);
    parts.station.pump(lease_ends);
    let vehicle_lease = check::get_json(&parts.vehicle, "/metrics")
        .ok()
        .and_then(|m| m.pointer("/mavlink/control/lease").cloned());
    let (verdict, detail) = match (lease, &vehicle_lease) {
        (None, _) => (
            Verdict::Incomplete,
            format!(
                "the adapter did not answer within {} s",
                RESTART_DEADLINE.as_secs()
            ),
        ),
        (Some(LeaseStatus::None), Some(Value::Null)) => (
            Verdict::Pass,
            "the restarted adapter holds no lease and the vehicle ended the old one".to_owned(),
        ),
        (lease, vehicle) => (
            Verdict::Fail,
            format!("adapter lease {lease:?}, vehicle lease {vehicle:?}"),
        ),
    };
    results.record(name, verdict, detail);
}

/// The adapter holds the lease through a total blackout of both directions, `checks` sends a
/// forced arm and streams throttle into it, and after the restore nothing it sent may have run;
/// control then needs the lease again.
pub(super) fn outage(
    results: &mut Results,
    parts: &mut Parts,
    agent: &AgentClient,
    (blackout_ms, restore_ms): (u64, u64),
) {
    if let Err(e) = agent.post("/timeline/start", "") {
        let detail = format!("cannot start the timeline: {e}");
        for name in ["outage_nothing_applied", "reacquisition_required"] {
            results.record(name, Verdict::Incomplete, detail.clone());
        }
        return;
    }
    let t0 = Instant::now();
    let blackout = t0 + Duration::from_millis(blackout_ms);
    let restore = t0 + Duration::from_millis(restore_ms);
    let second = Duration::from_secs(1);
    parts
        .station
        .pump(blackout.checked_sub(second).unwrap_or(blackout));
    let before = check::get_json(&parts.vehicle, "/metrics");
    let lease_before = lease_status(&parts.ground);
    parts.station.pump(blackout + second);
    parts.station.send(&intruder::arm());
    parts
        .station
        .stream(Some(throttle(THROTTLE)), SETPOINT_EVERY);
    parts.station.pump(restore + OUTAGE_WATCH);
    parts.station.stream(None, SETPOINT_EVERY);
    let after = check::get_json(&parts.vehicle, "/metrics");
    let lease_after = lease_status(&parts.ground);
    let seen = parts.observer.since(t0);
    let heartbeats = |from: Instant, to: Instant| {
        seen.iter()
            .filter(|s| s.vehicle_heartbeat() && (from..to).contains(&s.at))
            .count()
    };
    let after_restore = heartbeats(restore, Instant::now());
    let since_blackout: Vec<Seen> = seen.iter().filter(|s| s.at >= blackout).copied().collect();
    let held = lease_before == Some(LeaseStatus::Held);
    let evidence = Untouched {
        answering: held && heartbeats(t0, blackout) > 0 && after_restore > 0,
        heartbeats: after_restore,
        armed: armed(&since_blackout),
        applied: applied(&before, &after),
        cases: Vec::new(),
    };
    if let Some(applied) = evidence.applied {
        results.metrics.insert("outage_applied", float(applied));
    }
    let (verdict, detail) = untouched_verdict(&evidence);
    results.record("outage_nothing_applied", verdict, detail);
    let reacquire = reacquire(parts, lease_before, lease_after);
    if let Some((ms, _)) = reacquire.arm.answer {
        results.metrics.insert("reacquired_arm_ack_ms", ms);
    }
    let (verdict, detail) = reacquire_verdict(&reacquire);
    results.record("reacquisition_required", verdict, detail);
}

/// After the blackout: what the adapter thought of its lease, an arm before and after taking the
/// lease again, and the disarm after.
#[derive(Debug)]
struct Reacquire {
    /// The adapter's lease just before the blackout and after it.
    lease_before: Option<LeaseStatus>,
    lease_after: Option<LeaseStatus>,
    before: Ack,
    retaken: Option<LeaseStatus>,
    arm: Ack,
    disarm: Ack,
}

fn reacquire(
    parts: &mut Parts,
    lease_before: Option<LeaseStatus>,
    lease_after: Option<LeaseStatus>,
) -> Reacquire {
    let before = round_trip(&mut parts.station, &intruder::arm());
    let retaken = take_lease(&parts.ground);
    let arm = round_trip(&mut parts.station, &intruder::arm());
    let disarm = round_trip(&mut parts.station, &disarm());
    Reacquire {
        lease_before,
        lease_after,
        before,
        retaken,
        arm,
        disarm,
    }
}

/// The adapter lost the lease in the blackout and denied an arm until it took the lease again;
/// then arming and disarming worked.
fn reacquire_verdict(e: &Reacquire) -> (Verdict, String) {
    let result = |ack: &Ack| ack.answer.map(|(_, r)| r);
    if e.lease_before != Some(LeaseStatus::Held) {
        let detail = format!(
            "the adapter's lease before the blackout was {:?}, so losing it proves nothing",
            e.lease_before
        );
        return (Verdict::Incomplete, detail);
    }
    if e.lease_after != Some(LeaseStatus::Lost) {
        let detail = format!(
            "the adapter's lease after the blackout was {:?}, not lost",
            e.lease_after
        );
        return (Verdict::Fail, detail);
    }
    if result(&e.before) != Some(MavResult::MAV_RESULT_DENIED) {
        let detail = format!(
            "an arm before taking the lease again got {:?}",
            e.before.answer
        );
        return (Verdict::Fail, detail);
    }
    if e.retaken != Some(LeaseStatus::Held) {
        let detail = format!(
            "the adapter could not take the lease again: {:?}",
            e.retaken
        );
        return (Verdict::Incomplete, detail);
    }
    let accepted = Some(MavResult::MAV_RESULT_ACCEPTED);
    if result(&e.arm) != accepted || result(&e.disarm) != accepted {
        let detail = format!(
            "after taking the lease again: arm {:?}, disarm {:?}",
            e.arm.answer, e.disarm.answer
        );
        return (Verdict::Fail, detail);
    }
    let detail = "the adapter held the lease before the blackout and lost it; an arm was denied until it took the lease again, then arm and disarm were accepted";
    (Verdict::Pass, detail.to_owned())
}

/// Leaves SITL disarmed at its default telemetry rates and the lease free, whoever held it and
/// however a run ended.
pub(super) fn clean_up(parts: &mut Parts) {
    let _ = parts.ground.post("/lease", "");
    let until = Instant::now() + RESTART_DEADLINE;
    let from = parts.station.seen.len();
    parts.station.pump_until(until, |seen| {
        seen.get(from..)
            .unwrap_or_default()
            .iter()
            .any(Seen::vehicle_heartbeat)
    });
    round_trip(&mut parts.station, &force_disarm());
    watch(&mut parts.station, 0.0);
    let _ = parts.ground.delete("/lease");
}

fn last_setpoint(vehicle: &AgentClient) -> Option<Timing> {
    let metrics = check::get_json(vehicle, "/metrics").ok()?;
    let timing = metrics.pointer("/mavlink/control/last_continuous")?;
    let field = |name: &str| timing.get(name)?.as_u64();
    Some(Timing {
        sent: field("sent")?,
        received: field("received")?,
        applied: field("applied")?,
    })
}

fn top_speed(seen: &[Seen]) -> f32 {
    seen.iter()
        .filter_map(|s| match s.kind {
            Kind::Gps { speed } if s.from_vehicle => Some(speed),
            _ => None,
        })
        .fold(0.0, f32::max)
}

/// When the first message matching `kind` arrived after `after` (UNIX ms).
fn first_after(seen: &[Seen], after: Option<u64>, kind: impl Fn(Kind) -> bool) -> Option<u64> {
    let after = after?;
    seen.iter()
        .find(|s| s.from_vehicle && s.unix_ms > after && kind(s.kind))
        .map(|s| s.unix_ms)
}
