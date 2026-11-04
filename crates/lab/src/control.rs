//! `lab check control-freshness`: the control lease and command freshness end to end. `checks`
//! plays three parts: a second controller on the overlay that takes the lease and sends envelopes
//! the vehicle must refuse (`intruder`), a ground station of the adapter at 192.168.232.40:14552
//! that commands and drives the way `QGroundControl` does, and an observer of SITL's telemetry
//! straight from mavlink-router, which keeps watching while the adapter is down (`stop`).
//! `docs/testbed.md#control-freshness-check` defines every metric.

mod intruder;
mod real_mode;
mod sim_evidence;
pub mod sim_flight;
mod sim_probe;
mod stop;

use std::io;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{MANUAL_CONTROL_DATA, MavCmd, MavMessage, MavResult};
use serde::Deserialize;
use serde_json::{Value, json};

use thinlink_control::auth::VehicleId;
use thinlink_control::envelope::{ControlLimits, Refusal, ToGround};
use thinlink_control::ground::{LeaseStatus, Status};

use self::intruder::{Answer, Case, Intruder};
use crate::budget::{self, Budget};
use crate::check::{self, Results, Sample, Verdict, float, increase, percentile};
use crate::client::AgentClient;
use crate::mavlink::gcs::{Gcs, Kind, Seen};
use crate::mavlink::{FORCE_ARM_DISARM, command};
use crate::profile::{Action, Profile};

/// `ArduRover`'s manual mode, as `custom_mode`.
const MANUAL: u32 = 0;
/// `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`: `DO_SET_MODE` param 1 as `QGroundControl` sends it.
const CUSTOM_MODE_ENABLED: f32 = 1.0;
/// `VFR_HUD` (throttle) and `GPS_RAW_INT` (ground speed), asked for at 10 Hz while the check runs.
const WATCHED: [f32; 2] = [74.0, 24.0];
const WATCH_INTERVAL_US: f32 = 100_000.0;
/// SITL boots under qemu after `lab-up`; its first heartbeat can take a while.
const FIRST_TELEMETRY_DEADLINE: Duration = Duration::from_secs(60);
/// A GEO round trip and SITL's answer, with margin.
const ACK_TIMEOUT: Duration = Duration::from_secs(5);
/// Arm and disarm round trips through the adapter.
const ARM_ROUNDS: usize = 3;
/// How long the observer's thread reads between handing over what arrived.
const OBSERVE_SLICE: Duration = Duration::from_millis(50);
/// More than an hour of SITL telemetry; later messages are not kept.
const MAX_OBSERVED: usize = 500_000;
/// Reads what queued in a socket.
const DRAIN: Duration = Duration::from_millis(60);

#[derive(Debug, thiserror::Error)]
pub enum LimitsError {
    #[error(transparent)]
    Toml(#[from] toml::de::Error),
    #[error(transparent)]
    Invalid(#[from] thinlink_control::ControlError),
}

pub use self::real_mode::run as run_real_mode;

/// `[control]` of a limits file such as `testbed/limits.toml`, validated.
pub fn limits(text: &str) -> Result<ControlLimits, LimitsError> {
    #[derive(Deserialize)]
    struct File {
        control: ControlLimits,
    }
    let file: File = toml::from_str(text)?;
    Ok(file.control.validate()?)
}

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// Where the ground adapter sends `checks` its telemetry, as a ground station.
    pub gcs_listen: SocketAddr,
    /// Where mavlink-router sends `checks` SITL's telemetry, on the overlay.
    pub observer_listen: SocketAddr,
    /// The second controller's socket, on the overlay, and the key it signs with.
    pub controller: SocketAddr,
    pub controller_key: PathBuf,
    /// The vehicle's name, which its signatures are made for.
    pub vehicle_id: VehicleId,
    /// The vehicle's control endpoint.
    pub vehicle: SocketAddr,
    pub vehicle_metrics: String,
    /// The ground adapter's metrics and lease endpoints.
    pub ground_metrics: String,
    pub limits: ControlLimits,
}

impl Settings {
    fn to_json(&self) -> Value {
        let l = self.limits;
        json!({
            "gcs_listen": self.gcs_listen,
            "observer_listen": self.observer_listen,
            "controller": self.controller,
            "vehicle": self.vehicle,
            "vehicle_metrics": self.vehicle_metrics,
            "ground_metrics": self.ground_metrics,
            "challenge_lifetime_ms": l.challenge_lifetime_ms,
            "setpoint_lifetime_ms": l.setpoint_lifetime_ms,
            "lease_lifetime_ms": l.lease_lifetime_ms,
            "liveness_interval_ms": l.liveness_interval_ms,
            "ack_window_ms": l.ack_window_ms,
            "throttle_release_deadline_ms": l.throttle_release_deadline_ms,
            "arm_rounds": ARM_ROUNDS,
            "ack_timeout_s": ACK_TIMEOUT.as_secs_f64(),
        })
    }
}

/// The parts `checks` plays, and the endpoints it reads.
struct Parts {
    station: Gcs,
    observer: Observer,
    intruder: Intruder,
    vehicle: AgentClient,
    ground: AgentClient,
}

pub fn run(
    agent: &AgentClient,
    profile: &Profile,
    profile_text: &str,
    settings: &Settings,
    budget: Option<&Budget>,
) -> Results {
    let mut results = Results {
        thresholds: serde_json::to_value(budget).unwrap_or(Value::Null),
        settings: settings.to_json(),
        ..Results::default()
    };
    if !check::apply_profile(&mut results, agent, profile_text) {
        return results;
    }
    let mut parts = match start(settings) {
        Ok(parts) => parts,
        Err(e) => {
            results.record("telemetry_received", Verdict::Incomplete, e.to_string());
            return results;
        }
    };
    if !wait_for_telemetry(&mut results, &mut parts) {
        return results;
    }
    stop::clean_up(&mut parts);
    watch(&mut parts.station, WATCH_INTERVAL_US);
    let readings = intrude(&mut results, &mut parts, settings.limits);
    let acks = arm_rounds(&mut parts.station);
    record_acks(&mut results, &acks);
    resend(&mut results, &mut parts);
    stop::stop_while_streaming(&mut results, &mut parts);
    match blackout(profile) {
        Some(window) => stop::outage(&mut results, &mut parts, agent, window),
        None => stop::drive_and_kill(&mut results, &mut parts, settings.limits),
    }
    stop::clean_up(&mut parts);
    results.readings = readings;
    results.samples.extend(ack_samples(&acks));
    budget::grade(&mut results, budget);
    results
}

fn start(settings: &Settings) -> io::Result<Parts> {
    Ok(Parts {
        station: Gcs::bind(settings.gcs_listen)?,
        observer: Observer::start(settings.observer_listen)?,
        intruder: Intruder::bind(
            settings.controller,
            settings.vehicle,
            &settings.controller_key,
            &settings.vehicle_id,
        )?,
        vehicle: AgentClient::new(&settings.vehicle_metrics),
        ground: AgentClient::new(&settings.ground_metrics),
    })
}

/// A heartbeat through the adapter and one straight from mavlink-router: every later assertion
/// needs both. Without them the run stops here, incomplete.
fn wait_for_telemetry(results: &mut Results, parts: &mut Parts) -> bool {
    let started = Instant::now();
    let deadline = started + FIRST_TELEMETRY_DEADLINE;
    let through_adapter = parts
        .station
        .pump_until(deadline, |seen| seen.iter().any(Seen::vehicle_heartbeat));
    let observed = parts.observer.await_heartbeat(deadline);
    let name = "telemetry_received";
    if through_adapter && observed {
        let detail = format!(
            "vehicle heartbeats through the adapter and from mavlink-router after {} ms",
            started.elapsed().as_millis()
        );
        results.record(name, Verdict::Pass, detail);
        return true;
    }
    let detail = format!(
        "no vehicle heartbeat within {} s: through the adapter {through_adapter}, from mavlink-router {observed}",
        FIRST_TELEMETRY_DEADLINE.as_secs()
    );
    results.record(name, Verdict::Incomplete, detail);
    false
}

/// `checks` takes the lease and sends envelopes the vehicle must refuse; then the adapter takes the
/// lease and `checks` is refused it. Returns the vehicle metrics around the phase.
fn intrude(results: &mut Results, parts: &mut Parts, limits: ControlLimits) -> Value {
    let started = Instant::now();
    let before = check::get_json(&parts.vehicle, "/metrics");
    let epoch = before
        .as_ref()
        .ok()
        .and_then(|m| m.pointer("/mavlink/control/epoch")?.as_u64());
    let mut cases: Vec<Case> = epoch
        .and_then(|epoch| parts.intruder.unleased(epoch))
        .into_iter()
        .collect();
    let granted = matches!(parts.intruder.request(), Some(ToGround::LeaseGrant { .. }));
    let challenged = granted
        && parts
            .intruder
            .await_challenge(Instant::now() + ACK_TIMEOUT)
            .is_some();
    let probe = if challenged {
        parts.intruder.probe(limits)
    } else {
        intruder::Probe::default()
    };
    cases.extend(probe.cases.iter().cloned());
    parts.intruder.release();
    // While checks held the lease, everything the vehicle applied came from it.
    let held = check::get_json(&parts.vehicle, "/metrics");
    let handover = hand_over(parts);
    cases.extend(handover.former_holder.clone());
    let after = check::get_json(&parts.vehicle, "/metrics");
    let window: Vec<Seen> = parts.observer.since(started);
    let liveness = increase(&before, &held, "/mavlink/control/applied/liveness");
    let applied = applied(&before, &held)
        .and_then(|applied| applied.checked_sub(probe.continuous + probe.discrete_applied));
    let answering = challenged
        && probe.liveness > 0
        && liveness == Some(probe.liveness)
        && probe.discrete_applied == probe.discrete;
    let evidence = Untouched {
        answering,
        heartbeats: window.iter().filter(|s| s.vehicle_heartbeat()).count(),
        armed: armed(&window),
        applied,
        cases,
    };
    if let Some(applied) = applied {
        results.metrics.insert("invalid_applied", float(applied));
    }
    let (verdict, detail) = untouched_verdict(&evidence);
    results.record("invalid_refused", verdict, detail);
    let (verdict, detail) = one_lease_verdict(&handover, &after);
    results.record("one_lease", verdict, detail);
    let value = |r: &Result<Value, String>| r.clone().unwrap_or_else(|e| json!({ "error": e }));
    json!({
        "intruder_before": value(&before),
        "intruder_held": value(&held),
        "intruder_after": value(&after),
    })
}

/// What happened when the adapter took the lease `checks` had held.
#[derive(Debug, Default)]
struct Handover {
    /// The adapter's lease after `POST /lease`.
    ground: Option<LeaseStatus>,
    /// The vehicle's answer to `checks` asking again: granted, or denied for a reason.
    asked: Option<Result<(), Refusal>>,
    former_holder: Vec<Case>,
}

/// The adapter asks for the lease once the vehicle shows `checks`' release landed (a release has
/// no answer, and the adapter's request could overtake it); then `checks` asks again.
fn hand_over(parts: &mut Parts) -> Handover {
    await_no_lease(&parts.vehicle, Instant::now() + ACK_TIMEOUT);
    let ground = take_lease(&parts.ground);
    let asked = match parts.intruder.request() {
        Some(ToGround::LeaseDenied { reason, .. }) => Some(Err(reason)),
        Some(ToGround::LeaseGrant { .. }) => Some(Ok(())),
        _ => None,
    };
    Handover {
        ground,
        asked,
        former_holder: parts.intruder.after_handover(),
    }
}

/// `POST /lease` on the adapter: its lease afterwards, if it took it.
pub(crate) fn take_lease(ground: &AgentClient) -> Option<LeaseStatus> {
    let body = ground.post("/lease", "").ok()?;
    let status: Status = serde_json::from_str(&body).ok()?;
    Some(status.lease)
}

/// The adapter's lease (`GET /status`), if it answers.
fn lease_status(ground: &AgentClient) -> Option<LeaseStatus> {
    let body = ground.get("/status").ok()?;
    let status: Status = serde_json::from_str(&body).ok()?;
    Some(status.lease)
}

fn await_no_lease(vehicle: &AgentClient, until: Instant) -> bool {
    loop {
        let lease = check::get_json(vehicle, "/metrics")
            .ok()
            .and_then(|m| m.pointer("/mavlink/control/lease").cloned());
        if lease == Some(Value::Null) {
            return true;
        }
        if Instant::now() >= until {
            return false;
        }
        thread::sleep(OBSERVE_SLICE);
    }
}

/// The adapter holds the lease and the vehicle says so; `checks`, asking while it does, is denied.
fn one_lease_verdict(h: &Handover, after: &Result<Value, String>) -> (Verdict, String) {
    let holder = after
        .as_ref()
        .ok()
        .and_then(|m| m.pointer("/mavlink/control/lease/holder")?.as_str())
        .unwrap_or("none");
    if h.ground != Some(LeaseStatus::Held) {
        let detail = format!("the adapter did not take the lease: {:?}", h.ground);
        return (Verdict::Incomplete, detail);
    }
    if h.asked == Some(Ok(())) {
        return (
            Verdict::Fail,
            "both controllers were granted the lease".to_owned(),
        );
    }
    if h.asked != Some(Err(Refusal::LeaseHeld)) {
        let detail = format!("checks asked and got {:?}, not lease_held", h.asked);
        return (Verdict::Fail, detail);
    }
    let detail = format!(
        "the adapter holds the lease (vehicle: holder {holder}); checks was denied: lease_held"
    );
    (Verdict::Pass, detail)
}

/// A phase in which nothing may reach SITL, and the evidence.
#[derive(Debug, Clone)]
struct Untouched {
    /// The vehicle was shown to apply the valid envelopes (its counters rose by exactly them) or
    /// was heard just before: nothing applied then means something.
    answering: bool,
    /// Vehicle heartbeats seen meanwhile, and those that showed it armed.
    heartbeats: usize,
    armed: usize,
    /// Invalid discrete and continuous envelopes the vehicle applied meanwhile; `None` if
    /// unreadable or fewer than the valid ones were applied.
    applied: Option<u64>,
    /// Envelopes it had to refuse, and its answers.
    cases: Vec<Case>,
}

/// Nothing applied and SITL disarmed, with evidence that the vehicle was answering and refused
/// each envelope for the rule it breaks. Anything applied, an armed heartbeat or a refusal for the
/// wrong reason fails; missing evidence is incomplete.
fn untouched_verdict(e: &Untouched) -> (Verdict, String) {
    if let Some(applied @ 1..) = e.applied {
        return (
            Verdict::Fail,
            format!("the vehicle applied {applied} envelopes"),
        );
    }
    if e.armed > 0 {
        return (Verdict::Fail, format!("{} armed heartbeats", e.armed));
    }
    let right = |c: &Case| matches!(c.got, Some(Answer::Refused(r)) if c.expected.contains(&r));
    if let Some(case) = e.cases.iter().find(|c| c.got.is_some() && !right(c)) {
        let detail = format!(
            "{}: answered {:?}, expected a refusal as {:?}",
            case.name, case.got, case.expected
        );
        return (Verdict::Fail, detail);
    }
    if let Some(case) = e.cases.iter().find(|c| c.got.is_none()) {
        let detail = format!("{}: no answer from the vehicle", case.name);
        return (Verdict::Incomplete, detail);
    }
    if !e.answering {
        let detail = "the vehicle was not shown to answer, so nothing applied proves nothing";
        return (Verdict::Incomplete, detail.to_owned());
    }
    if e.heartbeats == 0 {
        let detail = "no vehicle heartbeat meanwhile, so disarmed proves nothing";
        return (Verdict::Incomplete, detail.to_owned());
    }
    let Some(_) = e.applied else {
        let detail = "vehicle metrics unreadable, applications not shown";
        return (Verdict::Incomplete, detail.to_owned());
    };
    let refused: Vec<String> = e
        .cases
        .iter()
        .map(|c| {
            format!(
                "{} ({})",
                c.name,
                c.got.map(|r| r.to_string()).unwrap_or_default()
            )
        })
        .collect();
    let mut detail = format!("nothing applied, {} heartbeats all disarmed", e.heartbeats);
    if !refused.is_empty() {
        detail = format!("{detail}; refused: {}", refused.join(", "));
    }
    (Verdict::Pass, detail)
}

/// Discrete and continuous envelopes the vehicle applied between two readings.
fn applied(before: &Result<Value, String>, after: &Result<Value, String>) -> Option<u64> {
    let discrete = increase(before, after, "/mavlink/control/applied/discrete")?;
    let continuous = increase(before, after, "/mavlink/control/applied/continuous")?;
    Some(discrete + continuous)
}

fn armed(seen: &[Seen]) -> usize {
    seen.iter()
        .filter(|s| s.from_vehicle && matches!(s.kind, Kind::Heartbeat { armed: true, .. }))
        .count()
}

/// One command through the adapter and its acknowledgement.
#[derive(Debug, Clone, Copy)]
struct Ack {
    sent_unix_ms: u64,
    command: MavCmd,
    /// Round trip in ms and result; `None` without an acknowledgement in time.
    answer: Option<(f64, MavResult)>,
}

/// Sends a `COMMAND_LONG` through the adapter and waits for the vehicle's system to acknowledge it.
/// What queued while the station was not read is read first, so an old acknowledgement cannot
/// count.
fn round_trip(gcs: &mut Gcs, message: &MavMessage) -> Ack {
    gcs.pump(Instant::now() + DRAIN);
    let command = match message {
        MavMessage::COMMAND_LONG(c) => c.command,
        _ => MavCmd::default(),
    };
    let (sent_at, sent_unix_ms) = (Instant::now(), thinlink_control::unix_ms());
    let ack = |s: &Seen| match s.kind {
        Kind::Ack {
            command: acked,
            result,
        } if s.from_vehicle && acked == command => Some((s.at, result)),
        _ => None,
    };
    let from = gcs.seen.len();
    let answer = gcs.send(message).then(|| {
        gcs.pump_until(sent_at + ACK_TIMEOUT, |seen| {
            seen.iter().any(|s| ack(s).is_some())
        });
        let (at, result) = gcs.seen.get(from..)?.iter().find_map(ack)?;
        Some((ms_between(sent_at, at), result))
    });
    Ack {
        sent_unix_ms,
        command,
        answer: answer.flatten(),
    }
}

/// Manual mode, then arm and disarm `ARM_ROUNDS` times, each waiting for its acknowledgement.
fn arm_rounds(gcs: &mut Gcs) -> Vec<Ack> {
    let mut acks = vec![round_trip(gcs, &set_mode(MANUAL))];
    for _ in 0..ARM_ROUNDS {
        acks.push(round_trip(gcs, &intruder::arm()));
        acks.push(round_trip(gcs, &disarm()));
    }
    acks
}

/// A station resends the last disarm (`confirmation` 1) within the ACK window, as after a lost
/// acknowledgement: the adapter keeps its transaction, the vehicle answers it from its cache
/// without writing it to SITL again, and the adapter acknowledges it `ACCEPTED`.
fn resend(results: &mut Results, parts: &mut Parts) {
    let before = check::get_json(&parts.vehicle, "/metrics");
    let mut again = disarm();
    if let MavMessage::COMMAND_LONG(c) = &mut again {
        c.confirmation = 1;
    }
    let ack = round_trip(&mut parts.station, &again);
    let after = check::get_json(&parts.vehicle, "/metrics");
    let repeated = increase(&before, &after, "/mavlink/control/repeated");
    let applied = increase(&before, &after, "/mavlink/control/applied/discrete");
    let accepted = matches!(ack.answer, Some((_, MavResult::MAV_RESULT_ACCEPTED)));
    let (verdict, detail) = match (applied, repeated) {
        (Some(1..), _) => (Verdict::Fail, "the resend was applied again".to_owned()),
        (None, _) | (_, None) => (Verdict::Incomplete, "vehicle metrics unreadable".to_owned()),
        (Some(0), Some(1)) if accepted => (
            Verdict::Pass,
            "answered from the vehicle's transaction cache, not applied again; ACCEPTED".to_owned(),
        ),
        _ => (
            Verdict::Fail,
            format!(
                "answer {:?}, vehicle repeated {repeated:?}, applied {applied:?}",
                ack.answer
            ),
        ),
    };
    results.record("resend_not_reapplied", verdict, detail);
}

fn record_acks(results: &mut Results, acks: &[Ack]) {
    let mut ms: Vec<f64> = acks.iter().filter_map(|a| Some(a.answer?.0)).collect();
    ms.sort_by(f64::total_cmp);
    let metrics = [
        ("discrete_ack_p50_ms", percentile(&ms, 50)),
        ("discrete_ack_max_ms", ms.last().copied()),
    ];
    results.metrics.extend(
        metrics
            .into_iter()
            .filter_map(|(name, value)| Some((name, value?))),
    );
    results.sample_counts.insert("discrete_acks", ms.len());
    let refused = acks.iter().find(|a| {
        a.answer
            .is_some_and(|(_, r)| r != MavResult::MAV_RESULT_ACCEPTED)
    });
    let (verdict, detail) = if let Some(a) = refused {
        (
            Verdict::Fail,
            format!("{:?} answered {:?}", a.command, a.answer),
        )
    } else if ms.len() < acks.len() {
        let detail = format!(
            "{} of {} commands unacknowledged",
            acks.len() - ms.len(),
            acks.len()
        );
        (Verdict::Incomplete, detail)
    } else {
        let detail = format!(
            "mode and {ARM_ROUNDS} arm and disarm rounds through the adapter, all {} accepted by SITL",
            acks.len()
        );
        (Verdict::Pass, detail)
    };
    results.record("discrete_acked", verdict, detail);
}

/// `discrete_ack` per command, `seq` its index.
fn ack_samples(acks: &[Ack]) -> Vec<Sample> {
    acks.iter()
        .zip(0_u32..)
        .filter_map(|(a, seq)| {
            Some(Sample {
                unix_s: float(a.sent_unix_ms) / 1000.0,
                series: "discrete_ack",
                seq,
                value: a.answer?.0,
            })
        })
        .collect()
}

/// When a profile's timeline cuts both directions and restores them, in ms after its start.
pub(crate) fn blackout(profile: &Profile) -> Option<(u64, u64)> {
    let at = |action: Action| {
        profile
            .timeline
            .iter()
            .find(|t| t.action == action)
            .map(|t| t.at_ms)
    };
    Some((at(Action::Blackout)?, at(Action::Restore)?))
}

/// `DO_SET_MODE` as `QGroundControl` sends it for `ArduPilot`.
fn set_mode(mode: u32) -> MavMessage {
    #[expect(
        clippy::cast_precision_loss,
        reason = "ArduRover's mode numbers are small"
    )]
    let mode = mode as f32;
    command(MavCmd::MAV_CMD_DO_SET_MODE, CUSTOM_MODE_ENABLED, mode)
}

/// `MANUAL_CONTROL` to SITL with throttle `z` (-1000 to 1000) and the wheel straight.
fn throttle(z: i16) -> MavMessage {
    MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA {
        target: 1,
        z,
        ..MANUAL_CONTROL_DATA::default()
    })
}

fn disarm() -> MavMessage {
    command(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
}

fn force_disarm() -> MavMessage {
    command(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, FORCE_ARM_DISARM)
}

/// Sets the interval of the `WATCHED` messages; 0 restores SITL's default.
fn watch(gcs: &mut Gcs, interval_us: f32) {
    for id in WATCHED {
        round_trip(
            gcs,
            &command(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, id, interval_us),
        );
    }
}

fn ms_between(from: Instant, to: Instant) -> f64 {
    to.saturating_duration_since(from).as_secs_f64() * 1000.0
}

/// SITL's telemetry straight from mavlink-router, read on a thread so each message is timed as it
/// arrives, whatever the check is doing.
struct Observer {
    seen: Arc<Mutex<Vec<Seen>>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Observer {
    fn start(listen: SocketAddr) -> io::Result<Self> {
        let mut gcs = Gcs::observe(listen)?;
        let seen = Arc::new(Mutex::new(Vec::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let (seen, stop) = (Arc::clone(&seen), Arc::clone(&stop));
            thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    gcs.pump(Instant::now() + OBSERVE_SLICE);
                    if let Ok(mut all) = seen.lock() {
                        let room = MAX_OBSERVED.saturating_sub(all.len());
                        all.extend(gcs.seen.drain(..).take(room));
                    }
                }
            })
        };
        Ok(Self {
            seen,
            stop,
            thread: Some(thread),
        })
    }

    /// What arrived before `to`.
    fn before(&self, to: Instant) -> Vec<Seen> {
        self.seen
            .lock()
            .map(|seen| seen.iter().filter(|s| s.at < to).copied().collect())
            .unwrap_or_default()
    }

    /// What arrived from `from` on.
    fn since(&self, from: Instant) -> Vec<Seen> {
        self.seen
            .lock()
            .map(|seen| seen.iter().filter(|s| s.at >= from).copied().collect())
            .unwrap_or_default()
    }

    fn await_heartbeat(&self, until: Instant) -> bool {
        loop {
            let heard = self
                .seen
                .lock()
                .is_ok_and(|seen| seen.iter().any(Seen::vehicle_heartbeat));
            if heard {
                return true;
            }
            if Instant::now() >= until {
                return false;
            }
            thread::sleep(OBSERVE_SLICE);
        }
    }
}

impl Drop for Observer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_applied_passes_only_with_evidence_from_an_answering_vehicle() {
        let refused = Case {
            name: "duplicate seq",
            expected: &[Refusal::DuplicateSeq],
            got: Some(Answer::Refused(Refusal::DuplicateSeq)),
        };
        let clean = Untouched {
            answering: true,
            heartbeats: 5,
            armed: 0,
            applied: Some(0),
            cases: vec![refused.clone()],
        };
        let with = |f: fn(&mut Untouched)| {
            let mut e = clean.clone();
            f(&mut e);
            e
        };
        let cases = [
            (clean.clone(), Verdict::Pass),
            (with(|e| e.applied = Some(1)), Verdict::Fail),
            (with(|e| e.armed = 1), Verdict::Fail),
            (
                with(|e| e.cases[0].got = Some(Answer::Refused(Refusal::ChallengeExpired))),
                Verdict::Fail,
            ),
            (
                with(|e| e.cases[0].got = Some(Answer::Taken)),
                Verdict::Fail,
            ),
            (
                with(|e| {
                    e.applied = Some(1);
                    e.answering = false;
                }),
                Verdict::Fail,
            ),
            (with(|e| e.cases[0].got = None), Verdict::Incomplete),
            (with(|e| e.answering = false), Verdict::Incomplete),
            (with(|e| e.heartbeats = 0), Verdict::Incomplete),
            (with(|e| e.applied = None), Verdict::Incomplete),
        ];
        for (evidence, expected) in cases {
            assert_eq!(untouched_verdict(&evidence).0, expected, "{evidence:?}");
        }
    }
}
