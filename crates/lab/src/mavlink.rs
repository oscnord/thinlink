//! `lab check mavlink`: the `MAVLink` path end to end, with `checks` as a ground station. The ground
//! adapter sends it telemetry at 192.168.232.40:14552 from a socket of its own, and it replies to
//! that socket the way `QGroundControl` does. It measures heartbeats, the `REQUEST_MESSAGE` round
//! trip and the telemetry rate, then shows that the adapter refuses an `ARM_DISARM` without the
//! control lease, that SITL's ports are closed to the overlay and that a forwarding cut silences
//! telemetry.
//! `docs/testbed.md#mavlink-check` defines every metric.

pub(crate) mod gcs;

use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, TcpStream};
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{COMMAND_LONG_DATA, MavCmd, MavMessage, MavResult};
use serde_json::{Value, json};

use thinlink_video::unix_ms;

use self::gcs::{Gcs, HEARTBEAT_EVERY, Kind, Seen};
use crate::budget::{self, Budget};
use crate::check::{self, Results, Sample, Verdict, drain_for, float, increase, percentile};
use crate::client::AgentClient;
use crate::ping;

/// `AUTOPILOT_VERSION`, as `REQUEST_MESSAGE` takes it (param 1).
const AUTOPILOT_VERSION: f32 = 148.0;
/// `ARM_DISARM` param 2 that arms without `ArduPilot`'s pre-arm checks (its
/// `magic_force_arm_disarm_value`): if the command reached SITL, SITL would arm.
pub(crate) const FORCE_ARM_DISARM: f32 = 21196.0;
const VEHICLE_OVERLAY: Ipv4Addr = Ipv4Addr::new(10, 77, 0, 2);
/// SITL's `MAVLink` serial ports; tcp 5760 is the companion's.
const SITL_PORTS: [u16; 3] = [5760, 5762, 5763];

/// SITL boots under qemu after `lab-up`; its first heartbeat can take a while.
const FIRST_TELEMETRY_DEADLINE: Duration = Duration::from_secs(60);
const REQUEST_EVERY: Duration = Duration::from_secs(2);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(5);
const ARM_ATTEMPTS: u32 = 3;
const ARM_EVERY: Duration = Duration::from_secs(1);
/// After the last attempt, a GEO round trip plus three heartbeats.
const ARM_WATCH: Duration = Duration::from_secs(4);
const PING_PROBES: u32 = 3;
const PING_TIMEOUT: Duration = Duration::from_secs(2);
const CONNECT_TIMEOUT: Duration = Duration::from_secs(2);
/// After the drain, three heartbeat intervals without a vehicle message.
const SILENCE: Duration = Duration::from_secs(3);
/// One `WireGuard` keepalive interval (15 s) plus margin, as in the boot check.
const RECOVERY_DEADLINE: Duration = Duration::from_secs(25);

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// Where the ground adapter sends `checks` its telemetry.
    pub listen: SocketAddr,
    pub duration: Duration,
    /// Base URLs of the vehicle and ground metrics endpoints.
    pub vehicle_metrics: String,
    pub ground_metrics: String,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "listen": self.listen,
            "duration_s": self.duration.as_secs_f64(),
            "request_every_s": REQUEST_EVERY.as_secs_f64(),
            "request_timeout_s": REQUEST_TIMEOUT.as_secs_f64(),
            "arm_attempts": ARM_ATTEMPTS,
            "cut_silence_s": SILENCE.as_secs_f64(),
            "recovery_deadline_s": RECOVERY_DEADLINE.as_secs_f64(),
            "vehicle_metrics": self.vehicle_metrics,
            "ground_metrics": self.ground_metrics,
        })
    }
}

pub fn run(
    agent: &AgentClient,
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
    let mut gcs = match Gcs::bind(settings.listen) {
        Ok(gcs) => gcs,
        Err(e) => {
            let detail = format!("cannot listen on {}: {e}", settings.listen);
            results.record("telemetry_received", Verdict::Incomplete, detail);
            return results;
        }
    };
    if !wait_for_telemetry(&mut results, &mut gcs) {
        for name in [
            "request_acked",
            "requested_message",
            "arm_refused",
            "sitl_ports_closed",
            "forwarding_cut",
            "telemetry_recovery",
        ] {
            results.record(name, Verdict::Incomplete, "no telemetry from the vehicle");
        }
        return results;
    }
    let start = Reading::take(settings);
    let requests = request_autopilot_version(&mut gcs, start.at + settings.duration);
    let end = Reading::take(settings);
    record_window(&mut results, &gcs.seen, &start, &end);
    record_requests(&mut results, &requests);
    let arm = try_to_arm(&mut gcs, settings);
    record_arm(&mut results, &arm);
    check_sitl_ports(&mut results);
    let rtt_max = requests
        .iter()
        .filter_map(Request::ack_rtt_ms)
        .reduce(f64::max);
    let drain = rtt_max.map_or(drain_for(f64::INFINITY), drain_for);
    let cut = cut_and_restore(&mut results, agent, &mut gcs, drain);
    record_cut(&mut results, &cut, &gcs.seen);
    results.sample_counts.insert("messages", gcs.seen.len());
    results
        .metrics
        .insert("messages_dropped", float(gcs.dropped));
    results
        .metrics
        .insert("undecodable", float(gcs.undecodable));
    results.samples = samples(&gcs.seen, &requests);
    results.readings = json!({
        "start": start.to_json(),
        "end": end.to_json(),
        "arm_before": arm.before.to_json(),
        "arm_after": arm.after.to_json(),
    });
    budget::grade(&mut results, budget);
    results
}

/// `COMMAND_LONG` to SITL's autopilot component.
pub(crate) fn command(command: MavCmd, param1: f32, param2: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command,
        param1,
        param2,
        target_system: 1,
        target_component: 1,
        ..COMMAND_LONG_DATA::default()
    })
}

fn wait_for_telemetry(results: &mut Results, gcs: &mut Gcs) -> bool {
    let started = Instant::now();
    let deadline = started + FIRST_TELEMETRY_DEADLINE;
    let name = "telemetry_received";
    if gcs.pump_until(deadline, |seen| seen.iter().any(Seen::vehicle_heartbeat)) {
        let detail = format!(
            "vehicle heartbeat after {} ms",
            started.elapsed().as_millis()
        );
        results.record(name, Verdict::Pass, detail);
        return true;
    }
    let detail = format!(
        "no vehicle heartbeat on {} within {} s",
        gcs.socket
            .local_addr()
            .map_or_else(|e| e.to_string(), |a| a.to_string()),
        FIRST_TELEMETRY_DEADLINE.as_secs()
    );
    results.record(name, Verdict::Fail, detail);
    false
}

/// Vehicle and ground metrics at one moment.
struct Reading {
    at: Instant,
    unix_ms: u64,
    vehicle: Result<Value, String>,
    ground: Result<Value, String>,
}

impl Reading {
    fn take(settings: &Settings) -> Self {
        Self {
            at: Instant::now(),
            unix_ms: unix_ms(),
            vehicle: check::get_json(&AgentClient::new(&settings.vehicle_metrics), "/metrics"),
            ground: check::get_json(&AgentClient::new(&settings.ground_metrics), "/metrics"),
        }
    }

    fn to_json(&self) -> Value {
        let value = |r: &Result<Value, String>| match r {
            Ok(v) => v.clone(),
            Err(e) => json!({ "error": e }),
        };
        json!({
            "unix_ms": self.unix_ms,
            "vehicle": value(&self.vehicle),
            "ground": value(&self.ground),
        })
    }
}

/// One `REQUEST_MESSAGE(AUTOPILOT_VERSION)` and what came back within the timeout.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Request {
    sent_at: Instant,
    sent_unix_ms: u64,
    ack: Option<(Instant, MavResult)>,
    message_at: Option<Instant>,
}

impl Request {
    /// The first acknowledgement of `REQUEST_MESSAGE` and the first `AUTOPILOT_VERSION` in `seen`.
    fn answered_by(sent_at: Instant, sent_unix_ms: u64, seen: &[Seen]) -> Self {
        let ack = seen.iter().find_map(|s| match s.kind {
            Kind::Ack {
                command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
                result,
            } if s.from_vehicle => Some((s.at, result)),
            _ => None,
        });
        let message_at = seen
            .iter()
            .find(|s| s.from_vehicle && s.kind == Kind::AutopilotVersion)
            .map(|s| s.at);
        Self {
            sent_at,
            sent_unix_ms,
            ack,
            message_at,
        }
    }

    fn complete(&self) -> bool {
        self.ack.is_some() && self.message_at.is_some()
    }

    fn ack_rtt_ms(&self) -> Option<f64> {
        self.ack.map(|(at, _)| ms_between(self.sent_at, at))
    }

    fn message_ms(&self) -> Option<f64> {
        self.message_at.map(|at| ms_between(self.sent_at, at))
    }
}

fn ms_between(from: Instant, to: Instant) -> f64 {
    to.saturating_duration_since(from).as_secs_f64() * 1000.0
}

/// Requests `AUTOPILOT_VERSION` every `REQUEST_EVERY` until `until`, one at a time, each waiting
/// for its answer up to `REQUEST_TIMEOUT`.
fn request_autopilot_version(gcs: &mut Gcs, until: Instant) -> Vec<Request> {
    let request = command(MavCmd::MAV_CMD_REQUEST_MESSAGE, AUTOPILOT_VERSION, 0.0);
    let mut requests = Vec::new();
    while Instant::now() < until {
        let (sent_at, sent_unix_ms) = (Instant::now(), unix_ms());
        if gcs.send(&request) {
            let from = gcs.seen.len();
            gcs.pump_until(sent_at + REQUEST_TIMEOUT, |seen| {
                Request::answered_by(sent_at, sent_unix_ms, seen).complete()
            });
            let seen = gcs.seen.get(from..).unwrap_or_default();
            requests.push(Request::answered_by(sent_at, sent_unix_ms, seen));
        }
        gcs.pump(sent_at + REQUEST_EVERY);
    }
    requests
}

/// Heartbeats, gaps and received bytes from the vehicle between two readings, plus the rates the
/// vehicle reports.
fn record_window(results: &mut Results, seen: &[Seen], start: &Reading, end: &Reading) {
    let seconds = end.at.saturating_duration_since(start.at).as_secs_f64();
    let inside: Vec<&Seen> = seen
        .iter()
        .filter(|s| s.from_vehicle && (start.at..=end.at).contains(&s.at))
        .collect();
    let heartbeats: Vec<Instant> = inside
        .iter()
        .filter(|s| s.vehicle_heartbeat())
        .map(|s| s.at)
        .collect();
    let received_bytes: u64 = inside.iter().map(|s| u64::from(s.size)).sum();
    let per_second =
        |count: Option<u64>| count.filter(|_| seconds > 0.0).map(|c| float(c) / seconds);
    let vehicle = |pointer| increase(&start.vehicle, &end.vehicle, pointer);
    let metrics = [
        (
            "heartbeats_per_s",
            per_second(Some(heartbeats.len() as u64)),
        ),
        (
            "heartbeat_gap_max_ms",
            Some(max_gap_ms(&heartbeats, start.at, end.at)),
        ),
        ("received_bytes_per_s", per_second(Some(received_bytes))),
        (
            "telemetry_bytes_per_s",
            per_second(vehicle("/mavlink/telemetry/bytes_out")),
        ),
        (
            "sitl_bytes_per_s",
            per_second(vehicle("/mavlink/sitl_bytes")),
        ),
        (
            "telemetry_rate_limited",
            vehicle("/mavlink/telemetry/dropped/rate_limited").map(float),
        ),
        (
            "telemetry_expired",
            vehicle("/mavlink/telemetry/dropped/expired").map(float),
        ),
    ];
    results.metrics.extend(
        metrics
            .into_iter()
            .filter_map(|(name, value)| Some((name, value?))),
    );
    results.sample_counts.insert("heartbeats", heartbeats.len());
}

/// Longest wait for a heartbeat, counting from `start` and up to `end`.
fn max_gap_ms(heartbeats: &[Instant], start: Instant, end: Instant) -> f64 {
    let mut last = start;
    let mut gap = Duration::ZERO;
    for &at in heartbeats.iter().chain(std::iter::once(&end)) {
        gap = gap.max(at.saturating_duration_since(last));
        last = at;
    }
    gap.as_secs_f64() * 1000.0
}

fn record_requests(results: &mut Results, requests: &[Request]) {
    let mut rtts: Vec<f64> = requests.iter().filter_map(Request::ack_rtt_ms).collect();
    let mut delays: Vec<f64> = requests.iter().filter_map(Request::message_ms).collect();
    rtts.sort_by(f64::total_cmp);
    delays.sort_by(f64::total_cmp);
    let accepted = requests
        .iter()
        .filter(|r| matches!(r.ack, Some((_, MavResult::MAV_RESULT_ACCEPTED))))
        .count();
    let unanswered = requests.iter().filter(|r| !r.complete()).count();
    let metrics = [
        ("request_ack_rtt_p50_ms", percentile(&rtts, 50)),
        ("request_ack_rtt_max_ms", rtts.last().copied()),
        ("requested_message_p50_ms", percentile(&delays, 50)),
        ("requested_message_max_ms", delays.last().copied()),
        ("requests_unanswered", Some(float(unanswered as u64))),
    ];
    results.metrics.extend(
        metrics
            .into_iter()
            .filter_map(|(name, value)| Some((name, value?))),
    );
    results.sample_counts.insert("requests", requests.len());
    let sent = requests.len();
    let (verdict, detail) = if accepted > 0 {
        (
            Verdict::Pass,
            format!("{accepted} of {sent} requests acknowledged as accepted"),
        )
    } else {
        (
            Verdict::Fail,
            format!("none of {sent} requests acknowledged as accepted"),
        )
    };
    results.record("request_acked", verdict, detail);
    let verdict = if delays.is_empty() {
        Verdict::Fail
    } else {
        Verdict::Pass
    };
    let detail = format!(
        "AUTOPILOT_VERSION arrived for {} of {sent} requests",
        delays.len()
    );
    results.record("requested_message", verdict, detail);
}

/// What the adapter and the vehicle did about forced `ARM_DISARM` commands.
struct ArmAttempt {
    before: Reading,
    after: Reading,
    evidence: ArmEvidence,
}

#[derive(Debug, Clone, Copy)]
struct ArmEvidence {
    sent: u32,
    /// Vehicle heartbeats from the first attempt on, and those that showed it armed.
    heartbeats: usize,
    armed: usize,
    /// `MAV_RESULT_DENIED` acknowledgements of `ARM_DISARM`: the adapter's answer without a lease.
    denied: usize,
    /// Any other acknowledgement of `ARM_DISARM`: SITL received the command.
    acks: usize,
    /// Increase of the adapter's count of commands it denied; `None` if unreadable.
    refused: Option<u64>,
}

fn try_to_arm(gcs: &mut Gcs, settings: &Settings) -> ArmAttempt {
    let arm = command(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, FORCE_ARM_DISARM);
    let before = Reading::take(settings);
    let from = gcs.seen.len();
    let mut sent = 0;
    for _ in 0..ARM_ATTEMPTS {
        let at = Instant::now();
        sent += u32::from(gcs.send(&arm));
        gcs.pump(at + ARM_EVERY);
    }
    gcs.pump(Instant::now() + ARM_WATCH);
    let after = Reading::take(settings);
    let watched = gcs.seen.get(from..).unwrap_or_default();
    let heartbeats: Vec<&Seen> = watched.iter().filter(|s| s.vehicle_heartbeat()).collect();
    let arm_acks: Vec<MavResult> = watched
        .iter()
        .filter(|s| s.from_vehicle)
        .filter_map(|s| match s.kind {
            Kind::Ack {
                command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                result,
            } => Some(result),
            _ => None,
        })
        .collect();
    let denied = arm_acks
        .iter()
        .filter(|&&r| r == MavResult::MAV_RESULT_DENIED)
        .count();
    let evidence = ArmEvidence {
        sent,
        heartbeats: heartbeats.len(),
        armed: heartbeats
            .iter()
            .filter(|s| matches!(s.kind, Kind::Heartbeat { armed: true, .. }))
            .count(),
        denied,
        acks: arm_acks.len() - denied,
        refused: increase(&before.ground, &after.ground, "/control/denied"),
    };
    ArmAttempt {
        before,
        after,
        evidence,
    }
}

/// Without the lease the adapter answers `DENIED` itself: SITL stayed disarmed and never answered,
/// while the vehicle was demonstrably alive and the adapter counted and answered the refusal. Any
/// sign that SITL got the command fails; missing evidence is incomplete.
fn arm_verdict(e: &ArmEvidence) -> (Verdict, String) {
    if e.armed > 0 || e.acks > 0 {
        let detail = format!(
            "SITL got the command: {} armed heartbeats, {} ARM_DISARM acknowledgements other than DENIED",
            e.armed, e.acks
        );
        return (Verdict::Fail, detail);
    }
    if e.sent == 0 {
        return (Verdict::Incomplete, "could not send ARM_DISARM".to_owned());
    }
    if e.heartbeats == 0 {
        let detail = "no vehicle heartbeat after the attempts, so disarmed proves nothing";
        return (Verdict::Incomplete, detail.to_owned());
    }
    match e.refused {
        None => (
            Verdict::Incomplete,
            "adapter metrics unreadable, refusal not shown".to_owned(),
        ),
        Some(0) => (
            Verdict::Incomplete,
            "the adapter never denied the command".to_owned(),
        ),
        Some(_) if e.denied == 0 => (
            Verdict::Incomplete,
            "the adapter's DENIED never arrived".to_owned(),
        ),
        Some(refused) => (
            Verdict::Pass,
            format!(
                "{refused} of {} forced ARM_DISARM denied by the adapter without a lease ({} DENIED acknowledgements); {} heartbeats, all disarmed",
                e.sent, e.denied, e.heartbeats
            ),
        ),
    }
}

fn record_arm(results: &mut Results, attempt: &ArmAttempt) {
    if let Some(refused) = attempt.evidence.refused {
        results.metrics.insert("arm_refused", float(refused));
    }
    let (verdict, detail) = arm_verdict(&attempt.evidence);
    results.record("arm_refused", verdict, detail);
}

/// SITL listens on every address of the vehicle namespace; `testbed/scripts/sitl.nft` must keep
/// its ports to loopback. The vehicle must answer a ping on the overlay first, or silence proves
/// nothing.
fn check_sitl_ports(results: &mut Results) {
    let name = "sitl_ports_closed";
    let target = VEHICLE_OVERLAY.to_string();
    let probe = match ping::command(&target, PING_PROBES, PING_TIMEOUT).run() {
        Ok(output) => ping::parse(&output),
        Err(e) => {
            results.record(name, Verdict::Incomplete, e.to_string());
            return;
        }
    };
    if probe.replies.is_empty() {
        let detail = format!("{target} does not answer pings, so closed ports prove nothing");
        results.record(name, Verdict::Incomplete, detail);
        return;
    }
    let open: Vec<u16> = SITL_PORTS
        .into_iter()
        .filter(|&port| {
            let addr = SocketAddr::V4(SocketAddrV4::new(VEHICLE_OVERLAY, port));
            TcpStream::connect_timeout(&addr, CONNECT_TIMEOUT).is_ok()
        })
        .collect();
    let (verdict, detail) = if open.is_empty() {
        (
            Verdict::Pass,
            format!("{target} answers pings; tcp {SITL_PORTS:?} accept no connection"),
        )
    } else {
        (
            Verdict::Fail,
            format!("{target} accepts connections on tcp {open:?}"),
        )
    };
    results.record(name, verdict, detail);
}

/// When the forwarding cut and the restore happened.
struct Cut {
    /// From here until the silence begins the vehicle must be heard, or silence proves nothing.
    heard_from: Instant,
    at: Instant,
    drain: Duration,
    restored_at: Option<Instant>,
    error: Option<String>,
}

fn cut_and_restore(
    results: &mut Results,
    agent: &AgentClient,
    gcs: &mut Gcs,
    drain: Duration,
) -> Cut {
    // Telemetry queued in the socket while the port probes blocked would otherwise be stamped as
    // arriving after the cut.
    let heard_from = Instant::now();
    gcs.pump(heard_from + HEARTBEAT_EVERY);
    let at = Instant::now();
    let error = agent
        .post("/forwarding", r#"{"enabled": false}"#)
        .err()
        .map(|e| format!("cannot cut forwarding: {e}"));
    gcs.pump(at + drain + SILENCE);
    let mut cut = Cut {
        heard_from,
        at,
        drain,
        restored_at: None,
        error,
    };
    // Restore even when the cut failed, so the testbed is left forwarding.
    if let Err(e) = agent.post("/forwarding", r#"{"enabled": true}"#) {
        let detail = format!("cannot restore forwarding: {e}");
        results.record("telemetry_recovery", Verdict::Fail, detail);
        return cut;
    }
    let restored_at = Instant::now();
    cut.restored_at = Some(restored_at);
    gcs.pump_until(restored_at + RECOVERY_DEADLINE, |seen| {
        seen.iter().any(|s| s.from_vehicle)
    });
    cut
}

fn record_cut(results: &mut Results, cut: &Cut, seen: &[Seen]) {
    results
        .metrics
        .insert("cut_drain_ms", cut.drain.as_secs_f64() * 1000.0);
    let before_restore = |s: &&Seen| cut.restored_at.is_none_or(|r| s.at <= r);
    let after_cut: Vec<&Seen> = seen
        .iter()
        .filter(|s| s.from_vehicle && s.at >= cut.at)
        .filter(before_restore)
        .collect();
    if let Some(last) = after_cut.last() {
        results
            .metrics
            .insert("cut_last_message_ms", ms_between(cut.at, last.at));
    }
    let silence_from = cut.at + cut.drain;
    let evidence = CutEvidence {
        error: cut.error.as_deref(),
        heard: seen
            .iter()
            .filter(|s| s.from_vehicle && (cut.heard_from..=silence_from).contains(&s.at))
            .count(),
        in_silence: after_cut.iter().filter(|s| s.at > silence_from).count(),
        drain: cut.drain,
    };
    let (verdict, detail) = cut_verdict(&evidence);
    results.record("forwarding_cut", verdict, detail);
    let Some(restored_at) = cut.restored_at else {
        return;
    };
    let recovered = seen
        .iter()
        .find(|s| s.from_vehicle && s.at > restored_at)
        .map(|s| ms_between(restored_at, s.at));
    let Some(ms) = recovered else {
        let detail = format!(
            "no vehicle message within {} s of the restore",
            RECOVERY_DEADLINE.as_secs()
        );
        results.record("telemetry_recovery", Verdict::Fail, detail);
        return;
    };
    results.metrics.insert("telemetry_recovery_ms", ms);
    let detail = format!("vehicle message {ms:.0} ms after the restore");
    results.record("telemetry_recovery", Verdict::Pass, detail);
}

/// What the vehicle sent around a forwarding cut.
#[derive(Debug, Clone, Copy)]
struct CutEvidence<'a> {
    error: Option<&'a str>,
    /// Vehicle messages from just before the cut until the silence began.
    heard: usize,
    /// Vehicle messages after the drain, before the restore.
    in_silence: usize,
    drain: Duration,
}

/// Silence after the drain passes only if the vehicle was heard just before it.
fn cut_verdict(e: &CutEvidence<'_>) -> (Verdict, String) {
    let drain_ms = e.drain.as_millis();
    if let Some(error) = e.error {
        return (Verdict::Fail, error.to_owned());
    }
    if e.in_silence > 0 {
        let detail = format!(
            "{} vehicle messages arrived more than {drain_ms} ms after the cut",
            e.in_silence
        );
        return (Verdict::Fail, detail);
    }
    if e.heard == 0 {
        let detail = "no vehicle message just before the cut, so the silence proves nothing";
        return (Verdict::Incomplete, detail.to_owned());
    }
    let detail = format!(
        "no vehicle message from {drain_ms} ms after the cut until the restore; {} heard just before",
        e.heard
    );
    (Verdict::Pass, detail)
}

/// `heartbeat_gap` for every vehicle heartbeat (gap since the previous one), `ack_rtt` and
/// `requested_message` for every answered request (`seq` its index).
fn samples(seen: &[Seen], requests: &[Request]) -> Vec<Sample> {
    let unix_s = |unix_ms: u64| float(unix_ms) / 1000.0;
    let heartbeats: Vec<&Seen> = seen.iter().filter(|s| s.vehicle_heartbeat()).collect();
    let gaps = heartbeats
        .windows(2)
        .zip(0_u32..)
        .filter_map(|(pair, seq)| {
            let [previous, current] = pair else {
                return None;
            };
            Some(Sample {
                unix_s: unix_s(current.unix_ms),
                series: "heartbeat_gap",
                seq,
                value: ms_between(previous.at, current.at),
            })
        });
    let answers = requests.iter().zip(0_u32..).flat_map(|(request, seq)| {
        let at = |series, ms: Option<f64>| {
            ms.map(|value| Sample {
                unix_s: unix_s(request.sent_unix_ms),
                series,
                seq,
                value,
            })
        };
        [
            at("ack_rtt", request.ack_rtt_ms()),
            at("requested_message", request.message_ms()),
        ]
        .into_iter()
        .flatten()
    });
    gaps.chain(answers).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn silence_after_a_cut_needs_the_vehicle_heard_just_before() {
        let heard = CutEvidence {
            error: None,
            heard: 12,
            in_silence: 0,
            drain: Duration::from_millis(500),
        };
        let cases = [
            (heard, Verdict::Pass),
            (CutEvidence { heard: 0, ..heard }, Verdict::Incomplete),
            (
                CutEvidence {
                    in_silence: 1,
                    ..heard
                },
                Verdict::Fail,
            ),
            (
                CutEvidence {
                    heard: 0,
                    in_silence: 1,
                    ..heard
                },
                Verdict::Fail,
            ),
            (
                CutEvidence {
                    error: Some("cannot cut forwarding"),
                    ..heard
                },
                Verdict::Fail,
            ),
        ];
        for (evidence, expected) in cases {
            assert_eq!(cut_verdict(&evidence).0, expected, "{evidence:?}");
        }
    }

    #[test]
    fn arming_is_refused_only_with_evidence_from_a_live_vehicle() {
        let refused = ArmEvidence {
            sent: 3,
            heartbeats: 6,
            armed: 0,
            denied: 3,
            acks: 0,
            refused: Some(3),
        };
        let cases = [
            (refused, Verdict::Pass),
            (
                ArmEvidence {
                    armed: 1,
                    ..refused
                },
                Verdict::Fail,
            ),
            (ArmEvidence { acks: 1, ..refused }, Verdict::Fail),
            (
                ArmEvidence {
                    armed: 2,
                    refused: None,
                    ..refused
                },
                Verdict::Fail,
            ),
            (
                ArmEvidence {
                    heartbeats: 0,
                    ..refused
                },
                Verdict::Incomplete,
            ),
            (
                ArmEvidence {
                    refused: None,
                    ..refused
                },
                Verdict::Incomplete,
            ),
            (
                ArmEvidence {
                    refused: Some(0),
                    ..refused
                },
                Verdict::Incomplete,
            ),
            (ArmEvidence { sent: 0, ..refused }, Verdict::Incomplete),
            (
                ArmEvidence {
                    denied: 0,
                    ..refused
                },
                Verdict::Incomplete,
            ),
        ];
        for (evidence, expected) in cases {
            assert_eq!(arm_verdict(&evidence).0, expected, "{evidence:?}");
        }
    }
}
