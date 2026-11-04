//! `lab check real-mode`: the vehicle in real mode against the `ArduRover` SITL, which real mode
//! accepts only because the test-only `sitl_in_real_mode` is set. With every interlock holding,
//! control reaches the autopilot; with any one failing, it does not. `checks` plays the ground
//! station through the adapter and, for the envelopes only a compromised relay or a stranger could
//! send, a second controller. Each case changes one thing, shows the vehicle refusing control for
//! it (the adapter's command answered denied, nothing applied, SITL disarmed) and puts it back.
//! The testbed's vehicle is restarted on another config by `just lab-check` (`lab: vehicle-config`)
//! for the cases that are set at startup (a missing key, a clock with no bound), and on its own
//! config at the end.

use std::thread;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::MavResult;
use serde_json::Value;

use thinlink_control::envelope::ToGround;
use thinlink_control::ground::LeaseStatus;

use super::intruder::{Answer, Case};
use super::{
    ACK_TIMEOUT, MANUAL, Parts, Settings, WATCH_INTERVAL_US, armed, disarm, intruder, round_trip,
    set_mode, start, take_lease, wait_for_telemetry, watch,
};
use crate::check::{self, Results, Verdict};
use crate::client::AgentClient;
use crate::mavlink::gcs::Kind;

const NORMAL: &str = "vehicle.toml";
const REAL: &str = "vehicle-real.toml";
const NO_KEY: &str = "vehicle-real-nokey.toml";
const NO_CLOCK: &str = "vehicle-real-noclock.toml";
/// What `just lab-check` waits for on stdout to start the vehicle on another config.
const CONFIG_REQUEST: &str = "lab: vehicle-config";
/// The vehicle restarts, SITL reconnects under qemu, and the parameters are read back.
const SWITCH_DEADLINE: Duration = Duration::from_secs(180);
/// A hook's effect shows within a tick or a probe.
const CHANGE_DEADLINE: Duration = Duration::from_secs(20);
const POLL: Duration = Duration::from_millis(500);
/// The bound injected to follow through the ground, in milliseconds.
const INJECTED_MS: u64 = 300;

/// One thing to break in real mode: the vehicle's hooks that break it and put it back, and the
/// interlock that must then fail.
struct Break {
    assertion: &'static str,
    what: &'static str,
    induce: &'static str,
    restore: &'static str,
    interlock: &'static str,
}

const BREAKS: [Break; 2] = [
    Break {
        assertion: "wrong_failsafe_param_refuses",
        what: "FS_GCS_ENABLE required to be 0, which SITL reports as 1",
        induce: "/real/require/FS_GCS_ENABLE/0",
        restore: "/real/require/reset",
        interlock: "failsafe_params",
    },
    Break {
        assertion: "sitl_refused",
        what: "the SITL allowance taken away",
        induce: "/real/allow-sitl/off",
        restore: "/real/allow-sitl/on",
        interlock: "identity",
    },
];

pub fn run(agent: &AgentClient, profile_text: &str, settings: &Settings) -> Results {
    let mut results = Results {
        settings: settings.to_json(),
        ..Results::default()
    };
    if !check::apply_profile(&mut results, agent, profile_text) {
        return results;
    }
    let mut parts = match start(settings) {
        Ok(parts) => parts,
        Err(e) => {
            results.record("real_mode_up", Verdict::Incomplete, e.to_string());
            return results;
        }
    };
    if bring_up(&mut results, &mut parts) {
        authenticated(&mut results, &mut parts);
        clock_follows(&mut results, &parts);
        for case in &BREAKS {
            breaks(&mut results, &mut parts, case);
        }
        stale_bound_keeps_control(&mut results, &mut parts);
        without_a_key(&mut results, &mut parts);
        without_a_clock(&mut results, &mut parts);
    }
    restore(&mut results, &parts);
    results
}

/// The vehicle on the real-mode config, its interlocks holding and control reaching the autopilot.
fn bring_up(results: &mut Results, parts: &mut Parts) -> bool {
    if !switch(results, parts, REAL, "real_mode_up") {
        return false;
    }
    if !wait_for_telemetry(results, parts) {
        return false;
    }
    let held = wait_for(SWITCH_DEADLINE, || {
        interlocks(&parts.vehicle).is_some_and(|f| f.is_empty())
    });
    let (verdict, detail) = if held {
        (
            Verdict::Pass,
            "mode real, no interlock failing, control to the autopilot".to_owned(),
        )
    } else {
        (
            Verdict::Incomplete,
            format!("interlocks still failing: {:?}", interlocks(&parts.vehicle)),
        )
    };
    results.record("interlocks_hold", verdict, detail);
    if !held {
        return false;
    }
    let (verdict, detail) = control_reaches_the_autopilot(parts);
    results.record("control_reaches_the_autopilot", verdict, detail);
    verdict == Verdict::Pass
}

/// A mode change, an arm and a disarm through the adapter, each accepted by SITL and each applied.
fn control_reaches_the_autopilot(parts: &mut Parts) -> (Verdict, String) {
    if !adapter_holds_lease(parts) {
        return (
            Verdict::Incomplete,
            "the adapter did not take the lease".to_owned(),
        );
    }
    // mavlink-router routes the autopilot's acknowledgements to the stations it has heard raw
    // traffic from; a station's heartbeats travel in envelopes, so it asks for telemetry first.
    watch(&mut parts.station, WATCH_INTERVAL_US);
    let heard = parts.station.seen.len();
    let before = check::get_json(&parts.vehicle, "/metrics");
    let answers = [
        round_trip(&mut parts.station, &set_mode(MANUAL)).answer,
        round_trip(&mut parts.station, &intruder::arm()).answer,
        round_trip(&mut parts.station, &disarm()).answer,
    ];
    let after = check::get_json(&parts.vehicle, "/metrics");
    let applied = check::increase(&before, &after, "/mavlink/control/applied/discrete");
    let accepted =
        |a: &Option<(f64, MavResult)>| matches!(a, Some((_, MavResult::MAV_RESULT_ACCEPTED)));
    if answers.iter().all(accepted) && applied == Some(3) {
        return (
            Verdict::Pass,
            "mode, arm and disarm accepted by SITL, 3 applied".to_owned(),
        );
    }
    let acks: Vec<_> = parts
        .station
        .seen
        .get(heard..)
        .unwrap_or_default()
        .iter()
        .filter(|s| matches!(s.kind, Kind::Ack { .. }))
        .map(|s| format!("{:?} from_vehicle {}", s.kind, s.from_vehicle))
        .collect();
    let detail = format!("answers {answers:?}, applied {applied:?}, acks seen {acks:?}");
    (Verdict::Fail, detail)
}

/// The envelopes a stranger or a compromised relay could send, to the vehicle with every interlock
/// holding: with no signature (a controller that has no key), signed with another key, and signed
/// over a harmless setpoint whose frame was then swapped. None may be applied.
fn authenticated(results: &mut Results, parts: &mut Parts) {
    let released = parts.ground.delete("/lease").is_ok();
    let free = wait_for(ACK_TIMEOUT, || lease_is_free(&parts.vehicle));
    let before = check::get_json(&parts.vehicle, "/metrics");
    let cases = forged_cases(parts, released && free);
    let after = check::get_json(&parts.vehicle, "/metrics");
    let applied = check::increase(&before, &after, "/mavlink/control/applied/discrete")
        .zip(check::increase(
            &before,
            &after,
            "/mavlink/control/applied/continuous",
        ))
        .map(|(discrete, continuous)| discrete + continuous);
    for (assertion, name, what) in [
        (
            "missing_key_refuses",
            "unsigned",
            "an envelope from a controller with no key",
        ),
        (
            "wrong_key_refuses",
            "wrong key",
            "an envelope signed with another key",
        ),
        (
            "forged_envelope_refuses",
            "tampered",
            "a signed setpoint with its frame swapped for full throttle",
        ),
        (
            "other_vehicle_refuses",
            "other vehicle",
            "an envelope signed for another vehicle",
        ),
        (
            "unsigned_release_refuses",
            "release unsigned",
            "a lease release with no signature",
        ),
        (
            "wrong_key_release_refuses",
            "release wrong key",
            "a lease release signed with another key",
        ),
    ] {
        let case = cases.iter().find(|c| c.name == name);
        let (verdict, detail) = forged_verdict(case, what, applied);
        results.record(assertion, verdict, detail);
    }
}

fn forged_cases(parts: &mut Parts, ready: bool) -> Vec<Case> {
    let granted = ready && matches!(parts.intruder.request(), Some(ToGround::LeaseGrant { .. }));
    let challenged = granted
        && parts
            .intruder
            .await_challenge(Instant::now() + ACK_TIMEOUT)
            .is_some();
    let cases = if challenged {
        parts.intruder.forged(None)
    } else {
        Vec::new()
    };
    parts.intruder.release();
    cases
}

fn forged_verdict(case: Option<&Case>, what: &str, applied: Option<u64>) -> (Verdict, String) {
    let Some(case) = case else {
        return (Verdict::Incomplete, format!("{what}: not sent"));
    };
    if applied.is_some_and(|n| n > 0) {
        return (
            Verdict::Fail,
            format!("{what}: the vehicle applied {applied:?} envelopes"),
        );
    }
    match case.got {
        Some(Answer::Refused(reason)) if case.expected.contains(&reason) => (
            Verdict::Pass,
            format!("{what}: refused {reason}, nothing applied"),
        ),
        Some(answer) => (Verdict::Fail, format!("{what}: answered {answer}")),
        None => (
            Verdict::Incomplete,
            format!("{what}: no answer from the vehicle"),
        ),
    }
}

/// The vehicle's bound reaches the ground, the sum is what the output judges by, and without a
/// bound the label reads `AGE UNKNOWN`.
fn clock_follows(results: &mut Results, parts: &Parts) {
    let ground_clock = |pointer: &str| {
        check::get_json(&parts.ground, "/metrics")
            .ok()
            .and_then(|m| m.pointer(pointer)?.as_u64())
    };
    let _ = parts
        .vehicle
        .post(&format!("/clock/inject/{INJECTED_MS}"), "");
    let followed = wait_for(CHANGE_DEADLINE, || {
        ground_clock("/clock/vehicle_ms")
            .is_some_and(|ms| (INJECTED_MS..INJECTED_MS + 30).contains(&ms))
    });
    let effective = ground_clock("/clock/effective_ms");
    let sum = effective.is_some_and(|ms| (INJECTED_MS + 10..INJECTED_MS + 60).contains(&ms));
    let (verdict, detail) = match (followed, sum) {
        (true, true) => (
            Verdict::Pass,
            format!(
                "injected {INJECTED_MS} ms: the ground's vehicle bound follows and the sum is {effective:?} ms"
            ),
        ),
        (true, false) => (
            Verdict::Fail,
            format!("the vehicle's bound arrived but the sum is {effective:?} ms"),
        ),
        (false, _) => (
            Verdict::Fail,
            "the injected bound did not reach the ground".to_owned(),
        ),
    };
    results.record("clock_bound_reaches_the_ground", verdict, detail);

    let _ = parts.vehicle.post("/clock/inject/missing", "");
    let unknown = wait_for(CHANGE_DEADLINE, || {
        ground_state(&parts.ground).as_deref() == Some("age_unknown")
            && ground_clock("/clock/effective_ms").is_none()
    });
    let state = ground_state(&parts.ground);
    let _ = parts.vehicle.post("/clock/inject/off", "");
    let live = wait_for(CHANGE_DEADLINE, || {
        ground_state(&parts.ground).as_deref() == Some("live")
    });
    let (verdict, detail) = match (unknown, live) {
        (true, true) => (
            Verdict::Pass,
            "without a vehicle bound the label read age_unknown; with it back, live".to_owned(),
        ),
        (false, _) => (
            Verdict::Fail,
            format!("without a bound the label read {state:?}, not age_unknown"),
        ),
        (true, false) => (Verdict::Fail, "the label did not return to live".to_owned()),
    };
    results.record("missing_bound_reads_age_unknown", verdict, detail);
}

fn ground_state(ground: &AgentClient) -> Option<String> {
    let metrics = check::get_json(ground, "/metrics").ok()?;
    metrics.pointer("/video/state")?.as_str().map(str::to_owned)
}

/// Breaks one thing, shows the vehicle refusing control and puts it back.
fn breaks(results: &mut Results, parts: &mut Parts, case: &Break) {
    let outcome = refuses_control(parts, case);
    let (verdict, detail) = outcome;
    results.record(case.assertion, verdict, format!("{}: {detail}", case.what));
}

fn refuses_control(parts: &mut Parts, case: &Break) -> (Verdict, String) {
    let _ = parts.vehicle.post(case.induce, "");
    let shown = wait_for(CHANGE_DEADLINE, || {
        interlocks(&parts.vehicle)
            .is_some_and(|failing| failing.iter().any(|f| f == case.interlock))
    });
    let verdict = if shown {
        arm_is_denied(parts, case.interlock)
    } else {
        (
            Verdict::Fail,
            format!("the vehicle did not report {} failing", case.interlock),
        )
    };
    let _ = parts.vehicle.post(case.restore, "");
    let restored = wait_for(CHANGE_DEADLINE, || {
        interlocks(&parts.vehicle).is_some_and(|f| f.is_empty())
    });
    if restored {
        return verdict;
    }
    (
        Verdict::Incomplete,
        format!("{}; the interlocks did not clear again", verdict.1),
    )
}

/// A forced arm through the adapter while `interlock` fails: the station must be told no, the vehicle
/// must have applied nothing, and SITL must stay disarmed.
fn arm_is_denied(parts: &mut Parts, interlock: &str) -> (Verdict, String) {
    if !adapter_holds_lease(parts) {
        return (
            Verdict::Incomplete,
            "the adapter did not take the lease".to_owned(),
        );
    }
    let before = check::get_json(&parts.vehicle, "/metrics");
    let heard = parts.station.seen.len();
    let ack = round_trip(&mut parts.station, &intruder::arm());
    let after = check::get_json(&parts.vehicle, "/metrics");
    let applied = check::increase(&before, &after, "/mavlink/control/applied/discrete");
    let sunk = check::increase(&before, &after, "/mavlink/control/sunk");
    parts.station.pump(Instant::now() + Duration::from_secs(1));
    let armed = armed(parts.station.seen.get(heard..).unwrap_or_default());
    match (ack.answer, applied, sunk) {
        (_, Some(1..), _) => (Verdict::Fail, "the vehicle applied the command".to_owned()),
        (Some((_, MavResult::MAV_RESULT_ACCEPTED)), ..) => (
            Verdict::Fail,
            "the station was told the command was accepted".to_owned(),
        ),
        _ if armed > 0 => (Verdict::Fail, format!("{armed} armed heartbeats")),
        (Some((_, MavResult::MAV_RESULT_DENIED)), Some(0), Some(1..)) => (
            Verdict::Pass,
            format!("{interlock} failing: the arm was denied, sunk, nothing applied"),
        ),
        (answer, applied, sunk) => (
            Verdict::Incomplete,
            format!("answer {answer:?}, applied {applied:?}, sunk {sunk:?}"),
        ),
    }
}

/// The vehicle started with no public key for one of its controllers refuses control.
fn without_a_key(results: &mut Results, parts: &mut Parts) {
    let start = Start {
        config: NO_KEY,
        up: "no_key_vehicle_up",
        assertion: "missing_key_at_the_vehicle_refuses",
        interlock: "controller_keys",
        what: "a controller without a public key",
    };
    refused_at_start(results, parts, &start);
}

/// The vehicle started with a clock that has no bound refuses control: real mode needs one to start.
fn without_a_clock(results: &mut Results, parts: &mut Parts) {
    let start = Start {
        config: NO_CLOCK,
        up: "no_clock_vehicle_up",
        assertion: "no_bound_at_start_refuses",
        interlock: "clock_bound",
        what: "a clock with no bound",
    };
    refused_at_start(results, parts, &start);
}

/// A real-mode config that breaks one interlock from the start.
struct Start {
    config: &'static str,
    up: &'static str,
    assertion: &'static str,
    interlock: &'static str,
    what: &'static str,
}

/// Restarts the vehicle on `start.config` and shows it refusing control for `start.interlock`.
fn refused_at_start(results: &mut Results, parts: &mut Parts, start: &Start) {
    if !switch(results, parts, start.config, start.up) {
        return;
    }
    let reported = wait_for(SWITCH_DEADLINE, || {
        interlocks(&parts.vehicle)
            .is_some_and(|failing| failing.iter().any(|f| f == start.interlock))
    });
    // The adapter's lease belongs to the vehicle that ended: ask for one from the new one.
    let _ = parts.ground.delete("/lease");
    let (verdict, detail) = if reported {
        arm_is_denied(parts, start.interlock)
    } else {
        (
            Verdict::Fail,
            format!("the vehicle did not report {} failing", start.interlock),
        )
    };
    results.record(
        start.assertion,
        verdict,
        format!("{}: {detail}", start.what),
    );
}

/// Once the vehicle has started, a clock bound that goes stale turns the label to `AGE UNKNOWN` and
/// nothing else: mode, arm and disarm still reach the autopilot and no interlock fails.
fn stale_bound_keeps_control(results: &mut Results, parts: &mut Parts) {
    let _ = parts.vehicle.post("/clock/inject/missing", "");
    let unknown = wait_for(CHANGE_DEADLINE, || {
        ground_state(&parts.ground).as_deref() == Some("age_unknown")
    });
    let holding = interlocks(&parts.vehicle).is_some_and(|f| f.is_empty());
    let (verdict, detail) = match (unknown, holding) {
        (true, true) => control_reaches_the_autopilot(parts),
        (false, _) => (
            Verdict::Incomplete,
            "the label did not read age_unknown, so the bound was not shown stale".to_owned(),
        ),
        (true, false) => (
            Verdict::Fail,
            format!("interlocks failing: {:?}", interlocks(&parts.vehicle)),
        ),
    };
    let _ = parts.vehicle.post("/clock/inject/off", "");
    results.record(
        "stale_bound_keeps_control",
        verdict,
        format!("with the bound gone and the label age_unknown: {detail}"),
    );
}

/// The vehicle back on its normal config, so the next check finds the testbed as `lab-up` made it.
fn restore(results: &mut Results, parts: &Parts) {
    let _ = parts.ground.delete("/lease");
    let _ = parts.vehicle.post("/real/require/reset", "");
    let ok = restart_vehicle(&parts.vehicle, NORMAL);
    let (verdict, detail) = if ok {
        (Verdict::Pass, format!("the vehicle is back on {NORMAL}"))
    } else {
        (
            Verdict::Fail,
            format!("the vehicle did not come back on {NORMAL}: run just lab-up"),
        )
    };
    results.record("testbed_restored", verdict, detail);
}

/// Asks `just lab-check` to start the vehicle on `config` and waits for it: a vehicle that began
/// after the request (its epoch is its start time) and answers.
fn switch(results: &mut Results, parts: &Parts, config: &str, name: &'static str) -> bool {
    let ok = restart_vehicle(&parts.vehicle, config);
    let (verdict, detail) = if ok {
        (Verdict::Pass, format!("the vehicle runs {config}"))
    } else {
        (
            Verdict::Incomplete,
            format!(
                "the vehicle did not restart on {config} within {} s",
                SWITCH_DEADLINE.as_secs()
            ),
        )
    };
    results.record(name, verdict, detail);
    ok
}

fn restart_vehicle(vehicle: &AgentClient, config: &str) -> bool {
    let epoch = |client: &AgentClient| {
        check::get_json(client, "/metrics")
            .ok()
            .and_then(|m| m.pointer("/mavlink/control/epoch")?.as_u64())
    };
    let before = epoch(vehicle);
    println!("{CONFIG_REQUEST} {config}");
    wait_for(SWITCH_DEADLINE, || {
        epoch(vehicle).is_some_and(|now| Some(now) != before)
    })
}

/// The adapter holds the lease with a fresh challenge.
fn adapter_holds_lease(parts: &Parts) -> bool {
    take_lease(&parts.ground) == Some(LeaseStatus::Held)
}

fn lease_is_free(vehicle: &AgentClient) -> bool {
    check::get_json(vehicle, "/metrics")
        .ok()
        .and_then(|m| m.pointer("/mavlink/control/lease").cloned())
        == Some(Value::Null)
}

/// The names of the interlocks the vehicle reports failing, `None` if it cannot be read.
fn interlocks(vehicle: &AgentClient) -> Option<Vec<String>> {
    let metrics = check::get_json(vehicle, "/metrics").ok()?;
    let failing = metrics.pointer("/mavlink/control/interlocks")?.as_array()?;
    Some(
        failing
            .iter()
            .filter_map(|f| Some(f.get("interlock")?.as_str()?.to_owned()))
            .collect(),
    )
}

/// Polls until `holds` or `deadline` passes.
fn wait_for(deadline: Duration, mut holds: impl FnMut() -> bool) -> bool {
    let until = Instant::now() + deadline;
    loop {
        if holds() {
            return true;
        }
        if Instant::now() >= until {
            return false;
        }
        thread::sleep(POLL);
    }
}
