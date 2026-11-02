//! `lab check boot`: what the checks container (ground side, `ctl_net` and overlay peer 10.77.0.4)
//! can observe of a freshly started testbed. `docs/testbed.md` explains each assertion.

use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, TcpStream};
use std::thread;
use std::time::{Duration, Instant};

use serde_json::json;

use crate::check::{self, Results, Sample, Verdict};
use crate::client::AgentClient;
use crate::command::ExecError;
use crate::ping::{self, Probe};
use crate::profile::Profile;

const VEHICLE_OVERLAY: &str = "10.77.0.2";
const GROUND_OVERLAY: &str = "10.77.0.3";
const VEHICLE_UNDERLAY: &str = "192.168.231.10";
const VEHICLE_CTL: &str = "192.168.233.10";
/// The router does not filter `ctl_net`, so its answer there proves checks can probe it.
const ROUTER_CTL: &str = "192.168.233.254";
/// The agent port on the router's `ground_net` address. The agent must only listen on `ctl_net`.
const AGENT_ON_GROUND_NET: SocketAddr =
    SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 168, 232, 254), 9000));

/// Right after start a peer may need a few handshake retries (5 s apart) before the overlay works.
const JOIN_DEADLINE: Duration = Duration::from_secs(30);
/// One `WireGuard` keepalive interval (15 s) plus margin.
const RECOVERY_DEADLINE: Duration = Duration::from_secs(25);
/// While waiting for a first reply, at most one probe this often.
const RETRY_EVERY: Duration = Duration::from_millis(500);
/// How long ping waits for replies; above the round trip of every shipped profile.
const REPLY_TIMEOUT: Duration = Duration::from_secs(2);
const RTT_PROBES: u32 = 10;
/// With fewer replies the RTT is not graded.
const MIN_RTT_REPLIES: usize = 5;
/// Silence passes only when every probe of a full stream goes unanswered, so one lost probe on
/// a lossy profile cannot pass for a blocked path.
const SILENCE_PROBES: u32 = 10;
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);
/// The median overlay RTT must sit in this window around the profile's configured round trip
/// (uplink plus downlink delay). For `geo` (600 ms) it is the 550-700 ms calibration target.
const RTT_BELOW_MS: f64 = 50.0;
const RTT_ABOVE_MS: f64 = 100.0;

/// Silence from the vehicle only means something while it is known to be up.
const NEED_LIVE_VEHICLE: [&str; 4] = [
    "cgnat",
    "ctl_net_carries_no_data",
    "forwarding_cut",
    "forwarding_recovery",
];

pub fn run(agent: &AgentClient, profile: &Profile, profile_text: &str) -> Results {
    let configured_rtt = configured_rtt_ms(profile);
    let (rtt_min, rtt_max) = rtt_window(configured_rtt);
    let mut results = Results {
        thresholds: json!({
            "overlay_rtt_median_ms": { "min": rtt_min, "max": rtt_max },
            "overlay_rtt_min_replies": MIN_RTT_REPLIES,
            "join_deadline_s": JOIN_DEADLINE.as_secs(),
            "recovery_deadline_s": RECOVERY_DEADLINE.as_secs(),
            "silence_probes": SILENCE_PROBES,
            "reply_timeout_s": REPLY_TIMEOUT.as_secs(),
        }),
        ..Results::default()
    };
    check_agent(&mut results, agent);
    let applied = check::apply_profile(&mut results, agent, profile_text);
    check_agent_only_on_ctl_net(&mut results);
    expect_reply(
        &mut results,
        "overlay_ground",
        GROUND_OVERLAY,
        JOIN_DEADLINE,
    );
    let vehicle = expect_reply(
        &mut results,
        "overlay_vehicle",
        VEHICLE_OVERLAY,
        JOIN_DEADLINE,
    );
    check_rtt(&mut results, applied, configured_rtt);
    if vehicle != Verdict::Pass {
        for name in NEED_LIVE_VEHICLE {
            results.record(
                name,
                Verdict::Incomplete,
                "the vehicle is down, so silence proves nothing",
            );
        }
        return results;
    }
    expect_silence(&mut results, "cgnat", VEHICLE_UNDERLAY);
    check_ctl_net_carries_no_data(&mut results);
    check_cut_and_recovery(&mut results, agent);
    results
}

fn check_agent(results: &mut Results, agent: &AgentClient) {
    let name = "agent_reachable";
    match agent.get("/timeline") {
        Ok(_) => results.record(name, Verdict::Pass, "GET /timeline answered"),
        Err(e) => results.record(name, Verdict::Fail, e.to_string()),
    };
}

fn check_agent_only_on_ctl_net(results: &mut Results) {
    let name = "agent_only_on_ctl_net";
    match TcpStream::connect_timeout(&AGENT_ON_GROUND_NET, CONNECT_TIMEOUT) {
        Ok(_) => results.record(
            name,
            Verdict::Fail,
            format!("{AGENT_ON_GROUND_NET} accepts connections"),
        ),
        Err(e) => results.record(name, Verdict::Pass, format!("{AGENT_ON_GROUND_NET}: {e}")),
    };
}

/// Sends `probes` echo requests to `target` and keeps the replies as samples of the probe `name`.
fn probe(
    results: &mut Results,
    name: &'static str,
    target: &str,
    probes: u32,
) -> Result<Probe, ExecError> {
    let probe = ping::parse(&ping::command(target, probes, REPLY_TIMEOUT).run()?);
    *results.sample_counts.entry(name).or_default() += probe.replies.len();
    results.samples.extend(probe.replies.iter().map(|r| Sample {
        unix_s: r.unix_s,
        series: name,
        seq: r.seq,
        value: r.rtt_ms,
    }));
    Ok(probe)
}

/// Probes `target` one echo at a time until the first reply or `deadline`. A lost probe, an ICMP
/// error or a NAT mapping that only returns with the next keepalive cannot end the wait early.
fn expect_reply(
    results: &mut Results,
    name: &'static str,
    target: &str,
    deadline: Duration,
) -> Verdict {
    let started = Instant::now();
    let mut sent: u32 = 0;
    while started.elapsed() < deadline {
        let attempt = Instant::now();
        let probe = match probe(results, name, target, 1) {
            Ok(probe) => probe,
            Err(e) => return results.record(name, Verdict::Incomplete, e.to_string()),
        };
        sent = sent.saturating_add(probe.transmitted);
        if !probe.replies.is_empty() {
            let ms = started.elapsed().as_millis();
            let detail = format!("{target} answered after {ms} ms, probe {sent}");
            return results.record(name, Verdict::Pass, detail);
        }
        thread::sleep(RETRY_EVERY.saturating_sub(attempt.elapsed()));
    }
    let detail = format!(
        "no reply from {target} to {sent} probes in {} s",
        deadline.as_secs()
    );
    results.record(name, Verdict::Fail, detail)
}

fn expect_silence(results: &mut Results, name: &'static str, target: &str) -> Verdict {
    let probe = match probe(results, name, target, SILENCE_PROBES) {
        Ok(probe) => probe,
        Err(e) => return results.record(name, Verdict::Incomplete, e.to_string()),
    };
    let detail = format!(
        "{} replies from {target} to {} of {SILENCE_PROBES} probes",
        probe.replies.len(),
        probe.transmitted
    );
    results.record(name, silence_verdict(&probe), detail)
}

fn silence_verdict(probe: &Probe) -> Verdict {
    if !probe.replies.is_empty() {
        return Verdict::Fail;
    }
    if probe.transmitted < SILENCE_PROBES {
        return Verdict::Incomplete;
    }
    Verdict::Pass
}

/// Every sidecar drops non-orchestration traffic on `ctl_net`, so the vehicle, which answers over
/// the overlay, must not answer a ping on its `ctl_net` address.
fn check_ctl_net_carries_no_data(results: &mut Results) {
    let name = "ctl_net_carries_no_data";
    let router_answers =
        probe(results, "ctl_net_control", ROUTER_CTL, 1).is_ok_and(|p| !p.replies.is_empty());
    if !router_answers {
        let detail =
            format!("{ROUTER_CTL} did not answer on ctl_net, so silence there proves nothing");
        results.record(name, Verdict::Incomplete, detail);
        return;
    }
    expect_silence(results, name, VEHICLE_CTL);
}

fn check_rtt(results: &mut Results, profile_applied: bool, configured_ms: f64) {
    let name = "overlay_rtt";
    if !profile_applied {
        results.record(
            name,
            Verdict::Incomplete,
            "no profile applied to calibrate against",
        );
        return;
    }
    let rtts: Vec<f64> = match probe(results, name, VEHICLE_OVERLAY, RTT_PROBES) {
        Ok(probe) => probe.replies.iter().map(|r| r.rtt_ms).collect(),
        Err(e) => {
            results.record(name, Verdict::Incomplete, e.to_string());
            return;
        }
    };
    if let Some(median) = median(&rtts) {
        results.metrics.insert("overlay_rtt_median_ms", median);
    }
    let (verdict, detail) = rtt_verdict(&rtts, configured_ms);
    results.record(name, verdict, detail);
}

fn check_cut_and_recovery(results: &mut Results, agent: &AgentClient) {
    match agent.post("/forwarding", r#"{"enabled": false}"#) {
        Ok(_) => expect_silence(results, "forwarding_cut", VEHICLE_OVERLAY),
        Err(e) => results.record("forwarding_cut", Verdict::Fail, e.to_string()),
    };
    // Restore even when the cut failed half way, so the testbed is left forwarding.
    if let Err(e) = agent.post("/forwarding", r#"{"enabled": true}"#) {
        let detail = format!("cannot restore forwarding: {e}");
        results.record("forwarding_recovery", Verdict::Fail, detail);
        return;
    }
    let started = Instant::now();
    let verdict = expect_reply(
        results,
        "forwarding_recovery",
        VEHICLE_OVERLAY,
        RECOVERY_DEADLINE,
    );
    if verdict == Verdict::Pass {
        let ms = started.elapsed().as_secs_f64() * 1000.0;
        results.metrics.insert("forwarding_recovery_ms", ms);
    }
}

fn configured_rtt_ms(profile: &Profile) -> f64 {
    f64::from(profile.uplink.delay_ms) + f64::from(profile.downlink.delay_ms)
}

fn rtt_window(configured_ms: f64) -> (f64, f64) {
    (
        (configured_ms - RTT_BELOW_MS).max(0.0),
        configured_ms + RTT_ABOVE_MS,
    )
}

fn rtt_verdict(rtts: &[f64], configured_ms: f64) -> (Verdict, String) {
    let Some(median) = median(rtts).filter(|_| rtts.len() >= MIN_RTT_REPLIES) else {
        let detail = format!(
            "{} of {RTT_PROBES} probes answered, {MIN_RTT_REPLIES} needed to grade",
            rtts.len()
        );
        return (Verdict::Incomplete, detail);
    };
    let (min, max) = rtt_window(configured_ms);
    let verdict = if (min..=max).contains(&median) {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    let detail = format!(
        "median {median:.1} ms over {} replies, window {min:.0}-{max:.0} ms",
        rtts.len()
    );
    (verdict, detail)
}

fn median(values: &[f64]) -> Option<f64> {
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    sorted.get(sorted.len() / 2).copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overlay_rtt_must_reflect_the_configured_round_trip() {
        let cases: &[(&[f64], f64, Verdict)] = &[
            (&[598.0, 612.0, 640.0, 605.0, 610.0], 600.0, Verdict::Pass),
            // Impairment weaker than configured, or a bypass of the router.
            (&[2.0, 540.0, 545.0, 548.0, 549.0], 600.0, Verdict::Fail),
            (&[700.0, 710.0, 730.0, 705.0, 720.0], 600.0, Verdict::Fail),
            (&[0.4, 1.2, 0.8, 0.5, 0.9], 0.0, Verdict::Pass),
            (&[150.0, 160.0, 1.0, 170.0, 155.0], 0.0, Verdict::Fail),
            // Too few replies to grade, however good they look.
            (&[], 600.0, Verdict::Incomplete),
            (&[600.0, 601.0, 602.0, 603.0], 600.0, Verdict::Incomplete),
        ];
        for (rtts, configured, expected) in cases {
            assert_eq!(
                rtt_verdict(rtts, *configured).0,
                *expected,
                "{rtts:?} vs {configured}"
            );
        }
    }

    #[test]
    fn silence_needs_a_full_unanswered_stream() {
        let reply = ping::Reply {
            unix_s: 0.0,
            seq: 1,
            rtt_ms: 1.0,
        };
        let cases: &[(u32, &[ping::Reply], Verdict)] = &[
            (SILENCE_PROBES, &[], Verdict::Pass),
            (SILENCE_PROBES, &[reply], Verdict::Fail),
            (SILENCE_PROBES - 1, &[], Verdict::Incomplete),
            (0, &[], Verdict::Incomplete),
        ];
        for (transmitted, replies, expected) in cases {
            let probe = Probe {
                transmitted: *transmitted,
                replies: replies.to_vec(),
            };
            assert_eq!(silence_verdict(&probe), *expected, "{probe:?}");
        }
    }
}
