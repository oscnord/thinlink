//! `lab check qgc-status`: the ground's freshness output as `QGroundControl` receives it. `checks`
//! plays the adapter's output on 192.168.232.40:5602 the way QGC plays a `MAVLink` camera's stream
//! (RTP/UDP, no jitter buffer) and reads each frame's stamp and state stamp. After a healthy window
//! it stalls the source with the vehicle's test-only hooks, a pause and then a freeze, and, under a
//! profile whose timeline blacks out the link, waits out the blackout: each time the output must
//! keep coming and say what is wrong before it shows live video again. It also asks the adapter's
//! camera component what QGC asks it (`camera`). `docs/measurement.md` defines every metric.

mod camera;

use std::collections::HashSet;
use std::net::SocketAddr;
use std::thread;
use std::time::{Duration, Instant};

use serde::Deserialize;
use serde_json::{Value, json};

use sat_teleop_media::MediaError;
use sat_teleop_media::output::FreshnessLimits;
use sat_teleop_media::receiver::{self, Observation, Source};
use sat_teleop_policy::freshness::State;
use sat_teleop_policy::stamp::plausible_age;

use crate::adaptation;
use crate::budget::{self, Budget};
use crate::check::{self, Results, Sample, Verdict, float, increase, percentile};
use crate::client::AgentClient;
use crate::control::blackout;
use crate::profile::Profile;
use crate::skeleton::Capture;

/// After the adapter decides a frame's state, its overlay and encoder, the local UDP hop and this
/// receiver's decoder take at most this long before the frame's age is read here.
const OUTPUT_ALLOWANCE_MS: f64 = 100.0;
const FIRST_FRAME_DEADLINE: Duration = Duration::from_secs(30);
/// A stall goes on this long after its last expected state showed, so a frame that looks live
/// meanwhile would be seen.
const HOLD: Duration = Duration::from_secs(1);
/// How much longer than its threshold an expected state may take to show.
const STALL_SLACK: Duration = Duration::from_secs(5);
/// Covers a keyframe interval plus a GEO round trip, and after a blackout the vehicle's return to
/// its rung before it (2 s of healthy answers, then a keyframe).
const RECOVERY_DEADLINE: Duration = Duration::from_secs(15);
const POLL_EVERY: Duration = Duration::from_millis(50);

const PAUSE: StallKind = StallKind {
    name: "pause",
    expected: &[
        (State::Stale, "pause_stale_ms"),
        (State::Disconnected, "pause_disconnected_ms"),
    ],
    recovery: "pause_recovery_ms",
    live: "pause_live_ms",
    gap: "pause_gap_max_ms",
};
const FREEZE: StallKind = StallKind {
    name: "freeze",
    expected: &[(State::Frozen, "freeze_frozen_ms")],
    recovery: "freeze_recovery_ms",
    live: "freeze_live_ms",
    gap: "freeze_gap_max_ms",
};
const BLACKOUT: StallKind = StallKind {
    name: "blackout",
    expected: &[
        (State::Stale, "blackout_stale_ms"),
        (State::Disconnected, "blackout_disconnected_ms"),
    ],
    recovery: "blackout_recovery_ms",
    live: "blackout_live_ms",
    gap: "blackout_gap_max_ms",
};

#[derive(Debug, thiserror::Error)]
pub enum FreshnessError {
    #[error(transparent)]
    Toml(#[from] toml::de::Error),
    #[error(transparent)]
    Invalid(#[from] MediaError),
}

/// `[freshness]` of a limits file such as `testbed/limits.toml`, validated.
pub fn freshness(text: &str) -> Result<FreshnessLimits, FreshnessError> {
    #[derive(Deserialize)]
    struct File {
        freshness: FreshnessLimits,
    }
    let file: File = toml::from_str(text)?;
    file.freshness.validate()?;
    Ok(file.freshness)
}

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// Where the ground adapter sends `checks` its video output.
    pub listen: SocketAddr,
    /// Where the ground adapter sends `checks` its telemetry, as a ground station.
    pub gcs_listen: SocketAddr,
    pub warmup: Duration,
    /// The healthy window.
    pub duration: Duration,
    /// Base URLs of the vehicle's metrics and hooks, and of the ground's metrics.
    pub vehicle_metrics: String,
    pub ground_metrics: String,
    /// The thresholds the adapter applies (`testbed/limits.toml`).
    pub freshness: FreshnessLimits,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "receiver": {
                "listen": self.listen,
                "transport": "rtp/udp, no jitter buffer",
                "decoder": receiver::DECODER,
            },
            "gcs_listen": self.gcs_listen,
            "warmup_s": self.warmup.as_secs_f64(),
            "duration_s": self.duration.as_secs_f64(),
            "freshness": self.freshness,
            "output_allowance_ms": OUTPUT_ALLOWANCE_MS,
            "hold_s": HOLD.as_secs_f64(),
            "recovery_deadline_s": RECOVERY_DEADLINE.as_secs_f64(),
            "vehicle_metrics": self.vehicle_metrics,
            "ground_metrics": self.ground_metrics,
        })
    }
}

/// A way to make the source go wrong, the states it must show in order, and its metric names:
/// recovery to a current picture (Live or `LowRate`), and to Live.
struct StallKind {
    name: &'static str,
    expected: &'static [(State, &'static str)],
    recovery: &'static str,
    live: &'static str,
    gap: &'static str,
}

/// When a stall began and was released; `error` if it could not be made.
struct Stall {
    kind: &'static StallKind,
    began: Instant,
    released: Instant,
    error: Option<String>,
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
    if let Err(e) = sat_teleop_media::init() {
        results.record("output_received", Verdict::Incomplete, e.to_string());
        return results;
    }
    let vehicle = AgentClient::new(&settings.vehicle_metrics);
    let ground = AgentClient::new(&settings.ground_metrics);
    // An aborted earlier run may have left the source paused or frozen; a refusal shows up again
    // when the stalls use the hooks.
    let _ = vehicle.post("/video/resume", "");
    let settled = adaptation::settle_at_top(&vehicle);
    adaptation::record_settled(&mut results, &settled);
    let started = Instant::now();
    let capture = Capture::start(Source::Rtp(settings.listen));
    let camera = camera::check(&mut results, settings.gcs_listen, &vehicle, &ground);
    thread::sleep(settings.warmup.saturating_sub(started.elapsed()));
    if !output_received(&mut results, &capture) {
        let captured = capture.finish();
        results.samples = samples(&captured.observations);
        return results;
    }
    let start = (Instant::now(), check::get_json(&ground, "/metrics"));
    thread::sleep(settings.duration);
    let end = (Instant::now(), check::get_json(&ground, "/metrics"));
    let window = capture.read(|all| Healthy::measure(all, start.0, end.0));
    record_healthy(&mut results, &window, &start.1, &end.1);
    let limits = settings.freshness;
    let pause = stall(
        &capture,
        &vehicle,
        &PAUSE,
        "/video/pause",
        ms(limits.disconnected_after_ms),
    );
    let freeze = stall(
        &capture,
        &vehicle,
        &FREEZE,
        "/video/freeze",
        ms(limits.frozen_after_ms),
    );
    let outage = blackout(profile).map(|times| link_blackout(agent, &capture, times));
    let captured = capture.finish();
    let all = &captured.observations;
    for stall in [Some(pause), Some(freeze), outage].into_iter().flatten() {
        record_stall(&mut results, &stall, all);
    }
    never_falsely_live(&mut results, all, &limits);
    results.metrics.insert(
        "observations_dropped",
        float(u64::try_from(captured.dropped).unwrap_or(u64::MAX)),
    );
    results.samples = samples(all);
    results.readings = json!({
        "camera": camera,
        "ground_window_start": reading(&start.1),
        "ground_window_end": reading(&end.1),
        "ground_at_finish": reading(&check::get_json(&ground, "/metrics")),
        "receiver_at_finish": captured.receiver,
    });
    budget::grade(&mut results, budget);
    results
}

fn reading(r: &Result<Value, String>) -> Value {
    match r {
        Ok(v) => v.clone(),
        Err(e) => json!({ "error": e }),
    }
}

fn ms(ms: u64) -> Duration {
    Duration::from_millis(ms)
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
}

fn state_of(o: &Observation) -> Option<State> {
    o.state.map(|s| s.state)
}

/// The frame tells the operator its picture is current: Live, or `LowRate`, which says so with
/// the rate.
fn current(o: &Observation) -> bool {
    matches!(state_of(o), Some(State::Live | State::LowRate))
}

/// The frame tells the operator its picture is old or no longer moving. `AgeUnknown` does not: it
/// says only that too few stamps read back, and a young picture may read Live after it.
fn stopped(o: &Observation) -> bool {
    matches!(
        state_of(o),
        Some(State::Stale | State::Frozen | State::Disconnected)
    )
}

/// Polls what arrived until `done` holds or `deadline` passes; returns `done`.
fn wait_until(capture: &Capture, deadline: Instant, done: impl Fn(&[Observation]) -> bool) -> bool {
    loop {
        if capture.read(|all| done(all)) {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        thread::sleep(POLL_EVERY);
    }
}

fn output_received(results: &mut Results, capture: &Capture) -> bool {
    let started = Instant::now();
    let deadline = started + FIRST_FRAME_DEADLINE;
    let name = "output_received";
    if wait_until(capture, deadline, |all| {
        all.iter().any(|o| o.state.is_some())
    }) {
        let detail = "frames with a state stamp arrive from the adapter";
        results.record(name, Verdict::Pass, detail);
        return true;
    }
    let detail = format!(
        "no frame with a state stamp within {} s after the warm-up",
        FIRST_FRAME_DEADLINE.as_secs()
    );
    results.record(name, Verdict::Fail, detail);
    false
}

/// The output in the healthy window. Only new frames (a readable, plausible stamp with a frame id
/// not seen before) count towards age.
#[derive(Debug, Default)]
pub(crate) struct Healthy {
    pub(crate) seconds: f64,
    pub(crate) frames: usize,
    /// Frames whose state stamp read back, and those that read Live.
    pub(crate) states: usize,
    pub(crate) live: usize,
    /// Ages of the new frames when they arrived here, sorted.
    pub(crate) ages_ms: Vec<f64>,
}

impl Healthy {
    pub(crate) fn measure(observations: &[Observation], start: Instant, end: Instant) -> Self {
        let mut window = Self {
            seconds: end.saturating_duration_since(start).as_secs_f64(),
            ..Self::default()
        };
        let mut seen = HashSet::new();
        for o in observations
            .iter()
            .filter(|o| (start..=end).contains(&o.received_at))
        {
            window.frames += 1;
            window.states += usize::from(o.state.is_some());
            window.live += usize::from(state_of(o) == Some(State::Live));
            let new = o
                .stamp
                .filter(|s| seen.insert(s.frame_id.0))
                .and_then(|s| plausible_age(o.received_ms, s));
            if let Some(age) = new {
                window.ages_ms.push(f64::from(age));
            }
        }
        window.ages_ms.sort_by(f64::total_cmp);
        window
    }
}

fn record_healthy(
    results: &mut Results,
    window: &Healthy,
    start: &Result<Value, String>,
    end: &Result<Value, String>,
) {
    let per_second = |n: usize| {
        (window.seconds > 0.0).then(|| float(u64::try_from(n).unwrap_or(u64::MAX)) / window.seconds)
    };
    let count = |n: usize| float(u64::try_from(n).unwrap_or(u64::MAX));
    let age_mean = mean(&window.ages_ms);
    let decode_new = increase(start, end, "/video/new_frames").filter(|n| *n > 0);
    let decode_total = increase(start, end, "/video/decode_age_ms_total");
    let decode_mean = decode_new
        .zip(decode_total)
        .map(|(n, total)| float(total) / float(n));
    let metrics = [
        ("output_frames_per_s", per_second(window.frames)),
        ("output_new_frames_per_s", per_second(window.ages_ms.len())),
        (
            "live_ratio",
            (window.states > 0).then(|| count(window.live) / count(window.states)),
        ),
        (
            "state_unreadable",
            Some(count(window.frames.saturating_sub(window.states))),
        ),
        ("output_age_p50_ms", percentile(&window.ages_ms, 50)),
        ("output_age_p95_ms", percentile(&window.ages_ms, 95)),
        ("output_age_max_ms", window.ages_ms.last().copied()),
        ("output_age_mean_ms", age_mean),
        ("decode_age_mean_ms", decode_mean),
        (
            "output_added_ms",
            age_mean
                .zip(decode_mean)
                .map(|(out, at_decode)| out - at_decode),
        ),
    ];
    results.metrics.extend(
        metrics
            .into_iter()
            .filter_map(|(name, value)| Some((name, value?))),
    );
    results.sample_counts.insert("window_frames", window.frames);
    results
        .sample_counts
        .insert("window_new_frames", window.ages_ms.len());
    let detail = format!(
        "{} of {} frames in {:.1} s read Live ({} new frames)",
        window.live,
        window.frames,
        window.seconds,
        window.ages_ms.len()
    );
    let verdict = if window.live > 0 {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record("healthy_live", verdict, detail);
}

fn mean(values: &[f64]) -> Option<f64> {
    let n = u64::try_from(values.len()).ok().filter(|n| *n > 0)?;
    Some(values.iter().sum::<f64>() / float(n))
}

/// Makes the source go wrong through a vehicle hook until the kind's last state showed (or its
/// threshold plus slack passed) and a little longer, then resumes it and waits for Live.
fn stall(
    capture: &Capture,
    vehicle: &AgentClient,
    kind: &'static StallKind,
    hook: &str,
    threshold: Duration,
) -> Stall {
    let began = Instant::now();
    if let Err(e) = vehicle.post(hook, "") {
        // The request may have landed all the same.
        let _ = vehicle.post("/video/resume", "");
        let error = Some(format!("cannot stall the source: {e}"));
        return Stall {
            kind,
            began,
            released: began,
            error,
        };
    }
    let last = kind.expected.last().map(|(state, _)| *state);
    let deadline = began + threshold + STALL_SLACK;
    let shown = |all: &[Observation]| {
        all.iter()
            .any(|o| o.received_at >= began && state_of(o) == last)
    };
    if wait_until(capture, deadline, shown) {
        thread::sleep(HOLD);
    }
    let released = Instant::now();
    let error = vehicle
        .post("/video/resume", "")
        .err()
        .map(|e| format!("cannot resume the source: {e}"));
    wait_for_live(capture, released);
    Stall {
        kind,
        began,
        released,
        error,
    }
}

/// Waits up to `RECOVERY_DEADLINE` after `released` for a frame that reads Live.
fn wait_for_live(capture: &Capture, released: Instant) {
    let live = |all: &[Observation]| {
        all.iter()
            .any(|o| o.received_at >= released && state_of(o) == Some(State::Live))
    };
    wait_until(capture, released + RECOVERY_DEADLINE, live);
}

/// Runs the profile's timeline, which blacks out both directions from `blackout_ms` to
/// `restore_ms`, and waits for Live after the restore.
fn link_blackout(
    agent: &AgentClient,
    capture: &Capture,
    (blackout_ms, restore_ms): (u64, u64),
) -> Stall {
    let started = Instant::now();
    if let Err(e) = agent.post("/timeline/start", "") {
        return Stall {
            kind: &BLACKOUT,
            began: started,
            released: started,
            error: Some(format!("cannot start the timeline: {e}")),
        };
    }
    let began = started + ms(blackout_ms);
    let released = started + ms(restore_ms);
    thread::sleep(released.saturating_duration_since(Instant::now()));
    wait_for_live(capture, released);
    Stall {
        kind: &BLACKOUT,
        began,
        released,
        error: None,
    }
}

/// What the output showed during a stall and after it.
#[derive(Debug, PartialEq)]
struct Indication {
    /// For each expected state, when it first showed after the stall began, in ms.
    shown_ms: Vec<Option<f64>>,
    /// Frames that read Live (or `LowRate`) after the first that said the picture was old or
    /// stopped (Stale, Frozen, Disconnected), before the release.
    live_after_alert: usize,
    /// From the release to the first Live (or `LowRate`) frame, and to the first Live one, in ms.
    recovery_ms: Option<f64>,
    live_ms: Option<f64>,
    /// Longest wait for an output frame between the start and the release, in ms.
    gap_max_ms: f64,
}

fn indicate(observations: &[Observation], stall: &Stall) -> Indication {
    let (began, released) = (stall.began, stall.released);
    let during: Vec<&Observation> = observations
        .iter()
        .filter(|o| (began..released).contains(&o.received_at))
        .collect();
    let after =
        |o: &Observation, from: Instant| millis(o.received_at.saturating_duration_since(from));
    let shown_ms = stall
        .kind
        .expected
        .iter()
        .map(|(state, _)| {
            during
                .iter()
                .find(|o| state_of(o) == Some(*state))
                .map(|o| after(o, began))
        })
        .collect();
    let live_after_alert = during
        .iter()
        .skip_while(|o| !stopped(o))
        .filter(|o| current(o))
        .count();
    let first_after = |shown: fn(&Observation) -> bool| {
        observations
            .iter()
            .find(|o| o.received_at >= released && shown(o))
            .map(|o| after(o, released))
    };
    let recovery_ms = first_after(current);
    let live_ms = first_after(|o| state_of(o) == Some(State::Live));
    let arrivals = during.iter().map(|o| o.received_at);
    let (gap, last) = arrivals.fold((Duration::ZERO, began), |(gap, last), at| {
        (gap.max(at.saturating_duration_since(last)), at)
    });
    Indication {
        shown_ms,
        live_after_alert,
        recovery_ms,
        live_ms,
        gap_max_ms: millis(gap.max(released.saturating_duration_since(last))),
    }
}

fn record_stall(results: &mut Results, stall: &Stall, observations: &[Observation]) {
    let kind = stall.kind;
    let indicated = format!("{}_indicated", kind.name);
    let recovered = format!("{}_recovered", kind.name);
    if let Some(error) = &stall.error {
        results.record(indicated, Verdict::Incomplete, error.clone());
        results.record(recovered, Verdict::Incomplete, error.clone());
        return;
    }
    let indication = indicate(observations, stall);
    for ((_, metric), shown) in kind.expected.iter().zip(&indication.shown_ms) {
        if let Some(shown) = shown {
            results.metrics.insert(metric, *shown);
        }
    }
    if let Some(recovery) = indication.recovery_ms {
        results.metrics.insert(kind.recovery, recovery);
    }
    if let Some(live) = indication.live_ms {
        results.metrics.insert(kind.live, live);
    }
    results.metrics.insert(kind.gap, indication.gap_max_ms);
    let (verdict, detail) = indication_verdict(kind, &indication);
    results.record(indicated, verdict, detail);
    let live = indication.live_ms.map_or_else(
        || format!("not Live within {} s", RECOVERY_DEADLINE.as_secs()),
        |ms| format!("Live after {ms:.0} ms"),
    );
    let (verdict, detail) = match indication.recovery_ms {
        Some(ms) => (
            Verdict::Pass,
            format!("current {ms:.0} ms after the release; {live}"),
        ),
        None => (
            Verdict::Fail,
            format!(
                "no Live or LowRate frame within {} s of the release",
                RECOVERY_DEADLINE.as_secs()
            ),
        ),
    };
    results.record(recovered, verdict, detail);
}

/// Every expected state showed, in order, and nothing read Live again until the release.
fn indication_verdict(kind: &StallKind, indication: &Indication) -> (Verdict, String) {
    let shown: Vec<String> = kind
        .expected
        .iter()
        .zip(&indication.shown_ms)
        .map(|((state, _), ms)| match ms {
            Some(ms) => format!("{} after {ms:.0} ms", state.name()),
            None => format!("no {}", state.name()),
        })
        .collect();
    let detail = format!(
        "{}; {} Live frames after the first alert; longest wait for a frame {:.0} ms",
        shown.join(", "),
        indication.live_after_alert,
        indication.gap_max_ms
    );
    let times: Option<Vec<f64>> = indication.shown_ms.iter().copied().collect();
    let in_order = times.is_some_and(|t| t.windows(2).all(|w| w.first() <= w.last()));
    let verdict = if in_order && indication.live_after_alert == 0 {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    (verdict, detail)
}

/// No frame reads Live (or `LowRate`) while its own stamp, as received here, is older than that
/// state's stale threshold plus what the output path adds after the adapter's decision.
fn never_falsely_live(
    results: &mut Results,
    observations: &[Observation],
    limits: &FreshnessLimits,
) {
    let limit = |state: State| {
        let stale_after_ms = match state {
            State::LowRate => limits.low_rate_stale_after_ms,
            _ => limits.stale_after_ms,
        };
        float(stale_after_ms) + OUTPUT_ALLOWANCE_MS
    };
    let current_ages: Vec<(State, f64)> = observations
        .iter()
        .filter(|o| current(o))
        .filter_map(|o| Some((state_of(o)?, plausible_age(o.received_ms, o.stamp?)?)))
        .map(|(state, age)| (state, f64::from(age)))
        .collect();
    let too_old = current_ages
        .iter()
        .filter(|(state, age)| *age > limit(*state))
        .count();
    let live_ages: Vec<f64> = current_ages
        .iter()
        .filter(|(state, _)| *state == State::Live)
        .map(|(_, age)| *age)
        .collect();
    let oldest = live_ages.iter().copied().fold(None, |max: Option<f64>, a| {
        Some(max.map_or(a, |m| m.max(a)))
    });
    if let Some(oldest) = oldest {
        results.metrics.insert("live_age_max_ms", oldest);
    }
    let name = "never_falsely_live";
    let Some(oldest) = oldest else {
        results.record(
            name,
            Verdict::Incomplete,
            "no Live frame with a readable stamp",
        );
        return;
    };
    let detail = format!(
        "{too_old} of {} Live or LowRate frames older than their stale threshold + {:.0} ms; \
         oldest Live {oldest:.0} ms",
        current_ages.len(),
        OUTPUT_ALLOWANCE_MS
    );
    let verdict = if too_old == 0 {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record(name, verdict, detail);
}

/// One row per output frame: its state (or `no_state`), its frame id and its age here.
fn samples(observations: &[Observation]) -> Vec<Sample> {
    observations
        .iter()
        .map(|o| Sample {
            unix_s: float(o.received_ms) / 1000.0,
            series: state_of(o).map_or("no_state", State::name),
            seq: o.stamp.map_or(0, |s| s.frame_id.0),
            value: o
                .stamp
                .and_then(|s| plausible_age(o.received_ms, s))
                .map_or(f64::NAN, f64::from),
        })
        .collect()
}
