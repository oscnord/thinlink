//! `lab check adaptation`: the vehicle's video adapting to the uplink. `checks` plays the relay's
//! stream like the skeleton check, samples the vehicle's ladder and the router's counters, and runs
//! the profile's timeline (`bandwidth-steps` steps the uplink rate down and up). After each
//! downward step the vehicle must offer less than the new cap within a deadline and the frame age
//! must come back down: no backlog. A second viewer must not change what the vehicle sends.
//! `docs/testbed.md#adaptation-check` defines every metric.

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::thread;
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use sat_teleop_video::receiver::{Observation, Source};
use sat_teleop_video::unix_ms;
use sat_teleop_video_core::stamp::plausible_age;

use crate::budget::{self, Budget};
use crate::check::{self, Results, Sample, Verdict, float, percentile};
use crate::client::AgentClient;
use crate::profile::{Action, Direction, Profile};
use crate::skeleton::Capture;

/// The ladder counts as settled once it has not changed for this long (at the rung expected):
/// longer than the controller's dwell (`dwell_ms` in `[adapt]`, 10 s), so a ladder still climbing
/// is not taken for a settled one.
const SETTLE_QUIET: Duration = Duration::from_secs(15);
/// The window for a check that needs the top rung: a ladder at the top only leaves it by a fall,
/// and the controller falls on silence after `stale_after_ms` (3 s) or on a standing problem within
/// `min_change_interval_ms` (2 s), so the climb `SETTLE_QUIET` waits out cannot be under way.
const SETTLE_QUIET_AT_TOP: Duration = Duration::from_secs(6);
/// A cut, the base window rolling over and five climbs fit in this.
const SETTLE_MAX: Duration = Duration::from_secs(120);
const POLL_EVERY: Duration = Duration::from_millis(500);
/// Each third of the second-viewer comparison; a whole number of 2 s keyframe intervals.
const VIEWER_WINDOW: Duration = Duration::from_secs(6);
/// The second receiver's time to connect and decode a keyframe.
const VIEWER_CONNECT: Duration = Duration::from_secs(2);
/// The base frame age is taken over this before the timeline starts, and a phase's final frame
/// age over this before its end.
const AGE_WINDOW: Duration = Duration::from_secs(5);
/// After a downward step, a frame this much older than the phase's final level still waited in a
/// queue, and a wait for a new frame this much longer than the phase's final interval was one
/// (a drain sends no video).
const RECOVERY_MARGIN_MS: f64 = 150.0;
/// Observation goes on this long after the timeline's last step.
const TAIL: Duration = Duration::from_secs(20);
/// After a downward step the vehicle offers less than the new cap within this.
pub const SETTLE_DEADLINE: Duration = Duration::from_secs(6);
/// The offered uplink is judged over this: one keyframe interval and two of SITL's once-a-second
/// telemetry bursts, so every window holds the same bursts wherever it starts. Readings come a
/// second apart, so the windows slide by a second.
const OFFERED_WINDOW: usize = 2;

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// RTSP URL of the relay's stream.
    pub url: String,
    pub warmup: Duration,
    /// The observation, for a profile without a timeline.
    pub duration: Duration,
    pub vehicle_metrics: String,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "url": self.url,
            "warmup_s": self.warmup.as_secs_f64(),
            "duration_s": self.duration.as_secs_f64(),
            "vehicle_metrics": self.vehicle_metrics,
            "settle_quiet_s": SETTLE_QUIET.as_secs_f64(),
            "settle_quiet_at_top_s": SETTLE_QUIET_AT_TOP.as_secs_f64(),
            "settle_max_s": SETTLE_MAX.as_secs_f64(),
            "viewer_window_s": VIEWER_WINDOW.as_secs_f64(),
            "age_window_s": AGE_WINDOW.as_secs_f64(),
            "recovery_margin_ms": RECOVERY_MARGIN_MS,
            "settle_deadline_s": SETTLE_DEADLINE.as_secs_f64(),
            "offered_window_s": OFFERED_WINDOW,
            "tail_s": TAIL.as_secs_f64(),
        })
    }
}

/// Where the ladder must settle before a run measures it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Expect {
    /// The best rung: the profile's uplink carries it.
    Top,
    /// Wherever the profile lets it settle.
    Any,
}

/// How long the vehicle's ladder took to settle, whether it stopped changing and where.
pub(crate) struct Settled {
    pub(crate) waited: Duration,
    quiet_window: Duration,
    pub(crate) quiet: bool,
    pub(crate) rung: Option<u64>,
    pub(crate) expect: Expect,
}

impl Settled {
    fn expected(&self) -> bool {
        self.expect == Expect::Any || self.rung == Some(0)
    }
}

/// `settle` for a check that does not test adaptation: it needs the top rung and is not waiting
/// for a climb.
pub(crate) fn settle_at_top(vehicle: &AgentClient) -> Settled {
    settle(vehicle, Expect::Top, SETTLE_QUIET_AT_TOP)
}

/// `settle` for a check that grades the picture wherever the profile lets the ladder settle.
pub(crate) fn settle_anywhere(vehicle: &AgentClient) -> Settled {
    settle(vehicle, Expect::Any, SETTLE_QUIET)
}

/// Waits until the vehicle's ladder has not changed for `quiet_window` at the rung expected, at
/// most `SETTLE_MAX`. A profile switch that changes the base delay, or a forwarding cut by the
/// check before, sets it moving for a while, and a capacity measured before holds a climb.
pub(crate) fn settle(vehicle: &AgentClient, expect: Expect, quiet_window: Duration) -> Settled {
    let started = Instant::now();
    let mut seen = None;
    let mut quiet_since = started;
    loop {
        let metrics = check::get_json(vehicle, "/metrics").ok();
        let at = |pointer: &str| metrics.as_ref()?.pointer(pointer)?.as_u64();
        let changes = at("/adapt/changes");
        if changes.is_none() || changes != seen {
            seen = changes;
            quiet_since = Instant::now();
        }
        let settled = Settled {
            waited: started.elapsed(),
            quiet_window,
            quiet: quiet_since.elapsed() >= quiet_window,
            rung: at("/adapt/rung"),
            expect,
        };
        if (settled.quiet && settled.expected()) || settled.waited >= SETTLE_MAX {
            return settled;
        }
        thread::sleep(POLL_EVERY);
    }
}

/// Records how long the ladder took to settle and where (`ladder_settled`).
pub(crate) fn record_settled(results: &mut Results, settled: &Settled) {
    let ms = settled.waited.as_secs_f64() * 1000.0;
    results.metrics.insert("ladder_settle_ms", ms);
    if let Some(rung) = settled.rung {
        results.metrics.insert("settled_rung", float(rung));
    }
    let rung = settled
        .rung
        .map_or_else(|| "an unknown rung".to_owned(), |r| format!("rung {r}"));
    let (verdict, detail) = match (settled.quiet, settled.expected()) {
        (true, true) => (
            Verdict::Pass,
            format!(
                "no rung change for {} s, at {rung}",
                settled.quiet_window.as_secs()
            ),
        ),
        (true, false) => (
            Verdict::Incomplete,
            format!("settled at {rung}, not the top rung the profile carries"),
        ),
        (false, _) => (
            Verdict::Incomplete,
            format!(
                "the rung still changed after {} s: the run measures a transient",
                SETTLE_MAX.as_secs()
            ),
        ),
    };
    results.record("ladder_settled", verdict, detail);
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
    if let Err(e) = sat_teleop_video::init() {
        results.record("video_received", Verdict::Incomplete, e.to_string());
        return results;
    }
    let vehicle = AgentClient::new(&settings.vehicle_metrics);
    // An aborted qgc-status run may have left the source paused or frozen.
    let _ = vehicle.post("/video/resume", "");
    let steps = rate_steps(profile);
    // A timeline starts from a baseline that carries the top rung; without one the profile
    // decides where the ladder settles.
    let expect = if steps.is_empty() {
        Expect::Any
    } else {
        Expect::Top
    };
    let settled = settle(&vehicle, expect, SETTLE_QUIET);
    record_settled(&mut results, &settled);
    let capture = Capture::start(Source::Rtsp(settings.url.clone()));
    thread::sleep(settings.warmup);
    let viewer = second_viewer(agent, &vehicle, &settings.url);
    record_viewer(&mut results, &viewer);
    let base_end = Instant::now();
    let base_from = base_end.checked_sub(AGE_WINDOW).unwrap_or(base_end);
    let base = capture.read(|all| ages_ms(&new_frames(all), base_from, base_end));
    let base_age_ms = percentile(&base, 50);
    let started = start_timeline(agent, &steps);
    let observe_for = steps
        .last()
        .map_or(settings.duration, |(at_ms, _)| ms(*at_ms) + TAIL);
    let polls = poll(agent, &vehicle, started.at, observe_for);
    let caps = caps(profile, &steps, &started, agent);
    let captured = capture.finish();
    let frames = new_frames(&captured.observations);
    let windows = offered(&polls);
    let observed = Span {
        from: started.at,
        to: polls.last().map_or(started.at, |p| p.at),
    };
    record_frames(&mut results, &frames, observed);
    record_ladder(&mut results, &polls);
    if let Some(error) = &started.error {
        results.record("timeline", Verdict::Fail, error.clone());
    }
    let base_age_ms = base_age_ms.filter(|_| !frames.is_empty());
    record_steps(
        &mut results,
        &caps,
        &windows,
        &frames,
        base_age_ms,
        observed,
    );
    results.samples = samples(&frames, &windows, &caps, &polls);
    results.readings = json!({
        "viewer": viewer.to_json(),
        "caps": caps.iter().map(|c| json!({
            "at_s": c.at.saturating_duration_since(started.at).as_secs_f64(),
            "bytes_per_s": c.bytes_per_s,
        })).collect::<Vec<_>>(),
        "vehicle_at_finish": check::get_json(&vehicle, "/metrics").unwrap_or(Value::Null),
        "receiver_at_finish": captured.receiver,
    });
    budget::grade(&mut results, budget);
    results
}

fn ms(ms: u64) -> Duration {
    Duration::from_millis(ms)
}

fn millis(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

/// A stretch of the run.
#[derive(Debug, Clone, Copy)]
struct Span {
    from: Instant,
    to: Instant,
}

impl Span {
    fn contains(self, at: Instant) -> bool {
        (self.from..self.to).contains(&at)
    }

    fn seconds(self) -> f64 {
        self.to.saturating_duration_since(self.from).as_secs_f64()
    }
}

/// A new frame (readable, plausible stamp, id not seen before): when it arrived and its age.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Frame {
    pub(crate) at: Instant,
    unix_ms: u64,
    id: u32,
    pub(crate) age_ms: f64,
}

pub(crate) fn new_frames(observations: &[Observation]) -> Vec<Frame> {
    let mut seen = HashSet::new();
    observations
        .iter()
        .filter_map(|o| {
            let stamp = o.stamp?;
            let age = plausible_age(o.received_ms, stamp)?;
            seen.insert(stamp.frame_id.0).then_some(Frame {
                at: o.received_at,
                unix_ms: o.received_ms,
                id: stamp.frame_id.0,
                age_ms: f64::from(age),
            })
        })
        .collect()
}

/// The ages of the frames that arrived in `[from, to)`, sorted.
pub(crate) fn ages_ms(frames: &[Frame], from: Instant, to: Instant) -> Vec<f64> {
    let span = Span { from, to };
    let mut ages: Vec<f64> = frames
        .iter()
        .filter(|f| span.contains(f.at))
        .map(|f| f.age_ms)
        .collect();
    ages.sort_by(f64::total_cmp);
    ages
}

/// The vehicle's offered uplink in three equal windows: with one viewer, with two, and with one
/// again. The encoder's rate drifts for a while after a rung change; the mean of the windows
/// before and after cancels that.
#[derive(Debug, Default)]
struct Viewer {
    one_bytes_per_s: Option<f64>,
    two_bytes_per_s: Option<f64>,
    again_bytes_per_s: Option<f64>,
    rung_changed: bool,
    second_frames: usize,
}

impl Viewer {
    fn to_json(&self) -> Value {
        json!({
            "one_bytes_per_s": self.one_bytes_per_s,
            "two_bytes_per_s": self.two_bytes_per_s,
            "again_bytes_per_s": self.again_bytes_per_s,
            "rung_changed": self.rung_changed,
            "second_receiver_frames": self.second_frames,
        })
    }
}

fn second_viewer(agent: &AgentClient, vehicle: &AgentClient, url: &str) -> Viewer {
    let changes = || {
        check::get_json(vehicle, "/metrics")
            .ok()
            .and_then(|m| m.pointer("/adapt/changes")?.as_u64())
    };
    let before = changes();
    let one = offered_over(agent, VIEWER_WINDOW);
    let second = Capture::start(Source::Rtsp(url.to_owned()));
    thread::sleep(VIEWER_CONNECT);
    let two = offered_over(agent, VIEWER_WINDOW);
    let captured = second.finish();
    let again = offered_over(agent, VIEWER_WINDOW);
    Viewer {
        one_bytes_per_s: one,
        two_bytes_per_s: two,
        again_bytes_per_s: again,
        rung_changed: before.is_none() || before != changes(),
        second_frames: new_frames(&captured.observations).len(),
    }
}

/// The vehicle's offered uplink over `window`, from the router.
fn offered_over(agent: &AgentClient, window: Duration) -> Option<f64> {
    let start = offered_bytes(agent)?;
    thread::sleep(window);
    let end = offered_bytes(agent)?;
    let seconds = end.0.saturating_duration_since(start.0).as_secs_f64();
    Some(float(end.1.checked_sub(start.1)?) / seconds)
}

/// Bytes the router received from `vehicle_net` (everything the vehicle offers the uplink, before
/// the router's queue and rate limit), and when: just after the reading, since the request around
/// it takes a varying while (it runs `tc` first).
pub(crate) fn offered_bytes(agent: &AgentClient) -> Option<(Instant, u64)> {
    let bytes = check::get_json(agent, "/counters")
        .ok()?
        .pointer("/downlink/net/rx_bytes")?
        .as_u64()?;
    Some((Instant::now(), bytes))
}

fn record_viewer(results: &mut Results, viewer: &Viewer) {
    let name = "second_viewer";
    let readings = (
        viewer.one_bytes_per_s,
        viewer.two_bytes_per_s,
        viewer.again_bytes_per_s,
    );
    let (Some(one), Some(two), Some(again)) = readings else {
        let detail = "the router counters did not read";
        results.record(name, Verdict::Incomplete, detail);
        return;
    };
    results.metrics.insert("viewer_one_bytes_per_s", one);
    results.metrics.insert("viewer_two_bytes_per_s", two);
    results.metrics.insert("viewer_again_bytes_per_s", again);
    let alone = f64::midpoint(one, again);
    let detail = format!(
        "{two:.0} B/s with two viewers, {one:.0} and {again:.0} B/s with one before and after \
         ({} frames to the second)",
        viewer.second_frames
    );
    if viewer.rung_changed || viewer.second_frames == 0 || alone <= 0.0 {
        let why = "the rung changed, or the second viewer got nothing";
        results.record(name, Verdict::Incomplete, format!("{detail}: {why}"));
        return;
    }
    let change = (two - alone).abs() / alone * 100.0;
    results.metrics.insert("viewer_change_percent", change);
    let detail = format!("{detail}: {change:.1} % from one viewer, graded by the budget");
    results.record(name, Verdict::Pass, detail);
}

/// The profile's uplink rate steps: when (ms into the timeline) and the new rate in kbit/s.
fn rate_steps(profile: &Profile) -> Vec<(u64, u32)> {
    profile
        .timeline
        .iter()
        .filter(|t| matches!(t.direction, Direction::Uplink | Direction::Both))
        .filter_map(|t| match t.action {
            Action::SetRate { rate_kbit } => Some((t.at_ms, rate_kbit)),
            Action::Blackout | Action::Restore => None,
        })
        .collect()
}

struct Started {
    at: Instant,
    error: Option<String>,
}

fn start_timeline(agent: &AgentClient, steps: &[(u64, u32)]) -> Started {
    let at = Instant::now();
    let error = (!steps.is_empty())
        .then(|| agent.post("/timeline/start", "").err())
        .flatten()
        .map(|e| format!("cannot start the timeline: {e}"));
    Started { at, error }
}

/// The uplink cap from a moment on, in bytes per second on the wire.
#[derive(Debug, Clone, Copy)]
struct Cap {
    at: Instant,
    bytes_per_s: f64,
}

/// The profile's base rate, then each step when the router applied it (`GET /timeline`), or when
/// it was planned if the router did not say.
fn caps(
    profile: &Profile,
    steps: &[(u64, u32)],
    started: &Started,
    agent: &AgentClient,
) -> Vec<Cap> {
    let applied: BTreeMap<u64, u64> = check::get_json(agent, "/timeline")
        .ok()
        .and_then(|t| t.get("transitions").cloned())
        .and_then(|t| t.as_array().cloned())
        .unwrap_or_default()
        .iter()
        .filter(|t| t["direction"] == "uplink")
        .filter_map(|t| Some((t["planned_ms"].as_u64()?, t["actual_ms"].as_u64()?)))
        .collect();
    let cap = |kbit: u32| float(u64::from(kbit)) * 125.0;
    let base = Cap {
        at: started.at,
        bytes_per_s: cap(profile.uplink.rate_kbit),
    };
    let stepped = steps.iter().map(|(planned, kbit)| Cap {
        at: started.at + ms(applied.get(planned).copied().unwrap_or(*planned)),
        bytes_per_s: cap(*kbit),
    });
    std::iter::once(base).chain(stepped).collect()
}

/// One sample of the vehicle (every poll) and the router (every second poll).
#[derive(Debug, Clone)]
struct Poll {
    at: Instant,
    unix_ms: u64,
    rung: Option<u64>,
    group: Option<String>,
    changes: Option<u64>,
    queue_delay_ms: Option<u64>,
    /// From the requested rung to its first frame, when the newest request was applied.
    apply_ms: Option<u64>,
    /// The vehicle's newest ladder changes: when on its monotonic clock, and why.
    recent_changes: Vec<(u64, String)>,
    offered: Option<(Instant, u64)>,
}

fn poll(agent: &AgentClient, vehicle: &AgentClient, from: Instant, length: Duration) -> Vec<Poll> {
    let mut polls = Vec::new();
    let mut tick = 0_u32;
    while from.elapsed() < length {
        let metrics = check::get_json(vehicle, "/metrics").ok();
        let at = |pointer: &str| metrics.as_ref()?.pointer(pointer)?.as_u64();
        let applied = metrics.as_ref().and_then(|m| {
            let same = m.pointer("/requested/seq")? == m.pointer("/applied/seq")?;
            m.pointer("/applied/after_request_ms")?
                .as_u64()
                .filter(|_| same)
        });
        polls.push(Poll {
            at: Instant::now(),
            unix_ms: unix_ms(),
            rung: at("/adapt/rung"),
            group: metrics
                .as_ref()
                .and_then(|m| m.pointer("/adapt/group")?.as_str().map(str::to_owned)),
            changes: at("/adapt/changes"),
            queue_delay_ms: at("/adapt/queue_delay_ms"),
            apply_ms: applied,
            recent_changes: metrics
                .as_ref()
                .and_then(|m| m.pointer("/adapt/recent_changes")?.as_array().cloned())
                .unwrap_or_default()
                .iter()
                .filter_map(|c| Some((c["at_ms"].as_u64()?, c["reason"].as_str()?.to_owned())))
                .collect(),
            offered: tick
                .is_multiple_of(2)
                .then(|| offered_bytes(agent))
                .flatten(),
        });
        tick += 1;
        thread::sleep(POLL_EVERY);
    }
    polls
}

/// The offered uplink between router readings `OFFERED_WINDOW` apart.
#[derive(Debug, Clone, Copy)]
struct Window {
    span: Span,
    unix_ms: u64,
    bytes_per_s: f64,
}

fn offered(polls: &[Poll]) -> Vec<Window> {
    let readings: Vec<(Instant, u64, u64)> = polls
        .iter()
        .filter_map(|p| p.offered.map(|(at, bytes)| (at, p.unix_ms, bytes)))
        .collect();
    readings
        .windows(OFFERED_WINDOW + 1)
        .filter_map(|run| {
            let (Some((from, _, start)), Some((to, unix_ms, end))) = (run.first(), run.last())
            else {
                return None;
            };
            let seconds = to.saturating_duration_since(*from).as_secs_f64();
            (seconds > 0.0).then(|| Window {
                span: Span {
                    from: *from,
                    to: *to,
                },
                unix_ms: *unix_ms,
                bytes_per_s: float(end.saturating_sub(*start)) / seconds,
            })
        })
        .collect()
}

fn record_frames(results: &mut Results, frames: &[Frame], observed: Span) {
    let ages = ages_ms(frames, observed.from, observed.to);
    let seconds = observed.seconds();
    let metrics = [
        ("frame_age_p50_ms", percentile(&ages, 50)),
        ("frame_age_p95_ms", percentile(&ages, 95)),
        ("frame_age_max_ms", ages.last().copied()),
        (
            "new_frames_per_s",
            (seconds > 0.0).then(|| float(ages.len() as u64) / seconds),
        ),
    ];
    results
        .metrics
        .extend(metrics.into_iter().filter_map(|(n, v)| Some((n, v?))));
    results.sample_counts.insert("new_frames", ages.len());
    let detail = format!("{} new frames in {seconds:.1} s", ages.len());
    let verdict = if ages.is_empty() {
        Verdict::Fail
    } else {
        Verdict::Pass
    };
    results.record("video_received", verdict, detail);
}

/// Rung changes per minute, time per rung group and the longest wait from a requested rung to its
/// first frame.
fn record_ladder(results: &mut Results, polls: &[Poll]) {
    let changes: Vec<u64> = polls.iter().filter_map(|p| p.changes).collect();
    let minutes = polls.first().zip(polls.last()).map_or(0.0, |(a, b)| {
        b.at.saturating_duration_since(a.at).as_secs_f64() / 60.0
    });
    if let (Some(first), Some(last)) = (changes.first(), changes.last()) {
        let changed = float(last.saturating_sub(*first));
        results.metrics.insert("transitions", changed);
        if minutes > 0.0 {
            results
                .metrics
                .insert("transitions_per_min", changed / minutes);
        }
    }
    let mut seconds: BTreeMap<&str, f64> = BTreeMap::new();
    for pair in polls.windows(2) {
        let [p, next] = pair else { continue };
        let Some(group) = p.group.as_deref() else {
            continue;
        };
        *seconds.entry(group).or_default() += next.at.saturating_duration_since(p.at).as_secs_f64();
    }
    for (group, metric) in [
        ("normal", "time_normal_s"),
        ("constrained", "time_constrained_s"),
        ("awareness_only", "time_awareness_only_s"),
    ] {
        results
            .metrics
            .insert(metric, seconds.get(group).copied().unwrap_or(0.0));
    }
    if let Some(apply) = polls.iter().filter_map(|p| p.apply_ms).max() {
        results.metrics.insert("apply_max_ms", float(apply));
    }
    let drains = drains_ms(polls);
    results.metrics.insert("drains", float(drains.len() as u64));
    let longest = drains.iter().max().copied().unwrap_or(0);
    results.metrics.insert("drain_max_ms", float(longest));
}

/// How long each drain during the observation kept the video paused: from the drain to the change
/// after it (the resume, or a fall to the floor), on the vehicle's monotonic clock. The vehicle
/// lists its last changes, so those the first reading already listed came before; a drain still
/// under way at the end does not count.
fn drains_ms(polls: &[Poll]) -> Vec<u64> {
    let before: BTreeSet<&(u64, String)> = polls
        .first()
        .map(|p| p.recent_changes.iter().collect())
        .unwrap_or_default();
    let changes: BTreeSet<&(u64, String)> = polls
        .iter()
        .flat_map(|p| &p.recent_changes)
        .filter(|change| !before.contains(change))
        .collect();
    let changes: Vec<&(u64, String)> = changes.into_iter().collect();
    changes
        .windows(2)
        .filter_map(|pair| match pair {
            [(drain, reason), (end, _)] if reason == "drain" => Some(end.saturating_sub(*drain)),
            _ => None,
        })
        .collect()
}

/// What followed one downward step of the uplink rate.
#[derive(Debug, Default)]
struct Step {
    /// From the step to the end of the last window above the new cap; 0 when none was.
    settle_ms: Option<f64>,
    /// Windows above the cap that began after `SETTLE_DEADLINE`.
    over_cap_windows: usize,
    windows: usize,
    /// The median frame age over the phase's last `AGE_WINDOW`, above the base.
    age_over_base_ms: Option<f64>,
    /// From the step to the last new frame that came late: older than the phase's final level plus
    /// the margin, or after a wait longer than the phase's final interval plus the margin.
    age_recovery_ms: Option<f64>,
}

fn record_steps(
    results: &mut Results,
    caps: &[Cap],
    windows: &[Window],
    frames: &[Frame],
    base_age_ms: Option<f64>,
    observed: Span,
) {
    if let Some(base) = base_age_ms {
        results.metrics.insert("base_age_ms", base);
    }
    let mut maxima: BTreeMap<&'static str, f64> = BTreeMap::new();
    let mut downward = 0;
    for (i, pair) in caps.windows(2).enumerate() {
        let [before, cap] = pair else { continue };
        if cap.bytes_per_s >= before.bytes_per_s {
            continue;
        }
        downward += 1;
        let end = caps.get(i + 2).map_or(observed.to, |next| next.at);
        let phase = Span {
            from: cap.at,
            to: end,
        };
        let step = measure_step(phase, cap.bytes_per_s, windows, frames, base_age_ms);
        record_step(results, downward, cap, &step);
        let metrics = [
            ("settle_max_ms", step.settle_ms),
            (
                "over_cap_windows",
                Some(float(step.over_cap_windows as u64)),
            ),
            ("age_over_base_max_ms", step.age_over_base_ms),
            ("age_recovery_max_ms", step.age_recovery_ms),
        ];
        for (name, value) in metrics {
            let Some(value) = value else { continue };
            let max = maxima.entry(name).or_insert(value);
            *max = max.max(value);
        }
    }
    results.metrics.extend(maxima);
}

fn measure_step(
    phase: Span,
    cap: f64,
    windows: &[Window],
    frames: &[Frame],
    base_age_ms: Option<f64>,
) -> Step {
    let inside: Vec<&Window> = windows
        .iter()
        .filter(|w| w.span.from >= phase.from && w.span.to <= phase.to)
        .collect();
    let over = |w: &&&Window| w.bytes_per_s > cap;
    let last_over = inside.iter().rev().find(over);
    let ends_over = inside.last().is_some_and(|w| w.bytes_per_s > cap);
    let settle_ms = (!inside.is_empty() && !ends_over).then(|| {
        last_over.map_or(0.0, |w| {
            millis(w.span.to.saturating_duration_since(phase.from))
        })
    });
    let deadline = phase.from + SETTLE_DEADLINE;
    let over_cap_windows = inside
        .iter()
        .filter(|w| w.span.from >= deadline)
        .filter(over)
        .count();
    let final_from = phase.to.checked_sub(AGE_WINDOW).unwrap_or(phase.from);
    let final_span = Span {
        from: final_from.max(phase.from),
        to: phase.to,
    };
    let final_age = percentile(&ages_ms(frames, final_span.from, final_span.to), 50);
    let final_interval = median_interval_ms(frames, final_span);
    let last_late = final_age.zip(final_interval).and_then(|(level, interval)| {
        frames
            .windows(2)
            .filter_map(|pair| match pair {
                [before, f] if phase.contains(f.at) => Some((before, f)),
                _ => None,
            })
            .filter(|(before, f)| {
                let waited = millis(f.at.saturating_duration_since(before.at));
                f.age_ms > level + RECOVERY_MARGIN_MS || waited > interval + RECOVERY_MARGIN_MS
            })
            .map(|(_, f)| f.at)
            .next_back()
    });
    Step {
        settle_ms,
        over_cap_windows,
        windows: inside.len(),
        age_over_base_ms: final_age.zip(base_age_ms).map(|(age, base)| age - base),
        age_recovery_ms: final_age
            .map(|_| last_late.map_or(0.0, |at| millis(at.saturating_duration_since(phase.from)))),
    }
}

/// The median wait between new frames that arrived in `span`.
fn median_interval_ms(frames: &[Frame], span: Span) -> Option<f64> {
    let inside: Vec<&Frame> = frames.iter().filter(|f| span.contains(f.at)).collect();
    let mut intervals: Vec<f64> = inside
        .windows(2)
        .filter_map(|pair| match pair {
            [before, f] => Some(millis(f.at.saturating_duration_since(before.at))),
            _ => None,
        })
        .collect();
    intervals.sort_by(f64::total_cmp);
    percentile(&intervals, 50)
}

fn record_step(results: &mut Results, n: usize, cap: &Cap, step: &Step) {
    let name = format!("step_{n}_offered_below_cap");
    let (verdict, detail) = match step.settle_ms {
        _ if step.windows == 0 => (Verdict::Incomplete, "no router reading".to_owned()),
        Some(ms) => (
            Verdict::Pass,
            format!(
                "below {:.0} B/s {ms:.0} ms after the step; {} of {} windows over it after {} s",
                cap.bytes_per_s,
                step.over_cap_windows,
                step.windows,
                SETTLE_DEADLINE.as_secs()
            ),
        ),
        None => (
            Verdict::Fail,
            format!(
                "still above {:.0} B/s when the phase ended",
                cap.bytes_per_s
            ),
        ),
    };
    results.record(name, verdict, detail);
    let name = format!("step_{n}_age_recovered");
    let (verdict, detail) = match (step.age_recovery_ms, step.age_over_base_ms) {
        (Some(recovery), Some(over)) => (
            Verdict::Pass,
            format!(
                "frames back within {RECOVERY_MARGIN_MS:.0} ms of the phase's final level \
                 {recovery:.0} ms after the step; that level {over:.0} ms over the base"
            ),
        ),
        _ => (
            Verdict::Incomplete,
            "no frames, or no base age, to judge by".to_owned(),
        ),
    };
    results.record(name, verdict, detail);
}

/// `frame_age` per new frame (`seq` its id), and per router window `offered` and `cap` in bytes
/// per second; per vehicle poll `rung` and `queue_delay` in ms.
fn samples(frames: &[Frame], windows: &[Window], caps: &[Cap], polls: &[Poll]) -> Vec<Sample> {
    let unix_s = |ms: u64| float(ms) / 1000.0;
    let cap_at = |at: Instant| {
        caps.iter()
            .rev()
            .find(|c| c.at <= at)
            .map_or(f64::NAN, |c| c.bytes_per_s)
    };
    let mut samples: Vec<Sample> = frames
        .iter()
        .map(|f| Sample {
            unix_s: unix_s(f.unix_ms),
            series: "frame_age",
            seq: f.id,
            value: f.age_ms,
        })
        .collect();
    for (seq, w) in (0_u32..).zip(windows) {
        let at = unix_s(w.unix_ms);
        samples.push(Sample {
            unix_s: at,
            series: "offered",
            seq,
            value: w.bytes_per_s,
        });
        samples.push(Sample {
            unix_s: at,
            series: "cap",
            seq,
            value: cap_at(w.span.from),
        });
    }
    for (seq, p) in (0_u32..).zip(polls) {
        let value = |v: Option<u64>| v.map_or(f64::NAN, float);
        samples.push(Sample {
            unix_s: unix_s(p.unix_ms),
            series: "rung",
            seq,
            value: value(p.rung),
        });
        samples.push(Sample {
            unix_s: unix_s(p.unix_ms),
            series: "queue_delay",
            seq,
            value: value(p.queue_delay_ms),
        });
    }
    samples
}
