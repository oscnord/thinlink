//! `lab check retrieval`: a crop or a snapshot of a frame captured seconds ago, fetched over the
//! link at lower priority than the live video. `checks` mounts the vehicle's snapshot volume, so
//! it knows which frame each job must come back with. It grades the object (the frame and the
//! rectangle it names), the transfer (time, bytes against the allowance, cancel), the requests the
//! job API must refuse, and the live video's frame age in two passes under the same profile, one
//! without retrieval and one with jobs running back to back. `docs/measurement.md` defines every
//! metric.

use std::fs;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use sat_teleop_detail::client::{Client, ClientError};
use sat_teleop_detail::crop::{self, Crop};
use sat_teleop_detail::store::{self, Entry};
use sat_teleop_detail::vehicle::RetrievalLimits;
use sat_teleop_detail::wire::{JobRequest, Kind};
use sat_teleop_media::receiver::Source;
use sat_teleop_media::unix_ms;

use crate::adaptation::{self, Expect, ages_ms, new_frames, offered_bytes};
use crate::budget::{self, Budget};
use crate::check::{self, Results, Verdict, counter, float, percentile};
use crate::client::AgentClient;
use crate::profile::Profile;
use crate::skeleton::Capture;

mod hostile;

/// How far back the jobs look: far enough that the frame is retained, close enough that it is not
/// the oldest.
const AGO: Duration = Duration::from_secs(8);
/// Snapshots the store must hold before the check starts.
const HISTORY: u64 = 12;
const HISTORY_DEADLINE: Duration = Duration::from_secs(60);
const CROP: Crop = Crop {
    x: 0.25,
    y: 0.25,
    w: 0.5,
    h: 0.5,
};
const JOB_MAX_BYTES: u32 = 524_288;
const JOB_LIFE_MS: u32 = 180_000;
/// The longest a job may take in the checks that wait for one.
const JOB_DEADLINE: Duration = Duration::from_secs(170);
const CLIENT_TIMEOUT: Duration = Duration::from_secs(10);
const POLL_EVERY: Duration = Duration::from_millis(250);
const SAMPLE_EVERY: Duration = Duration::from_millis(100);
/// The vehicle stops within this of a cancel, at its next chunk, or the check fails to grade it.
const CANCEL_DEADLINE: Duration = Duration::from_secs(30);
/// After the cancel nothing more may arrive for this long.
const CANCEL_QUIET: Duration = Duration::from_secs(4);
/// Before a pass the ladder must have been quiet this long at the top rung.
const PASS_SETTLE: Duration = Duration::from_secs(15);
/// A pass under a profile with a timeline runs this long after its last step.
const TAIL: Duration = Duration::from_secs(20);

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// RTSP URL of the relay's stream.
    pub url: String,
    pub warmup: Duration,
    /// A pass under a profile without a timeline.
    pub duration: Duration,
    pub vehicle_metrics: String,
    pub relay_metrics: String,
    /// The relay's job API for the ground, and for the vehicle (on the overlay).
    pub ground_api: String,
    pub vehicle_api: String,
    /// The vehicle's snapshot directory as `checks` mounts it.
    pub dir: PathBuf,
    /// `[retrieval]` allowance of the limits, bytes per second on the wire.
    pub allowance: u32,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "url": self.url,
            "warmup_s": self.warmup.as_secs_f64(),
            "duration_s": self.duration.as_secs_f64(),
            "vehicle_metrics": self.vehicle_metrics,
            "relay_metrics": self.relay_metrics,
            "ground_api": self.ground_api,
            "vehicle_api": self.vehicle_api,
            "dir": self.dir,
            "allowance_bytes_per_s": self.allowance,
            "ago_s": AGO.as_secs(),
            "crop": [CROP.x, CROP.y, CROP.w, CROP.h],
            "job_max_bytes": JOB_MAX_BYTES,
            "cancel_deadline_s": CANCEL_DEADLINE.as_secs_f64(),
            "pass_settle_s": PASS_SETTLE.as_secs_f64(),
            "tail_s": TAIL.as_secs_f64(),
        })
    }
}

/// The allowance in `[retrieval]` of the limits, bytes per second on the wire.
pub fn allowance(limits: &str) -> Result<u32, String> {
    let mut table: toml::Table = toml::from_str(limits).map_err(|e| e.to_string())?;
    let limits: RetrievalLimits = table
        .remove("retrieval")
        .ok_or("no [retrieval] table")?
        .try_into()
        .map_err(|e: toml::de::Error| e.to_string())?;
    let limits = limits.validate().map_err(|e| e.to_string())?;
    Ok(limits.allowance_bytes_per_s)
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
        results.record("video_received", Verdict::Incomplete, e.to_string());
        return results;
    }
    let vehicle = AgentClient::new(&settings.vehicle_metrics);
    let relay = AgentClient::new(&settings.relay_metrics);
    let ground = Client::new(&settings.ground_api, CLIENT_TIMEOUT)
        .with_token(crate::client::token_for(&settings.ground_api));
    let settled = adaptation::settle_at_top(&vehicle);
    adaptation::record_settled(&mut results, &settled);
    if !wait_for_history(&vehicle) {
        let detail = format!("the store holds fewer than {HISTORY} snapshots");
        results.record("history", Verdict::Incomplete, detail);
        return results;
    }
    object(&mut results, &ground, settings);
    hostile::refusals(&mut results, settings, &relay);
    cancel(&mut results, &ground, &vehicle, &relay);
    paired(
        &mut results,
        agent,
        &vehicle,
        &ground,
        profile,
        profile_text,
        settings,
    );
    budget::grade(&mut results, budget);
    results
}

fn wait_for_history(vehicle: &AgentClient) -> bool {
    let started = Instant::now();
    while started.elapsed() < HISTORY_DEADLINE {
        let reading = check::get_json(vehicle, "/metrics");
        if counter(&reading, "/retained/store/count").is_some_and(|n| n >= HISTORY) {
            return true;
        }
        thread::sleep(POLL_EVERY * 4);
    }
    false
}

fn crop_request(at_capture_ms: u64) -> JobRequest {
    JobRequest {
        kind: Kind::Crop,
        at_capture_ms,
        crop: Some(CROP),
        max_bytes: JOB_MAX_BYTES,
        expires_in_ms: JOB_LIFE_MS,
    }
}

fn snapshot_request(at_capture_ms: u64) -> JobRequest {
    JobRequest {
        kind: Kind::Snapshot,
        at_capture_ms,
        crop: None,
        max_bytes: JOB_MAX_BYTES,
        expires_in_ms: JOB_LIFE_MS,
    }
}

fn ago_ms() -> u64 {
    unix_ms().saturating_sub(u64::try_from(AGO.as_millis()).unwrap_or(0))
}

/// What the store held for a time asked for: the newest snapshot at or before it, and its bytes,
/// read from the volume before the job runs so a later eviction cannot change the answer.
fn retained_at(settings: &Settings, at_capture_ms: u64) -> Option<(Entry, Vec<u8>)> {
    let listing = store::list(&settings.dir).ok()?;
    let entry = listing
        .entries
        .into_iter()
        .rfind(|e| e.capture_ms <= at_capture_ms)?;
    let bytes = fs::read(store::entry_path(&settings.dir, entry)).ok()?;
    Some((entry, bytes))
}

/// A crop and a snapshot of the frame captured `AGO` ago come back as that frame: its capture time
/// and id, its size and the rectangle asked for, and the snapshot byte for byte.
fn object(results: &mut Results, ground: &Client, settings: &Settings) {
    let at = ago_ms();
    let Some((truth, stored)) = retained_at(settings, at) else {
        let detail = "no retained snapshot to compare with";
        results.record("crop_matches_retained", Verdict::Incomplete, detail);
        results.record("snapshot_matches_retained", Verdict::Incomplete, detail);
        return;
    };
    let source = crop::dimensions(&stored);
    let (verdict, detail) = match fetch_job(ground, &crop_request(at), results, "crop") {
        Err(e) => (Verdict::Fail, e),
        Ok(fetched) => judge_crop(&fetched, truth, source, at),
    };
    results.record("crop_matches_retained", verdict, detail);
    let (verdict, detail) = match fetch_job(ground, &snapshot_request(at), results, "snapshot") {
        Err(e) => (Verdict::Fail, e),
        Ok(fetched) => judge_snapshot(&fetched, truth, &stored, at),
    };
    results.record("snapshot_matches_retained", verdict, detail);
}

/// Requests a job, waits for it, and records its completion time and size under `name`.
fn fetch_job(
    ground: &Client,
    job: &JobRequest,
    results: &mut Results,
    name: &str,
) -> Result<sat_teleop_detail::client::Fetched, String> {
    let started = Instant::now();
    let id = ground.request(job).map_err(|e| e.to_string())?;
    let fetched = ground.fetch(&id, JOB_DEADLINE).map_err(|e| e.to_string());
    let seconds = started.elapsed().as_secs_f64();
    let _ = ground.cancel(&id);
    let fetched = fetched?;
    let (completion, size) = if name == "crop" {
        ("crop_completion_s", "crop_bytes")
    } else {
        ("snapshot_completion_s", "snapshot_bytes")
    };
    results.metrics.insert(completion, seconds);
    results
        .metrics
        .insert(size, float(fetched.jpeg.len() as u64));
    Ok(fetched)
}

fn judge_crop(
    fetched: &sat_teleop_detail::client::Fetched,
    truth: Entry,
    source: Option<(u32, u32)>,
    at: u64,
) -> (Verdict, String) {
    let meta = fetched.meta;
    let Some(source) = source else {
        return (
            Verdict::Incomplete,
            "the retained file is no JPEG".to_owned(),
        );
    };
    let expected = crop::pixel_rect(source.0, source.1, CROP).ok();
    let decoded = crop::dimensions(&fetched.jpeg);
    let rules = [
        (
            (meta.capture_ms, meta.frame_id) == (truth.capture_ms, truth.frame_id),
            "the frame is not the retained one",
        ),
        (
            meta.capture_ms <= at,
            "the frame was captured after the time asked for",
        ),
        (
            (meta.source_width, meta.source_height) == source,
            "the source size is not the retained file's",
        ),
        (
            expected.is_some() && meta.crop == expected,
            "the rectangle is not the one asked for",
        ),
        (
            decoded == expected.map(|r| (r.w, r.h)),
            "the JPEG's size is not the rectangle's",
        ),
    ];
    let detail = format!(
        "frame {} captured at {} ms, crop {:?} of {}x{}, JPEG {:?}",
        meta.frame_id, meta.capture_ms, meta.crop, source.0, source.1, decoded
    );
    match rules.iter().find(|(holds, _)| !holds) {
        Some((_, rule)) => (Verdict::Fail, format!("{rule}: {detail}")),
        None => (Verdict::Pass, detail),
    }
}

fn judge_snapshot(
    fetched: &sat_teleop_detail::client::Fetched,
    truth: Entry,
    stored: &[u8],
    at: u64,
) -> (Verdict, String) {
    let meta = fetched.meta;
    let rules = [
        (
            (meta.capture_ms, meta.frame_id) == (truth.capture_ms, truth.frame_id),
            "the frame is not the retained one",
        ),
        (
            meta.capture_ms <= at,
            "the frame was captured after the time asked for",
        ),
        (meta.crop.is_none(), "a snapshot carries no crop"),
        (
            fetched.jpeg == stored,
            "the bytes are not the retained file's",
        ),
    ];
    let detail = format!(
        "frame {} captured at {} ms, {} bytes",
        meta.frame_id,
        meta.capture_ms,
        fetched.jpeg.len()
    );
    match rules.iter().find(|(holds, _)| !holds) {
        Some((_, rule)) => (Verdict::Fail, format!("{rule}: {detail}")),
        None => (Verdict::Pass, detail),
    }
}

/// A snapshot, the largest job, is cancelled once the vehicle has sent two chunks of it: the
/// vehicle must stop at its next chunk, and the relay must receive nothing more for it.
fn cancel(results: &mut Results, ground: &Client, vehicle: &AgentClient, relay: &AgentClient) {
    let name = "cancel_stops_transfer";
    let before = check::get_json(vehicle, "/metrics");
    let id = match ground.request(&snapshot_request(ago_ms())) {
        Ok(id) => id,
        Err(e) => {
            results.record(
                name,
                Verdict::Incomplete,
                format!("cannot request the job: {e}"),
            );
            return;
        }
    };
    let chunks_before = counter(&before, "/retrieval/chunks_sent").unwrap_or(0);
    let mid_transfer = wait_until(CANCEL_DEADLINE * 3, || {
        let now = check::get_json(vehicle, "/metrics");
        counter(&now, "/retrieval/chunks_sent").is_some_and(|n| n >= chunks_before + 2)
    });
    if !mid_transfer {
        let _ = ground.cancel(&id);
        let detail =
            "the transfer did not get under way, or it finished before it could be cancelled";
        results.record(name, Verdict::Incomplete, detail);
        return;
    }
    let cancelled_before = counter(
        &check::get_json(vehicle, "/metrics"),
        "/retrieval/jobs_cancelled",
    );
    let asked = Instant::now();
    let accepted = ground.cancel(&id);
    let stopped = wait_until(CANCEL_DEADLINE, || {
        let now = check::get_json(vehicle, "/metrics");
        counter(&now, "/retrieval/jobs_cancelled") > cancelled_before
    });
    let stop_ms = asked.elapsed().as_secs_f64() * 1000.0;
    let chunks_at_stop = counter(&check::get_json(relay, "/metrics"), "/detail/chunks");
    thread::sleep(CANCEL_QUIET);
    let chunks_later = counter(&check::get_json(relay, "/metrics"), "/detail/chunks");
    let gone = matches!(ground.poll(&id), Err(ClientError::Unknown));
    if stopped {
        results.metrics.insert("cancel_stop_ms", stop_ms);
    }
    let detail = format!(
        "the vehicle noticed after {stop_ms:.0} ms; chunks at the relay {chunks_at_stop:?} then {chunks_later:?}"
    );
    let rules = [
        (accepted.is_ok(), "the relay refused the cancel"),
        (stopped, "the vehicle did not stop within the deadline"),
        (
            chunks_at_stop == chunks_later,
            "chunks kept arriving after the vehicle stopped",
        ),
        (gone, "the cancelled job is still known"),
    ];
    let (verdict, detail) = match rules.iter().find(|(holds, _)| !holds) {
        Some((_, rule)) => (Verdict::Fail, format!("{rule}: {detail}")),
        None => (Verdict::Pass, detail),
    };
    results.record(name, verdict, detail);
}

fn wait_until(deadline: Duration, done: impl Fn() -> bool) -> bool {
    let started = Instant::now();
    while started.elapsed() < deadline {
        if done() {
            return true;
        }
        thread::sleep(Duration::from_millis(100));
    }
    false
}

/// What one pass measured.
#[derive(Debug, Default)]
struct Pass {
    /// The profile has a timeline.
    timeline: bool,
    seconds: f64,
    ages: Vec<f64>,
    /// The vehicle's retrieval counters over the pass.
    wire_bytes: Option<u64>,
    chunks: Option<u64>,
    jobs_done: Option<u64>,
    holds: Option<u64>,
    /// What the vehicle offered the uplink, from the router.
    offered_bytes: Option<u64>,
    /// How long each job took, request to object.
    completions_s: Vec<f64>,
    held_ms: Option<u64>,
    /// The vehicle's retrieval state every `SAMPLE_EVERY` through the pass.
    seen: Vec<Seen>,
}

/// One reading of what retrieval was doing.
#[derive(Debug, Clone)]
struct Seen {
    held: bool,
    hold: Option<String>,
    chunks: u64,
}

/// The live video's frame age in two passes under the same profile, one without retrieval and one
/// with jobs running back to back, and what retrieval cost on the wire.
fn paired(
    results: &mut Results,
    agent: &AgentClient,
    vehicle: &AgentClient,
    ground: &Client,
    profile: &Profile,
    profile_text: &str,
    settings: &Settings,
) {
    let length = profile
        .timeline
        .last()
        .map_or(settings.duration, |t| Duration::from_millis(t.at_ms) + TAIL);
    let timeline = !profile.timeline.is_empty();
    let run = |retrieving| {
        pass(
            agent,
            vehicle,
            ground,
            profile_text,
            settings,
            Plan {
                retrieving,
                timeline,
                length,
            },
        )
    };
    let without = run(false);
    let with = run(true);
    let (Some(without), Some(with)) = (without, with) else {
        let detail = "a pass could not be measured: no settled ladder, or no video";
        results.record("live_unaffected", Verdict::Incomplete, detail);
        return;
    };
    record_passes(results, &without, &with, settings);
}

#[derive(Clone, Copy)]
struct Plan {
    retrieving: bool,
    timeline: bool,
    length: Duration,
}

/// Re-applies the profile (which stops a timeline and restores the base rate), waits for the top
/// rung, plays the stream and, with retrieval, runs jobs back to back for the pass.
fn pass(
    agent: &AgentClient,
    vehicle: &AgentClient,
    ground: &Client,
    profile_text: &str,
    settings: &Settings,
    plan: Plan,
) -> Option<Pass> {
    agent.post("/profile", profile_text).ok()?;
    let settled = adaptation::settle(vehicle, Expect::Top, PASS_SETTLE);
    if !(settled.quiet && settled.rung == Some(0)) {
        return None;
    }
    let capture = Capture::start(Source::Rtsp(settings.url.clone()));
    thread::sleep(settings.warmup);
    let jobs = plan.retrieving.then(|| Jobs::start(ground.clone()));
    let before = check::get_json(vehicle, "/metrics");
    let offered_start = offered_bytes(agent);
    let started = Instant::now();
    if plan.timeline {
        let _ = agent.post("/timeline/start", "");
    }
    let seen = watch(vehicle, plan.length);
    let ended = Instant::now();
    let after = check::get_json(vehicle, "/metrics");
    let offered_end = offered_bytes(agent);
    let completions_s = jobs.map(Jobs::stop).unwrap_or_default();
    let captured = capture.finish();
    let frames = new_frames(&captured.observations);
    let delta = |pointer: &str| check::increase(&before, &after, pointer);
    Some(Pass {
        timeline: plan.timeline,
        seconds: ended.saturating_duration_since(started).as_secs_f64(),
        ages: ages_ms(&frames, started, ended),
        wire_bytes: delta("/retrieval/wire_bytes"),
        chunks: delta("/retrieval/chunks_sent"),
        jobs_done: delta("/retrieval/jobs_done"),
        holds: delta("/retrieval/holds"),
        held_ms: delta("/retrieval/held_ms"),
        seen,
        offered_bytes: offered_start
            .zip(offered_end)
            .and_then(|(a, b)| b.1.checked_sub(a.1)),
        completions_s,
    })
}

/// The vehicle's retrieval state every `SAMPLE_EVERY` for `length`.
fn watch(vehicle: &AgentClient, length: Duration) -> Vec<Seen> {
    let started = Instant::now();
    let mut seen = Vec::new();
    while started.elapsed() < length {
        let reading = check::get_json(vehicle, "/metrics");
        let state = reading
            .as_ref()
            .ok()
            .and_then(|m| m.pointer("/retrieval/state")?.as_str().map(str::to_owned));
        let hold = reading
            .as_ref()
            .ok()
            .and_then(|m| m.pointer("/retrieval/hold")?.as_str().map(str::to_owned));
        if let (Some(state), Some(chunks)) = (state, counter(&reading, "/retrieval/chunks_sent")) {
            seen.push(Seen {
                held: state == "held",
                hold,
                chunks,
            });
        }
        thread::sleep(SAMPLE_EVERY);
    }
    seen
}

/// Crop jobs for the frame `AGO` ago, one after the other, until stopped; each job's time from
/// request to object.
struct Jobs {
    stop: Arc<AtomicBool>,
    thread: JoinHandle<Vec<f64>>,
}

impl Jobs {
    fn start(ground: Client) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let stop = Arc::clone(&stop);
            thread::spawn(move || run_jobs(&ground, &stop))
        };
        Self { stop, thread }
    }

    fn stop(self) -> Vec<f64> {
        self.stop.store(true, Ordering::Relaxed);
        self.thread.join().unwrap_or_default()
    }
}

fn run_jobs(ground: &Client, stop: &AtomicBool) -> Vec<f64> {
    let mut completions = Vec::new();
    while !stop.load(Ordering::Relaxed) {
        let started = Instant::now();
        let Ok(id) = ground.request(&crop_request(ago_ms())) else {
            thread::sleep(POLL_EVERY * 4);
            continue;
        };
        let mut done = false;
        while !stop.load(Ordering::Relaxed) && started.elapsed() < JOB_DEADLINE {
            match ground.poll(&id) {
                Ok(Some(_)) => {
                    done = true;
                    break;
                }
                Ok(None) => thread::sleep(POLL_EVERY),
                Err(_) => break,
            }
        }
        if done {
            completions.push(started.elapsed().as_secs_f64());
        }
        let _ = ground.cancel(&id);
    }
    completions
}

fn record_passes(results: &mut Results, without: &Pass, with: &Pass, settings: &Settings) {
    let (Some(p50), Some(p95)) = (percentile(&without.ages, 50), percentile(&without.ages, 95))
    else {
        results.record(
            "live_unaffected",
            Verdict::Incomplete,
            "no frames without retrieval",
        );
        return;
    };
    let (Some(r50), Some(r95)) = (percentile(&with.ages, 50), percentile(&with.ages, 95)) else {
        results.record(
            "live_unaffected",
            Verdict::Incomplete,
            "no frames with retrieval",
        );
        return;
    };
    let metrics = &mut results.metrics;
    metrics.insert("age_p50_without_ms", p50);
    metrics.insert("age_p95_without_ms", p95);
    metrics.insert("age_p50_with_ms", r50);
    metrics.insert("age_p95_with_ms", r95);
    metrics.insert("age_p50_delta_ms", r50 - p50);
    metrics.insert("age_p95_delta_ms", r95 - p95);
    let detail = format!(
        "frame age p50 {r50:.0} ms and p95 {r95:.0} ms with retrieval, {p50:.0} and {p95:.0} ms without; graded by the budget"
    );
    results.record("live_unaffected", Verdict::Pass, detail);
    record_cost(results, without, with, settings);
    if with.timeline {
        record_yield(results, with);
    }
}

/// Under a timeline retrieval must have held back at some point (the ladder left its normal rungs,
/// a drain, stale feedback) and sent no chunk while it was held: two readings in a row that show it
/// held may not show more chunks sent.
fn record_yield(results: &mut Results, with: &Pass) {
    let name = "retrieval_yields";
    let sent_while_held = with
        .seen
        .windows(2)
        .filter(|pair| match pair {
            [a, b] => a.held && b.held && b.chunks > a.chunks,
            _ => false,
        })
        .count();
    let mut reasons: Vec<&str> = with
        .seen
        .iter()
        .filter(|s| s.held)
        .filter_map(|s| s.hold.as_deref())
        .collect();
    reasons.sort_unstable();
    reasons.dedup();
    results
        .metrics
        .insert("retrieval_sent_while_held", float(sent_while_held as u64));
    let detail = format!(
        "held for {:.1} s in {} stretches, because of {reasons:?}; {sent_while_held} chunks sent while held",
        float(with.held_ms.unwrap_or(0)) / 1000.0,
        with.holds.unwrap_or(0)
    );
    let (verdict, detail) = if reasons.is_empty() {
        (
            Verdict::Incomplete,
            format!("retrieval was never held back under the timeline: {detail}"),
        )
    } else if sent_while_held > 0 {
        (Verdict::Fail, detail)
    } else {
        (Verdict::Pass, detail)
    };
    results.record(name, verdict, detail);
}

/// What retrieval put on the wire against its allowance: the vehicle's own count and the router's
/// difference between the passes.
fn record_cost(results: &mut Results, without: &Pass, with: &Pass, settings: &Settings) {
    let name = "retrieval_within_allowance";
    let allowance = float(u64::from(settings.allowance));
    let (Some(wire), Some(jobs)) = (with.wire_bytes, with.jobs_done) else {
        results.record(
            name,
            Verdict::Incomplete,
            "the vehicle's retrieval counters did not read",
        );
        return;
    };
    // The bucket starts with a second of the allowance.
    let allowed = allowance * (with.seconds + 1.0);
    let over = (float(wire) - allowed).max(0.0);
    let metrics = &mut results.metrics;
    metrics.insert("retrieval_wire_bytes_per_s", float(wire) / with.seconds);
    metrics.insert("retrieval_over_allowance_bytes", over);
    metrics.insert("retrieval_jobs_done", float(jobs));
    metrics.insert("retrieval_chunks", float(with.chunks.unwrap_or(0)));
    metrics.insert("retrieval_holds", float(with.holds.unwrap_or(0)));
    metrics.insert(
        "retrieval_held_s",
        float(with.held_ms.unwrap_or(0)) / 1000.0,
    );
    if let (Some(a), Some(b)) = (without.offered_bytes, with.offered_bytes) {
        let extra = (float(b) / with.seconds) - (float(a) / without.seconds);
        metrics.insert("offered_uplink_delta_bytes_per_s", extra);
    }
    let mut times = with.completions_s.clone();
    times.sort_by(f64::total_cmp);
    if let Some(p50) = percentile(&times, 50) {
        metrics.insert("job_completion_p50_s", p50);
    }
    let detail = format!(
        "{wire} bytes on the wire in {:.0} s against an allowance of {allowance:.0} B/s ({allowed:.0} with the bucket's first second), {jobs} jobs done",
        with.seconds
    );
    let verdict = if over > 0.0 {
        Verdict::Fail
    } else {
        Verdict::Pass
    };
    results.record(name, verdict, detail);
}
