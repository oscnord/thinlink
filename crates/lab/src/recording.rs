//! `lab check recording`: the vehicle's retained snapshots. `checks` mounts the vehicle's snapshot
//! volume, so it sees the files the store writes. The store must hold its quota (the oldest
//! snapshots go first), a restarted vehicle must rescan the directory, delete what a crash left and
//! carry on with the gap reported, and storage too slow for the snapshot rate must drop snapshots
//! and never delay live frames. `docs/testbed.md#recording-check` defines every metric.

use std::fs;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use thinlink_detail::store::{self, Entry, StoreConfig, StoreSettings};
use thinlink_video::receiver::{Observation, Source};

use crate::adaptation::{self, Frame, ages_ms, new_frames};
use crate::budget::{self, Budget};
use crate::check::{self, Results, Verdict, counter, float, percentile};
use crate::client::AgentClient;
use crate::skeleton::Capture;

/// What `just lab-check` waits for on stdout to restart the vehicle's companion.
const RESTART_REQUEST: &str = "lab: restart companion";
const POLL_EVERY: Duration = Duration::from_millis(500);
/// While timing the restart: its time is graded, so it is read to this resolution.
const RESTART_POLL_EVERY: Duration = Duration::from_millis(100);
/// For the store to start writing, and to reach its quota: 4 MiB at about 1 snapshot a second.
const START_DEADLINE: Duration = Duration::from_secs(30);
const QUOTA_DEADLINE: Duration = Duration::from_secs(240);
/// How long the quota is watched once reached, and the frame age taken as the healthy level.
const QUOTA_WATCH: Duration = Duration::from_secs(20);
/// For a restarted vehicle to answer, and then to write a snapshot again.
const RESTART_DEADLINE: Duration = Duration::from_secs(90);
const RESUME_DEADLINE: Duration = Duration::from_secs(30);
/// How long writes are slowed down, longer than the writer's queue takes to fill.
const SLOW_WINDOW: Duration = Duration::from_secs(20);
const SLOW_RECOVERY: Duration = Duration::from_secs(20);
/// The snapshot interval, by the vehicle config's `[snapshot] fps = 1`.
const INTERVAL_MS: f64 = 1000.0;
/// Planted next to the snapshots before the restart: a half-written snapshot a crash would leave,
/// and two names that are no snapshots.
const PLANTED_TMP: &str = "1-1.jpg.tmp";
const PLANTED_OTHERS: [&str; 2] = ["notes.txt", "x-y.jpg"];

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// RTSP URL of the relay's stream.
    pub url: String,
    pub warmup: Duration,
    pub vehicle_metrics: String,
    /// The vehicle's snapshot directory as `checks` mounts it.
    pub dir: PathBuf,
    /// The vehicle's store limits, from its config.
    pub store: StoreConfig,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "url": self.url,
            "warmup_s": self.warmup.as_secs_f64(),
            "vehicle_metrics": self.vehicle_metrics,
            "dir": self.dir,
            "max_bytes": self.store.limits.max_bytes,
            "max_age_ms": self.store.limits.max_age_ms,
            "gap_after_ms": self.store.gap_after_ms,
            "quota_deadline_s": QUOTA_DEADLINE.as_secs_f64(),
            "quota_watch_s": QUOTA_WATCH.as_secs_f64(),
            "slow_window_s": SLOW_WINDOW.as_secs_f64(),
            "restart_request": RESTART_REQUEST,
        })
    }
}

/// `[store]` of the vehicle's config, validated as the vehicle validates it.
pub fn store(config: &str) -> Result<StoreConfig, String> {
    let mut table: toml::Table = toml::from_str(config).map_err(|e| e.to_string())?;
    let settings: StoreSettings = table
        .remove("store")
        .ok_or("no [store] table")?
        .try_into()
        .map_err(|e: toml::de::Error| e.to_string())?;
    settings.validate().map_err(|e| e.to_string())
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
    if let Err(e) = thinlink_video::init() {
        results.record("video_received", Verdict::Incomplete, e.to_string());
        return results;
    }
    let vehicle = AgentClient::new(&settings.vehicle_metrics);
    // An aborted run may have left the writes slow.
    let _ = vehicle.post("/snapshots/normal", "");
    let settled = adaptation::settle_at_top(&vehicle);
    adaptation::record_settled(&mut results, &settled);
    let capture = Capture::start(Source::Rtsp(settings.url.clone()));
    thread::sleep(settings.warmup);
    let healthy = hold_quota(&mut results, &vehicle, settings);
    let restarted = restart(&mut results, &vehicle, settings);
    let slow = slow_storage(&mut results, &vehicle);
    gaps_reported(&mut results, &vehicle, settings);
    let captured = capture.finish();
    let frames = frames_across_restart(&captured.observations, restarted);
    record_live(&mut results, &frames, healthy, slow);
    results.readings = json!({
        "vehicle_at_finish": check::get_json(&vehicle, "/metrics").unwrap_or(Value::Null),
        "receiver_at_finish": captured.receiver,
    });
    budget::grade(&mut results, budget);
    results
}

/// A stretch of the run.
#[derive(Debug, Clone, Copy)]
struct Span {
    from: Instant,
    to: Instant,
}

fn store_counter(reading: &Result<Value, String>, name: &str) -> Option<u64> {
    counter(reading, &format!("/retained/store/{name}"))
}

fn metrics(vehicle: &AgentClient) -> Result<Value, String> {
    check::get_json(vehicle, "/metrics")
}

/// Polls until `done` holds for the vehicle's metrics or `deadline` passes; returns the last
/// reading and whether it held.
fn wait_for(
    vehicle: &AgentClient,
    every: Duration,
    deadline: Duration,
    done: impl Fn(&Result<Value, String>) -> bool,
) -> (Result<Value, String>, bool) {
    let started = Instant::now();
    loop {
        let reading = metrics(vehicle);
        let holds = done(&reading);
        if holds || started.elapsed() >= deadline {
            return (reading, holds);
        }
        thread::sleep(every);
    }
}

fn listing(dir: &Path) -> Vec<Entry> {
    store::list(dir).map(|l| l.entries).unwrap_or_default()
}

fn median_interval_ms(entries: &[Entry]) -> Option<f64> {
    let mut steps: Vec<f64> = entries
        .windows(2)
        .filter_map(|pair| match pair {
            [a, b] => Some(float(b.capture_ms.saturating_sub(a.capture_ms))),
            _ => None,
        })
        .collect();
    steps.sort_by(f64::total_cmp);
    percentile(&steps, 50)
}

/// Waits for the store to start writing and to reach its quota, then watches it hold it: the
/// bytes in the directory and in the store's own count never pass the quota, and the oldest
/// snapshot only ever gets newer. Returns the stretch watched, as the healthy level of live video.
fn hold_quota(results: &mut Results, vehicle: &AgentClient, settings: &Settings) -> Option<Span> {
    let max_bytes = settings.store.limits.max_bytes;
    let (started, wrote) = wait_for(vehicle, POLL_EVERY, START_DEADLINE, |m| {
        store_counter(m, "written").is_some_and(|n| n >= 3)
    });
    let verdict = if wrote {
        Verdict::Pass
    } else {
        Verdict::Incomplete
    };
    let detail = format!(
        "{} snapshots written",
        store_counter(&started, "written").unwrap_or(0)
    );
    results.record("recording_started", verdict, detail);
    if !wrote {
        return None;
    }
    let (_, reached) = wait_for(vehicle, POLL_EVERY, QUOTA_DEADLINE, |m| {
        store_counter(m, "evicted").is_some_and(|n| n > 0)
    });
    let from = Instant::now();
    let mut worst_bytes = 0_u64;
    let mut oldest = 0_u64;
    let mut moved_back = false;
    let mut last = Value::Null;
    while from.elapsed() < QUOTA_WATCH {
        let reading = metrics(vehicle);
        let oldest_now = store_counter(&reading, "oldest_ms").unwrap_or(0);
        moved_back |= oldest_now < oldest;
        oldest = oldest.max(oldest_now);
        let dir_bytes: u64 = listing(&settings.dir).iter().map(|e| e.bytes).sum();
        let store_bytes = store_counter(&reading, "bytes").unwrap_or(0);
        worst_bytes = worst_bytes.max(dir_bytes).max(store_bytes);
        last = reading.unwrap_or(Value::Null);
        thread::sleep(POLL_EVERY);
    }
    let span = Span {
        from,
        to: Instant::now(),
    };
    let entries = listing(&settings.dir);
    let interval = median_interval_ms(&entries);
    let evicted = last
        .pointer("/retained/store/evicted")
        .and_then(Value::as_u64);
    results
        .metrics
        .insert("store_bytes_max", float(worst_bytes));
    results.metrics.insert(
        "store_over_quota_bytes",
        float(worst_bytes.saturating_sub(max_bytes)),
    );
    results
        .metrics
        .insert("store_count", float(entries.len() as u64));
    results.metrics.extend(
        interval
            .map(|ms| ("snapshot_interval_median_ms", ms))
            .into_iter()
            .chain(evicted.map(|n| ("store_evicted", float(n)))),
    );
    let (verdict, detail) = quota_verdict(reached, worst_bytes, max_bytes, moved_back);
    results.record("quota_held", verdict, detail);
    let (verdict, detail) = cadence_verdict(interval);
    results.record("snapshot_cadence", verdict, detail);
    Some(span)
}

fn quota_verdict(reached: bool, worst: u64, max: u64, moved_back: bool) -> (Verdict, String) {
    let detail = format!("at most {worst} bytes held against a quota of {max}");
    if worst > max {
        return (Verdict::Fail, format!("{detail}: over the quota"));
    }
    if moved_back {
        let why = "the oldest snapshot got older: something other than the oldest was evicted";
        return (Verdict::Fail, format!("{detail}: {why}"));
    }
    if !reached {
        let why = "the quota was never reached, so nothing was evicted";
        return (Verdict::Incomplete, format!("{detail}: {why}"));
    }
    (
        Verdict::Pass,
        format!("{detail}, evicting the oldest first"),
    )
}

fn cadence_verdict(interval: Option<f64>) -> (Verdict, String) {
    let Some(ms) = interval else {
        return (Verdict::Incomplete, "fewer than two snapshots".to_owned());
    };
    let detail = format!("snapshots {ms:.0} ms apart (median), {INTERVAL_MS:.0} ms configured");
    if (ms - INTERVAL_MS).abs() <= INTERVAL_MS * 0.1 {
        (Verdict::Pass, detail)
    } else {
        (Verdict::Fail, detail)
    }
}

/// Plants what a crash leaves, restarts the vehicle's companion (`just lab-check` does that when
/// this prints the request), and checks the rescan: the half-written file gone, the names that
/// are no snapshots left alone and not indexed, the snapshots kept, writing resumed, the gap
/// reported. Returns when the restart was requested, if it ran.
fn restart(results: &mut Results, vehicle: &AgentClient, settings: &Settings) -> Option<Instant> {
    let names = [PLANTED_TMP].into_iter().chain(PLANTED_OTHERS);
    let planted = names
        .map(|name| settings.dir.join(name))
        .all(|path| fs::write(path, b"not a snapshot").is_ok());
    let before = metrics(vehicle);
    let before_files = listing(&settings.dir);
    let written_before = store_counter(&before, "written");
    let (Some(written_before), true) = (written_before, planted) else {
        let detail = "cannot plant the files or read the store before the restart";
        results.record("restart_recovery", Verdict::Incomplete, detail);
        return None;
    };
    let requested = Instant::now();
    println!("{RESTART_REQUEST}");
    let (after, back) = wait_for(vehicle, RESTART_POLL_EVERY, RESTART_DEADLINE, |m| {
        store_counter(m, "written").is_some_and(|n| n < written_before)
    });
    if !back {
        let detail =
            "the vehicle did not restart: run it through `just lab-check`, which restarts it";
        results.record("restart_recovery", Verdict::Incomplete, detail);
        remove_planted(&settings.dir);
        return None;
    }
    let (_, wrote) = wait_for(vehicle, RESTART_POLL_EVERY, RESUME_DEADLINE, |m| {
        store_counter(m, "written").is_some_and(|n| n >= 1)
    });
    if wrote {
        let resume_ms = requested.elapsed().as_secs_f64() * 1000.0;
        results.metrics.insert("restart_resume_ms", resume_ms);
    }
    let tmp_gone = !settings.dir.join(PLANTED_TMP).exists();
    let others_left = PLANTED_OTHERS
        .iter()
        .all(|name| settings.dir.join(name).exists());
    remove_planted(&settings.dir);
    let recovered = RecoveryReading {
        tmp_removed: counter(&after, "/retained/store/recovery/tmp_removed"),
        ignored: counter(&after, "/retained/store/recovery/ignored"),
        kept: counter(&after, "/retained/store/recovery/kept"),
        files_before: before_files.len() as u64,
        tmp_gone,
        others_left,
        wrote,
    };
    let (verdict, detail) = recovery_verdict(&recovered);
    results.record("restart_recovery", verdict, detail);
    Some(requested)
}

/// A restarted vehicle numbers its frames from 0 again, so ids from before the restart are not seen
/// again: each side of the restart counts its own new frames. Observations are in arrival order.
fn frames_across_restart(observations: &[Observation], restarted: Option<Instant>) -> Vec<Frame> {
    let Some(at) = restarted else {
        return new_frames(observations);
    };
    let (before, after) =
        observations.split_at(observations.partition_point(|o| o.received_at < at));
    let mut frames = new_frames(before);
    frames.extend(new_frames(after));
    frames
}

fn remove_planted(dir: &Path) {
    for name in [PLANTED_TMP].into_iter().chain(PLANTED_OTHERS) {
        let _ = fs::remove_file(dir.join(name));
    }
}

#[derive(Debug)]
struct RecoveryReading {
    tmp_removed: Option<u64>,
    ignored: Option<u64>,
    kept: Option<u64>,
    files_before: u64,
    tmp_gone: bool,
    others_left: bool,
    wrote: bool,
}

/// The rescan found what was there: the planted `.tmp` deleted, the planted names ignored but left,
/// the snapshots that were on disk indexed (a snapshot or two may land or go between the
/// listings), and writing resumed.
fn recovery_verdict(r: &RecoveryReading) -> (Verdict, String) {
    let detail = format!("{r:?}");
    let (Some(tmp_removed), Some(ignored), Some(kept)) = (r.tmp_removed, r.ignored, r.kept) else {
        return (
            Verdict::Incomplete,
            format!("no recovery reading: {detail}"),
        );
    };
    let rules = [
        (
            r.tmp_gone && tmp_removed >= 1,
            "the half-written file was not deleted",
        ),
        (
            r.others_left && ignored >= 2,
            "names that are no snapshots were not left alone",
        ),
        (
            kept.abs_diff(r.files_before) <= 2,
            "the snapshots on disk were not all indexed",
        ),
        (r.wrote, "no snapshot was written after the restart"),
    ];
    match rules.iter().find(|(holds, _)| !holds) {
        Some((_, rule)) => (Verdict::Fail, format!("{rule}: {detail}")),
        None => (Verdict::Pass, detail),
    }
}

/// Makes every snapshot write take longer than the snapshot interval and watches: snapshots must be
/// dropped (counted) and the live frames come on as before. Writes are set back to normal at the
/// end, also when the hook did not answer.
fn slow_storage(results: &mut Results, vehicle: &AgentClient) -> Option<Span> {
    let before = metrics(vehicle);
    let slowed = vehicle.post("/snapshots/slow", "");
    if let Err(e) = &slowed {
        let detail = format!("the vehicle's slow-write hook does not answer: {e}");
        results.record("slow_storage_drops", Verdict::Incomplete, detail);
        return None;
    }
    let from = Instant::now();
    thread::sleep(SLOW_WINDOW);
    let span = Span {
        from,
        to: Instant::now(),
    };
    let during = metrics(vehicle);
    let restored = vehicle.post("/snapshots/normal", "");
    let (after, resumed) = wait_for(vehicle, POLL_EVERY, SLOW_RECOVERY, |m| {
        let written = |m: &Result<Value, String>| store_counter(m, "written");
        written(m) > written(&during) && counter(m, "/retained/write_delay_ms") == Some(0)
    });
    let dropped = check::increase(&before, &during, "/snapshots/dropped");
    let encoded = check::increase(&before, &during, "/frames_encoded");
    let seconds = span.to.saturating_duration_since(span.from).as_secs_f64();
    results.metrics.extend(
        dropped
            .map(|n| ("slow_dropped", float(n)))
            .into_iter()
            .chain(encoded.map(|n| ("slow_encoded_per_s", float(n) / seconds))),
    );
    let (verdict, detail) = match dropped {
        Some(n) if n >= 1 => (
            Verdict::Pass,
            format!(
                "{n} snapshots dropped in {seconds:.0} s of slow writes, none of them live frames"
            ),
        ),
        Some(_) => (
            Verdict::Fail,
            "no snapshot was dropped: slow storage must drop snapshots, counted".to_owned(),
        ),
        None => (Verdict::Incomplete, "the counters did not read".to_owned()),
    };
    results.record("slow_storage_drops", verdict, detail);
    let verdict = if resumed && restored.is_ok() {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    let written = store_counter(&after, "written").unwrap_or(0);
    let detail =
        format!("snapshots written again after the writes were back to normal: {written} in all");
    results.record("slow_storage_recovers", verdict, detail);
    Some(span)
}

/// The directory has gaps now (the slow writes left snapshots 3 s apart): the store must report
/// the same ones. The store is read before and after the directory, since a snapshot may land
/// between, so the directory's count may lie between the two readings.
fn gaps_reported(results: &mut Results, vehicle: &AgentClient, settings: &Settings) {
    let name = "gap_reported";
    let before = metrics(vehicle);
    let entries = listing(&settings.dir);
    let after = metrics(vehicle);
    let in_files = entries
        .windows(2)
        .filter(|pair| match pair {
            [a, b] => b.capture_ms.saturating_sub(a.capture_ms) > settings.store.gap_after_ms,
            _ => false,
        })
        .count() as u64;
    let (Some(first), Some(second)) = (
        store_counter(&before, "gaps"),
        store_counter(&after, "gaps"),
    ) else {
        results.record(name, Verdict::Incomplete, "the store's gaps did not read");
        return;
    };
    let largest = store_counter(&after, "largest_gap_ms").unwrap_or(0);
    let detail = format!(
        "the directory has {in_files} gap(s) of more than {} ms, the store reports {first} then {second}, the largest {largest} ms",
        settings.store.gap_after_ms
    );
    let verdict = if in_files == 0 {
        Verdict::Incomplete
    } else if (first.min(second)..=first.max(second)).contains(&in_files) {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record(name, verdict, detail);
}

/// Live frame age and pace in the healthy stretch and during slow storage.
fn record_live(results: &mut Results, frames: &[Frame], healthy: Option<Span>, slow: Option<Span>) {
    let (Some(healthy), Some(slow)) = (healthy, slow) else {
        results.record(
            "live_unaffected",
            Verdict::Incomplete,
            "the healthy or the slow stretch was not measured",
        );
        return;
    };
    let base = ages_ms(frames, healthy.from, healthy.to);
    let ages = ages_ms(frames, slow.from, slow.to);
    let seconds = slow.to.saturating_duration_since(slow.from).as_secs_f64();
    let pace = float(ages.len() as u64) / seconds;
    let (Some(base_p95), Some(slow_p95)) = (percentile(&base, 95), percentile(&ages, 95)) else {
        results.record(
            "live_unaffected",
            Verdict::Incomplete,
            "no frames in a stretch",
        );
        return;
    };
    results.metrics.insert("healthy_frame_age_p95_ms", base_p95);
    results.metrics.insert("slow_frame_age_p95_ms", slow_p95);
    results
        .metrics
        .insert("slow_age_p95_delta_ms", slow_p95 - base_p95);
    results.metrics.insert("slow_new_frames_per_s", pace);
    let detail = format!(
        "frame age p95 {slow_p95:.0} ms during slow writes against {base_p95:.0} ms healthy, {pace:.1} new frames/s; graded by the budget"
    );
    results.record("live_unaffected", Verdict::Pass, detail);
}

#[cfg(test)]
mod tests {
    use thinlink_video_core::stamp::{CaptureMs, FrameId, Stamp};

    use super::*;

    #[test]
    fn frame_ids_restarting_after_the_vehicle_restart_still_count() {
        let t0 = Instant::now();
        let seen = |at_ms: u64, id: u32| Observation {
            received_at: t0 + Duration::from_millis(at_ms),
            received_ms: 1_800_000_000_000 + at_ms,
            stamp: Some(Stamp {
                frame_id: FrameId(id),
                capture_ms: CaptureMs::from_unix_ms(1_800_000_000_000 + at_ms - 10),
            }),
            state: None,
        };
        let observations = [
            seen(100, 500),
            seen(200, 501),
            seen(1100, 0),
            seen(1200, 1),
            seen(1300, 500),
        ];
        let frames = frames_across_restart(&observations, Some(t0 + Duration::from_millis(1000)));
        assert_eq!(frames.len(), 5);
    }
}
