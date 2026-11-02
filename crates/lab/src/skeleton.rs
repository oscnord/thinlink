//! `lab check skeleton`: the live video path end to end. A headless receiver in `checks` pulls the
//! relay's RTSP stream and reads the stamp of every decoded frame; router counters give the uplink
//! rate. Then a forwarding cut must silence the video and restoring it must bring fresh frames
//! back. `docs/measurement.md` defines every metric.

use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::sync_channel;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use sat_teleop_media::receiver::{self, Observation, Source};
use sat_teleop_media::{Health, HealthSnapshot, supervise, unix_ms};
use sat_teleop_policy::stamp::{self, CaptureMs, MAX_PLAUSIBLE_AGE_MS, plausible_age};

use crate::adaptation;
use crate::budget::{self, Budget};
use crate::check::{self, Results, Sample, Verdict, drain_for, float, increase, percentile};
use crate::client::AgentClient;

const OBSERVATION_QUEUE: usize = 64;
/// Over an hour at 60 fps; later observations are counted, not kept.
const MAX_OBSERVATIONS: usize = 250_000;
/// After the drain the video must stay silent this long.
const SILENCE: Duration = Duration::from_secs(2);
/// Covers a keyframe interval plus a GEO round trip.
const RECOVERY_DEADLINE: Duration = Duration::from_secs(15);
const POLL_EVERY: Duration = Duration::from_millis(100);

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// RTSP URL of the relay's stream.
    pub url: String,
    pub warmup: Duration,
    pub duration: Duration,
    /// Base URLs of the vehicle and relay metrics endpoints.
    pub vehicle_metrics: String,
    pub relay_metrics: String,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "receiver": {
                "url": self.url,
                "protocols": receiver::PROTOCOLS,
                "latency_ms": receiver::LATENCY_MS,
                "drop_on_latency": true,
                "decoder": receiver::DECODER,
            },
            "warmup_s": self.warmup.as_secs_f64(),
            "duration_s": self.duration.as_secs_f64(),
            "cut_silence_s": SILENCE.as_secs_f64(),
            "recovery_deadline_s": RECOVERY_DEADLINE.as_secs_f64(),
            "vehicle_metrics": self.vehicle_metrics,
            "relay_metrics": self.relay_metrics,
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
    if let Err(e) = sat_teleop_media::init() {
        results.record("video_received", Verdict::Incomplete, e.to_string());
        return results;
    }
    let settled = adaptation::settle_at_top(&AgentClient::new(&settings.vehicle_metrics));
    adaptation::record_settled(&mut results, &settled);
    let capture = Capture::start(Source::Rtsp(settings.url.clone()));
    thread::sleep(settings.warmup);
    let start = Reading::take(agent, settings, &capture);
    thread::sleep(settings.duration);
    let end = Reading::take(agent, settings, &capture);
    let window = capture.read(|all| Window::measure(all, start.at, end.at));
    let cut = window
        .ages_ms
        .last()
        .map(|&age_max| cut_and_restore(&mut results, agent, &capture, drain_for(age_max)));
    let captured = capture.finish();
    record_window(&mut results, &window, &end.receiver, captured.dropped);
    record_rates(&mut results, &start, &end);
    match cut {
        Some(cut) => record_cut(&mut results, &cut, &captured.observations),
        None => {
            for name in ["forwarding_cut", "video_recovery"] {
                results.record(name, Verdict::Incomplete, "no video before the cut");
            }
        }
    }
    results.samples = samples(&captured.observations);
    results.readings = json!({
        "start": start.to_json(),
        "end": end.to_json(),
        "receiver_at_finish": captured.receiver,
    });
    budget::grade(&mut results, budget);
    results
}

/// The headless receiver on one thread, restarted after a failure, and its observations collected
/// on another.
pub(crate) struct Capture {
    stop: Arc<AtomicBool>,
    health: Arc<Health>,
    collected: Arc<Mutex<Collected>>,
    threads: [JoinHandle<()>; 2],
}

#[derive(Debug, Default)]
struct Collected {
    observations: Vec<Observation>,
    dropped: usize,
}

pub(crate) struct Captured {
    pub(crate) observations: Vec<Observation>,
    pub(crate) dropped: usize,
    pub(crate) receiver: HealthSnapshot,
}

impl Capture {
    pub(crate) fn start(source: Source) -> Self {
        let (sender, observations) = sync_channel(OBSERVATION_QUEUE);
        let stop = Arc::new(AtomicBool::new(false));
        let health = Arc::new(Health::default());
        let collected = Arc::new(Mutex::new(Collected::default()));
        let receiver = {
            let (stop, health) = (Arc::clone(&stop), Arc::clone(&health));
            thread::spawn(move || supervise(&health, || receiver::receive(&source, &stop, &sender)))
        };
        let collector = {
            let collected = Arc::clone(&collected);
            thread::spawn(move || {
                for observation in observations {
                    if let Ok(mut collected) = collected.lock() {
                        collected.push(observation);
                    }
                }
            })
        };
        Self {
            stop,
            health,
            collected,
            threads: [receiver, collector],
        }
    }

    pub(crate) fn read<T: Default>(&self, f: impl FnOnce(&[Observation]) -> T) -> T {
        self.collected
            .lock()
            .map(|collected| f(&collected.observations))
            .unwrap_or_default()
    }

    pub(crate) fn finish(self) -> Captured {
        self.stop.store(true, Ordering::Relaxed);
        for thread in self.threads {
            let _ = thread.join();
        }
        let collected = self
            .collected
            .lock()
            .map(|mut c| std::mem::take(&mut *c))
            .unwrap_or_default();
        Captured {
            observations: collected.observations,
            dropped: collected.dropped,
            receiver: self.health.snapshot(),
        }
    }
}

impl Collected {
    fn push(&mut self, observation: Observation) {
        if self.observations.len() < MAX_OBSERVATIONS {
            self.observations.push(observation);
        } else {
            self.dropped += 1;
        }
    }
}

/// Router counters, application metrics and the receiver's health at one moment.
struct Reading {
    at: Instant,
    unix_ms: u64,
    receiver: HealthSnapshot,
    router: Result<Value, String>,
    vehicle: Result<Value, String>,
    relay: Result<Value, String>,
}

impl Reading {
    fn take(agent: &AgentClient, settings: &Settings, capture: &Capture) -> Self {
        Self {
            at: Instant::now(),
            unix_ms: unix_ms(),
            receiver: capture.health.snapshot(),
            router: check::get_json(agent, "/counters"),
            vehicle: check::get_json(&AgentClient::new(&settings.vehicle_metrics), "/metrics"),
            relay: check::get_json(&AgentClient::new(&settings.relay_metrics), "/metrics"),
        }
    }

    fn to_json(&self) -> Value {
        let value = |r: &Result<Value, String>| match r {
            Ok(v) => v.clone(),
            Err(e) => json!({ "error": e }),
        };
        json!({
            "unix_ms": self.unix_ms,
            "receiver": self.receiver,
            "router": value(&self.router),
            "vehicle": value(&self.vehicle),
            "relay": value(&self.relay),
        })
    }
}

/// What the receiver saw between two instants. Only new frames (a readable, plausible stamp with a
/// frame id not seen before) count towards age and cadence.
#[derive(Debug, Default, PartialEq)]
struct Window {
    seconds: f64,
    observations: usize,
    unreadable: usize,
    implausible: usize,
    repeated: usize,
    /// Frame ids missing between consecutive new frames: lost anywhere from encoder to decoder.
    skipped_ids: u64,
    /// Ages of the new frames, sorted.
    ages_ms: Vec<f64>,
    /// Longest wait for a new frame, counting from the window start and up to its end.
    max_gap_ms: f64,
}

impl Window {
    fn measure(observations: &[Observation], start: Instant, end: Instant) -> Self {
        let mut window = Self {
            seconds: end.saturating_duration_since(start).as_secs_f64(),
            ..Self::default()
        };
        let mut seen = HashSet::new();
        let mut last_new = start;
        let mut last_id = None;
        let mut max_gap = Duration::ZERO;
        let inside = observations
            .iter()
            .filter(|o| (start..=end).contains(&o.received_at));
        for observation in inside {
            window.observations += 1;
            let Some(stamp) = observation.stamp else {
                window.unreadable += 1;
                continue;
            };
            let Some(age) = plausible_age(observation.received_ms, stamp) else {
                window.implausible += 1;
                continue;
            };
            if !seen.insert(stamp.frame_id.0) {
                window.repeated += 1;
                continue;
            }
            window.skipped_ids += skipped(last_id, stamp.frame_id.0);
            last_id = Some(stamp.frame_id.0);
            window.ages_ms.push(f64::from(age));
            max_gap = max_gap.max(observation.received_at.saturating_duration_since(last_new));
            last_new = observation.received_at;
        }
        max_gap = max_gap.max(end.saturating_duration_since(last_new));
        window.max_gap_ms = max_gap.as_secs_f64() * 1000.0;
        window.ages_ms.sort_by(f64::total_cmp);
        window
    }
}

/// Ids between `previous` and `id`; none when ids went backwards (the sender restarted).
fn skipped(previous: Option<u32>, id: u32) -> u64 {
    previous
        .and_then(|previous| id.checked_sub(previous))
        .map_or(0, |step| u64::from(step.saturating_sub(1)))
}

/// `receiver` is the receiver's health when the window ended, before the deliberate cut.
fn record_window(
    results: &mut Results,
    window: &Window,
    receiver: &HealthSnapshot,
    dropped: usize,
) {
    let new_frames = window.ages_ms.len();
    let metrics = [
        ("frame_age_p50_ms", percentile(&window.ages_ms, 50)),
        ("frame_age_p95_ms", percentile(&window.ages_ms, 95)),
        ("frame_age_max_ms", window.ages_ms.last().copied()),
        (
            "new_frames_per_s",
            (window.seconds > 0.0).then(|| float(new_frames as u64) / window.seconds),
        ),
        ("max_frame_gap_ms", Some(window.max_gap_ms)),
        ("unreadable_stamps", Some(float(window.unreadable as u64))),
        (
            "unreadable_ratio",
            (window.observations > 0)
                .then(|| float(window.unreadable as u64) / float(window.observations as u64)),
        ),
        ("implausible_stamps", Some(float(window.implausible as u64))),
        ("repeated_frames", Some(float(window.repeated as u64))),
        ("skipped_frame_ids", Some(float(window.skipped_ids))),
        ("receiver_runs", Some(float(receiver.runs))),
        ("observations_dropped", Some(float(dropped as u64))),
    ];
    results.metrics.extend(
        metrics
            .into_iter()
            .filter_map(|(name, value)| Some((name, value?))),
    );
    results
        .sample_counts
        .insert("observations", window.observations);
    results.sample_counts.insert("new_frames", new_frames);
    let detail = format!(
        "{new_frames} new frames in {:.1} s ({} observations, {} unreadable); receiver runs: {}",
        window.seconds, window.observations, window.unreadable, receiver.runs
    );
    let verdict = if new_frames > 0 {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record("video_received", verdict, detail);
}

fn record_rates(results: &mut Results, start: &Reading, end: &Reading) {
    let seconds = end.at.saturating_duration_since(start.at).as_secs_f64();
    let per_second =
        |count: Option<u64>| count.filter(|_| seconds > 0.0).map(|c| float(c) / seconds);
    let rates = [
        (
            "uplink_bytes_per_s",
            per_second(increase(&start.router, &end.router, "/uplink/net/tx_bytes")),
        ),
        (
            "relay_rtp_bytes_per_s",
            per_second(increase(&start.relay, &end.relay, "/rtp_bytes")),
        ),
        (
            "vehicle_frames_per_s",
            per_second(increase(&start.vehicle, &end.vehicle, "/frames_encoded")),
        ),
        (
            "relay_lost_packets",
            increase(&start.relay, &end.relay, "/lost").map(float),
        ),
        ("uplink_drops", root_qdisc(start, end, "drops")),
    ];
    results.metrics.extend(
        rates
            .into_iter()
            .filter_map(|(name, value)| Some((name, value?))),
    );
}

/// Increase of a counter of the uplink root qdisc (handle `1:`).
fn root_qdisc(start: &Reading, end: &Reading, field: &str) -> Option<f64> {
    let read = |reading: &Reading| {
        let qdiscs = reading
            .router
            .as_ref()
            .ok()?
            .pointer("/uplink/qdiscs")?
            .as_array()?;
        let root = qdiscs.iter().find(|q| q["handle"] == "1:")?;
        root.get(field)?.as_u64()
    };
    read(end)?.checked_sub(read(start)?).map(float)
}

/// When the forwarding cut and the restore happened, and the first fresh frame after it.
struct Cut {
    at: Instant,
    unix_ms: u64,
    drain: Duration,
    restored_at: Option<Instant>,
    error: Option<String>,
    recovered: Option<Observation>,
}

fn cut_and_restore(
    results: &mut Results,
    agent: &AgentClient,
    capture: &Capture,
    drain: Duration,
) -> Cut {
    let mut cut = Cut {
        at: Instant::now(),
        unix_ms: unix_ms(),
        drain,
        restored_at: None,
        error: None,
        recovered: None,
    };
    if let Err(e) = agent.post("/forwarding", r#"{"enabled": false}"#) {
        cut.error = Some(format!("cannot cut forwarding: {e}"));
    }
    thread::sleep(drain.saturating_add(SILENCE));
    // Restore even when the cut failed, so the testbed is left forwarding.
    if let Err(e) = agent.post("/forwarding", r#"{"enabled": true}"#) {
        results.record(
            "video_recovery",
            Verdict::Fail,
            format!("cannot restore forwarding: {e}"),
        );
        return cut;
    }
    let restored_at = Instant::now();
    cut.restored_at = Some(restored_at);
    while restored_at.elapsed() < RECOVERY_DEADLINE && cut.recovered.is_none() {
        thread::sleep(POLL_EVERY);
        cut.recovered = capture.read(|all| first_fresh(all, restored_at, cut.unix_ms));
    }
    cut
}

/// The first new frame after `restored_at` that was captured after the cut began.
fn first_fresh(
    observations: &[Observation],
    restored_at: Instant,
    cut_unix_ms: u64,
) -> Option<Observation> {
    let cut_ms = CaptureMs::from_unix_ms(cut_unix_ms).0;
    observations.iter().copied().find(|o| {
        let Some(stamp) = o.stamp else {
            return false;
        };
        let captured_after_cut = stamp.capture_ms.0.wrapping_sub(cut_ms) <= MAX_PLAUSIBLE_AGE_MS;
        let plausible = plausible_age(o.received_ms, stamp).is_some();
        o.received_at > restored_at && captured_after_cut && plausible
    })
}

fn record_cut(results: &mut Results, cut: &Cut, observations: &[Observation]) {
    results
        .metrics
        .insert("cut_drain_ms", cut.drain.as_secs_f64() * 1000.0);
    let before_restore = |o: &&Observation| cut.restored_at.is_none_or(|r| o.received_at <= r);
    let after_cut: Vec<&Observation> = observations
        .iter()
        .filter(|o| o.received_at >= cut.at)
        .filter(before_restore)
        .collect();
    if let Some(last) = after_cut.last() {
        let ms = last
            .received_at
            .saturating_duration_since(cut.at)
            .as_secs_f64()
            * 1000.0;
        results.metrics.insert("cut_last_frame_ms", ms);
    }
    let silence_from = cut.at + cut.drain;
    let in_silence = after_cut
        .iter()
        .filter(|o| o.received_at > silence_from)
        .count();
    let (verdict, detail) = match &cut.error {
        Some(error) => (Verdict::Fail, error.clone()),
        None if in_silence > 0 => (
            Verdict::Fail,
            format!(
                "{in_silence} frames arrived more than {} ms after the cut",
                cut.drain.as_millis()
            ),
        ),
        None => (
            Verdict::Pass,
            format!(
                "no frames from {} ms after the cut until the restore ({} drained before)",
                cut.drain.as_millis(),
                after_cut.len()
            ),
        ),
    };
    results.record("forwarding_cut", verdict, detail);
    let Some(restored_at) = cut.restored_at else {
        return;
    };
    let Some(fresh) = cut.recovered else {
        let detail = format!(
            "no fresh frame within {} s of the restore",
            RECOVERY_DEADLINE.as_secs()
        );
        results.record("video_recovery", Verdict::Fail, detail);
        return;
    };
    let ms = fresh
        .received_at
        .saturating_duration_since(restored_at)
        .as_secs_f64()
        * 1000.0;
    results.metrics.insert("video_recovery_ms", ms);
    let age = fresh
        .stamp
        .and_then(|s| plausible_age(fresh.received_ms, s));
    if let Some(age) = age {
        results
            .metrics
            .insert("recovery_frame_age_ms", f64::from(age));
    }
    results.record(
        "video_recovery",
        Verdict::Pass,
        format!("fresh frame {ms:.0} ms after the restore"),
    );
}

/// One row per observation: `frame_age` for new frames, `repeated`, `implausible` (raw age) and
/// `unreadable` (no value).
fn samples(observations: &[Observation]) -> Vec<Sample> {
    let mut seen = HashSet::new();
    observations
        .iter()
        .map(|o| {
            let unix_s = float(o.received_ms) / 1000.0;
            let Some(stamp) = o.stamp else {
                return Sample {
                    unix_s,
                    series: "unreadable",
                    seq: 0,
                    value: f64::NAN,
                };
            };
            let age = stamp::age_ms(CaptureMs::from_unix_ms(o.received_ms).0, stamp);
            let series = if plausible_age(o.received_ms, stamp).is_none() {
                "implausible"
            } else if seen.insert(stamp.frame_id.0) {
                "frame_age"
            } else {
                "repeated"
            };
            Sample {
                unix_s,
                series,
                seq: stamp.frame_id.0,
                value: f64::from(age),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use sat_teleop_policy::stamp::{FrameId, Stamp};

    use super::*;

    const T0_MS: u64 = 1_800_000_000_000;

    /// An observation `at_ms` after `t0`, of frame `id` captured `age_ms` earlier (or unreadable).
    fn seen(t0: Instant, at_ms: u64, frame: Option<(u32, u64)>) -> Observation {
        let received_ms = T0_MS + at_ms;
        Observation {
            received_at: t0 + Duration::from_millis(at_ms),
            received_ms,
            stamp: frame.map(|(id, age_ms)| Stamp {
                frame_id: FrameId(id),
                capture_ms: CaptureMs::from_unix_ms(received_ms - age_ms),
            }),
            state: None,
        }
    }

    #[test]
    fn counts_only_new_plausible_frames_towards_age_and_cadence() {
        let t0 = Instant::now();
        let observations = [
            seen(t0, 100, Some((1, 120))),
            seen(t0, 200, Some((2, 130))),
            seen(t0, 300, Some((2, 230))),       // repeated frame
            seen(t0, 400, None),                 // unreadable
            seen(t0, 500, Some((5, 140))),       // ids 3 and 4 never arrived
            seen(t0, 600, Some((6, 3_000_000))), // implausible: a misread stamp
            seen(t0, 1200, Some((7, 110))),      // 6 counts as skipped
            seen(t0, 2500, Some((8, 100))),      // after the window
        ];
        let window = Window::measure(&observations, t0, t0 + Duration::from_secs(2));
        assert_eq!(
            window,
            Window {
                seconds: 2.0,
                observations: 7,
                unreadable: 1,
                implausible: 1,
                repeated: 1,
                skipped_ids: 3,
                ages_ms: vec![110.0, 120.0, 130.0, 140.0],
                max_gap_ms: 800.0,
            }
        );
    }

    #[test]
    fn silence_is_one_gap_the_length_of_the_window() {
        let t0 = Instant::now();
        let window = Window::measure(&[], t0, t0 + Duration::from_secs(30));
        assert_eq!(window.max_gap_ms, 30_000.0);
        assert_eq!(window.ages_ms, Vec::<f64>::new());
    }

    #[test]
    fn recovery_needs_a_frame_captured_after_the_cut() {
        let t0 = Instant::now();
        let cut_ms = T0_MS + 1000;
        let restored_at = t0 + Duration::from_millis(4000);
        let observations = [
            seen(t0, 900, Some((10, 100))),
            seen(t0, 4100, Some((11, 3200))), // stale: captured before the cut
            seen(t0, 4200, None),
            seen(t0, 4300, Some((60, 200))),
        ];
        let fresh = first_fresh(&observations, restored_at, cut_ms).unwrap();
        assert_eq!(fresh.stamp.unwrap().frame_id, FrameId(60));
        assert!(first_fresh(&observations[..3], restored_at, cut_ms).is_none());
    }
}
