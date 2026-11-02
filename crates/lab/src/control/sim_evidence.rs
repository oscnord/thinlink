//! What `sim-flight` records beside its verdicts: the vehicle's control endpoint and adaptation in
//! each phase, every mode change with who asked for it, the autopilot's status texts and how the
//! video read. Evidence for the person reading a result, graded only where `sim_flight` says so.

use std::collections::BTreeMap;
use std::time::Instant;

use serde_json::{Map, Value, json};

use sat_teleop_media::receiver::Observation;

use super::sim_flight::{count, record_metrics, state};
use super::sim_probe::{self, Reading};
use crate::check::{Results, Sample, float};
use crate::mavlink::gcs::{Kind, Seen};
use crate::qgc_status::Healthy;

/// A part of the flight whose causes are recorded apart.
#[derive(Debug, Clone, Copy)]
pub(super) enum Phase {
    Takeoff,
    Leg,
    Brake,
    Land,
}

impl Phase {
    /// Its name, and the names of its metrics: refused envelopes, the uplink's largest queue delay
    /// and the lease's losses.
    pub const fn names(self) -> [&'static str; 4] {
        match self {
            Self::Takeoff => [
                "takeoff",
                "rejected_takeoff",
                "queue_delay_max_ms_takeoff",
                "lease_losses_takeoff",
            ],
            Self::Leg => [
                "leg",
                "rejected_leg",
                "queue_delay_max_ms_leg",
                "lease_losses_leg",
            ],
            Self::Brake => [
                "brake",
                "rejected_brake",
                "queue_delay_max_ms_brake",
                "lease_losses_brake",
            ],
            Self::Land => [
                "land",
                "rejected_land",
                "queue_delay_max_ms_land",
                "lease_losses_land",
            ],
        }
    }
}

/// When a phase began and ended.
#[derive(Debug, Clone, Copy)]
pub(super) struct PhaseRun {
    pub phase: Phase,
    pub from: Instant,
    pub to: Instant,
    pub from_unix_ms: u64,
    pub to_unix_ms: u64,
}

/// A mode the check asked the autopilot for (UNIX ms) and whether it accepted.
#[derive(Debug, Clone, Copy)]
pub(super) struct Commanded {
    pub unix_ms: u64,
    pub mode: u32,
    pub accepted: bool,
}

/// A change of the copter's mode, as its heartbeats report it.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct ModeChange {
    pub unix_ms: u64,
    pub from: u32,
    pub to: u32,
}

/// A mode counts as asked for when the autopilot reports it this soon after the request.
const COMMANDED_WITHIN_MS: u64 = 5_000;
/// What the autopilot said within this long of a mode change is attached to it.
const SAID_BEFORE_MS: u64 = 3_000;
const SAID_AFTER_MS: u64 = 1_000;

pub(super) fn mode_changes(seen: &[Seen]) -> Vec<ModeChange> {
    let mut changes = Vec::new();
    let mut previous = None;
    for (s, (mode, _)) in seen.iter().filter_map(|s| Some((s, state(s)?))) {
        if let Some(from) = previous.filter(|from| *from != mode) {
            changes.push(ModeChange {
                unix_ms: s.unix_ms,
                from,
                to: mode,
            });
        }
        previous = Some(mode);
    }
    changes
}

/// The autopilot's `STATUSTEXT`s (UNIX ms, text).
pub(super) fn status_texts(seen: &[Seen]) -> Vec<(u64, String)> {
    seen.iter()
        .filter(|s| s.from_vehicle)
        .filter_map(|s| match s.kind {
            Kind::StatusText { text } => Some((s.unix_ms, text.to_str().ok()?.to_owned())),
            _ => None,
        })
        .collect()
}

fn mode_change_json(change: &ModeChange, commanded: bool, texts: &[(u64, String)]) -> Value {
    let said: Vec<&str> = texts
        .iter()
        .filter(|(at, _)| {
            (change.unix_ms.saturating_sub(SAID_BEFORE_MS)..=change.unix_ms + SAID_AFTER_MS)
                .contains(at)
        })
        .map(|(_, text)| text.as_str())
        .collect();
    json!({
        "unix_ms": change.unix_ms,
        "from": change.from,
        "to": change.to,
        "commanded": commanded,
        "status_texts": said,
    })
}

/// The leg's frames at the ground output by what they read: the labels, the state stamps and the
/// in-picture stamps that read back and were plausible, in all and per second.
pub(super) fn video_json(all: &[Observation], from: Instant, to: Instant) -> Value {
    let window = Healthy::measure(all, from, to);
    let mut labels: BTreeMap<String, u64> = BTreeMap::new();
    let mut per_second: BTreeMap<u64, (u64, u64)> = BTreeMap::new();
    let inside = all.iter().filter(|o| (from..=to).contains(&o.received_at));
    let mut stamped = 0_u64;
    for o in inside {
        let label = o
            .state
            .map_or_else(|| "unreadable".to_owned(), |s| format!("{:?}", s.state));
        let live = label == "Live";
        *labels.entry(label).or_default() += 1;
        stamped += u64::from(o.stamp.is_some());
        let second = o.received_at.saturating_duration_since(from).as_secs();
        let bucket = per_second.entry(second).or_default();
        bucket.0 += 1;
        bucket.1 += u64::from(live);
    }
    json!({
        "labels": labels,
        "frames": window.frames,
        "in_picture_stamps_read": stamped,
        "new_frames_with_plausible_stamps": window.ages_ms.len(),
        "per_second_frames_and_live": per_second,
    })
}

/// What the vehicle's control endpoint and adaptation said in each phase: the metrics, and the
/// phases' readings for the evidence file.
pub(super) fn record_phases(
    results: &mut Results,
    readings: &[Reading],
    phases: &[PhaseRun],
) -> Map<String, Value> {
    let mut detail = Map::new();
    for run in phases {
        let w = sim_probe::window(readings, run.from, run.to);
        let [name, rejected, queue_delay, lease_losses] = run.phase.names();
        record_metrics(
            results,
            [
                (rejected, Some(float(w.rejected_total()))),
                (queue_delay, w.queue_delay_max_ms),
                (lease_losses, Some(float(w.lease_losses))),
            ],
        );
        let reading = json!({
            "from_unix_ms": run.from_unix_ms,
            "to_unix_ms": run.to_unix_ms,
            "window": w.to_json(),
        });
        detail.insert(name.to_owned(), reading);
    }
    detail
}

/// What the run as a whole says: the adaptation and the uplink, and every mode change with who
/// asked for it and what the autopilot said about it. Writes the evidence file's readings, with
/// the `phases` and the video already there.
pub(super) fn record_run(
    results: &mut Results,
    readings: &[Reading],
    phases: &Map<String, Value>,
    run_from: Instant,
    commanded: &[Commanded],
    seen: &[Seen],
) {
    let run = sim_probe::window(readings, run_from, Instant::now());
    let changes = mode_changes(seen);
    let texts = status_texts(seen);
    let uncommanded = changes
        .iter()
        .filter(|c| !was_commanded(commanded, c))
        .count();
    record_metrics(
        results,
        [
            ("adaptation_drains", Some(count(run.drains()))),
            ("uplink_queue_delay_max_ms", run.queue_delay_max_ms),
            ("mode_changes", Some(count(changes.len()))),
            ("uncommanded_mode_changes", Some(count(uncommanded))),
        ],
    );
    let changes_json: Vec<Value> = changes
        .iter()
        .map(|c| mode_change_json(c, was_commanded(commanded, c), &texts))
        .collect();
    results.readings = json!({
        "video": std::mem::take(&mut results.readings),
        "phases": phases,
        "run": run.to_json(),
        "mode_changes": changes_json,
        "status_texts": texts.iter().map(|(at, t)| json!({ "unix_ms": at, "text": t })).collect::<Vec<_>>(),
    });
    results.samples.extend(sim_probe::samples(readings));
    results
        .samples
        .extend(changes.iter().zip(0_u32..).map(|(c, seq)| Sample {
            unix_s: float(c.unix_ms) / 1000.0,
            series: "mode",
            seq,
            value: f64::from(c.to),
        }));
}

/// Whether the check asked for the mode `change` went to, shortly before.
fn was_commanded(commanded: &[Commanded], change: &ModeChange) -> bool {
    commanded.iter().any(|c| {
        c.accepted
            && c.mode == change.to
            && (c.unix_ms..=c.unix_ms + COMMANDED_WITHIN_MS).contains(&change.unix_ms)
    })
}
