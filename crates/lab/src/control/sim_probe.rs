//! What the vehicle's control endpoint and uplink adaptation say while `sim-flight` runs, read on a
//! thread every `EVERY` so the check's own timing is untouched: the refusals per reason, the lease,
//! the uplink's queue delay and round trip, the adaptation's steps and the age of the challenge the
//! ground holds. A phase's window of readings says why the copter did what it did there.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use sat_teleop_media::unix_ms;

use crate::check::{self, Sample, float};
use crate::client::AgentClient;

const EVERY: Duration = Duration::from_millis(250);

/// One adaptation step the vehicle logged.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Step {
    pub reason: String,
    pub to: u64,
    pub unix_ms: u64,
}

#[derive(Debug, Clone)]
pub(super) struct Reading {
    pub at: Instant,
    pub unix_ms: u64,
    /// Refused envelopes per reason, counted since the vehicle started.
    pub rejected: BTreeMap<String, u64>,
    pub lease: bool,
    pub queue_delay_ms: Option<f64>,
    pub rtt_ms: Option<f64>,
    /// The age of the newest challenge at the ground.
    pub challenge_age_ms: Option<u64>,
    pub steps: Vec<Step>,
}

fn reading(vehicle: &Value, ground: Option<&Value>) -> Reading {
    let number = |pointer: &str| vehicle.pointer(pointer)?.as_f64();
    let rejected = vehicle
        .pointer("/mavlink/control/rejected")
        .and_then(Value::as_object)
        .into_iter()
        .flatten()
        .filter_map(|(reason, n)| Some((reason.clone(), n.as_u64()?)))
        .collect();
    let steps = vehicle
        .pointer("/adapt/recent_changes")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|c| {
            Some(Step {
                reason: c.get("reason")?.as_str()?.to_owned(),
                to: c.get("to")?.as_u64()?,
                unix_ms: c.get("unix_ms")?.as_u64()?,
            })
        })
        .collect();
    Reading {
        at: Instant::now(),
        unix_ms: unix_ms(),
        rejected,
        lease: vehicle
            .pointer("/mavlink/control/lease")
            .is_some_and(|l| !l.is_null()),
        queue_delay_ms: number("/adapt/queue_delay_ms"),
        rtt_ms: number("/adapt/rtt_ms"),
        challenge_age_ms: ground
            .and_then(|g| g.pointer("/control/status/challenge_age_ms")?.as_u64()),
        steps,
    }
}

pub(super) struct Probe {
    readings: Arc<Mutex<Vec<Reading>>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Probe {
    pub fn start(vehicle: &str, ground: &str) -> Self {
        let readings = Arc::new(Mutex::new(Vec::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let (vehicle, ground) = (AgentClient::new(vehicle), AgentClient::new(ground));
        let thread = {
            let (readings, stop) = (Arc::clone(&readings), Arc::clone(&stop));
            thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    let started = Instant::now();
                    if let Ok(v) = check::get_json(&vehicle, "/metrics")
                        && let Ok(mut all) = readings.lock()
                    {
                        let g = check::get_json(&ground, "/metrics").ok();
                        all.push(reading(&v, g.as_ref()));
                    }
                    thread::sleep(EVERY.saturating_sub(started.elapsed()));
                }
            })
        };
        Self {
            readings,
            stop,
            thread: Some(thread),
        }
    }

    pub fn readings(&self) -> Vec<Reading> {
        self.readings.lock().map(|r| r.clone()).unwrap_or_default()
    }
}

impl Drop for Probe {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// What happened between two moments.
#[derive(Debug, Default, PartialEq)]
pub(super) struct Window {
    /// Envelopes refused per reason.
    pub rejected: BTreeMap<String, u64>,
    pub queue_delay_max_ms: Option<f64>,
    pub rtt_max_ms: Option<f64>,
    pub challenge_age_max_ms: Option<u64>,
    pub lease_losses: u64,
    pub steps: Vec<Step>,
}

impl Window {
    pub fn rejected_total(&self) -> u64 {
        self.rejected.values().sum()
    }

    pub fn drains(&self) -> usize {
        self.steps.iter().filter(|s| s.reason == "drain").count()
    }

    pub fn to_json(&self) -> Value {
        let steps: Vec<Value> = self
            .steps
            .iter()
            .map(|s| json!({ "reason": s.reason, "to": s.to, "unix_ms": s.unix_ms }))
            .collect();
        json!({
            "rejected": self.rejected,
            "queue_delay_max_ms": self.queue_delay_max_ms,
            "rtt_max_ms": self.rtt_max_ms,
            "challenge_age_max_ms": self.challenge_age_max_ms,
            "lease_losses": self.lease_losses,
            "adaptation_steps": steps,
        })
    }
}

/// The window from `from` to `to`: counter deltas against the last reading before `from` (or the
/// first inside), maxima and the lease's losses over the readings inside it.
pub(super) fn window(readings: &[Reading], from: Instant, to: Instant) -> Window {
    let inside: Vec<&Reading> = readings
        .iter()
        .filter(|r| r.at >= from && r.at <= to)
        .collect();
    let base = readings
        .iter()
        .rfind(|r| r.at < from)
        .or(inside.first().copied());
    let (Some(base), Some(last)) = (base, inside.last()) else {
        return Window::default();
    };
    let rejected = last
        .rejected
        .iter()
        .map(|(reason, n)| {
            let before = base.rejected.get(reason).copied().unwrap_or(0);
            (reason.clone(), n.saturating_sub(before))
        })
        .filter(|(_, n)| *n > 0)
        .collect();
    let mut lease = base.lease;
    let mut lease_losses = 0;
    for r in &inside {
        lease_losses += u64::from(lease && !r.lease);
        lease = r.lease;
    }
    Window {
        rejected,
        queue_delay_max_ms: max(inside.iter().filter_map(|r| r.queue_delay_ms)),
        rtt_max_ms: max(inside.iter().filter_map(|r| r.rtt_ms)),
        challenge_age_max_ms: inside.iter().filter_map(|r| r.challenge_age_ms).max(),
        lease_losses,
        steps: last
            .steps
            .iter()
            .filter(|s| s.unix_ms >= base.unix_ms)
            .cloned()
            .collect(),
    }
}

fn max(values: impl Iterator<Item = f64>) -> Option<f64> {
    values.reduce(f64::max)
}

/// Every reading as timeseries samples.
pub(super) fn samples(readings: &[Reading]) -> impl Iterator<Item = Sample> + '_ {
    readings.iter().zip(0_u32..).flat_map(|(r, seq)| {
        let unix_s = float(r.unix_ms) / 1000.0;
        let rejected: u64 = r.rejected.values().sum();
        let series = [
            ("queue_delay_ms", r.queue_delay_ms),
            ("rtt_ms", r.rtt_ms),
            ("challenge_age_ms", r.challenge_age_ms.map(float)),
            ("rejected", Some(float(rejected))),
            ("lease", Some(f64::from(u8::from(r.lease)))),
        ];
        series.into_iter().filter_map(move |(series, value)| {
            Some(Sample {
                unix_s,
                series,
                seq,
                value: value?,
            })
        })
    })
}
