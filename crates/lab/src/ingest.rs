//! `lab check ingest`: the vehicle takes its video from the camera ingest while the feeder container
//! sends a test pattern from the address the vehicle expects and a trickle from one it does not.
//! The ingest must decode the first and drop the second, at a decode cost that is recorded. How
//! the frames fare through the link is `skeleton`'s to grade. `docs/demo.md` explains the setup.

use std::thread;
use std::time::Duration;

use serde_json::{Value, json};

use crate::check::{self, Results, Verdict, float};
use crate::client::AgentClient;

/// The feeder's frame rate; the ingest must decode at least this share of it.
const FEEDER_FPS: f64 = 30.0;
const MIN_DECODED_SHARE: f64 = 0.8;
/// A decode longer than this on average would eat into the frame age budget.
const MAX_DECODE_MEAN_MS: f64 = 20.0;

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    pub warmup: Duration,
    pub duration: Duration,
    pub vehicle_metrics: String,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "warmup_s": self.warmup.as_secs_f64(),
            "duration_s": self.duration.as_secs_f64(),
            "vehicle_metrics": self.vehicle_metrics,
        })
    }
}

pub fn run(agent: &AgentClient, profile_text: &str, settings: &Settings) -> Results {
    let mut results = Results {
        thresholds: json!({
            "feeder_fps": FEEDER_FPS,
            "min_decoded_share": MIN_DECODED_SHARE,
            "max_decode_mean_ms": MAX_DECODE_MEAN_MS,
        }),
        settings: settings.to_json(),
        ..Results::default()
    };
    if !check::apply_profile(&mut results, agent, profile_text) {
        return results;
    }
    thread::sleep(settings.warmup);
    let vehicle = AgentClient::new(&settings.vehicle_metrics);
    let start = check::get_json(&vehicle, "/metrics");
    thread::sleep(settings.duration);
    let end = check::get_json(&vehicle, "/metrics");
    results.readings = json!({
        "vehicle_at_start": start.clone().unwrap_or(Value::Null),
        "vehicle_at_end": end.clone().unwrap_or(Value::Null),
    });
    check_decoded(&mut results, &start, &end, settings.duration);
    check_refused(&mut results, &start, &end);
    check_decode_cost(&mut results, &end);
    results
}

fn check_decoded(
    results: &mut Results,
    start: &Result<Value, String>,
    end: &Result<Value, String>,
    duration: Duration,
) {
    let name = "feeder_decoded";
    let Some(frames) = check::increase(start, end, "/ingest/frames_decoded") else {
        results.record(name, Verdict::Incomplete, "no readings of the ingest");
        return;
    };
    let wanted = FEEDER_FPS * duration.as_secs_f64() * MIN_DECODED_SHARE;
    let verdict = if float(frames) >= wanted {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record(
        name,
        verdict,
        format!("{frames} frames decoded in {duration:?}, at least {wanted:.0} wanted"),
    );
}

fn check_refused(
    results: &mut Results,
    start: &Result<Value, String>,
    end: &Result<Value, String>,
) {
    let name = "foreign_address_refused";
    let Some(refused) = check::increase(start, end, "/ingest/refused") else {
        results.record(name, Verdict::Incomplete, "no readings of the ingest");
        return;
    };
    let verdict = if refused > 0 {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record(
        name,
        verdict,
        format!("{refused} datagrams from the feeder's second address were dropped"),
    );
}

fn check_decode_cost(results: &mut Results, end: &Result<Value, String>) {
    let name = "decode_cost";
    let reading = |pointer: &str| end.as_ref().ok()?.pointer(pointer)?.as_f64();
    let (Some(mean), Some(max)) = (
        reading("/ingest/decode_ms_mean"),
        reading("/ingest/decode_ms_max"),
    ) else {
        results.record(name, Verdict::Incomplete, "no readings of the ingest");
        return;
    };
    results.metrics.insert("ingest_decode_ms_mean", mean);
    results.metrics.insert("ingest_decode_ms_max", max);
    let verdict = if mean <= MAX_DECODE_MEAN_MS {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record(
        name,
        verdict,
        format!("decode mean {mean:.2} ms, max {max:.2} ms since the vehicle started"),
    );
}
