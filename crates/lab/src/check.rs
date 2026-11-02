//! The `lab check` results contract: verdicts, `summary.json` and `timeseries.csv`.

use std::collections::BTreeMap;
use std::fmt::{self, Write as _};
use std::fs;
use std::io;
use std::path::Path;
use std::time::Duration;

use serde::Serialize;
use serde_json::Value;

use crate::client::AgentClient;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Verdict {
    Pass,
    Fail,
    /// Not measured or not graded. Never a pass.
    Incomplete,
}

impl Verdict {
    pub fn exit_code(self) -> u8 {
        match self {
            Self::Pass => 0,
            Self::Fail => 1,
            Self::Incomplete => 2,
        }
    }

    /// Any failure fails the run. Otherwise anything unmeasured, or nothing at all, is incomplete.
    pub fn overall(verdicts: &[Self]) -> Self {
        if verdicts.contains(&Self::Fail) {
            return Self::Fail;
        }
        if verdicts.is_empty() || verdicts.contains(&Self::Incomplete) {
            return Self::Incomplete;
        }
        Self::Pass
    }
}

impl fmt::Display for Verdict {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.pad(match self {
            Self::Pass => "pass",
            Self::Fail => "fail",
            Self::Incomplete => "incomplete",
        })
    }
}

#[derive(Debug, Serialize)]
pub struct Assertion {
    pub name: String,
    pub verdict: Verdict,
    pub detail: String,
}

/// One `timeseries.csv` row: a value of one series at a time, `seq` identifying it within the
/// series (a ping sequence number, a frame id).
#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub unix_s: f64,
    pub series: &'static str,
    pub seq: u32,
    pub value: f64,
}

/// What a scenario observed. `samples` go to `timeseries.csv`, everything else to `summary.json`.
#[derive(Debug, Default, Serialize)]
pub struct Results {
    pub assertions: Vec<Assertion>,
    pub metrics: BTreeMap<&'static str, f64>,
    pub sample_counts: BTreeMap<&'static str, usize>,
    /// The grading thresholds this run used.
    pub thresholds: Value,
    /// What the router agent reported applying (commands and `tc` state).
    pub effective_profile: Value,
    /// Requested scenario settings, where a scenario has any beyond the profile.
    #[serde(skip_serializing_if = "Value::is_null")]
    pub settings: Value,
    /// Raw readings of router counters and application metrics, where a scenario takes any.
    #[serde(skip_serializing_if = "Value::is_null")]
    pub readings: Value,
    #[serde(skip)]
    pub samples: Vec<Sample>,
}

impl Results {
    /// Records an assertion, prints it to stderr and returns its verdict.
    pub fn record(
        &mut self,
        name: impl Into<String>,
        verdict: Verdict,
        detail: impl Into<String>,
    ) -> Verdict {
        let name = name.into();
        let detail = detail.into();
        eprintln!("{verdict:<10} {name}: {detail}");
        self.assertions.push(Assertion {
            name,
            verdict,
            detail,
        });
        verdict
    }

    pub fn verdict(&self) -> Verdict {
        let verdicts: Vec<Verdict> = self.assertions.iter().map(|a| a.verdict).collect();
        Verdict::overall(&verdicts)
    }
}

#[derive(Debug, Serialize)]
pub struct Summary<'a> {
    pub run_id: &'a str,
    pub scenario: &'a str,
    pub profile: &'a str,
    pub verdict: Verdict,
    pub git_revision: Option<&'a str>,
    /// The vehicle's adaptation steps, control refusals and pacer when the check ended, where the
    /// vehicle answered (`vehicle_at_end`).
    #[serde(skip_serializing_if = "Value::is_null")]
    pub vehicle_at_end: Value,
    #[serde(flatten)]
    pub results: &'a Results,
}

/// What the vehicle says at the end of a check about why the video and the control did what they
/// did: its latest adaptation steps (with `at_ms` and `unix_ms`), the control envelopes it refused
/// per reason and the pacer. `Null` when it does not answer.
pub fn vehicle_at_end(vehicle: &AgentClient) -> Value {
    let Ok(metrics) = get_json(vehicle, "/metrics") else {
        return Value::Null;
    };
    serde_json::json!({
        "adaptation_steps": metrics.pointer("/adapt/recent_changes"),
        "control_rejected": metrics.pointer("/mavlink/control/rejected"),
        "pace_bytes_per_s": metrics.pointer("/pace_bytes_per_s"),
        "paced_dropped": metrics.pointer("/paced_dropped"),
    })
}

/// Applies the profile through the router agent and records `profile_applied`; the agent's report
/// becomes the effective profile. Returns whether it was applied.
pub fn apply_profile(results: &mut Results, agent: &AgentClient, profile_text: &str) -> bool {
    let name = "profile_applied";
    match agent.post("/profile", profile_text) {
        Ok(body) => {
            results.effective_profile = serde_json::from_str(&body).unwrap_or(Value::String(body));
            results.record(name, Verdict::Pass, "the router agent applied it");
            true
        }
        Err(e) => {
            results.record(name, Verdict::Fail, e.to_string());
            false
        }
    }
}

/// `GET path` on `client` as JSON; any failure as text, for recording in `readings`.
pub fn get_json(client: &AgentClient, path: &str) -> Result<Value, String> {
    let body = client.get(path).map_err(|e| e.to_string())?;
    serde_json::from_str::<Value>(&body).map_err(|e| e.to_string())
}

/// The unsigned number at `pointer` in a reading, if the reading succeeded and holds one.
pub fn counter(reading: &Result<Value, String>, pointer: &str) -> Option<u64> {
    reading.as_ref().ok()?.pointer(pointer)?.as_u64()
}

/// Increase of the counter at `pointer` between two readings of the same source.
pub fn increase(
    start: &Result<Value, String>,
    end: &Result<Value, String>,
    pointer: &str,
) -> Option<u64> {
    counter(end, pointer)?.checked_sub(counter(start, pointer)?)
}

#[expect(
    clippy::cast_precision_loss,
    reason = "counts stay far below 2^53, where f64 is exact"
)]
pub fn float(count: impl Into<u64>) -> f64 {
    count.into() as f64
}

/// Nearest-rank percentile of sorted values.
pub fn percentile(sorted: &[f64], percent: usize) -> Option<f64> {
    let rank = (sorted.len() * percent).div_ceil(100);
    sorted.get(rank.saturating_sub(1)).copied()
}

/// After a forwarding cut, data already queued at the router keeps arriving for about the largest
/// delay seen before it; this margin covers processing on top.
const DRAIN_MARGIN: Duration = Duration::from_millis(500);
const MAX_DRAIN: Duration = Duration::from_secs(10);

/// How long to let queued data drain after a cut, from the largest delay seen in milliseconds.
pub fn drain_for(delay_max_ms: f64) -> Duration {
    Duration::try_from_secs_f64(delay_max_ms / 1000.0)
        .unwrap_or(MAX_DRAIN)
        .saturating_add(DRAIN_MARGIN)
        .min(MAX_DRAIN)
}

/// Writes `summary.json` and `timeseries.csv` into `dir`, creating it.
pub fn write(dir: &Path, summary: &Summary<'_>) -> io::Result<()> {
    fs::create_dir_all(dir)?;
    let json = serde_json::to_string_pretty(summary).map_err(io::Error::other)?;
    fs::write(dir.join("summary.json"), json + "\n")?;
    let mut csv = String::from("unix_s,series,seq,value\n");
    for s in &summary.results.samples {
        // Writing into a String cannot fail.
        let _ = writeln!(csv, "{:.6},{},{},{:.3}", s.unix_s, s.series, s.seq, s.value);
    }
    fs::write(dir.join("timeseries.csv"), csv)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentiles_use_the_nearest_rank() {
        let values: Vec<f64> = (1..=20).map(f64::from).collect();
        let cases = [(50, Some(10.0)), (95, Some(19.0)), (100, Some(20.0))];
        for (percent, expected) in cases {
            assert_eq!(percentile(&values, percent), expected, "p{percent}");
        }
        assert_eq!(percentile(&[], 50), None);
        assert_eq!(percentile(&[7.0], 95), Some(7.0));
    }

    #[test]
    fn missing_evidence_is_never_a_pass() {
        use Verdict::{Fail, Incomplete, Pass};
        let cases: &[(&[Verdict], Verdict)] = &[
            (&[], Incomplete),
            (&[Pass, Pass], Pass),
            (&[Pass, Incomplete], Incomplete),
            (&[Incomplete, Fail, Pass], Fail),
        ];
        for (verdicts, expected) in cases {
            assert_eq!(Verdict::overall(verdicts), *expected, "{verdicts:?}");
        }
    }
}
