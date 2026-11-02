//! Acceptance budgets (`perf/budgets.toml`): per scenario and profile, limits on named metrics.
//! A budget in calibration is measured and recorded but never graded; a run against it is
//! incomplete, never a pass.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::check::{Results, Verdict};

#[derive(Debug, thiserror::Error)]
pub enum BudgetError {
    #[error("budgets do not parse: {0}")]
    Toml(#[from] toml::de::Error),
    #[error("{scenario}.{profile}: {reason}")]
    Invalid {
        scenario: String,
        profile: String,
        reason: &'static str,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Status {
    /// Measured and recorded only; no thresholds yet.
    Calibration,
    /// Every limit is graded.
    Enforced,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Budget {
    pub status: Status,
    /// Where the limits come from, citing the run ids they were derived from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rationale: Option<String>,
    /// Metric name to the largest passing value.
    #[serde(default)]
    pub max: BTreeMap<String, f64>,
    /// Metric name to the smallest passing value.
    #[serde(default)]
    pub min: BTreeMap<String, f64>,
}

/// Scenario name to profile name to budget.
#[derive(Debug, Default)]
pub struct Budgets(BTreeMap<String, BTreeMap<String, Budget>>);

impl Budgets {
    pub fn parse(text: &str) -> Result<Self, BudgetError> {
        let table: BTreeMap<String, BTreeMap<String, Budget>> = toml::from_str(text)?;
        for (scenario, profiles) in &table {
            for (profile, budget) in profiles {
                budget.validate().map_err(|reason| BudgetError::Invalid {
                    scenario: scenario.clone(),
                    profile: profile.clone(),
                    reason,
                })?;
            }
        }
        Ok(Self(table))
    }

    pub fn get(&self, scenario: &str, profile: &str) -> Option<&Budget> {
        self.0.get(scenario)?.get(profile)
    }
}

impl Budget {
    fn validate(&self) -> Result<(), &'static str> {
        let limits = self.max.values().chain(self.min.values());
        if limits.clone().any(|v| !v.is_finite()) {
            return Err("limits must be finite");
        }
        let has_limits = limits.count() > 0;
        let has_rationale = self
            .rationale
            .as_deref()
            .is_some_and(|r| !r.trim().is_empty());
        match self.status {
            Status::Calibration if has_limits => Err("a calibration budget has no limits"),
            Status::Enforced if !has_limits => Err("an enforced budget needs at least one limit"),
            Status::Enforced if !has_rationale => Err("an enforced budget needs a rationale"),
            _ => Ok(()),
        }
    }
}

/// Grades `results.metrics` against `budget` as assertions. Without a budget, or in calibration,
/// one `budget` assertion records the run as incomplete; a limit on a metric that was not measured
/// is incomplete too.
pub fn grade(results: &mut Results, budget: Option<&Budget>) {
    let Some(budget) = budget else {
        results.record("budget", Verdict::Incomplete, "no budget for this profile");
        return;
    };
    if budget.status == Status::Calibration {
        let detail = "calibration: measured and recorded, not graded";
        results.record("budget", Verdict::Incomplete, detail);
        return;
    }
    let limits = budget
        .max
        .iter()
        .map(|(metric, limit)| (metric, *limit, Bound::Max))
        .chain(budget.min.iter().map(|(m, l)| (m, *l, Bound::Min)));
    for (metric, limit, bound) in limits {
        let measured = results.metrics.get(metric.as_str()).copied();
        let (verdict, detail) = judge(measured, limit, bound);
        results.record(format!("budget.{metric}"), verdict, detail);
    }
}

#[derive(Debug, Clone, Copy)]
enum Bound {
    Max,
    Min,
}

fn judge(measured: Option<f64>, limit: f64, bound: Bound) -> (Verdict, String) {
    let Some(value) = measured else {
        return (Verdict::Incomplete, "not measured".to_owned());
    };
    let (holds, relation) = match bound {
        Bound::Max => (value <= limit, "max"),
        Bound::Min => (value >= limit, "min"),
    };
    let verdict = if holds { Verdict::Pass } else { Verdict::Fail };
    (verdict, format!("measured {value:.3}, {relation} {limit}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    const ENFORCED: &str = r#"
        [skeleton.lan]
        status = "enforced"
        rationale = "from runs a and b"
        max = { frame_age_p95_ms = 200.0 }
        min = { new_frames_per_s = 14.0 }
    "#;

    type Metrics<'a> = &'a [(&'static str, f64)];
    type Expected<'a> = &'a [(&'static str, Verdict)];

    fn graded(text: &str, metrics: Metrics<'_>) -> Vec<(String, Verdict)> {
        let budgets = Budgets::parse(text).unwrap();
        let mut results = Results::default();
        results.metrics.extend(metrics.iter().copied());
        grade(&mut results, budgets.get("skeleton", "lan"));
        results
            .assertions
            .into_iter()
            .map(|a| (a.name, a.verdict))
            .collect()
    }

    #[test]
    fn only_an_enforced_budget_with_measurements_can_pass() {
        use Verdict::{Fail, Incomplete, Pass};
        let calibration = "[skeleton.lan]\nstatus = \"calibration\"\n";
        let cases: &[(&str, Metrics<'_>, Expected<'_>)] = &[
            ("", &[("frame_age_p95_ms", 1.0)], &[("budget", Incomplete)]),
            (
                calibration,
                &[("frame_age_p95_ms", 1.0)],
                &[("budget", Incomplete)],
            ),
            (
                ENFORCED,
                &[("frame_age_p95_ms", 150.0), ("new_frames_per_s", 15.0)],
                &[
                    ("budget.frame_age_p95_ms", Pass),
                    ("budget.new_frames_per_s", Pass),
                ],
            ),
            (
                ENFORCED,
                &[("frame_age_p95_ms", 250.0), ("new_frames_per_s", 15.0)],
                &[
                    ("budget.frame_age_p95_ms", Fail),
                    ("budget.new_frames_per_s", Pass),
                ],
            ),
            (
                ENFORCED,
                &[("frame_age_p95_ms", 150.0)],
                &[
                    ("budget.frame_age_p95_ms", Pass),
                    ("budget.new_frames_per_s", Incomplete),
                ],
            ),
        ];
        for (text, metrics, expected) in cases {
            let expected: Vec<(String, Verdict)> = expected
                .iter()
                .map(|(n, v)| ((*n).to_owned(), *v))
                .collect();
            assert_eq!(graded(text, metrics), expected, "{text} {metrics:?}");
        }
    }

    #[test]
    fn rejects_budgets_that_could_hide_a_missing_threshold() {
        let cases = [
            "[skeleton.lan]\nstatus = \"enforced\"\nrationale = \"r\"\n",
            "[skeleton.lan]\nstatus = \"enforced\"\nmax = { a = 1.0 }\n",
            "[skeleton.lan]\nstatus = \"calibration\"\nmax = { a = 1.0 }\n",
            "[skeleton.lan]\nstatus = \"enforced\"\nrationale = \"r\"\nmax = { a = nan }\n",
            "[skeleton.lan]\nstatus = \"graded\"\n",
            "[skeleton.lan]\nstatus = \"calibration\"\nlimit = 3\n",
        ];
        for text in cases {
            assert!(Budgets::parse(text).is_err(), "{text}");
        }
    }

    #[test]
    fn shipped_budgets_parse() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../perf/budgets.toml");
        Budgets::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    }
}
