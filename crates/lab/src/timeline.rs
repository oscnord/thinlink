use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde::Serialize;

use crate::command::Command;
use crate::interface::Interfaces;
use crate::profile::{Action, Profile, Side};
use crate::tc;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Step {
    pub at_ms: u64,
    pub side: Side,
    pub action: Action,
    pub dev: String,
    pub command: Command,
}

/// One step per interface, in timeline order.
pub fn plan(profile: &Profile, interfaces: &Interfaces) -> Vec<Step> {
    let seed = profile.meta.seed;
    let step = |at_ms, side, action| {
        let dev = interfaces.dev(side);
        Step {
            at_ms,
            side,
            action,
            dev: dev.to_owned(),
            command: tc::timeline_step(dev, profile.link(side), seed, action),
        }
    };
    profile
        .timeline
        .iter()
        .flat_map(|e| e.direction.sides().iter().map(move |&side| (e, side)))
        .map(|(e, side)| step(e.at_ms, side, e.action))
        .collect()
}

/// What happened, with measured offsets from the timeline start.
#[derive(Debug, Clone, Serialize)]
pub struct Transition {
    pub planned_ms: u64,
    pub actual_ms: u64,
    pub direction: &'static str,
    pub action: &'static str,
    pub rate_kbit: Option<u32>,
    pub dev: String,
    pub command: String,
    pub error: Option<String>,
}

fn record(step: &Step, started: Instant, outcome: Result<String, String>) -> Transition {
    let (action, rate_kbit) = match step.action {
        Action::SetRate { rate_kbit } => ("set-rate", Some(rate_kbit)),
        Action::Blackout => ("blackout", None),
        Action::Restore => ("restore", None),
    };
    Transition {
        planned_ms: step.at_ms,
        actual_ms: u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX),
        direction: side_name(step.side),
        action,
        rate_kbit,
        dev: step.dev.clone(),
        command: step.command.display(),
        error: outcome.err(),
    }
}

fn side_name(side: Side) -> &'static str {
    match side {
        Side::Uplink => "uplink",
        Side::Downlink => "downlink",
    }
}

/// A running timeline thread. Dropping `cancel` stops it between steps.
pub struct TimelineRun {
    cancel: Sender<()>,
    log: Arc<Mutex<Vec<Transition>>>,
    handle: JoinHandle<()>,
}

impl TimelineRun {
    pub fn start(steps: Vec<Step>) -> Self {
        let (cancel, cancelled) = mpsc::channel::<()>();
        let log = Arc::new(Mutex::new(Vec::new()));
        let thread_log = Arc::clone(&log);
        let started = Instant::now();
        let handle = thread::spawn(move || {
            for step in &steps {
                let deadline = started + Duration::from_millis(step.at_ms);
                let wait = deadline.saturating_duration_since(Instant::now());
                if !matches!(cancelled.recv_timeout(wait), Err(RecvTimeoutError::Timeout)) {
                    return;
                }
                let outcome = step.command.run().map_err(|e| e.to_string());
                thread_log
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .push(record(step, started, outcome));
            }
        });
        Self {
            cancel,
            log,
            handle,
        }
    }

    pub fn running(&self) -> bool {
        !self.handle.is_finished()
    }

    pub fn transitions(&self) -> Vec<Transition> {
        self.log
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// Waits for the step in flight, if any, so no `tc` runs after this returns.
    pub fn stop(self) {
        drop(self.cancel);
        if self.handle.join().is_err() {
            eprintln!("timeline thread panicked");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_directions_expand_to_one_step_per_interface() {
        let profile = Profile::parse(
            r#"
[profile]
name = "outage"
description = "t"
provenance = "synthetic"
source = "t"

[uplink]
delay_ms = 300
rate_kbit = 512

[downlink]
delay_ms = 300
rate_kbit = 2048

[[timeline]]
at_ms = 10000
action = "blackout"
direction = "both"

[[timeline]]
at_ms = 20000
action = "set-rate"
direction = "downlink"
rate_kbit = 64
"#,
        )
        .unwrap();
        let interfaces = Interfaces::resolve("up0", "down0").unwrap();
        let steps = plan(&profile, &interfaces);
        let got: Vec<(u64, &str, &str)> = steps
            .iter()
            .map(|s| (s.at_ms, s.dev.as_str(), s.command.argv[1].as_str()))
            .collect();
        assert_eq!(
            got,
            [
                (10000, "up0", "qdisc"),
                (10000, "down0", "qdisc"),
                (20000, "down0", "class")
            ]
        );
        assert!(steps[1].command.display().contains("loss random 100%"));
    }

    fn step(at_ms: u64, argv: &[&str]) -> Step {
        Step {
            at_ms,
            side: Side::Uplink,
            action: Action::Restore,
            dev: "up0".to_owned(),
            command: Command::new(argv),
        }
    }

    fn wait_until_finished(run: &TimelineRun) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while run.running() && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(10));
        }
        assert!(!run.running(), "timeline did not finish");
    }

    #[test]
    fn stopping_before_the_first_deadline_runs_nothing() {
        let marker =
            std::env::temp_dir().join(format!("thinlink-timeline-stop-{}", std::process::id()));
        let _ = std::fs::remove_file(&marker);
        let touch = step(300, &["touch", marker.to_str().unwrap()]);
        TimelineRun::start(vec![touch]).stop();
        thread::sleep(Duration::from_millis(600));
        let ran = marker.exists();
        let _ = std::fs::remove_file(&marker);
        assert!(!ran, "step ran after stop");
    }

    #[test]
    fn steps_run_in_order_and_a_failing_step_is_recorded() {
        let run = TimelineRun::start(vec![
            step(0, &["true"]),
            step(20, &["false"]),
            step(40, &["true"]),
        ]);
        wait_until_finished(&run);
        let got: Vec<(u64, bool)> = run
            .transitions()
            .iter()
            .map(|t| (t.planned_ms, t.error.is_some()))
            .collect();
        assert_eq!(got, [(0, false), (20, true), (40, false)]);
        assert!(
            run.transitions()
                .iter()
                .all(|t| t.actual_ms >= t.planned_ms)
        );
        run.stop();
    }
}
