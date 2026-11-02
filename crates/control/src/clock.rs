//! A machine's clock bound at runtime. `Chrony` runs `chronyc -c tracking` every few seconds and
//! derives the bound from it (`sat_teleop_policy::clock::from_tracking`); `ChronyAt` asks the
//! chronyd of another host, which a container needs for its host's; `Shared` is the testbed,
//! whose containers read one kernel clock (bound 0); `Fixed` is a number the operator measured, and
//! holds only until the wall clock steps against the monotonic clock by more than that number (a
//! time service setting the clock, a virtual machine resuming from sleep): `poll` compares the two
//! clocks, and after a step the bound is gone until the process restarts. Test hooks can inject a
//! bound, or its absence, over any source.

use std::process::{Command, Stdio};
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use sat_teleop_policy::clock::{self, Reading};

use crate::unix_ms;

/// How often chrony is asked.
const POLL: Duration = Duration::from_secs(2);
/// `chronyc` waits this long for `chronyd` and does not retry.
const CHRONYC_COMMANDS: [&str; 3] = ["timeout 1000", "retries 0", "tracking"];

#[derive(Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ClockSource {
    /// The `chronyd` on this machine.
    Chrony,
    /// The `chronyd` at this host, over its command port (UDP 323), which answers `tracking` to the
    /// addresses its `cmdallow` names without any privilege. A container reaches its host's
    /// `chronyd` this way (`docs/real-vehicle.md`, "Clocks").
    ChronyAt(String),
    /// One clock for every machine: the testbed.
    Shared,
    /// A bound in milliseconds, measured by hand; dropped after a step of the wall clock.
    Fixed(u64),
}

#[derive(Debug)]
pub struct Clock {
    source: ClockSource,
    /// The newest `chronyc` answer and when it came; `None` inside until chrony reports
    /// synchronised.
    chrony: Mutex<Option<(Option<u64>, Instant)>>,
    injected: Mutex<Injection>,
    /// A fresh bound has been seen since the process started (`started`).
    started: AtomicBool,
    /// The last state `poll` reported, for logging changes only.
    fresh: AtomicBool,
    /// The wall clock (UNIX milliseconds) and the monotonic clock at the last comparison.
    compared: Mutex<(u64, Instant)>,
    /// A `Fixed` bound's wall clock stepped by more than the bound: it holds no longer.
    stepped: AtomicBool,
}

/// What a test makes of the clock's bound.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Injection {
    /// The source decides.
    #[default]
    Off,
    /// The clock has no bound.
    Missing,
    /// The clock has this bound, in milliseconds.
    Bound(u64),
}

#[derive(Debug, Serialize)]
pub struct ClockSnapshot {
    pub source: ClockSource,
    /// The bound now, `None` when there is none to trust.
    pub bound_ms: Option<u64>,
    /// Whether a fresh bound was seen since the process started; real mode needs one to start.
    pub started: bool,
    pub injected: bool,
}

impl Clock {
    pub fn new(source: ClockSource) -> Self {
        Self {
            source,
            chrony: Mutex::new(None),
            injected: Mutex::new(Injection::Off),
            started: AtomicBool::new(false),
            fresh: AtomicBool::new(false),
            compared: Mutex::new((unix_ms(), Instant::now())),
            stepped: AtomicBool::new(false),
        }
    }

    pub fn source(&self) -> &ClockSource {
        &self.source
    }

    /// The bound as it stands, with its age.
    pub fn reading(&self) -> Option<Reading> {
        match self.injected.lock().map(|i| *i).unwrap_or_default() {
            Injection::Missing => return None,
            Injection::Bound(bound_ms) => {
                return Some(Reading {
                    bound_ms,
                    age_ms: 0,
                });
            }
            Injection::Off => {}
        }
        match &self.source {
            ClockSource::Shared => Some(Reading {
                bound_ms: 0,
                age_ms: 0,
            }),
            ClockSource::Fixed(bound_ms) => {
                let holds = !self.stepped.load(Ordering::Relaxed);
                holds.then_some(Reading {
                    bound_ms: *bound_ms,
                    age_ms: 0,
                })
            }
            ClockSource::Chrony | ClockSource::ChronyAt(_) => {
                let (bound, at) = (*self.chrony.lock().ok()?)?;
                let age_ms = u64::try_from(at.elapsed().as_millis()).unwrap_or(u64::MAX);
                bound.map(|bound_ms| Reading { bound_ms, age_ms })
            }
        }
    }

    /// The bound to trust now: the reading, grown by its age; `None` when stale or missing.
    pub fn bound_ms(&self) -> Option<u64> {
        self.reading().and_then(clock::aged)
    }

    /// Whether the clock has had a fresh bound since the process started. The first evaluation that
    /// finds one latches it: real mode needs it before it first sends control, and a bound that
    /// goes stale afterwards only turns the video label to `AGE UNKNOWN`.
    pub fn started(&self) -> bool {
        if self.bound_ms().is_some() {
            self.started.store(true, Ordering::Relaxed);
        }
        self.started.load(Ordering::Relaxed)
    }

    /// Test only: replaces the bound.
    pub fn inject(&self, bound: Injection) {
        if let Ok(mut injected) = self.injected.lock() {
            *injected = bound;
        }
    }

    pub fn snapshot(&self) -> ClockSnapshot {
        ClockSnapshot {
            source: self.source.clone(),
            bound_ms: self.bound_ms(),
            started: self.started(),
            injected: self.injected.lock().is_ok_and(|i| *i != Injection::Off),
        }
    }

    /// Asks chrony, or compares a fixed bound's clocks, every `POLL` until the process ends;
    /// returns at once for any other source.
    pub fn poll(&self) {
        match &self.source {
            ClockSource::Chrony => self.poll_chrony(None),
            ClockSource::ChronyAt(host) => self.poll_chrony(Some(host)),
            ClockSource::Fixed(bound_ms) => loop {
                thread::sleep(POLL);
                self.compare(*bound_ms, unix_ms(), Instant::now());
            },
            ClockSource::Shared => {}
        }
    }

    fn poll_chrony(&self, host: Option<&str>) {
        loop {
            let bound = tracking(host).and_then(|line| clock::from_tracking(&line));
            if let Ok(mut chrony) = self.chrony.lock() {
                *chrony = Some((bound, Instant::now()));
            }
            self.report();
            thread::sleep(POLL);
        }
    }
}

impl Clock {
    /// Checks that the wall clock advanced like the monotonic clock since the last comparison
    /// (`sat_teleop_policy::clock::fixed_holds`); a step beyond `bound_ms` ends the bound for good.
    fn compare(&self, bound_ms: u64, wall_ms: u64, now: Instant) {
        let Ok(mut last) = self.compared.lock() else {
            return;
        };
        let (wall_then, mono_then) = std::mem::replace(&mut *last, (wall_ms, now));
        drop(last);
        let wall = wall_ms.saturating_sub(wall_then);
        let mono = u64::try_from(now.duration_since(mono_then).as_millis()).unwrap_or(u64::MAX);
        if clock::fixed_holds(bound_ms, wall, mono) || self.stepped.swap(true, Ordering::Relaxed) {
            return;
        }
        eprintln!(
            "the wall clock stepped by {} ms against the monotonic clock: the fixed bound of {bound_ms} ms no longer holds, so the video label reads AGE UNKNOWN until the process restarts",
            wall.abs_diff(mono)
        );
    }

    /// Logs when the bound turns fresh or stale. A stale one in flight changes nothing but the
    /// video label.
    fn report(&self) {
        let bound = self.bound_ms();
        if self.fresh.swap(bound.is_some(), Ordering::Relaxed) == bound.is_some() {
            return;
        }
        match bound {
            Some(ms) => eprintln!("clock bound fresh again: {ms} ms"),
            None => eprintln!(
                "clock bound stale: chrony is not synchronised or does not answer; the video label reads AGE UNKNOWN until it does"
            ),
        }
    }
}

/// The one line `chronyc -c tracking` prints; `None` if chrony is not installed or not running. The
/// timeout is a `chronyc` command: chrony 4.6 has no `-t` option.
fn tracking(host: Option<&str>) -> Option<String> {
    let output = Command::new("chronyc")
        .arg("-c")
        .args(host.map(|host| ["-h", host]).iter().flatten())
        .arg("-m")
        .args(CHRONYC_COMMANDS)
        .stdin(Stdio::null())
        .stderr(Stdio::null())
        .output()
        .ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fixed bound stops holding once the wall clock steps by more than it, and stays gone.
    #[test]
    fn a_fixed_bound_ends_with_a_step_of_the_wall_clock() {
        let clock = Clock::new(ClockSource::Fixed(50));
        let (wall, mono) = *clock.compared.lock().unwrap();
        let after = |s: u64| mono + Duration::from_secs(s);
        clock.compare(50, wall + 2_000, after(2));
        assert_eq!(clock.bound_ms(), Some(50), "a steady clock");
        clock.compare(50, wall + 4_000 + 3_600_000, after(4));
        assert_eq!(clock.bound_ms(), None, "the wall clock jumped an hour");
        clock.compare(50, wall + 6_000 + 3_600_000, after(6));
        assert_eq!(clock.bound_ms(), None, "steady again changes nothing");
    }

    /// Real mode needs a fresh bound to start; one that goes stale afterwards leaves `started` set.
    #[test]
    fn a_bound_that_goes_stale_stays_started() {
        let clock = Clock::new(ClockSource::Chrony);
        assert!(!clock.started(), "no bound has been seen");
        clock.inject(Injection::Bound(20));
        assert!(clock.started());
        clock.inject(Injection::Missing);
        assert_eq!(clock.bound_ms(), None);
        assert!(clock.started(), "the start is not undone");
    }
}
