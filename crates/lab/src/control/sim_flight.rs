//! `lab check sim-flight`: the simulated flight (`docs/simulation.md`). `checks` is the ground
//! station: it takes the lease, arms the copter, takes it off, flies a leg on `MANUAL_CONTROL` and
//! then loses the control twice. Without the setpoint stream (the lease and the ground adapter
//! still there) the copter must brake and hold, and refuse to be put back into a mode that flies
//! on the sticks; with the ground adapter killed it must land. All the while the camera's video
//! must reach the ground output, which says how it reads. The times, the sag and the Live share
//! are graded against `perf/budgets.toml`. `checks` only reads what the copter reports: a copter
//! that falls is a failure, not a state to repair.

use std::net::SocketAddr;
use std::thread;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    COMMAND_LONG_DATA, MANUAL_CONTROL_DATA, MavCmd, MavMessage,
};
use serde_json::{Value, json};

use sat_teleop_control::envelope::ControlLimits;
use sat_teleop_media::receiver::Source;
use sat_teleop_media::unix_ms;

use super::sim_evidence::{Commanded, Phase, PhaseRun, record_phases, record_run, video_json};
use super::sim_probe::Probe;
use super::stop::accepted as ack_accepted;
use super::{Parts, force_disarm, ms_between, round_trip, set_mode, start, wait_for_telemetry};
use crate::budget::{self, Budget};
use crate::check::{self, Results, Sample, Verdict, float, percentile};
use crate::client::AgentClient;
use crate::mavlink::command;
use crate::mavlink::gcs::{Gcs, Kind, Seen};
use crate::qgc_status::Healthy;
use crate::skeleton::Capture;

/// `ArduCopter`'s custom modes.
const GUIDED: u32 = 4;
const LOITER: u32 = 5;
const LAND: u32 = 9;
const BRAKE: u32 = 17;
/// What `QGroundControl` sends at 25 Hz, in `MANUAL_CONTROL` units: stick middle (hold altitude) and
/// 40 % pitch forward (about 4 m/s in Loiter).
const SETPOINT_EVERY: Duration = Duration::from_millis(40);
const THROTTLE_MIDDLE: i16 = 500;
const PITCH_FORWARD: i16 = 400;
/// Takeoff altitude (m), and what counts as having got there.
const TAKEOFF_M: f32 = 10.0;
const AIRBORNE_M: f32 = 8.0;
/// The copter's pre-arm checks (EKF, GPS, gyros) take a while under qemu.
const ARM_DEADLINE: Duration = Duration::from_secs(180);
const ARM_EVERY: Duration = Duration::from_secs(4);
const CLIMB_DEADLINE: Duration = Duration::from_secs(60);
/// The leg is flown this long; Loiter must keep its altitude within this band meanwhile (m).
const LEG_FOR: Duration = Duration::from_secs(8);
const HOLD_BAND_M: f32 = 3.0;
/// Flying at least this fast (m/s) counts as having flown the leg.
const MOVING: f32 = 1.0;
/// Lost setpoints: the stream stops, heartbeats and lease stay, and the copter is watched this long.
/// The time to Brake and the sag are `brake_ms` and `brake_sag_m` of the budget.
const BRAKE_WATCH: Duration = Duration::from_secs(10);
/// How long the copter is watched after a mode change it must refuse.
const HELD_WATCH: Duration = Duration::from_secs(6);
/// Lost ground adapter: the copter has this long to be on the ground and disarmed. The time to
/// Land, the sag before it and the time to the ground are `land_ms`, `land_sag_m` and
/// `on_ground_ms` of the budget.
const LAND_WITHIN: Duration = Duration::from_secs(60);
/// Down to this altitude (m) counts as on the ground.
const ON_GROUND_M: f32 = 0.5;
/// Land's fastest descent is `WP_SPD_DN`, 1.5 m/s: `LAND_SPD_HIGH_MS` is 0, which means `WP_SPD_DN`
/// is used (`ArduCopter/mode.cpp`, `land_run_vertical_control`), and neither parameter is set in
/// the pinned `copter.parm`, `gazebo-iris.parm`, `gazebo-iris-gimbal.parm` or
/// `testbed/sat-teleop-copter.parm`, so the 4.7.2 default holds (`AC_WPNav.cpp`,
/// `WP_SPD_DOWN_DEFAULT`); `LAND_SPD_MS`, the final stage, is slower (0.5 m/s, `config.h`). With 25 %
/// to spare, in m/s; a copter that falls once Land has begun exceeds it within a second.
const LAND_DESCENT_MAX_M_S: f64 = 1.5 * 1.25;
/// Descent is the altitude lost over this long, so a position report's jitter is no descent.
const DESCENT_WINDOW_MS: u64 = 1_000;
/// What `just lab-check` waits for on stdout to kill the adapter.
const KILL_REQUEST: &str = "lab: kill ground";
const KILL_DEADLINE: Duration = Duration::from_secs(15);
/// No setpoint applied for this long while `checks` still sends them: the adapter is down.
const STREAM_GONE: Duration = Duration::from_secs(1);
/// For the first frame of video.
const FIRST_FRAME_DEADLINE: Duration = Duration::from_secs(60);
const SAMPLE_EVERY: Duration = Duration::from_millis(100);
/// `GLOBAL_POSITION_INT`, `GPS_RAW_INT` and `VFR_HUD` at 10 Hz while the check runs.
const WATCHED: [f32; 3] = [33.0, 24.0, 74.0];
const WATCH_INTERVAL_US: f32 = 100_000.0;

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    pub gcs_listen: SocketAddr,
    pub observer_listen: SocketAddr,
    /// Where the ground adapter sends `checks` its video output.
    pub output_listen: SocketAddr,
    pub vehicle_metrics: String,
    pub ground_metrics: String,
    pub limits: ControlLimits,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "gcs_listen": self.gcs_listen,
            "observer_listen": self.observer_listen,
            "output_listen": self.output_listen,
            "vehicle_metrics": self.vehicle_metrics,
            "ground_metrics": self.ground_metrics,
            "takeoff_m": TAKEOFF_M,
            "leg_s": LEG_FOR.as_secs_f64(),
            "land_descent_max_m_s": LAND_DESCENT_MAX_M_S,
        })
    }
}

/// What the copter reported at one moment after the control went away.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Point {
    /// Milliseconds after the vehicle's last setpoint.
    ms: u64,
    mode: u32,
    armed: bool,
    alt_m: f32,
}

/// The copter's altitude above home (m) of a position report.
fn altitude(s: &Seen) -> Option<f32> {
    match s.kind {
        Kind::Position { rel_alt_m, .. } if s.from_vehicle => Some(rel_alt_m),
        _ => None,
    }
}

/// The copter's ground speed (m/s) of a GPS report.
fn speed(s: &Seen) -> Option<f32> {
    match s.kind {
        Kind::Gps { speed } if s.from_vehicle => Some(speed),
        _ => None,
    }
}

/// The copter's custom mode and whether it is armed, from its heartbeat.
pub(super) fn state(s: &Seen) -> Option<(u32, bool)> {
    match s.kind {
        Kind::Heartbeat { mode, armed } if s.from_vehicle => Some((mode, armed)),
        _ => None,
    }
}

/// The copter's reports from `after` (UNIX ms) on, one `Point` per position or heartbeat once an
/// altitude is known, with the latest mode and armed state (`start` before any heartbeat came).
fn trace(seen: &[Seen], after: u64, start: (u32, bool)) -> Vec<Point> {
    let (mut mode, mut armed) = start;
    let mut alt = None;
    let mut points = Vec::new();
    for s in seen {
        if let Some(reported) = state(s) {
            (mode, armed) = reported;
        } else if let Some(reported) = altitude(s) {
            alt = Some(reported);
        } else {
            continue;
        }
        if let (Some(alt_m), true) = (alt, s.unix_ms >= after) {
            points.push(Point {
                ms: s.unix_ms - after,
                mode,
                armed,
                alt_m,
            });
        }
    }
    points
}

/// When the copter first reported `Brake` or `Land`, in ms after the last setpoint.
fn brake_ms(points: &[Point]) -> Option<u64> {
    let braking = |p: &&Point| p.mode == BRAKE || p.mode == LAND;
    points.iter().find(braking).map(|p| p.ms)
}

/// When the copter first reported `mode`, in ms after the last setpoint.
fn mode_ms(points: &[Point], mode: u32) -> Option<u64> {
    points.iter().find(|p| p.mode == mode).map(|p| p.ms)
}

/// How far the copter sank below `from_alt` (m) in `points`.
fn sag_m(points: &[Point], from_alt: f32) -> f32 {
    let lowest = points.iter().map(|p| p.alt_m).fold(f32::INFINITY, f32::min);
    (from_alt - lowest).max(0.0)
}

/// The fastest descent (m/s) over any `DESCENT_WINDOW_MS` of `points`.
fn max_descent_m_s(points: &[Point]) -> f64 {
    let over = |from: &Point| {
        let later = points
            .iter()
            .find(|p| p.ms >= from.ms + DESCENT_WINDOW_MS)?;
        let seconds = float(later.ms - from.ms) / 1000.0;
        Some(f64::from(from.alt_m - later.alt_m) / seconds)
    };
    points.iter().filter_map(over).fold(0.0, f64::max)
}

/// The copter's `RC_OVERRIDE_TIME` (s) in `testbed/sat-teleop-copter.parm`: a stream that stops is
/// noticed this long after its last setpoint, so no Brake the stop caused comes sooner.
const RC_OVERRIDE_MS: u64 = 500;

/// Why a Brake after this stop proves nothing: the failsafe under test begins when the check stops
/// the stream, so the copter must still have been flying the leg's mode, with setpoints applied
/// until then (`lifetime_ms` is the vehicle's refresh bound for one). A lapse of the stream on the
/// way to the vehicle brakes the copter early, and that brake is not the one being tested.
fn early_brake(mode_at_stop: u32, unapplied_ms: u64, lifetime_ms: u64) -> Option<String> {
    if mode_at_stop != LOITER {
        return Some(format!(
            "the copter was already in mode {mode_at_stop} when the check stopped the stream"
        ));
    }
    (unapplied_ms > lifetime_ms).then(|| {
        format!(
            "the vehicle had applied no setpoint for {unapplied_ms} ms when the check stopped the stream (refreshed for {lifetime_ms} ms at most)"
        )
    })
}

/// The copter in the air lost its setpoints: it must be braking (or already landing) by the end
/// of the trace. How soon and how far it sank are the budget's. An `early` Brake is incomplete.
fn brake_verdict(points: &[Point], from_alt: f32, early: Option<String>) -> (Verdict, String) {
    if let Some(reason) = early {
        return (Verdict::Incomplete, reason);
    }
    let Some(last) = points.last() else {
        return (Verdict::Incomplete, "no position reports".to_owned());
    };
    let Some(braked) = brake_ms(points) else {
        let detail = format!(
            "no Brake in {} ms after the last setpoint (mode {})",
            last.ms, last.mode
        );
        return (Verdict::Fail, detail);
    };
    if braked < RC_OVERRIDE_MS {
        let detail = format!(
            "Brake {braked} ms after the last setpoint, sooner than the {RC_OVERRIDE_MS} ms override timeout: it began before the stop"
        );
        return (Verdict::Incomplete, detail);
    }
    let detail = format!(
        "Brake {braked} ms after the last setpoint, the copter sank at most {:.1} m over {} ms",
        sag_m(points, from_alt),
        last.ms
    );
    (Verdict::Pass, detail)
}

/// The copter lost the ground adapter: it must be landing, never fall once Land has begun, and be
/// on the ground and disarmed by the end of the trace. How soon and how far it sank are the
/// budget's.
fn land_verdict(points: &[Point]) -> (Verdict, String) {
    let Some(last) = points.last() else {
        return (Verdict::Incomplete, "no position reports".to_owned());
    };
    let Some(landing) = points.iter().find(|p| p.mode == LAND) else {
        let detail = format!("no Land in {} ms after the last setpoint", last.ms);
        return (Verdict::Fail, detail);
    };
    let after: Vec<Point> = points
        .iter()
        .filter(|p| p.ms >= landing.ms)
        .copied()
        .collect();
    let Some(down) = after.iter().find(|p| p.alt_m <= ON_GROUND_M) else {
        let detail = format!(
            "Land after {} ms, but still at {:.1} m after {} ms",
            landing.ms, last.alt_m, last.ms
        );
        return (Verdict::Fail, detail);
    };
    let descent = max_descent_m_s(&after);
    if descent > LAND_DESCENT_MAX_M_S {
        let detail = format!(
            "the copter descended {descent:.1} m/s after Land began, at most {LAND_DESCENT_MAX_M_S} m/s"
        );
        return (Verdict::Fail, detail);
    }
    if last.armed {
        let detail = format!("on the ground after {} ms but still armed", down.ms);
        return (Verdict::Fail, detail);
    }
    let detail = format!(
        "Land {} ms after the last setpoint, on the ground after {} ms, descending at most {descent:.1} m/s, disarmed",
        landing.ms, down.ms
    );
    (Verdict::Pass, detail)
}

/// Asked for a mode that takes the throttle from the sticks while none are sent, the braked
/// copter must refuse and stay in Brake. `lost_m` is the altitude it lost meanwhile.
fn braked_verdict(accepted: bool, mode: Option<u32>, lost_m: f32) -> (Verdict, String) {
    let Some(mode) = mode else {
        return (Verdict::Incomplete, "no heartbeat afterwards".to_owned());
    };
    if accepted || mode != BRAKE {
        let detail = format!(
            "the mode change was accepted: {accepted}, the copter is in mode {mode} and lost {lost_m:.1} m"
        );
        return (Verdict::Fail, detail);
    }
    let detail =
        format!("the mode change was refused and the copter stayed in Brake, losing {lost_m:.1} m");
    (Verdict::Pass, detail)
}

/// The leg flown: the copter must have moved and held its altitude within `HOLD_BAND_M`.
fn leg_verdict(top_speed: f32, deviation: f32, reported: bool) -> (Verdict, String) {
    if !reported {
        let detail = "no position reports during the leg".to_owned();
        return (Verdict::Incomplete, detail);
    }
    if top_speed <= MOVING {
        return (
            Verdict::Fail,
            format!("the copter reached only {top_speed:.1} m/s"),
        );
    }
    if deviation > HOLD_BAND_M {
        let detail = format!("the altitude moved {deviation:.1} m, within {HOLD_BAND_M} m");
        return (Verdict::Fail, detail);
    }
    let detail = format!(
        "{top_speed:.1} m/s with the altitude within {deviation:.1} m (within {HOLD_BAND_M} m)"
    );
    (Verdict::Pass, detail)
}

/// The takeoff: the copter must have reached `AIRBORNE_M`.
fn takeoff_verdict(alt: f32, seconds: f64, accepted: bool) -> (Verdict, String) {
    let detail = format!("{alt:.1} m after {seconds:.0} s (takeoff accepted: {accepted})");
    let verdict = if alt >= AIRBORNE_M {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    (verdict, detail)
}

/// The ground output's video over the leg: it must read Live for at least `min_live` of the
/// frames whose state stamp read back, with new frames that carry plausible stamps.
fn video_verdict(window: &Healthy, min_live: Option<f64>) -> (Verdict, String) {
    let detail = format!(
        "{} of {} frames in {:.1} s read Live; {} new frames with plausible stamps",
        window.live,
        window.states,
        window.seconds,
        window.ages_ms.len()
    );
    if window.states == 0 {
        return (Verdict::Incomplete, detail);
    }
    let Some(min_live) = min_live else {
        return (
            Verdict::Incomplete,
            format!("{detail}; no minimum Live share"),
        );
    };
    let share = count(window.live) / count(window.states);
    let reads = share >= min_live && !window.ages_ms.is_empty();
    let verdict = if reads { Verdict::Pass } else { Verdict::Fail };
    (
        verdict,
        format!("{detail} (at least {min_live} of the frames)"),
    )
}

/// Simulated seconds per wall second, from `GLOBAL_POSITION_INT`'s boot times.
fn real_time_factor(seen: &[Seen]) -> Option<f64> {
    let mut stamps = seen
        .iter()
        .filter(|s| s.from_vehicle)
        .filter_map(|s| match s.kind {
            Kind::Position { boot_ms, .. } => Some((s.at, boot_ms)),
            _ => None,
        });
    let (first_at, first_boot) = stamps.next()?;
    let (last_at, last_boot) = stamps.next_back()?;
    let wall = ms_between(first_at, last_at);
    (wall > 0.0).then(|| float(u64::from(last_boot.wrapping_sub(first_boot))) / wall)
}

fn takeoff(altitude_m: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        param7: altitude_m,
        target_system: 1,
        target_component: 1,
        ..COMMAND_LONG_DATA::default()
    })
}

/// A plain arm, as `QGroundControl` sends it: the copter's pre-arm checks decide.
fn arm() -> MavMessage {
    command(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
}

fn manual(pitch: i16) -> MavMessage {
    MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA {
        target: 1,
        x: pitch,
        z: THROTTLE_MIDDLE,
        ..MANUAL_CONTROL_DATA::default()
    })
}

pub(super) fn count(n: usize) -> f64 {
    float(u64::try_from(n).unwrap_or(u64::MAX))
}

/// Records the metrics that were measured.
pub(super) fn record_metrics<const N: usize>(
    results: &mut Results,
    metrics: [(&'static str, Option<f64>); N],
) {
    let measured = metrics
        .into_iter()
        .filter_map(|(name, value)| Some((name, value?)));
    results.metrics.extend(measured);
}

pub fn run(
    agent: &AgentClient,
    profile_text: &str,
    settings: &Settings,
    budget: Option<&Budget>,
) -> Results {
    let mut results = Results {
        settings: settings.to_json(),
        thresholds: serde_json::to_value(budget).unwrap_or(Value::Null),
        ..Results::default()
    };
    if !check::apply_profile(&mut results, agent, profile_text) {
        return results;
    }
    if let Err(e) = sat_teleop_media::init() {
        results.record("telemetry_received", Verdict::Incomplete, e.to_string());
        return results;
    }
    let mut flight = match Flight::start(settings) {
        Ok(flight) => flight,
        Err(e) => {
            results.record("telemetry_received", Verdict::Incomplete, e.to_string());
            return results;
        }
    };
    if wait_for_telemetry(&mut results, &mut flight.parts)
        && flight.timed(Phase::Takeoff, |f| f.take_off(&mut results))
    {
        flight.timed(Phase::Leg, |f| f.fly_leg(&mut results));
        flight.timed(Phase::Brake, |f| f.lose_setpoints(&mut results));
        flight.timed(Phase::Land, |f| f.lose_ground(&mut results));
    }
    let min_live = budget.and_then(|b| b.min.get("video_live_ratio").copied());
    flight.finish(&mut results, min_live);
    budget::grade(&mut results, budget);
    results
}

/// The ground station, the observer of the copter's telemetry and the receiver of the video.
struct Flight {
    parts: Parts,
    capture: Option<Capture>,
    /// Where the copter was in the air last (m).
    altitude: f32,
    began: Instant,
    /// When the leg's video window opened and closed.
    leg: Option<(Instant, Instant)>,
    /// What the vehicle's control endpoint and adaptation said meanwhile.
    probe: Probe,
    /// The phases flown, with when they began and ended.
    phases: Vec<PhaseRun>,
    /// The modes the check asked for.
    commanded: Vec<Commanded>,
    /// How long the vehicle refreshes a setpoint (`ControlLimits`).
    setpoint_lifetime_ms: u64,
}

impl Flight {
    fn start(settings: &Settings) -> std::io::Result<Self> {
        let parts = start(&super::Settings {
            gcs_listen: settings.gcs_listen,
            observer_listen: settings.observer_listen,
            controller: "10.77.0.4:0".parse().map_err(std::io::Error::other)?,
            controller_key: "/secrets/ctl-checks.key".into(),
            vehicle_id: "testbed".parse().map_err(std::io::Error::other)?,
            vehicle: "10.77.0.2:7700".parse().map_err(std::io::Error::other)?,
            vehicle_metrics: settings.vehicle_metrics.clone(),
            ground_metrics: settings.ground_metrics.clone(),
            limits: settings.limits,
        })?;
        Ok(Self {
            parts,
            capture: Some(Capture::start(Source::Rtp(settings.output_listen))),
            altitude: 0.0,
            began: Instant::now(),
            leg: None,
            probe: Probe::start(&settings.vehicle_metrics, &settings.ground_metrics),
            phases: Vec::new(),
            commanded: Vec::new(),
            setpoint_lifetime_ms: settings.limits.setpoint_lifetime_ms,
        })
    }

    /// Runs `step` and records when it began and ended as `phase`.
    fn timed<T>(&mut self, phase: Phase, step: impl FnOnce(&mut Self) -> T) -> T {
        let (from, from_unix_ms) = (Instant::now(), unix_ms());
        let result = step(self);
        self.phases.push(PhaseRun {
            phase,
            from,
            to: Instant::now(),
            from_unix_ms,
            to_unix_ms: unix_ms(),
        });
        result
    }

    fn station(&mut self) -> &mut Gcs {
        &mut self.parts.station
    }

    /// Holds the setpoint stream (stick middle, `pitch` forward) for `duration`.
    fn stream(&mut self, pitch: i16, duration: Duration) {
        self.station().stream(Some(manual(pitch)), SETPOINT_EVERY);
        self.station().pump(Instant::now() + duration);
    }

    fn command(&mut self, message: &MavMessage) -> bool {
        ack_accepted(&round_trip(self.station(), message))
    }

    /// Asks for `mode` and keeps what was asked, so a change nobody asked for shows.
    fn change_mode(&mut self, mode: u32) -> bool {
        let unix_ms = unix_ms();
        let accepted = self.command(&set_mode(mode));
        self.commanded.push(Commanded {
            unix_ms,
            mode,
            accepted,
        });
        accepted
    }

    /// The copter's latest altitude reported before `to`.
    fn altitude_before(&self, to: Instant) -> Option<f32> {
        let seen = self.parts.observer.before(to);
        seen.iter().rev().find_map(altitude)
    }

    /// The copter's latest altitude reported from `from` on.
    fn altitude_since(&self, from: Instant) -> Option<f32> {
        self.parts
            .observer
            .since(from)
            .iter()
            .rev()
            .find_map(altitude)
    }

    /// Whether the copter has reported from `from` on that it is on the ground and disarmed.
    fn landed_since(&self, from: Instant) -> bool {
        let seen = self.parts.observer.since(from);
        let down = seen
            .iter()
            .rev()
            .find_map(altitude)
            .is_some_and(|alt| alt <= ON_GROUND_M);
        down && seen
            .iter()
            .rev()
            .find_map(state)
            .is_some_and(|(_, armed)| !armed)
    }

    /// The copter's mode and armed state at `from`: its latest heartbeat before then.
    fn state_at(&self, from: Instant) -> (u32, bool) {
        let before = self.parts.observer.before(from);
        before.iter().rev().find_map(state).unwrap_or_default()
    }

    fn video_received(&mut self, results: &mut Results) {
        let started = Instant::now();
        let capture = self.capture.as_ref();
        let arrived = capture.is_some_and(|c| {
            loop {
                if c.read(|all| all.iter().any(|o| o.state.is_some())) {
                    return true;
                }
                if started.elapsed() >= FIRST_FRAME_DEADLINE {
                    return false;
                }
                thread::sleep(SAMPLE_EVERY);
            }
        });
        let name = "video_received";
        if arrived {
            let detail = "frames with a state stamp arrive from the ground output";
            results.record(name, Verdict::Pass, detail);
            return;
        }
        let detail = format!(
            "no frame with a state stamp within {} s",
            FIRST_FRAME_DEADLINE.as_secs()
        );
        results.record(name, Verdict::Fail, detail);
    }

    /// Lease, setpoint stream, Guided, arm (retried while the pre-arm checks refuse) and takeoff.
    /// `false` when it could not be done: the rest is incomplete.
    fn take_off(&mut self, results: &mut Results) -> bool {
        self.video_received(results);
        self.begin_streaming();
        self.armed_in_guided(results) && self.climb(results)
    }

    /// The lease, the telemetry the check reads and a second of stick middle.
    fn begin_streaming(&mut self) {
        let _ = self.parts.ground.post("/lease", "");
        for id in WATCHED {
            let interval = command(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, id, WATCH_INTERVAL_US);
            round_trip(self.station(), &interval);
        }
        self.stream(0, Duration::from_secs(1));
    }

    fn armed_in_guided(&mut self, results: &mut Results) -> bool {
        let guided = self.change_mode(GUIDED);
        let armed = guided && self.arm_when_ready();
        if armed {
            results.record("armed", Verdict::Pass, "armed in Guided under the lease");
            return true;
        }
        let detail = format!(
            "no arm within {} s (Guided accepted: {guided})",
            ARM_DEADLINE.as_secs()
        );
        results.record("armed", Verdict::Incomplete, detail);
        false
    }

    fn arm_when_ready(&mut self) -> bool {
        let deadline = Instant::now() + ARM_DEADLINE;
        while Instant::now() < deadline {
            if self.command(&arm()) {
                return true;
            }
            self.stream(0, ARM_EVERY);
        }
        false
    }

    /// Takes off and waits for `AIRBORNE_M`; whether the copter got there.
    fn climb(&mut self, results: &mut Results) -> bool {
        let from = Instant::now();
        let accepted = self.command(&takeoff(TAKEOFF_M));
        while from.elapsed() < CLIMB_DEADLINE {
            self.stream(0, SAMPLE_EVERY);
            if self
                .altitude_since(from)
                .is_some_and(|alt| alt >= AIRBORNE_M)
            {
                break;
            }
        }
        let alt = self.altitude_since(from).unwrap_or(0.0);
        let seconds = from.elapsed().as_secs_f64();
        record_metrics(results, [("takeoff_s", Some(seconds))]);
        let (verdict, detail) = takeoff_verdict(alt, seconds, accepted);
        results.record("takeoff", verdict, detail);
        self.altitude = alt;
        verdict == Verdict::Pass
    }

    /// Loiter, then pitch forward for `LEG_FOR`: the copter must fly and keep its altitude, and
    /// the video window of the leg is measured at the ground.
    fn fly_leg(&mut self, results: &mut Results) {
        self.change_mode(LOITER);
        self.stream(0, Duration::from_secs(2));
        let from = Instant::now();
        self.stream(PITCH_FORWARD, LEG_FOR);
        self.leg = Some((from, Instant::now()));
        let seen = self.parts.observer.since(from);
        let top = seen.iter().filter_map(speed).fold(0.0, f32::max);
        let alts: Vec<f32> = seen.iter().filter_map(altitude).collect();
        let deviation = alts
            .iter()
            .map(|a| (a - self.altitude).abs())
            .fold(0.0, f32::max);
        record_metrics(
            results,
            [
                ("leg_top_speed_m_s", Some(f64::from(top))),
                ("leg_altitude_deviation_m", Some(f64::from(deviation))),
            ],
        );
        let (verdict, detail) = leg_verdict(top, deviation, !alts.is_empty());
        results.record("leg_flown", verdict, detail);
    }

    /// The stream stops; heartbeats and the lease stay. The copter brakes, and then is asked to
    /// fly on the sticks it no longer has.
    fn lose_setpoints(&mut self, results: &mut Results) {
        let from_alt = self
            .altitude_before(Instant::now())
            .unwrap_or(self.altitude);
        let stopped_unix_ms = unix_ms();
        self.station().stream(None, SETPOINT_EVERY);
        let last = last_setpoint_unix_ms(&self.parts.vehicle);
        let from = Instant::now();
        self.station().pump(from + BRAKE_WATCH);
        let last_arrived = last_setpoint_unix_ms(&self.parts.vehicle);
        let (mode, armed) = self.state_at(from);
        let seen = self.parts.observer.since(from);
        let Some(after) = last else {
            let detail = "no setpoint time from the vehicle";
            results.record("brake_on_lost_setpoints", Verdict::Incomplete, detail);
            return;
        };
        let points = trace(&seen, after, (mode, armed));
        let unapplied_ms = stopped_unix_ms.saturating_sub(last_arrived.unwrap_or(after));
        let early = early_brake(mode, unapplied_ms, self.setpoint_lifetime_ms);
        let (verdict, detail) = brake_verdict(&points, from_alt, early);
        record_metrics(
            results,
            [
                (
                    "setpoints_in_flight_ms",
                    last_arrived.map(|l| float(l.saturating_sub(after))),
                ),
                ("brake_ms", brake_ms(&points).map(float)),
                ("brake_sag_m", Some(f64::from(sag_m(&points, from_alt)))),
            ],
        );
        results.record("brake_on_lost_setpoints", verdict, detail);
        results.samples.extend(altitude_samples(&seen));
        self.refuse_sticks_mode(results, from_alt);
    }

    /// Loiter without sticks: the copter must refuse it and stay braked.
    fn refuse_sticks_mode(&mut self, results: &mut Results, from_alt: f32) {
        let from = Instant::now();
        let accepted = self.change_mode(LOITER);
        self.station().pump(from + HELD_WATCH);
        let seen = self.parts.observer.since(from);
        let mode = seen.iter().rev().find_map(state).map(|(mode, _)| mode);
        let lowest = seen
            .iter()
            .filter_map(altitude)
            .fold(f32::INFINITY, f32::min);
        let lost = (from_alt - lowest).max(0.0);
        let (verdict, detail) = braked_verdict(accepted, mode, lost);
        results.record("no_drop_after_brake", verdict, detail);
    }

    /// The adapter is killed (`just lab-check` does it on request): nothing reaches the copter.
    fn lose_ground(&mut self, results: &mut Results) {
        self.stream(0, SETPOINT_EVERY);
        let asked = Instant::now();
        let name = "land_on_lost_ground";
        let Some(after) = self.await_adapter_death(asked) else {
            let detail = "the adapter kept sending setpoints: it was not killed (run through `just lab-check`)";
            results.record(name, Verdict::Incomplete, detail);
            return;
        };
        let (mode, armed) = self.state_at(asked);
        let from_alt = self.altitude_before(asked).unwrap_or(self.altitude);
        self.await_landed(asked);
        let seen = self.parts.observer.since(asked);
        let points = trace(&seen, after, (mode, armed));
        let (verdict, detail) = land_verdict(&points);
        let landing = mode_ms(&points, LAND);
        let before: Vec<Point> = points
            .iter()
            .take_while(|p| Some(p.ms) < landing)
            .copied()
            .collect();
        let on_ground = points
            .iter()
            .find(|p| landing.is_some_and(|l| p.ms >= l) && p.alt_m <= ON_GROUND_M);
        record_metrics(
            results,
            [
                ("land_ms", landing.map(float)),
                (
                    "land_sag_m",
                    landing.map(|_| f64::from(sag_m(&before, from_alt))),
                ),
                ("on_ground_ms", on_ground.map(|p| float(p.ms))),
            ],
        );
        results.record(name, verdict, detail);
        results.samples.extend(altitude_samples(&seen));
    }

    /// Asks for the adapter to be killed and waits until the vehicle sees no more setpoints; the
    /// time of the last one (UNIX ms), or `None` when they never stopped.
    fn await_adapter_death(&mut self, asked: Instant) -> Option<u64> {
        println!("{KILL_REQUEST}");
        let mut last = last_setpoint_unix_ms(&self.parts.vehicle);
        let mut changed = Instant::now();
        while Instant::now() < asked + KILL_DEADLINE {
            self.station().pump(Instant::now() + SAMPLE_EVERY);
            let latest = last_setpoint_unix_ms(&self.parts.vehicle);
            if latest != last {
                (last, changed) = (latest, Instant::now());
            } else if changed.elapsed() >= STREAM_GONE {
                return last;
            }
        }
        None
    }

    /// Watches until the copter is on the ground and disarmed, or `LAND_WITHIN` has passed.
    fn await_landed(&mut self, asked: Instant) {
        let until = Instant::now() + LAND_WITHIN;
        self.station().stream(None, SETPOINT_EVERY);
        while Instant::now() < until {
            thread::sleep(SAMPLE_EVERY);
            if self.landed_since(asked) {
                return;
            }
        }
    }

    /// The video's label and age over the leg, and the simulation's pace; then leaves SITL
    /// disarmed with the lease and the stream settings back as they were.
    fn finish(mut self, results: &mut Results, min_live: Option<f64>) {
        if let (Some(capture), Some((from, to))) = (self.capture.take(), self.leg) {
            let window = capture.read(|all| Healthy::measure(all, from, to));
            record_video(results, &window, min_live);
            results.readings = capture.read(|all| video_json(all, from, to));
            capture.finish();
        }
        let seen = self.parts.observer.since(self.began);
        record_metrics(results, [("sim_real_time_factor", real_time_factor(&seen))]);
        let readings = self.probe.readings();
        let phases = record_phases(results, &readings, &self.phases);
        record_run(
            results,
            &readings,
            &phases,
            self.began,
            &self.commanded,
            &seen,
        );
        super::stop::clean_up(&mut self.parts);
        round_trip(&mut self.parts.station, &force_disarm());
    }
}

/// The video as the ground output labelled it during the leg.
fn record_video(results: &mut Results, window: &Healthy, min_live: Option<f64>) {
    record_metrics(
        results,
        [
            (
                "video_live_ratio",
                (window.states > 0).then(|| count(window.live) / count(window.states)),
            ),
            (
                "video_new_frames_per_s",
                (window.seconds > 0.0).then(|| count(window.ages_ms.len()) / window.seconds),
            ),
            ("video_age_p50_ms", percentile(&window.ages_ms, 50)),
            ("video_age_p95_ms", percentile(&window.ages_ms, 95)),
        ],
    );
    let (verdict, detail) = video_verdict(window, min_live);
    results.record("video_live", verdict, detail);
}

/// `altitude` per position report.
fn altitude_samples(seen: &[Seen]) -> impl Iterator<Item = Sample> + '_ {
    seen.iter()
        .filter_map(|s| Some((s.unix_ms, altitude(s)?)))
        .zip(0_u32..)
        .map(|((unix_ms, alt), seq)| Sample {
            unix_s: float(unix_ms) / 1000.0,
            series: "altitude",
            seq,
            value: f64::from(alt),
        })
}

/// When the vehicle last wrote a setpoint from an envelope (UNIX ms).
fn last_setpoint_unix_ms(vehicle: &AgentClient) -> Option<u64> {
    let metrics = check::get_json(vehicle, "/metrics").ok()?;
    metrics
        .pointer("/mavlink/control/last_continuous/applied")?
        .as_u64()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(ms: u64, mode: u32, alt_m: f32) -> Point {
        Point {
            ms,
            mode,
            armed: true,
            alt_m,
        }
    }

    #[test]
    fn a_copter_that_loses_its_setpoints_must_brake() {
        let held = |mode_at: u64| -> Vec<Point> {
            (0..=10)
                .map(|i| {
                    let ms = i * 1_000;
                    point(ms, if ms >= mode_at { BRAKE } else { LOITER }, 10.0)
                })
                .collect()
        };
        let cases = [
            (held(1_000), Verdict::Pass),
            (
                vec![
                    point(100, LOITER, 10.0),
                    point(RC_OVERRIDE_MS - 1, BRAKE, 10.0),
                ],
                Verdict::Incomplete,
            ),
            (held(99_000), Verdict::Fail),
            (Vec::new(), Verdict::Incomplete),
        ];
        for (points, expected) in cases {
            assert_eq!(brake_verdict(&points, 10.0, None).0, expected, "{points:?}");
        }
    }

    #[test]
    fn a_copter_that_loses_the_ground_must_land_and_disarm_without_falling() {
        // Land at 3 s, then `descent` m/s until it is down, disarmed or not at the end.
        let flight = |descent: f32, down: bool, armed_at_end: bool| {
            let mut points = Vec::new();
            for i in 0..=30_u16 {
                let ms = u64::from(i) * 1_000;
                let landing = ms >= 3_000;
                let fallen = f32::from(i.saturating_sub(3)) * descent;
                let alt = match (landing, down) {
                    (false, _) => 10.0,
                    (true, true) => (10.0 - fallen).max(0.1),
                    (true, false) => (10.0 - fallen).max(5.0),
                };
                let mut p = point(ms, if landing { LAND } else { BRAKE }, alt);
                p.armed = ms < 30_000 || armed_at_end;
                points.push(p);
            }
            points
        };
        let cases = [
            (flight(0.4, true, false), Verdict::Pass),
            (flight(0.4, false, false), Verdict::Fail),
            (flight(0.4, true, true), Verdict::Fail),
            (flight(5.0, true, false), Verdict::Fail),
            (Vec::new(), Verdict::Incomplete),
        ];
        for (points, expected) in cases {
            assert_eq!(land_verdict(&points).0, expected, "{points:?}");
        }
    }

    #[test]
    fn the_video_must_read_live_for_the_budgets_share_of_its_frames() {
        let window = |live, states| Healthy {
            live,
            states,
            ages_ms: vec![10.0],
            ..Healthy::default()
        };
        let cases = [
            (window(100, 100), Some(0.9), Verdict::Pass),
            (window(1, 100), Some(0.9), Verdict::Fail),
            (window(100, 100), None, Verdict::Incomplete),
            (window(0, 0), Some(0.9), Verdict::Incomplete),
        ];
        for (window, min, expected) in cases {
            assert_eq!(
                video_verdict(&window, min).0,
                expected,
                "{window:?} {min:?}"
            );
        }
    }
}
