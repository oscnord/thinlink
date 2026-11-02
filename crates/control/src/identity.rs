//! Where control goes: to the autopilot, or to the logging sink. In SITL mode (the default) the
//! vehicle sends control only to an autopilot that has said it is SITL. In real mode it sends
//! control only to an autopilot that is not SITL (unless the test-only allowance is set) once every
//! interlock holds: its public keys, the failsafe parameters read back from the autopilot and the
//! clock bound. Whatever is in doubt, control goes to the sink, and `failures` says why. The clock
//! bound only gates the start: once it has been fresh, a stale one no longer withdraws control or
//! liveness, since it feeds a display label and a failsafe must not hang on it; the keys and the
//! parameters keep gating in flight.
//!
//! The marker is a parameter only SITL builds have. `ArduPilot` 4.7.2 (`fb773567896014da007bca58358704d4a0d8516f`)
//! registers the `SIM_` parameter group in `Rover/Parameters.cpp` and `ArduCopter/Parameters.cpp`
//! under `#if AP_SIM_ENABLED`, and `Tools/ardupilotwaf/boards.py` defines `AP_SIM_ENABLED` as 1 for
//! the SITL board and as 0 for every other board except a `ChibiOS` build made with the simulation
//! linked in. `SIM_WIND_SPD` is in that group (`libraries/SITL/SITL.cpp`). A normal flight
//! controller build answers `PARAM_REQUEST_READ` of a parameter it lacks with nothing.

use std::time::Duration;

use mavlink::MavHeader;
use mavlink::dialects::ardupilotmega::{MavMessage, PARAM_REQUEST_READ_DATA};

use serde::{Deserialize, Serialize};

use crate::frame::{Frame, datagram_frames};
use crate::gate::Vehicle;

/// The parameter whose presence says the autopilot is SITL.
pub const SITL_MARKER: &str = "SIM_WIND_SPD";
/// How often an unidentified autopilot is asked for the marker.
pub const PROBE_EVERY: Duration = Duration::from_secs(1);
/// In real mode an autopilot is taken not to be SITL once it ignored this many probes.
pub const REAL_PROBES: u8 = 3;
/// The bridge's own identity on the autopilot's link: not the ground station's system id (255), so
/// the probe never counts as the ground station's heartbeat for `ArduPilot`'s GCS failsafe.
const PROBE_SYSTEM: u8 = 254;
/// `MAV_COMP_ID_ONBOARD_COMPUTER`.
const PROBE_COMPONENT: u8 = 191;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlTarget {
    /// Control frames are written to the autopilot.
    Autopilot,
    /// Control frames are logged and counted, never written.
    LoggingSink,
}

/// What is known of the autopilot on the link: the system and component of its heartbeat, the
/// vehicle type that heartbeat reports and whether that same system and component has reported
/// the SITL marker. The marker belongs to the autopilot that answered it: a heartbeat from another
/// address or of another type starts over.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[must_use]
pub struct Autopilot {
    pub address: Option<(u8, u8)>,
    pub vehicle: Option<Vehicle>,
    pub sitl_marker: bool,
    /// Marker probes sent to this autopilot so far.
    pub probes: u8,
}

impl Autopilot {
    /// The autopilot's heartbeat from `address`.
    pub fn heartbeat(self, address: (u8, u8), vehicle: Option<Vehicle>) -> Self {
        let same = self.address == Some(address) && self.vehicle == vehicle;
        Self {
            address: Some(address),
            vehicle,
            sitl_marker: self.sitl_marker && same,
            probes: if same { self.probes } else { 0 },
        }
    }

    /// A marker probe was sent.
    pub fn probed(self) -> Self {
        Self {
            probes: self.probes.saturating_add(1),
            ..self
        }
    }

    /// A `PARAM_VALUE` from `address`; only the autopilot's own address counts.
    pub fn param_value(self, address: (u8, u8), message: &MavMessage) -> Self {
        let from_autopilot = self.address == Some(address);
        Self {
            sitl_marker: self.sitl_marker || (from_autopilot && is_sitl_marker(message)),
            ..self
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    /// The autopilot must be SITL.
    #[default]
    Sitl,
    /// The autopilot must be a flight controller and every interlock must hold.
    Real,
}

/// The vehicle's mode and, for tests only, whether real mode accepts SITL.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Policy {
    pub mode: Mode,
    pub allow_sitl: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Interlock {
    Identity,
    ControllerKeys,
    FailsafeParams,
    ClockBound,
}

/// An interlock that does not hold, and why.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Failure {
    pub interlock: Interlock,
    pub reason: String,
}

/// What the decision needs to know. A reason is `Some` while its interlock does not hold.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Facts {
    pub sitl_marker: bool,
    pub probes: u8,
    pub vehicle: Option<Vehicle>,
    pub keys: Option<String>,
    pub params: Option<String>,
    /// Why there is no fresh clock bound now.
    pub clock: Option<String>,
    /// The clock has had a fresh bound since the process started.
    pub clock_started: bool,
}

/// The interlocks that do not hold; control goes to the autopilot when there are none. The only
/// place that decides.
pub fn failures(policy: Policy, facts: &Facts) -> Vec<Failure> {
    let real = policy.mode == Mode::Real;
    let candidates = [
        (Interlock::Identity, identity_failure(policy, facts)),
        (
            Interlock::ControllerKeys,
            facts.keys.clone().filter(|_| real),
        ),
        (
            Interlock::FailsafeParams,
            facts.params.clone().filter(|_| real),
        ),
        (
            Interlock::ClockBound,
            facts.clock.clone().filter(|_| real && !facts.clock_started),
        ),
    ];
    candidates
        .into_iter()
        .filter_map(|(interlock, reason)| {
            Some(Failure {
                interlock,
                reason: reason?,
            })
        })
        .collect()
}

fn identity_failure(policy: Policy, facts: &Facts) -> Option<String> {
    if facts.vehicle.is_none() {
        return Some(
            "the autopilot has sent no heartbeat of a vehicle type the gate knows".to_owned(),
        );
    }
    let reason = match (policy.mode, facts.sitl_marker) {
        (Mode::Sitl, false) => "the autopilot has not reported SIM_WIND_SPD: it is not SITL",
        (Mode::Real, true) if !policy.allow_sitl => {
            "the autopilot is SITL (it reports SIM_WIND_SPD), which real mode refuses"
        }
        (Mode::Real, false) if facts.probes < REAL_PROBES && !policy.allow_sitl => {
            "the autopilot has not yet ignored the SITL probes"
        }
        (Mode::Sitl, true) | (Mode::Real, _) => return None,
    };
    Some(reason.to_owned())
}

/// Where control goes: to the autopilot only when no interlock fails.
pub fn control_target(policy: Policy, facts: &Facts) -> ControlTarget {
    if failures(policy, facts).is_empty() {
        ControlTarget::Autopilot
    } else {
        ControlTarget::LoggingSink
    }
}

/// Whether `message` is the autopilot reporting the SITL marker parameter.
pub fn is_sitl_marker(message: &MavMessage) -> bool {
    let MavMessage::PARAM_VALUE(p) = message else {
        return false;
    };
    p.param_id.to_str() == Ok(SITL_MARKER)
}

/// A `PARAM_REQUEST_READ` of the marker for the autopilot at `system` and `component`.
pub fn probe(system: u8, component: u8) -> Option<Frame> {
    read_request(system, component, SITL_MARKER)
}

/// A `PARAM_REQUEST_READ` of the parameter `name` for the autopilot at `system` and `component`.
pub fn read_request(system: u8, component: u8, name: &str) -> Option<Frame> {
    let request = MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
        param_index: -1,
        target_system: system,
        target_component: component,
        param_id: name.into(),
    });
    let header = MavHeader {
        system_id: PROBE_SYSTEM.into(),
        component_id: PROBE_COMPONENT,
        sequence: 0,
    };
    let mut bytes = Vec::new();
    mavlink::write_v2_msg(&mut bytes, header, &request).ok()?;
    datagram_frames(&bytes).next()
}

#[cfg(test)]
mod tests {
    use mavlink::dialects::ardupilotmega::PARAM_VALUE_DATA;

    use super::*;

    #[test]
    fn control_goes_to_the_autopilot_only_when_every_interlock_holds() {
        use ControlTarget::{Autopilot as Goes, LoggingSink as Sink};
        let sitl = Policy::default();
        let real = Policy {
            mode: Mode::Real,
            allow_sitl: false,
        };
        let real_on_sitl = Policy {
            allow_sitl: true,
            ..real
        };
        let facts =
            |sitl_marker, probes, vehicle, keys: Option<&str>, params: Option<&str>| Facts {
                sitl_marker,
                probes,
                vehicle,
                keys: keys.map(str::to_owned),
                params: params.map(str::to_owned),
                clock: None,
                clock_started: false,
            };
        let no_clock = |facts: Facts| Facts {
            clock: Some("no fresh clock bound".to_owned()),
            ..facts
        };
        let went_stale = |facts: Facts| Facts {
            clock_started: true,
            ..no_clock(facts)
        };
        let copter = Some(Vehicle::Copter);
        let (miss, bad) = (Some("no key"), Some("FS_GCS_ENABLE is 0, not 5"));
        // (policy, facts, target, the interlocks that fail)
        let cases = [
            (
                sitl,
                facts(false, 0, None, None, None),
                Sink,
                vec![Interlock::Identity],
            ),
            (
                sitl,
                facts(false, 9, Some(Vehicle::Rover), None, None),
                Sink,
                vec![Interlock::Identity],
            ),
            (
                sitl,
                facts(true, 0, None, None, None),
                Sink,
                vec![Interlock::Identity],
            ),
            (sitl, facts(true, 0, copter, None, None), Goes, vec![]),
            // SITL mode never waits for keys or parameters (the verifier refuses a keyless sender).
            (sitl, facts(true, 0, copter, miss, bad), Goes, vec![]),
            (
                real,
                facts(false, REAL_PROBES, copter, None, None),
                Goes,
                vec![],
            ),
            (
                real,
                facts(false, REAL_PROBES - 1, copter, None, None),
                Sink,
                vec![Interlock::Identity],
            ),
            (
                real,
                facts(true, 0, copter, None, None),
                Sink,
                vec![Interlock::Identity],
            ),
            (
                real,
                facts(false, REAL_PROBES, None, None, None),
                Sink,
                vec![Interlock::Identity],
            ),
            (
                real,
                facts(false, REAL_PROBES, copter, miss, None),
                Sink,
                vec![Interlock::ControllerKeys],
            ),
            (
                real,
                facts(false, REAL_PROBES, copter, None, bad),
                Sink,
                vec![Interlock::FailsafeParams],
            ),
            (
                real,
                facts(true, 0, copter, miss, bad),
                Sink,
                vec![
                    Interlock::Identity,
                    Interlock::ControllerKeys,
                    Interlock::FailsafeParams,
                ],
            ),
            (
                real_on_sitl,
                facts(true, 0, copter, None, None),
                Goes,
                vec![],
            ),
            (
                real_on_sitl,
                facts(false, 0, copter, None, None),
                Goes,
                vec![],
            ),
            (
                real_on_sitl,
                facts(true, 0, copter, miss, None),
                Sink,
                vec![Interlock::ControllerKeys],
            ),
            (
                real_on_sitl,
                facts(true, 0, copter, None, bad),
                Sink,
                vec![Interlock::FailsafeParams],
            ),
            (
                real,
                no_clock(facts(false, REAL_PROBES, copter, None, None)),
                Sink,
                vec![Interlock::ClockBound],
            ),
            (
                real_on_sitl,
                no_clock(facts(true, 0, copter, None, None)),
                Sink,
                vec![Interlock::ClockBound],
            ),
            (
                sitl,
                no_clock(facts(true, 0, copter, None, None)),
                Goes,
                vec![],
            ),
            // A bound that goes stale after the start never withdraws control.
            (
                real,
                went_stale(facts(false, REAL_PROBES, copter, None, None)),
                Goes,
                vec![],
            ),
            (
                real_on_sitl,
                went_stale(facts(true, 0, copter, None, None)),
                Goes,
                vec![],
            ),
            // A parameter that drifts after the start does, whatever the clock.
            (
                real,
                went_stale(facts(false, REAL_PROBES, copter, None, bad)),
                Sink,
                vec![Interlock::FailsafeParams],
            ),
        ];
        for (policy, facts, target, failing) in cases {
            let failed: Vec<Interlock> = failures(policy, &facts)
                .iter()
                .map(|f| f.interlock)
                .collect();
            assert_eq!(failed, failing, "{policy:?} {facts:?}");
            assert_eq!(
                control_target(policy, &facts),
                target,
                "{policy:?} {facts:?}"
            );
        }
    }

    #[test]
    fn the_marker_belongs_to_the_autopilot_that_answered_it() {
        let marker = MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_id: SITL_MARKER.into(),
            ..PARAM_VALUE_DATA::default()
        });
        let (sim, other) = ((1, 1), (2, 1));
        let sitl = Autopilot::default()
            .heartbeat(sim, Some(Vehicle::Copter))
            .param_value(sim, &marker);
        assert!(sitl.sitl_marker);

        let steps = [
            sitl.heartbeat(other, Some(Vehicle::Copter)),
            sitl.heartbeat(sim, Some(Vehicle::Rover)),
            sitl.heartbeat(sim, None),
            Autopilot::default()
                .heartbeat(sim, Some(Vehicle::Copter))
                .param_value(other, &marker),
        ];
        for after in steps {
            assert!(!after.sitl_marker, "{after:?}");
            assert_eq!(after.probes, 0, "{after:?}");
        }
        assert!(sitl.heartbeat(sim, Some(Vehicle::Copter)).sitl_marker);
    }

    #[test]
    fn only_the_exact_marker_parameter_identifies_sitl() {
        let value = |name: &str| {
            MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_id: name.into(),
                ..PARAM_VALUE_DATA::default()
            })
        };
        assert!(is_sitl_marker(&value(SITL_MARKER)));
        assert!(!is_sitl_marker(&value("SIM_WIND_DIR")));
        assert!(!is_sitl_marker(&value("SIM_WIND_SPD2")));
        assert!(!is_sitl_marker(&value("WIND_SPD")));
    }
}
