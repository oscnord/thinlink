//! The command gate: which ground-station messages may reach the autopilot, and how. Read-only
//! traffic `QGroundControl` needs to load a vehicle travels raw, lease or not. Everything that
//! changes the vehicle travels only inside control envelopes from the lease holder, and only
//! the kinds below; the rest never reaches SITL.

use std::hash::{DefaultHasher, Hash, Hasher};

use mavlink::dialects::ardupilotmega::{
    COMMAND_LONG_DATA, MISSION_ITEM_INT_DATA, MavAutopilot, MavCmd, MavFrame, MavMessage,
    MavMissionType, MavType,
};
use serde::Serialize;

use sat_teleop_policy::control::Kind as Class;

use crate::envelope::Kind;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    Forward,
    /// A message type outside the allowlist.
    NotAllowlisted,
    /// `COMMAND_LONG` or `COMMAND_INT` with a command outside the allowlist.
    CommandNotAllowlisted,
}

/// Commands that only ask for information or set telemetry rates.
#[expect(deprecated, reason = "QGroundControl and ArduPilot still use them")]
const READ_ONLY_COMMANDS: [MavCmd; 4] = [
    MavCmd::MAV_CMD_REQUEST_MESSAGE,
    MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
    MavCmd::MAV_CMD_REQUEST_PROTOCOL_VERSION,
    MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
];

/// Discrete commands, as `COMMAND_LONG`: what `QGroundControl` 5.1.5 sends to arm or disarm, to
/// set an `ArduPilot` mode (which is also how it lands, returns and brakes a copter), to take a
/// copter off and to start a mission from the ground (`ArduPilot` sets Auto itself), and
/// `REQUEST_MESSAGE`. Arming and taking off are allowed only because the bridge's target is SITL;
/// starting a mission only with `allow_missions`.
const DISCRETE_COMMANDS: [MavCmd; 5] = [
    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
    MavCmd::MAV_CMD_DO_SET_MODE,
    MavCmd::MAV_CMD_NAV_TAKEOFF,
    MavCmd::MAV_CMD_MISSION_START,
    MavCmd::MAV_CMD_REQUEST_MESSAGE,
];

/// What the vehicle does with a message that arrives raw from the relay.
#[expect(deprecated, reason = "QGroundControl and ArduPilot still use them")]
pub fn decide(message: &MavMessage) -> Decision {
    match message {
        MavMessage::PARAM_REQUEST_LIST(_)
        | MavMessage::PARAM_REQUEST_READ(_)
        | MavMessage::MISSION_REQUEST_LIST(_)
        | MavMessage::MISSION_REQUEST_INT(_)
        | MavMessage::MISSION_REQUEST(_)
        | MavMessage::MISSION_ACK(_)
        | MavMessage::REQUEST_DATA_STREAM(_)
        | MavMessage::TIMESYNC(_)
        | MavMessage::SYSTEM_TIME(_) => Decision::Forward,
        MavMessage::COMMAND_LONG(c) => decide_command(c.command),
        MavMessage::COMMAND_INT(c) => decide_command(c.command),
        _ => Decision::NotAllowlisted,
    }
}

fn decide_command(command: MavCmd) -> Decision {
    if READ_ONLY_COMMANDS.contains(&command) {
        Decision::Forward
    } else {
        Decision::CommandNotAllowlisted
    }
}

/// The envelope kind that may carry `message`, if any: a ground station's heartbeat as liveness,
/// `MANUAL_CONTROL` as a continuous setpoint, the discrete commands and the mission upload
/// (`MISSION_COUNT`, `MISSION_ITEM_INT`, `MISSION_CLEAR_ALL`) as discrete.
pub fn wrap(message: &MavMessage) -> Option<Class> {
    match message {
        MavMessage::HEARTBEAT(h) if h.mavtype == MavType::MAV_TYPE_GCS => Some(Class::Liveness),
        MavMessage::MANUAL_CONTROL(_) => Some(Class::Continuous),
        MavMessage::MISSION_COUNT(_)
        | MavMessage::MISSION_ITEM_INT(_)
        | MavMessage::MISSION_CLEAR_ALL(_) => Some(Class::Discrete),
        MavMessage::COMMAND_LONG(c) if DISCRETE_COMMANDS.contains(&c.command) => {
            Some(Class::Discrete)
        }
        _ => None,
    }
}

/// The autopilot behind the bridge, as its own `HEARTBEAT` says; the modes a ground station may
/// select depend on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Vehicle {
    Copter,
    Rover,
}

/// `ArduCopter` custom modes (`ArduCopter/mode.h`, `Mode::Number`) a ground station may select:
/// Stabilize, `AltHold`, Guided, Loiter, RTL, Land, `PosHold`, Brake.
const COPTER_MODES: [u8; 8] = [0, 2, 4, 5, 6, 9, 16, 17];
/// Auto (`Mode::Number::AUTO`), which flies the uploaded mission; allowed only with `allow_missions`.
const COPTER_AUTO: u8 = 3;
/// Of those, the modes that take the throttle from the sticks. Defence in depth: the autopilot
/// already holds its height in its radio failsafe, and a drop with no stick input did not
/// reproduce in SITL.
const COPTER_STICK_MODES: [u8; 4] = [0, 2, 5, 16];
/// `ArduRover` custom modes (`Rover/mode.h`, `Mode::Number`): Manual, Hold and RTL.
const ROVER_MODES: [u8; 3] = [0, 4, 11];
/// `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`: `DO_SET_MODE` param 1 as `QGroundControl` sends it. `ArduPilot`
/// reads param 2 as the mode only when this bit is set (`GCS_MAVLINK::_set_mode_common`).
const CUSTOM_MODE_ENABLED: f32 = 1.0;
/// The highest takeoff altitude (m above home) a ground station may ask for, and the highest a
/// mission item may.
const MAX_TAKEOFF_M: f32 = 120.0;
/// The most items a mission upload may announce. Every item is a transaction of its own and the
/// vehicle remembers 64 (`sat_teleop_policy::control`), oldest first: more would push an arm or a
/// takeoff out of its memory, and a retry of it would run again.
const MAX_MISSION_ITEMS: u16 = 48;
/// The longest a mission item may hold the copter (s): a waypoint's hold, a timed loiter, a delay.
const MAX_HOLD_S: f32 = 300.0;
/// The most turns a counted loiter may fly.
const MAX_LOITER_TURNS: f32 = 10.0;
/// The commands a waypoint mission may hold: flying (waypoint, takeoff, land, return, a timed or
/// counted loiter), changing speed and waiting. Anything else (the fence, a mode, a servo,
/// a relay, a gripper, a parachute, an auxiliary function) can undo what the autopilot's own bounds
/// rely on, and an unlimited loiter would never let a mission flown out of reach end.
const MISSION_COMMANDS: [MavCmd; 8] = [
    MavCmd::MAV_CMD_NAV_WAYPOINT,
    MavCmd::MAV_CMD_NAV_TAKEOFF,
    MavCmd::MAV_CMD_NAV_LAND,
    MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MavCmd::MAV_CMD_NAV_LOITER_TIME,
    MavCmd::MAV_CMD_NAV_LOITER_TURNS,
    MavCmd::MAV_CMD_DO_CHANGE_SPEED,
    MavCmd::MAV_CMD_CONDITION_DELAY,
];

impl Vehicle {
    fn modes(self, missions: bool) -> impl Iterator<Item = u8> {
        let listed: &[u8] = match self {
            Self::Copter => &COPTER_MODES,
            Self::Rover => &ROVER_MODES,
        };
        let auto = (missions && self == Self::Copter).then_some(COPTER_AUTO);
        listed.iter().copied().chain(auto)
    }

    fn needs_sticks(self, mode: u8) -> bool {
        self == Self::Copter && COPTER_STICK_MODES.contains(&mode)
    }
}

/// Whether `message` is an `ArduPilot` autopilot's own `HEARTBEAT`, of any vehicle type.
pub fn is_autopilot_heartbeat(message: &MavMessage) -> bool {
    matches!(
        message,
        MavMessage::HEARTBEAT(h) if h.autopilot == MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA
    )
}

/// The vehicle an autopilot's `HEARTBEAT` describes, if it is one the gate knows.
pub fn vehicle_of(message: &MavMessage) -> Option<Vehicle> {
    let MavMessage::HEARTBEAT(h) = message else {
        return None;
    };
    if !is_autopilot_heartbeat(message) {
        return None;
    }
    match h.mavtype {
        MavType::MAV_TYPE_QUADROTOR
        | MavType::MAV_TYPE_HEXAROTOR
        | MavType::MAV_TYPE_OCTOROTOR
        | MavType::MAV_TYPE_TRICOPTER
        | MavType::MAV_TYPE_DECAROTOR
        | MavType::MAV_TYPE_DODECAROTOR
        | MavType::MAV_TYPE_COAXIAL
        | MavType::MAV_TYPE_HELICOPTER => Some(Vehicle::Copter),
        MavType::MAV_TYPE_GROUND_ROVER => Some(Vehicle::Rover),
        _ => None,
    }
}

/// Why the vehicle refuses a discrete command it would otherwise pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refused {
    /// A mode change while the vehicle type is unknown, to a mode outside the type's list, or not
    /// shaped as `QGroundControl` sends it.
    ModeNotAllowed,
    /// A copter mode change into a mode that takes the throttle from the sticks, with no fresh
    /// `MANUAL_CONTROL` setpoint from the lease holder. Defence in depth: the autopilot already
    /// holds its height in its radio failsafe.
    NoFreshSticks,
    /// A takeoff altitude that is not a positive number up to `MAX_TAKEOFF_M`.
    TakeoffAltitude,
    /// A mission upload or start with `allow_missions` off, for anything but a copter, or of a
    /// mission type other than the waypoint mission (the fence and rally points are the
    /// autopilot's own bounds, and no ground station may rewrite them over the link).
    MissionNotAllowed,
    /// A mission item outside what `item_refused` allows.
    MissionItemNotAllowed,
    /// A `MISSION_COUNT` above `MAX_MISSION_ITEMS`.
    MissionTooLong,
}

/// Whether the vehicle refuses a discrete message: a `COMMAND_LONG` or a mission upload message.
/// `sticks_fresh` is whether the lease holder's `MANUAL_CONTROL` setpoint is fresh now and
/// `missions` whether the vehicle was configured with `allow_missions`. Messages that carry no
/// such limit pass.
pub fn refused(
    vehicle: Option<Vehicle>,
    message: &MavMessage,
    sticks_fresh: bool,
    missions: bool,
) -> Option<Refused> {
    let mission = |kind| mission_refused(vehicle, missions, kind);
    match message {
        MavMessage::COMMAND_LONG(c) => match c.command {
            MavCmd::MAV_CMD_DO_SET_MODE => mode_refused(vehicle, c, sticks_fresh, missions),
            MavCmd::MAV_CMD_NAV_TAKEOFF => takeoff_refused(c.param7),
            MavCmd::MAV_CMD_MISSION_START => mission(MavMissionType::MAV_MISSION_TYPE_MISSION),
            _ => None,
        },
        MavMessage::MISSION_COUNT(m) => mission(m.mission_type)
            .or_else(|| (m.count > MAX_MISSION_ITEMS).then_some(Refused::MissionTooLong)),
        MavMessage::MISSION_ITEM_INT(m) => mission(m.mission_type)
            .or_else(|| item_refused(m).then_some(Refused::MissionItemNotAllowed)),
        MavMessage::MISSION_CLEAR_ALL(m) => mission(m.mission_type),
        _ => None,
    }
}

fn mission_refused(
    vehicle: Option<Vehicle>,
    missions: bool,
    kind: MavMissionType,
) -> Option<Refused> {
    let allowed = missions
        && vehicle == Some(Vehicle::Copter)
        && kind == MavMissionType::MAV_MISSION_TYPE_MISSION;
    (!allowed).then_some(Refused::MissionNotAllowed)
}

/// Whether a mission item may not be stored. Item 0 is the home position: `ArduPilot` never reads
/// what it is sent for it (`AP_Mission::replace_cmd` returns without writing index 0 and
/// `read_cmd_from_storage` answers index 0 with the AHRS home, `ArduPilot` `fb773567`), so it may
/// have any frame and altitude but must be a waypoint, as `QGroundControl` sends it. Any other
/// item is one of `MISSION_COMMANDS`, and an item with a location (a waypoint, takeoff, land or
/// loiter) is relative to home, above it by up to `MAX_TAKEOFF_M` (a landing may ask for 0). No
/// item may have `current` above 1: `ArduPilot` takes a `MISSION_ITEM_INT` with `current` 2 as a
/// Guided "fly to here" (`handle_guided_request`) and with 3 as an altitude change, outside the
/// mission and the upload protocol and whatever its seq (`GCS_MAVLINK::handle_mission_item`,
/// `GCS_Common.cpp`, `fb773567`).
#[expect(deprecated, reason = "the same frame as MAV_FRAME_GLOBAL_RELATIVE_ALT")]
fn item_refused(item: &MISSION_ITEM_INT_DATA) -> bool {
    if item.current > 1 {
        return true;
    }
    if item.seq == 0 {
        return item.command != MavCmd::MAV_CMD_NAV_WAYPOINT;
    }
    if !MISSION_COMMANDS.contains(&item.command) {
        return true;
    }
    if hold_refused(item) {
        return true;
    }
    let located = matches!(
        item.command,
        MavCmd::MAV_CMD_NAV_WAYPOINT
            | MavCmd::MAV_CMD_NAV_TAKEOFF
            | MavCmd::MAV_CMD_NAV_LAND
            | MavCmd::MAV_CMD_NAV_LOITER_TIME
            | MavCmd::MAV_CMD_NAV_LOITER_TURNS
    );
    if !located {
        return false;
    }
    let relative = matches!(
        item.frame,
        MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT | MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    );
    let lowest = if item.command == MavCmd::MAV_CMD_NAV_LAND {
        0.0
    } else {
        f32::MIN_POSITIVE
    };
    // false for NaN too
    let high_enough = item.z >= lowest && item.z <= MAX_TAKEOFF_M;
    !(relative && high_enough)
}

/// Whether an item holds the copter too long for a mission that must end: a waypoint's hold, a timed
/// loiter and a delay (param 1, seconds) or a counted loiter (param 1, turns). Not a number in range
/// (NaN, negative) is refused too. Jumps are not mission commands: a loop, or nested jumps, would
/// multiply what the caps allow.
fn hold_refused(item: &MISSION_ITEM_INT_DATA) -> bool {
    let limit = match item.command {
        MavCmd::MAV_CMD_NAV_WAYPOINT
        | MavCmd::MAV_CMD_NAV_LOITER_TIME
        | MavCmd::MAV_CMD_CONDITION_DELAY => MAX_HOLD_S,
        MavCmd::MAV_CMD_NAV_LOITER_TURNS => MAX_LOITER_TURNS,
        _ => return false,
    };
    !(0.0..=limit).contains(&item.param1)
}

fn mode_refused(
    vehicle: Option<Vehicle>,
    c: &COMMAND_LONG_DATA,
    sticks_fresh: bool,
    missions: bool,
) -> Option<Refused> {
    let custom = c.param1.to_bits() == CUSTOM_MODE_ENABLED.to_bits();
    let mode = vehicle.filter(|_| custom).and_then(|vehicle| {
        let whole = |mode: &u8| f32::from(*mode).to_bits() == c.param2.to_bits();
        vehicle.modes(missions).find(whole)
    });
    let (Some(vehicle), Some(mode)) = (vehicle, mode) else {
        return Some(Refused::ModeNotAllowed);
    };
    (vehicle.needs_sticks(mode) && !sticks_fresh).then_some(Refused::NoFreshSticks)
}

fn takeoff_refused(altitude_m: f32) -> Option<Refused> {
    // false for NaN too
    let allowed = altitude_m > 0.0 && altitude_m <= MAX_TAKEOFF_M;
    (!allowed).then_some(Refused::TakeoffAltitude)
}

/// Whether an envelope of `kind` may carry `message`.
pub fn admits(kind: Kind, message: &MavMessage) -> bool {
    wrap(message) == Some(Class::from(kind))
}

/// A `COMMAND_LONG` as its sender meant it: all of it but `confirmation`, which grows with each
/// resend. Parameters compare by their bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CommandKey {
    command: u32,
    target_system: u8,
    target_component: u8,
    params: [u32; 7],
}

impl CommandKey {
    pub fn of(message: &MavMessage) -> Option<Self> {
        let MavMessage::COMMAND_LONG(c) = message else {
            return None;
        };
        let params = [
            c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7,
        ];
        Some(Self {
            command: c.command as u32,
            target_system: c.target_system,
            target_component: c.target_component,
            params: params.map(f32::to_bits),
        })
    }

    /// A digest for the vehicle's transaction cache, stable within one process.
    pub fn digest(&self) -> u64 {
        let mut hasher = DefaultHasher::new();
        self.hash(&mut hasher);
        hasher.finish()
    }
}

/// How the ground adapter handles a ground station's message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// Read-only: straight to mavlink-router, lease or not.
    Raw,
    /// Inside an envelope while the adapter holds the lease.
    Envelope(Class),
    /// A command that changes the vehicle but no envelope may carry: answered by the adapter.
    Unsupported,
    /// Dropped.
    Refuse,
}

pub fn route(message: &MavMessage) -> Route {
    if decide(message) == Decision::Forward {
        return Route::Raw;
    }
    if let Some(wrap) = wrap(message) {
        return Route::Envelope(wrap);
    }
    if matches!(
        message,
        MavMessage::COMMAND_LONG(_) | MavMessage::COMMAND_INT(_)
    ) {
        return Route::Unsupported;
    }
    Route::Refuse
}

#[cfg(test)]
mod tests {
    #[expect(
        deprecated,
        reason = "a ground station may still send SET_MODE; it stays refused"
    )]
    use mavlink::dialects::ardupilotmega::{
        COMMAND_INT_DATA, COMMAND_LONG_DATA, HEARTBEAT_DATA, MANUAL_CONTROL_DATA,
        MISSION_CLEAR_ALL_DATA, MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA,
        MISSION_SET_CURRENT_DATA, PARAM_SET_DATA, RC_CHANNELS_OVERRIDE_DATA, SET_MODE_DATA,
    };

    use super::*;

    fn long(command: MavCmd) -> MavMessage {
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command,
            ..COMMAND_LONG_DATA::default()
        })
    }

    fn int(command: MavCmd) -> MavMessage {
        MavMessage::COMMAND_INT(COMMAND_INT_DATA {
            command,
            ..COMMAND_INT_DATA::default()
        })
    }

    fn heartbeat(mavtype: MavType) -> MavMessage {
        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            mavtype,
            ..HEARTBEAT_DATA::default()
        })
    }

    #[test]
    #[expect(
        deprecated,
        reason = "a ground station may still send SET_MODE; it stays refused"
    )]
    fn only_read_only_traffic_travels_raw_and_envelopes_carry_only_their_kind() {
        use Class::{Continuous, Discrete, Liveness};
        use Decision::{CommandNotAllowlisted, Forward, NotAllowlisted};
        use Route::{Envelope, Raw, Refuse, Unsupported};
        // (message, raw from the relay, route at the ground)
        let cases = [
            (long(MavCmd::MAV_CMD_REQUEST_MESSAGE), Forward, Raw),
            (
                long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                CommandNotAllowlisted,
                Envelope(Discrete),
            ),
            (
                long(MavCmd::MAV_CMD_DO_SET_MODE),
                CommandNotAllowlisted,
                Envelope(Discrete),
            ),
            (
                long(MavCmd::MAV_CMD_NAV_TAKEOFF),
                CommandNotAllowlisted,
                Envelope(Discrete),
            ),
            (
                long(MavCmd::MAV_CMD_MISSION_START),
                CommandNotAllowlisted,
                Envelope(Discrete),
            ),
            (
                MavMessage::MISSION_COUNT(MISSION_COUNT_DATA::default()),
                NotAllowlisted,
                Envelope(Discrete),
            ),
            (
                MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA::default()),
                NotAllowlisted,
                Envelope(Discrete),
            ),
            (
                MavMessage::MISSION_CLEAR_ALL(MISSION_CLEAR_ALL_DATA::default()),
                NotAllowlisted,
                Envelope(Discrete),
            ),
            (
                MavMessage::MISSION_SET_CURRENT(MISSION_SET_CURRENT_DATA::default()),
                NotAllowlisted,
                Refuse,
            ),
            (
                int(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                CommandNotAllowlisted,
                Unsupported,
            ),
            (
                int(MavCmd::MAV_CMD_DO_SET_MODE),
                CommandNotAllowlisted,
                Unsupported,
            ),
            (
                long(MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN),
                CommandNotAllowlisted,
                Unsupported,
            ),
            (
                heartbeat(MavType::MAV_TYPE_GCS),
                NotAllowlisted,
                Envelope(Liveness),
            ),
            (
                heartbeat(MavType::MAV_TYPE_GROUND_ROVER),
                NotAllowlisted,
                Refuse,
            ),
            (
                MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA::default()),
                NotAllowlisted,
                Envelope(Continuous),
            ),
            (
                MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA::default()),
                NotAllowlisted,
                Refuse,
            ),
            (
                MavMessage::SET_MODE(SET_MODE_DATA::default()),
                NotAllowlisted,
                Refuse,
            ),
            (
                MavMessage::PARAM_SET(PARAM_SET_DATA::default()),
                NotAllowlisted,
                Refuse,
            ),
        ];
        for (message, raw, routed) in &cases {
            assert_eq!(decide(message), *raw, "{message:?}");
            assert_eq!(route(message), *routed, "{message:?}");
        }
        let arm = long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
        assert!(admits(Kind::Discrete { txn: 1 }, &arm));
        assert!(!admits(Kind::Continuous, &arm));
        assert!(!admits(Kind::Liveness, &arm));
    }

    fn set_mode(param1: f32, mode: f32) -> MavMessage {
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command: MavCmd::MAV_CMD_DO_SET_MODE,
            param1,
            param2: mode,
            ..COMMAND_LONG_DATA::default()
        })
    }

    fn takeoff(altitude_m: f32) -> MavMessage {
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command: MavCmd::MAV_CMD_NAV_TAKEOFF,
            param7: altitude_m,
            ..COMMAND_LONG_DATA::default()
        })
    }

    #[test]
    fn mode_changes_and_takeoffs_are_limited_by_vehicle_type_and_sticks() {
        use Refused::{ModeNotAllowed, NoFreshSticks, TakeoffAltitude};
        use Vehicle::{Copter, Rover};
        let custom = 1.0;
        // (vehicle the autopilot's heartbeat described, message, sticks fresh, refusal)
        let cases = [
            (Some(Copter), set_mode(custom, 5.0), true, None),
            (Some(Rover), set_mode(custom, 0.0), false, None),
            (
                Some(Copter),
                set_mode(custom, 5.0),
                false,
                Some(NoFreshSticks),
            ),
            (
                Some(Rover),
                set_mode(custom, 17.0),
                true,
                Some(ModeNotAllowed),
            ),
            (Some(Copter), set_mode(0.0, 4.0), true, Some(ModeNotAllowed)),
            (
                Some(Copter),
                set_mode(custom, 4.5),
                true,
                Some(ModeNotAllowed),
            ),
            (None, set_mode(custom, 4.0), true, Some(ModeNotAllowed)),
            (Some(Copter), takeoff(10.0), false, None),
            (Some(Copter), takeoff(120.5), false, Some(TakeoffAltitude)),
            (Some(Copter), takeoff(-5.0), false, Some(TakeoffAltitude)),
            (
                Some(Copter),
                takeoff(f32::NAN),
                false,
                Some(TakeoffAltitude),
            ),
            (
                None,
                long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM),
                false,
                None,
            ),
        ];
        for (vehicle, message, sticks, expected) in &cases {
            assert_eq!(
                refused(*vehicle, message, *sticks, false),
                *expected,
                "{vehicle:?} {message:?} sticks fresh: {sticks}"
            );
        }
    }

    #[test]
    fn missions_and_auto_pass_only_with_the_switch_for_a_copter_and_the_waypoint_mission() {
        use Refused::{MissionNotAllowed, ModeNotAllowed};
        use Vehicle::{Copter, Rover};
        let count = |mission_type| {
            MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
                mission_type,
                ..MISSION_COUNT_DATA::default()
            })
        };
        let long_count = |count| {
            MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
                count,
                ..MISSION_COUNT_DATA::default()
            })
        };
        let item = |mission_type| {
            MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
                mission_type,
                ..MISSION_ITEM_INT_DATA::default()
            })
        };
        let clear = |mission_type| {
            MavMessage::MISSION_CLEAR_ALL(MISSION_CLEAR_ALL_DATA {
                mission_type,
                ..MISSION_CLEAR_ALL_DATA::default()
            })
        };
        let waypoints = MavMissionType::MAV_MISSION_TYPE_MISSION;
        let fence = MavMissionType::MAV_MISSION_TYPE_FENCE;
        let rally = MavMissionType::MAV_MISSION_TYPE_RALLY;
        let start = long(MavCmd::MAV_CMD_MISSION_START);
        // (vehicle, message, switch on, refusal)
        let cases = [
            (Some(Copter), count(waypoints), true, None),
            (Some(Copter), item(waypoints), true, None),
            (Some(Copter), clear(waypoints), true, None),
            (Some(Copter), start.clone(), true, None),
            (Some(Copter), set_mode(1.0, 3.0), true, None),
            (
                Some(Copter),
                count(waypoints),
                false,
                Some(MissionNotAllowed),
            ),
            (
                Some(Copter),
                item(waypoints),
                false,
                Some(MissionNotAllowed),
            ),
            (
                Some(Copter),
                clear(waypoints),
                false,
                Some(MissionNotAllowed),
            ),
            (Some(Copter), start.clone(), false, Some(MissionNotAllowed)),
            (
                Some(Copter),
                set_mode(1.0, 3.0),
                false,
                Some(ModeNotAllowed),
            ),
            (Some(Copter), long_count(48), true, None),
            (
                Some(Copter),
                long_count(49),
                true,
                Some(Refused::MissionTooLong),
            ),
            (Some(Copter), count(fence), true, Some(MissionNotAllowed)),
            (Some(Copter), item(rally), true, Some(MissionNotAllowed)),
            (Some(Copter), clear(fence), true, Some(MissionNotAllowed)),
            (Some(Rover), count(waypoints), true, Some(MissionNotAllowed)),
            (Some(Rover), start, true, Some(MissionNotAllowed)),
            (Some(Rover), set_mode(1.0, 3.0), true, Some(ModeNotAllowed)),
            (None, count(waypoints), true, Some(MissionNotAllowed)),
            (None, set_mode(1.0, 3.0), true, Some(ModeNotAllowed)),
        ];
        for (vehicle, message, missions, expected) in &cases {
            assert_eq!(
                refused(*vehicle, message, false, *missions),
                *expected,
                "{vehicle:?} {message:?} switch: {missions}"
            );
        }
    }

    #[test]
    fn a_mission_item_must_be_a_flying_command_relative_to_home_and_low_enough() {
        use MavCmd::{
            MAV_CMD_CONDITION_DELAY, MAV_CMD_DO_AUX_FUNCTION, MAV_CMD_DO_CHANGE_SPEED,
            MAV_CMD_DO_FENCE_ENABLE, MAV_CMD_DO_JUMP, MAV_CMD_DO_SET_MODE, MAV_CMD_DO_SET_SERVO,
            MAV_CMD_NAV_LAND, MAV_CMD_NAV_LOITER_TIME, MAV_CMD_NAV_LOITER_TURNS,
            MAV_CMD_NAV_LOITER_UNLIM, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_NAV_TAKEOFF,
            MAV_CMD_NAV_WAYPOINT,
        };
        let relative = MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT;
        let item = |seq, command, frame, z| {
            MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
                seq,
                command,
                frame,
                z,
                ..MISSION_ITEM_INT_DATA::default()
            })
        };
        let refused_item = Some(Refused::MissionItemNotAllowed);
        let hold = |command, param1| {
            MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
                seq: 3,
                command,
                frame: relative,
                z: 10.0,
                param1,
                ..MISSION_ITEM_INT_DATA::default()
            })
        };
        let current = |seq, current| {
            MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
                seq,
                current,
                command: MAV_CMD_NAV_WAYPOINT,
                frame: relative,
                z: 10.0,
                ..MISSION_ITEM_INT_DATA::default()
            })
        };
        // (message, refusal), with the switch on for a copter
        let cases = [
            // the home placeholder: any frame, any altitude, but a waypoint
            (
                item(0, MAV_CMD_NAV_WAYPOINT, MavFrame::MAV_FRAME_GLOBAL, 700.0),
                None,
            ),
            (
                item(0, MAV_CMD_DO_FENCE_ENABLE, relative, 0.0),
                refused_item,
            ),
            (item(1, MAV_CMD_NAV_TAKEOFF, relative, 10.0), None),
            (item(2, MAV_CMD_NAV_WAYPOINT, relative, 120.0), None),
            (item(2, MAV_CMD_NAV_WAYPOINT, relative, 120.5), refused_item),
            (item(2, MAV_CMD_NAV_WAYPOINT, relative, 0.0), refused_item),
            (
                item(2, MAV_CMD_NAV_WAYPOINT, relative, f32::NAN),
                refused_item,
            ),
            (
                item(2, MAV_CMD_NAV_WAYPOINT, MavFrame::MAV_FRAME_GLOBAL, 50.0),
                refused_item,
            ),
            (
                item(
                    2,
                    MAV_CMD_NAV_TAKEOFF,
                    MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
                    50.0,
                ),
                refused_item,
            ),
            (item(3, MAV_CMD_NAV_LOITER_TIME, relative, 30.0), None),
            (item(4, MAV_CMD_NAV_LAND, relative, 0.0), None),
            (item(4, MAV_CMD_NAV_LAND, relative, 200.0), refused_item),
            (
                item(
                    5,
                    MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    MavFrame::MAV_FRAME_GLOBAL,
                    0.0,
                ),
                None,
            ),
            // current 2 and 3 are a Guided goto and an altitude change, whatever the seq
            (current(0, 1), None),
            (current(2, 1), None),
            (current(0, 2), refused_item),
            (current(2, 2), refused_item),
            (current(0, 3), refused_item),
            (current(2, 3), refused_item),
            // a mission out of reach must end: no jumps, no unlimited loiter, bounded holds
            (
                item(5, MAV_CMD_DO_JUMP, MavFrame::MAV_FRAME_MISSION, 0.0),
                refused_item,
            ),
            (hold(MAV_CMD_NAV_WAYPOINT, 300.0), None),
            (hold(MAV_CMD_NAV_WAYPOINT, 301.0), refused_item),
            (hold(MAV_CMD_NAV_WAYPOINT, -1.0), refused_item),
            (hold(MAV_CMD_NAV_WAYPOINT, f32::NAN), refused_item),
            (hold(MAV_CMD_NAV_LOITER_TIME, 300.0), None),
            (hold(MAV_CMD_NAV_LOITER_TIME, 301.0), refused_item),
            (hold(MAV_CMD_CONDITION_DELAY, 300.0), None),
            (hold(MAV_CMD_CONDITION_DELAY, 301.0), refused_item),
            (hold(MAV_CMD_CONDITION_DELAY, f32::NAN), refused_item),
            (hold(MAV_CMD_NAV_LOITER_TURNS, 10.0), None),
            (hold(MAV_CMD_NAV_LOITER_TURNS, 11.0), refused_item),
            (hold(MAV_CMD_NAV_LOITER_TURNS, -1.0), refused_item),
            (
                item(3, MAV_CMD_NAV_LOITER_UNLIM, relative, 30.0),
                refused_item,
            ),
            (
                item(5, MAV_CMD_DO_CHANGE_SPEED, MavFrame::MAV_FRAME_MISSION, 0.0),
                None,
            ),
            (
                item(5, MAV_CMD_CONDITION_DELAY, MavFrame::MAV_FRAME_MISSION, 0.0),
                None,
            ),
            (
                item(5, MAV_CMD_DO_FENCE_ENABLE, MavFrame::MAV_FRAME_MISSION, 0.0),
                refused_item,
            ),
            (
                item(5, MAV_CMD_DO_SET_MODE, MavFrame::MAV_FRAME_MISSION, 0.0),
                refused_item,
            ),
            (
                item(5, MAV_CMD_DO_AUX_FUNCTION, MavFrame::MAV_FRAME_MISSION, 0.0),
                refused_item,
            ),
            (
                item(5, MAV_CMD_DO_SET_SERVO, MavFrame::MAV_FRAME_MISSION, 0.0),
                refused_item,
            ),
        ];
        for (message, expected) in &cases {
            assert_eq!(
                refused(Some(Vehicle::Copter), message, false, true),
                *expected,
                "{message:?}"
            );
        }
    }

    #[test]
    fn only_an_ardupilot_heartbeat_of_a_known_type_names_the_vehicle() {
        let autopilot = |mavtype, autopilot| {
            MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                mavtype,
                autopilot,
                ..HEARTBEAT_DATA::default()
            })
        };
        let ardupilot = MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA;
        let cases = [
            (
                MavType::MAV_TYPE_QUADROTOR,
                ardupilot,
                Some(Vehicle::Copter),
            ),
            (
                MavType::MAV_TYPE_GROUND_ROVER,
                ardupilot,
                Some(Vehicle::Rover),
            ),
            (MavType::MAV_TYPE_FIXED_WING, ardupilot, None),
            (
                MavType::MAV_TYPE_GCS,
                MavAutopilot::MAV_AUTOPILOT_INVALID,
                None,
            ),
            (
                MavType::MAV_TYPE_QUADROTOR,
                MavAutopilot::MAV_AUTOPILOT_INVALID,
                None,
            ),
        ];
        for (mavtype, pilot, expected) in cases {
            assert_eq!(
                vehicle_of(&autopilot(mavtype, pilot)),
                expected,
                "{mavtype:?}"
            );
        }
    }
}
