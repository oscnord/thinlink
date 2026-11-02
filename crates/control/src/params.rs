//! The failsafe and geofence parameters real mode relies on. The vehicle asks the autopilot for each
//! one with `PARAM_REQUEST_READ`, keeps what comes back (with the system and component that said it,
//! and the type it reported) and compares it with the type and value the config requires. A parameter that is missing, has not been read lately or differs refuses
//! control: the autopilot's own failsafes are what stops the vehicle when the link goes, so they
//! must be what the operator said they are. The list and its sources are in `docs/real-vehicle.md`.

use std::collections::BTreeMap;
use std::time::Duration;

use mavlink::dialects::ardupilotmega::{MavMessage, MavParamType};
use serde::Deserialize;

/// A reading older than this does not count: the parameter may have been changed since.
pub const MAX_AGE: Duration = Duration::from_secs(30);
/// A parameter is read again once its reading is this old.
pub const REREAD_AFTER: Duration = Duration::from_secs(10);
/// `PARAM_VALUE` carries a float; the values that matter here are whole numbers or short decimals.
const TOLERANCE: f32 = 1e-4;
/// The longest name `MAVLink` carries.
const MAX_NAME: usize = 16;

/// The type a `PARAM_VALUE` reports, as `MAV_PARAM_TYPE` names it (the 64-bit types are left out:
/// no failsafe parameter has one).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamType {
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Real32,
}

impl ParamType {
    fn of(reported: MavParamType) -> Option<Self> {
        match reported {
            MavParamType::MAV_PARAM_TYPE_UINT8 => Some(Self::Uint8),
            MavParamType::MAV_PARAM_TYPE_INT8 => Some(Self::Int8),
            MavParamType::MAV_PARAM_TYPE_UINT16 => Some(Self::Uint16),
            MavParamType::MAV_PARAM_TYPE_INT16 => Some(Self::Int16),
            MavParamType::MAV_PARAM_TYPE_UINT32 => Some(Self::Uint32),
            MavParamType::MAV_PARAM_TYPE_INT32 => Some(Self::Int32),
            MavParamType::MAV_PARAM_TYPE_REAL32 => Some(Self::Real32),
            _ => None,
        }
    }
}

/// How a reading is compared with `Required::value`.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Compare {
    /// The reading equals the value.
    #[default]
    Equal,
    /// The value is a bit mask: every bit set in it is set in the reading, the others are free
    /// (a bitmask parameter such as `FS_OPTIONS`).
    BitsSet,
    /// The value is a bit mask: every bit set in it is clear in the reading, the others are free.
    BitsClear,
    /// The reading is at most the value (a limit such as `FENCE_ALT_MAX`).
    AtMost,
    /// The reading is at least the value (a floor such as `BATT_FS_LOW_ACT`).
    AtLeast,
}

/// One parameter, the type the autopilot reports it as and the value it must have.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Required {
    pub name: String,
    #[serde(rename = "type")]
    pub kind: ParamType,
    pub value: f32,
    #[serde(default)]
    pub compare: Compare,
}

/// What the autopilot last said a parameter is, with the type it said it in, and how long ago.
/// `kind` is `None` for a type the config cannot name.
#[derive(Debug, Clone, Copy)]
pub struct Reading {
    pub value: f32,
    pub kind: Option<ParamType>,
    pub age: Duration,
}

/// A `PARAM_VALUE`.
#[derive(Debug, Clone, PartialEq)]
pub struct Value {
    pub name: String,
    pub value: f32,
    pub kind: Option<ParamType>,
}

pub fn validate(required: &[Required]) -> Result<(), String> {
    let bad = |r: &Required| {
        let is_mask = matches!(r.compare, Compare::BitsSet | Compare::BitsClear);
        let mask_ok = !is_mask || bits(r.value).is_some_and(|b| b != 0);
        r.name.is_empty() || r.name.len() > MAX_NAME || !r.value.is_finite() || !mask_ok
    };
    match required.iter().find(|r| bad(r)) {
        Some(r) => Err(format!(
            "`{}` is not a parameter name of 1 to {MAX_NAME} characters with a finite value \
             (a `bits_set` or `bits_clear` value is a whole number from 1 to 2^24 - 1)",
            r.name
        )),
        None => Ok(()),
    }
}

/// What a mission flown out of reach needs the autopilot to be set to, as entries of `require`
/// (`ArduPilot` `fb773567`): the GCS failsafe and the radio failsafe let a mission in Auto carry on
/// (`FS_OPTIONS` bits 0 and 1, `RC_CONTINUE_IF_AUTO` and `GCS_CONTINUE_IF_AUTO`; a lost link stops
/// `MANUAL_CONTROL` too, which is a radio failsafe) and the GCS failsafe still acts in the stick
/// modes (bit 4, `GCS_CONTINUE_IF_PILOT_CONTROL`, clear: `ArduCopter/Copter.h`, `FailsafeOption`,
/// `events.cpp`). A geofence is on, with the maximum altitude and the circle (`FENCE_TYPE` bits 0 and
/// 1, which need no upload: the gate forbids fence uploads), a ceiling no higher than the 120 m the
/// gate holds a takeoff to, and RTL or Land on a breach (`FENCE_ACTION` 1; 4 would brake and hold).
/// The circle's radius stays the operator's choice. A battery failsafe (`BATT_FS_LOW_ACT` 1 or more:
/// not "none") is what ends a mission that runs long.
const MISSION_RULES: [Rule; 7] = [
    Rule {
        name: "FS_OPTIONS",
        compare: Compare::BitsSet,
        ok: |v| has_bits(v, 0b11),
        text: "FS_OPTIONS with `compare = \"bits_set\"` and bits 0 and 1 (value 3)",
    },
    Rule {
        name: "FS_OPTIONS",
        compare: Compare::BitsClear,
        ok: |v| has_bits(v, 0b1_0000),
        text: "FS_OPTIONS with `compare = \"bits_clear\"` and bit 4 (value 16)",
    },
    Rule {
        name: "FENCE_ENABLE",
        compare: Compare::Equal,
        ok: |v| bits(v) == Some(1),
        text: "FENCE_ENABLE equal to 1",
    },
    Rule {
        name: "FENCE_ACTION",
        compare: Compare::Equal,
        ok: |v| bits(v) == Some(1),
        text: "FENCE_ACTION equal to 1",
    },
    Rule {
        name: "FENCE_TYPE",
        compare: Compare::BitsSet,
        ok: |v| has_bits(v, 0b11),
        text: "FENCE_TYPE with `compare = \"bits_set\"` and bits 0 and 1 (value 3 or more)",
    },
    Rule {
        name: "FENCE_ALT_MAX",
        compare: Compare::AtMost,
        ok: |v| v > 0.0 && v <= MAX_MISSION_ALTITUDE_M,
        text: "FENCE_ALT_MAX with `compare = \"at_most\"` and a value above 0 and up to 120",
    },
    Rule {
        name: "BATT_FS_LOW_ACT",
        compare: Compare::AtLeast,
        ok: |v| v >= 1.0,
        text: "BATT_FS_LOW_ACT with `compare = \"at_least\"` and a value of 1 or more",
    },
];
/// One entry `require` must hold for missions: a parameter, how it is compared, a test of the
/// value and the words for the error.
struct Rule {
    name: &'static str,
    compare: Compare,
    ok: fn(f32) -> bool,
    text: &'static str,
}

/// The ceiling of the fence a mission may rely on (m): the gate's takeoff limit.
const MAX_MISSION_ALTITUDE_M: f32 = 120.0;

fn has_bits(value: f32, mask: u32) -> bool {
    bits(value).is_some_and(|b| b & mask == mask)
}

/// Whether `required` holds all of that. Checked when `allow_missions` is on in real mode.
pub fn validate_missions(required: &[Required]) -> Result<(), String> {
    let met = |rule: &Rule| {
        required
            .iter()
            .any(|r| r.name == rule.name && r.compare == rule.compare && (rule.ok)(r.value))
    };
    let missing: Vec<&str> = MISSION_RULES
        .iter()
        .filter(|rule| !met(rule))
        .map(|rule| rule.text)
        .collect();
    if missing.is_empty() {
        return Ok(());
    }
    Err(format!(
        "allow_missions in real mode needs `require` to hold {}",
        missing.join("; ")
    ))
}

/// The name, value and type in `message` when it is a `PARAM_VALUE`.
pub fn value_of(message: &MavMessage) -> Option<Value> {
    let MavMessage::PARAM_VALUE(p) = message else {
        return None;
    };
    Some(Value {
        name: p.param_id.to_str().ok()?.to_owned(),
        value: p.param_value,
        kind: ParamType::of(p.param_type),
    })
}

/// Why control is refused, or `None` when every required parameter was read lately and is right.
/// An empty required set is itself a refusal: real mode with nothing to check is not safe.
pub fn failure(required: &[Required], readings: &BTreeMap<String, Reading>) -> Option<String> {
    if required.is_empty() {
        return Some("no failsafe parameters are required in the config".to_owned());
    }
    let problems: Vec<String> = required
        .iter()
        .filter_map(|r| problem(r, readings.get(&r.name)))
        .collect();
    (!problems.is_empty()).then(|| problems.join("; "))
}

fn problem(required: &Required, reading: Option<&Reading>) -> Option<String> {
    let name = &required.name;
    let Some(reading) = reading else {
        return Some(format!("{name} was not read"));
    };
    if reading.age > MAX_AGE {
        return Some(format!(
            "{name} was last read {} s ago",
            reading.age.as_secs()
        ));
    }
    if reading.kind != Some(required.kind) {
        return Some(format!(
            "{name} is reported as {}, the config requires {:?}",
            reading.kind.map_or_else(
                || "a type the config cannot name".to_owned(),
                |k| format!("{k:?}")
            ),
            required.kind
        ));
    }
    if !holds(required, reading.value) {
        let need = match required.compare {
            Compare::Equal => required.value.to_string(),
            Compare::BitsSet => format!("bits {} set", required.value),
            Compare::BitsClear => format!("bits {} clear", required.value),
            Compare::AtMost => format!("at most {}", required.value),
            Compare::AtLeast => format!("at least {}", required.value),
        };
        return Some(format!(
            "{name} is {}, the config requires {need}",
            reading.value
        ));
    }
    None
}

/// Whether a reading of `value` satisfies `required`.
fn holds(required: &Required, value: f32) -> bool {
    match required.compare {
        Compare::Equal => (value - required.value).abs() <= TOLERANCE,
        Compare::BitsSet => bits(value)
            .zip(bits(required.value))
            .is_some_and(|(have, want)| have & want == want),
        Compare::BitsClear => bits(value)
            .zip(bits(required.value))
            .is_some_and(|(have, mask)| have & mask == 0),
        Compare::AtMost => value <= required.value + TOLERANCE,
        Compare::AtLeast => value >= required.value - TOLERANCE,
    }
}

/// `value` as a bit mask: a whole number below 2^24, where an `f32` holds every integer.
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "checked whole and in range"
)]
fn bits(value: f32) -> Option<u32> {
    let whole = value.fract() == 0.0 && (0.0..16_777_216.0).contains(&value);
    whole.then_some(value as u32)
}

/// The parameter to ask for now: one never read or read longest ago, if it is due.
pub fn next_to_read<'a>(
    required: &'a [Required],
    readings: &BTreeMap<String, Reading>,
) -> Option<&'a str> {
    required
        .iter()
        .map(|r| {
            (
                r.name.as_str(),
                readings.get(&r.name).map(|reading| reading.age),
            )
        })
        .filter(|(_, age)| age.is_none_or(|age| age >= REREAD_AFTER))
        .rev()
        .max_by_key(|(_, age)| age.unwrap_or(Duration::MAX))
        .map(|(name, _)| name)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn required() -> Vec<Required> {
        let make = |name: &str, kind, value| Required {
            name: name.to_owned(),
            kind,
            value,
            compare: Compare::Equal,
        };
        vec![
            make("FS_GCS_ENABLE", ParamType::Int8, 5.0),
            make("RC_OVERRIDE_TIME", ParamType::Real32, 0.5),
        ]
    }

    /// (name, type, value, age in seconds)
    type Said = (&'static str, ParamType, f32, u64);

    fn readings(list: &[Said]) -> BTreeMap<String, Reading> {
        list.iter()
            .map(|(name, kind, value, age)| {
                let age = Duration::from_secs(*age);
                let kind = Some(*kind);
                (
                    (*name).to_owned(),
                    Reading {
                        value: *value,
                        kind,
                        age,
                    },
                )
            })
            .collect()
    }

    #[test]
    fn control_needs_every_required_parameter_read_lately_equal_and_of_the_required_type() {
        use ParamType::{Int8, Int16, Real32};
        let right: [Said; 2] = [
            ("FS_GCS_ENABLE", Int8, 5.0, 1),
            ("RC_OVERRIDE_TIME", Real32, 0.5, 1),
        ];
        let with = |first: Said| [first, right[1]];
        let cases = [
            (required(), readings(&right), None),
            (
                required(),
                readings(&[
                    ("FS_GCS_ENABLE", Int8, 5.0, 29),
                    ("RC_OVERRIDE_TIME", Real32, 0.5, 0),
                ]),
                None,
            ),
            (
                required(),
                readings(&right[..1]),
                Some("RC_OVERRIDE_TIME was not read"),
            ),
            (
                required(),
                readings(&[]),
                Some("FS_GCS_ENABLE was not read; RC_OVERRIDE_TIME was not read"),
            ),
            (
                required(),
                readings(&with(("FS_GCS_ENABLE", Int8, 0.0, 1))),
                Some("FS_GCS_ENABLE is 0, the config requires 5"),
            ),
            (
                required(),
                readings(&[
                    ("FS_GCS_ENABLE", Int8, 5.0, 1),
                    ("RC_OVERRIDE_TIME", Real32, 3.0, 1),
                ]),
                Some("RC_OVERRIDE_TIME is 3, the config requires 0.5"),
            ),
            (
                required(),
                readings(&with(("FS_GCS_ENABLE", Int8, 5.0, 31))),
                Some("FS_GCS_ENABLE was last read 31 s ago"),
            ),
            // The right value in the wrong type is not the parameter the config named.
            (
                required(),
                readings(&with(("FS_GCS_ENABLE", Int16, 5.0, 1))),
                Some("FS_GCS_ENABLE is reported as Int16, the config requires Int8"),
            ),
            (
                vec![],
                readings(&right),
                Some("no failsafe parameters are required in the config"),
            ),
        ];
        for (required, readings, expected) in cases {
            assert_eq!(
                failure(&required, &readings).as_deref(),
                expected,
                "{readings:?}"
            );
        }
    }

    #[test]
    fn the_parameter_read_longest_ago_is_asked_for_once_it_is_due() {
        use ParamType::{Int8, Real32};
        let required = required();
        let due = |list| next_to_read(&required, &readings(list));
        assert_eq!(due(&[]), Some("FS_GCS_ENABLE"));
        assert_eq!(
            due(&[("FS_GCS_ENABLE", Int8, 5.0, 1)]),
            Some("RC_OVERRIDE_TIME")
        );
        assert_eq!(
            due(&[
                ("FS_GCS_ENABLE", Int8, 5.0, 1),
                ("RC_OVERRIDE_TIME", Real32, 0.5, 2)
            ]),
            None
        );
        assert_eq!(
            due(&[
                ("FS_GCS_ENABLE", Int8, 5.0, 11),
                ("RC_OVERRIDE_TIME", Real32, 0.5, 14)
            ]),
            Some("RC_OVERRIDE_TIME")
        );
    }

    #[test]
    fn a_bit_mask_needs_its_bits_set_and_ignores_the_others() {
        use ParamType::Int32;
        let options = |mask| {
            vec![Required {
                name: "FS_OPTIONS".to_owned(),
                kind: Int32,
                value: mask,
                compare: Compare::BitsSet,
            }]
        };
        let said = |value| readings(&[("FS_OPTIONS", Int32, value, 1)]);
        // (required mask, reported value, expected)
        let cases = [
            (2.0, 2.0, None),
            (2.0, 15.0, None),
            (
                2.0,
                13.0,
                Some("FS_OPTIONS is 13, the config requires bits 2 set"),
            ),
            (
                2.0,
                0.0,
                Some("FS_OPTIONS is 0, the config requires bits 2 set"),
            ),
            (
                6.0,
                2.0,
                Some("FS_OPTIONS is 2, the config requires bits 6 set"),
            ),
            (6.0, 7.0, None),
            (
                2.0,
                2.5,
                Some("FS_OPTIONS is 2.5, the config requires bits 2 set"),
            ),
            (
                2.0,
                -1.0,
                Some("FS_OPTIONS is -1, the config requires bits 2 set"),
            ),
        ];
        for (mask, value, expected) in cases {
            assert_eq!(
                failure(&options(mask), &said(value)).as_deref(),
                expected,
                "mask {mask}, reading {value}"
            );
        }
        assert!(validate(&options(0.0)).is_err());
        assert!(validate(&options(2.5)).is_err());
        assert!(validate(&options(2.0)).is_ok());
    }

    #[test]
    fn missions_in_real_mode_need_the_failsafe_and_a_fence_the_gate_can_trust() {
        let make = |name: &str, kind, value, compare| Required {
            name: name.to_owned(),
            kind,
            value,
            compare,
        };
        let valid = || {
            vec![
                make("FS_OPTIONS", ParamType::Int32, 3.0, Compare::BitsSet),
                make("FS_OPTIONS", ParamType::Int32, 16.0, Compare::BitsClear),
                make("FENCE_ENABLE", ParamType::Int8, 1.0, Compare::Equal),
                make("FENCE_ACTION", ParamType::Int8, 1.0, Compare::Equal),
                make("FENCE_TYPE", ParamType::Int8, 7.0, Compare::BitsSet),
                make("FENCE_ALT_MAX", ParamType::Real32, 100.0, Compare::AtMost),
                make("BATT_FS_LOW_ACT", ParamType::Int8, 1.0, Compare::AtLeast),
            ]
        };
        let with = |index: usize, value: f32, compare: Compare| {
            let mut all = valid();
            all[index].value = value;
            all[index].compare = compare;
            all
        };
        let without = |index: usize| {
            let mut all = valid();
            all.remove(index);
            all
        };
        let cases = [
            (valid(), true),
            (with(0, 15.0, Compare::BitsSet), true),
            (with(4, 3.0, Compare::BitsSet), true),
            (with(5, 120.0, Compare::AtMost), true),
            // the GCS failsafe alone: a lost link stops the sticks, which is the radio failsafe
            (with(0, 2.0, Compare::BitsSet), false),
            (with(0, 1.0, Compare::BitsSet), false),
            (with(0, 3.0, Compare::Equal), false),
            (without(1), false),
            (with(1, 8.0, Compare::BitsClear), false),
            (with(2, 0.0, Compare::Equal), false),
            (with(3, 0.0, Compare::Equal), false),
            (with(3, 4.0, Compare::Equal), false),
            (without(4), false),
            (with(4, 1.0, Compare::BitsSet), false),
            (with(4, 2.0, Compare::BitsSet), false),
            (without(5), false),
            (with(5, 121.0, Compare::AtMost), false),
            (with(5, 100.0, Compare::Equal), false),
            (with(6, 2.0, Compare::AtLeast), true),
            (with(6, 0.0, Compare::AtLeast), false),
            (with(6, 1.0, Compare::Equal), false),
            (without(6), false),
            (vec![], false),
        ];
        for (required, expected) in cases {
            assert_eq!(
                validate_missions(&required).is_ok(),
                expected,
                "{required:?}"
            );
        }
    }

    #[test]
    fn a_limit_and_a_clear_mask_compare_the_reading() {
        use ParamType::{Int8, Int32, Real32};
        let one = |name: &str, kind, value, compare| {
            vec![Required {
                name: name.to_owned(),
                kind,
                value,
                compare,
            }]
        };
        let said = |name, kind, value| readings(&[(name, kind, value, 1)]);
        // (requirement, reported value, expected)
        let cases: Vec<(Vec<Required>, f32, Option<&str>)> = vec![
            (
                one("FS_OPTIONS", Int32, 16.0, Compare::BitsClear),
                15.0,
                None,
            ),
            (
                one("FS_OPTIONS", Int32, 16.0, Compare::BitsClear),
                31.0,
                Some("FS_OPTIONS is 31, the config requires bits 16 clear"),
            ),
            (
                one("FENCE_ALT_MAX", Real32, 120.0, Compare::AtMost),
                100.0,
                None,
            ),
            (
                one("FENCE_ALT_MAX", Real32, 120.0, Compare::AtMost),
                120.0,
                None,
            ),
            (
                one("FENCE_ALT_MAX", Real32, 120.0, Compare::AtMost),
                150.0,
                Some("FENCE_ALT_MAX is 150, the config requires at most 120"),
            ),
        ];
        for (required, value, expected) in cases {
            let name = match required[0].kind {
                Int32 => "FS_OPTIONS",
                Int8 => "BATT_FS_LOW_ACT",
                _ => "FENCE_ALT_MAX",
            };
            let kind = required[0].kind;
            assert_eq!(
                failure(&required, &said(name, kind, value)).as_deref(),
                expected,
                "{name} {value}"
            );
        }
        let mask = one("FS_OPTIONS", Int32, 0.0, Compare::BitsClear);
        assert!(validate(&mask).is_err());
    }
}
