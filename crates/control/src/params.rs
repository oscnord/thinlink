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

/// One parameter, the type the autopilot reports it as and the value it must have.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Required {
    pub name: String,
    #[serde(rename = "type")]
    pub kind: ParamType,
    pub value: f32,
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
    let bad = |r: &Required| r.name.is_empty() || r.name.len() > MAX_NAME || !r.value.is_finite();
    match required.iter().find(|r| bad(r)) {
        Some(r) => Err(format!(
            "`{}` is not a parameter name of 1 to {MAX_NAME} characters with a finite value",
            r.name
        )),
        None => Ok(()),
    }
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
    if (reading.value - required.value).abs() > TOLERANCE {
        return Some(format!(
            "{name} is {}, the config requires {}",
            reading.value, required.value
        ));
    }
    None
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
}
