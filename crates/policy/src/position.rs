//! Where the vehicle was, as the autopilot last said: kept with snapshots, and only while fresh.

use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

/// A position older than this is not used: the autopilot stopped saying, or the bridge to it broke.
pub const MAX_AGE: Duration = Duration::from_secs(2);

/// The autopilot's `GLOBAL_POSITION_INT`, in its units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Position {
    /// Degrees times 1e7, north positive.
    pub lat_e7: i32,
    /// Degrees times 1e7, east positive.
    pub lon_e7: i32,
    /// Millimetres above home.
    pub rel_alt_mm: i32,
    /// Centidegrees from north, `None` when the autopilot does not know.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_cdeg: Option<u16>,
}

/// The newest position and when it was received, if it is no older than [`MAX_AGE`] at `now`.
pub fn fresh(newest: Option<(Instant, Position)>, now: Instant) -> Option<Position> {
    let (at, position) = newest?;
    (now.saturating_duration_since(at) <= MAX_AGE).then_some(position)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_position_is_fresh_for_two_seconds() {
        let t0 = Instant::now();
        let position = Position {
            lat_e7: 590_000_000,
            lon_e7: 180_000_000,
            rel_alt_mm: 12_000,
            heading_cdeg: Some(9000),
        };
        let at = |ms| t0 + Duration::from_millis(ms);
        let cases = [
            ("just received", Some((t0, position)), at(0), Some(position)),
            (
                "at the limit",
                Some((t0, position)),
                at(2000),
                Some(position),
            ),
            ("past the limit", Some((t0, position)), at(2001), None),
            ("never received", None, at(0), None),
        ];
        for (name, newest, now, expected) in cases {
            assert_eq!(fresh(newest, now), expected, "{name}");
        }
    }
}
