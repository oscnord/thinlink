//! The autopilot's newest position, read from the telemetry the bridge already forwards.

use std::sync::{Mutex, PoisonError};
use std::time::Instant;

use mavlink::MessageData;
use mavlink::dialects::ardupilotmega::{GLOBAL_POSITION_INT_DATA, MavMessage};

use thinlink_policy::position::{self, Position};

use crate::frame::Frame;

/// `GLOBAL_POSITION_INT`'s `hdg` when the heading is unknown.
const HEADING_UNKNOWN: u16 = u16::MAX;

#[derive(Debug, Default)]
pub struct NewestPosition(Mutex<Option<(Instant, Position)>>);

impl NewestPosition {
    /// Keeps the position `frame` carries, if it is the autopilot's `GLOBAL_POSITION_INT`.
    /// `autopilot` is the heartbeat address learned so far; frames from anyone else are ignored.
    pub fn note(&self, frame: &Frame, autopilot: Option<(u8, u8)>, now: Instant) {
        if frame.message_id() != GLOBAL_POSITION_INT_DATA::ID {
            return;
        }
        let from = u8::try_from(frame.system_id())
            .ok()
            .zip(Some(frame.component_id()));
        if from.is_none() || from != autopilot {
            return;
        }
        let Some(position) = frame.decode().ok().as_ref().and_then(position_of) else {
            return;
        };
        *self.0.lock().unwrap_or_else(PoisonError::into_inner) = Some((now, position));
    }

    /// The newest position, or nothing when it is older than `position::MAX_AGE`.
    pub fn fresh(&self, now: Instant) -> Option<Position> {
        position::fresh(*self.0.lock().unwrap_or_else(PoisonError::into_inner), now)
    }

    /// Forgets the position: the autopilot is gone, and its last word with it.
    pub fn forget(&self) {
        *self.0.lock().unwrap_or_else(PoisonError::into_inner) = None;
    }
}

/// The position a message carries. Latitude and longitude both zero is the autopilot's "no fix".
fn position_of(message: &MavMessage) -> Option<Position> {
    let MavMessage::GLOBAL_POSITION_INT(m) = message else {
        return None;
    };
    if m.lat == 0 && m.lon == 0 {
        return None;
    }
    Some(Position {
        lat_e7: m.lat,
        lon_e7: m.lon,
        rel_alt_mm: m.relative_alt,
        heading_cdeg: (m.hdg != HEADING_UNKNOWN).then_some(m.hdg),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::encoded;

    fn global(lat: i32, lon: i32, hdg: u16) -> MavMessage {
        MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
            lat,
            lon,
            relative_alt: 15_000,
            hdg,
            ..GLOBAL_POSITION_INT_DATA::default()
        })
    }

    #[test]
    fn only_the_autopilots_fix_is_kept_and_it_ages_out() {
        let t0 = Instant::now();
        let newest = NewestPosition::default();
        let autopilot = Some((1, 1));
        let fix = encoded(&global(590_000_000, 180_000_000, 9000));
        newest.note(&fix, None, t0);
        newest.note(&fix, Some((1, 2)), t0);
        assert_eq!(newest.fresh(t0), None, "from no autopilot we know");
        newest.note(&encoded(&global(0, 0, 9000)), autopilot, t0);
        assert_eq!(newest.fresh(t0), None, "no fix");
        newest.note(&fix, autopilot, t0);
        let kept = newest.fresh(t0 + position::MAX_AGE);
        assert_eq!(kept.map(|p| p.rel_alt_mm), Some(15_000));
        assert_eq!(kept.and_then(|p| p.heading_cdeg), Some(9000));
        assert_eq!(newest.fresh(t0 + position::MAX_AGE * 2), None);
        newest.note(
            &encoded(&global(590_000_000, 180_000_000, u16::MAX)),
            autopilot,
            t0,
        );
        assert_eq!(newest.fresh(t0).map(|p| p.heading_cdeg), Some(None));
    }
}
