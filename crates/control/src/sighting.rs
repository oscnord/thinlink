//! Tells the pilot of the sightings made while the link was gone: one `STATUSTEXT` each, oldest
//! first, from the autopilot's address so a ground station files it under the vehicle.

use std::sync::{Mutex, PoisonError};
use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::dialects::ardupilotmega::{MavMessage, MavSeverity, STATUSTEXT_DATA};

use sat_teleop_policy::sighting::{Sightings, status_text};

use crate::frame::{Frame, datagram_frames};
use crate::telemetry::RateCap;

/// The least time between two texts, so a long list does not scroll the pilot's message list past
/// reading.
const GAP: Duration = Duration::from_millis(500);

#[derive(Debug, Default)]
pub struct Announcer {
    last: Option<Instant>,
    sequence: u8,
}

impl Announcer {
    /// The next sighting's text as a frame to send up, if the ground is heard, the autopilot is
    /// known, the gap is over and the telemetry budget has room for it beside the telemetry. A
    /// sighting that finds no room stays unsent.
    pub fn next(
        &mut self,
        now: Instant,
        autopilot: Option<(u8, u8)>,
        heard: bool,
        sightings: &Mutex<Sightings>,
        cap: &mut RateCap,
    ) -> Option<Frame> {
        let (system, component) = autopilot.filter(|_| heard)?;
        if self
            .last
            .is_some_and(|at| now.saturating_duration_since(at) < GAP)
        {
            return None;
        }
        let mut sent = None;
        sightings
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .send_next(|sighting| {
                let header = MavHeader {
                    system_id: system.into(),
                    component_id: component,
                    sequence: self.sequence,
                };
                let text = status_text(sighting);
                sent = status_frame(header, MavSeverity::MAV_SEVERITY_INFO, &text)
                    .filter(|f| cap.take_bulk(now, f.size()));
                sent.is_some()
            });
        if sent.is_some() {
            self.last = Some(now);
            self.sequence = self.sequence.wrapping_add(1);
        }
        sent
    }
}

/// A `STATUSTEXT` of `severity` as a frame to send up from `header`'s identity.
pub(crate) fn status_frame(header: MavHeader, severity: MavSeverity, text: &str) -> Option<Frame> {
    let message = MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity,
        text: text.into(),
        ..STATUSTEXT_DATA::default()
    });
    let mut bytes = Vec::new();
    mavlink::write_v2_msg(&mut bytes, header, &message).ok()?;
    datagram_frames(&bytes).next()
}

#[cfg(test)]
mod tests {
    use sat_teleop_policy::kind::Kind;
    use sat_teleop_policy::position::Position;
    use sat_teleop_policy::region::NormRect;
    use sat_teleop_policy::sighting::Seen;

    use super::*;
    use crate::telemetry::TelemetryLimits;

    fn list(times: &[u64]) -> Mutex<Sightings> {
        let mut sightings = Sightings::default();
        for (i, &capture_ms) in (0_u8..).zip(times) {
            let x = 0.1 + 0.3 * f32::from(i);
            sightings.record(Seen {
                kind: Kind::Person,
                capture_ms,
                position: Position {
                    lat_e7: 593_293_149,
                    lon_e7: 180_686_150,
                    rel_alt_mm: 12_000,
                    heading_cdeg: None,
                },
                rect: NormRect {
                    x0: x,
                    y0: 0.4,
                    x1: x + 0.1,
                    y1: 0.6,
                },
                score: 0.5,
            });
        }
        Mutex::new(sightings)
    }

    fn new_cap(bytes_per_s: u32) -> RateCap {
        RateCap::new(TelemetryLimits {
            bytes_per_s,
            offered_percent: 200,
            essential_bytes_per_s: 200,
        })
    }

    fn text_of(frame: &Frame) -> String {
        let Ok(MavMessage::STATUSTEXT(t)) = frame.decode() else {
            panic!("not a STATUSTEXT");
        };
        assert_eq!(t.severity, MavSeverity::MAV_SEVERITY_INFO);
        t.text.to_str().unwrap().to_owned()
    }

    #[test]
    fn sightings_go_up_oldest_first_once_each_paced_and_only_with_the_ground_heard() {
        let t0 = Instant::now();
        let at = |ms| t0 + Duration::from_millis(ms);
        let sightings = list(&[1_000, 60_000_000]);
        let mut cap = new_cap(8000);
        let mut announcer = Announcer::default();
        let autopilot = Some((1, 1));
        assert!(
            announcer
                .next(at(0), autopilot, false, &sightings, &mut cap)
                .is_none()
        );
        assert!(
            announcer
                .next(at(0), None, true, &sightings, &mut cap)
                .is_none()
        );
        let first = announcer
            .next(at(0), autopilot, true, &sightings, &mut cap)
            .unwrap();
        assert_eq!(text_of(&first), "person 59.32931 18.06862 12m 00:00:01");
        assert_eq!(first.system_id(), 1);
        assert!(
            announcer
                .next(at(100), autopilot, true, &sightings, &mut cap)
                .is_none(),
            "paced"
        );
        let second = announcer
            .next(at(600), autopilot, true, &sightings, &mut cap)
            .unwrap();
        assert_eq!(text_of(&second), "person 59.32931 18.06862 12m 16:40:00");
        assert!(
            announcer
                .next(at(5_000), autopilot, true, &sightings, &mut cap)
                .is_none()
        );
    }

    #[test]
    fn a_sighting_without_room_in_the_budget_waits() {
        let t0 = Instant::now();
        let sightings = list(&[1_000]);
        let mut cap = new_cap(60);
        let mut announcer = Announcer::default();
        let next = |announcer: &mut Announcer, cap: &mut RateCap, at| {
            announcer.next(at, Some((1, 1)), true, &sightings, cap)
        };
        assert!(
            next(&mut announcer, &mut cap, t0).is_none(),
            "a frame is over half of 60 B/s"
        );
        assert!(next(&mut announcer, &mut cap, t0 + Duration::from_secs(10)).is_none());
        let mut roomy = new_cap(8000);
        assert!(next(&mut announcer, &mut roomy, t0 + Duration::from_secs(11)).is_some());
    }
}
