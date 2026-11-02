//! People the drone's detector saw while the ground was out of reach: kept in a bounded list,
//! repeats merged, and told to the pilot as one short line each when the link returns.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::position::Position;
use crate::region::NormRect;

/// The most sightings kept; the oldest go first. A person who stays in view is one sighting, so
/// this is hundreds of separate people or passes, and the list stays a few tens of kilobytes.
pub const MAX_SIGHTINGS: usize = 256;
/// A detection joins a sighting last seen no longer ago than this.
pub const MERGE_WITHIN_MS: u64 = 2000;
/// ... and whose box centre is no further than this away, in fractions of the picture's width and
/// height.
pub const MERGE_DISTANCE: f32 = 0.15;
/// `STATUSTEXT` carries 50 characters.
pub const MAX_TEXT: usize = 50;

/// A person (or the same person, seen again and again) and where the drone was when it first saw
/// them.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sighting {
    /// Capture time of the first and the last detection merged into this, UNIX milliseconds.
    pub first_ms: u64,
    pub last_ms: u64,
    /// The drone's position, altitude and heading at the first detection.
    pub position: Position,
    /// The box at the last detection.
    pub rect: NormRect,
    /// The best score of the detections merged.
    pub score: f32,
    /// Already told to the pilot as a `STATUSTEXT`.
    pub sent: bool,
}

/// One detection to record.
#[derive(Debug, Clone, Copy)]
pub struct Seen {
    pub capture_ms: u64,
    pub position: Position,
    pub rect: NormRect,
    pub score: f32,
}

/// The bounded list of sightings, oldest first.
#[derive(Debug, Default)]
pub struct Sightings(VecDeque<Sighting>);

impl Sightings {
    /// Records `seen`: into the sighting it repeats, else as a new one.
    pub fn record(&mut self, seen: Seen) {
        match self.0.iter_mut().rev().find(|s| repeats(s, &seen)) {
            Some(known) => {
                known.last_ms = seen.capture_ms;
                known.rect = seen.rect;
                known.score = known.score.max(seen.score);
            }
            None => self.add(seen),
        }
    }

    fn add(&mut self, seen: Seen) {
        if self.0.len() == MAX_SIGHTINGS {
            self.0.pop_front();
        }
        self.0.push_back(Sighting {
            first_ms: seen.capture_ms,
            last_ms: seen.capture_ms,
            position: seen.position,
            rect: seen.rect,
            score: seen.score,
            sent: false,
        });
    }

    /// Offers the oldest sighting not yet sent to `send`; it counts as sent when `send` says it
    /// was.
    pub fn send_next(&mut self, send: impl FnOnce(&Sighting) -> bool) {
        let Some(next) = self.0.iter_mut().find(|s| !s.sent) else {
            return;
        };
        next.sent = send(next);
    }

    /// The sightings first seen at or after `from_ms`, oldest first, at most `limit`.
    pub fn from(&self, from_ms: u64, limit: usize) -> Vec<Sighting> {
        self.0
            .iter()
            .filter(|s| s.first_ms >= from_ms)
            .take(limit)
            .copied()
            .collect()
    }
}

/// Whether `seen` is the person `known` already is: seen shortly after it was last seen, close by
/// in the picture, and not in the very frame that made it (two people stand in one frame).
fn repeats(known: &Sighting, seen: &Seen) -> bool {
    let after = seen.capture_ms.saturating_sub(known.last_ms);
    seen.capture_ms > known.last_ms
        && after <= MERGE_WITHIN_MS
        && centre_distance(&known.rect, &seen.rect) <= MERGE_DISTANCE
}

fn centre_distance(a: &NormRect, b: &NormRect) -> f32 {
    let centre = |r: &NormRect| (r.x0.midpoint(r.x1), r.y0.midpoint(r.y1));
    let ((ax, ay), (bx, by)) = (centre(a), centre(b));
    (ax - bx).hypot(ay - by)
}

/// The sighting as one `STATUSTEXT` line: `person 59.32931 18.06862 12m 14:02:31`, the position
/// to a metre or so, the altitude in whole metres above home and the UTC time of day it was first
/// seen.
pub fn status_text(s: &Sighting) -> String {
    let p = &s.position;
    format!(
        "person {} {} {}m {}",
        degrees(p.lat_e7),
        degrees(p.lon_e7),
        (i64::from(p.rel_alt_mm) + if p.rel_alt_mm < 0 { -500 } else { 500 }) / 1000,
        utc_time(s.first_ms),
    )
}

/// The UTC time of day of a UNIX time in milliseconds, as `14:02:31`.
pub fn utc_time(unix_ms: u64) -> String {
    let seconds = unix_ms / 1000 % 86_400;
    format!(
        "{:02}:{:02}:{:02}",
        seconds / 3600,
        seconds / 60 % 60,
        seconds % 60
    )
}

/// Degrees times 1e7 as decimal degrees to five places, rounded half away from zero.
fn degrees(e7: i32) -> String {
    let e7 = i64::from(e7);
    let rounded = (e7.abs() + 50) / 100;
    let sign = if e7 < 0 && rounded > 0 { "-" } else { "" };
    format!("{sign}{}.{:05}", rounded / 100_000, rounded % 100_000)
}

#[cfg(test)]
mod tests {
    use super::*;

    const POSITION: Position = Position {
        lat_e7: 593_293_149,
        lon_e7: 180_686_150,
        rel_alt_mm: 12_400,
        heading_cdeg: Some(9000),
    };

    fn rect(x: f32, y: f32) -> NormRect {
        NormRect {
            x0: x - 0.05,
            y0: y - 0.1,
            x1: x + 0.05,
            y1: y + 0.1,
        }
    }

    fn seen(capture_ms: u64, x: f32, y: f32) -> Seen {
        Seen {
            capture_ms,
            position: POSITION,
            rect: rect(x, y),
            score: 0.5,
        }
    }

    #[test]
    fn repeats_merge_into_one_sighting_and_others_do_not() {
        let cases = [
            ("the next detection of a person", seen(1500, 0.52, 0.5), 1),
            ("at the time limit", seen(3000, 0.5, 0.5), 1),
            ("too late", seen(3001, 0.5, 0.5), 2),
            ("too far", seen(1500, 0.7, 0.5), 2),
            ("the same frame is another person", seen(1000, 0.5, 0.5), 2),
        ];
        for (name, second, expected) in cases {
            let mut sightings = Sightings::default();
            sightings.record(seen(1000, 0.5, 0.5));
            sightings.record(second);
            assert_eq!(sightings.from(0, usize::MAX).len(), expected, "{name}");
        }
        let mut sightings = Sightings::default();
        for ms in [1000, 2500, 4000] {
            sightings.record(seen(ms, 0.5, 0.5));
        }
        let all = sightings.from(0, 8);
        assert_eq!(
            all.len(),
            1,
            "each repeat is within the limit of the one before"
        );
        assert_eq!((all[0].first_ms, all[0].last_ms), (1000, 4000));
    }

    #[test]
    fn the_list_is_bounded_and_keeps_the_newest() {
        let mut sightings = Sightings::default();
        for i in 0..MAX_SIGHTINGS as u64 + 3 {
            sightings.record(seen(i * 10_000, 0.5, 0.5));
        }
        let all = sightings.from(0, usize::MAX);
        assert_eq!(all.len(), MAX_SIGHTINGS);
        assert_eq!(all[0].first_ms, 30_000);
    }

    #[test]
    fn each_sighting_is_sent_once_oldest_first_and_only_when_sending_worked() {
        let mut sightings = Sightings::default();
        sightings.record(seen(1000, 0.2, 0.5));
        sightings.record(seen(1000, 0.8, 0.5));
        let mut told = Vec::new();
        sightings.send_next(|_| false);
        for _ in 0..3 {
            sightings.send_next(|s| {
                told.push(s.rect.x0);
                true
            });
        }
        assert_eq!(told.len(), 2);
        assert!(told[0] < told[1]);
    }

    #[test]
    fn the_text_rounds_the_position_and_fits_a_status_text() {
        let at = |lat_e7, lon_e7, rel_alt_mm, first_ms| Sighting {
            first_ms,
            last_ms: first_ms,
            position: Position {
                lat_e7,
                lon_e7,
                rel_alt_mm,
                heading_cdeg: None,
            },
            rect: rect(0.5, 0.5),
            score: 0.5,
            sent: false,
        };
        let cases = [
            (
                at(593_293_149, 180_686_150, 12_400, 51_751_000),
                "person 59.32931 18.06862 12m 14:22:31",
            ),
            (
                at(-593_293_150, -5_000, 12_500, 0),
                "person -59.32932 -0.00050 13m 00:00:00",
            ),
            (
                at(-49, 0, -600, 86_399_999),
                "person 0.00000 0.00000 -1m 23:59:59",
            ),
            (
                at(-900_000_000, -1_800_000_000, i32::MIN, 1000),
                "person -90.00000 -180.00000 -2147484m 00:00:01",
            ),
        ];
        for (sighting, expected) in cases {
            let text = status_text(&sighting);
            assert_eq!(text, expected);
            assert!(text.len() <= MAX_TEXT, "{text}");
        }
    }
}
