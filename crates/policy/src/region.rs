use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Instant;

use serde::{Deserialize, Serialize};

use crate::follow::{Lost, Order};
use crate::kind::Kind;

/// Narrowest side a rectangle may have, as a fraction of the picture.
const MIN_SIDE: f32 = 0.01;

/// A rectangle as fractions of the picture, so it means the same part of the picture at any size.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NormRect {
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
}

/// A rectangle in pixels, half open, inside the frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PixelRect {
    pub x0: usize,
    pub y0: usize,
    pub x1: usize,
    pub y1: usize,
}

/// A region and the vehicle's monotonic time (ms) it was set or seen.
#[derive(Debug, Clone, Copy)]
pub struct Timed {
    pub rect: NormRect,
    pub at_ms: u64,
}

/// The default keeps nothing active: regional video is off until the limits are configured.
#[derive(Debug, Default, Clone, Copy, Deserialize)]
pub struct RegionLimits {
    pub pilot_ttl_ms: u64,
    pub detection_ttl_ms: u64,
    pub max_regions: usize,
    pub max_area_percent: u8,
}

impl NormRect {
    /// The rectangle between two corners given in any order, clamped to the picture. `None` when a
    /// corner is not finite or a side is under 1 % of the picture.
    pub fn new(xa: f32, ya: f32, xb: f32, yb: f32) -> Option<Self> {
        if ![xa, ya, xb, yb].iter().all(|v| v.is_finite()) {
            return None;
        }
        let (xa, xb) = (xa.clamp(0.0, 1.0), xb.clamp(0.0, 1.0));
        let (ya, yb) = (ya.clamp(0.0, 1.0), yb.clamp(0.0, 1.0));
        let rect = Self {
            x0: xa.min(xb),
            y0: ya.min(yb),
            x1: xa.max(xb),
            y1: ya.max(yb),
        };
        (rect.x1 - rect.x0 >= MIN_SIDE && rect.y1 - rect.y0 >= MIN_SIDE).then_some(rect)
    }

    /// A box around a point: `radius_of_width` of the picture's width either side, and that times
    /// `aspect` (width over height) of the picture's height, so it is square on screen.
    pub fn around(x: f32, y: f32, radius_of_width: f32, aspect: f32) -> Option<Self> {
        let ry = radius_of_width * aspect;
        Self::new(x - radius_of_width, y - ry, x + radius_of_width, y + ry)
    }

    pub fn area(&self) -> f32 {
        (self.x1 - self.x0) * (self.y1 - self.y0)
    }

    /// Intersection over union with `other`: 0 when apart, 1 when equal.
    pub fn overlap(&self, other: &Self) -> f32 {
        let width = (self.x1.min(other.x1) - self.x0.max(other.x0)).max(0.0);
        let height = (self.y1.min(other.y1) - self.y0.max(other.y0)).max(0.0);
        let shared = width * height;
        shared / (self.area() + other.area() - shared)
    }

    /// The pixels that cover the rectangle in a `width` x `height` frame.
    pub fn pixels(&self, width: usize, height: usize) -> PixelRect {
        PixelRect {
            x0: scale(self.x0, width, f32::floor),
            y0: scale(self.y0, height, f32::floor),
            x1: scale(self.x1, width, f32::ceil),
            y1: scale(self.y1, height, f32::ceil),
        }
    }
}

#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    reason = "a fraction in 0..=1 of a frame side, rounded and clamped to the side"
)]
fn scale(fraction: f32, side: usize, round: fn(f32) -> f32) -> usize {
    (round(fraction * side as f32) as usize).min(side)
}

/// The pilot's region and the people the detector found, shared between the vehicle's control loop,
/// its detector and its sender. Clones share all of it and the clock: `Timed::at_ms` and `now_ms`
/// count from the cell's creation, so every side of `active` reads the same monotonic time. The
/// regions the sender favours and the pilot's box the control loop confirms both come from
/// here, so they cannot disagree.
#[derive(Debug, Clone)]
pub struct RegionCell {
    pilot: Arc<Mutex<Option<Timed>>>,
    detections: Arc<Mutex<Vec<(Kind, Timed)>>>,
    limits: Arc<Mutex<RegionLimits>>,
    follow: Arc<Mutex<Follow>>,
    origin: Instant,
}

/// The follow of the pilot's box: the box that began it, where the target is now and how follows
/// have ended. Off unless `RegionCell::enable_follow` was called, and then the pilot's box is
/// only where the follow starts: what is favoured and confirmed is where the target is now.
#[derive(Debug, Default)]
struct Follow {
    enabled: bool,
    /// Counts the boxes the pilot has drawn, so the tracker knows a new one from a repeat.
    generation: u64,
    seed: Option<NormRect>,
    /// None when nothing is followed: no box, or the follow ended.
    now: Option<Timed>,
    lost: u64,
    last_lost: Option<Lost>,
}

impl Follow {
    /// Starts following `region` unless it is the box already followed (the ground repeats it).
    fn begin(&mut self, region: Timed) {
        if !self.enabled || self.seed == Some(region.rect) {
            return;
        }
        self.generation += 1;
        self.seed = Some(region.rect);
        self.now = Some(region);
        self.last_lost = None;
    }

    fn end(&mut self) {
        self.seed = None;
        self.now = None;
    }
}

impl Default for RegionCell {
    fn default() -> Self {
        Self {
            pilot: Arc::default(),
            detections: Arc::default(),
            limits: Arc::default(),
            follow: Arc::default(),
            origin: Instant::now(),
        }
    }
}

fn locked<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

impl RegionCell {
    pub fn now_ms(&self) -> u64 {
        u64::try_from(self.origin.elapsed().as_millis()).unwrap_or(u64::MAX)
    }

    pub fn configure(&self, limits: RegionLimits) {
        *locked(&self.limits) = limits;
    }

    pub fn set(&self, region: Timed) {
        *locked(&self.pilot) = Some(region);
        locked(&self.follow).begin(region);
    }

    pub fn clear(&self) {
        *locked(&self.pilot) = None;
        locked(&self.follow).end();
    }

    /// From now on the pilot's box starts a follow (see `Follow`).
    pub fn enable_follow(&self) {
        locked(&self.follow).enabled = true;
    }

    /// What the tracker is to follow: none when nothing is, or the follow ended.
    pub fn follow_order(&self) -> Option<Order> {
        let follow = locked(&self.follow);
        follow.now?;
        Some(Order {
            generation: follow.generation,
            seed: follow.seed?,
        })
    }

    /// The target of request `generation` is at `rect` in the frame taken at `at_ms`. Ignored if
    /// the pilot has since drawn another box or cleared it.
    pub fn follow_moved(&self, generation: u64, rect: NormRect, at_ms: u64) {
        let mut follow = locked(&self.follow);
        if follow.generation == generation && follow.now.is_some() {
            follow.now = Some(Timed { rect, at_ms });
        }
    }

    /// Request `generation` ended: the target is lost. Ignored like `follow_moved`.
    pub fn follow_lost(&self, generation: u64, why: Lost) {
        let mut follow = locked(&self.follow);
        if follow.generation != generation || follow.now.is_none() {
            return;
        }
        follow.now = None;
        follow.lost += 1;
        follow.last_lost = Some(why);
    }

    /// How many follows ended lost, and why the last did.
    pub fn follow_lost_count(&self) -> (u64, Option<Lost>) {
        let follow = locked(&self.follow);
        (follow.lost, follow.last_lost)
    }

    /// Where the target is now, if the tracker has said so within `detection_ttl_ms`.
    pub fn followed(&self) -> Option<NormRect> {
        let now = locked(&self.follow).now?;
        let ttl = locked(&self.limits).detection_ttl_ms;
        (self.now_ms().saturating_sub(now.at_ms) <= ttl).then_some(now.rect)
    }

    /// The box the vehicle confirms to the ground: where the target is now when following, else
    /// the pilot's box the sender favours.
    pub fn confirmed(&self) -> Option<NormRect> {
        if locked(&self.follow).enabled {
            return self.followed();
        }
        self.pilot_in_use()
    }

    pub fn latest(&self) -> Option<Timed> {
        *locked(&self.pilot)
    }

    /// Replaces the detections with `found`, all seen at `at_ms`.
    pub fn publish_detections(
        &self,
        found: impl IntoIterator<Item = (Kind, NormRect)>,
        at_ms: u64,
    ) {
        *locked(&self.detections) = found
            .into_iter()
            .map(|(kind, rect)| (kind, Timed { rect, at_ms }))
            .collect();
    }

    /// The regions to favour now (see [`active`]).
    pub fn regions(&self) -> Vec<NormRect> {
        self.active_with(self.pilot_box())
    }

    /// The pilot's box as the sender sees it: where the target is now when following.
    fn pilot_box(&self) -> Option<Timed> {
        let follow = locked(&self.follow);
        if follow.enabled {
            return follow.now;
        }
        drop(follow);
        self.latest()
    }

    /// The detections that are active on their own, with their kind: the ones the sender outlines.
    /// Overlapping boxes were merged by the detector, so a rectangle names one kind.
    pub fn detected(&self) -> Vec<(Kind, NormRect)> {
        let kept = self.active_with(None);
        let found = locked(&self.detections);
        kept.into_iter()
            .filter_map(|rect| {
                let (kind, _) = found.iter().find(|(_, t)| t.rect == rect)?;
                Some((*kind, rect))
            })
            .collect()
    }

    /// The pilot's box if the sender keeps it sharp now: fresh, and not dropped for the area or the
    /// number of regions. This, not the box last received, is what the vehicle confirms.
    pub fn pilot_in_use(&self) -> Option<NormRect> {
        let pilot = self.pilot_box()?.rect;
        self.regions().first().copied().filter(|r| *r == pilot)
    }

    fn active_with(&self, pilot: Option<Timed>) -> Vec<NormRect> {
        let detections: Vec<Timed> = locked(&self.detections).iter().map(|(_, t)| *t).collect();
        let limits = *locked(&self.limits);
        active(pilot, &detections, self.now_ms(), &limits)
    }
}

/// The regions to favour: the pilot's first while it is at most `pilot_ttl_ms` old (a TTL of 0
/// means there is no pilot box), then the
/// detections at most `detection_ttl_ms` old, largest first, `max_regions` in all. Empty when
/// together they cover more than `max_area_percent` of the picture, since favouring them then
/// gains nothing.
pub fn active(
    pilot: Option<Timed>,
    detections: &[Timed],
    now_ms: u64,
    limits: &RegionLimits,
) -> Vec<NormRect> {
    let fresh = |t: &Timed, ttl_ms: u64| now_ms.saturating_sub(t.at_ms) <= ttl_ms;
    let mut found: Vec<NormRect> = detections
        .iter()
        .filter(|t| fresh(t, limits.detection_ttl_ms))
        .map(|t| t.rect)
        .collect();
    found.sort_by(|a, b| b.area().total_cmp(&a.area()));
    let regions: Vec<NormRect> = pilot
        .filter(|t| limits.pilot_ttl_ms > 0 && fresh(t, limits.pilot_ttl_ms))
        .map(|t| t.rect)
        .into_iter()
        .chain(found)
        .take(limits.max_regions)
        .collect();
    let covered: f32 = regions.iter().map(NormRect::area).sum();
    if covered * 100.0 > f32::from(limits.max_area_percent) {
        return Vec::new();
    }
    regions
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIMITS: RegionLimits = RegionLimits {
        pilot_ttl_ms: 3000,
        detection_ttl_ms: 1000,
        max_regions: 4,
        max_area_percent: 50,
    };

    fn rect(w: f32, h: f32) -> NormRect {
        NormRect {
            x0: 0.0,
            y0: 0.0,
            x1: w,
            y1: h,
        }
    }

    fn at(rect: NormRect, at_ms: u64) -> Timed {
        Timed { rect, at_ms }
    }

    #[test]
    fn new_normalises_and_clamps() {
        assert_eq!(
            NormRect::new(0.9, 0.8, 0.2, -0.1),
            Some(NormRect {
                x0: 0.2,
                y0: 0.0,
                x1: 0.9,
                y1: 0.8
            })
        );
    }

    #[test]
    fn too_small_is_none() {
        assert_eq!(NormRect::new(0.5, 0.5, 0.505, 0.9), None);
        assert_eq!(NormRect::new(0.5, 0.5, f32::NAN, 0.9), None);
    }

    #[test]
    fn around_a_point() {
        let r = NormRect::around(0.5, 0.5, 0.1, 16.0 / 9.0).unwrap();
        assert!((r.x0 - 0.4).abs() < 1e-6 && (r.x1 - 0.6).abs() < 1e-6);
        assert!((r.y0 - 0.3222).abs() < 1e-3 && (r.y1 - 0.6778).abs() < 1e-3);
    }

    #[test]
    fn active_picks_by_age_size_and_area() {
        let pilot = rect(0.2, 0.2);
        let (big, small) = (rect(0.2, 0.15), rect(0.1, 0.1));
        let two = RegionLimits {
            max_regions: 2,
            ..LIMITS
        };
        // (name, now, pilot, detections, limits, expected)
        let cases = [
            (
                "pilot and the larger detection",
                3000,
                Some(at(pilot, 0)),
                vec![at(small, 2500), at(big, 2500)],
                two,
                vec![pilot, big],
            ),
            (
                "a pilot ttl of 0 means no pilot box",
                0,
                Some(at(pilot, 0)),
                vec![at(big, 0)],
                RegionLimits {
                    pilot_ttl_ms: 0,
                    ..LIMITS
                },
                vec![big],
            ),
            (
                "pilot too old",
                3001,
                Some(at(pilot, 0)),
                vec![at(big, 3000)],
                LIMITS,
                vec![big],
            ),
            (
                "detection too old",
                4000,
                Some(at(pilot, 3000)),
                vec![at(big, 2999)],
                LIMITS,
                vec![pilot],
            ),
            (
                "too much of the picture",
                3000,
                Some(at(rect(0.6, 1.0), 3000)),
                vec![],
                LIMITS,
                vec![],
            ),
        ];
        for (name, now, pilot, detections, limits, expected) in cases {
            assert_eq!(active(pilot, &detections, now, &limits), expected, "{name}");
        }
    }

    #[test]
    fn the_pilot_box_in_use_is_the_one_active_keeps() {
        let pilot = rect(0.2, 0.2);
        let crowded = vec![rect(0.5, 0.5), rect(0.6, 0.5)];
        // (name, limits, pilot, detections, expected)
        let cases = [
            ("fits", LIMITS, Some(pilot), vec![], Some(pilot)),
            ("none received", LIMITS, None, vec![], None),
            (
                "off by default",
                RegionLimits::default(),
                Some(pilot),
                vec![],
                None,
            ),
            (
                "over the area alone",
                LIMITS,
                Some(rect(0.8, 0.8)),
                vec![],
                None,
            ),
            (
                "detections push it over",
                LIMITS,
                Some(pilot),
                crowded,
                None,
            ),
        ];
        for (name, limits, received, detections, expected) in cases {
            let cell = RegionCell::default();
            cell.configure(limits);
            if let Some(rect) = received {
                cell.set(at(rect, cell.now_ms()));
            }
            cell.publish_detections(
                detections.into_iter().map(|r| (Kind::Person, r)),
                cell.now_ms(),
            );
            assert_eq!(cell.pilot_in_use(), expected, "{name}");
        }
    }

    #[test]
    fn detections_keep_their_kind_and_only_the_active_ones_are_outlined() {
        let (big, small) = (rect(0.2, 0.15), rect(0.1, 0.1));
        let cell = RegionCell::default();
        cell.configure(RegionLimits {
            max_regions: 1,
            ..LIMITS
        });
        cell.publish_detections([(Kind::Person, small), (Kind::Car, big)], cell.now_ms());
        assert_eq!(cell.detected(), vec![(Kind::Car, big)]);
    }

    #[test]
    fn a_follow_starts_with_the_pilot_box_and_a_repeat_or_a_lost_one_does_not_restart_it() {
        let cell = RegionCell::default();
        cell.configure(LIMITS);
        cell.enable_follow();
        let (drawn, moved) = (rect(0.2, 0.2), rect(0.3, 0.2));
        cell.set(at(drawn, cell.now_ms()));
        let order = cell.follow_order().unwrap();
        assert_eq!((order.generation, order.seed), (1, drawn));
        assert_eq!(cell.confirmed(), Some(drawn));

        cell.follow_moved(order.generation, moved, cell.now_ms());
        cell.set(at(drawn, cell.now_ms()));
        assert_eq!(cell.follow_order().map(|o| o.generation), Some(1));
        assert_eq!(cell.confirmed(), Some(moved));
        assert_eq!(cell.regions(), vec![moved]);

        cell.follow_lost(order.generation, Lost::Score);
        cell.set(at(drawn, cell.now_ms()));
        assert_eq!(cell.follow_order(), None);
        assert_eq!(cell.confirmed(), None);
        assert_eq!(cell.regions(), vec![]);
        assert_eq!(cell.follow_lost_count(), (1, Some(Lost::Score)));

        cell.clear();
        cell.set(at(drawn, cell.now_ms()));
        assert_eq!(cell.follow_order().map(|o| o.generation), Some(2));
        cell.follow_moved(order.generation, moved, cell.now_ms());
        assert_eq!(
            cell.confirmed(),
            Some(drawn),
            "a report for the old box is ignored"
        );
    }
}
