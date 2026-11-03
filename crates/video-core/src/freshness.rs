//! Whether the ground's video output may be shown as live. The ground passes in what it observed
//! of its source (when frames arrived, which carried a new stamp, the newest one's age, the
//! source's frame interval, how many stamps read back) and gets the state to show. Time arrives as
//! elapsed milliseconds.

use crate::stamp::{CaptureMs, FrameId};

/// New frames count towards the source's frame interval for this long before the newest.
const CADENCE_WINDOW_MS: u32 = 1000;

/// What the output tells the operator about the picture it shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// New frames keep arriving and the newest is younger than the stale threshold.
    Live,
    /// The newest frame is older than the stale threshold.
    Stale,
    /// Frames keep arriving but none is new: the source repeats itself.
    Frozen,
    /// The source is down or nothing has arrived for the disconnect threshold.
    Disconnected,
    /// The age cannot be trusted: too few stamps read back, or no clock bound.
    AgeUnknown,
    /// Live, but the source sends frames too far apart for the stale threshold: the picture is
    /// younger than the low-rate stale threshold and gets that old between frames.
    LowRate,
}

/// The 4-bit codes of the state stamp. Zero and the codes above 6 are unused, so a uniform black
/// or white stamp row never reads as a state.
const CODES: [(State, u8); 6] = [
    (State::Live, 1),
    (State::Stale, 2),
    (State::Frozen, 3),
    (State::Disconnected, 4),
    (State::AgeUnknown, 5),
    (State::LowRate, 6),
];

impl State {
    pub fn code(self) -> u8 {
        CODES
            .iter()
            .find(|(state, _)| *state == self)
            .map_or(0, |(_, code)| *code)
    }

    pub fn from_code(code: u8) -> Option<Self> {
        CODES
            .iter()
            .find(|(_, c)| *c == code)
            .map(|(state, _)| *state)
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Live => "live",
            Self::Stale => "stale",
            Self::Frozen => "frozen",
            Self::Disconnected => "disconnected",
            Self::AgeUnknown => "age_unknown",
            Self::LowRate => "low_rate",
        }
    }
}

/// `[freshness]` of `testbed/limits.toml`, validated by the caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Thresholds {
    /// The newest new frame is older than this: Stale.
    pub stale_after_ms: u64,
    /// No new frame for this long while frames keep arriving: Frozen. At a low rate, one frame
    /// interval longer.
    pub frozen_after_ms: u64,
    /// No frame at all for this long: Disconnected.
    pub disconnected_after_ms: u64,
    /// Below this share of readable stamps among the recent frames the age cannot be trusted.
    pub min_readable_percent: u8,
    /// New frames captured further apart than this are a low rate.
    pub low_rate_interval_ms: u64,
    /// At a low rate, the newest new frame is older than this: Stale.
    pub low_rate_stale_after_ms: u64,
}

impl Thresholds {
    /// The age past which the newest frame is stale: longer at a low rate, whose frames get older
    /// between them.
    fn stale_after_ms(self, frame_interval_ms: Option<u64>) -> u64 {
        if is_low_rate(self, frame_interval_ms) {
            self.low_rate_stale_after_ms
        } else {
            self.stale_after_ms
        }
    }

    /// How long nothing new may come while frames still arrive: at a low rate a whole frame
    /// interval longer, or a source repeating itself at one frame a second would never read as
    /// frozen.
    fn frozen_after_ms(self, frame_interval_ms: Option<u64>) -> u64 {
        match frame_interval_ms.filter(|_| is_low_rate(self, frame_interval_ms)) {
            Some(interval) => self.frozen_after_ms.saturating_add(interval),
            None => self.frozen_after_ms,
        }
    }
}

fn is_low_rate(t: Thresholds, frame_interval_ms: Option<u64>) -> bool {
    frame_interval_ms.is_some_and(|ms| ms > t.low_rate_interval_ms)
}

/// Whether the source pipeline is running (connected to its server).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Link {
    Up,
    Down,
}

/// What the ground observed of its source at one moment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Observed {
    pub link: Link,
    /// Since the last decoded frame of any kind; `None` before the first.
    pub since_frame_ms: Option<u64>,
    /// Since the newest frame with a new stamp arrived; `None` before the first.
    pub since_new_frame_ms: Option<u64>,
    /// The newest new frame's age now: its age when it arrived plus the time since.
    pub age_ms: Option<u64>,
    /// The source's frame interval, [`cadence_ms`] of its new frames.
    pub frame_interval_ms: Option<u64>,
    /// Readable stamps among the `decoded` most recent frames.
    pub readable: u32,
    pub decoded: u32,
    /// How far the vehicle's clock may be from the ground's; `None` when nothing bounds it, and
    /// then no age is trusted.
    pub clock_bound_ms: Option<u64>,
}

/// The state to show. Disconnected comes first (nothing to judge), then an untrustworthy age, then
/// a source repeating itself while frames arrive, then age. An age counts with its clock bound
/// added, so only video provably younger than the stale threshold is Live. A source whose frames
/// are captured further apart than `low_rate_interval_ms` is judged against
/// `low_rate_stale_after_ms` and shown as `LowRate`, never `Live`: its picture ages by a whole
/// frame interval between frames, which would otherwise flicker between Live and Stale.
pub fn classify(t: Thresholds, o: Observed) -> State {
    let arriving_within = |ms: u64| o.since_frame_ms.is_some_and(|since| since <= ms);
    if o.link == Link::Down || !arriving_within(t.disconnected_after_ms) {
        return State::Disconnected;
    }
    let readable_percent = u64::from(o.readable) * 100;
    if readable_percent < u64::from(t.min_readable_percent) * u64::from(o.decoded) {
        return State::AgeUnknown;
    }
    let frozen_after_ms = t.frozen_after_ms(o.frame_interval_ms);
    let nothing_new = o
        .since_new_frame_ms
        .is_none_or(|since| since > frozen_after_ms);
    if nothing_new && arriving_within(frozen_after_ms) {
        return State::Frozen;
    }
    let (Some(age), Some(bound)) = (o.age_ms, o.clock_bound_ms) else {
        return State::AgeUnknown;
    };
    if age.saturating_add(bound) > t.stale_after_ms(o.frame_interval_ms) {
        return State::Stale;
    }
    if is_low_rate(t, o.frame_interval_ms) {
        return State::LowRate;
    }
    State::Live
}

/// How many milliseconds from now `classify` may give another state if no frame arrives: the
/// earliest of the newest frame's age plus the clock bound passing the stale threshold (the
/// low-rate one at a low rate), the time since the newest new frame passing the frozen threshold
/// (frames are still arriving until then: the last frame is at most as old) and the time since the
/// last frame passing `disconnected_after_ms`. `None` when none of them lies ahead.
pub fn next_change_ms(t: Thresholds, o: Observed) -> Option<u64> {
    let until = |limit: u64, elapsed: u64| limit.checked_sub(elapsed).map(|left| left + 1);
    let stale = o.age_ms.zip(o.clock_bound_ms).and_then(|(age, bound)| {
        until(
            t.stale_after_ms(o.frame_interval_ms),
            age.saturating_add(bound),
        )
    });
    let frozen = o
        .since_new_frame_ms
        .and_then(|since| until(t.frozen_after_ms(o.frame_interval_ms), since));
    let disconnected = o
        .since_frame_ms
        .and_then(|since| until(t.disconnected_after_ms, since));
    [stale, frozen, disconnected].into_iter().flatten().min()
}

/// The source's frame interval from its newest new frames' ids and capture times, oldest first: the
/// lower median of the capture time per frame id between consecutive frames whose later frame was
/// captured within a second of the newest. Dividing by the id step keeps the interval the source's
/// own when frames between them were unreadable or lost; a pair whose id does not advance (a
/// vehicle restart, an id wrap) is skipped. One gap among frames at full rate leaves it at the
/// full rate's interval; after a slowdown it reads slow within two slow frames. `None` before two
/// usable frames.
pub fn cadence_ms(frames: &[(FrameId, CaptureMs)]) -> Option<u64> {
    let (_, newest) = frames.last()?;
    let mut intervals: Vec<u32> = frames
        .windows(2)
        .filter_map(|pair| match pair {
            [(earlier_id, earlier), (later_id, later)]
                if newest.0.wrapping_sub(later.0) < CADENCE_WINDOW_MS =>
            {
                let steps = later_id.0.checked_sub(earlier_id.0).filter(|s| *s > 0)?;
                Some(later.0.wrapping_sub(earlier.0) / steps)
            }
            _ => None,
        })
        .collect();
    intervals.sort_unstable();
    let middle = intervals.len().checked_sub(1)? / 2;
    intervals.get(middle).map(|ms| u64::from(*ms))
}

/// When the state of the frame in the encoder and sender stops being true. The ground's output
/// decides a state when it hands a frame over; the frame reaches the network later, and a stall in
/// between (the process descheduled, the machine busy) must not let it go out calling old video
/// live. Times are on one clock, in any unit.
#[derive(Debug, Default)]
pub struct LabelDeadline {
    deadline: Option<u64>,
    /// Whether the frame is late, fixed by the first of its packets so they all share one fate.
    late: Option<bool>,
}

impl LabelDeadline {
    /// A new frame starts, its state holding until `deadline`; `None` when it claims nothing a
    /// delay could make false.
    pub fn arm(&mut self, deadline: Option<u64>) {
        *self = Self {
            deadline,
            late: None,
        };
    }

    /// Whether the frame's state no longer holds at `now`, its first packet being sent.
    pub fn late(&mut self, now: u64) -> bool {
        let deadline = self.deadline;
        *self
            .late
            .get_or_insert_with(|| deadline.is_some_and(|deadline| now > deadline))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIMITS: Thresholds = Thresholds {
        stale_after_ms: 600,
        frozen_after_ms: 500,
        disconnected_after_ms: 2000,
        min_readable_percent: 50,
        low_rate_interval_ms: 150,
        low_rate_stale_after_ms: 1600,
    };
    /// A frame every 67 ms, the newest 330 ms old (GEO), every stamp readable, a shared clock.
    const HEALTHY: Observed = Observed {
        link: Link::Up,
        since_frame_ms: Some(30),
        since_new_frame_ms: Some(30),
        age_ms: Some(330),
        frame_interval_ms: Some(67),
        readable: 15,
        decoded: 15,
        clock_bound_ms: Some(0),
    };
    /// Two frames a second under GEO, just before the next frame: the last arrived 499 ms ago.
    const TWO_FPS: Observed = Observed {
        since_frame_ms: Some(499),
        since_new_frame_ms: Some(499),
        age_ms: Some(829),
        frame_interval_ms: Some(500),
        readable: 2,
        decoded: 2,
        ..HEALTHY
    };

    #[test]
    fn only_new_verifiably_young_frames_are_live() {
        use State::{AgeUnknown, Disconnected, Frozen, Live, LowRate, Stale};
        let frames = |ids_times: &[(u32, u32)]| -> Vec<(FrameId, CaptureMs)> {
            ids_times
                .iter()
                .map(|(id, ms)| (FrameId(*id), CaptureMs(*ms)))
                .collect()
        };
        let gap: Vec<(FrameId, CaptureMs)> = (0..15)
            .map(|i| (FrameId(i), CaptureMs(i * 67)))
            .chain([(FrameId(15), CaptureMs(14 * 67 + 300))])
            .collect();
        // Fifteen frames a second, but only every fifth frame readable.
        let sparse = frames(&[(0, 0), (5, 333), (10, 667), (15, 1000)]);
        let two_fps = frames(&[(0, 0), (1, 500), (2, 1000), (3, 1500)]);
        // (case, observed, state, ms until the state may change without a frame)
        let cases = [
            ("healthy", HEALTHY, Live, Some(271)),
            (
                "at the stale threshold",
                Observed {
                    age_ms: Some(600),
                    ..HEALTHY
                },
                Live,
                Some(1),
            ),
            (
                "older than the stale threshold",
                Observed {
                    age_ms: Some(601),
                    ..HEALTHY
                },
                Stale,
                Some(471),
            ),
            (
                "young, but not within the clock bound",
                Observed {
                    age_ms: Some(550),
                    clock_bound_ms: Some(100),
                    ..HEALTHY
                },
                Stale,
                Some(471),
            ),
            (
                "no clock bound",
                Observed {
                    clock_bound_ms: None,
                    ..HEALTHY
                },
                AgeUnknown,
                Some(471),
            ),
            (
                "too few stamps read back",
                Observed {
                    readable: 7,
                    ..HEALTHY
                },
                AgeUnknown,
                Some(271),
            ),
            (
                "no stamp reads back",
                Observed {
                    since_new_frame_ms: Some(1500),
                    age_ms: Some(1800),
                    readable: 0,
                    ..HEALTHY
                },
                AgeUnknown,
                Some(1971),
            ),
            (
                "frozen source over a healthy transport",
                Observed {
                    since_new_frame_ms: Some(501),
                    age_ms: Some(831),
                    ..HEALTHY
                },
                Frozen,
                Some(1971),
            ),
            (
                "blackout: nothing arrives, the last frame ages",
                Observed {
                    since_frame_ms: Some(900),
                    since_new_frame_ms: Some(900),
                    age_ms: Some(1230),
                    ..HEALTHY
                },
                Stale,
                Some(1101),
            ),
            (
                "blackout past the disconnect threshold",
                Observed {
                    since_frame_ms: Some(2001),
                    since_new_frame_ms: Some(2001),
                    age_ms: Some(2331),
                    ..HEALTHY
                },
                Disconnected,
                None,
            ),
            (
                "source pipeline down",
                Observed {
                    link: Link::Down,
                    ..HEALTHY
                },
                Disconnected,
                Some(271),
            ),
            (
                "two frames a second, just before the next one",
                TWO_FPS,
                LowRate,
                Some(502),
            ),
            (
                "two frames a second, the next one just arrived",
                Observed {
                    since_frame_ms: Some(0),
                    since_new_frame_ms: Some(0),
                    age_ms: Some(330),
                    ..TWO_FPS
                },
                LowRate,
                Some(1001),
            ),
            (
                "one frame a second, just before the next one",
                Observed {
                    since_frame_ms: Some(999),
                    since_new_frame_ms: Some(999),
                    age_ms: Some(1329),
                    frame_interval_ms: Some(1000),
                    readable: 1,
                    decoded: 1,
                    ..HEALTHY
                },
                LowRate,
                Some(272),
            ),
            (
                // A repeat arrived 800 ms ago, the frozen frame 1800 ms ago: frames arrive within
                // the frozen threshold plus the interval, nothing new for longer.
                "one frame a second, repeating itself",
                Observed {
                    since_frame_ms: Some(800),
                    since_new_frame_ms: Some(1800),
                    age_ms: Some(2130),
                    frame_interval_ms: Some(1000),
                    readable: 1,
                    decoded: 1,
                    ..HEALTHY
                },
                Frozen,
                Some(1201),
            ),
            (
                // Fifteen frames a second, the last 300 ms after the one before, then silence.
                "a single gap at full rate, then silence",
                Observed {
                    since_frame_ms: Some(700),
                    since_new_frame_ms: Some(700),
                    age_ms: Some(1030),
                    frame_interval_ms: cadence_ms(&gap),
                    ..HEALTHY
                },
                Stale,
                Some(1301),
            ),
            (
                "a full-rate source with sparse readable frames, then silence",
                Observed {
                    since_frame_ms: Some(700),
                    since_new_frame_ms: Some(700),
                    age_ms: Some(1030),
                    frame_interval_ms: cadence_ms(&sparse),
                    ..HEALTHY
                },
                Stale,
                Some(1301),
            ),
            (
                "a true two frames a second source",
                Observed {
                    frame_interval_ms: cadence_ms(&two_fps),
                    ..TWO_FPS
                },
                LowRate,
                Some(502),
            ),
            (
                "a low rate past its own stale threshold",
                Observed {
                    since_frame_ms: Some(1300),
                    since_new_frame_ms: Some(1300),
                    age_ms: Some(1630),
                    ..TWO_FPS
                },
                Stale,
                Some(701),
            ),
            (
                "full rate as old as a low rate just before its next frame",
                Observed {
                    age_ms: Some(829),
                    ..HEALTHY
                },
                Stale,
                Some(471),
            ),
            (
                "no frame yet",
                Observed {
                    since_frame_ms: None,
                    since_new_frame_ms: None,
                    age_ms: None,
                    frame_interval_ms: None,
                    readable: 0,
                    decoded: 0,
                    ..HEALTHY
                },
                Disconnected,
                None,
            ),
        ];
        for (name, observed, state, change) in cases {
            assert_eq!(classify(LIMITS, observed), state, "{name}");
            assert_eq!(
                next_change_ms(LIMITS, observed),
                change,
                "{name}: next change"
            );
        }
    }

    #[test]
    fn a_frame_is_late_or_not_in_all_its_packets() {
        let mut frame = LabelDeadline::default();
        frame.arm(Some(100));
        assert!(!frame.late(100));
        assert!(!frame.late(150));
        frame.arm(Some(100));
        assert!(frame.late(101));
        assert!(frame.late(50));
        frame.arm(None);
        assert!(!frame.late(u64::MAX));
    }
}
