//! Adapting the live video to the uplink: a ladder of encodings, best first, and a controller that
//! walks it from the relay's feedback. Latency first: the controller keeps the uplink queue empty
//! and only then raises picture quality. Queue delay (the round trip minus the smallest round trip
//! of a recent window) is the main signal and cuts the rate at once, to the rung the measured
//! uplink can carry; loss counts in GCC's bands (draft-ietf-rmcat-gcc-02, section 6), so random
//! loss holds the ladder instead of pulling it down. A rung up needs a healthy dwell and room in
//! the uplink last measured. Time arrives as milliseconds of the vehicle's monotonic clock.

use std::collections::VecDeque;

use thiserror::Error;

use crate::budget::{self, Budget, Usage};
use crate::pace;

/// Answers kept for the windows; at four probes a second the longest window holds far fewer.
const MAX_SAMPLES: usize = 1024;
/// A rung whose climb failed gets its hold back only from an uplink measured at least this share of
/// the one it failed at: the capacities measured at the same 128 kbit/s step spread 12.3-14.0 kB/s,
/// about 7 % around their mean (runs 1793548366795, 1793548686388, 1793548986264, 1793549731777),
/// so less is noise, not a link that grew.
const GROWN_PERCENT: u64 = 110;
/// A climb into a rung the uplink last measured cannot carry sends at most this share of that
/// uplink, and raises it by the same factor after each healthy `min_change_interval_ms`. A queue
/// builds for about the round trip before the controller sees it, so what it peaks at is the
/// round trip times the excess: the whole rung (240 % of the uplink) peaked at 589, 617 and 929 ms
/// (runs 1793588432518, 1793587951607, 1793616684672), 10 % over should stay near 250 ms.
const PROBE_PERCENT: u64 = 110;

/// What a rung is for, best first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Group {
    /// Driving picture.
    Normal,
    /// Usable picture at a reduced resolution or rate.
    Constrained,
    /// Enough to see where the vehicle is, at one or two frames a second.
    AwarenessOnly,
}

impl Group {
    pub fn name(self) -> &'static str {
        match self {
            Self::Normal => "normal",
            Self::Constrained => "constrained",
            Self::AwarenessOnly => "awareness_only",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rung {
    pub width: u16,
    pub height: u16,
    pub fps: u16,
    /// The most video the encoder may send in any one second.
    pub bitrate_kbit: u32,
    pub group: Group,
}

impl Rung {
    /// What the rung costs on the wire with `packets_per_frame` RTP packets per frame.
    pub fn wire_bytes_per_s(self, packets_per_frame: u32) -> u32 {
        budget::video_bytes_per_s(self.bitrate_kbit, self.fps, packets_per_frame)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum LadderError {
    #[error("the ladder has no rungs")]
    Empty,
    #[error("rung {0} does not send less than the rung above it")]
    Unordered(usize),
    #[error("rung {0} is in a better group than the rung above it")]
    Groups(usize),
    #[error("the start rung {0} is not on the ladder")]
    Start(usize),
}

/// Rungs from the best (index 0) to the floor, each sending less than the one above it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ladder(Vec<Rung>);

impl Ladder {
    pub fn new(rungs: Vec<Rung>) -> Result<Self, LadderError> {
        if rungs.is_empty() {
            return Err(LadderError::Empty);
        }
        for (below, pair) in rungs.windows(2).enumerate().map(|(i, p)| (i + 1, p)) {
            let [above, rung] = pair else { continue };
            if rung.bitrate_kbit >= above.bitrate_kbit {
                return Err(LadderError::Unordered(below));
            }
            if rung.group < above.group {
                return Err(LadderError::Groups(below));
            }
        }
        Ok(Self(rungs))
    }

    pub fn rung(&self, index: usize) -> Option<Rung> {
        self.0.get(index).copied()
    }

    /// The index of the lowest rung.
    pub fn floor(&self) -> usize {
        self.0.len().saturating_sub(1)
    }

    /// The best rung from `from` down whose cost fits `bytes_per_s`; the floor if none does.
    fn fitting(&self, from: usize, bytes_per_s: u32, packets_per_frame: u32) -> usize {
        (from..self.0.len())
            .find(|&i| {
                self.rung(i)
                    .is_some_and(|r| r.wire_bytes_per_s(packets_per_frame) <= bytes_per_s)
            })
            .unwrap_or_else(|| self.floor())
    }
}

/// `[adapt]` of `testbed/limits.toml`, validated by the caller. Milliseconds unless named.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Thresholds {
    /// The smallest round trip of this window is the uplink's empty-queue base.
    pub min_rtt_window_ms: u64,
    /// Queue delay above this is a queue: cut while it rises or stands. A single answer more than
    /// twice this late is a deep queue (jitter never comes close): cut without waiting for the
    /// trend.
    pub queue_delay_high_ms: u64,
    /// A cut whose answer is at least this late also pauses the video until the queue has drained.
    pub drain_above_ms: u64,
    /// Queue delay at most this is back at the base: the uplink is healthy.
    pub queue_delay_base_ms: u64,
    /// The span over which the queue delay's trend (its least-squares slope) is taken.
    pub trend_window_ms: u64,
    /// A queue delay changing at least this fast (ms per second) rises or drains; slower, it is
    /// flat.
    pub trend_ms_per_s: u64,
    /// A cut fits rungs into this share of the measured uplink, so the rest drains the queue.
    pub capacity_fit_percent: u32,
    /// Loss is counted over this window, from answers to probes sent since the last change.
    pub loss_window_ms: u64,
    /// Fewer packets than this in the window say nothing about loss.
    pub loss_min_packets: u32,
    /// Loss below this percentage allows a rung up; above `loss_high_percent` cuts one rung; in
    /// between it holds.
    pub loss_low_percent: u32,
    pub loss_high_percent: u32,
    /// No answer for this long, once one came: fall to the floor.
    pub stale_after_ms: u64,
    /// Healthy for this long after a fall that no queue led to: back to the rung before it, or
    /// the best the capacity measured carries.
    pub outage_recovery_ms: u64,
    /// Healthy for this long before a rung up.
    pub dwell_ms: u64,
    /// At most one change this often.
    pub min_change_interval_ms: u64,
    /// The uplink measured at a cut bounds a rung up for this long; then it may probe for more.
    /// Each time a climb to a rung fails (its queue is cut within a dwell), that rung's hold
    /// doubles, counted from that cut, up to `capacity_hold_max_ms`; a later measurement of at
    /// least 10 % more uplink than at the failure resets it.
    pub capacity_hold_ms: u64,
    pub capacity_hold_max_ms: u64,
}

/// One answer from the relay, as the vehicle measured it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Feedback {
    /// When the probe it answers left the vehicle.
    pub probe_sent_ms: u64,
    pub rtt_ms: u64,
    /// Video packets the relay received in its window, and those lost on the way (not those that
    /// only came late).
    pub received: u32,
    pub lost: u32,
    /// Everything the uplink carried from the vehicle, bytes per second on the wire: the video as
    /// the relay received it, the rest as the vehicle sent it, in the share the video arrived in.
    /// `None` until enough was measured.
    pub uplink_bytes_per_s: Option<u32>,
    pub usage: Usage,
    /// What retrieval of retained detail asks to reserve while a job is waiting to send, on the
    /// wire, zero while idle: taken out of the uplink before the video is fitted to it, unless
    /// that would push the video out of the normal rungs.
    pub retrieval: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reason {
    /// The uplink queue rose or stood above the threshold.
    QueueDelay,
    /// Loss above the high band.
    Loss,
    /// No answer for the stale threshold.
    Stale,
    /// Healthy for the dwell.
    Healthy,
    /// A cut for queue delay taken back: the delay did not fall with the rate, so it was the
    /// path's base delay that grew (a route change), not a queue.
    BaseShift,
    /// Healthy again after an outage.
    Recovered,
    /// Healthy for the dwell at a rung held to a ceiling: the ceiling goes up.
    Probe,
}

impl Reason {
    pub fn name(self) -> &'static str {
        match self {
            Self::QueueDelay => "queue_delay",
            Self::Loss => "loss",
            Self::Stale => "stale",
            Self::Healthy => "healthy",
            Self::BaseShift => "base_shift",
            Self::Recovered => "recovered",
            Self::Probe => "probe",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    Hold,
    /// Send this rung from now on (with video, if it was paused).
    Step {
        to: usize,
        reason: Reason,
    },
    /// The uplink queue is deep: send this rung, but no video until the queue has drained.
    Drain {
        to: usize,
    },
    /// The queue has drained: send video again, starting with a keyframe.
    Resume,
}

/// What the controller makes of its recent answers, for metrics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    pub rung: usize,
    pub rtt_ms: Option<u64>,
    pub min_rtt_ms: Option<u64>,
    pub queue_delay_ms: Option<u64>,
    /// Lost and all video packets of the loss window.
    pub loss: Option<(u32, u32)>,
    /// The uplink measured at the last cut on a rising queue.
    pub capacity: Option<u32>,
}

#[derive(Debug, Clone, Copy)]
struct Sample {
    sent_ms: u64,
    received_ms: u64,
    rtt_ms: u64,
    /// The smaller round trip of this answer and the one before, minus the base: a single late
    /// answer is jitter, two in a row are a queue.
    queue_delay_ms: u64,
    /// This answer's round trip minus the base.
    raw_queue_delay_ms: u64,
    received: u32,
    lost: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Trend {
    /// Rising at least `trend_ms_per_s`: the uplink carries less than is sent.
    Rising,
    /// Neither rising nor draining; `falling` when the slope is below zero at all.
    Flat { falling: bool },
    /// Falling at least `trend_ms_per_s`.
    Draining,
    /// Too few answers since the last change.
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Loss {
    Low,
    Moderate,
    High,
    Unknown,
}

#[derive(Debug, Clone, Copy)]
struct Signals {
    queue_delay_ms: Option<u64>,
    raw_queue_delay_ms: Option<u64>,
    /// See [`Controller::deep`].
    deep: bool,
    trend: Trend,
    loss: Loss,
}

impl Signals {
    fn queued(self, limits: &Thresholds) -> bool {
        let high = limits.queue_delay_high_ms;
        self.queue_delay_ms.is_some_and(|ms| ms > high) || self.deep
    }

    fn healthy(self, limits: &Thresholds) -> bool {
        let at_base = self
            .queue_delay_ms
            .is_some_and(|ms| ms <= limits.queue_delay_base_ms);
        at_base && matches!(self.loss, Loss::Low | Loss::Unknown)
    }
}

/// An uplink rate measured while the queue rose: what the uplink carries. Kept until a later cut
/// measures it again or a base shift clears it, so no climb after the first measurement is blind.
#[derive(Debug, Clone, Copy)]
struct Capacity {
    bytes_per_s: u32,
    /// When it was measured, or last confirmed by a failed climb: the holds count from here.
    measured_ms: u64,
}

pub struct Controller {
    ladder: Ladder,
    limits: Thresholds,
    budget: Budget,
    current: usize,
    last_answer_ms: Option<u64>,
    last_change_ms: Option<u64>,
    healthy_since_ms: Option<u64>,
    capacity: Option<Capacity>,
    /// Per rung: how long the measured capacity bounds a climb to it, and the capacity measured
    /// when a climb to it last failed.
    holds: Vec<u64>,
    failed_at: Vec<Option<u32>>,
    /// The rung and time of the last change, when that change was a climb.
    climbed: Option<(usize, u64)>,
    /// The rung before the last change, when that change was a cut for queue delay.
    cut_from: Option<usize>,
    /// While the rung climbed to is held below its own bitrate: the most it may send, in kbit/s.
    ceiling_kbit: Option<u32>,
    /// The most video the controller has tested on the wire, in bytes per second, since the last
    /// cut: the ceilings raised and the rung bitrate reached after a healthy dwell
    /// each. It is believed together with the capacity, so a recovered link is not held to the
    /// uplink measured at a cut.
    proven: u32,
    /// The video is paused while the queue drains.
    draining: bool,
    /// The last drain ended without the queue draining.
    grown_base: bool,
    /// The rung before a fall to the floor that no queue led to, until the link is back.
    fallen_from: Option<usize>,
    /// Round trips that arrived before this do not count towards the base.
    base_from_ms: u64,
    last_base_shift_ms: Option<u64>,
    samples: VecDeque<Sample>,
}

impl Controller {
    pub fn new(
        ladder: Ladder,
        limits: Thresholds,
        budget: Budget,
        start: usize,
    ) -> Result<Self, LadderError> {
        if ladder.rung(start).is_none() {
            return Err(LadderError::Start(start));
        }
        let rungs = ladder.floor() + 1;
        Ok(Self {
            ladder,
            holds: vec![limits.capacity_hold_ms; rungs],
            failed_at: vec![None; rungs],
            climbed: None,
            limits,
            budget,
            current: start,
            last_answer_ms: None,
            last_change_ms: None,
            healthy_since_ms: None,
            capacity: None,
            cut_from: None,
            ceiling_kbit: None,
            proven: 0,
            draining: false,
            grown_base: false,
            fallen_from: None,
            base_from_ms: 0,
            last_base_shift_ms: None,
            samples: VecDeque::new(),
        })
    }

    /// The uplink measured at the last cut on a rising queue.
    pub fn capacity(&self) -> Option<u32> {
        self.capacity.map(|c| c.bytes_per_s)
    }

    /// What the sender's pacer holds the video to, in bytes per second on the wire
    /// (`pace::rate`): the bitrate the controller allows the current rung (its ceiling while a
    /// climb into it is paced, else its own), against the uplink it believes in.
    pub fn pace_bytes_per_s(&self, usage: Usage) -> Option<u32> {
        let rung = self.ladder.rung(self.current)?;
        let kbit = self
            .ceiling_kbit
            .map_or(rung.bitrate_kbit, |c| c.min(rung.bitrate_kbit));
        let allowed = budget::video_bytes_per_s(kbit, rung.fps, usage.video_packets_per_frame);
        let believed = self.capacity().map(|c| c.max(self.proven));
        Some(pace::rate(allowed, believed))
    }

    /// The most the current rung may send, in kbit/s, while a climb into it is paced.
    pub fn ceiling_kbit(&self) -> Option<u32> {
        self.ceiling_kbit
    }

    /// Takes an answer, if one arrived, and decides at `now_ms`. Call it also without one, at
    /// least every probe interval, so silence is noticed.
    pub fn observe(&mut self, feedback: Option<Feedback>, now_ms: u64) -> Decision {
        let fresh = feedback.filter(|f| self.record(f, now_ms));
        if self.stale(now_ms) {
            return self.fall(now_ms);
        }
        let Some(feedback) = fresh else {
            return Decision::Hold;
        };
        let signals = self.signals(now_ms);
        if self.draining {
            return self.drained(signals, &feedback, now_ms);
        }
        self.track_health(signals, now_ms);
        match self.change(signals, &feedback, now_ms) {
            Decision::Hold => self.pause_if_deep(signals, now_ms),
            decision => decision,
        }
    }

    fn change(&mut self, signals: Signals, f: &Feedback, now_ms: u64) -> Decision {
        if !self.may_change(signals, now_ms) {
            return Decision::Hold;
        }
        if signals.queued(&self.limits) {
            return self.queue(signals, f, now_ms);
        }
        if signals.loss == Loss::High {
            return self.step(self.below(), Reason::Loss, now_ms);
        }
        if let Some(from) = self.fallen_from.filter(|_| self.recovered(now_ms)) {
            return self.recover(from, f, now_ms);
        }
        let paced = self.healthy_for(self.limits.min_change_interval_ms, now_ms);
        if let Some(ceiling) = self.ceiling_kbit.filter(|c| paced && self.held_back(*c, f)) {
            return self.raise(ceiling, f, now_ms);
        }
        if paced && self.outgrown(f) {
            return self.forget_capacity(now_ms);
        }
        if self.dwelled(now_ms) {
            return self.climb(f, now_ms);
        }
        Decision::Hold
    }

    /// A queue at least `drain_above_ms` deep pauses the video at any answer, not only at a cut:
    /// within the change interval, or while a cut's headroom drains it at a quarter of the rate a
    /// pause would. Not after a drain that drained nothing, until the rung changes or the delay
    /// falls under the threshold: that delay is the base grown, for the base-shift rule.
    fn pause_if_deep(&mut self, signals: Signals, now_ms: u64) -> Decision {
        let deep = signals
            .raw_queue_delay_ms
            .is_some_and(|ms| ms >= self.limits.drain_above_ms);
        self.grown_base &= deep;
        if !deep || self.grown_base {
            return Decision::Hold;
        }
        self.draining = true;
        self.last_change_ms = Some(now_ms);
        self.healthy_since_ms = None;
        Decision::Drain { to: self.current }
    }

    pub fn status(&self, now_ms: u64) -> Status {
        let signals = self.signals(now_ms);
        Status {
            rung: self.current,
            rtt_ms: self.samples.back().map(|s| s.rtt_ms),
            min_rtt_ms: self.min_rtt(now_ms),
            queue_delay_ms: signals.queue_delay_ms,
            loss: self.loss_counts(now_ms),
            capacity: self.capacity(),
        }
    }

    /// Keeps an answer for the windows; false (and nothing kept) for one older than the newest.
    fn record(&mut self, f: &Feedback, now_ms: u64) -> bool {
        if self
            .samples
            .back()
            .is_some_and(|s| f.probe_sent_ms <= s.sent_ms)
        {
            return false;
        }
        self.last_answer_ms = Some(now_ms);
        let keep_ms = [
            self.limits.min_rtt_window_ms,
            self.limits.loss_window_ms,
            self.limits.trend_window_ms,
        ]
        .into_iter()
        .max()
        .unwrap_or(0);
        while self.samples.len() >= MAX_SAMPLES
            || self
                .samples
                .front()
                .is_some_and(|s| s.received_ms.saturating_add(keep_ms) < now_ms)
        {
            self.samples.pop_front();
        }
        let min_rtt = self.min_rtt(now_ms).map_or(f.rtt_ms, |m| m.min(f.rtt_ms));
        let filtered = self
            .samples
            .back()
            .map_or(f.rtt_ms, |s| s.rtt_ms.min(f.rtt_ms));
        self.samples.push_back(Sample {
            sent_ms: f.probe_sent_ms,
            received_ms: now_ms,
            rtt_ms: f.rtt_ms,
            queue_delay_ms: filtered.saturating_sub(min_rtt),
            raw_queue_delay_ms: f.rtt_ms.saturating_sub(min_rtt),
            received: f.received,
            lost: f.lost,
        });
        true
    }

    /// The smallest round trip of the window, counting none from before a base shift.
    fn min_rtt(&self, now_ms: u64) -> Option<u64> {
        let from = now_ms
            .saturating_sub(self.limits.min_rtt_window_ms)
            .max(self.base_from_ms);
        self.samples
            .iter()
            .filter(|s| s.received_ms >= from)
            .map(|s| s.rtt_ms)
            .min()
    }

    /// Long silent after an answer. Before the first, the start rung stands: the relay may not be
    /// up yet, and the vehicle would send that rung anyway.
    fn stale(&self, now_ms: u64) -> bool {
        self.last_answer_ms
            .is_some_and(|heard| now_ms.saturating_sub(heard) > self.limits.stale_after_ms)
    }

    /// Answers to probes sent since the last change: what the change led to.
    fn since_change(&self) -> impl DoubleEndedIterator<Item = &Sample> {
        let from = self.last_change_ms.unwrap_or(0);
        self.samples.iter().filter(move |s| s.sent_ms >= from)
    }

    fn signals(&self, now_ms: u64) -> Signals {
        let newest = self.samples.back();
        Signals {
            queue_delay_ms: newest.map(|s| s.queue_delay_ms),
            raw_queue_delay_ms: newest.map(|s| s.raw_queue_delay_ms),
            deep: self.deep(),
            trend: self.trend(),
            loss: self.loss(now_ms),
        }
    }

    /// An answer to a probe sent since the last change more than twice the threshold late: a queue
    /// (jitter never comes close), unless that change was a cut for one, which leaves the queue
    /// still there and draining for the trend to tell.
    fn deep(&self) -> bool {
        self.cut_from.is_none()
            && self
                .since_change()
                .next_back()
                .is_some_and(|s| s.raw_queue_delay_ms > self.deep_ms())
    }

    fn deep_ms(&self) -> u64 {
        self.limits.queue_delay_high_ms.saturating_mul(2)
    }

    /// The least-squares slope of the queue delay over the trend window of probes sent since the
    /// last change, which needs answers spanning at least half the window. The first answer after
    /// a change is left out: its queue delay is filtered with an answer from before the change.
    fn trend(&self) -> Trend {
        let Some(newest) = self.since_change().next_back() else {
            return Trend::Unknown;
        };
        let from = newest.sent_ms.saturating_sub(self.limits.trend_window_ms);
        let window: Vec<&Sample> = self
            .since_change()
            .skip(1)
            .filter(|s| s.sent_ms >= from)
            .collect();
        let Some(oldest) = window.first() else {
            return Trend::Unknown;
        };
        let span_ms = newest.sent_ms.saturating_sub(oldest.sent_ms);
        if span_ms.saturating_mul(2) < self.limits.trend_window_ms {
            return Trend::Unknown;
        }
        let points = window
            .iter()
            .map(|s| (s.sent_ms.saturating_sub(oldest.sent_ms), s.queue_delay_ms));
        let Some(slope) = slope_per_s(points) else {
            return Trend::Unknown;
        };
        let steep = i128::from(self.limits.trend_ms_per_s);
        if slope >= steep {
            return Trend::Rising;
        }
        if slope <= -steep {
            return Trend::Draining;
        }
        Trend::Flat { falling: slope < 0 }
    }

    /// Lost and all video packets in the loss window, since the last change.
    fn loss_counts(&self, now_ms: u64) -> Option<(u32, u32)> {
        let from = now_ms.saturating_sub(self.limits.loss_window_ms);
        let (lost, all) = self.since_change().filter(|s| s.received_ms >= from).fold(
            (0_u32, 0_u32),
            |(lost, all), s| {
                (
                    lost.saturating_add(s.lost),
                    all.saturating_add(s.received).saturating_add(s.lost),
                )
            },
        );
        (all >= self.limits.loss_min_packets).then_some((lost, all))
    }

    fn loss(&self, now_ms: u64) -> Loss {
        let Some((lost, all)) = self.loss_counts(now_ms) else {
            return Loss::Unknown;
        };
        let percent = u64::from(lost) * 100;
        if percent < u64::from(self.limits.loss_low_percent) * u64::from(all) {
            return Loss::Low;
        }
        if percent > u64::from(self.limits.loss_high_percent) * u64::from(all) {
            return Loss::High;
        }
        Loss::Moderate
    }

    /// The healthy streak counts only answers to probes sent since the last change.
    fn track_health(&mut self, signals: Signals, now_ms: u64) {
        if self.since_change().next_back().is_none() {
            return;
        }
        if signals.healthy(&self.limits) {
            self.healthy_since_ms.get_or_insert(now_ms);
        } else {
            self.healthy_since_ms = None;
        }
    }

    /// Long enough since the last change, or a deep queue, which already shows it. Every signal
    /// counts only answers to probes sent since it, so nothing changes again before the last
    /// change shows in the feedback.
    fn may_change(&self, signals: Signals, now_ms: u64) -> bool {
        signals.deep
            || self.last_change_ms.is_none_or(|changed| {
                now_ms.saturating_sub(changed) >= self.limits.min_change_interval_ms
            })
    }

    fn dwelled(&self, now_ms: u64) -> bool {
        self.healthy_for(self.limits.dwell_ms, now_ms)
    }

    fn recovered(&self, now_ms: u64) -> bool {
        self.healthy_for(self.limits.outage_recovery_ms, now_ms)
    }

    fn healthy_for(&self, ms: u64, now_ms: u64) -> bool {
        self.healthy_since_ms
            .is_some_and(|since| now_ms.saturating_sub(since) >= ms)
    }

    fn below(&self) -> usize {
        (self.current + 1).min(self.ladder.floor())
    }

    /// A rising queue means the uplink carries less than is sent, so what arrives measures it: cut
    /// to the best rung that fits a share of it after the reserve, at least one rung (one rung
    /// while nothing was measured yet). A queue that stands without falling at all is cut one
    /// rung; if it still does not fall after a cut, the cut changed nothing, so it was the base
    /// delay that grew: take the cut back and count round trips from now on (at most once per
    /// window, or a queue that stands at the cut's rate would bounce). One that falls, however
    /// slowly, is draining: hold. A deep queue is cut as rising without waiting for the trend,
    /// and a cut with an answer at least `drain_above_ms` late pauses the video until the queue has
    /// drained.
    fn queue(&mut self, signals: Signals, f: &Feedback, now_ms: u64) -> Decision {
        if signals.trend == Trend::Rising || signals.deep {
            let to = match f.uplink_bytes_per_s {
                Some(bytes_per_s) => {
                    self.measured(bytes_per_s, now_ms);
                    self.fitting(self.below(), f)
                }
                None => self.below(),
            };
            return self.cut_or_drain(to, signals.raw_queue_delay_ms.unwrap_or(0), now_ms);
        }
        if signals.trend != (Trend::Flat { falling: false }) {
            return Decision::Hold;
        }
        if let Some(from) = self.cut_from.filter(|_| self.may_shift_base(now_ms)) {
            return self.shift_base(from, now_ms);
        }
        self.fail_climb(self.capacity().unwrap_or(u32::MAX), now_ms);
        self.cut(self.below(), now_ms)
    }

    /// Video sent while the cut's spare share drains a queue would arrive late by the queue for
    /// several times its length; paused, the whole uplink drains it, and the first new frame
    /// arrives fresh. A shallower queue the cut's headroom drains soon enough.
    fn cut_or_drain(&mut self, to: usize, queue_ms: u64, now_ms: u64) -> Decision {
        if queue_ms < self.limits.drain_above_ms {
            return self.cut(to, now_ms);
        }
        let from = self.current;
        self.step(to, Reason::QueueDelay, now_ms);
        // Also at the floor, where the step changes nothing: the drain's answers count from now.
        self.last_change_ms = Some(now_ms);
        self.cut_from = Some(from);
        self.draining = true;
        Decision::Drain { to }
    }

    /// The video resumes once an answer to a probe sent during the drain shows the queue back at
    /// the base, or once the queue delay is not draining over the trend window: without video a
    /// queue drains at most of the uplink's rate, many times `trend_ms_per_s`, so one that does
    /// not was no queue but the base delay growing, which the base-shift rule takes back once the
    /// video runs again. It resumes at the best rung the uplink measured meanwhile carries.
    fn drained(&mut self, signals: Signals, f: &Feedback, now_ms: u64) -> Decision {
        let at_base = self.since_change().next_back().is_some()
            && signals
                .queue_delay_ms
                .is_some_and(|ms| ms <= self.limits.queue_delay_base_ms);
        let stuck = !matches!(signals.trend, Trend::Draining | Trend::Unknown);
        if !at_base && !stuck {
            self.remeasure(signals, f, now_ms);
            return Decision::Hold;
        }
        self.draining = false;
        self.grown_base = !at_base;
        // A drained queue shows the cut worked: there is no base shift to take it back for.
        self.cut_from = self.cut_from.filter(|_| !at_base);
        self.last_change_ms = Some(now_ms);
        self.healthy_since_ms = None;
        let to = self.fitting(self.current, f);
        if to == self.current {
            return Decision::Resume;
        }
        self.current = to;
        self.ceiling_kbit = None;
        Decision::Step {
            to,
            reason: Reason::QueueDelay,
        }
    }

    /// Answers to probes sent before the drain come back through the queue that held them: the
    /// relay's windows behind them lie within it, where the uplink carried what was sent before at
    /// its full rate, while at a deep cut those windows may reach back to before the queue. The
    /// lower measurement counts. A probe sent during the pause measures nothing: what got through
    /// just ahead of it is the little the vehicle sent meanwhile, no video.
    fn remeasure(&mut self, signals: Signals, f: &Feedback, now_ms: u64) {
        let high = self.limits.queue_delay_high_ms;
        let queued = signals.raw_queue_delay_ms.is_some_and(|ms| ms > high);
        let sent_before = self.last_change_ms.is_some_and(|at| f.probe_sent_ms < at);
        let measured = f.uplink_bytes_per_s.filter(|_| queued && sent_before);
        let lower = measured.filter(|bytes| self.capacity.is_none_or(|c| *bytes < c.bytes_per_s));
        if let Some(bytes_per_s) = lower {
            self.capacity = Some(Capacity {
                bytes_per_s,
                measured_ms: now_ms,
            });
        }
    }

    /// No answer for the stale threshold: the floor, with video, also out of a drain. Unless the
    /// last answer showed a deep queue, the silence is an outage, which says nothing about the
    /// uplink: the rung before it and the capacity are kept for when the link is back.
    fn fall(&mut self, now_ms: u64) -> Decision {
        let queued = self
            .samples
            .back()
            .is_some_and(|s| s.raw_queue_delay_ms > self.deep_ms());
        if !queued && self.current != self.ladder.floor() {
            self.fallen_from = Some(self.current);
        }
        let resume = std::mem::take(&mut self.draining);
        match self.step(self.ladder.floor(), Reason::Stale, now_ms) {
            Decision::Hold if resume => Decision::Resume,
            decision => decision,
        }
    }

    fn fitting(&self, from: usize, f: &Feedback) -> usize {
        let video = self.share().map_or(u32::MAX, |share| {
            let reserve = self.reserve(from, share, f.usage, f.retrieval);
            self.budget.split(share, f.usage, reserve).video
        });
        self.ladder
            .fitting(from, video, f.usage.video_packets_per_frame)
    }

    /// The part of the measured uplink the video is fitted to.
    fn share(&self) -> Option<u32> {
        self.capacity.map(|c| {
            let share =
                u64::from(c.bytes_per_s) * u64::from(self.limits.capacity_fit_percent) / 100;
            u32::try_from(share).unwrap_or(u32::MAX)
        })
    }

    /// What retrieval is granted of `wanted` when the video is fitted from rung `from`: all of it
    /// while the rung that fits the rest is still a normal one.
    fn reserve(&self, from: usize, share: u32, usage: Usage, wanted: u32) -> u32 {
        self.budget
            .retrieval_reserve(share, usage, wanted, |video| {
                let rung = self
                    .ladder
                    .fitting(from, video, usage.video_packets_per_frame);
                self.ladder
                    .rung(rung)
                    .is_some_and(|r| r.group == Group::Normal)
            })
    }

    /// Whether `allowance` could be reserved for retrieval at `now_ms`: the uplink measured at the
    /// last cut, while it still bounds the current rung, carries a normal rung beside it. Decided
    /// from capacity alone, whether or not a job runs.
    pub fn retrieval_fits(&self, allowance: u32, usage: Usage, now_ms: u64) -> bool {
        let Some(share) = self.share().filter(|_| self.bounds(self.current, now_ms)) else {
            return true;
        };
        self.reserve(self.current, share, usage, allowance) == allowance
    }

    /// Whether the uplink measured at the last cut still bounds `rung`: its hold has not run out.
    fn bounds(&self, rung: usize, now_ms: u64) -> bool {
        let hold = self.holds.get(rung).copied().unwrap_or(0);
        self.capacity
            .is_some_and(|c| now_ms.saturating_sub(c.measured_ms) < hold)
    }

    fn may_shift_base(&self, now_ms: u64) -> bool {
        self.last_base_shift_ms
            .is_none_or(|at| now_ms.saturating_sub(at) >= self.limits.min_rtt_window_ms)
    }

    fn shift_base(&mut self, from: usize, now_ms: u64) -> Decision {
        self.base_from_ms = now_ms;
        self.last_base_shift_ms = Some(now_ms);
        self.capacity = None;
        self.step(from, Reason::BaseShift, now_ms)
    }

    /// Remembers what the uplink carries. A climb cut within a dwell failed: its rung's hold
    /// doubles. Rungs whose climbs failed at clearly less uplink than this get their hold back.
    fn measured(&mut self, bytes_per_s: u32, now_ms: u64) {
        let base = self.limits.capacity_hold_ms;
        let grown = |at: u32| u64::from(at) * GROWN_PERCENT / 100 < u64::from(bytes_per_s);
        self.holds
            .iter_mut()
            .zip(&mut self.failed_at)
            .filter(|(_, failed)| failed.is_some_and(grown))
            .for_each(|(hold, failed)| {
                *hold = base;
                *failed = None;
            });
        self.fail_climb(bytes_per_s, now_ms);
        self.proven = 0;
        self.capacity = Some(Capacity {
            bytes_per_s,
            measured_ms: now_ms,
        });
    }

    /// A cut within a dwell of a climb, or of the last raise of its ceiling, fails that rung.
    fn fail_climb(&mut self, bytes_per_s: u32, now_ms: u64) {
        let dwell = self.limits.dwell_ms;
        let Some((rung, _)) = self
            .climbed
            .filter(|(rung, at)| *rung == self.current && now_ms.saturating_sub(*at) < dwell)
        else {
            return;
        };
        self.back_off(rung, bytes_per_s);
        if let Some(capacity) = self.capacity.as_mut() {
            capacity.measured_ms = now_ms;
        }
    }

    fn back_off(&mut self, rung: usize, bytes_per_s: u32) {
        let max = self.limits.capacity_hold_max_ms;
        if let Some(hold) = self.holds.get_mut(rung) {
            *hold = hold.saturating_mul(2).min(max);
        }
        if let Some(failed) = self.failed_at.get_mut(rung) {
            *failed = Some(bytes_per_s);
        }
    }

    /// Back to the rung before the outage, or the best below it that the capacity measured at the
    /// last cut carries. Like a climb, cut within a dwell it doubles that
    /// rung's hold.
    fn recover(&mut self, from: usize, f: &Feedback, now_ms: u64) -> Decision {
        self.fallen_from = None;
        let to = self.fitting(from, f);
        let decision = self.step(to, Reason::Recovered, now_ms);
        self.climbed = Some((to, now_ms));
        decision
    }

    /// One rung up, as far as the capacity still holds it.
    fn climb(&mut self, f: &Feedback, now_ms: u64) -> Decision {
        let Some(up) = self.current.checked_sub(1) else {
            return Decision::Hold;
        };
        if self.bounded(up, f, now_ms) != up {
            return Decision::Hold;
        }
        let ceiling = self.probe_ceiling(up, f);
        let decision = self.step(up, Reason::Healthy, now_ms);
        self.climbed = Some((up, now_ms));
        self.ceiling_kbit = ceiling;
        decision
    }

    /// What a climb into `rung` is held to when an uplink was measured and the rung costs more
    /// than `PROBE_PERCENT` of it; never below the bitrate of
    /// the rung climbed from.
    fn probe_ceiling(&self, rung: usize, f: &Feedback) -> Option<u32> {
        let capacity = self.capacity?;
        let target = self.ladder.rung(rung)?;
        let allowed = u64::from(capacity.bytes_per_s) * PROBE_PERCENT / 100;
        let kbit = budget::video_kbit_within(
            u32::try_from(allowed).unwrap_or(u32::MAX),
            target.fps,
            f.usage.video_packets_per_frame,
        );
        let from = self.ladder.rung(self.current).map_or(0, |r| r.bitrate_kbit);
        Some(kbit.max(from)).filter(|kbit| *kbit < target.bitrate_kbit)
    }

    /// Whether the uplink carried (nearly) all the ceiling allows: a source that sends less has
    /// not shown that the uplink carries more, so there is nothing to raise it for.
    fn held_back(&self, ceiling: u32, f: &Feedback) -> bool {
        let Some(rung) = self.ladder.rung(self.current) else {
            return false;
        };
        let allowed = budget::video_bytes_per_s(ceiling, rung.fps, f.usage.video_packets_per_frame);
        f.uplink_bytes_per_s
            .is_some_and(|carried| u64::from(carried) * 100 >= u64::from(allowed) * 90)
    }

    /// Whether the uplink carried clearly more than the capacity measured at the last cut: the link
    /// has grown since. A source that fills its ceiling is raised instead (`held_back`), so this is
    /// one that sends less than the ceiling but more than the capacity.
    fn outgrown(&self, f: &Feedback) -> bool {
        let (Some(capacity), Some(carried)) = (self.capacity(), f.uplink_bytes_per_s) else {
            return false;
        };
        u64::from(capacity) * GROWN_PERCENT / 100 < u64::from(carried)
    }

    /// Forgets an outgrown capacity, as at a fresh start. A ceiling it set goes with it: held to
    /// it, a picture that compresses well never fills it, and the encoder's one-frame buffer at
    /// the ceiling squeezes every keyframe for good.
    fn forget_capacity(&mut self, now_ms: u64) -> Decision {
        self.capacity = None;
        self.proven = 0;
        if self.ceiling_kbit.take().is_none() {
            return Decision::Hold;
        }
        self.last_change_ms = Some(now_ms);
        self.healthy_since_ms = None;
        Decision::Step {
            to: self.current,
            reason: Reason::Probe,
        }
    }

    /// Healthy at `ceiling` for the change interval: it goes up by `PROBE_PERCENT`, to the rung's own bitrate at
    /// most, which ends the pacing. Like a climb, one cut within a dwell of this fails the rung.
    fn raise(&mut self, ceiling: u32, f: &Feedback, now_ms: u64) -> Decision {
        let rung = self.ladder.rung(self.current);
        let nominal = rung.map_or(0, |r| r.bitrate_kbit);
        let raised = u64::from(ceiling) * PROBE_PERCENT / 100;
        let raised = u32::try_from(raised).unwrap_or(u32::MAX);
        self.ceiling_kbit = Some(raised).filter(|kbit| *kbit < nominal);
        let tested = budget::video_bytes_per_s(
            raised.min(nominal),
            rung.map_or(0, |r| r.fps),
            f.usage.video_packets_per_frame,
        );
        self.proven = self.proven.max(tested);
        self.last_change_ms = Some(now_ms);
        self.healthy_since_ms = None;
        self.climbed = Some((self.current, now_ms));
        Decision::Step {
            to: self.current,
            reason: Reason::Probe,
        }
    }

    /// `rung`, or while that rung's hold on the uplink measured at the last cut has not run out,
    /// the best rung from it down that the measurement carries. Past the hold the capacity says
    /// nothing about the rung: the link may have grown since.
    fn bounded(&self, rung: usize, f: &Feedback, now_ms: u64) -> usize {
        if !self.bounds(rung, now_ms) {
            return rung;
        }
        self.fitting(rung, f)
    }

    fn cut(&mut self, to: usize, now_ms: u64) -> Decision {
        let from = self.current;
        let decision = self.step(to, Reason::QueueDelay, now_ms);
        if decision != Decision::Hold {
            self.cut_from = Some(from);
        }
        decision
    }

    fn step(&mut self, to: usize, reason: Reason, now_ms: u64) -> Decision {
        if to == self.current {
            return Decision::Hold;
        }
        self.current = to;
        self.last_change_ms = Some(now_ms);
        self.healthy_since_ms = None;
        self.cut_from = None;
        self.climbed = None;
        self.ceiling_kbit = None;
        self.grown_base = false;
        Decision::Step { to, reason }
    }
}

/// The least-squares slope of (ms, value) points in value per second; `None` for fewer than two
/// distinct times.
fn slope_per_s(points: impl Iterator<Item = (u64, u64)>) -> Option<i128> {
    let (mut n, mut sx, mut sy, mut sxx, mut sxy) = (0_i128, 0_i128, 0_i128, 0_i128, 0_i128);
    for (x, y) in points {
        let (x, y) = (i128::from(x), i128::from(y));
        n += 1;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    let denominator = n * sxx - sx * sx;
    (denominator != 0).then(|| (n * sxy - sx * sy) * 1000 / denominator)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NORMAL: Rung = Rung {
        width: 1280,
        height: 720,
        fps: 15,
        bitrate_kbit: 1000,
        group: Group::Normal,
    };
    const LIMITS: Thresholds = Thresholds {
        min_rtt_window_ms: 10_000,
        queue_delay_high_ms: 150,
        drain_above_ms: 300,
        queue_delay_base_ms: 50,
        trend_window_ms: 1000,
        trend_ms_per_s: 100,
        capacity_fit_percent: 75,
        loss_window_ms: 5000,
        loss_min_packets: 50,
        loss_low_percent: 2,
        loss_high_percent: 10,
        stale_after_ms: 3000,
        outage_recovery_ms: 2000,
        dwell_ms: 10_000,
        min_change_interval_ms: 2000,
        capacity_hold_ms: 30_000,
        capacity_hold_max_ms: 300_000,
    };
    const PROBE_EVERY_MS: u64 = 250;
    /// The uplink a cut measures unless a case says otherwise: room for every rung.
    const ROOMY: u32 = 1_000_000;

    fn ladder() -> Ladder {
        let rung = |fps, bitrate_kbit, group| Rung {
            fps,
            bitrate_kbit,
            group,
            ..NORMAL
        };
        Ladder::new(vec![
            rung(15, 1000, Group::Normal),
            rung(15, 500, Group::Normal),
            rung(5, 100, Group::Constrained),
            rung(1, 25, Group::AwarenessOnly),
        ])
        .unwrap()
    }

    type Script = fn(u64) -> Option<(u64, u32, u32, u32)>;

    /// Scripted: how the link answers the probe sent at a time (round trip, video packets received
    /// and lost, the uplink it measured), `None` when the probe or its answer is lost. Queue: a
    /// FIFO uplink of `capacity` bytes a second at a time (0: a blackout that drops everything)
    /// behind a base round trip, filled by the rung the controller picks (nothing while it
    /// drains), one packet a frame.
    enum Link {
        Script(Script),
        Queue {
            base_rtt_ms: u64,
            capacity: fn(u64) -> u32,
        },
    }
    /// A name, the start rung, the link, how long it runs and the changes expected.
    type Case = (&'static str, usize, Link, u64, &'static [(u64, Change)]);

    /// A decision other than holding: a step to a rung and why, a cut that pauses the video while
    /// the queue drains, and the video resuming.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Change {
        To(usize, Reason),
        Drain(usize),
        Resume,
    }

    #[expect(
        clippy::unnecessary_wraps,
        reason = "a scripted link, which may leave a probe unanswered; this one answers them all"
    )]
    fn healthy(_: u64) -> Option<(u64, u32, u32, u32)> {
        Some((600, 100, 0, ROOMY))
    }

    fn feedback(sent: u64, rtt: u64, received: u32, lost: u32, uplink: u32) -> Feedback {
        Feedback {
            probe_sent_ms: sent,
            rtt_ms: rtt,
            received,
            lost,
            uplink_bytes_per_s: Some(uplink),
            usage: Usage {
                telemetry_offered: None,
                telemetry_datagrams_per_s: 0,
                control: 0,
                video_packets_per_frame: 1,
            },
            retrieval: 0,
        }
    }

    /// Runs `link` for `until_ms` with a probe every 250 ms and returns every change but the
    /// raising of a ceiling.
    fn run(start: usize, link: &Link, until_ms: u64) -> Vec<(u64, Change)> {
        changes(start, link, until_ms)
            .into_iter()
            .filter(|(_, change)| !matches!(change, Change::To(_, Reason::Probe)))
            .collect()
    }

    fn changes(start: usize, link: &Link, until_ms: u64) -> Vec<(u64, Change)> {
        let budget = Budget {
            telemetry_percent: 100,
            telemetry_floor: 0,
            telemetry_cap: 0,
            control: 0,
        };
        let mut controller = Controller::new(ladder(), LIMITS, budget, start).unwrap();
        let decisions = match link {
            Link::Script(script) => scripted(&mut controller, *script, until_ms),
            Link::Queue {
                base_rtt_ms,
                capacity,
            } => queued(&mut controller, start, *base_rtt_ms, *capacity, until_ms),
        };
        decisions
            .into_iter()
            .filter_map(|(at, decision)| match decision {
                Decision::Step { to, reason } => Some((at, Change::To(to, reason))),
                Decision::Drain { to } => Some((at, Change::Drain(to))),
                Decision::Resume => Some((at, Change::Resume)),
                Decision::Hold => None,
            })
            .collect()
    }

    fn scripted(
        controller: &mut Controller,
        script: Script,
        until_ms: u64,
    ) -> Vec<(u64, Decision)> {
        let mut events: Vec<(u64, Option<Feedback>)> = (0..until_ms)
            .step_by(usize::try_from(PROBE_EVERY_MS).unwrap())
            .flat_map(|sent| {
                let answer = script(sent).map(|(rtt, received, lost, uplink)| {
                    (
                        sent + rtt,
                        Some(feedback(sent, rtt, received, lost, uplink)),
                    )
                });
                [Some((sent, None)), answer]
            })
            .flatten()
            .filter(|(at, _)| *at < until_ms)
            .collect();
        events.sort_by_key(|(at, f)| (*at, f.is_none()));
        events
            .into_iter()
            .map(|(at, f)| (at, controller.observe(f, at)))
            .collect()
    }

    /// Steps of 10 ms: the queue gains what the rung sends and loses what the uplink carries; a
    /// probe every 250 ms sees the queue it waits behind.
    fn queued(
        controller: &mut Controller,
        start: usize,
        base_rtt_ms: u64,
        capacity: fn(u64) -> u32,
        until_ms: u64,
    ) -> Vec<(u64, Decision)> {
        let ladder = ladder();
        let cost = |rung: usize, ceiling: Option<u32>| {
            let rung = ladder.rung(rung).unwrap();
            let kbit = ceiling.map_or(rung.bitrate_kbit, |c| c.min(rung.bitrate_kbit));
            u64::from(
                Rung {
                    bitrate_kbit: kbit,
                    ..rung
                }
                .wire_bytes_per_s(1),
            )
        };
        let mut rung = start;
        let mut paused = false;
        let mut queue = 0_u64;
        let mut answers: Vec<(u64, Feedback)> = Vec::new();
        let mut decisions = Vec::new();
        for now in (0..until_ms).step_by(10) {
            let carried = u64::from(capacity(now));
            let sent = if paused {
                0
            } else {
                cost(rung, controller.ceiling_kbit())
            };
            if carried > 0 {
                queue = (queue + sent / 100).saturating_sub(carried / 100);
            }
            let probe = now % PROBE_EVERY_MS == 0;
            if probe && carried > 0 {
                let rtt = base_rtt_ms + queue * 1000 / carried;
                let uplink = if queue > 0 { carried } else { sent };
                let uplink = u32::try_from(uplink).unwrap();
                answers.push((now + rtt, feedback(now, rtt, 100, 0, uplink)));
            }
            let mut due: Vec<(u64, Feedback)> =
                answers.extract_if(.., |(at, _)| *at <= now).collect();
            due.sort_by_key(|(at, _)| *at);
            let tick = probe.then_some((now, None));
            for (at, f) in due.into_iter().map(|(at, f)| (at, Some(f))).chain(tick) {
                let decision = controller.observe(f, at);
                match decision {
                    Decision::Step { to, .. } => (rung, paused) = (to, false),
                    Decision::Drain { to } => (rung, paused) = (to, true),
                    Decision::Resume => paused = false,
                    Decision::Hold => {}
                }
                decisions.push((at, decision));
            }
        }
        decisions
    }

    #[test]
    fn a_ceiling_goes_up_only_when_the_uplink_carried_all_of_it() {
        // The cut at 14.1 s measures 50 kB/s; the climb to the 500 kbit/s rung at 44.1 s is held
        // to 110 % of it, and the source then sends 5 kB/s, far below the ceiling: nothing shows
        // that the uplink carries more.
        let link = Link::Script(|sent| {
            let grown = sent.clamp(12_000, 14_100) - 12_000;
            let drained = sent.saturating_sub(14_100);
            let queue = (grown / 5).saturating_sub(drained);
            let carried = if sent < 20_000 { 50_000 } else { 5000 };
            Some((600 + queue, 100, 0, carried))
        });
        let changes = changes(0, &link, 80_000);
        assert!(changes.contains(&(44_100, Change::To(1, Reason::Healthy))));
        assert!(
            !changes
                .iter()
                .any(|(_, c)| matches!(c, Change::To(_, Reason::Probe)))
        );
    }

    #[test]
    fn a_link_that_carries_more_than_the_capacity_measured_outgrows_it() {
        // The cut at 14.1 s measures 10 kB/s; afterwards the link carries 20 kB/s with no queue,
        // less than the ceiling a climb would get and twice the capacity: the link grew.
        let link = |sent: u64| {
            let grown = sent.clamp(12_000, 14_100) - 12_000;
            let drained = sent.saturating_sub(14_100);
            let queue = (grown / 5).saturating_sub(drained);
            let carried = if sent < 20_000 { 10_000 } else { 20_000 };
            Some((600 + queue, 100, 0, carried))
        };
        let budget = Budget {
            telemetry_percent: 100,
            telemetry_floor: 0,
            telemetry_cap: 0,
            control: 0,
        };
        let mut controller = Controller::new(ladder(), LIMITS, budget, 0).unwrap();
        scripted(&mut controller, link, 80_000);
        assert_eq!(controller.capacity(), None);
        assert_eq!(controller.ceiling_kbit(), None);
    }

    #[test]
    fn walks_the_ladder_down_fast_and_up_slowly() {
        use Change::{Drain, Resume, To};
        use Reason::{BaseShift, Healthy, Loss, QueueDelay, Recovered, Stale};
        let cases: &[Case] = &[
            (
                // Jitter of +-40 ms and one answer 200 ms late now and then: no queue.
                "steady healthy feedback holds the top",
                0,
                Link::Script(|sent| {
                    let jitter = sent / PROBE_EVERY_MS * 37 % 81;
                    let late = if sent % 7000 == 3000 { 200 } else { 0 };
                    Some((560 + jitter + late, 99, 1, ROOMY))
                }),
                60_000,
                &[],
            ),
            (
                // Healthy from the first answer (600 ms); a rung up after the dwell, the next a
                // dwell after an answer to a probe sent since.
                "recovery waits out the dwell",
                2,
                Link::Script(healthy),
                30_000,
                &[(10_600, To(1, Healthy)), (21_350, To(0, Healthy))],
            ),
            (
                // From 12 s the queue grows 200 ms a second. The first answer with two round trips
                // in a row over the threshold arrives at 14.1 s; the rung below has room. The queue
                // keeps growing (as it would if that were not enough): the next answer, more than
                // 300 ms late, pauses the video within the change interval.
                "a growing queue cuts at once",
                0,
                Link::Script(|sent| {
                    let queue = sent.saturating_sub(12_000) / 5;
                    Some((600 + queue, 100, 0, ROOMY))
                }),
                15_000,
                &[(14_100, To(1, QueueDelay)), (14_400, Drain(1))],
            ),
            (
                // As above, but the uplink measured 15 kB/s: the cut fits rungs into 75 % of it
                // (11.25 kB/s), which the 100 kbit/s rung (13.1 kB/s with headers) would exceed
                // although it fits the whole: the floor, skipping two rungs.
                "the measured uplink with headroom picks the rung",
                0,
                Link::Script(|sent| {
                    let queue = sent.saturating_sub(12_000) / 5;
                    Some((600 + queue, 100, 0, 15_000))
                }),
                15_000,
                &[(14_100, To(3, QueueDelay)), (14_400, Drain(3))],
            ),
            (
                // After the cut the queue drains at only 50 ms a second: slower than the trend
                // threshold, but falling, so it is no base shift and the cut stands. Over 300 ms it
                // pauses the video; the pause drains nothing here, so the video resumes and the
                // delay is not paused again.
                "a slowly draining queue is not taken for a base shift",
                0,
                Link::Script(|sent| {
                    let grown = sent.clamp(12_000, 14_100) - 12_000;
                    let drained = sent.saturating_sub(14_100) / 20;
                    let queue = (grown / 5).saturating_sub(drained);
                    Some((600 + queue, 100, 0, ROOMY))
                }),
                25_000,
                &[
                    (14_100, To(1, QueueDelay)),
                    (14_400, Drain(1)),
                    (16_213, Resume),
                ],
            ),
            (
                // The queue grows until the cut, then drains at 400 ms a second: above the
                // threshold for seconds yet, but falling, so one cut is enough; over 300 ms the
                // video pauses until it has drained.
                "a draining queue is not cut again",
                0,
                Link::Script(|sent| {
                    let grown = sent.clamp(12_000, 14_100) - 12_000;
                    let drained = sent.saturating_sub(14_100) * 2 / 5;
                    let queue = (grown / 5).saturating_sub(drained);
                    Some((600 + queue, 100, 0, ROOMY))
                }),
                25_000,
                &[
                    (14_100, To(1, QueueDelay)),
                    (14_400, Drain(1)),
                    (15_850, Resume),
                ],
            ),
            (
                // After the cut (and a pause while the queue is over 300 ms) the queue drains within
                // a second; healthy from 15 s, but the uplink measured at the cut (40 kB/s) cannot
                // carry the rung above until it is 30 s old.
                "the uplink measured at a cut holds the climb",
                0,
                Link::Script(|sent| {
                    let grown = sent.clamp(12_000, 14_100) - 12_000;
                    let drained = sent.saturating_sub(14_100);
                    let queue = (grown / 5).saturating_sub(drained);
                    Some((600 + queue, 100, 0, 40_000))
                }),
                50_000,
                &[
                    (14_100, To(2, QueueDelay)),
                    (14_400, Drain(2)),
                    (15_120, Resume),
                    (44_100, To(1, Healthy)),
                ],
            ),
            (
                // A queue building at 50 ms a second, too slowly to count as rising, passes the
                // threshold at 16 s: it stands, so it is cut one rung, and it drains.
                "a standing queue is cut one rung",
                0,
                Link::Script(|sent| {
                    let grown = sent.clamp(12_000, 16_275) - 12_000;
                    let drained = sent.saturating_sub(16_275) / 2;
                    let queue = (grown / 20).saturating_sub(drained);
                    Some((600 + queue, 100, 0, ROOMY))
                }),
                20_000,
                &[(16_275, To(1, QueueDelay))],
            ),
            (
                // The round trip steps from 600 to 1200 ms at 12 s and stays: it looks like 600 ms
                // of queue, so a cut that drains, but the delay does not fall; the video resumes
                // once it has not fallen over the trend window, and once it still has not fallen
                // with the video back the cut is taken back and 1200 ms is the new base. Answers to
                // probes sent while the video is paused, when the uplink carries little, measure
                // nothing.
                "a step in the base delay is taken back",
                0,
                Link::Script(|sent| {
                    let rtt = if sent < 12_000 { 600 } else { 1200 };
                    let uplink = if sent < 13_200 { ROOMY } else { 3000 };
                    Some((rtt, 100, 0, uplink))
                }),
                30_000,
                &[
                    (13_200, Drain(1)),
                    (15_200, Resume),
                    (17_200, To(0, BaseShift)),
                ],
            ),
            (
                // As above, but the round trip then falls 50 ms a second: too slowly for a queue
                // drained with the video paused, so the video resumes all the same.
                "a drain ends when the delay falls too slowly for a queue",
                0,
                Link::Script(|sent| {
                    let fallen = sent.saturating_sub(12_000) / 20;
                    let rtt = if sent < 12_000 { 600 } else { 1200 - fallen };
                    Some((rtt, 100, 0, ROOMY))
                }),
                20_000,
                &[(13_200, Drain(1)), (15_100, Resume)],
            ),
            (
                // A queue that grows 2 s a second from 12 s cuts and drains at the first deep
                // answer, which measures 60 kB/s (its windows reaching back to before the queue);
                // the answers already on their way come back through the queue while the video is
                // paused and measure 15 kB/s, so the video resumes a rung lower than the cut chose.
                "a drain measures the uplink again and resumes at the rung it carries",
                0,
                Link::Script(|sent| {
                    let grown = sent.clamp(12_000, 13_500) - 12_000;
                    let queue = (grown * 2).saturating_sub(sent.saturating_sub(13_500));
                    let uplink = if sent < 12_500 { 60_000 } else { 15_000 };
                    Some((600 + queue, 100, 0, uplink))
                }),
                18_000,
                &[(13_350, Drain(2)), (17_100, To(3, QueueDelay))],
            ),
            (
                // 5 % loss holds: no rung up however long it stays.
                "loss between the bands holds",
                2,
                Link::Script(|_| Some((600, 95, 5, ROOMY))),
                40_000,
                &[],
            ),
            (
                "loss below the low band allows a rung up",
                2,
                Link::Script(|_| Some((600, 99, 1, ROOMY))),
                12_000,
                &[(10_600, To(1, Healthy))],
            ),
            (
                // On a 20 ms link with 15 % loss each cut is answered at once: still one change
                // per 2 s.
                "loss above the high band cuts one rung per interval",
                0,
                Link::Script(|_| Some((20, 85, 15, ROOMY))),
                8000,
                &[(20, To(1, Loss)), (2020, To(2, Loss)), (4020, To(3, Loss))],
            ),
            (
                // The last answer before the outage (to the probe sent at 4.75 s) arrives at
                // 5.35 s; the first after it at 12.6 s, and 2 s healthy bring back the rung before.
                "an outage falls to the floor and comes back to the rung before it",
                0,
                Link::Script(|sent| {
                    (!(5000..12_000).contains(&sent)).then_some((600, 100, 0, ROOMY))
                }),
                16_000,
                &[(8500, To(3, Stale)), (14_600, To(0, Recovered))],
            ),
            (
                // A queue growing two seconds a second, then silence: no outage but a queue too
                // deep to answer, so the floor stays until a dwell of healthy answers.
                "silence after a deep queue is no outage",
                0,
                Link::Script(|sent| match sent {
                    5000..5500 => Some((600 + (sent - 5000) * 2, 100, 0, ROOMY)),
                    5500..12_000 => None,
                    _ => Some((600, 100, 0, ROOMY)),
                }),
                24_000,
                &[
                    (6350, Drain(1)),
                    (9500, To(3, Stale)),
                    (22_600, To(2, Healthy)),
                ],
            ),
            (
                // The relay answers only from 5 s: the start rung stands until then.
                "no answer yet holds the start rung",
                1,
                Link::Script(|sent| (sent >= 5000).then_some((600, 100, 0, ROOMY))),
                8000,
                &[],
            ),
            (
                // A 20 kB/s uplink that cannot carry the 500 kbit/s rung grows to 21 kB/s at 40 s,
                // which still does not carry it. The second climb is held to 110 % of the 20 kB/s
                // measured at the first and cut a few seconds later without measuring anything
                // new, which doubles the rung's hold all the same: the next climb comes 120 s
                // after the second cut, not 30 s.
                "a climb held to a ceiling that fails still doubles its rung's hold",
                2,
                Link::Queue {
                    base_rtt_ms: 600,
                    capacity: |now| if now < 40_000 { 20_000 } else { 21_000 },
                },
                210_000,
                &[
                    (10_600, To(1, Healthy)),
                    (11_682, Drain(2)),
                    (14_850, Resume),
                    (71_850, To(1, Healthy)),
                    (76_517, To(2, QueueDelay)),
                    (196_600, To(1, Healthy)),
                    (201_267, To(2, QueueDelay)),
                ],
            ),
            (
                // The uplink grows from 20 to 200 kB/s at 40 s: once the doubled hold from the
                // failed climb runs out, a climb past the 20 kB/s measured is held to
                // 110 % of it and raised every 2.75 s until the rung sends its own bitrate, which
                // takes 36 s for the 500 kbit/s rung. Carrying three times the 20 kB/s, the link
                // has outgrown that measurement, which is forgotten: the climb to the 1000 kbit/s
                // rung is not held, and after a blackout from 140 to 150 s the jump back returns
                // to it.
                "an outgrown capacity no longer bounds the jump back after an outage",
                2,
                Link::Queue {
                    base_rtt_ms: 600,
                    capacity: |now| match now {
                        ..40_000 => 20_000,
                        140_000..150_000 => 0,
                        _ => 200_000,
                    },
                },
                170_000,
                &[
                    (10_600, To(1, Healthy)),
                    (11_682, Drain(2)),
                    (14_850, Resume),
                    (71_850, To(1, Healthy)),
                    (118_350, To(0, Healthy)),
                    (143_500, To(3, Stale)),
                    (152_600, To(0, Recovered)),
                ],
            ),
            (
                // Rung 1 runs on 200 kB/s until a blackout from 5 to 15 s, after which the link
                // carries 20 kB/s. Nothing measured yet, the jump back to rung 1 overshoots and is
                // cut, which measures the uplink and, as for a failed climb, doubles rung 1's hold:
                // the next climb to it comes 60 s after the cut. A blackout right after that climb
                // leaves rung 1 the rung before it, but the jump back now stops at the rung the
                // measured uplink carries.
                "repeated outages on a link that cannot carry the pre-outage rung overshoot once",
                1,
                Link::Queue {
                    base_rtt_ms: 600,
                    capacity: |now| match now {
                        ..5000 => 200_000,
                        5000..15_000 | 79_000..89_000 => 0,
                        _ => 20_000,
                    },
                },
                100_000,
                &[
                    (8500, To(3, Stale)),
                    (17_600, To(1, Recovered)),
                    (18_682, Drain(2)),
                    (21_850, Resume),
                    (78_850, To(1, Healthy)),
                    (82_500, To(3, Stale)),
                    (91_850, To(2, Recovered)),
                ],
            ),
            (
                // At the floor a deep queue still pauses the video; with no answer after that it
                // resumes when the feedback goes stale, so a drain never outlasts the feedback.
                "silence during a drain resumes the video",
                3,
                Link::Script(|sent| match sent {
                    ..6000 => Some((600, 100, 0, ROOMY)),
                    6000..7000 => Some((1200, 100, 0, ROOMY)),
                    _ => None,
                }),
                12_000,
                &[(7200, Drain(3)), (11_000, Resume)],
            ),
            (
                // 20 kB/s carries the 100 kbit/s rung (13 kB/s) but not the 500 kbit/s one
                // (64 kB/s). The first climb to it is cut and drained at the first answer more
                // than 300 ms late, within the change interval, and its hold doubles: the next
                // climb comes 60 s after that cut, held to 110 % of the uplink measured, and is
                // cut as a standing queue without a drain, which doubles the hold again from that
                // cut, up to 5 minutes: however old the measurement, no climb goes unheld and
                // drains again. A blackout from 20 to 30 s falls to the floor; 2 s
                // after it the rung before comes back, and the capacity, kept through the outage,
                // still holds the climb.
                "a failed climb doubles the hold on its rung",
                2,
                Link::Queue {
                    base_rtt_ms: 600,
                    capacity: |now| match now {
                        20_000..30_000 => 0,
                        _ => 20_000,
                    },
                },
                1_100_000,
                &[
                    (10_600, To(1, Healthy)),
                    (11_682, Drain(2)),
                    (14_850, Resume),
                    (23_500, To(3, Stale)),
                    (32_600, To(2, Recovered)),
                    (71_850, To(1, Healthy)),
                    (74_530, To(2, QueueDelay)),
                    (194_600, To(1, Healthy)),
                    (197_280, To(2, QueueDelay)),
                    (437_350, To(1, Healthy)),
                    (440_030, To(2, QueueDelay)),
                    (740_100, To(1, Healthy)),
                    (742_780, To(2, QueueDelay)),
                    (1_042_850, To(1, Healthy)),
                    (1_045_530, To(2, QueueDelay)),
                ],
            ),
            (
                // From 12 s the uplink carries 40 kB/s, about a third of the top rung. The first
                // answer more than 300 ms late cuts to the rung that fits and pauses the video; it
                // resumes once an answer to a probe sent since shows the queue gone. The drop to
                // 5 kB/s at 20 s is a new queue, cut and drained the same way.
                "a deep queue drains with the video paused",
                0,
                Link::Queue {
                    base_rtt_ms: 600,
                    capacity: |now| match now {
                        ..12_000 => 200_000,
                        12_000..20_000 => 40_000,
                        _ => 5000,
                    },
                },
                25_000,
                &[
                    (13_414, Drain(2)),
                    (17_123, Resume),
                    (21_271, Drain(3)),
                    (24_100, Resume),
                ],
            ),
        ];
        for (name, start, link, until_ms, expected) in cases {
            assert_eq!(run(*start, link, *until_ms), *expected, "{name}");
        }
    }
}
