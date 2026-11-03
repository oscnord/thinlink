//! Where a frame's time goes inside one process: the time from a frame's stamp (or from a point
//! earlier in the pipeline) to a later point, kept for the last [`WINDOW`] frames and summarised
//! as percentiles for the metrics endpoint. the `latency` tracer of `GStreamer` is not usable for this:
//! in the runtime image's 1.26.2 it leaves every pipeline stuck in PREROLLING.

use std::collections::VecDeque;
use std::sync::{Mutex, PoisonError};
use std::time::{Duration, Instant};

use serde::Serialize;

/// Samples kept per stage: about a minute of video at 15 frames per second.
const WINDOW: usize = 1024;
/// Stamps remembered while their frames are in flight.
const IN_FLIGHT: usize = 64;

/// The last [`WINDOW`] durations of one stage.
#[derive(Debug)]
pub struct Stage {
    samples: Mutex<VecDeque<u32>>,
}

/// Percentiles of a [`Stage`]'s window, in milliseconds.
#[derive(Debug, Serialize)]
pub struct StageSnapshot {
    pub n: usize,
    pub p50_ms: f64,
    pub p95_ms: f64,
    pub max_ms: f64,
}

impl Default for Stage {
    fn default() -> Self {
        Self::new()
    }
}

impl Stage {
    pub const fn new() -> Self {
        Self {
            samples: Mutex::new(VecDeque::new()),
        }
    }

    pub fn record(&self, elapsed: Duration) {
        let us = u32::try_from(elapsed.as_micros()).unwrap_or(u32::MAX);
        let mut samples = self.samples.lock().unwrap_or_else(PoisonError::into_inner);
        if samples.len() == WINDOW {
            samples.pop_front();
        }
        samples.push_back(us);
    }

    pub fn snapshot(&self) -> StageSnapshot {
        let mut sorted: Vec<u32> = self
            .samples
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .iter()
            .copied()
            .collect();
        sorted.sort_unstable();
        StageSnapshot {
            n: sorted.len(),
            p50_ms: nearest_rank(&sorted, 50),
            p95_ms: nearest_rank(&sorted, 95),
            max_ms: nearest_rank(&sorted, 100),
        }
    }
}

/// Nearest-rank percentile of sorted microseconds, in milliseconds; 0 for no samples.
fn nearest_rank(sorted: &[u32], percent: usize) -> f64 {
    let rank = (percent * sorted.len()).div_ceil(100).max(1);
    sorted
        .get(rank - 1)
        .map_or(0.0, |us| f64::from(*us) / 1000.0)
}

/// When frames passed the point a stage starts at, by presentation time, so a later point of the
/// pipeline can tell how long a frame has been on its way.
#[derive(Debug, Default)]
pub struct Marks {
    marks: Mutex<VecDeque<(u64, Instant)>>,
}

impl Marks {
    pub fn mark(&self, pts: u64) {
        self.mark_at(pts, Instant::now());
    }

    /// Marks the frame with presentation time `pts` as having passed at `at`.
    pub fn mark_at(&self, pts: u64, at: Instant) {
        let mut marks = self.marks.lock().unwrap_or_else(PoisonError::into_inner);
        if marks.len() == IN_FLIGHT {
            marks.pop_front();
        }
        marks.push_back((pts, at));
    }

    /// When the frame with presentation time `pts` was marked, if it still is.
    pub fn at(&self, pts: u64) -> Option<Instant> {
        let marks = self.marks.lock().unwrap_or_else(PoisonError::into_inner);
        marks
            .iter()
            .rev()
            .find(|(marked, _)| *marked == pts)
            .map(|(_, at)| *at)
    }

    /// The time since the frame with presentation time `pts` was marked, if it still is.
    pub fn since(&self, pts: u64) -> Option<Duration> {
        self.at(pts).map(|at| at.elapsed())
    }
}

/// Frames inside an encoder that neither drops nor reorders them, in the order they went in: x264
/// without lookahead or B-frames gives out one picture per picture in, but with timestamps of its
/// own, so frames are matched by order.
#[derive(Debug, Default)]
pub struct InEncoder {
    frames: Mutex<VecDeque<(Instant, Instant)>>,
}

impl InEncoder {
    /// A frame stamped at `stamped` went into the encoder now.
    pub fn enter(&self, stamped: Instant) {
        let mut frames = self.frames.lock().unwrap_or_else(PoisonError::into_inner);
        if frames.len() == IN_FLIGHT {
            frames.pop_front();
        }
        frames.push_back((stamped, Instant::now()));
    }

    /// The oldest frame in the encoder came out: when it was stamped and the time it spent inside.
    pub fn leave(&self) -> Option<(Instant, Duration)> {
        let mut frames = self.frames.lock().unwrap_or_else(PoisonError::into_inner);
        frames
            .pop_front()
            .map(|(stamped, entered)| (stamped, entered.elapsed()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentiles_use_the_nearest_rank_and_the_window_forgets_old_samples() {
        let stage = Stage::default();
        for ms in 1..=100 {
            stage.record(Duration::from_millis(ms));
        }
        let snapshot = stage.snapshot();
        assert_eq!(
            (
                snapshot.n,
                snapshot.p50_ms,
                snapshot.p95_ms,
                snapshot.max_ms
            ),
            (100, 50.0, 95.0, 100.0)
        );
        for _ in 0..WINDOW {
            stage.record(Duration::from_millis(7));
        }
        let snapshot = stage.snapshot();
        assert_eq!((snapshot.n, snapshot.max_ms), (WINDOW, 7.0));
    }
}
