//! Byte-rate budgets: a token bucket that keeps headroom for preferred traffic.

use std::time::Instant;

/// How much of the bucket a message may use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Priority {
    /// May spend the bucket down to empty.
    Preferred,
    /// May only spend what lies above the reserve, so it is refused first when over budget.
    Bulk,
}

/// A token bucket in bytes. It refills at `bytes_per_s` up to one second of budget; bulk traffic
/// leaves the lower half of that to preferred traffic.
#[derive(Debug, Clone)]
pub struct TokenBucket {
    bytes_per_s: f64,
    /// The most the bucket holds; one second of budget unless set otherwise.
    burst: Option<f64>,
    tokens: f64,
    refilled_at: Option<Instant>,
}

impl TokenBucket {
    /// A full bucket.
    pub fn new(bytes_per_s: u32) -> Self {
        let bytes_per_s = f64::from(bytes_per_s);
        Self {
            bytes_per_s,
            burst: None,
            tokens: bytes_per_s,
            refilled_at: None,
        }
    }

    /// A bucket that holds at most `burst_bytes`, full.
    pub fn with_burst(bytes_per_s: u32, burst_bytes: u32) -> Self {
        Self {
            burst: Some(f64::from(burst_bytes)),
            tokens: f64::from(burst_bytes),
            ..Self::new(bytes_per_s)
        }
    }

    fn capacity(&self) -> f64 {
        self.burst.unwrap_or(self.bytes_per_s)
    }

    /// Refills at `bytes_per_s` from now on, holding at most the bucket's burst.
    pub fn set_rate(&mut self, bytes_per_s: u32) {
        self.bytes_per_s = f64::from(bytes_per_s);
        self.tokens = self.tokens.min(self.capacity());
    }

    /// Takes `bytes` at `now` when the budget allows it for `priority`; otherwise takes nothing.
    pub fn take(&mut self, now: Instant, bytes: u32, priority: Priority) -> bool {
        self.refill(now);
        let floor = match priority {
            Priority::Preferred => 0.0,
            Priority::Bulk => self.bytes_per_s / 2.0,
        };
        let left = self.tokens - f64::from(bytes);
        if left < floor {
            return false;
        }
        self.tokens = left;
        true
    }

    fn refill(&mut self, now: Instant) {
        let elapsed = self
            .refilled_at
            .map_or(0.0, |at| now.saturating_duration_since(at).as_secs_f64());
        self.tokens = (self.tokens + elapsed * self.bytes_per_s).min(self.capacity());
        self.refilled_at = Some(self.refilled_at.map_or(now, |at| at.max(now)));
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    #[test]
    fn bulk_is_refused_before_preferred_traffic() {
        use Priority::{Bulk, Preferred};
        let t0 = Instant::now();
        let mut bucket = TokenBucket::new(1000);
        // (ms after t0, bytes, priority, granted)
        let steps = [
            (0, 500, Bulk, true),
            (0, 1, Bulk, false),
            (0, 500, Preferred, true),
            (0, 1, Preferred, false),
            // 400 ms refill 400 bytes: below the bulk reserve, usable by preferred traffic.
            (400, 1, Bulk, false),
            (400, 400, Preferred, true),
            // Refill stops at one second of budget.
            (10_000, 500, Bulk, true),
            (10_000, 1, Bulk, false),
            // Time going backwards refills nothing.
            (5_000, 1, Preferred, true),
            (5_000, 500, Preferred, false),
        ];
        for (at_ms, bytes, priority, granted) in steps {
            let now = t0 + Duration::from_millis(at_ms);
            assert_eq!(
                bucket.take(now, bytes, priority),
                granted,
                "{at_ms} ms, {bytes} bytes, {priority:?}"
            );
        }
    }
}
