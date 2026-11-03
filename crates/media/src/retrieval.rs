//! What retrieval of retained detail asks the vehicle's adaptation before it sends.

use std::time::Instant;

use sat_teleop_policy::retrieval::Conditions;

use sat_teleop_video::adapt::{AdaptLimits, AdaptStats};

/// The link as retrieval needs it to decide (`sat_teleop_policy::retrieval::permit`).
pub fn conditions(
    stats: &AdaptStats,
    now: Instant,
    limits: &AdaptLimits,
    allowance: u32,
) -> Conditions {
    let since_answer_ms = stats.since_answer_ms(now);
    let s = stats.snapshot();
    Conditions {
        group: s.group_kind,
        draining: s.draining,
        since_answer_ms,
        stale_after_ms: limits.stale_after_ms,
        queue_delay_ms: s.queue_delay_ms,
        queue_delay_base_ms: limits.queue_delay_base_ms,
        allowance: if s.reserve_fits { allowance } else { 0 },
    }
}
