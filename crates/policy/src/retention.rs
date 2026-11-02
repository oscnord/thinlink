//! Retention of the vehicle's retained snapshots: which to delete so the store stays within its
//! byte quota, its maximum age and its free-space floor, while the snapshots taken out of reach
//! are kept for the operator up to a share of the quota.

use std::time::Instant;

/// One retained snapshot.
#[derive(Debug, Clone, Copy)]
pub struct Object {
    /// When it was captured, in UNIX milliseconds.
    pub capture_ms: u64,
    pub bytes: u64,
    /// A retrieval is reading it: kept until then, whatever the limits say.
    pub pinned_until: Option<Instant>,
    /// Taken while the ground was not heard and not fetched since.
    pub outage: bool,
}

impl Object {
    fn pinned(&self, now: Instant) -> bool {
        self.pinned_until.is_some_and(|until| until > now)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Limits {
    pub max_bytes: u64,
    pub max_age_ms: u64,
    /// The volume keeps at least this much free.
    pub min_free_bytes: u64,
    /// Outage snapshots are kept against the limits above up to this percentage of `max_bytes`.
    pub outage_share_percent: u8,
    /// An outage snapshot is kept this long after it was captured.
    pub outage_keep_ms: u64,
}

/// The outage snapshots that retention keeps: unfetched and within `outage_keep_ms`, the newest
/// first until the next would take them over their share of the quota.
fn kept_for_outage(objects: &[Object], limits: Limits, now_unix_ms: u64) -> Vec<bool> {
    let share = limits.max_bytes / 100 * u64::from(limits.outage_share_percent);
    let mut newest_first: Vec<usize> = (0..objects.len()).collect();
    newest_first.sort_by_key(|&i| std::cmp::Reverse(objects.get(i).map(|o| o.capture_ms)));
    let mut kept = vec![false; objects.len()];
    let mut held: u64 = 0;
    for index in newest_first {
        let Some(object) = objects.get(index) else {
            continue;
        };
        let young = now_unix_ms.saturating_sub(object.capture_ms) <= limits.outage_keep_ms;
        if !object.outage || !young {
            continue;
        }
        held = held.saturating_add(object.bytes);
        if held > share {
            break;
        }
        if let Some(flag) = kept.get_mut(index) {
            *flag = true;
        }
    }
    kept
}

/// Indices into `objects` to delete, oldest first: every object older than `max_age_ms`, then the
/// oldest until the store holds at most `max_bytes` and the volume (`free_bytes` free now) at least
/// `min_free_bytes`. Pinned objects are skipped, so a store whose limits only pinned objects could
/// meet stays over them until the pins expire. Outage snapshots are skipped too, up to their
/// share of the quota and within `outage_keep_ms`, whatever `max_age_ms` says.
pub fn evict(
    objects: &[Object],
    limits: Limits,
    free_bytes: u64,
    now_unix_ms: u64,
    now: Instant,
) -> Vec<usize> {
    let outage_kept = kept_for_outage(objects, limits, now_unix_ms);
    let mut oldest_first: Vec<usize> = (0..objects.len()).collect();
    oldest_first.sort_by_key(|&i| objects.get(i).map(|o| o.capture_ms));
    let mut stored: u64 = objects.iter().map(|o| o.bytes).sum();
    let mut free = free_bytes;
    let mut evicted = Vec::new();
    for index in oldest_first {
        let kept = outage_kept.get(index).copied().unwrap_or(false);
        let Some(object) = objects.get(index).filter(|o| !o.pinned(now) && !kept) else {
            continue;
        };
        let too_old = now_unix_ms.saturating_sub(object.capture_ms) > limits.max_age_ms;
        let over = stored > limits.max_bytes || free < limits.min_free_bytes;
        if !too_old && !over {
            break;
        }
        stored = stored.saturating_sub(object.bytes);
        free = free.saturating_add(object.bytes);
        evicted.push(index);
    }
    evicted
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    type Case = (&'static str, Vec<Object>, Limits, u64, Vec<usize>);

    const NOW_MS: u64 = 1_000_000;
    const LIMITS: Limits = Limits {
        max_bytes: 300,
        max_age_ms: 60_000,
        min_free_bytes: 1000,
        outage_share_percent: 50,
        outage_keep_ms: 600_000,
    };

    #[test]
    fn evicts_the_oldest_unpinned_until_every_limit_holds() {
        let now = Instant::now();
        let pinned = |secs: u64| Some(now + Duration::from_secs(secs));
        let object = |age_ms: u64, bytes, pinned_until| Object {
            capture_ms: NOW_MS - age_ms,
            bytes,
            pinned_until,
            outage: false,
        };
        // Name, objects as (age in ms, bytes, pinned until), limits, free bytes and the indices
        // evicted, in eviction order.
        let cases: Vec<Case> = vec![
            (
                "within every limit",
                vec![object(30_000, 100, None), object(10_000, 100, None)],
                LIMITS,
                5000,
                vec![],
            ),
            (
                "over the byte quota: the oldest go until it holds",
                vec![
                    object(30_000, 100, None),
                    object(20_000, 100, None),
                    object(10_000, 100, None),
                    object(1000, 100, None),
                ],
                LIMITS,
                5000,
                vec![0],
            ),
            (
                "older than the maximum age, though under the quota",
                vec![object(61_000, 10, None), object(59_000, 10, None)],
                LIMITS,
                5000,
                vec![0],
            ),
            (
                "below the free-space floor: each eviction frees its bytes",
                vec![
                    object(30_000, 100, None),
                    object(20_000, 100, None),
                    object(10_000, 100, None),
                ],
                LIMITS,
                850,
                vec![0, 1],
            ),
            (
                "input order does not matter",
                vec![
                    object(1000, 100, None),
                    object(30_000, 100, None),
                    object(10_000, 100, None),
                    object(20_000, 100, None),
                ],
                LIMITS,
                5000,
                vec![1],
            ),
            (
                "a pinned object is skipped, the next oldest goes",
                vec![
                    object(30_000, 100, pinned(5)),
                    object(20_000, 100, None),
                    object(10_000, 100, None),
                    object(1000, 100, None),
                ],
                LIMITS,
                5000,
                vec![1],
            ),
            (
                "a pin that ran out no longer protects",
                vec![
                    object(30_000, 100, Some(now)),
                    object(20_000, 100, None),
                    object(10_000, 100, None),
                    object(1000, 100, None),
                ],
                LIMITS,
                5000,
                vec![0],
            ),
            (
                "everything pinned: nothing goes, over the limits",
                vec![
                    object(61_000, 200, pinned(5)),
                    object(10_000, 200, pinned(5)),
                ],
                LIMITS,
                10,
                vec![],
            ),
        ];
        for (name, objects, limits, free, expected) in cases {
            let evicted = evict(&objects, limits, free, NOW_MS, now);
            assert_eq!(evicted, expected, "{name}");
        }
    }

    #[test]
    fn outage_snapshots_outlive_the_limits_within_their_share_until_fetched_or_old() {
        let now = Instant::now();
        let object = |age_ms: u64, outage| Object {
            capture_ms: NOW_MS - age_ms,
            bytes: 100,
            pinned_until: None,
            outage,
        };
        // Name, objects as (age in ms, outage), free bytes and the indices evicted, oldest first.
        // The quota holds 300 bytes and the outage share 150, one snapshot of 100.
        let cases: Vec<(&str, Vec<Object>, u64, Vec<usize>)> = vec![
            (
                "past the maximum age, an outage snapshot stays and an ordinary one goes",
                vec![object(90_000, true), object(80_000, false)],
                5000,
                vec![1],
            ),
            (
                "over the quota the ordinary snapshots go before the outage ones",
                vec![
                    object(40_000, true),
                    object(30_000, false),
                    object(20_000, false),
                    object(10_000, false),
                ],
                5000,
                vec![1],
            ),
            (
                "over their share the newest outage snapshot wins and the older goes",
                vec![
                    object(30_000, true),
                    object(20_000, true),
                    object(10_000, false),
                    object(1000, false),
                ],
                5000,
                vec![0],
            ),
            (
                "fetched snapshots are ordinary again",
                vec![object(90_000, false), object(80_000, true)],
                5000,
                vec![0],
            ),
            (
                "an outage snapshot older than the keep time is ordinary again",
                vec![object(700_000, true), object(10_000, false)],
                5000,
                vec![0],
            ),
        ];
        for (name, objects, free, expected) in cases {
            let evicted = evict(&objects, LIMITS, free, NOW_MS, now);
            assert_eq!(evicted, expected, "{name}");
        }
    }
}
