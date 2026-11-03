//! The vehicle's telemetry budget. The autopilot's heartbeat always goes out, the vehicle state
//! next, keeping the lower half of a one-second token bucket. Replies to requests the gate admits wait in a bounded queue when the
//! budget runs short, since `ArduPilot` sends a whole parameter list at once (about 46 KB in 0.6 s
//! over SITL's TCP port). Everything else is dropped when over budget or while replies wait.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use mavlink::MessageData;
#[expect(
    deprecated,
    reason = "ArduPilot answers MISSION_REQUEST with MISSION_ITEM"
)]
use mavlink::dialects::ardupilotmega::{
    ATTITUDE_DATA, AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA, GLOBAL_POSITION_INT_DATA,
    HEARTBEAT_DATA, MISSION_ACK_DATA, MISSION_COUNT_DATA, MISSION_CURRENT_DATA, MISSION_ITEM_DATA,
    MISSION_ITEM_INT_DATA, MISSION_ITEM_REACHED_DATA, MISSION_REQUEST_DATA,
    MISSION_REQUEST_INT_DATA, PARAM_VALUE_DATA, SYS_STATUS_DATA,
};
use serde::Deserialize;

use sat_teleop_video_core::rate::{Priority, TokenBucket};

use crate::ControlError;
use crate::frame::Frame;

const STATE: [u32; 6] = [
    SYS_STATUS_DATA::ID,
    GLOBAL_POSITION_INT_DATA::ID,
    ATTITUDE_DATA::ID,
    COMMAND_ACK_DATA::ID,
    MISSION_CURRENT_DATA::ID,
    MISSION_ITEM_REACHED_DATA::ID,
];

#[expect(
    deprecated,
    reason = "ArduPilot answers MISSION_REQUEST with MISSION_ITEM"
)]
const REPLIES: [u32; 8] = [
    AUTOPILOT_VERSION_DATA::ID,
    PARAM_VALUE_DATA::ID,
    MISSION_COUNT_DATA::ID,
    MISSION_ITEM_INT_DATA::ID,
    MISSION_ITEM_DATA::ID,
    MISSION_ACK_DATA::ID,
    MISSION_REQUEST_INT_DATA::ID,
    MISSION_REQUEST_DATA::ID,
];

/// Holds a full `ArduRover` parameter list with room to spare.
const MAX_WAITING_BYTES: usize = 64 * 1024;
/// A reply that waited longer is stale: the ground station will have asked again.
const MAX_WAIT: Duration = Duration::from_secs(10);

const BUDGET_RANGE: std::ops::RangeInclusive<u32> = 1_000..=1_000_000;

/// `[telemetry]` of `testbed/limits.toml`, in `MAVLink` frame bytes per second from the vehicle
/// to the relay.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TelemetryLimits {
    /// The cap, and the cap while the uplink is unmeasured.
    pub bytes_per_s: u32,
    /// The uplink budget gives telemetry this percentage of what SITL offers, between the floor
    /// and the cap.
    pub offered_percent: u32,
    /// The vehicle's state at one message a second: the cap never goes below it.
    pub essential_bytes_per_s: u32,
}

impl TelemetryLimits {
    pub fn validate(self) -> Result<Self, ControlError> {
        let rules = [
            (
                BUDGET_RANGE.contains(&self.bytes_per_s),
                "bytes_per_s must be 1000-1000000",
            ),
            (
                (100..=1000).contains(&self.offered_percent),
                "offered_percent must be 100-1000",
            ),
            (
                (1..=self.bytes_per_s).contains(&self.essential_bytes_per_s),
                "essential_bytes_per_s must be 1 up to bytes_per_s",
            ),
        ];
        match rules.iter().find(|(holds, _)| !holds) {
            Some((_, rule)) => Err(ControlError::Telemetry(rule)),
            None => Ok(self),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Class {
    /// The autopilot's heartbeat: spends the budget when it can and goes out even when it cannot.
    /// A ground station declares the link lost after 3.5 s without a vehicle message, and the
    /// essential floor of the budget counts the heartbeat's 21 bytes a second.
    Heartbeat,
    /// The vehicle's state and command acknowledgements: may use the whole budget.
    State,
    /// Answers to admitted requests: wait for the upper half of the budget.
    Reply,
    /// Everything else: the upper half of the budget, dropped while replies wait.
    Bulk,
}

pub fn class(message_id: u32) -> Class {
    if message_id == HEARTBEAT_DATA::ID {
        return Class::Heartbeat;
    }
    if STATE.contains(&message_id) {
        return Class::State;
    }
    if REPLIES.contains(&message_id) {
        return Class::Reply;
    }
    Class::Bulk
}

#[expect(
    clippy::large_enum_variant,
    reason = "a frame lives only until it is sent; boxing would allocate for every frame"
)]
pub enum Admission {
    Send(Frame),
    /// A reply waiting for budget; [`RateCap::release`] hands it out later.
    Queued,
    Dropped,
}

#[expect(
    clippy::large_enum_variant,
    reason = "a frame lives only until it is sent; boxing would allocate for every frame"
)]
pub enum Released {
    Send(Frame),
    /// A reply that waited longer than the limit, now dropped.
    Expired,
}

pub struct RateCap {
    bucket: TokenBucket,
    bytes_per_s: u32,
    waiting: VecDeque<(Instant, Frame)>,
    waiting_bytes: usize,
}

impl RateCap {
    pub fn new(limits: TelemetryLimits) -> Self {
        Self {
            bucket: TokenBucket::new(limits.bytes_per_s),
            bytes_per_s: limits.bytes_per_s,
            waiting: VecDeque::new(),
            waiting_bytes: 0,
        }
    }

    /// Changes the cap; a no-op when it is the cap already.
    pub fn set_rate(&mut self, bytes_per_s: u32) {
        if bytes_per_s == self.bytes_per_s {
            return;
        }
        self.bytes_per_s = bytes_per_s;
        self.bucket.set_rate(bytes_per_s);
    }

    /// What to do with a frame from SITL at `now`.
    pub fn admit(&mut self, now: Instant, frame: Frame) -> Admission {
        match class(frame.message_id()) {
            Class::Heartbeat => {
                self.bucket.take(now, frame.size(), Priority::Preferred);
                Admission::Send(frame)
            }
            Class::State => self.spend(now, frame, Priority::Preferred),
            Class::Bulk if !self.waiting.is_empty() => Admission::Dropped,
            Class::Bulk => self.spend(now, frame, Priority::Bulk),
            Class::Reply => self.send_or_wait(now, frame),
        }
    }

    /// Takes room for a frame of `bytes` that is not telemetry of the autopilot's: it uses the upper
    /// half of the budget only, and never while replies wait.
    pub fn take_bulk(&mut self, now: Instant, bytes: u32) -> bool {
        self.waiting.is_empty() && self.bucket.take(now, bytes, Priority::Bulk)
    }

    /// The oldest waiting reply, once it fits the budget or has waited too long.
    pub fn release(&mut self, now: Instant) -> Option<Released> {
        let (since, frame) = self.waiting.front()?;
        let expired = now.saturating_duration_since(*since) > MAX_WAIT;
        if !expired && !self.bucket.take(now, frame.size(), Priority::Bulk) {
            return None;
        }
        let (_, frame) = self.waiting.pop_front()?;
        self.waiting_bytes = self.waiting_bytes.saturating_sub(frame.bytes().len());
        Some(if expired {
            Released::Expired
        } else {
            Released::Send(frame)
        })
    }

    /// Drops every waiting reply and returns how many there were: replies to a SITL connection
    /// that is gone must not go out late on the next one.
    pub fn clear(&mut self) -> usize {
        let waiting = self.waiting.len();
        self.waiting.clear();
        self.waiting_bytes = 0;
        waiting
    }

    fn spend(&mut self, now: Instant, frame: Frame, priority: Priority) -> Admission {
        if self.bucket.take(now, frame.size(), priority) {
            Admission::Send(frame)
        } else {
            Admission::Dropped
        }
    }

    /// Replies keep their order: one goes straight out only when none waits and it fits.
    fn send_or_wait(&mut self, now: Instant, frame: Frame) -> Admission {
        if self.waiting.is_empty() && self.bucket.take(now, frame.size(), Priority::Bulk) {
            return Admission::Send(frame);
        }
        self.wait(now, frame)
    }

    fn wait(&mut self, now: Instant, frame: Frame) -> Admission {
        let waiting_bytes = self.waiting_bytes + frame.bytes().len();
        if waiting_bytes > MAX_WAITING_BYTES {
            return Admission::Dropped;
        }
        self.waiting_bytes = waiting_bytes;
        self.waiting.push_back((now, frame));
        Admission::Queued
    }
}

#[cfg(test)]
mod tests {
    use mavlink::dialects::ardupilotmega::{
        ATTITUDE_DATA, HEARTBEAT_DATA, MavMessage, PARAM_VALUE_DATA, SYSTEM_TIME_DATA,
    };

    use super::*;
    use crate::frame::encoded;

    #[test]
    #[expect(
        deprecated,
        reason = "ArduPilot asks for upload items with MISSION_REQUEST"
    )]
    fn a_mission_uploads_requests_and_acks_are_never_bulk() {
        let ids = [
            (MISSION_REQUEST_DATA::ID, Class::Reply),
            (MISSION_REQUEST_INT_DATA::ID, Class::Reply),
            (MISSION_ACK_DATA::ID, Class::Reply),
            (MISSION_CURRENT_DATA::ID, Class::State),
            (MISSION_ITEM_REACHED_DATA::ID, Class::State),
        ];
        for (id, expected) in ids {
            assert_eq!(class(id), expected, "message {id}");
        }
    }

    fn param() -> Frame {
        encoded(&MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_count: 1256,
            param_index: 7,
            ..PARAM_VALUE_DATA::default()
        }))
    }

    /// Releases everything due at `now`: (sent, expired).
    fn release_all(cap: &mut RateCap, now: Instant) -> (usize, usize) {
        let (mut sent, mut expired) = (0, 0);
        while let Some(released) = cap.release(now) {
            match released {
                Released::Send(_) => sent += 1,
                Released::Expired => expired += 1,
            }
        }
        (sent, expired)
    }

    #[test]
    fn a_reply_burst_waits_within_bounds_while_bulk_yields_to_it() {
        let t0 = Instant::now();
        let mut cap = RateCap::new(TelemetryLimits {
            bytes_per_s: 1000,
            offered_percent: 200,
            essential_bytes_per_s: 200,
        });
        let size = param().bytes().len();
        let (mut sent, mut queued, mut dropped) = (0, 0, 0);
        for _ in 0..MAX_WAITING_BYTES / size + 100 {
            match cap.admit(t0, param()) {
                Admission::Send(_) => sent += 1,
                Admission::Queued => queued += 1,
                Admission::Dropped => dropped += 1,
            }
        }
        assert!(
            sent > 0 && sent * size <= 500,
            "replies use the upper half only"
        );
        assert_eq!(
            queued,
            MAX_WAITING_BYTES / size,
            "replies wait up to the byte bound"
        );
        assert!(dropped > 0);
        let bulk = encoded(&MavMessage::SYSTEM_TIME(SYSTEM_TIME_DATA::default()));
        assert!(matches!(cap.admit(t0, bulk), Admission::Dropped));
        let heartbeat = || encoded(&MavMessage::HEARTBEAT(HEARTBEAT_DATA::default()));
        assert!(matches!(cap.admit(t0, heartbeat()), Admission::Send(_)));
        let state = || encoded(&MavMessage::ATTITUDE(ATTITUDE_DATA::default()));
        while matches!(cap.admit(t0, state()), Admission::Send(_)) {}
        assert!(
            matches!(cap.admit(t0, heartbeat()), Admission::Send(_)),
            "the heartbeat goes out with the bucket empty"
        );
        assert_eq!(release_all(&mut cap, t0), (0, 0));
        let (released, expired) = release_all(&mut cap, t0 + Duration::from_secs(1));
        assert!(released > 0 && released * size <= 500 && expired == 0);
        let late = t0 + MAX_WAIT + Duration::from_millis(1);
        assert_eq!(release_all(&mut cap, late), (0, queued - released));
        assert!(matches!(cap.admit(late, param()), Admission::Send(_)));
    }
}
