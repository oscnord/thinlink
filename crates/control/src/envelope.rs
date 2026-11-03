//! The control envelope protocol: JSON over UDP on the overlay between a controller (the ground
//! adapter on 10.77.0.3:7701) and the vehicle (10.77.0.2:7700), one message per datagram of at
//! most 1200 bytes. Every state-changing `MAVLink` message travels inside an envelope; the vehicle
//! checks it with `sat_teleop_policy::control` before anything reaches SITL.

use std::fmt;
use std::time::Duration;

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use sat_teleop_policy::control::{self as policy, Limits, Reason};
use sat_teleop_policy::region::NormRect;

use crate::ControlError;
use crate::frame::{Frame, datagram_frames};

/// Larger datagrams are refused unread.
pub const MAX_ENVELOPE: usize = 1200;
/// The vehicle sends the lease holder a new challenge this often.
pub const CHALLENGE_EVERY: Duration = Duration::from_millis(500);
/// The vehicle writes a fresh continuous setpoint to SITL at least this often.
pub const REFRESH_EVERY: Duration = Duration::from_millis(200);
/// The vehicle's control endpoint looks at its timers at least this often: the granularity of its
/// challenges and refreshes.
pub const TICK: Duration = Duration::from_millis(20);
/// The ground looks at what the stations asked to track, and sends the box, this often.
pub const TRACKING_TICK: Duration = Duration::from_millis(100);
/// Bounds every lifetime: a stale command must die within seconds, not minutes.
const MAX_LIFETIME_MS: u64 = 60_000;

/// Controller to vehicle.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ToVehicle {
    Envelope(Envelope),
    /// `peer` is informational: the overlay source address identifies the controller, and `sig`
    /// (`auth`, over the nonce) proves it holds the key.
    LeaseRequest {
        peer: String,
        nonce: u64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        sig: Option<String>,
    },
    /// `sig` is over the vehicle's session and the lease (`auth`).
    LeaseRelease {
        lease: u64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        sig: Option<String>,
    },
    /// Asks for the vehicle's clock bound; any listed controller may, lease or not.
    ClockQuery {
        nonce: u64,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Envelope {
    pub epoch: u64,
    pub lease: u64,
    pub seq: u64,
    /// The newest challenge token the controller received.
    pub challenge: u64,
    pub kind: Kind,
    /// One `MAVLink` 2 frame, base64. A liveness envelope without one only keeps the lease.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mavlink: Option<String>,
    /// A `Kind::Region` envelope's box; none clears it. Covered by the signature like every field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub region: Option<NormRect>,
    /// The controller's wall clock when it sent the envelope. Measurement only, never freshness.
    pub sent_unix_ms: u64,
    /// The controller's Ed25519 signature over every other field (`auth`), base64.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sig: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    /// A ground station's heartbeat, or nothing (keeps the lease).
    Liveness,
    /// A setpoint that replaces the previous one.
    Continuous,
    /// A command run at most once per transaction id.
    Discrete { txn: u64 },
    /// The pilot's box on the picture: carries no `MAVLink`, never reaches the autopilot.
    Region,
}

impl From<Kind> for policy::Kind {
    fn from(kind: Kind) -> Self {
        match kind {
            Kind::Liveness => Self::Liveness,
            Kind::Continuous => Self::Continuous,
            Kind::Discrete { .. } => Self::Discrete,
            Kind::Region => Self::Region,
        }
    }
}

/// Vehicle to controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ToGround {
    LeaseGrant {
        nonce: u64,
        lease: u64,
        epoch: u64,
    },
    LeaseDenied {
        nonce: u64,
        reason: Refusal,
    },
    Challenge {
        epoch: u64,
        token: u64,
        lifetime_ms: u64,
    },
    /// The answer to every discrete envelope.
    CommandResult {
        txn: u64,
        outcome: Outcome,
    },
    /// The answer to a `ClockQuery`: how far the vehicle's clock can be from true time right now,
    /// in milliseconds, or none when it has no bound it trusts.
    Clock {
        nonce: u64,
        bound_ms: Option<u64>,
    },
    /// What the vehicle now applies as the pilot's box, `None` when it has none: the answer to every
    /// accepted region envelope, and sent whenever the box is cleared.
    Region {
        region: Option<NormRect>,
    },
    /// The vehicle followed the pilot's box and lost it. The box is gone: the ground stops
    /// tracking, so tracking status goes idle and the pilot draws another. Sent again for a few
    /// seconds, so one dropped datagram does not leave the old box tracked; `region_seq` is the
    /// last region envelope the vehicle applied before the loss, so the ground ignores it once it
    /// holds a box sent after that.
    Lost {
        region_seq: u64,
    },
    /// A refused liveness, continuous or region envelope, lease release or unreadable datagram; `seq` when
    /// it could be read.
    Rejected {
        seq: Option<u64>,
        reason: Refusal,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    /// Written to SITL; SITL's own `COMMAND_ACK` follows in the telemetry.
    Applied,
    /// A retry of a transaction already applied: not written again. Its first `COMMAND_ACK` was
    /// evidently lost, so the controller answers the station itself.
    Repeated,
    Rejected {
        reason: Refusal,
    },
    /// Fresh, but not a command an envelope may carry.
    Unsupported,
}

/// Why the vehicle refused a lease request, a release, an envelope or a datagram: the policy's
/// reasons and the transport's, in snake case on the wire and in metrics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Refusal {
    /// Not on the vehicle's list of controllers.
    NotController,
    /// An envelope without a signature.
    Unsigned,
    /// A signature that is not the controller's over this envelope, or the vehicle holds no key
    /// for the controller.
    BadSignature,
    /// A signed lease request whose nonce is not higher than the controller's last accepted one.
    ReplayedRequest,
    LeaseHeld,
    NoLease,
    LeaseExpired,
    NotHolder,
    WrongLease,
    WrongEpoch,
    UnknownChallenge,
    ChallengeExpired,
    DuplicateSeq,
    ReorderedSeq,
    /// A known transaction id with a different command.
    TxnConflict,
    /// Larger than `MAX_ENVELOPE`.
    Oversize,
    /// Not a message of this protocol, or its `mavlink` is not one `MAVLink` 2 frame of the
    /// dialect.
    Malformed,
    /// A `MAVLink` message the envelope's kind may not carry.
    NotAllowlisted,
    /// SITL is not connected, or writing to it failed.
    NoDestination,
    /// A mode change the vehicle's type does not allow, or while its type is unknown.
    ModeNotAllowed,
    /// A copter mode change into a mode that takes the throttle from the sticks, with no fresh
    /// setpoint flowing. Defence in depth: the autopilot already holds its height in its radio
    /// failsafe.
    NoFreshSticks,
    /// A takeoff altitude that is not a positive number up to the gate's limit.
    TakeoffAltitude,
    /// A mission upload or start with `allow_missions` off, for anything but a copter, or of a
    /// mission type other than the waypoint mission.
    MissionNotAllowed,
    /// A mission item the gate does not allow: a command outside the flying ones, a frame not
    /// relative to home or an altitude outside 0 to 120 m.
    MissionItemNotAllowed,
    /// A mission upload announcing more items than the gate allows.
    MissionTooLong,
}

impl From<Reason> for Refusal {
    fn from(reason: Reason) -> Self {
        match reason {
            Reason::NotController => Self::NotController,
            Reason::LeaseHeld => Self::LeaseHeld,
            Reason::NoLease => Self::NoLease,
            Reason::LeaseExpired => Self::LeaseExpired,
            Reason::NotHolder => Self::NotHolder,
            Reason::WrongLease => Self::WrongLease,
            Reason::WrongEpoch => Self::WrongEpoch,
            Reason::UnknownChallenge => Self::UnknownChallenge,
            Reason::ChallengeExpired => Self::ChallengeExpired,
            Reason::DuplicateSeq => Self::DuplicateSeq,
            Reason::ReorderedSeq => Self::ReorderedSeq,
            Reason::TxnConflict => Self::TxnConflict,
        }
    }
}

impl fmt::Display for Refusal {
    /// The name on the wire.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = serde_json::to_string(self).map_err(|_| fmt::Error)?;
        f.write_str(name.trim_matches('"'))
    }
}

impl Refusal {
    /// After this refusal the controller no longer holds the lease and must request it again.
    pub fn ends_lease(self) -> bool {
        matches!(
            self,
            Self::NoLease
                | Self::LeaseExpired
                | Self::NotHolder
                | Self::WrongLease
                | Self::WrongEpoch
        )
    }
}

/// One protocol message from a datagram.
pub fn decode<T: DeserializeOwned>(datagram: &[u8]) -> Result<T, Refusal> {
    if datagram.len() > MAX_ENVELOPE {
        return Err(Refusal::Oversize);
    }
    serde_json::from_slice(datagram).map_err(|_| Refusal::Malformed)
}

/// A protocol message as one datagram.
pub fn encode(message: &impl Serialize) -> Result<Vec<u8>, ControlError> {
    let bytes = serde_json::to_vec(message).map_err(|e| ControlError::Encode(e.to_string()))?;
    if bytes.len() > MAX_ENVELOPE {
        return Err(ControlError::Encode(format!(
            "{} bytes, above {MAX_ENVELOPE}",
            bytes.len()
        )));
    }
    Ok(bytes)
}

/// `frame` for an envelope's `mavlink`.
pub fn encode_frame(frame: &Frame) -> String {
    STANDARD.encode(frame.bytes())
}

/// The frame an envelope's `mavlink` carries: exactly one `MAVLink` 2 frame of the dialect and
/// nothing else.
pub fn frame(text: &str) -> Option<Frame> {
    let bytes = STANDARD.decode(text).ok()?;
    let mut frames = datagram_frames(&bytes);
    let frame = frames.next()?;
    let whole = frames.next().is_none() && frame.bytes().len() == bytes.len();
    (whole && frame.is_v2()).then_some(frame)
}

/// `[control]` of `testbed/limits.toml`, in milliseconds. `docs/architecture.md` derives them.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlLimits {
    /// A challenge counts this long after the vehicle issued it.
    pub challenge_lifetime_ms: u64,
    /// The vehicle refreshes a continuous setpoint this long after it arrived, at most.
    pub setpoint_lifetime_ms: u64,
    /// The vehicle ends a lease this long after the holder's last valid envelope.
    pub lease_lifetime_ms: u64,
    /// The ground sends a liveness envelope when it has sent nothing for this long.
    pub liveness_interval_ms: u64,
    /// The ground treats a station's resend of a command within this long as the same transaction.
    pub ack_window_ms: u64,
    /// After the vehicle's last setpoint, SITL's throttle must be back to zero within this.
    pub throttle_release_deadline_ms: u64,
}

impl ControlLimits {
    pub fn validate(self) -> Result<Self, ControlError> {
        let challenge_every = duration_ms(CHALLENGE_EVERY);
        let rules = [
            (
                self.challenge_lifetime_ms > challenge_every,
                "challenge_lifetime_ms must exceed the 500 ms challenge interval",
            ),
            (
                self.setpoint_lifetime_ms > duration_ms(REFRESH_EVERY),
                "setpoint_lifetime_ms must exceed the 200 ms refresh interval",
            ),
            (
                self.setpoint_lifetime_ms <= self.challenge_lifetime_ms,
                "setpoint_lifetime_ms must not exceed challenge_lifetime_ms",
            ),
            (
                self.liveness_interval_ms >= challenge_every,
                "liveness_interval_ms must be at least the 500 ms challenge interval",
            ),
            (
                self.lease_lifetime_ms >= self.liveness_interval_ms.saturating_mul(2),
                "lease_lifetime_ms must cover two liveness intervals",
            ),
            (
                self.throttle_release_deadline_ms > self.setpoint_lifetime_ms,
                "throttle_release_deadline_ms must exceed setpoint_lifetime_ms",
            ),
            (
                [
                    self.challenge_lifetime_ms,
                    self.lease_lifetime_ms,
                    self.liveness_interval_ms,
                    self.ack_window_ms,
                    self.throttle_release_deadline_ms,
                ]
                .iter()
                .all(|&ms| ms <= MAX_LIFETIME_MS),
                "every value must be at most 60000 ms",
            ),
        ];
        match rules.iter().find(|(holds, _)| !holds) {
            Some((_, rule)) => Err(ControlError::Limits(rule)),
            None => Ok(self),
        }
    }

    pub fn policy(self) -> Limits {
        Limits {
            challenge_ms: self.challenge_lifetime_ms,
            setpoint_ms: self.setpoint_lifetime_ms,
            lease_ms: self.lease_lifetime_ms,
        }
    }
}

/// What the ground reads of `[region]` in `testbed/limits.toml`: how often the pilot's box is sent
/// while tracking, and how long a box the vehicle confirmed counts as applied. The vehicle's
/// sender reads the rest of the table.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct TrackingLimits {
    pub pilot_refresh_ms: u64,
    pub pilot_ttl_ms: u64,
}

impl TrackingLimits {
    pub fn validate(self) -> Result<Self, ControlError> {
        let rules = [
            (
                self.pilot_refresh_ms >= duration_ms(TRACKING_TICK),
                "pilot_refresh_ms must be at least the 100 ms tracking tick",
            ),
            (
                self.pilot_ttl_ms >= self.pilot_refresh_ms.saturating_mul(2),
                "pilot_ttl_ms must cover two refreshes",
            ),
            (
                self.pilot_ttl_ms <= MAX_LIFETIME_MS,
                "pilot_ttl_ms must be at most 60000 ms",
            ),
        ];
        match rules.iter().find(|(holds, _)| !holds) {
            Some((_, rule)) => Err(ControlError::Limits(rule)),
            None => Ok(self),
        }
    }
}

fn duration_ms(duration: Duration) -> u64 {
    u64::try_from(duration.as_millis()).unwrap_or(u64::MAX)
}
