//! Retrieval of retained detail yields to live video and control: when it may send, what a chunk
//! costs on the wire and how large a chunk the allowance carries.

use crate::adapt::Group;
use crate::budget::DATAGRAM_OVERHEAD;

/// The largest chunk of an upload.
pub const MAX_CHUNK: usize = 16 * 1024;
const MIN_CHUNK: usize = 512;
/// An HTTP request's line and headers around a chunk, rounded up.
const REQUEST_HEADERS: usize = 320;
/// Payload of a `WireGuard` packet carrying TCP: the tunnel's 1420-byte MTU less IPv4 and TCP.
const SEGMENT: usize = 1380;
/// TCP's header is 12 bytes longer than the UDP header `DATAGRAM_OVERHEAD` counts.
const TCP_EXTRA: u32 = 12;

/// What the vehicle knows of the link and of adaptation when retrieval asks to send.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Conditions {
    /// The rung's group, `None` before adaptation runs.
    pub group: Option<Group>,
    /// The video is paused while the uplink queue drains.
    pub draining: bool,
    /// Milliseconds since the relay last answered a probe; `None` if it never did.
    pub since_answer_ms: Option<u64>,
    pub stale_after_ms: u64,
    pub queue_delay_ms: Option<u64>,
    /// A queue at most this long is no queue (`queue_delay_base_ms` of the adaptation limits).
    pub queue_delay_base_ms: u64,
    /// What retrieval may send, in wire bytes per second.
    pub allowance: u32,
}

/// Why retrieval holds back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hold {
    NoAllowance,
    NotNormal,
    Draining,
    FeedbackStale,
    Queueing,
}

impl Hold {
    pub fn name(self) -> &'static str {
        match self {
            Self::NoAllowance => "no_allowance",
            Self::NotNormal => "not_normal",
            Self::Draining => "draining",
            Self::FeedbackStale => "feedback_stale",
            Self::Queueing => "queueing",
        }
    }
}

/// Whether retrieval may send its next chunk: only with an allowance, at a normal rung, while the
/// relay's feedback is fresh and shows no queue. A queue is the first sign retrieval is costing
/// the live video, so it stops itself on one; unknown feedback is no permission.
pub fn permit(c: &Conditions) -> Result<(), Hold> {
    if c.allowance == 0 {
        return Err(Hold::NoAllowance);
    }
    if c.group != Some(Group::Normal) {
        return Err(Hold::NotNormal);
    }
    if c.draining {
        return Err(Hold::Draining);
    }
    if c.since_answer_ms.is_none_or(|ms| ms >= c.stale_after_ms) {
        return Err(Hold::FeedbackStale);
    }
    if c.queue_delay_ms.is_none_or(|ms| ms > c.queue_delay_base_ms) {
        return Err(Hold::Queueing);
    }
    Ok(())
}

/// The chunk size for an allowance: what it carries in half a second, so a chunk fits the token
/// bucket (one second of the allowance) with room, within `MAX_CHUNK`.
pub fn chunk_size(allowance: u32) -> usize {
    let half_second = usize::try_from(allowance / 2).unwrap_or(MAX_CHUNK);
    half_second.clamp(MIN_CHUNK, MAX_CHUNK)
}

/// A conservative cost on the wire of one HTTP request with `payload` body bytes: the request,
/// the segments it travels in and the acknowledgement of the response.
pub fn request_wire_bytes(payload: usize) -> u32 {
    let total = payload.saturating_add(REQUEST_HEADERS);
    let segments = u32::try_from(total.div_ceil(SEGMENT)).unwrap_or(u32::MAX);
    let per_packet = DATAGRAM_OVERHEAD + TCP_EXTRA;
    let bytes = u32::try_from(total).unwrap_or(u32::MAX);
    bytes.saturating_add(segments.saturating_add(1).saturating_mul(per_packet))
}
