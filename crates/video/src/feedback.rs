//! Media feedback inside the overlay: the vehicle sends the relay a probe every probe interval
//! (`10.77.0.1:5006/udp`) and the relay answers it at once with what its ingress received since its
//! previous answer. JSON, one message per datagram, at most [`MAX_DATAGRAM`] bytes; whatever does
//! not parse, or comes from anyone but the configured peer, is dropped and counted.

use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::ingress::{IngressSnapshot, IngressStats};
use crate::{Health, HealthSnapshot, MediaError};

/// Larger datagrams are dropped unread.
pub const MAX_DATAGRAM: usize = 512;

/// Vehicle to relay.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Probe {
    pub seq: u64,
    /// The vehicle's monotonic clock when it sent the probe; the relay only echoes it.
    pub sent_mono_us: u64,
}

/// Relay to vehicle: the answer to one probe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Report {
    pub probe_seq: u64,
    pub echo_sent_mono_us: u64,
    /// Since the relay's previous answer; the counts below are over this window.
    pub window_ms: u64,
    /// RTP bytes (headers included) and packets the ingress received.
    pub rx_bytes: u64,
    pub rx_packets: u64,
    /// Packets the jitter buffer gave up on that did not arrive late either: lost on the way, not
    /// reordered.
    pub lost_packets: u64,
    /// The jitter buffer's interarrival jitter.
    pub jitter_ms: u64,
}

/// One message from a datagram; `None` when it is too large or not that message.
pub fn decode<T: DeserializeOwned>(datagram: &[u8]) -> Option<T> {
    if datagram.len() > MAX_DATAGRAM {
        return None;
    }
    serde_json::from_slice(datagram).ok()
}

/// `[feedback]` of `testbed/config/relay.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FeedbackConfig {
    /// The relay's overlay address and feedback port.
    pub listen: SocketAddr,
    /// Only probes from this overlay address (the vehicle's) are answered, so nobody else can take
    /// the windows the vehicle measures with.
    pub vehicle: IpAddr,
}

#[derive(Debug, Default)]
pub struct FeedbackStats {
    pub health: Health,
    answered: AtomicU64,
    /// Datagrams that were no probe, or came from anyone but the vehicle.
    refused: AtomicU64,
    send_failed: AtomicU64,
}

#[derive(Debug, Serialize)]
pub struct FeedbackSnapshot {
    pub answered: u64,
    pub refused: u64,
    pub send_failed: u64,
    #[serde(flatten)]
    pub health: HealthSnapshot,
}

impl FeedbackStats {
    pub fn snapshot(&self) -> FeedbackSnapshot {
        let load = |counter: &AtomicU64| counter.load(Ordering::Relaxed);
        FeedbackSnapshot {
            answered: load(&self.answered),
            refused: load(&self.refused),
            send_failed: load(&self.send_failed),
            health: self.health.snapshot(),
        }
    }
}

/// The ingress counts at the relay's previous answer.
struct Window {
    at: Instant,
    ingress: IngressSnapshot,
}

/// Answers probes until the socket fails. Until `wg0` is up the overlay address cannot be bound,
/// so run it under [`crate::supervise`].
pub fn answer(
    config: &FeedbackConfig,
    ingress: &IngressStats,
    stats: &Arc<FeedbackStats>,
) -> Result<(), MediaError> {
    let socket = UdpSocket::bind(config.listen).map_err(|e| MediaError::Socket {
        addr: config.listen,
        reason: e.to_string(),
    })?;
    let mut window = Window {
        at: Instant::now(),
        ingress: ingress.snapshot(),
    };
    // One byte more than allowed, so an oversized datagram shows as such instead of truncated.
    let mut buf = [0_u8; MAX_DATAGRAM + 1];
    loop {
        let (len, from) = socket.recv_from(&mut buf).map_err(|e| MediaError::Socket {
            addr: config.listen,
            reason: e.to_string(),
        })?;
        let probe = buf
            .get(..len)
            .and_then(decode::<Probe>)
            .filter(|_| from.ip() == config.vehicle);
        let Some(probe) = probe else {
            stats.refused.fetch_add(1, Ordering::Relaxed);
            continue;
        };
        let now = Instant::now();
        let current = ingress.snapshot();
        let report = report(probe, &window, now, &current);
        window = Window {
            at: now,
            ingress: current,
        };
        let sent = serde_json::to_vec(&report)
            .ok()
            .and_then(|bytes| socket.send_to(&bytes, from).ok());
        let counter = if sent.is_some() {
            &stats.answered
        } else {
            &stats.send_failed
        };
        counter.fetch_add(1, Ordering::Relaxed);
    }
}

fn report(probe: Probe, window: &Window, now: Instant, current: &IngressSnapshot) -> Report {
    let previous = &window.ingress;
    let lost = |s: &IngressSnapshot| s.lost.saturating_sub(s.late);
    Report {
        probe_seq: probe.seq,
        echo_sent_mono_us: probe.sent_mono_us,
        window_ms: u64::try_from(now.saturating_duration_since(window.at).as_millis())
            .unwrap_or(u64::MAX),
        rx_bytes: current.rtp_bytes.saturating_sub(previous.rtp_bytes),
        rx_packets: current.rtp_packets.saturating_sub(previous.rtp_packets),
        lost_packets: lost(current).saturating_sub(lost(previous)),
        jitter_ms: current.avg_jitter_us / 1000,
    }
}
