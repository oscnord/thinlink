//! Counters and link state the bridges report on `GET /metrics`.

use std::collections::BTreeMap;
use std::fmt::Display;
use std::io;
use std::sync::Mutex;

use serde::Serialize;

use crate::frame::Frame;

/// The same receive error is logged again after this many repeats (a minute at the 100 ms pause
/// the bridges take after one).
const LOG_EVERY: u64 = 600;

/// Why a bridge did not forward a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DropReason {
    /// Over the telemetry byte budget, or a reply with the waiting queue full.
    RateLimited,
    /// A reply that waited for budget longer than the limit, or whose SITL connection is gone.
    Expired,
    /// A message type the gate does not admit.
    NotAllowlisted,
    /// `COMMAND_LONG` or `COMMAND_INT` with a command the gate does not admit.
    CommandNotAllowlisted,
    /// CRC-valid, but a field holds a value the dialect does not define.
    Undecodable,
    /// A datagram from anyone but the expected peer; counted per datagram.
    UnknownSource,
    /// Nowhere to send it yet: SITL not connected, mavlink-router not heard from.
    NoDestination,
    SendFailed,
}

#[derive(Debug, Default, Clone, Copy, Serialize)]
pub struct Drops {
    pub rate_limited: u64,
    pub expired: u64,
    pub not_allowlisted: u64,
    pub command_not_allowlisted: u64,
    pub undecodable: u64,
    pub unknown_source: u64,
    pub no_destination: u64,
    pub send_failed: u64,
}

/// One direction through a bridge. Frames are CRC-valid `MAVLink` frames; bytes are their size on
/// the wire, without UDP, IP or `WireGuard` headers. A frame sent to two ground stations counts
/// twice out.
#[derive(Debug, Default, Clone, Copy, Serialize)]
pub struct FlowCounts {
    pub messages_in: u64,
    pub bytes_in: u64,
    pub messages_out: u64,
    pub bytes_out: u64,
    pub dropped: Drops,
    /// Failed reads on the socket this direction's input arrives on.
    pub receive_errors: u64,
}

#[derive(Debug, Default)]
pub struct Flow {
    counts: Mutex<FlowCounts>,
    last_receive_error: Mutex<Option<String>>,
}

impl Flow {
    pub fn received(&self, frame: &Frame) {
        self.update(|c| {
            c.messages_in += 1;
            c.bytes_in += u64::from(frame.size());
        });
    }

    pub fn forwarded(&self, frame: &Frame) {
        self.update(|c| {
            c.messages_out += 1;
            c.bytes_out += u64::from(frame.size());
        });
    }

    pub fn dropped(&self, reason: DropReason) {
        self.update(|c| {
            let d = &mut c.dropped;
            let counter = match reason {
                DropReason::RateLimited => &mut d.rate_limited,
                DropReason::Expired => &mut d.expired,
                DropReason::NotAllowlisted => &mut d.not_allowlisted,
                DropReason::CommandNotAllowlisted => &mut d.command_not_allowlisted,
                DropReason::Undecodable => &mut d.undecodable,
                DropReason::UnknownSource => &mut d.unknown_source,
                DropReason::NoDestination => &mut d.no_destination,
                DropReason::SendFailed => &mut d.send_failed,
            };
            *counter += 1;
        });
    }

    /// Counts a failed read on `socket`. It is logged when it differs from the last one logged,
    /// and otherwise every `LOG_EVERY` repeats, so a socket failing in a loop cannot flood stderr.
    pub fn receive_failed(&self, socket: &str, error: &io::Error) {
        let mut errors = 0;
        self.update(|c| {
            c.receive_errors += 1;
            errors = c.receive_errors;
        });
        let message = error.to_string();
        let Ok(mut last) = self.last_receive_error.lock() else {
            return;
        };
        if last.as_deref() == Some(message.as_str()) && errors % LOG_EVERY != 0 {
            return;
        }
        eprintln!("mavlink: cannot read from {socket} ({errors} errors so far): {message}");
        *last = Some(message);
    }

    pub fn snapshot(&self) -> FlowCounts {
        self.counts.lock().map(|c| *c).unwrap_or_default()
    }

    fn update(&self, f: impl FnOnce(&mut FlowCounts)) {
        if let Ok(mut counts) = self.counts.lock() {
            f(&mut counts);
        }
    }
}

/// Drops counted by what was dropped: a message name such as `PARAM_SET`, a refused command such
/// as `COMMAND_LONG MAV_CMD_DO_SET_MODE`, or `undecodable <id>`. Keys come from the dialect's
/// names and ids (a CRC-valid frame has an id the dialect defines), so the map stays bounded.
#[derive(Debug, Default)]
pub struct DropsByMessage(Mutex<BTreeMap<String, u64>>);

impl DropsByMessage {
    pub fn count(&self, key: String) {
        if let Ok(mut drops) = self.0.lock() {
            *drops.entry(key).or_default() += 1;
        }
    }

    pub fn snapshot(&self) -> BTreeMap<String, u64> {
        self.0.lock().map(|d| d.clone()).unwrap_or_default()
    }
}

/// A connection or socket a bridge depends on. After a failure it reports down with the error
/// until it is up again.
#[derive(Debug, Default)]
pub struct Link(Mutex<LinkState>);

#[derive(Debug, Default, Clone, Serialize)]
pub struct LinkState {
    pub up: bool,
    /// Times it came up; above 1 means it failed and recovered.
    pub ups: u64,
    pub last_error: Option<String>,
}

impl Link {
    pub fn up(&self) {
        if let Ok(mut state) = self.0.lock() {
            state.up = true;
            state.ups += 1;
        }
    }

    pub fn down(&self, error: &impl Display) {
        if let Ok(mut state) = self.0.lock() {
            state.up = false;
            state.last_error = Some(error.to_string());
        }
    }

    pub fn snapshot(&self) -> LinkState {
        self.0.lock().map(|s| s.clone()).unwrap_or_default()
    }
}
