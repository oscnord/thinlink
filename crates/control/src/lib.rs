//! `MAVLink` between an autopilot (`ArduRover` or `ArduCopter` SITL, or a flight controller over a serial port) and ground stations: the vehicle bridge with its command gate
//! and telemetry budget, the ground bridge, and the control envelope protocol between them. Frames
//! are read and checked with the `mavlink` crate (`ardupilotmega` dialect) and forwarded byte for
//! byte.

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::stats::Link;

pub mod auth;
pub mod autopilot;
pub mod clock;
pub mod envelope;
pub mod frame;
pub mod gate;
pub mod ground;
pub mod identity;
pub mod landing;
pub mod params;
pub mod position;
pub mod sighting;
pub mod stats;
pub mod telemetry;
pub mod vehicle;

/// Until `wg0` is up an overlay address cannot be bound.
const BIND_RETRY: Duration = Duration::from_secs(1);

#[derive(Debug, thiserror::Error)]
pub enum ControlError {
    #[error("cannot open a UDP socket on {addr}: {source}")]
    Bind { addr: SocketAddr, source: io::Error },
    #[error("cannot resolve ground station `{name}`: {reason}")]
    Resolve { name: String, reason: String },
    #[error("invalid telemetry limits: {0}")]
    Telemetry(&'static str),
    #[error("invalid control limits: {0}")]
    Limits(&'static str),
    #[error(
        "a TCP autopilot must be on loopback, not {0}: over TCP the bridge talks to SITL in its own network namespace"
    )]
    NotSitl(SocketAddr),
    #[error("invalid real-mode settings: {0}")]
    Real(String),
    #[error("no controllers: list the overlay addresses that may control the vehicle")]
    NoControllers,
    #[error(transparent)]
    Key(#[from] auth::KeyError),
    #[error("cannot encode a control message: {0}")]
    Encode(String),
}

/// Binds `listen`, an overlay address, retrying every second until `wg0` is up; `link` reports
/// the attempts.
fn bind_overlay(listen: SocketAddr, link: &Link) -> UdpSocket {
    loop {
        match UdpSocket::bind(listen) {
            Ok(socket) => {
                link.up();
                return socket;
            }
            Err(e) => link.down(&format!("cannot bind {listen}: {e}")),
        }
        thread::sleep(BIND_RETRY);
    }
}

/// Wall-clock UNIX milliseconds, for measurement only; zero if the clock reads before 1970.
pub fn unix_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_millis()).unwrap_or(u64::MAX))
}

/// A read timeout; the caller polls again.
fn timed_out(error: &io::Error) -> bool {
    matches!(
        error.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
    )
}
