//! The ground bridge: telemetry from mavlink-router on the overlay to each ground station, and what
//! the ground stations send back. Each ground station has its own UDP socket that both sends its
//! telemetry and hears it, so a station that replies to the address it hears from
//! (`QGroundControl` does) needs no further setup. Read-only traffic goes raw to mavlink-router;
//! everything that changes the vehicle goes through the controller (`control`), in envelopes or
//! not at all. Messages to the camera component are the adapter's own (`camera`).

mod camera;
mod control;

use std::net::{IpAddr, Ipv4Addr, SocketAddr, ToSocketAddrs, UdpSocket};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{ATTITUDE_DATA, MavMessage};
use serde::{Deserialize, Serialize};

pub use self::camera::{Camera, CameraCounts, CameraStream};
pub use self::control::{ControlSnapshot, Controller, LeaseStatus, Status, TakeError};
use thinlink_policy::clock;
use thinlink_policy::predict::Attitude;

use crate::auth::VehicleId;
use crate::clock::{Clock, ClockSnapshot, ClockSource};
use crate::frame::{Frame, MAX_DATAGRAM, datagram_frames};
use crate::gate::{self, Route};
use crate::stats::{DropReason, Flow, FlowCounts, Link, LinkState};
use crate::{ControlError, bind_overlay};

const RECV_ERROR_PAUSE: Duration = Duration::from_millis(100);
/// The camera's tracking status is due this often at most.
const STATUS_TICK: Duration = Duration::from_millis(50);
/// How often the vehicle is asked for its clock bound: well inside the 15 s a reading lives.
const CLOCK_QUERY_EVERY: Duration = Duration::from_secs(2);
/// How often the sum of the two bounds is worked out again for the video output: a bound that
/// expires is noticed this late.
const BOUND_REFRESH_EVERY: Duration = Duration::from_millis(100);
/// `GroundStats::bound` when there is no bound to trust.
const NO_BOUND: u64 = u64::MAX;
/// The autopilot's component id (`MAV_COMP_ID_AUTOPILOT1`); the camera component has its own.
const AUTOPILOT_COMPONENT: u8 = 1;

/// `testbed/config/ground.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundConfig {
    /// Overlay address mavlink-router sends telemetry to.
    pub listen: SocketAddr,
    /// mavlink-router's overlay address. Its endpoint sends from a port of its choosing, so only
    /// the address is checked, and ground-station traffic goes to the port it last sent from.
    pub relay: IpAddr,
    /// Ground stations as `host:port`.
    pub gcs: Vec<String>,
    /// The control socket, on the ground's overlay address.
    pub control: SocketAddr,
    /// The vehicle's control endpoint.
    pub vehicle: SocketAddr,
    /// Where this machine's clock bound comes from. The ground adds the vehicle's bound to it and
    /// shows video as live only within the sum. In a container `chronyd` on the host is out of
    /// reach: use `fixed` there.
    pub clock: ClockSource,
    /// A file with the key this adapter signs its messages with (`auth`).
    pub control_key: PathBuf,
    /// The vehicle's name, as its config has it: what every signature is made for.
    pub vehicle_id: VehicleId,
}

#[derive(Debug)]
pub struct GroundStats {
    /// This machine's clock.
    pub clock: Arc<Clock>,
    /// The overlay socket.
    pub listen: Link,
    relay_endpoint: Mutex<Option<SocketAddr>>,
    /// The autopilot's newest `ATTITUDE`.
    attitude: Mutex<Option<Attitude>>,
    started: Instant,
    /// The sum of the two clock bounds as `refresh_bound` last worked it out, `NO_BOUND` for none:
    /// the video output reads it on every pass, and must not wait for the controller's lock.
    bound: AtomicU64,
    /// mavlink-router to the ground stations.
    pub telemetry: Flow,
    /// The ground stations' read-only traffic to mavlink-router; what the controller handles
    /// counts in, not out.
    pub gcs: Flow,
}

/// The clock bounds in force (`GET /metrics`).
#[derive(Debug, Serialize)]
pub struct ClockView {
    pub own: ClockSnapshot,
    /// What the vehicle last reported, grown by its age; `None` when stale or never answered.
    pub vehicle_ms: Option<u64>,
    /// The sum the freshness output judges by; `None` means no age is trusted.
    pub effective_ms: Option<u64>,
}

#[derive(Debug, Serialize)]
pub struct GroundSnapshot {
    pub listen: LinkState,
    /// Where mavlink-router was last heard from; `null` until telemetry arrives.
    pub relay_endpoint: Option<SocketAddr>,
    pub telemetry: FlowCounts,
    pub gcs: FlowCounts,
}

impl GroundStats {
    pub fn new(clock: ClockSource) -> Self {
        Self {
            clock: Arc::new(Clock::new(clock)),
            listen: Link::default(),
            relay_endpoint: Mutex::new(None),
            attitude: Mutex::new(None),
            started: Instant::now(),
            bound: AtomicU64::new(NO_BOUND),
            telemetry: Flow::default(),
            gcs: Flow::default(),
        }
    }

    /// The bound on the difference between the vehicle's clock and this one: the two bounds added,
    /// as `refresh_bound` last worked it out. `None` unless both are fresh, and then no age is
    /// trusted. Takes no lock.
    pub fn clock_bound(&self) -> Option<u64> {
        let bound = self.bound.load(Ordering::Relaxed);
        (bound != NO_BOUND).then_some(bound)
    }

    fn combined_bound(&self, controller: &Controller) -> Option<u64> {
        clock::combined(self.clock.reading(), controller.vehicle_clock())
    }

    fn refresh_bound(&self, controller: &Controller) {
        let bound = self.combined_bound(controller).unwrap_or(NO_BOUND);
        self.bound.store(bound, Ordering::Relaxed);
    }

    pub fn clock_view(&self, controller: &Controller) -> ClockView {
        ClockView {
            own: self.clock.snapshot(),
            vehicle_ms: controller.vehicle_clock().and_then(clock::aged),
            effective_ms: self.combined_bound(controller),
        }
    }

    pub fn snapshot(&self) -> GroundSnapshot {
        GroundSnapshot {
            listen: self.listen.snapshot(),
            relay_endpoint: self.relay_endpoint(),
            telemetry: self.telemetry.snapshot(),
            gcs: self.gcs.snapshot(),
        }
    }

    /// Milliseconds since this was created, the clock `attitude`'s receive time is on.
    pub fn monotonic_ms(&self) -> u64 {
        u64::try_from(self.started.elapsed().as_millis()).unwrap_or(u64::MAX)
    }

    /// The newest `ATTITUDE` the autopilot sent, with when it arrived (`monotonic_ms`).
    pub fn attitude(&self) -> Option<Attitude> {
        self.attitude.lock().ok().and_then(|a| *a)
    }

    fn note_attitude(&self, frame: &Frame) {
        let Some(attitude) = attitude_of(frame, self.monotonic_ms()) else {
            return;
        };
        if let Ok(mut current) = self.attitude.lock() {
            *current = Some(attitude);
        }
    }

    fn relay_endpoint(&self) -> Option<SocketAddr> {
        self.relay_endpoint.lock().ok().and_then(|e| *e)
    }

    fn set_relay_endpoint(&self, endpoint: SocketAddr) {
        if let Ok(mut current) = self.relay_endpoint.lock() {
            *current = Some(endpoint);
        }
    }
}

/// The attitude an autopilot's `ATTITUDE` frame carries, received at `at_ms`.
fn attitude_of(frame: &Frame, at_ms: u64) -> Option<Attitude> {
    if frame.component_id() != AUTOPILOT_COMPONENT {
        return None;
    }
    let Ok(MavMessage::ATTITUDE(ATTITUDE_DATA {
        roll,
        pitch,
        pitchspeed,
        yawspeed,
        ..
    })) = frame.decode()
    else {
        return None;
    };
    Some(Attitude {
        roll_rad: roll,
        pitch_rad: pitch,
        yaw_rate_rad_s: yawspeed,
        pitch_rate_rad_s: pitchspeed,
        at_ms,
    })
}

/// Resolves each ground station to its first IPv4 address.
pub fn resolve(gcs: &[String]) -> Result<Vec<SocketAddr>, ControlError> {
    gcs.iter()
        .map(|name| {
            let error = |reason: String| ControlError::Resolve {
                name: name.clone(),
                reason,
            };
            name.to_socket_addrs()
                .map_err(|e| error(e.to_string()))?
                .find(SocketAddr::is_ipv4)
                .ok_or_else(|| error("no IPv4 address".to_owned()))
        })
        .collect()
}

/// A ground station and the socket that talks to it.
struct Station {
    address: SocketAddr,
    socket: UdpSocket,
}

/// Runs the bridge, `controller` and, if there is one, `camera` until the process ends. Returns
/// only when a ground station's socket cannot be opened.
pub fn run(
    config: &GroundConfig,
    stations: &[SocketAddr],
    stats: &Arc<GroundStats>,
    controller: &Arc<Controller>,
    camera: Option<&Arc<Camera>>,
) -> Result<(), ControlError> {
    let any = SocketAddr::from((Ipv4Addr::UNSPECIFIED, 0));
    let stations: Vec<Arc<Station>> = stations
        .iter()
        .map(|&address| {
            let socket =
                UdpSocket::bind(any).map_err(|source| ControlError::Bind { addr: any, source })?;
            Ok(Arc::new(Station { address, socket }))
        })
        .collect::<Result<_, ControlError>>()?;
    {
        let (controller, stations) = (Arc::clone(controller), stations.clone());
        thread::spawn(move || controller.serve(&stations));
    }
    {
        let clock = Arc::clone(&stats.clock);
        thread::spawn(move || clock.poll());
    }
    {
        let controller = Arc::clone(controller);
        thread::spawn(move || {
            loop {
                controller.ask_clock();
                thread::sleep(CLOCK_QUERY_EVERY);
            }
        });
    }
    {
        let (stats, controller) = (Arc::clone(stats), Arc::clone(controller));
        thread::spawn(move || {
            loop {
                stats.refresh_bound(&controller);
                thread::sleep(BOUND_REFRESH_EVERY);
            }
        });
    }
    let overlay = Arc::new(bind_overlay(config.listen, &stats.listen));
    for (index, station) in stations.iter().enumerate() {
        let (station, overlay, stats, controller, camera) = (
            Arc::clone(station),
            Arc::clone(&overlay),
            Arc::clone(stats),
            Arc::clone(controller),
            camera.cloned(),
        );
        thread::spawn(move || {
            station_to_relay(
                index,
                &station,
                &overlay,
                &stats,
                &controller,
                camera.as_deref(),
            );
        });
    }
    if let Some(camera) = camera {
        {
            let (controller, camera) = (Arc::clone(controller), Arc::clone(camera));
            thread::spawn(move || controller.keep_region(&camera));
        }
        let (camera, stations) = (Arc::clone(camera), stations.clone());
        thread::spawn(move || send_tracking_status(&camera, &stations));
    }
    relay_to_stations(
        &overlay,
        config.relay,
        &stations,
        stats,
        camera.map(Arc::as_ref),
    );
    Ok(())
}

/// Sends the camera's tracking status to every station at the interval it asked for.
fn send_tracking_status(camera: &Camera, stations: &[Arc<Station>]) {
    loop {
        if let Some(status) = camera.status_frame(Instant::now()) {
            for station in stations {
                let _ = station.socket.send_to(&status, station.address);
            }
        }
        thread::sleep(STATUS_TICK);
    }
}

/// Telemetry from mavlink-router to every ground station, each vehicle heartbeat followed by the
/// camera's.
fn relay_to_stations(
    overlay: &UdpSocket,
    relay: IpAddr,
    stations: &[Arc<Station>],
    stats: &GroundStats,
    camera: Option<&Camera>,
) {
    let mut buf = [0_u8; MAX_DATAGRAM];
    loop {
        let (len, from) = match overlay.recv_from(&mut buf) {
            Ok(received) => received,
            Err(e) => {
                stats.telemetry.receive_failed("the overlay socket", &e);
                thread::sleep(RECV_ERROR_PAUSE);
                continue;
            }
        };
        if from.ip() != relay {
            stats.telemetry.dropped(DropReason::UnknownSource);
            continue;
        }
        stats.set_relay_endpoint(from);
        for frame in datagram_frames(buf.get(..len).unwrap_or_default()) {
            stats.telemetry.received(&frame);
            stats.note_attitude(&frame);
            for station in stations {
                match station.socket.send_to(frame.bytes(), station.address) {
                    Ok(_) => stats.telemetry.forwarded(&frame),
                    Err(_) => stats.telemetry.dropped(DropReason::SendFailed),
                }
            }
            let Some(heartbeat) = camera.and_then(|camera| camera.follow(&frame)) else {
                continue;
            };
            for station in stations {
                let _ = station.socket.send_to(&heartbeat, station.address);
            }
        }
    }
}

/// What one ground station sends: messages to the camera to the camera, read-only frames to
/// mavlink-router, the rest to the controller.
fn station_to_relay(
    index: usize,
    station: &Station,
    overlay: &UdpSocket,
    stats: &GroundStats,
    controller: &Controller,
    camera: Option<&Camera>,
) {
    let mut buf = [0_u8; MAX_DATAGRAM];
    loop {
        let (len, from) = match station.socket.recv_from(&mut buf) {
            Ok(received) => received,
            Err(e) => {
                let socket = format!("the socket for {}", station.address);
                stats.gcs.receive_failed(&socket, &e);
                thread::sleep(RECV_ERROR_PAUSE);
                continue;
            }
        };
        if from != station.address {
            stats.gcs.dropped(DropReason::UnknownSource);
            continue;
        }
        for frame in datagram_frames(buf.get(..len).unwrap_or_default()) {
            stats.gcs.received(&frame);
            let Ok(message) = frame.decode() else {
                stats.gcs.dropped(DropReason::Undecodable);
                continue;
            };
            if let Some(camera) = camera.filter(|_| camera::addressed(&message)) {
                for answer in camera.answer(&message) {
                    let _ = station.socket.send_to(&answer, station.address);
                }
                continue;
            }
            let ack = match gate::route(&message) {
                Route::Raw => {
                    forward(&frame, overlay, stats);
                    None
                }
                Route::Refuse => {
                    controller.refuse(&message);
                    None
                }
                Route::Unsupported => controller.unsupported(&frame, &message),
                Route::Envelope(class) => controller.envelope(index, &frame, &message, class),
            };
            if let Some(ack) = ack {
                let _ = station.socket.send_to(&ack, station.address);
            }
        }
    }
}

fn forward(frame: &Frame, overlay: &UdpSocket, stats: &GroundStats) {
    let Some(relay) = stats.relay_endpoint() else {
        stats.gcs.dropped(DropReason::NoDestination);
        return;
    };
    match overlay.send_to(frame.bytes(), relay) {
        Ok(_) => stats.gcs.forwarded(frame),
        Err(_) => stats.gcs.dropped(DropReason::SendFailed),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(component: u8, message: &MavMessage) -> Frame {
        let header = mavlink::MavHeader {
            system_id: 1,
            component_id: component,
            sequence: 0,
        };
        let mut bytes = Vec::new();
        mavlink::write_v2_msg(&mut bytes, header, message).unwrap();
        datagram_frames(&bytes).next().unwrap()
    }

    #[test]
    fn only_the_autopilots_attitude_is_kept() {
        let attitude = MavMessage::ATTITUDE(ATTITUDE_DATA {
            roll: 0.1,
            pitch: 0.2,
            yawspeed: 0.3,
            pitchspeed: 0.4,
            ..ATTITUDE_DATA::default()
        });
        let got = attitude_of(&frame(1, &attitude), 7).unwrap();
        assert_eq!(
            (
                got.roll_rad,
                got.pitch_rad,
                got.yaw_rate_rad_s,
                got.pitch_rate_rad_s,
                got.at_ms
            ),
            (0.1, 0.2, 0.3, 0.4, 7)
        );
        assert!(attitude_of(&frame(100, &attitude), 7).is_none());
    }
}
