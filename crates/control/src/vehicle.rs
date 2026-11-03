//! The vehicle bridge: the autopilot (`ArduRover` or `ArduCopter` SITL over TCP, or a flight controller over a serial port) on one side, mavlink-router on the relay over UDP
//! inside `WireGuard` on the other. Telemetry goes up within the byte budget, one frame per
//! datagram; from the relay only the read-only traffic the gate admits is written to SITL, and
//! state-changing messages arrive only in control envelopes (`control`). Nothing is queued towards
//! SITL, so a reconnect never replays old commands.

mod control;
#[cfg(test)]
mod sitl_identity;

use std::collections::BTreeMap;
use std::io::{self, Read, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use mavlink::Message;
use mavlink::MessageData;
use mavlink::dialects::ardupilotmega::{GLOBAL_POSITION_INT_DATA, MavMessage};
use serde::{Deserialize, Serialize};

use sat_teleop_policy::landing::Setup;
use sat_teleop_policy::sighting::Sightings;

use self::control::{ControlSnapshot, ControlStats};
use crate::auth::VehicleId;
use crate::autopilot::{Autopilot, Wire};
use crate::clock::ClockSource;
use crate::envelope::ControlLimits;
use crate::frame::{Frame, FrameReader, MAX_DATAGRAM, datagram_frames};
use crate::gate::{self, Decision};
use crate::identity::{self, Mode};
use crate::landing::{self, Landing};
use crate::params::{self, Required};
use crate::position::NewestPosition;
use crate::sighting::Announcer;
use crate::stats::{DropReason, DropsByMessage, Flow, FlowCounts, Link, LinkState};
use crate::telemetry::{Admission, RateCap, Released, TelemetryLimits};
use crate::{ControlError, timed_out};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(2);
/// Read timeout on SITL's port, so waiting replies go out between SITL's frames.
const POLL: Duration = Duration::from_millis(20);
/// SITL sends a heartbeat every second; this much silence means it hangs.
const SITL_SILENCE: Duration = Duration::from_secs(5);
const WRITE_TIMEOUT: Duration = Duration::from_secs(1);
const FIRST_RETRY: Duration = Duration::from_millis(500);
const MAX_RETRY: Duration = Duration::from_secs(5);
const RECV_ERROR_PAUSE: Duration = Duration::from_millis(100);

/// `[mavlink]` of `testbed/config/vehicle.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleConfig {
    /// The autopilot's `MAVLink` port: SITL's TCP port (serial 0) in the vehicle's network
    /// namespace, or a flight controller's serial port.
    pub autopilot: Autopilot,
    /// mavlink-router's UDP server endpoint on the relay. Only datagrams from it are read.
    pub relay: SocketAddr,
    /// The control endpoint, on the vehicle's overlay address.
    pub control: SocketAddr,
    /// This vehicle's name: what every signature is made for, so one made for another vehicle is
    /// refused. The ground's config names the same one.
    pub vehicle_id: VehicleId,
    /// The only overlay addresses that may ask for the lease or send envelopes, each with the
    /// public key its messages must be signed with. The relay attests the addresses, not
    /// `WireGuard` end to end; the keys are what a compromised relay lacks (`docs/architecture.md`,
    /// "Control protocol").
    pub controllers: Vec<ControllerConfig>,
    /// `sitl` (the default) sends control only to an autopilot that reports itself SITL; `real`
    /// only to one that does not, and only while every interlock holds (`identity`).
    #[serde(default)]
    pub mode: Mode,
    /// Where the clock bound comes from; the ground adds it to its own to judge the video's age.
    /// Real mode needs `chrony` (`shared` and `fixed` only with `sitl_in_real_mode`).
    pub clock: ClockSource,
    /// Test only, and only in testbed builds: real mode also accepts SITL. The vehicle's loader
    /// refuses it unless `debug_hooks` is set.
    #[cfg(feature = "testbed")]
    #[serde(default)]
    pub sitl_in_real_mode: bool,
    /// Real mode: the parameters the autopilot must report, with their values.
    #[serde(default)]
    pub require: Vec<Required>,
    /// Off by default. On, the lease holder may upload a waypoint mission and fly it in Auto (a
    /// copter only), and a copter that loses the link finishes it by the autopilot's own failsafe
    /// (`docs/real-vehicle.md`). In real mode `require` must then hold the failsafe and fence
    /// values that make that safe.
    #[serde(default)]
    pub allow_missions: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerConfig {
    pub address: IpAddr,
    /// A file with the controller's public key (`auth`).
    pub public_key: PathBuf,
}

impl VehicleConfig {
    /// A TCP autopilot is `ArduRover` or `ArduCopter` SITL in the vehicle's network namespace: any
    /// target off loopback is refused. Whatever the autopilot, control reaches it only once it has
    /// identified itself as SITL (`identity`). Someone must be allowed to control it.
    pub fn validate(self) -> Result<Self, ControlError> {
        if let Autopilot::Tcp(addr) = &self.autopilot
            && !addr.ip().is_loopback()
        {
            return Err(ControlError::NotSitl(*addr));
        }
        if self.controllers.is_empty() {
            return Err(ControlError::NoControllers);
        }
        params::validate(&self.require).map_err(ControlError::Real)?;
        if self.allow_missions && self.mode == Mode::Real {
            params::validate_missions(&self.require).map_err(ControlError::Real)?;
        }
        let measured = matches!(self.clock, ClockSource::Chrony | ClockSource::ChronyAt(_));
        if self.mode == Mode::Real && !measured && !self.allows_sitl() {
            return Err(ControlError::Real(
                "real mode needs clock = \"chrony\": a clock bound must be measured".to_owned(),
            ));
        }
        Ok(self)
    }

    /// Whether real mode also accepts SITL: the testbed's allowance, which a release build lacks.
    #[cfg(feature = "testbed")]
    pub fn allows_sitl(&self) -> bool {
        self.sitl_in_real_mode
    }

    #[cfg(not(feature = "testbed"))]
    pub fn allows_sitl(&self) -> bool {
        false
    }
}

#[derive(Debug, Default)]
pub struct VehicleStats {
    pub sitl: Link,
    /// Every byte read from SITL, whether or not it formed a frame of the dialect.
    sitl_bytes: AtomicU64,
    /// SITL to the relay.
    pub telemetry: Flow,
    /// The relay to SITL, through the gate.
    pub gcs: Flow,
    /// What the gate refused, by message (and command).
    gcs_refused: DropsByMessage,
    /// The control endpoint and what its envelopes wrote to SITL.
    pub control: ControlStats,
    /// The telemetry cap in frame bytes per second: `[telemetry]` of the limits until the uplink
    /// budget sets it.
    telemetry_cap: AtomicU32,
    /// The autopilot's newest position, for the snapshots the vehicle keeps.
    pub position: NewestPosition,
    /// The people seen while the ground was out of reach: told to the ground when it is heard
    /// again, listed on request.
    pub sightings: Mutex<Sightings>,
    /// The landing target switch and the sightings it may send (`landing`).
    pub landing: Landing,
}

#[derive(Debug, Serialize)]
pub struct VehicleSnapshot {
    pub telemetry_cap: u32,
    pub sitl: LinkState,
    pub sitl_bytes: u64,
    pub telemetry: FlowCounts,
    pub gcs: FlowCounts,
    pub gcs_refused: BTreeMap<String, u64>,
    pub control: ControlSnapshot,
    pub landing: landing::LandingSnapshot,
}

impl VehicleStats {
    /// Counters for a vehicle configured as `config` says: the controllers' keys, the mode, the
    /// required parameters and the clock. `landing` is `None` with the landing target switch off.
    pub fn new(config: &VehicleConfig, landing: Option<Setup>) -> Self {
        let stats = Self {
            landing: Landing::new(landing),
            ..Self::default()
        };
        stats.control.configure(config);
        stats
    }

    pub fn snapshot(&self) -> VehicleSnapshot {
        VehicleSnapshot {
            telemetry_cap: self.telemetry_cap.load(Ordering::Relaxed),
            sitl: self.sitl.snapshot(),
            sitl_bytes: self.sitl_bytes.load(Ordering::Relaxed),
            telemetry: self.telemetry.snapshot(),
            gcs: self.gcs.snapshot(),
            gcs_refused: self.gcs_refused.snapshot(),
            control: self.control.snapshot(),
            landing: self.landing.snapshot(),
        }
    }

    /// Sets the telemetry cap from the uplink budget (`sat_teleop_video_core::budget::Budget::telemetry_cap`,
    /// never below the vehicle's state); the bridge applies it at its next frame.
    pub fn set_telemetry_cap(&self, bytes_per_s: u32) {
        self.telemetry_cap.store(bytes_per_s, Ordering::Relaxed);
    }
}

/// The current SITL connection, for writing what the gate admits.
type SitlWriter = Arc<Mutex<Option<Box<dyn Wire>>>>;

/// Runs the bridge and the control endpoint until the process ends, reconnecting to SITL with
/// backoff. `ground_heard` says whether the ground can be reached: the sightings are told to it
/// only then. Returns only when it cannot open its UDP socket.
pub fn run(
    config: VehicleConfig,
    limits: TelemetryLimits,
    control: ControlLimits,
    ground_heard: impl Fn() -> bool,
    stats: &Arc<VehicleStats>,
) -> Result<(), ControlError> {
    let VehicleConfig {
        autopilot,
        relay,
        control: endpoint,
        ..
    } = config;
    let any = SocketAddr::from((Ipv4Addr::UNSPECIFIED, 0));
    let bind = |socket: io::Result<UdpSocket>| {
        socket.map_err(|source| ControlError::Bind { addr: any, source })
    };
    let socket = bind(UdpSocket::bind(any))?;
    let inbound = bind(socket.try_clone())?;
    let writer = SitlWriter::default();
    if let Some(clock) = stats.control.clock() {
        thread::spawn(move || clock.poll());
    }
    {
        let (writer, stats) = (Arc::clone(&writer), Arc::clone(stats));
        thread::spawn(move || relay_to_sitl(&inbound, relay, &writer, &stats));
    }
    {
        let (writer, stats) = (Arc::clone(&writer), Arc::clone(stats));
        thread::spawn(move || {
            control::serve(endpoint, control, &writer, &stats.control);
        });
    }
    stats.set_telemetry_cap(limits.bytes_per_s);
    let mut cap = RateCap::new(limits);
    let mut retry = FIRST_RETRY;
    loop {
        match connect(&autopilot) {
            Ok(stream) => {
                retry = FIRST_RETRY;
                let error = serve_sitl(
                    stream,
                    &socket,
                    relay,
                    &writer,
                    &mut cap,
                    &ground_heard,
                    stats,
                );
                stats.sitl.down(&error);
            }
            Err(error) => stats.sitl.down(&error),
        }
        thread::sleep(retry);
        retry = (retry * 2).min(MAX_RETRY);
    }
}

fn connect(autopilot: &Autopilot) -> io::Result<Box<dyn Wire>> {
    autopilot.connect(CONNECT_TIMEOUT, POLL, WRITE_TIMEOUT)
}

/// Forwards SITL's telemetry until the connection fails, and lends it to the gate meanwhile.
fn serve_sitl(
    stream: Box<dyn Wire>,
    socket: &UdpSocket,
    relay: SocketAddr,
    writer: &SitlWriter,
    cap: &mut RateCap,
    ground_heard: &dyn Fn() -> bool,
    stats: &VehicleStats,
) -> io::Error {
    let clone = match stream.try_clone() {
        Ok(clone) => clone,
        Err(error) => return error,
    };
    set_writer(writer, Some(clone));
    stats.sitl.up();
    for _ in 0..cap.clear() {
        stats.telemetry.dropped(DropReason::Expired);
    }
    let mut frames = FrameReader::new(Counted::new(stream));
    let mut last_frame = Instant::now();
    let mut autopilot = None;
    let mut probe = Probe::default();
    let mut announcer = Announcer::default();
    let error = loop {
        cap.set_rate(stats.telemetry_cap.load(Ordering::Relaxed));
        release_waiting(cap, socket, relay, &stats.telemetry);
        let heard = ground_heard();
        let told = announcer.next(Instant::now(), autopilot, heard, &stats.sightings, cap);
        if let Some(frame) = told {
            send_up(&frame, socket, relay, &stats.telemetry);
        }
        let notice = stats.landing.notice_frame(Instant::now(), autopilot, cap);
        if let Some(frame) = notice {
            send_up(&frame, socket, relay, &stats.telemetry);
        }
        send_landing_target(writer, stats);
        let needs_position = stats.position.fresh(Instant::now()).is_none();
        let needs_mount = stats.landing.wants_mount(Instant::now());
        probe.ask_if_due(
            autopilot,
            writer,
            &stats.control,
            needs_position,
            needs_mount,
        );
        let frame = match frames.next_frame() {
            Ok(frame) => frame,
            Err(e) if timed_out(&e) && last_frame.elapsed() < SITL_SILENCE => continue,
            Err(e) if timed_out(&e) => break silent(SITL_SILENCE),
            Err(e) => break e,
        };
        last_frame = Instant::now();
        autopilot = learn_vehicle(&frame, &stats.control).or(autopilot);
        stats.position.note(&frame, autopilot, last_frame);
        stats.landing.note_mount(&frame, autopilot, last_frame);
        stats
            .sitl_bytes
            .fetch_add(frames.source_mut().take(), Ordering::Relaxed);
        stats.telemetry.received(&frame);
        match cap.admit(last_frame, frame) {
            Admission::Send(frame) => send_up(&frame, socket, relay, &stats.telemetry),
            Admission::Queued => {}
            Admission::Dropped => stats.telemetry.dropped(DropReason::RateLimited),
        }
    };
    set_writer(writer, None);
    stats.control.forget_autopilot();
    stats.position.forget();
    stats.landing.forget();
    error
}

/// The only place a `LANDING_TARGET` reaches the autopilot: the newest sighting, if the control target
/// and the policy allow it (`landing`), written under the connection's lock like any control frame.
fn send_landing_target(writer: &SitlWriter, stats: &VehicleStats) {
    let now = Instant::now();
    let height = stats.position.fresh(now).map(|p| p.rel_alt_mm);
    let target = stats.control.target();
    if let Some(frame) = stats.landing.next_frame(now, target, height) {
        write_to_sitl(&frame, writer, &Flow::default());
    }
}

/// Takes the vehicle type from the autopilot's own heartbeat (a type the gate does not know
/// replaces the one before it) and notes the SITL marker parameter when that autopilot reports it.
/// Returns the heartbeat's system and component, the address to ask for the marker.
fn learn_vehicle(frame: &Frame, control: &ControlStats) -> Option<(u8, u8)> {
    let message = frame.decode().ok()?;
    let address = (u8::try_from(frame.system_id()).ok()?, frame.component_id());
    control.param_value(address, &message);
    if !gate::is_autopilot_heartbeat(&message) {
        return None;
    }
    control.heartbeat(address, gate::vehicle_of(&message));
    Some(address)
}

/// Asks the autopilot for its position stream while there is no fresh position, for the SITL marker
/// once a second until it has answered, and in real mode for the required failsafe parameters, one
/// request at a time. Writes go to the connection the control endpoint shares, under its lock, and
/// are not counted as traffic.
#[derive(Default)]
struct Probe {
    position: Option<Instant>,
    marker: Option<Instant>,
    param: Option<Instant>,
}

/// How often the position stream is asked for while none arrives: an autopilot sends nothing it
/// was not asked for, and forgets what it was asked when it restarts.
const POSITION_ASK_EVERY: Duration = Duration::from_secs(5);
/// The position stream's interval, well inside the age a snapshot's position may have.
const POSITION_INTERVAL: Duration = Duration::from_millis(500);

/// The least time between two parameter requests: the autopilot answers each within a few
/// milliseconds, and a lost one is asked for again.
const PARAM_GAP: Duration = Duration::from_millis(500);

impl Probe {
    fn ask_if_due(
        &mut self,
        autopilot: Option<(u8, u8)>,
        writer: &SitlWriter,
        control: &ControlStats,
        needs_position: bool,
        needs_mount: bool,
    ) {
        let Some((system, component)) = autopilot else {
            return;
        };
        let mount = needs_mount
            .then(|| landing::mount_request(system, component))
            .flatten();
        if let Some(frame) = mount {
            write_to_sitl(&frame, writer, &Flow::default());
        }
        if let Some(frame) = self.next_request(system, component, control, needs_position) {
            write_to_sitl(&frame, writer, &Flow::default());
        }
    }

    fn next_request(
        &mut self,
        system: u8,
        component: u8,
        control: &ControlStats,
        needs_position: bool,
    ) -> Option<Frame> {
        let position_due = self
            .position
            .is_none_or(|at| at.elapsed() >= POSITION_ASK_EVERY);
        if needs_position && position_due {
            self.position = Some(Instant::now());
            let id = GLOBAL_POSITION_INT_DATA::ID;
            return identity::stream_request(system, component, id, POSITION_INTERVAL);
        }
        let marker_due = self
            .marker
            .is_none_or(|at| at.elapsed() >= identity::PROBE_EVERY);
        if marker_due && !control.sitl_marker() {
            self.marker = Some(Instant::now());
            control.probed();
            return identity::probe(system, component);
        }
        let param_due = self.param.is_none_or(|at| at.elapsed() >= PARAM_GAP);
        let name = control.param_to_read().filter(|_| param_due)?;
        self.param = Some(Instant::now());
        identity::read_request(system, component, &name)
    }
}

fn silent(silence: Duration) -> io::Error {
    let message = format!("SITL sent nothing for {} s", silence.as_secs());
    io::Error::new(io::ErrorKind::TimedOut, message)
}

/// Sends the waiting replies the budget now allows and drops those that waited too long.
fn release_waiting(cap: &mut RateCap, socket: &UdpSocket, relay: SocketAddr, flow: &Flow) {
    while let Some(released) = cap.release(Instant::now()) {
        match released {
            Released::Send(frame) => send_up(&frame, socket, relay, flow),
            Released::Expired => flow.dropped(DropReason::Expired),
        }
    }
}

fn send_up(frame: &Frame, socket: &UdpSocket, relay: SocketAddr, flow: &Flow) {
    match socket.send_to(frame.bytes(), relay) {
        Ok(_) => flow.forwarded(frame),
        Err(_) => flow.dropped(DropReason::SendFailed),
    }
}

fn set_writer(writer: &SitlWriter, stream: Option<Box<dyn Wire>>) {
    if let Ok(mut current) = writer.lock() {
        *current = stream;
    }
}

/// Reads datagrams from the relay and writes the frames the gate admits to SITL.
fn relay_to_sitl(socket: &UdpSocket, relay: SocketAddr, writer: &SitlWriter, stats: &VehicleStats) {
    let mut buf = [0_u8; MAX_DATAGRAM];
    loop {
        let (len, from) = match socket.recv_from(&mut buf) {
            Ok(received) => received,
            Err(e) => {
                stats.gcs.receive_failed("the relay socket", &e);
                thread::sleep(RECV_ERROR_PAUSE);
                continue;
            }
        };
        if from != relay {
            stats.gcs.dropped(DropReason::UnknownSource);
            continue;
        }
        for frame in datagram_frames(buf.get(..len).unwrap_or_default()) {
            stats.gcs.received(&frame);
            match refusal(&frame) {
                Some((reason, what)) => {
                    stats.gcs.dropped(reason);
                    stats.gcs_refused.count(what);
                }
                None => {
                    write_to_sitl(&frame, writer, &stats.gcs);
                }
            }
        }
    }
}

/// Why the gate refuses `frame` and what it refused, or `None` when it may reach SITL.
fn refusal(frame: &Frame) -> Option<(DropReason, String)> {
    let Ok(message) = frame.decode() else {
        let what = format!("undecodable {}", frame.message_id());
        return Some((DropReason::Undecodable, what));
    };
    let name = message.message_name();
    match gate::decide(&message) {
        Decision::Forward => None,
        Decision::NotAllowlisted => Some((DropReason::NotAllowlisted, name.to_owned())),
        Decision::CommandNotAllowlisted => {
            let what = format!("{name} {}", command_name(&message));
            Some((DropReason::CommandNotAllowlisted, what))
        }
    }
}

fn command_name(message: &MavMessage) -> String {
    match message {
        MavMessage::COMMAND_LONG(c) => format!("{:?}", c.command),
        MavMessage::COMMAND_INT(c) => format!("{:?}", c.command),
        _ => String::new(),
    }
}

/// Writes `frame` to SITL; false when it is not connected or the write failed. A failed write may
/// leave part of a frame in the stream, and SITL's parser could then resync on bytes inside a later
/// frame's payload, which a ground station chooses: a way past the gate. So the connection is shut
/// down and `serve_sitl` reconnects.
fn write_to_sitl(frame: &Frame, writer: &SitlWriter, flow: &Flow) -> bool {
    let Ok(mut current) = writer.lock() else {
        flow.dropped(DropReason::NoDestination);
        return false;
    };
    let Some(stream) = current.as_mut() else {
        flow.dropped(DropReason::NoDestination);
        return false;
    };
    if stream.write_all(frame.bytes()).is_ok() {
        flow.forwarded(frame);
        return true;
    }
    flow.dropped(DropReason::SendFailed);
    stream.shutdown();
    *current = None;
    false
}

/// Counts the bytes read through it.
struct Counted<R> {
    inner: R,
    bytes: u64,
}

impl<R> Counted<R> {
    fn new(inner: R) -> Self {
        Self { inner, bytes: 0 }
    }

    /// Bytes read since the last call.
    fn take(&mut self) -> u64 {
        std::mem::take(&mut self.bytes)
    }
}

impl<R: Read> Read for Counted<R> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = self.inner.read(buf)?;
        self.bytes += n as u64;
        Ok(n)
    }
}

#[cfg(test)]
mod tests {
    use std::net::{Shutdown, TcpListener, TcpStream};

    use mavlink::dialects::ardupilotmega::{
        HEARTBEAT_DATA, MavAutopilot, MavCmd, MavMessage, MavType,
    };

    use super::*;
    use crate::frame::encoded;
    use crate::gate::Vehicle;

    #[test]
    fn a_failed_write_closes_the_sitl_connection() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let mut reader = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let _sitl = listener.accept().unwrap();
        let stream = reader.try_clone().unwrap();
        stream.shutdown(Shutdown::Write).unwrap();
        let writer = SitlWriter::new(Mutex::new(Some(Box::new(stream))));
        let flow = Flow::default();
        let frame = encoded(&MavMessage::HEARTBEAT(HEARTBEAT_DATA::default()));
        assert!(!write_to_sitl(&frame, &writer, &flow));
        assert!(writer.lock().unwrap().is_none());
        assert_eq!(flow.snapshot().dropped.send_failed, 1);
        assert_eq!(
            reader.read(&mut [0; 16]).unwrap(),
            0,
            "the reader must reconnect"
        );
    }

    #[test]
    fn the_bridge_asks_for_the_position_stream_while_it_has_no_position() {
        let control = ControlStats::default();
        let mut probe = Probe::default();
        let first = probe.next_request(1, 1, &control, true).unwrap();
        let Ok(MavMessage::COMMAND_LONG(asked)) = first.decode() else {
            panic!("expected a stream request, got {}", first.message_id());
        };
        assert_eq!(asked.command, MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL);
        assert!((asked.param1 - 33.0).abs() < f32::EPSILON);
        // Not again within the interval: the marker probe goes next.
        let next = probe.next_request(1, 1, &control, true).unwrap();
        assert!(matches!(
            next.decode(),
            Ok(MavMessage::PARAM_REQUEST_READ(_))
        ));
        // With a fresh position nothing is asked for it.
        let fresh = Probe::default()
            .next_request(1, 1, &control, false)
            .unwrap();
        assert!(matches!(
            fresh.decode(),
            Ok(MavMessage::PARAM_REQUEST_READ(_))
        ));
    }

    #[test]
    fn a_vehicle_type_the_gate_does_not_know_replaces_the_one_before() {
        let heartbeat = |mavtype| {
            encoded(&MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                mavtype,
                autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                ..HEARTBEAT_DATA::default()
            }))
        };
        let control = ControlStats::default();
        learn_vehicle(&heartbeat(MavType::MAV_TYPE_QUADROTOR), &control);
        assert_eq!(
            control.snapshot().counts.autopilot.vehicle,
            Some(Vehicle::Copter)
        );
        learn_vehicle(&heartbeat(MavType::MAV_TYPE_FIXED_WING), &control);
        assert_eq!(control.snapshot().counts.autopilot.vehicle, None);
    }
}
