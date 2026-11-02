//! The vehicle writes control to its autopilot only after the autopilot has reported the SITL marker
//! parameter: until then an envelope that passes every freshness check goes to the logging sink.
//! A stand-in autopilot on loopback sends a copter's heartbeat and answers the marker probe when
//! told to; the test drives the vehicle's control endpoint like a controller would.

use std::io::Write;
use std::net::{TcpListener, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::dialects::ardupilotmega::{
    COMMAND_LONG_DATA, HEARTBEAT_DATA, MANUAL_CONTROL_DATA, MavAutopilot, MavCmd, MavMessage,
    MavType, PARAM_VALUE_DATA,
};

use crate::auth::{self, Signer, VehicleId};
use crate::autopilot::Autopilot;
use crate::clock::ClockSource;
use crate::envelope::{
    self, ControlLimits, Envelope, Kind, Outcome, Refusal, ToGround, ToVehicle, encode_frame,
};
use crate::frame::{FrameReader, datagram_frames};
use crate::identity::{ControlTarget, Mode, SITL_MARKER};
use crate::telemetry::TelemetryLimits;
use crate::vehicle::{self, ControllerConfig, VehicleConfig, VehicleStats};

const LIMITS: ControlLimits = ControlLimits {
    challenge_lifetime_ms: 1500,
    setpoint_lifetime_ms: 500,
    lease_lifetime_ms: 3000,
    liveness_interval_ms: 1000,
    ack_window_ms: 4000,
    throttle_release_deadline_ms: 1825,
};

fn bytes(message: &MavMessage) -> Vec<u8> {
    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };
    let mut bytes = Vec::new();
    mavlink::write_v2_msg(&mut bytes, header, message).unwrap();
    bytes
}

/// A copter that answers the marker probe once `identified` is set; counts the `MANUAL_CONTROL`
/// frames written to it.
fn stand_in_autopilot(
    identified: Arc<AtomicBool>,
    written: Arc<AtomicU32>,
) -> std::net::SocketAddr {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut out = stream.try_clone().unwrap();
        let heartbeat = bytes(&MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            ..HEARTBEAT_DATA::default()
        }));
        let marker = bytes(&MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_id: SITL_MARKER.into(),
            ..PARAM_VALUE_DATA::default()
        }));
        stream
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();
        let mut frames = FrameReader::new(stream);
        loop {
            out.write_all(&heartbeat).unwrap();
            while let Ok(frame) = frames.next_frame() {
                match frame.decode() {
                    Ok(MavMessage::PARAM_REQUEST_READ(_)) if identified.load(Ordering::Relaxed) => {
                        out.write_all(&marker).unwrap();
                    }
                    Ok(MavMessage::MANUAL_CONTROL(_)) => {
                        written.fetch_add(1, Ordering::Relaxed);
                    }
                    _ => {}
                }
            }
        }
    });
    addr
}

/// What the controller needs from the vehicle: its lease, epoch and newest challenge.
struct Controller {
    socket: UdpSocket,
    vehicle: std::net::SocketAddr,
    epoch: u64,
    lease: u64,
    challenge: u64,
    seq: u64,
    results: Vec<Outcome>,
    signer: Signer,
}

impl Controller {
    fn take(vehicle: std::net::SocketAddr, signer: Signer) -> Self {
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        socket
            .set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        let request = ToVehicle::LeaseRequest {
            peer: "test".to_owned(),
            nonce: 1,
            sig: signer.sign_request(1).ok(),
        };
        socket
            .send_to(&envelope::encode(&request).unwrap(), vehicle)
            .unwrap();
        let mut controller = Self {
            socket,
            vehicle,
            epoch: 0,
            lease: 0,
            challenge: 0,
            seq: 0,
            results: Vec::new(),
            signer,
        };
        let deadline = Instant::now() + Duration::from_secs(5);
        while controller.challenge == 0 && Instant::now() < deadline {
            controller.listen();
        }
        assert_ne!(controller.challenge, 0, "no lease and challenge");
        controller
    }

    /// Reads what the vehicle sent, keeping the newest challenge.
    fn listen(&mut self) {
        let mut buf = [0_u8; 1200];
        while let Ok((len, _)) = self.socket.recv_from(&mut buf) {
            match envelope::decode::<ToGround>(&buf[..len]) {
                Ok(ToGround::LeaseGrant { lease, epoch, .. }) => {
                    self.lease = lease;
                    self.epoch = epoch;
                }
                Ok(ToGround::Challenge { token, .. }) => self.challenge = token,
                Ok(ToGround::CommandResult { outcome, .. }) => self.results.push(outcome),
                _ => {}
            }
        }
    }

    /// Gives the lease back, signed.
    fn release(&self) {
        let sig = self.signer.sign_release(self.epoch, self.lease).ok();
        let release = ToVehicle::LeaseRelease {
            lease: self.lease,
            sig,
        };
        let bytes = envelope::encode(&release).unwrap();
        self.socket.send_to(&bytes, self.vehicle).unwrap();
    }

    fn drive(&mut self) {
        self.send(
            Kind::Continuous,
            &MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA::default()),
        );
    }

    fn arm(&mut self, txn: u64) {
        let arm = COMMAND_LONG_DATA {
            command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            param1: 1.0,
            ..COMMAND_LONG_DATA::default()
        };
        self.send(Kind::Discrete { txn }, &MavMessage::COMMAND_LONG(arm));
    }

    fn send(&mut self, kind: Kind, message: &MavMessage) {
        self.listen();
        self.seq += 1;
        let frame = datagram_frames(&bytes(message)).next().unwrap();
        let envelope = self
            .signer
            .sign(Envelope {
                epoch: self.epoch,
                lease: self.lease,
                seq: self.seq,
                challenge: self.challenge,
                kind,
                mavlink: Some(encode_frame(&frame)),
                region: None,
                sent_unix_ms: 0,
                sig: None,
            })
            .unwrap();
        let datagram = envelope::encode(&ToVehicle::Envelope(envelope)).unwrap();
        self.socket.send_to(&datagram, self.vehicle).unwrap();
    }
}

fn wait_for(what: &str, mut holds: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while !holds() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        thread::sleep(Duration::from_millis(50));
    }
}

const VEHICLE: &str = "test-vehicle";

/// A vehicle in SITL mode against the stand-in autopilot, with the key of the controller at
/// 127.0.0.1.
struct Bench {
    stats: Arc<VehicleStats>,
    control: std::net::SocketAddr,
    signer: Signer,
    vehicle: VehicleId,
    identified: Arc<AtomicBool>,
    written: Arc<AtomicU32>,
}

fn bench() -> Bench {
    let identified = Arc::new(AtomicBool::new(false));
    let written = Arc::new(AtomicU32::new(0));
    let autopilot = stand_in_autopilot(Arc::clone(&identified), Arc::clone(&written));
    let free_port = || {
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        socket.local_addr().unwrap()
    };
    let (relay, control) = (free_port(), free_port());
    let keys =
        std::env::temp_dir().join(format!("sat-teleop-keys-{}-{control}", std::process::id()));
    std::fs::create_dir_all(&keys).unwrap();
    auth::generate(&keys, "test").unwrap();
    let vehicle: VehicleId = VEHICLE.parse().unwrap();
    let signer = Signer::load(&keys.join("ctl-test.key"), &vehicle).unwrap();
    let config = VehicleConfig {
        autopilot: Autopilot::Tcp(autopilot),
        relay,
        control,
        vehicle_id: vehicle.clone(),
        controllers: vec![ControllerConfig {
            address: "127.0.0.1".parse().unwrap(),
            public_key: keys.join("ctl-test.pub"),
        }],
        mode: Mode::Sitl,
        clock: ClockSource::Shared,
        #[cfg(feature = "testbed")]
        sitl_in_real_mode: false,
        require: Vec::new(),
        allow_missions: false,
    };
    let limits = TelemetryLimits {
        bytes_per_s: 8000,
        offered_percent: 250,
        essential_bytes_per_s: 200,
    };
    let stats = Arc::new(VehicleStats::new(&config));
    let (started, running) = mpsc::channel();
    {
        let stats = Arc::clone(&stats);
        thread::spawn(move || {
            started.send(()).unwrap();
            let _ = vehicle::run(config, limits, LIMITS, || false, &stats);
        });
    }
    running.recv().unwrap();
    wait_for("the vehicle's type from the heartbeat", || {
        stats.snapshot().control.counts.autopilot.vehicle.is_some()
    });
    Bench {
        stats,
        control,
        signer,
        vehicle,
        identified,
        written,
    }
}

#[test]
fn control_reaches_the_autopilot_only_after_it_reports_the_sitl_marker() {
    let Bench {
        stats,
        control,
        signer,
        identified,
        written,
        ..
    } = bench();
    let mut controller = Controller::take(control, signer);
    for _ in 0..5 {
        controller.drive();
        thread::sleep(Duration::from_millis(100));
    }
    let snapshot = stats.snapshot().control;
    assert_eq!(snapshot.target, ControlTarget::LoggingSink);
    assert!(snapshot.counts.sunk > 0, "the envelopes were not sunk");
    assert_eq!(
        written.load(Ordering::Relaxed),
        0,
        "control reached the autopilot"
    );

    controller.arm(1);
    wait_for("the answer to a command that went to the sink", || {
        controller.listen();
        !controller.results.is_empty()
    });
    assert!(
        matches!(
            controller.results.as_slice(),
            [Outcome::Rejected {
                reason: Refusal::NoDestination
            }]
        ),
        "a sunk command was answered {:?}",
        controller.results
    );
    assert_eq!(stats.snapshot().control.counts.applied.discrete, 0);

    identified.store(true, Ordering::Relaxed);
    wait_for("the marker", || {
        stats.snapshot().control.target == ControlTarget::Autopilot
    });
    wait_for("a setpoint at the autopilot", || {
        controller.drive();
        thread::sleep(Duration::from_millis(100));
        written.load(Ordering::Relaxed) > 0
    });
}

/// A signed request that was recorded cannot be sent again, nor can an older one.
#[test]
fn a_recorded_lease_request_is_refused_again() {
    let Bench {
        stats,
        control,
        signer,
        ..
    } = bench();
    let controller = Controller::take(control, signer);
    for nonce in [1, 0] {
        let request = ToVehicle::LeaseRequest {
            peer: "relay".to_owned(),
            nonce,
            sig: controller.signer.sign_request(nonce).ok(),
        };
        let bytes = envelope::encode(&request).unwrap();
        controller.socket.send_to(&bytes, control).unwrap();
    }
    wait_for("the two refusals", || {
        let rejected = stats.snapshot().control.counts.rejected;
        rejected.get(&Refusal::ReplayedRequest) == Some(&2)
    });
    let counts = stats.snapshot().control.counts;
    assert_eq!(counts.grants, 1, "a replay was granted");
    assert_eq!(counts.lease.map(|l| l.id), Some(controller.lease));
}

/// A relay that can spoof the holder's address still cannot end or take the lease without the
/// holder's key.
#[test]
fn a_lease_request_or_release_needs_the_controllers_signature() {
    let Bench {
        stats,
        control,
        signer,
        vehicle,
        ..
    } = bench();
    let controller = Controller::take(control, signer);
    let lease = controller.lease;
    let stranger = Signer::random(&vehicle).unwrap();
    let datagrams = [
        ToVehicle::LeaseRelease { lease, sig: None },
        ToVehicle::LeaseRelease {
            lease,
            sig: stranger.sign_release(controller.epoch, lease).ok(),
        },
        ToVehicle::LeaseRequest {
            peer: "relay".to_owned(),
            nonce: 2,
            sig: None,
        },
        ToVehicle::LeaseRequest {
            peer: "relay".to_owned(),
            nonce: 3,
            sig: stranger.sign_request(3).ok(),
        },
    ];
    for datagram in datagrams {
        let bytes = envelope::encode(&datagram).unwrap();
        controller.socket.send_to(&bytes, control).unwrap();
    }
    wait_for("the four refusals", || {
        let rejected = stats.snapshot().control.counts.rejected;
        rejected.values().sum::<u64>() == 4
    });
    let counts = stats.snapshot().control.counts;
    assert_eq!(counts.rejected.get(&Refusal::Unsigned), Some(&2));
    assert_eq!(counts.rejected.get(&Refusal::BadSignature), Some(&2));
    assert_eq!(counts.grants, 1, "a request was granted");
    assert_eq!(counts.lease.map(|l| l.id), Some(lease), "the lease changed");

    controller.release();
    wait_for("the holder's own release", || {
        stats.snapshot().control.counts.lease.is_none()
    });
}
