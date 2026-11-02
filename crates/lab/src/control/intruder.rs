//! `checks` as a second controller on the overlay (10.77.0.4): it takes the lease and sends
//! envelopes the vehicle must refuse, each carrying a forced arm or full throttle that would change
//! SITL if applied, between valid envelopes that keep the lease and show the vehicle still takes
//! its envelopes. Each kind counts its own sequence numbers.

use std::fmt;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::path::Path;
use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::dialects::ardupilotmega::{
    HEARTBEAT_DATA, MavAutopilot, MavCmd, MavMessage, MavState, MavType,
};

use sat_teleop_control::auth::{Signer, VehicleId};
use sat_teleop_control::envelope::{
    self, ControlLimits, Envelope, Kind, MAX_ENVELOPE, Outcome, Refusal, ToGround, ToVehicle,
    encode_frame,
};
use sat_teleop_control::frame::{Frame, MAX_DATAGRAM, datagram_frames};
use sat_teleop_control::unix_ms;

use super::throttle;
use crate::mavlink::command;

/// `ARM_DISARM` param 2 that arms without `ArduPilot`'s pre-arm checks (its
/// `magic_force_arm_value`), as `QGroundControl`'s Force Arm sends it.
const FORCE_ARM: f32 = 2989.0;
/// `MANUAL_CONTROL` z: full throttle ahead, and none.
const FULL_THROTTLE: i16 = 1000;
const NO_THROTTLE: i16 = 0;
/// `AUTOPILOT_VERSION`, as `REQUEST_MESSAGE` takes it: a harmless discrete command.
const AUTOPILOT_VERSION: f32 = 148.0;
/// Receive timeout, and so the granularity of waiting.
const POLL: Duration = Duration::from_millis(20);
/// A reply takes a round trip: a GEO one with margin.
const REPLY_TIMEOUT: Duration = Duration::from_secs(3);
/// Long enough for an envelope to arrive before the next one leaves, despite jitter, and short
/// enough to keep the newest challenge fresh.
const SETTLE: Duration = Duration::from_secs(1);
/// The least time between two valid liveness envelopes: over twice the largest jitter of a profile
/// (30 ms), so the link cannot swap them and the vehicle refuse the first as a reordered number.
const LIVENESS_GAP: Duration = Duration::from_millis(100);
/// More than any run receives; later replies are not kept.
const MAX_HEARD: usize = 10_000;

/// How the vehicle answered an envelope it had to refuse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Answer {
    Refused(Refusal),
    /// A command result other than a rejection: the vehicle took the envelope.
    Taken,
}

impl fmt::Display for Answer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Refused(reason) => reason.fmt(f),
            Self::Taken => f.write_str("taken"),
        }
    }
}

/// One envelope the vehicle must refuse, and its answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Case {
    pub(super) name: &'static str,
    /// Reasons that count as the right refusal.
    pub(super) expected: &'static [Refusal],
    /// The vehicle's answer; `None` without one.
    pub(super) got: Option<Answer>,
}

/// The envelopes `probe` sent: the invalid ones with their answers, and the valid ones by kind.
#[derive(Debug, Default)]
pub(super) struct Probe {
    pub(super) cases: Vec<Case>,
    pub(super) liveness: u64,
    /// Neutral setpoints.
    pub(super) continuous: u64,
    /// `REQUEST_MESSAGE`s, and those the vehicle answered as applied.
    pub(super) discrete: u64,
    pub(super) discrete_applied: u64,
}

/// The last sequence number sent of each kind.
#[derive(Debug, Default)]
struct Seqs {
    liveness: u64,
    continuous: u64,
    discrete: u64,
}

impl Seqs {
    fn next(&mut self, kind: Kind) -> u64 {
        let last = match kind {
            Kind::Liveness => &mut self.liveness,
            Kind::Continuous => &mut self.continuous,
            Kind::Discrete { .. } => &mut self.discrete,
        };
        *last += 1;
        *last
    }
}

struct Heard {
    at: Instant,
    message: ToGround,
}

pub(super) struct Intruder {
    socket: UdpSocket,
    vehicle: SocketAddr,
    heard: Vec<Heard>,
    /// The lease the vehicle granted, and its session.
    lease: Option<(u64, u64)>,
    seqs: Seqs,
    last_liveness: Option<Instant>,
    txn: u64,
    /// `checks`' own key, which the vehicle holds the public half of.
    signer: Signer,
    vehicle_id: VehicleId,
}

impl Intruder {
    pub(super) fn bind(
        local: SocketAddr,
        vehicle: SocketAddr,
        key: &Path,
        vehicle_id: &VehicleId,
    ) -> io::Result<Self> {
        let signer = Signer::load(key, vehicle_id).map_err(io::Error::other)?;
        let socket = UdpSocket::bind(local)?;
        socket.set_read_timeout(Some(POLL))?;
        Ok(Self {
            socket,
            vehicle,
            heard: Vec::new(),
            lease: None,
            seqs: Seqs::default(),
            last_liveness: None,
            txn: 0,
            signer,
            vehicle_id: vehicle_id.clone(),
        })
    }

    /// Asks for the lease; the vehicle's answer (grant or denial), if one came.
    pub(super) fn request(&mut self) -> Option<ToGround> {
        let nonce = unix_ms();
        let request = ToVehicle::LeaseRequest {
            peer: "checks".to_owned(),
            nonce,
            sig: self.signer.sign_request(nonce).ok(),
        };
        let answer = |m: &ToGround| match m {
            ToGround::LeaseGrant { nonce: n, .. } | ToGround::LeaseDenied { nonce: n, .. } => {
                *n == nonce
            }
            _ => false,
        };
        if !self.send(&request) {
            return None;
        }
        let reply = self.wait(|m| answer(m).then(|| m.clone()))?;
        if let ToGround::LeaseGrant { lease, epoch, .. } = reply {
            self.lease = Some((lease, epoch));
            self.seqs = Seqs::default();
        }
        Some(reply)
    }

    pub(super) fn release(&self) -> bool {
        self.lease.is_some_and(|(lease, epoch)| {
            let sig = self.signer.sign_release(epoch, lease).ok();
            self.send(&ToVehicle::LeaseRelease { lease, sig })
        })
    }

    /// Reads until a challenge arrives or `until`; the newest one.
    pub(super) fn await_challenge(&mut self, until: Instant) -> Option<u64> {
        let from = self.heard.len();
        self.pump_until(until, |heard| {
            heard
                .get(from..)
                .unwrap_or_default()
                .iter()
                .any(|h| matches!(h.message, ToGround::Challenge { .. }))
        });
        self.challenge().map(|(token, _)| token)
    }

    /// An envelope of `epoch` under lease 1 with challenge 1, as if `checks` held a lease: the
    /// vehicle must refuse it.
    pub(super) fn unleased(&mut self, epoch: u64) -> Option<Case> {
        let kind = self.txn();
        let envelope = self
            .signer
            .sign(Envelope {
                epoch,
                lease: 1,
                seq: 1,
                challenge: 1,
                kind,
                mavlink: Some(encode_frame(&frame(&arm())?)),
                sent_unix_ms: unix_ms(),
                sig: None,
            })
            .ok()?;
        Some(self.refused(
            "without a lease",
            &[Refusal::NoLease, Refusal::NotHolder],
            &envelope,
        ))
    }

    /// The envelopes a misbehaving lease holder might send, each answered with a refusal, between
    /// valid ones: one before every case, so the lease never lapses.
    pub(super) fn probe(&mut self, limits: ControlLimits) -> Probe {
        let mut probe = Probe::default();
        let Some((_, epoch)) = self.lease else {
            return probe;
        };
        self.liveness(&mut probe);
        if let Some(e) = self.forced_arm() {
            let old = self.resign(Envelope {
                epoch: epoch.wrapping_sub(1),
                ..e
            });
            probe
                .cases
                .push(self.refused("old epoch", &[Refusal::WrongEpoch], &old));
        }
        // A neutral setpoint lands, then full throttle with its sequence number.
        let landed = self.setpoint(&mut probe);
        self.pump_until(Instant::now() + SETTLE, |_| false);
        let full = self.envelope_for(Kind::Continuous, &throttle(FULL_THROTTLE));
        if let (Some(seq), Some(e)) = (landed, full) {
            let duplicate = self.resign(Envelope { seq, ..e });
            probe
                .cases
                .push(self.refused("duplicate seq", &[Refusal::DuplicateSeq], &duplicate));
        }
        // A discrete number is set aside, a later one lands, then a forced arm with the first.
        let reserved = self.seqs.next(Kind::Discrete { txn: 0 });
        let applied = self.request_message(&mut probe);
        if let Some(e) = self.forced_arm() {
            let reordered = self.resign(Envelope { seq: reserved, ..e });
            probe
                .cases
                .push(self.refused("reordered seq", &[Refusal::ReorderedSeq], &reordered));
        }
        // A forced arm reusing the applied request's transaction.
        if let Some(e) = applied.and_then(|kind| self.envelope_for(kind, &arm())) {
            probe
                .cases
                .push(self.refused("reused transaction", &[Refusal::TxnConflict], &e));
        }
        self.liveness(&mut probe);
        let stale = self.aged_challenge(limits);
        self.liveness(&mut probe);
        if let Some(case) = stale.and_then(|token| self.stale(token)) {
            probe.cases.push(case);
        }
        self.liveness(&mut probe);
        let forged = self.forged(Some(&mut probe));
        probe.cases.extend(forged);
        self.liveness(&mut probe);
        if let Some(case) = self.oversize() {
            probe.cases.push(case);
        }
        self.liveness(&mut probe);
        // The last one lands before anyone reads the vehicle's counters.
        self.pump_until(Instant::now() + SETTLE, |_| false);
        probe
    }

    /// `checks` signs `envelope` again after a change that is not the one under test.
    fn resign(&self, envelope: Envelope) -> Envelope {
        let bare = Envelope {
            sig: None,
            ..envelope
        };
        self.signer.sign(bare.clone()).unwrap_or(bare)
    }

    /// The ways a message fails authentication. Envelopes carry a forced arm or full throttle that
    /// would change SITL if applied: no signature, a signature of another key, the right key's
    /// signature made for another vehicle, and a valid signature over a harmless setpoint with its
    /// frame swapped for full throttle. Releases of the held lease would end it if applied: with
    /// no signature and with another key's.
    pub(super) fn forged(&mut self, mut probe: Option<&mut Probe>) -> Vec<Case> {
        let mut cases = Vec::new();
        self.keep_alive(&mut probe);
        if let Some(e) = self.forced_arm() {
            let unsigned = Envelope { sig: None, ..e };
            cases.push(self.refused("unsigned", &[Refusal::Unsigned], &unsigned));
        }
        self.keep_alive(&mut probe);
        if let (Some(e), Ok(other)) = (self.forced_arm(), Signer::random(&self.vehicle_id)) {
            let wrong = other.sign(Envelope { sig: None, ..e }).ok();
            cases.extend(wrong.map(|e| self.refused("wrong key", &[Refusal::BadSignature], &e)));
        }
        self.keep_alive(&mut probe);
        let elsewhere = "another-vehicle".parse().ok();
        if let (Some(e), Some(id)) = (self.forced_arm(), elsewhere) {
            let signed = self.signer.for_vehicle(&id).sign(e).ok();
            let name = "other vehicle";
            cases.extend(signed.map(|e| self.refused(name, &[Refusal::BadSignature], &e)));
        }
        self.keep_alive(&mut probe);
        cases.extend(self.forged_releases(&mut probe));
        self.keep_alive(&mut probe);
        let harmless = self.envelope_for(Kind::Continuous, &throttle(NO_THROTTLE));
        let full = frame(&throttle(FULL_THROTTLE)).map(|f| encode_frame(&f));
        if let (Some(e), Some(full)) = (harmless, full) {
            let tampered = Envelope {
                mavlink: Some(full),
                ..e
            };
            cases.push(self.refused("tampered", &[Refusal::BadSignature], &tampered));
        }
        cases
    }

    /// Releases of the held lease that carry no signature, then another key's.
    fn forged_releases(&mut self, probe: &mut Option<&mut Probe>) -> Vec<Case> {
        let Some((lease, epoch)) = self.lease else {
            return Vec::new();
        };
        let strangers = Signer::random(&self.vehicle_id)
            .and_then(|other| other.sign_release(epoch, lease))
            .ok();
        let releases: [(&'static str, &'static [Refusal], Option<String>); 2] = [
            ("release unsigned", &[Refusal::Unsigned], None),
            ("release wrong key", &[Refusal::BadSignature], strangers),
        ];
        let mut cases = Vec::new();
        for (name, expected, sig) in releases {
            self.keep_alive(probe);
            let sent = self.send(&ToVehicle::LeaseRelease { lease, sig });
            let unread = |m: &ToGround| match m {
                ToGround::Rejected { seq: None, reason } => Some(Answer::Refused(*reason)),
                _ => None,
            };
            let got = sent.then(|| self.wait(unread)).flatten();
            cases.push(Case {
                name,
                expected,
                got,
            });
        }
        cases
    }

    /// An envelope with the lease `checks` held before, now that another controller holds it.
    pub(super) fn after_handover(&mut self) -> Option<Case> {
        let e = self.forced_arm()?;
        Some(self.refused("from a former holder", &[Refusal::NotHolder], &e))
    }

    /// The newest challenge, once it is older than its lifetime: expired, or no longer one of the
    /// four most recent, by the time an envelope with it arrives.
    fn aged_challenge(&mut self, limits: ControlLimits) -> Option<u64> {
        let (token, received) = self.challenge()?;
        let past = Duration::from_millis(limits.challenge_lifetime_ms.saturating_add(100));
        self.pump_until(received + past, |_| false);
        Some(token)
    }

    fn stale(&mut self, token: u64) -> Option<Case> {
        let e = self.forced_arm()?;
        let stale = self.resign(Envelope {
            challenge: token,
            ..e
        });
        Some(self.refused(
            "expired challenge",
            &[Refusal::ChallengeExpired, Refusal::UnknownChallenge],
            &stale,
        ))
    }

    /// A valid envelope padded with JSON whitespace past `MAX_ENVELOPE`.
    fn oversize(&mut self) -> Option<Case> {
        let e = self.forced_arm()?;
        let mut bytes = envelope::encode(&ToVehicle::Envelope(e)).ok()?;
        bytes.resize(MAX_ENVELOPE + 100, b' ');
        let sent = self.socket.send_to(&bytes, self.vehicle).is_ok();
        let unread = |m: &ToGround| match m {
            ToGround::Rejected { seq: None, reason } => Some(Answer::Refused(*reason)),
            _ => None,
        };
        let got = sent.then(|| self.wait(unread)).flatten();
        Some(Case {
            name: "oversize",
            expected: &[Refusal::Oversize],
            got,
        })
    }

    /// A valid liveness envelope between two slow cases, when the caller counts them: on a long
    /// link each case waits a round trip, and the lease lapses after a few without one.
    fn keep_alive(&mut self, probe: &mut Option<&mut Probe>) {
        if let Some(probe) = probe.as_deref_mut() {
            self.liveness(probe);
        }
    }

    /// Sends a valid liveness envelope with a ground station's heartbeat, `LIVENESS_GAP` after
    /// the last.
    fn liveness(&mut self, probe: &mut Probe) {
        let not_before = self
            .last_liveness
            .map_or_else(Instant::now, |at| at + LIVENESS_GAP);
        self.pump_until(not_before, |_| false);
        self.last_liveness = Some(Instant::now());
        let sent = self
            .envelope_for(Kind::Liveness, &gcs_heartbeat())
            .is_some_and(|e| self.send(&ToVehicle::Envelope(e)));
        probe.liveness += u64::from(sent);
    }

    /// Sends a valid neutral setpoint; its sequence number.
    fn setpoint(&mut self, probe: &mut Probe) -> Option<u64> {
        let e = self.envelope_for(Kind::Continuous, &throttle(NO_THROTTLE))?;
        let seq = e.seq;
        let sent = self.send(&ToVehicle::Envelope(e));
        probe.continuous += u64::from(sent);
        sent.then_some(seq)
    }

    /// Sends a valid `REQUEST_MESSAGE` and waits for its command result; its kind (with its
    /// transaction) once applied.
    fn request_message(&mut self, probe: &mut Probe) -> Option<Kind> {
        let kind = self.txn();
        let request = command(MavCmd::MAV_CMD_REQUEST_MESSAGE, AUTOPILOT_VERSION, 0.0);
        let e = self.envelope_for(kind, &request)?;
        if !self.send(&ToVehicle::Envelope(e)) {
            return None;
        }
        probe.discrete += 1;
        let applied = |m: &ToGround| match m {
            ToGround::CommandResult { txn, outcome } if Kind::Discrete { txn: *txn } == kind => {
                Some(*outcome == Outcome::Applied)
            }
            _ => None,
        };
        let applied = self.wait(applied).unwrap_or(false);
        probe.discrete_applied += u64::from(applied);
        applied.then_some(kind)
    }

    /// Sends `envelope` and waits for the vehicle's refusal of it.
    fn refused(
        &mut self,
        name: &'static str,
        expected: &'static [Refusal],
        envelope: &Envelope,
    ) -> Case {
        let sent = self.send(&ToVehicle::Envelope(envelope.clone()));
        let got = sent.then(|| self.wait(|m| refusal(m, envelope))).flatten();
        Case {
            name,
            expected,
            got,
        }
    }

    /// The next envelope of `kind` under the held lease, with the newest challenge.
    fn envelope_for(&mut self, kind: Kind, message: &MavMessage) -> Option<Envelope> {
        let (lease, epoch) = self.lease?;
        let (challenge, _) = self.challenge()?;
        self.signer
            .sign(Envelope {
                epoch,
                lease,
                seq: self.seqs.next(kind),
                challenge,
                kind,
                mavlink: Some(encode_frame(&frame(message)?)),
                sent_unix_ms: unix_ms(),
                sig: None,
            })
            .ok()
    }

    fn txn(&mut self) -> Kind {
        self.txn += 1;
        Kind::Discrete { txn: self.txn }
    }

    /// The next envelope: a forced arm in a transaction of its own.
    fn forced_arm(&mut self) -> Option<Envelope> {
        let kind = self.txn();
        self.envelope_for(kind, &arm())
    }

    /// The newest challenge and when it arrived.
    fn challenge(&self) -> Option<(u64, Instant)> {
        self.heard.iter().rev().find_map(|h| match h.message {
            ToGround::Challenge { token, .. } => Some((token, h.at)),
            _ => None,
        })
    }

    /// Reads for up to `REPLY_TIMEOUT` until `pick` finds an answer among what arrives.
    fn wait<T>(&mut self, pick: impl Fn(&ToGround) -> Option<T>) -> Option<T> {
        let from = self.heard.len();
        let until = Instant::now() + REPLY_TIMEOUT;
        self.pump_until(until, |heard| {
            heard
                .get(from..)
                .unwrap_or_default()
                .iter()
                .any(|h| pick(&h.message).is_some())
        });
        self.heard
            .get(from..)?
            .iter()
            .find_map(|h| pick(&h.message))
    }

    fn send(&self, message: &ToVehicle) -> bool {
        envelope::encode(message)
            .is_ok_and(|bytes| self.socket.send_to(&bytes, self.vehicle).is_ok())
    }

    /// Reads until `until` or until `done` holds; returns `done`.
    fn pump_until(&mut self, until: Instant, done: impl Fn(&[Heard]) -> bool) -> bool {
        let mut buf = [0_u8; MAX_DATAGRAM];
        loop {
            if done(&self.heard) {
                return true;
            }
            if Instant::now() >= until {
                return false;
            }
            let Ok((len, from)) = self.socket.recv_from(&mut buf) else {
                continue;
            };
            let at = Instant::now();
            let message = envelope::decode::<ToGround>(buf.get(..len).unwrap_or_default());
            let keep = from == self.vehicle && self.heard.len() < MAX_HEARD;
            if let (Ok(message), true) = (message, keep) {
                self.heard.push(Heard { at, message });
            }
        }
    }
}

/// The vehicle's answer to `envelope` in `message`: a discrete envelope is answered with its
/// command result, a refused other one with its sequence number.
fn refusal(message: &ToGround, envelope: &Envelope) -> Option<Answer> {
    match (message, envelope.kind) {
        (ToGround::CommandResult { txn, outcome }, Kind::Discrete { txn: sent })
            if *txn == sent =>
        {
            Some(match outcome {
                Outcome::Rejected { reason } => Answer::Refused(*reason),
                Outcome::Applied | Outcome::Repeated | Outcome::Unsupported => Answer::Taken,
            })
        }
        (ToGround::Rejected { seq, reason }, _) if *seq == Some(envelope.seq) => {
            Some(Answer::Refused(*reason))
        }
        _ => None,
    }
}

/// A forced `ARM_DISARM`: SITL would arm at once if it got it.
pub(super) fn arm() -> MavMessage {
    command(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, FORCE_ARM)
}

fn gcs_heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
        ..HEARTBEAT_DATA::default()
    })
}

/// `message` as one `MAVLink` 2 frame from `QGroundControl`'s ids.
fn frame(message: &MavMessage) -> Option<Frame> {
    let header = MavHeader {
        system_id: 255,
        component_id: 190,
        sequence: 0,
    };
    let mut bytes = Vec::new();
    mavlink::write_v2_msg(&mut bytes, header, message).ok()?;
    datagram_frames(&bytes).next()
}
