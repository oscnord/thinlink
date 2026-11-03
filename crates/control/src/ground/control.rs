//! The ground adapter as the ground stations' only command endpoint. It holds the vehicle's lease
//! when asked to (`take`), wraps a station's heartbeat, `MANUAL_CONTROL` and discrete commands into
//! envelopes while it holds the lease with a fresh challenge, and answers a station's
//! state-changing command itself otherwise: `MAV_RESULT_DENIED` without the lease,
//! `MAV_RESULT_UNSUPPORTED` for a command no envelope may carry (a mission upload message gets the
//! same answers as a `MISSION_ACK`). Nothing waits to be sent later.

use std::collections::{BTreeMap, VecDeque};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    COMMAND_ACK_DATA, MISSION_ACK_DATA, MavCmd, MavMessage, MavMissionResult, MavMissionType,
    MavResult,
};
use mavlink::{MavHeader, Message};
use serde::{Deserialize, Serialize};

use sat_teleop_policy::clock::{self, Reading};
use sat_teleop_policy::control::Kind as Class;
use sat_teleop_policy::region::NormRect;

use super::{Camera, Station};
use crate::auth::Signer;
use crate::envelope::{
    self, ControlLimits, Envelope, Kind, Outcome, Refusal, TRACKING_TICK, ToGround, ToVehicle,
    encode_frame,
};
use crate::frame::{Frame, MAX_DATAGRAM};
use crate::gate::CommandKey;
use crate::stats::{Link, LinkState};
use crate::{bind_overlay, unix_ms};

/// How long `take` waits for the vehicle's answer: a GEO round trip with margin.
const LEASE_ANSWER: Duration = Duration::from_secs(3);
/// Discrete commands remembered for the ACK window; the oldest is forgotten first.
const MAX_SENT: usize = 64;
const RECV_ERROR_PAUSE: Duration = Duration::from_millis(100);

#[derive(Debug, thiserror::Error)]
pub enum TakeError {
    #[error("the control socket is not up yet")]
    NotReady,
    #[error("cannot send the lease request: {0}")]
    Send(String),
    #[error("the vehicle denied the lease: {0}")]
    Denied(Refusal),
    #[error("no answer from the vehicle within {} s", LEASE_ANSWER.as_secs())]
    NoAnswer,
    #[error("the vehicle granted the lease but sent no challenge within {} s", LEASE_ANSWER.as_secs())]
    NoChallenge,
}

/// Where the adapter stands with the lease.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LeaseStatus {
    None,
    Requested,
    Held,
    Denied,
    /// The vehicle refused an envelope for a lease reason, or sent no challenge for a lease
    /// lifetime.
    Lost,
}

/// The lease as the adapter knows it (`GET /status`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Status {
    pub lease: LeaseStatus,
    pub id: Option<u64>,
    pub epoch: Option<u64>,
    /// Why it was denied or lost.
    pub reason: Option<String>,
    /// Since the newest challenge arrived, while held.
    pub challenge_age_ms: Option<u64>,
}

#[derive(Debug, Default, Clone, Serialize)]
pub struct ControlCounts {
    /// Envelopes sent, by kind; `keepalive` is liveness without a frame.
    pub wrapped: Wrapped,
    /// Heartbeats and `MANUAL_CONTROL` dropped without a lease and a fresh challenge.
    pub no_lease: u64,
    /// Commands answered `MAV_RESULT_DENIED` by the adapter.
    pub denied: u64,
    /// Commands answered `MAV_RESULT_UNSUPPORTED` by the adapter.
    pub unsupported: u64,
    /// Other state-changing messages, dropped, by name.
    pub refused: BTreeMap<&'static str, u64>,
    /// The vehicle's command results.
    pub results: Results,
    /// The vehicle's refusals, by reason.
    pub vehicle_rejected: BTreeMap<Refusal, u64>,
    pub send_failed: u64,
    /// Datagrams on the control socket from anyone but the vehicle, or unreadable.
    pub ignored: u64,
}

#[derive(Debug, Default, Clone, Copy, Serialize)]
pub struct Wrapped {
    pub keepalive: u64,
    pub liveness: u64,
    pub continuous: u64,
    pub discrete: u64,
    pub region: u64,
}

#[derive(Debug, Default, Clone, Copy, Serialize)]
pub struct Results {
    pub applied: u64,
    /// Retries the vehicle answered from its cache; the adapter acknowledged them `ACCEPTED`.
    pub repeated: u64,
    pub rejected: u64,
    pub unsupported: u64,
}

#[derive(Debug, Serialize)]
pub struct ControlSnapshot {
    pub link: LinkState,
    pub status: Status,
    #[serde(flatten)]
    pub counts: ControlCounts,
}

pub struct Controller {
    listen: SocketAddr,
    vehicle: SocketAddr,
    limits: ControlLimits,
    signer: Signer,
    /// The control socket, once `wg0` is up.
    socket: OnceLock<UdpSocket>,
    state: Mutex<State>,
    /// The camera component whose tracking this sends to the vehicle and whose status it feeds.
    camera: OnceLock<Arc<Camera>>,
    /// Signalled when a lease request is answered.
    answered: Condvar,
    link: Link,
}

#[derive(Default)]
struct State {
    lease: Lease,
    next_txn: u64,
    /// Discrete commands sent within the ACK window, oldest first.
    sent: VecDeque<Sent>,
    /// The nonce of the newest clock query, and the vehicle's answer to it.
    clock_nonce: u64,
    vehicle_clock: Option<(Option<u64>, Instant)>,
    /// When the newest clock query went out, and the round trip of its answer.
    clock_sent: Option<Instant>,
    clock_rtt: Option<Duration>,
    counts: ControlCounts,
}

#[derive(Default)]
enum Lease {
    #[default]
    None,
    Requested {
        nonce: u64,
    },
    Held(Held),
    Denied(Refusal),
    Lost(String),
}

struct Held {
    id: u64,
    epoch: u64,
    next_seq: NextSeq,
    /// The newest challenge token.
    challenge: Option<u64>,
    /// When the newest challenge arrived; the grant until the first one.
    heard_at: Instant,
    /// When the last envelope went out.
    last_sent: Instant,
}

/// The last sequence number sent of each kind; each kind counts its own.
#[derive(Default)]
struct NextSeq {
    liveness: u64,
    continuous: u64,
    discrete: u64,
    region: u64,
}

impl NextSeq {
    fn take(&mut self, kind: Class) -> u64 {
        let last = match kind {
            Class::Liveness => &mut self.liveness,
            Class::Continuous => &mut self.continuous,
            Class::Discrete => &mut self.discrete,
            Class::Region => &mut self.region,
        };
        *last += 1;
        *last
    }
}

/// A discrete command sent, kept for the ACK window whether or not its result came: a resend of it
/// reuses its transaction, and its result is answered to its station.
#[derive(Debug, Clone, Copy)]
struct Sent {
    txn: u64,
    station: usize,
    /// The command, for a resend to find it; a mission upload message is never resent as one (the
    /// station repeats it on its own timer and the autopilot answers a repeat).
    command: Option<CommandKey>,
    ack: AckTo,
    at: Instant,
}

/// A station's message, as the answer to it needs it: a `COMMAND_ACK` for a command, a
/// `MISSION_ACK` for a mission upload message. The answer comes from the message's target system
/// and component, which is how `QGroundControl` matches it.
#[derive(Debug, Clone, Copy)]
enum AckTo {
    Command {
        command: MavCmd,
        vehicle: (u8, u8),
    },
    Mission {
        mission_type: MavMissionType,
        vehicle: (u8, u8),
        /// The station's own system and component, the ack's target.
        station: (u8, u8),
    },
}

impl Controller {
    pub fn new(
        listen: SocketAddr,
        vehicle: SocketAddr,
        limits: ControlLimits,
        signer: Signer,
    ) -> Self {
        Self {
            listen,
            vehicle,
            limits,
            signer,
            socket: OnceLock::new(),
            state: Mutex::new(State::default()),
            camera: OnceLock::new(),
            answered: Condvar::new(),
            link: Link::default(),
        }
    }

    /// Asks the vehicle for the lease and waits for its answer and, after a grant, for the first
    /// challenge, so a command sent right after can go out. Holding it with a fresh challenge is
    /// enough; holding it without asks again.
    pub fn take(&self) -> Result<Status, TakeError> {
        let socket = self.socket.get().ok_or(TakeError::NotReady)?;
        let mut state = self.lock();
        state.expire(self.limits);
        if state.fresh(self.limits) {
            return Ok(state.status());
        }
        let nonce = unix_ms();
        let sig = self
            .signer
            .sign_request(nonce)
            .map_err(|e| TakeError::Send(e.to_string()))?;
        let request = ToVehicle::LeaseRequest {
            peer: "sat-teleop-ground".to_owned(),
            nonce,
            sig: Some(sig),
        };
        let sent = envelope::encode(&request)
            .map_err(|e| e.to_string())
            .and_then(|bytes| {
                socket
                    .send_to(&bytes, self.vehicle)
                    .map_err(|e| e.to_string())
            });
        sent.map_err(TakeError::Send)?;
        state.set_lease(Lease::Requested { nonce });
        let waiting = |s: &mut State| match &s.lease {
            Lease::Requested { nonce: n } => *n == nonce,
            Lease::Held(held) => held.challenge.is_none(),
            _ => false,
        };
        let (mut state, _) = self
            .answered
            .wait_timeout_while(state, LEASE_ANSWER, waiting)
            .map_err(|_| TakeError::NoAnswer)?;
        match &state.lease {
            Lease::Held(held) if held.challenge.is_some() => Ok(state.status()),
            Lease::Held(_) => Err(TakeError::NoChallenge),
            Lease::Denied(reason) => Err(TakeError::Denied(*reason)),
            _ => {
                state.set_lease(Lease::None);
                Err(TakeError::NoAnswer)
            }
        }
    }

    /// Asks the vehicle for its clock bound; its answer arrives on the control socket.
    pub fn ask_clock(&self) {
        let nonce = {
            let mut state = self.lock();
            state.clock_nonce = unix_ms();
            state.clock_sent = Some(Instant::now());
            state.clock_nonce
        };
        self.send(&ToVehicle::ClockQuery { nonce });
    }

    /// The vehicle's clock bound as it last reported it, and how long ago.
    pub fn vehicle_clock(&self) -> Option<Reading> {
        let (bound, at) = self.lock().vehicle_clock?;
        let age_ms = u64::try_from(at.elapsed().as_millis()).unwrap_or(u64::MAX);
        bound.map(|bound_ms| Reading { bound_ms, age_ms })
    }

    /// How long a control envelope takes to reach the vehicle: half the round trip of the newest
    /// clock query, while its reading is fresh. `None` before the first answer or once stale.
    pub fn uplink_delay_ms(&self) -> Option<u64> {
        let reading = self.vehicle_clock()?;
        clock::aged(reading)?;
        let rtt = self.lock().clock_rtt?;
        u64::try_from(rtt.as_millis() / 2).ok()
    }

    /// Gives the lease back; returns the status after.
    pub fn release(&self) -> Status {
        let mut state = self.lock();
        if let Lease::Held(held) = &state.lease {
            let sig = self.signer.sign_release(held.epoch, held.id).ok();
            let release = ToVehicle::LeaseRelease {
                lease: held.id,
                sig,
            };
            let sent = self.send(&release);
            state.counts.send_failed += u64::from(!sent);
        }
        state.set_lease(Lease::None);
        state.status()
    }

    pub fn status(&self) -> Status {
        let mut state = self.lock();
        state.expire(self.limits);
        state.status()
    }

    pub fn snapshot(&self) -> ControlSnapshot {
        let mut state = self.lock();
        state.expire(self.limits);
        ControlSnapshot {
            link: self.link.snapshot(),
            status: state.status(),
            counts: state.counts.clone(),
        }
    }

    /// Sends the pilot's box to the vehicle whenever it changes and every `refresh_every` while
    /// tracking, and a clear when tracking stops, until the process ends. Nothing is sent without
    /// the lease; the next tick tries again.
    pub(super) fn keep_region(&self, camera: &Arc<Camera>) {
        let _ = self.camera.set(Arc::clone(camera));
        let every_ms = u64::try_from(camera.refresh_every().as_millis()).unwrap_or(u64::MAX);
        let started = Instant::now();
        let mut sent: Option<NormRect> = None;
        let mut last_sent_ms: Option<u64> = None;
        loop {
            let wanted = camera.tracking();
            let now_ms = u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX);
            if region_needed(wanted, sent, last_sent_ms, now_ms, every_ms)
                && let Some(seq) = self.send_region(wanted)
            {
                camera.box_sent(seq, wanted);
                (sent, last_sent_ms) = (wanted, Some(now_ms));
            }
            thread::sleep(TRACKING_TICK);
        }
    }

    /// Sends `region` (none clears it) in a region envelope and returns its sequence number; none
    /// without the lease and a fresh challenge.
    fn send_region(&self, region: Option<NormRect>) -> Option<u64> {
        let mut state = self.lock();
        state.expire(self.limits);
        if !(state.fresh(self.limits) && self.wrap(&mut state, Kind::Region, None, region)) {
            return None;
        }
        let Lease::Held(held) = &state.lease else {
            return None;
        };
        Some(held.next_seq.region)
    }

    /// Binds the control socket and handles the vehicle's messages until the process ends.
    pub(super) fn serve(&self, stations: &[Arc<Station>]) {
        let socket = self
            .socket
            .get_or_init(|| bind_overlay(self.listen, &self.link));
        let mut buf = [0_u8; MAX_DATAGRAM];
        loop {
            match socket.recv_from(&mut buf) {
                Ok((len, from)) => {
                    self.vehicle_said(buf.get(..len).unwrap_or_default(), from, stations);
                }
                Err(e) => {
                    self.link
                        .down(&format!("cannot read the control socket: {e}"));
                    thread::sleep(RECV_ERROR_PAUSE);
                }
            }
        }
    }

    /// Counts a station's state-changing message that nothing may carry, and drops it.
    pub(super) fn refuse(&self, message: &MavMessage) {
        let mut state = self.lock();
        *state
            .counts
            .refused
            .entry(message.message_name())
            .or_default() += 1;
    }

    /// Answers a station's command no envelope may carry: `UNSUPPORTED` with the lease, `DENIED`
    /// without. Returns the `COMMAND_ACK` to send back.
    pub(super) fn unsupported(&self, frame: &Frame, message: &MavMessage) -> Option<Vec<u8>> {
        let mut state = self.lock();
        state.expire(self.limits);
        let result = if state.fresh(self.limits) {
            MavResult::MAV_RESULT_UNSUPPORTED
        } else {
            MavResult::MAV_RESULT_DENIED
        };
        state.answer(ack_to(frame, message), result)
    }

    /// Sends a station's message in an envelope of `class` while the adapter holds the lease with a
    /// fresh challenge; otherwise drops it, or denies it if it is a command. Returns a
    /// `COMMAND_ACK` to send back, if any.
    pub(super) fn envelope(
        &self,
        station: usize,
        frame: &Frame,
        message: &MavMessage,
        class: Class,
    ) -> Option<Vec<u8>> {
        let mut state = self.lock();
        state.expire(self.limits);
        let ack = ack_to(frame, message);
        if !state.fresh(self.limits) {
            if class == Class::Discrete {
                return state.answer(ack, MavResult::MAV_RESULT_DENIED);
            }
            state.counts.no_lease += 1;
            return None;
        }
        let window = ms(self.limits.ack_window_ms);
        let kind = match (class, ack, CommandKey::of(message)) {
            (Class::Liveness, ..) => Kind::Liveness,
            (Class::Continuous, ..) => Kind::Continuous,
            (Class::Discrete, Some(ack), command) => Kind::Discrete {
                txn: state.txn(station, command, ack, confirmation(message), window),
            },
            (Class::Discrete | Class::Region, ..) => return None,
        };
        if self.wrap(&mut state, kind, Some(frame), None) || class != Class::Discrete {
            return None;
        }
        state.answer(ack, MavResult::MAV_RESULT_DENIED)
    }

    fn vehicle_said(&self, datagram: &[u8], from: SocketAddr, stations: &[Arc<Station>]) {
        let mut state = self.lock();
        let message = envelope::decode::<ToGround>(datagram);
        let Some(message) = message.ok().filter(|_| from == self.vehicle) else {
            state.counts.ignored += 1;
            return;
        };
        match message {
            ToGround::LeaseGrant {
                nonce,
                lease,
                epoch,
            } => {
                if state.requested(nonce) {
                    let now = Instant::now();
                    state.set_lease(Lease::Held(Held {
                        id: lease,
                        epoch,
                        next_seq: NextSeq::default(),
                        challenge: None,
                        heard_at: now,
                        last_sent: now,
                    }));
                    self.answered.notify_all();
                }
            }
            ToGround::LeaseDenied { nonce, reason } => {
                if state.requested(nonce) {
                    state.set_lease(Lease::Denied(reason));
                    self.answered.notify_all();
                }
            }
            ToGround::Challenge { epoch, token, .. } => self.challenge(&mut state, epoch, token),
            ToGround::CommandResult { txn, outcome } => {
                let sent = state.sent.iter().find(|s| s.txn == txn).copied();
                let result = state.result(&outcome);
                let ack = sent.zip(result).and_then(|(sent, result)| {
                    let bytes = ack_bytes(sent.ack, result)?;
                    let station = stations.get(sent.station)?;
                    Some((station, bytes))
                });
                if let Some((station, bytes)) = ack {
                    let sent = station.socket.send_to(&bytes, station.address).is_ok();
                    state.counts.send_failed += u64::from(!sent);
                }
            }
            ToGround::Clock { nonce, bound_ms } => {
                if nonce == state.clock_nonce {
                    state.vehicle_clock = Some((bound_ms, Instant::now()));
                    state.clock_rtt = state.clock_sent.map(|sent| sent.elapsed());
                }
            }
            ToGround::Region { region } => {
                if let Some(camera) = self.camera.get() {
                    camera.applied(region, Instant::now());
                }
            }
            ToGround::Lost { region_seq } => {
                if let Some(camera) = self.camera.get() {
                    camera.lost(region_seq);
                }
            }
            ToGround::Rejected { reason, .. } => state.vehicle_rejected(reason),
        }
    }

    /// Keeps the newest challenge of the held lease, and keeps the lease alive when nothing else
    /// went out for a liveness interval.
    fn challenge(&self, state: &mut State, epoch: u64, token: u64) {
        let Lease::Held(held) = &mut state.lease else {
            return;
        };
        if held.epoch != epoch {
            return;
        }
        if held.challenge.replace(token).is_none() {
            self.answered.notify_all();
        }
        held.heard_at = Instant::now();
        let idle = held.last_sent.elapsed() >= ms(self.limits.liveness_interval_ms);
        if idle {
            self.wrap(state, Kind::Liveness, None, None);
        }
    }

    /// Sends `frame` (or, for liveness and region envelopes, nothing) and `region` in the next
    /// envelope of the held lease.
    fn wrap(
        &self,
        state: &mut State,
        kind: Kind,
        frame: Option<&Frame>,
        region: Option<NormRect>,
    ) -> bool {
        let Lease::Held(held) = &mut state.lease else {
            return false;
        };
        let Some(challenge) = held.challenge else {
            return false;
        };
        let signed = self.signer.sign(Envelope {
            epoch: held.epoch,
            lease: held.id,
            seq: held.next_seq.take(kind.into()),
            challenge,
            kind,
            mavlink: frame.map(encode_frame),
            region,
            sent_unix_ms: unix_ms(),
            sig: None,
        });
        held.last_sent = Instant::now();
        let envelope = signed.map(ToVehicle::Envelope);
        if !envelope.is_ok_and(|e| self.send(&e)) {
            state.counts.send_failed += 1;
            return false;
        }
        let w = &mut state.counts.wrapped;
        let counter = match (kind, frame) {
            (Kind::Liveness, None) => &mut w.keepalive,
            (Kind::Liveness, Some(_)) => &mut w.liveness,
            (Kind::Continuous, _) => &mut w.continuous,
            (Kind::Discrete { .. }, _) => &mut w.discrete,
            (Kind::Region, _) => &mut w.region,
        };
        *counter += 1;
        true
    }

    fn send(&self, message: &ToVehicle) -> bool {
        let Some(socket) = self.socket.get() else {
            return false;
        };
        envelope::encode(message).is_ok_and(|bytes| socket.send_to(&bytes, self.vehicle).is_ok())
    }

    fn lock(&self) -> MutexGuard<'_, State> {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

impl State {
    fn requested(&self, nonce: u64) -> bool {
        matches!(self.lease, Lease::Requested { nonce: n } if n == nonce)
    }

    /// Ends a held lease the vehicle has not been heard from (no challenge) for a lease lifetime:
    /// it has ended it too.
    fn expire(&mut self, limits: ControlLimits) {
        let Lease::Held(held) = &self.lease else {
            return;
        };
        if held.heard_at.elapsed() > ms(limits.lease_lifetime_ms) {
            let reason = format!("no challenge for {} ms", limits.lease_lifetime_ms);
            self.set_lease(Lease::Lost(reason));
        }
    }

    /// Held, with a challenge the vehicle still counts as fresh: one it issued no earlier than we
    /// received it.
    fn fresh(&self, limits: ControlLimits) -> bool {
        let Lease::Held(held) = &self.lease else {
            return false;
        };
        held.challenge.is_some() && held.heard_at.elapsed() <= ms(limits.challenge_lifetime_ms)
    }

    /// Answers a station's command itself with `result`, counted.
    fn answer(&mut self, ack: Option<AckTo>, result: MavResult) -> Option<Vec<u8>> {
        let counter = match result {
            MavResult::MAV_RESULT_UNSUPPORTED => &mut self.counts.unsupported,
            _ => &mut self.counts.denied,
        };
        *counter += 1;
        ack.and_then(|ack| ack_bytes(ack, result))
    }

    /// Every lease change forgets the commands sent: transactions belong to a lease.
    fn set_lease(&mut self, lease: Lease) {
        self.lease = lease;
        self.sent.clear();
    }

    /// The transaction of a station's command: a resend of one sent within the ACK window keeps
    /// its transaction, so the vehicle answers it without running it again; anything else gets a
    /// new one.
    fn txn(
        &mut self,
        station: usize,
        command: Option<CommandKey>,
        ack: AckTo,
        confirmation: u8,
        window: Duration,
    ) -> u64 {
        let now = Instant::now();
        self.sent
            .retain(|s| now.saturating_duration_since(s.at) <= window);
        let again = command
            .and_then(|command| resent(&self.sent, station, command, confirmation, now, window));
        if let Some(txn) = again {
            return txn;
        }
        self.next_txn += 1;
        if self.sent.len() == MAX_SENT {
            self.sent.pop_front();
        }
        self.sent.push_back(Sent {
            txn: self.next_txn,
            station,
            command,
            ack,
            at: now,
        });
        self.next_txn
    }

    /// Counts a command result; the `COMMAND_ACK` result for the station, if the adapter must
    /// answer (SITL answers applied commands itself).
    fn result(&mut self, outcome: &Outcome) -> Option<MavResult> {
        match outcome {
            Outcome::Applied => {
                self.counts.results.applied += 1;
                None
            }
            Outcome::Repeated => {
                self.counts.results.repeated += 1;
                Some(MavResult::MAV_RESULT_ACCEPTED)
            }
            Outcome::Unsupported => {
                self.counts.results.unsupported += 1;
                Some(MavResult::MAV_RESULT_UNSUPPORTED)
            }
            Outcome::Rejected { reason } => {
                self.counts.results.rejected += 1;
                self.vehicle_rejected(*reason);
                Some(MavResult::MAV_RESULT_DENIED)
            }
        }
    }

    fn vehicle_rejected(&mut self, reason: Refusal) {
        if reason.ends_lease() && matches!(self.lease, Lease::Held(_)) {
            self.set_lease(Lease::Lost(format!(
                "the vehicle refused an envelope: {reason}"
            )));
        }
        *self.counts.vehicle_rejected.entry(reason).or_default() += 1;
    }

    fn status(&self) -> Status {
        let none = Status {
            lease: LeaseStatus::None,
            id: None,
            epoch: None,
            reason: None,
            challenge_age_ms: None,
        };
        match &self.lease {
            Lease::None => none,
            Lease::Requested { .. } => Status {
                lease: LeaseStatus::Requested,
                ..none
            },
            Lease::Held(held) => Status {
                lease: LeaseStatus::Held,
                id: Some(held.id),
                epoch: Some(held.epoch),
                challenge_age_ms: held.challenge.map(|_| {
                    u64::try_from(held.heard_at.elapsed().as_millis()).unwrap_or(u64::MAX)
                }),
                ..none
            },
            Lease::Denied(reason) => Status {
                lease: LeaseStatus::Denied,
                reason: Some(reason.to_string()),
                ..none
            },
            Lease::Lost(reason) => Status {
                lease: LeaseStatus::Lost,
                reason: Some(reason.clone()),
                ..none
            },
        }
    }
}

fn ack_to(frame: &Frame, message: &MavMessage) -> Option<AckTo> {
    let station = (
        u8::try_from(frame.system_id()).unwrap_or(u8::MAX),
        frame.component_id(),
    );
    match message {
        MavMessage::COMMAND_LONG(c) => Some(AckTo::Command {
            command: c.command,
            vehicle: (c.target_system, c.target_component),
        }),
        MavMessage::COMMAND_INT(c) => Some(AckTo::Command {
            command: c.command,
            vehicle: (c.target_system, c.target_component),
        }),
        MavMessage::MISSION_COUNT(m) => Some(mission_ack_to(
            m.target_system,
            m.target_component,
            m.mission_type,
            station,
        )),
        MavMessage::MISSION_ITEM_INT(m) => Some(mission_ack_to(
            m.target_system,
            m.target_component,
            m.mission_type,
            station,
        )),
        MavMessage::MISSION_CLEAR_ALL(m) => Some(mission_ack_to(
            m.target_system,
            m.target_component,
            m.mission_type,
            station,
        )),
        _ => None,
    }
}

fn mission_ack_to(
    system: u8,
    component: u8,
    mission_type: MavMissionType,
    station: (u8, u8),
) -> AckTo {
    AckTo::Mission {
        mission_type,
        vehicle: (system, component),
        station,
    }
}

fn confirmation(message: &MavMessage) -> u8 {
    match message {
        MavMessage::COMMAND_LONG(c) => c.confirmation,
        _ => 0,
    }
}

/// The answer to a station's message from its target: a `COMMAND_ACK`, or for a mission upload
/// message a `MISSION_ACK` (`DENIED` or `UNSUPPORTED`; an applied one is answered by the autopilot
/// through the telemetry). `None` when there is nothing to say.
fn ack_bytes(ack: AckTo, result: MavResult) -> Option<Vec<u8>> {
    let (vehicle, message) = match ack {
        AckTo::Command { command, vehicle } => (
            vehicle,
            MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
                command,
                result,
                ..COMMAND_ACK_DATA::default()
            }),
        ),
        AckTo::Mission {
            mission_type,
            vehicle,
            station,
        } => {
            let mavtype = match result {
                MavResult::MAV_RESULT_DENIED => MavMissionResult::MAV_MISSION_DENIED,
                MavResult::MAV_RESULT_UNSUPPORTED => MavMissionResult::MAV_MISSION_UNSUPPORTED,
                _ => return None,
            };
            let message = MavMessage::MISSION_ACK(MISSION_ACK_DATA {
                target_system: station.0,
                target_component: station.1,
                mavtype,
                mission_type,
                ..MISSION_ACK_DATA::default()
            });
            (vehicle, message)
        }
    };
    let header = MavHeader {
        system_id: u32::from(vehicle.0),
        component_id: vehicle.1,
        sequence: 0,
    };
    let mut bytes = Vec::new();
    mavlink::write_v2_msg(&mut bytes, header, &message).ok()?;
    Some(bytes)
}

fn ms(ms: u64) -> Duration {
    Duration::from_millis(ms)
}

/// The transaction of an earlier command that `command` from `station` resends: `confirmation`
/// above 0, the same command, sent no longer than `window` before `now`.
fn resent(
    sent: &VecDeque<Sent>,
    station: usize,
    command: CommandKey,
    confirmation: u8,
    now: Instant,
    window: Duration,
) -> Option<u64> {
    if confirmation == 0 {
        return None;
    }
    sent.iter()
        .rev()
        .find(|s| {
            s.station == station
                && s.command == Some(command)
                && now.saturating_duration_since(s.at) <= window
        })
        .map(|s| s.txn)
}

/// Whether the box must go to the vehicle now: it changed (or tracking stopped), or it is the one
/// being kept and `every_ms` passed since the last send.
fn region_needed(
    wanted: Option<NormRect>,
    sent: Option<NormRect>,
    last_sent_ms: Option<u64>,
    now_ms: u64,
    every_ms: u64,
) -> bool {
    wanted != sent || (wanted.is_some() && region_due(last_sent_ms, now_ms, every_ms))
}

/// Whether a refresh is due: at once when none was sent, then every `every_ms`.
fn region_due(last_sent_ms: Option<u64>, now_ms: u64, every_ms: u64) -> bool {
    last_sent_ms.is_none_or(|last| now_ms.saturating_sub(last) >= every_ms)
}

#[cfg(test)]
mod tests {
    use mavlink::dialects::ardupilotmega::{COMMAND_LONG_DATA, MISSION_COUNT_DATA};

    use super::*;

    fn arm(param1: f32, confirmation: u8) -> MavMessage {
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            param1,
            target_system: 1,
            target_component: 1,
            confirmation,
            ..COMMAND_LONG_DATA::default()
        })
    }

    #[test]
    fn the_box_goes_when_it_changes_stops_or_is_due() {
        let a = NormRect::new(0.2, 0.1, 0.6, 0.7);
        let b = NormRect::new(0.1, 0.1, 0.3, 0.3);
        // (wanted, sent, last sent, now, needed)
        let cases = [
            (a, None, None, 0, true),
            (a, a, Some(0), 499, false),
            (a, a, Some(0), 500, true),
            (b, a, Some(0), 10, true),
            (None, a, Some(0), 10, true),
            (None, None, Some(0), 10_000, false),
        ];
        for (wanted, sent, last, now, needed) in cases {
            assert_eq!(
                region_needed(wanted, sent, last, now, 500),
                needed,
                "{wanted:?} after {sent:?} at {now}"
            );
        }
    }

    #[test]
    fn only_a_resend_of_the_same_command_within_the_window_keeps_its_transaction() {
        let t0 = Instant::now();
        let window = Duration::from_millis(4000);
        let key = |message: &MavMessage| CommandKey::of(message).unwrap();
        let sent = |txn, station, message: &MavMessage, after_ms| Sent {
            txn,
            station,
            command: Some(key(message)),
            ack: AckTo::Command {
                command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                vehicle: (1, 1),
            },
            at: t0 + Duration::from_millis(after_ms),
        };
        // An arm (txn 1) from station 0, then a disarm (txn 2) from station 0.
        let earlier = VecDeque::from([sent(1, 0, &arm(1.0, 0), 0), sent(2, 0, &arm(0.0, 0), 100)]);
        // (station, command, ms after t0, expected transaction)
        let cases = [
            (0, arm(1.0, 1), 1200, Some(1)),
            (0, arm(0.0, 2), 2500, Some(2)),
            (0, arm(1.0, 0), 1200, None),
            (1, arm(1.0, 1), 1200, None),
            (0, arm(1.0, 1), 4001, None),
            (0, arm(0.5, 1), 1200, None),
        ];
        for (station, message, ms, expected) in cases {
            let confirmation = confirmation(&message);
            let now = t0 + Duration::from_millis(ms);
            let txn = resent(&earlier, station, key(&message), confirmation, now, window);
            assert_eq!(txn, expected, "station {station}, {message:?} at {ms} ms");
        }
    }

    #[test]
    fn a_refused_mission_message_is_answered_with_a_mission_ack_from_the_vehicle() {
        let count = MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
            count: 4,
            target_system: 1,
            target_component: 1,
            ..MISSION_COUNT_DATA::default()
        });
        let mut bytes = Vec::new();
        let header = MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        };
        mavlink::write_v2_msg(&mut bytes, header, &count).unwrap();
        let frame = crate::frame::datagram_frames(&bytes).next().unwrap();
        let ack = ack_to(&frame, &count).unwrap();
        let cases = [
            (
                MavResult::MAV_RESULT_DENIED,
                Some(MavMissionResult::MAV_MISSION_DENIED),
            ),
            (
                MavResult::MAV_RESULT_UNSUPPORTED,
                Some(MavMissionResult::MAV_MISSION_UNSUPPORTED),
            ),
            (MavResult::MAV_RESULT_ACCEPTED, None),
        ];
        for (result, expected) in cases {
            let answer = ack_bytes(ack, result).map(|bytes| {
                let ack = crate::frame::datagram_frames(&bytes).next().unwrap();
                (ack.system_id(), ack.component_id(), ack.decode().unwrap())
            });
            let Some(expected) = expected else {
                assert!(answer.is_none());
                continue;
            };
            let Some((1, 1, MavMessage::MISSION_ACK(got))) = answer else {
                panic!("{result:?}: {answer:?}");
            };
            assert_eq!(
                (got.mavtype, got.target_system, got.target_component),
                (expected, 255, 190)
            );
        }
    }

    #[test]
    fn identical_mission_messages_get_separate_transactions() {
        let mut state = State::default();
        let ack = AckTo::Mission {
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            vehicle: (1, 1),
            station: (255, 190),
        };
        let window = Duration::from_millis(4000);
        let txns: Vec<u64> = (0..3).map(|_| state.txn(0, None, ack, 1, window)).collect();
        assert_eq!(txns, [1, 2, 3]);
    }
}
