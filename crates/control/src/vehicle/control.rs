//! The vehicle's control endpoint on its overlay address: lease requests and releases, a challenge
//! for the holder every 500 ms, and the holder's envelopes, checked with `sat_teleop_policy::control` on
//! this process's monotonic clock. An admitted envelope's frame goes to SITL at once; the newest
//! fresh continuous setpoint is written again whenever 200 ms pass without one, until it expires.
//! Nothing waits: a stale, duplicate or reordered envelope is refused, never queued or replayed.

use std::collections::{BTreeMap, HashMap};
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::MavMessage;
use serde::Serialize;

use sat_teleop_policy::control::{
    Admitted, ChallengeToken, Epoch, Header, LeaseId, LeaseState, PeerId, Seq, Txn, TxnId,
};
use sat_teleop_policy::region::{NormRect, RegionCell, Timed};

use super::{ControllerConfig, SitlWriter, VehicleConfig, write_to_sitl};
use crate::auth::{VehicleId, Verifier};
use crate::clock::{Clock, ClockSnapshot};
use crate::envelope::{
    self, CHALLENGE_EVERY, ControlLimits, Envelope, Kind, REFRESH_EVERY, Refusal, TICK, ToGround,
    ToVehicle,
};
use crate::frame::{Frame, MAX_DATAGRAM};
use crate::gate::{self, CommandKey, Refused, Vehicle};
use crate::identity::{self, Autopilot, ControlTarget, Facts, Failure, Policy};
use crate::params::{self, ParamType, Reading, Required};
use crate::stats::{Flow, FlowCounts, Link, LinkState};
use crate::{bind_overlay, timed_out, unix_ms};

const RECV_ERROR_PAUSE: Duration = Duration::from_millis(100);

#[derive(Debug, Default)]
pub struct ControlStats {
    /// The control socket.
    pub link: Link,
    /// Frames from envelopes and refreshes written to SITL; receive errors of the control socket.
    pub sitl: Flow,
    /// The pilot's box, set by `Kind::Region` envelopes; the sender reads it. Never forwarded.
    pub region: RegionCell,
    counts: Mutex<ControlCounts>,
    keys: Mutex<Keys>,
    real: Mutex<Real>,
}

/// The system and component a `MAVLink` message came from.
type Address = (u8, u8);

/// What real mode needs: the policy, the parameters required and what the autopilot last said of
/// each, by the system and component that said it.
#[derive(Debug, Default)]
struct Real {
    policy: Policy,
    /// `allow_missions` of the config.
    allow_missions: bool,
    required: Vec<Required>,
    /// `required` as the config set it, which the test hooks change and restore.
    #[cfg(feature = "testbed")]
    configured: Vec<Required>,
    readings: BTreeMap<(Address, String), Said>,
    clock: Option<Arc<Clock>>,
}

/// A `PARAM_VALUE` kept: its value, the type it came in and when.
#[derive(Debug, Clone, Copy)]
struct Said {
    value: f32,
    kind: Option<ParamType>,
    at: Instant,
}

impl Real {
    /// The readings that came from `address`, the autopilot's own, and from no one else.
    fn readings(&self, address: Option<Address>) -> BTreeMap<String, Reading> {
        let Some(address) = address else {
            return BTreeMap::new();
        };
        self.readings
            .iter()
            .filter(|((from, _), _)| *from == address)
            .map(|((_, name), said)| {
                let age = said.at.elapsed();
                let (value, kind) = (said.value, said.kind);
                (name.clone(), Reading { value, kind, age })
            })
            .collect()
    }
}

/// The controllers' public keys. A controller whose key cannot be read has none: its envelopes are
/// refused `bad_signature`, and the reason is reported.
#[derive(Debug, Default)]
struct Keys {
    addresses: Vec<IpAddr>,
    verifiers: BTreeMap<IpAddr, Verifier>,
    errors: Vec<String>,
}

#[derive(Debug, Default, Clone, Serialize)]
pub struct ControlCounts {
    /// This session, the vehicle's start time in UNIX milliseconds.
    pub epoch: u64,
    /// The autopilot's heartbeat address, the vehicle type it reports (none while unknown, when
    /// every mode change is refused) and whether that autopilot reported the SITL marker; all
    /// forgotten when SITL disconnects.
    #[serde(flatten)]
    pub autopilot: Autopilot,
    /// Frames of liveness and continuous envelopes and refreshes that went to the logging sink
    /// instead of the autopilot (counted in `applied` as well), and discrete commands refused for it.
    pub sunk: u64,
    /// The unexpired lease, if any.
    pub lease: Option<LeaseView>,
    /// Controllers whose public key could not be loaded.
    pub key_errors: Vec<String>,
    /// Envelopes whose signature was checked, and the time that took in all and at most, in
    /// microseconds.
    pub verified: u64,
    pub verify_us_total: u64,
    pub verify_us_max: u64,
    pub grants: u64,
    pub challenges: u64,
    /// Admitted envelopes whose frame reached SITL, by kind; `keepalive` is liveness without one.
    pub applied: Applied,
    /// Discrete retries answered from the transaction cache, not applied again.
    pub repeated: u64,
    /// Discrete envelopes with a command no envelope may carry, answered `Unsupported` before any
    /// freshness check.
    pub unsupported: u64,
    /// Fresh setpoints written again to SITL.
    pub refreshed: u64,
    /// Answers sent to a controller (grants, denials, challenges, results, rejections), and their
    /// payload bytes.
    pub replies: u64,
    pub reply_bytes: u64,
    /// Answers to a controller that could not be sent.
    pub reply_failed: u64,
    /// Refused requests, releases and envelopes, by reason.
    pub rejected: BTreeMap<Refusal, u64>,
    pub last_continuous: Option<Timing>,
    pub last_discrete: Option<Timing>,
    pub last_refresh_unix_ms: Option<u64>,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct LeaseView {
    pub id: u64,
    pub holder: SocketAddr,
}

#[derive(Debug, Default, Clone, Copy, Serialize)]
pub struct Applied {
    pub keepalive: u64,
    pub liveness: u64,
    pub continuous: u64,
    pub discrete: u64,
    /// Region envelopes that set or cleared the pilot's box: nothing of them reaches SITL.
    pub region: u64,
}

/// One applied envelope, in UNIX milliseconds: when the controller sent it (its clock), when it
/// arrived and when its frame was written to SITL (this vehicle's clock). The testbed's containers
/// share one clock.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Timing {
    pub sent: u64,
    pub received: u64,
    pub applied: u64,
}

#[derive(Debug, Serialize)]
pub struct ControlSnapshot {
    pub link: LinkState,
    pub sitl: FlowCounts,
    /// Where control frames go now.
    pub target: ControlTarget,
    pub mode: identity::Mode,
    /// Whether mission upload and Auto are admitted (`allow_missions`).
    pub allow_missions: bool,
    /// The interlocks that do not hold, and why: the reasons control goes to the logging sink.
    pub interlocks: Vec<Failure>,
    /// The vehicle's clock bound.
    pub clock: Option<ClockSnapshot>,
    #[serde(flatten)]
    pub counts: ControlCounts,
}

impl ControlStats {
    pub fn snapshot(&self) -> ControlSnapshot {
        ControlSnapshot {
            link: self.link.snapshot(),
            sitl: self.sitl.snapshot(),
            target: self.target(),
            mode: self.policy().mode,
            allow_missions: self.allow_missions(),
            interlocks: self.failures(),
            clock: self.clock().map(|clock| clock.snapshot()),
            counts: self.counts.lock().map(|c| c.clone()).unwrap_or_default(),
        }
    }

    /// Where control frames go: the logging sink until every interlock holds.
    pub fn target(&self) -> ControlTarget {
        identity::control_target(self.policy(), &self.facts())
    }

    /// The interlocks that do not hold now.
    pub fn failures(&self) -> Vec<Failure> {
        identity::failures(self.policy(), &self.facts())
    }

    /// Whether the config switched mission upload and Auto on.
    pub fn allow_missions(&self) -> bool {
        self.real.lock().is_ok_and(|r| r.allow_missions)
    }

    fn policy(&self) -> Policy {
        self.real.lock().map(|r| r.policy).unwrap_or_default()
    }

    fn facts(&self) -> Facts {
        let autopilot = self.autopilot();
        Facts {
            sitl_marker: autopilot.sitl_marker,
            probes: autopilot.probes,
            vehicle: autopilot.vehicle,
            keys: self.key_failure(),
            params: self.param_failure(autopilot.address),
            clock: self.clock_failure(),
            clock_started: self.clock().is_some_and(|clock| clock.started()),
        }
    }

    fn key_failure(&self) -> Option<String> {
        let errors = self
            .keys
            .lock()
            .map(|k| k.errors.clone())
            .unwrap_or_default();
        (!errors.is_empty()).then(|| format!("no key for {}", errors.join("; ")))
    }

    /// Why the failsafe parameters are not what real mode requires; only the autopilot at
    /// `address` counts, and nothing is required in SITL mode.
    fn param_failure(&self, address: Option<(u8, u8)>) -> Option<String> {
        let real = self.real.lock().ok()?;
        let readings = real.readings(address);
        params::failure(&real.required, &readings)
    }

    /// Sets the mode, the required parameters, the clock and the controllers' keys from `config`.
    pub(super) fn configure(&self, config: &VehicleConfig) {
        if let Ok(mut real) = self.real.lock() {
            real.policy = Policy {
                mode: config.mode,
                allow_sitl: config.allows_sitl(),
            };
            real.allow_missions = config.allow_missions;
            real.required.clone_from(&config.require);
            #[cfg(feature = "testbed")]
            real.configured.clone_from(&config.require);
            real.clock = Some(Arc::new(Clock::new(config.clock.clone())));
        }
        self.set_keys(&config.controllers, &config.vehicle_id);
    }

    /// Test only: whether real mode accepts SITL.
    #[cfg(feature = "testbed")]
    pub fn set_allow_sitl(&self, allow: bool) {
        if let Ok(mut real) = self.real.lock() {
            real.policy.allow_sitl = allow;
        }
    }

    /// Test only: requires `name` to be `value`, in place of what the config says of it.
    #[cfg(feature = "testbed")]
    pub fn require(&self, name: &str, value: f32) {
        let Ok(mut real) = self.real.lock() else {
            return;
        };
        let kind = real
            .required
            .iter()
            .find(|r| r.name == name)
            .map_or(ParamType::Real32, |r| r.kind);
        real.required.retain(|r| r.name != name);
        real.required.push(Required {
            name: name.to_owned(),
            kind,
            value,
            compare: params::Compare::Equal,
        });
    }

    /// Test only: requires what the config says.
    #[cfg(feature = "testbed")]
    pub fn reset_required(&self) {
        if let Ok(mut real) = self.real.lock() {
            let Real {
                required,
                configured,
                ..
            } = &mut *real;
            required.clone_from(configured);
        }
    }

    /// The clock the bound comes from, to poll and to inject into.
    pub fn clock(&self) -> Option<Arc<Clock>> {
        self.real.lock().ok().and_then(|real| real.clock.clone())
    }

    /// The bound the vehicle's clock has now, `None` when it has none to trust.
    fn clock_bound(&self) -> Option<u64> {
        self.clock().and_then(|clock| clock.bound_ms())
    }

    fn clock_failure(&self) -> Option<String> {
        let source = self.clock()?.source().clone();
        self.clock_bound()
            .is_none()
            .then(|| format!("no fresh bound on the clock ({source:?}): chrony is not synchronised, or not running"))
    }

    /// The parameter to ask the autopilot for now, if any is due.
    pub(super) fn param_to_read(&self) -> Option<String> {
        let address = self.autopilot().address;
        let real = self.real.lock().ok()?;
        if real.policy.mode != identity::Mode::Real {
            return None;
        }
        let readings = real.readings(address);
        params::next_to_read(&real.required, &readings).map(str::to_owned)
    }

    pub(super) fn probed(&self) {
        self.update(|c| c.autopilot = c.autopilot.probed());
    }

    /// Loads the controllers' public keys.
    pub(super) fn set_keys(&self, controllers: &[ControllerConfig], vehicle: &VehicleId) {
        let mut keys = Keys::default();
        for controller in controllers {
            keys.addresses.push(controller.address);
            match Verifier::load(&controller.public_key, vehicle) {
                Ok(verifier) => {
                    keys.verifiers.insert(controller.address, verifier);
                }
                Err(e) => {
                    eprintln!("controller {}: {e}", controller.address);
                    keys.errors.push(format!("{}: {e}", controller.address));
                }
            }
        }
        self.update(|c| c.key_errors.clone_from(&keys.errors));
        if let Ok(mut current) = self.keys.lock() {
            *current = keys;
        }
    }

    /// Runs `check` (a verification of a signed message) with the key of the controller it came
    /// from, and times it.
    fn authenticate(
        &self,
        from: IpAddr,
        check: impl FnOnce(&Verifier) -> Result<(), Refusal>,
    ) -> Result<(), Refusal> {
        let verifier = self
            .keys
            .lock()
            .ok()
            .and_then(|keys| keys.verifiers.get(&from).cloned());
        let started = Instant::now();
        let checked = verifier
            .ok_or(Refusal::BadSignature)
            .and_then(|verifier| check(&verifier));
        let us = u64::try_from(started.elapsed().as_micros()).unwrap_or(u64::MAX);
        self.update(|c| {
            c.verified += 1;
            c.verify_us_total = c.verify_us_total.saturating_add(us);
            c.verify_us_max = c.verify_us_max.max(us);
        });
        checked
    }

    fn update(&self, f: impl FnOnce(&mut ControlCounts)) {
        if let Ok(mut counts) = self.counts.lock() {
            f(&mut counts);
        }
    }

    pub(super) fn heartbeat(&self, address: (u8, u8), vehicle: Option<Vehicle>) {
        self.update(|c| c.autopilot = c.autopilot.heartbeat(address, vehicle));
    }

    pub(super) fn param_value(&self, address: (u8, u8), message: &MavMessage) {
        self.update(|c| c.autopilot = c.autopilot.param_value(address, message));
        let Some(param) = params::value_of(message) else {
            return;
        };
        let from_autopilot = self.autopilot().address == Some(address);
        if let Ok(mut real) = self.real.lock()
            && from_autopilot
            && real.required.iter().any(|r| r.name == param.name)
        {
            let said = Said {
                value: param.value,
                kind: param.kind,
                at: Instant::now(),
            };
            real.readings.insert((address, param.name), said);
        }
    }

    /// SITL disconnected: nothing is known of the autopilot until it speaks again.
    pub(super) fn forget_autopilot(&self) {
        self.update(|c| c.autopilot = Autopilot::default());
        if let Ok(mut real) = self.real.lock() {
            real.readings.clear();
        }
    }

    pub(super) fn sitl_marker(&self) -> bool {
        self.autopilot().sitl_marker
    }

    fn autopilot(&self) -> Autopilot {
        self.counts.lock().map(|c| c.autopilot).unwrap_or_default()
    }

    /// The addresses of the controllers that were configured, key or not.
    fn controllers(&self) -> Vec<IpAddr> {
        self.keys
            .lock()
            .map(|keys| keys.addresses.clone())
            .unwrap_or_default()
    }

    /// Takes the place of a write to the autopilot; logged with the reasons once, then every 600
    /// frames.
    fn sink(&self, frame: &Frame) {
        let mut sunk = 0;
        self.update(|c| {
            c.sunk += 1;
            sunk = c.sunk;
        });
        if sunk % 600 == 1 {
            let id = frame.message_id();
            let why: Vec<String> = self
                .failures()
                .iter()
                .map(|f| format!("{:?}: {}", f.interlock, f.reason))
                .collect();
            eprintln!(
                "control frame {id} went to the logging sink ({sunk} so far): {}",
                why.join("; ")
            );
        }
    }

    fn vehicle(&self) -> Option<Vehicle> {
        self.autopilot().vehicle
    }

    fn rejected(&self, refusal: Refusal) {
        self.update(|c| *c.rejected.entry(refusal).or_default() += 1);
    }
}

/// Serves the control endpoint until the process ends.
pub(super) fn serve(
    listen: SocketAddr,
    limits: ControlLimits,
    writer: &SitlWriter,
    stats: &ControlStats,
) {
    let socket = bind_overlay(listen, &stats.link);
    if let Err(e) = socket.set_read_timeout(Some(TICK)) {
        stats
            .link
            .down(&format!("cannot set a read timeout on {listen}: {e}"));
        return;
    }
    let epoch = Epoch(unix_ms());
    stats.update(|c| c.epoch = epoch.0);
    let mut endpoint = Endpoint {
        socket,
        writer,
        stats,
        started: Instant::now(),
        challenge_lifetime_ms: limits.challenge_lifetime_ms,
        state: LeaseState::new(
            epoch,
            limits.policy(),
            stats.controllers().into_iter().map(PeerId).collect(),
        ),
        holder: None,
        told: Told::default(),
        next_challenge_ms: 0,
        last_setpoint_ms: 0,
        last_nonce: HashMap::new(),
    };
    let mut buf = [0_u8; MAX_DATAGRAM];
    loop {
        match endpoint.socket.recv_from(&mut buf) {
            Ok((len, from)) => endpoint.receive(buf.get(..len).unwrap_or_default(), from),
            Err(e) if timed_out(&e) => {}
            Err(e) => {
                stats.sitl.receive_failed("the control socket", &e);
                thread::sleep(RECV_ERROR_PAUSE);
            }
        }
        endpoint.tick();
    }
}

struct Endpoint<'a> {
    socket: UdpSocket,
    writer: &'a SitlWriter,
    stats: &'a ControlStats,
    started: Instant,
    challenge_lifetime_ms: u64,
    state: LeaseState<Frame>,
    /// Where the holder asked from; its challenges go there.
    holder: Option<SocketAddr>,
    /// What the holder was last told of the followed box.
    told: Told,
    next_challenge_ms: u64,
    /// When a setpoint was last written to SITL, first or again.
    last_setpoint_ms: u64,
    /// The highest lease-request nonce accepted from each controller.
    last_nonce: HashMap<IpAddr, u64>,
}

/// Where the followed box was last told to the holder, and when, in this endpoint's milliseconds.
#[derive(Default)]
struct Told {
    lost: u64,
    target: Option<NormRect>,
    at_ms: u64,
    /// The sequence number of the last region envelope applied.
    region_seq: u64,
    /// The sequence number named by the lost message, and when the follow ended.
    lost_seq: u64,
    lost_at_ms: Option<u64>,
}

/// The followed box is told to the holder when it moves, no more often than this.
const FOLLOW_REPORT_EVERY_MS: u64 = 250;
/// The lost message is repeated on that tick for this long.
const LOST_REPEAT_FOR_MS: u64 = 3000;

impl Endpoint<'_> {
    /// Writes `frame` to SITL once it is identified, otherwise to the logging sink; false when SITL
    /// is the target and the write failed.
    fn deliver(&self, frame: &Frame) -> bool {
        match self.stats.target() {
            ControlTarget::Autopilot => write_to_sitl(frame, self.writer, &self.stats.sitl),
            ControlTarget::LoggingSink => {
                self.stats.sink(frame);
                true
            }
        }
    }

    /// Writes a discrete command to SITL; false when SITL is not the target (the command goes to the
    /// logging sink and is refused, never reported as applied) or the write failed.
    fn deliver_discrete(&self, frame: &Frame) -> bool {
        match self.stats.target() {
            ControlTarget::Autopilot => write_to_sitl(frame, self.writer, &self.stats.sitl),
            ControlTarget::LoggingSink => {
                self.stats.sink(frame);
                false
            }
        }
    }

    fn now_ms(&self) -> u64 {
        u64::try_from(self.started.elapsed().as_millis()).unwrap_or(u64::MAX)
    }

    fn receive(&mut self, datagram: &[u8], from: SocketAddr) {
        if let Err(reason) = self.state.controller(PeerId(from.ip())) {
            return self.refuse(from, None, None, reason.into());
        }
        let now = self.now_ms();
        match envelope::decode::<ToVehicle>(datagram) {
            Ok(ToVehicle::LeaseRequest { nonce, sig, .. }) => {
                self.request(from, nonce, sig.as_deref(), now);
            }
            Ok(ToVehicle::LeaseRelease { lease, sig }) => {
                self.release(from, lease, sig.as_deref(), now);
            }
            Ok(ToVehicle::ClockQuery { nonce }) => self.clock(from, nonce),
            Ok(ToVehicle::Envelope(envelope)) => self.signed_envelope(&envelope, from, now),
            Err(refusal) => self.refuse(from, None, None, refusal),
        }
    }

    fn clock(&self, to: SocketAddr, nonce: u64) {
        let bound_ms = self.stats.clock_bound();
        self.reply(to, &ToGround::Clock { nonce, bound_ms });
    }

    /// Only a request signed by its sender's key reaches the lease.
    fn request(&mut self, from: SocketAddr, nonce: u64, sig: Option<&str>, now: u64) {
        let signed = self
            .stats
            .authenticate(from.ip(), |v| v.verify_request(nonce, sig));
        if let Err(reason) = signed {
            return self.deny(from, nonce, reason);
        }
        let last = self.last_nonce.entry(from.ip()).or_default();
        if nonce <= *last {
            return self.deny(from, nonce, Refusal::ReplayedRequest);
        }
        *last = nonce;
        match self.state.request(PeerId(from.ip()), now) {
            Ok(grant) => {
                self.clear_region();
                self.holder = Some(from);
                self.next_challenge_ms = now;
                self.stats.update(|c| c.grants += 1);
                let grant = ToGround::LeaseGrant {
                    nonce,
                    lease: grant.lease.0,
                    epoch: grant.epoch.0,
                };
                self.reply(from, &grant);
            }
            Err(reason) => self.deny(from, nonce, reason.into()),
        }
    }

    fn deny(&self, to: SocketAddr, nonce: u64, reason: Refusal) {
        self.stats.rejected(reason);
        self.reply(to, &ToGround::LeaseDenied { nonce, reason });
    }

    /// Only a release signed by its sender's key, for this session, reaches the lease.
    fn release(&mut self, from: SocketAddr, lease: u64, sig: Option<&str>, now: u64) {
        let epoch = self.state.epoch().0;
        let signed = self
            .stats
            .authenticate(from.ip(), |v| v.verify_release(epoch, lease, sig));
        if let Err(reason) = signed {
            return self.refuse(from, None, None, reason);
        }
        match self.state.release(PeerId(from.ip()), LeaseId(lease), now) {
            Ok(()) => {
                self.clear_region();
                self.holder = None;
            }
            Err(reason) => self.refuse(from, None, None, reason.into()),
        }
    }

    /// Only an envelope signed by its sender's key goes on to the other checks.
    fn signed_envelope(&mut self, envelope: &Envelope, from: SocketAddr, now: u64) {
        match self.stats.authenticate(from.ip(), |v| v.verify(envelope)) {
            Ok(()) => self.envelope(envelope, from, now),
            Err(refusal) => self.refuse(from, Some(envelope.seq), txn(envelope.kind), refusal),
        }
    }

    /// Checks the frame and whether the envelope's kind may carry it, then freshness and order;
    /// only then does anything reach SITL. A refused envelope changes nothing.
    fn envelope(&mut self, envelope: &Envelope, from: SocketAddr, now: u64) {
        let received_unix_ms = unix_ms();
        let refuse = |this: &Self, refusal| {
            this.refuse(from, Some(envelope.seq), txn(envelope.kind), refusal);
        };
        let Ok(carried) = carried(envelope) else {
            return refuse(self, Refusal::Malformed);
        };
        let Ok(region) = region_of(envelope) else {
            return refuse(self, Refusal::Malformed);
        };
        let allowed = match &carried {
            Some((_, message)) => gate::admits(envelope.kind, message),
            None => matches!(envelope.kind, Kind::Liveness | Kind::Region),
        };
        if !allowed {
            return self.not_allowed(envelope, from);
        }
        let header = Header {
            epoch: Epoch(envelope.epoch),
            lease: LeaseId(envelope.lease),
            kind: envelope.kind.into(),
            seq: Seq(envelope.seq),
            challenge: ChallengeToken(envelope.challenge),
            txn: discrete_txn(envelope.kind, carried.as_ref()),
        };
        let admitted = match self.state.admit(PeerId(from.ip()), &header, now) {
            Ok(admitted) => admitted,
            Err(reason) => return refuse(self, reason.into()),
        };
        let timing = |applied| Timing {
            sent: envelope.sent_unix_ms,
            received: received_unix_ms,
            applied,
        };
        if envelope.kind == Kind::Region {
            self.told.region_seq = envelope.seq;
            return self.set_region(region, from);
        }
        let Some(carried) = carried else {
            return self.stats.update(|c| c.applied.keepalive += 1);
        };
        let applied = match envelope.kind {
            Kind::Discrete { txn } => {
                self.discrete(&admitted, TxnId(txn), &carried, from, now, timing);
                return;
            }
            kind => self.apply(&admitted, kind, carried.0, now, timing),
        };
        if let Err(refusal) = applied {
            refuse(self, refusal);
        }
    }

    /// Makes `region` the pilot's box, or clears it for `None`, and tells the holder which box the
    /// sender keeps sharp now: none when it was dropped for its size or the detections', or when
    /// regional video is off, so the ground never reports tracking of a box that is not used.
    fn set_region(&mut self, region: Option<NormRect>, to: SocketAddr) {
        match region {
            Some(rect) => self.stats.region.set(Timed {
                rect,
                at_ms: self.stats.region.now_ms(),
            }),
            None => self.stats.region.clear(),
        }
        self.stats.update(|c| c.applied.region += 1);
        let region = self.stats.region.confirmed();
        self.reply(to, &ToGround::Region { region });
    }

    /// Drops the pilot's box when the lease it came with is gone, and tells the holder.
    fn clear_region(&mut self) {
        if self.stats.region.latest().is_none() {
            return;
        }
        self.stats.region.clear();
        if let Some(holder) = self.holder {
            self.reply(holder, &ToGround::Region { region: None });
        }
    }

    /// An envelope whose kind may not carry its frame: a discrete one is answered `Unsupported`,
    /// anything else refused.
    fn not_allowed(&self, envelope: &Envelope, from: SocketAddr) {
        let Kind::Discrete { txn } = envelope.kind else {
            return self.refuse(from, Some(envelope.seq), None, Refusal::NotAllowlisted);
        };
        self.stats.update(|c| c.unsupported += 1);
        self.result(from, TxnId(txn), envelope::Outcome::Unsupported);
    }

    /// Writes an admitted liveness or continuous frame; a setpoint also becomes the one to refresh.
    fn apply(
        &mut self,
        admitted: &Admitted,
        kind: Kind,
        frame: Frame,
        now: u64,
        timing: impl Fn(u64) -> Timing,
    ) -> Result<(), Refusal> {
        self.stats.sitl.received(&frame);
        if !self.deliver(&frame) {
            return Err(Refusal::NoDestination);
        }
        if kind == Kind::Liveness {
            self.stats.update(|c| c.applied.liveness += 1);
            return Ok(());
        }
        let timing = timing(unix_ms());
        self.state.offer_setpoint(admitted, frame, now);
        self.last_setpoint_ms = now;
        self.stats.update(|c| {
            c.applied.continuous += 1;
            c.last_continuous = Some(timing);
        });
        Ok(())
    }

    /// Runs a transaction once: a retry is answered `Repeated` and not written again; a command
    /// the gate refuses (`gate::refused`) or that has no SITL to go to is rejected and not recorded; a
    /// new one is written to SITL, whose `COMMAND_ACK` follows in the telemetry.
    fn discrete(
        &mut self,
        admitted: &Admitted,
        txn: TxnId,
        (frame, message): &(Frame, MavMessage),
        from: SocketAddr,
        now: u64,
        timing: impl Fn(u64) -> Timing,
    ) {
        if admitted.repeated() {
            self.state.record(admitted);
            self.stats.update(|c| c.repeated += 1);
            return self.result(from, txn, envelope::Outcome::Repeated);
        }
        let sticks_fresh = self.state.setpoint(now).is_some();
        if let Some(refused) = gate::refused(
            self.stats.vehicle(),
            message,
            sticks_fresh,
            self.stats.allow_missions(),
        ) {
            return self.refuse(from, None, Some(txn), refusal_of(refused));
        }
        self.stats.sitl.received(frame);
        if !self.deliver_discrete(frame) {
            return self.refuse(from, None, Some(txn), Refusal::NoDestination);
        }
        let timing = timing(unix_ms());
        self.state.record(admitted);
        self.stats.update(|c| {
            c.applied.discrete += 1;
            c.last_discrete = Some(timing);
        });
        self.result(from, txn, envelope::Outcome::Applied);
    }

    fn result(&self, to: SocketAddr, txn: TxnId, outcome: envelope::Outcome) {
        self.reply(
            to,
            &ToGround::CommandResult {
                txn: txn.0,
                outcome,
            },
        );
    }

    /// Counts a refusal and tells the sender: a discrete envelope gets its command result, anything
    /// else a rejection.
    fn refuse(&self, to: SocketAddr, seq: Option<u64>, txn: Option<TxnId>, refusal: Refusal) {
        self.stats.rejected(refusal);
        let reply = match txn {
            Some(txn) => ToGround::CommandResult {
                txn: txn.0,
                outcome: envelope::Outcome::Rejected { reason: refusal },
            },
            None => ToGround::Rejected {
                seq,
                reason: refusal,
            },
        };
        self.reply(to, &reply);
    }

    fn reply(&self, to: SocketAddr, message: &ToGround) {
        let sent = envelope::encode(message)
            .ok()
            .and_then(|bytes| self.socket.send_to(&bytes, to).ok());
        self.stats.update(|c| match sent {
            Some(len) => {
                c.replies += 1;
                c.reply_bytes += u64::try_from(len).unwrap_or(u64::MAX);
            }
            None => c.reply_failed += 1,
        });
    }

    /// Issues the holder's challenge when due and refreshes a fresh setpoint that has not been
    /// written for `REFRESH_EVERY`.
    fn tick(&mut self) {
        let now = self.now_ms();
        if now >= self.next_challenge_ms {
            self.next_challenge_ms = now.saturating_add(ms(CHALLENGE_EVERY));
            self.challenge(now);
        }
        if now.saturating_sub(self.last_setpoint_ms) >= ms(REFRESH_EVERY) {
            self.refresh(now);
        }
        if self.state.holder(now).is_none() {
            self.clear_region();
        }
        self.tell_follow(now);
        let lease = self
            .state
            .holder(now)
            .zip(self.holder)
            .map(|((_, id), holder)| LeaseView { id: id.0, holder });
        self.stats.update(|c| c.lease = lease);
    }

    /// Tells the holder where the followed box is when it has moved, and when the follow ended.
    fn tell_follow(&mut self, now: u64) {
        let (lost, _) = self.stats.region.follow_lost_count();
        let ended = lost != self.told.lost;
        if ended {
            self.told = Told {
                lost,
                target: None,
                at_ms: now,
                lost_seq: self.told.region_seq,
                lost_at_ms: Some(now),
                ..self.told
            };
        }
        let Some(holder) = self.holder else {
            return;
        };
        let lost_message = ToGround::Lost {
            region_seq: self.told.lost_seq,
        };
        if ended {
            return self.reply(holder, &lost_message);
        }
        let due = now.saturating_sub(self.told.at_ms) >= FOLLOW_REPORT_EVERY_MS;
        let following = self.stats.region.follow_order().is_some();
        if due && !following && lost_is_repeated(self.told.lost_at_ms, now) {
            self.told.at_ms = now;
            return self.reply(holder, &lost_message);
        }
        let target = self.stats.region.followed();
        if target.is_none() || target == self.told.target || !due {
            return;
        }
        self.told.target = target;
        self.told.at_ms = now;
        self.reply(holder, &ToGround::Region { region: target });
    }

    fn challenge(&mut self, now: u64) {
        let Some(holder) = self.holder else {
            return;
        };
        let Some(token) = self.state.issue_challenge(now) else {
            return;
        };
        self.stats.update(|c| c.challenges += 1);
        let challenge = ToGround::Challenge {
            epoch: self.state.epoch().0,
            token: token.0,
            lifetime_ms: self.challenge_lifetime_ms,
        };
        self.reply(holder, &challenge);
    }

    fn refresh(&mut self, now: u64) {
        let Some(setpoint) = self.state.setpoint(now) else {
            return;
        };
        if !self.deliver(setpoint) {
            return;
        }
        self.last_setpoint_ms = now;
        let at = unix_ms();
        self.stats.update(|c| {
            c.refreshed += 1;
            c.last_refresh_unix_ms = Some(at);
        });
    }
}

/// The box a region envelope carries, normalised; `Err` when it is not a valid rectangle. Other
/// kinds carry none.
fn region_of(envelope: &Envelope) -> Result<Option<NormRect>, Refusal> {
    if envelope.kind != Kind::Region {
        return Ok(None);
    }
    envelope
        .region
        .map(|r| NormRect::new(r.x0, r.y0, r.x1, r.y1).ok_or(Refusal::Malformed))
        .transpose()
}

/// The frame an envelope carries, decoded; `Err` when it carries something that is not one
/// `MAVLink` 2 frame of the dialect.
fn carried(envelope: &Envelope) -> Result<Option<(Frame, MavMessage)>, Refusal> {
    let Some(text) = envelope.mavlink.as_deref() else {
        return Ok(None);
    };
    let frame = envelope::frame(text).ok_or(Refusal::Malformed)?;
    let message = frame.decode().map_err(|_| Refusal::Malformed)?;
    Ok(Some((frame, message)))
}

fn refusal_of(refused: Refused) -> Refusal {
    match refused {
        Refused::ModeNotAllowed => Refusal::ModeNotAllowed,
        Refused::NoFreshSticks => Refusal::NoFreshSticks,
        Refused::TakeoffAltitude => Refusal::TakeoffAltitude,
        Refused::MissionNotAllowed => Refusal::MissionNotAllowed,
        Refused::MissionItemNotAllowed => Refusal::MissionItemNotAllowed,
        Refused::MissionTooLong => Refusal::MissionTooLong,
    }
}

/// A discrete envelope's transaction for the policy: its id and its command's digest.
fn discrete_txn(kind: Kind, carried: Option<&(Frame, MavMessage)>) -> Option<Txn> {
    let Kind::Discrete { txn } = kind else {
        return None;
    };
    let digest = carried
        .and_then(|(_, message)| CommandKey::of(message))
        .map_or(0, |key| key.digest());
    Some(Txn {
        id: TxnId(txn),
        digest,
    })
}

fn txn(kind: Kind) -> Option<TxnId> {
    match kind {
        Kind::Discrete { txn } => Some(TxnId(txn)),
        Kind::Liveness | Kind::Continuous | Kind::Region => None,
    }
}

fn ms(duration: Duration) -> u64 {
    u64::try_from(duration.as_millis()).unwrap_or(u64::MAX)
}

/// Whether the lost message is still repeated at `now`, given when the follow ended.
fn lost_is_repeated(lost_at_ms: Option<u64>, now: u64) -> bool {
    lost_at_ms.is_some_and(|at| now.saturating_sub(at) < LOST_REPEAT_FOR_MS)
}

#[cfg(test)]
mod tests {
    use mavlink::dialects::ardupilotmega::{
        HEARTBEAT_DATA, MavParamType, MavType, PARAM_VALUE_DATA,
    };

    use sat_teleop_policy::follow::Lost;

    use super::*;
    use crate::params::ParamType;
    use sat_teleop_policy::region::RegionLimits;

    fn said(name: &str, value: f32, kind: MavParamType) -> MavMessage {
        MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_id: name.into(),
            param_value: value,
            param_type: kind,
            ..PARAM_VALUE_DATA::default()
        })
    }

    /// Only the autopilot's own answer counts: another component's answer to the same name is
    /// neither a reading nor a replacement for the autopilot's.
    #[test]
    fn a_parameter_counts_only_as_the_autopilot_said_it() {
        let stats = ControlStats::default();
        if let Ok(mut real) = stats.real.lock() {
            real.required = vec![Required {
                name: "FS_GCS_ENABLE".to_owned(),
                kind: ParamType::Int8,
                value: 5.0,
                compare: crate::params::Compare::Equal,
            }];
        }
        let (autopilot, other) = ((1, 1), (1, 200));
        stats.heartbeat(autopilot, Some(Vehicle::Copter));
        let right = said("FS_GCS_ENABLE", 5.0, MavParamType::MAV_PARAM_TYPE_INT8);
        let wrong = said("FS_GCS_ENABLE", 0.0, MavParamType::MAV_PARAM_TYPE_INT8);
        let failure = || stats.param_failure(stats.autopilot().address);

        stats.param_value(other, &right);
        assert!(
            failure().is_some_and(|f| f.contains("was not read")),
            "{:?}",
            failure()
        );
        stats.param_value(autopilot, &right);
        assert_eq!(failure(), None);
        stats.param_value(other, &wrong);
        assert_eq!(failure(), None, "another component changed the reading");
        stats.param_value(autopilot, &wrong);
        assert!(failure().is_some_and(|f| f.contains("is 0")));
    }

    /// A controller on loopback with a key, and a vehicle endpoint it talks to. Without an
    /// autopilot every frame that would reach one goes to the logging sink and is counted in `sunk`.
    struct Rig {
        stats: ControlStats,
        writer: SitlWriter,
        signer: crate::auth::Signer,
        controller: UdpSocket,
    }

    struct Taken {
        epoch: u64,
        lease: u64,
        challenge: u64,
    }

    const EPOCH: u64 = 7;
    const REGION_LIMITS: RegionLimits = RegionLimits {
        pilot_ttl_ms: 3000,
        detection_ttl_ms: 1000,
        max_regions: 4,
        max_area_percent: 50,
    };
    const OTHER_CONTROLLER: [u8; 4] = [127, 0, 0, 2];

    fn rig() -> Rig {
        let vehicle = "v1".parse().unwrap();
        let signer = crate::auth::Signer::random(&vehicle).unwrap();
        let stats = ControlStats::default();
        stats.region.configure(REGION_LIMITS);
        let ip = IpAddr::from([127, 0, 0, 1]);
        stats
            .keys
            .lock()
            .unwrap()
            .verifiers
            .insert(ip, signer.verifier());
        stats
            .keys
            .lock()
            .unwrap()
            .verifiers
            .insert(IpAddr::from(OTHER_CONTROLLER), signer.verifier());
        let controller = UdpSocket::bind("127.0.0.1:0").unwrap();
        controller
            .set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        Rig {
            stats,
            writer: SitlWriter::default(),
            signer,
            controller,
        }
    }

    impl Rig {
        fn endpoint(&self) -> Endpoint<'_> {
            let limits = sat_teleop_policy::control::Limits {
                challenge_ms: 5000,
                setpoint_ms: 1000,
                lease_ms: 10_000,
            };
            Endpoint {
                socket: UdpSocket::bind("127.0.0.1:0").unwrap(),
                writer: &self.writer,
                stats: &self.stats,
                started: Instant::now(),
                challenge_lifetime_ms: limits.challenge_ms,
                state: LeaseState::new(
                    Epoch(EPOCH),
                    limits,
                    vec![
                        PeerId(IpAddr::from([127, 0, 0, 1])),
                        PeerId(IpAddr::from(OTHER_CONTROLLER)),
                    ],
                ),
                holder: None,
                told: Told::default(),
                next_challenge_ms: 0,
                last_setpoint_ms: 0,
                last_nonce: HashMap::new(),
            }
        }

        fn send(&self, endpoint: &mut Endpoint<'_>, message: &ToVehicle) {
            let from = self.controller.local_addr().unwrap();
            endpoint.receive(&envelope::encode(message).unwrap(), from);
        }

        fn reply(&self) -> Option<ToGround> {
            let mut buf = [0_u8; 2048];
            let len = self.controller.recv(&mut buf).ok()?;
            envelope::decode(buf.get(..len)?).ok()
        }

        /// Requests the lease and takes the first challenge.
        fn take(&self, endpoint: &mut Endpoint<'_>) -> Taken {
            let sig = self.signer.sign_request(1).unwrap();
            let request = ToVehicle::LeaseRequest {
                peer: "test".to_owned(),
                nonce: 1,
                sig: Some(sig),
            };
            self.send(endpoint, &request);
            let Some(ToGround::LeaseGrant { lease, epoch, .. }) = self.reply() else {
                panic!("no grant");
            };
            endpoint.tick();
            let Some(ToGround::Challenge { token, .. }) = self.reply() else {
                panic!("no challenge");
            };
            Taken {
                epoch,
                lease,
                challenge: token,
            }
        }

        fn region_envelope(
            &self,
            taken: &Taken,
            seq: u64,
            region: Option<NormRect>,
            mavlink: Option<String>,
        ) -> ToVehicle {
            let envelope = Envelope {
                epoch: taken.epoch,
                lease: taken.lease,
                seq,
                challenge: taken.challenge,
                kind: Kind::Region,
                mavlink,
                region,
                sent_unix_ms: 0,
                sig: None,
            };
            ToVehicle::Envelope(self.signer.sign(envelope).unwrap())
        }

        fn nothing_reached_the_autopilot(&self) {
            assert_eq!(self.stats.snapshot().counts.sunk, 0);
            assert_eq!(self.stats.sitl.snapshot().messages_in, 0);
        }
    }

    fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> NormRect {
        NormRect { x0, y0, x1, y1 }
    }

    #[test]
    fn a_followed_box_is_told_where_it_is_and_then_that_it_is_lost() {
        let rig = rig();
        rig.stats.region.enable_follow();
        let mut endpoint = rig.endpoint();
        let taken = rig.take(&mut endpoint);
        let (drawn, moved) = (rect(0.2, 0.1, 0.6, 0.7), rect(0.3, 0.1, 0.7, 0.7));
        rig.send(
            &mut endpoint,
            &rig.region_envelope(&taken, 1, Some(drawn), None),
        );
        assert!(matches!(rig.reply(), Some(ToGround::Region { region: Some(r) }) if r == drawn));

        let generation = rig.stats.region.follow_order().unwrap().generation;
        rig.stats
            .region
            .follow_moved(generation, moved, rig.stats.region.now_ms());
        thread::sleep(Duration::from_millis(FOLLOW_REPORT_EVERY_MS + 50));
        endpoint.tick();
        assert!(matches!(rig.reply(), Some(ToGround::Region { region: Some(r) }) if r == moved));

        rig.stats.region.follow_lost(generation, Lost::Score);
        endpoint.tick();
        assert!(matches!(
            rig.reply(),
            Some(ToGround::Lost { region_seq: 1 })
        ));
        endpoint.tick();
        assert!(
            rig.reply().is_none(),
            "not again before the next report tick"
        );
        thread::sleep(Duration::from_millis(FOLLOW_REPORT_EVERY_MS + 50));
        endpoint.tick();
        let repeated = std::iter::from_fn(|| rig.reply())
            .find(|reply| !matches!(reply, ToGround::Challenge { .. }));
        assert!(matches!(repeated, Some(ToGround::Lost { region_seq: 1 })));
        rig.nothing_reached_the_autopilot();
    }

    #[test]
    fn region_envelope_sets_the_region_and_reaches_no_autopilot() {
        let rig = rig();
        let mut endpoint = rig.endpoint();
        let taken = rig.take(&mut endpoint);
        let wanted = rect(0.2, 0.1, 0.6, 0.7);
        rig.send(
            &mut endpoint,
            &rig.region_envelope(&taken, 1, Some(wanted), None),
        );
        assert!(matches!(rig.reply(), Some(ToGround::Region { region: Some(r) }) if r == wanted));
        assert_eq!(rig.stats.region.latest().map(|t| t.rect), Some(wanted));
        rig.nothing_reached_the_autopilot();
    }

    #[test]
    fn a_region_the_sender_does_not_keep_is_confirmed_as_none() {
        let rig = rig();
        let mut endpoint = rig.endpoint();
        let taken = rig.take(&mut endpoint);
        let most_of_the_picture = rect(0.0, 0.0, 0.9, 0.9);
        rig.send(
            &mut endpoint,
            &rig.region_envelope(&taken, 1, Some(most_of_the_picture), None),
        );
        assert!(matches!(
            rig.reply(),
            Some(ToGround::Region { region: None })
        ));
        assert!(rig.stats.region.latest().is_some());
    }

    #[test]
    fn region_envelope_with_a_mavlink_frame_is_refused() {
        let rig = rig();
        let mut endpoint = rig.endpoint();
        let taken = rig.take(&mut endpoint);
        let first = rect(0.2, 0.1, 0.6, 0.7);
        rig.send(
            &mut endpoint,
            &rig.region_envelope(&taken, 1, Some(first), None),
        );
        rig.reply();
        let frame = crate::frame::encoded(&MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            mavtype: MavType::MAV_TYPE_GCS,
            ..HEARTBEAT_DATA::default()
        }));
        let carrying = rig.region_envelope(
            &taken,
            2,
            Some(rect(0.5, 0.5, 0.9, 0.9)),
            Some(envelope::encode_frame(&frame)),
        );
        rig.send(&mut endpoint, &carrying);
        assert!(matches!(
            rig.reply(),
            Some(ToGround::Rejected {
                reason: Refusal::NotAllowlisted,
                ..
            })
        ));
        assert_eq!(rig.stats.region.latest().map(|t| t.rect), Some(first));
        rig.nothing_reached_the_autopilot();
    }

    #[test]
    fn a_degenerate_region_is_refused_and_the_old_one_stays() {
        let rig = rig();
        let mut endpoint = rig.endpoint();
        let taken = rig.take(&mut endpoint);
        let first = rect(0.2, 0.1, 0.6, 0.7);
        rig.send(
            &mut endpoint,
            &rig.region_envelope(&taken, 1, Some(first), None),
        );
        rig.reply();
        let sliver = rect(0.5, 0.5, 0.5, 0.9);
        rig.send(
            &mut endpoint,
            &rig.region_envelope(&taken, 2, Some(sliver), None),
        );
        assert!(matches!(
            rig.reply(),
            Some(ToGround::Rejected {
                reason: Refusal::Malformed,
                ..
            })
        ));
        assert_eq!(rig.stats.region.latest().map(|t| t.rect), Some(first));
        let next = rect(0.1, 0.1, 0.3, 0.3);
        rig.send(
            &mut endpoint,
            &rig.region_envelope(&taken, 2, Some(next), None),
        );
        assert!(matches!(rig.reply(), Some(ToGround::Region { region: Some(r) }) if r == next));
    }

    #[test]
    fn region_needs_the_lease() {
        let rig = rig();
        let mut endpoint = rig.endpoint();
        let wanted = Some(rect(0.2, 0.1, 0.6, 0.7));
        let refused = |reason| {
            let counts = rig.stats.snapshot().counts;
            counts.rejected.get(&reason).copied().unwrap_or_default()
        };

        let unheld = Taken {
            epoch: EPOCH,
            lease: 1,
            challenge: 1,
        };
        rig.send(
            &mut endpoint,
            &rig.region_envelope(&unheld, 1, wanted, None),
        );
        rig.reply();
        assert_eq!(refused(Refusal::NoLease), 1);

        // A listed controller that is not the holder.
        let taken = rig.take(&mut endpoint);
        let other = SocketAddr::from((OTHER_CONTROLLER, 9));
        let envelope = rig.region_envelope(&taken, 1, wanted, None);
        endpoint.receive(&envelope::encode(&envelope).unwrap(), other);
        assert_eq!(refused(Refusal::NotHolder), 1);
        assert!(rig.stats.region.latest().is_none());
    }

    #[test]
    fn lease_change_clears_the_region() {
        let rig = rig();
        let cleared = |rig: &Rig| {
            assert!(matches!(
                rig.reply(),
                Some(ToGround::Region { region: None })
            ));
            assert!(rig.stats.region.latest().is_none());
        };

        let mut endpoint = rig.endpoint();
        let taken = rig.take(&mut endpoint);
        let envelope = rig.region_envelope(&taken, 1, Some(rect(0.2, 0.1, 0.6, 0.7)), None);
        rig.send(&mut endpoint, &envelope);
        rig.reply();
        let sig = rig.signer.sign_release(taken.epoch, taken.lease).unwrap();
        rig.send(
            &mut endpoint,
            &ToVehicle::LeaseRelease {
                lease: taken.lease,
                sig: Some(sig),
            },
        );
        cleared(&rig);

        let mut endpoint = rig.endpoint();
        let taken = rig.take(&mut endpoint);
        let envelope = rig.region_envelope(&taken, 1, Some(rect(0.2, 0.1, 0.6, 0.7)), None);
        rig.send(&mut endpoint, &envelope);
        rig.reply();
        endpoint.started -= Duration::from_secs(60);
        endpoint.tick();
        cleared(&rig);
    }
}
