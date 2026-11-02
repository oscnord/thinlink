//! The vehicle's control lease and command freshness. One controller holds the lease at a time;
//! its envelopes count only when they carry a challenge the vehicle issued recently in this
//! session and a sequence number above the last one. Continuous setpoints keep only the newest,
//! for a short time; discrete commands run at most once per transaction. Time is milliseconds of
//! the vehicle's monotonic clock, passed in.

use std::collections::VecDeque;
use std::net::IpAddr;

/// Challenges an envelope may name: the most recent ones only.
const RECENT_CHALLENGES: usize = 4;
/// Discrete transactions remembered, least recently used dropped first.
const TXN_CACHE: usize = 64;

/// One vehicle session; a new one each time the vehicle starts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Epoch(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LeaseId(pub u64);

/// A controller, identified by its overlay address (attested by the relay, which checks each
/// `WireGuard` peer's address).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PeerId(pub IpAddr);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Seq(pub u64);

/// Proves an envelope was made after the vehicle issued it. Not secret: it proves freshness, not
/// identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ChallengeToken(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TxnId(pub u64);

/// Why a lease request or an envelope is refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reason {
    /// Not on the vehicle's list of controllers.
    NotController,
    /// Another controller holds an unexpired lease.
    LeaseHeld,
    NoLease,
    LeaseExpired,
    /// The envelope comes from another controller than the holder.
    NotHolder,
    /// From the holder, but naming another lease than its current one.
    WrongLease,
    WrongEpoch,
    /// Not among the challenges issued most recently in this session.
    UnknownChallenge,
    ChallengeExpired,
    DuplicateSeq,
    ReorderedSeq,
    /// A known transaction id with a different command: a retry must repeat the command exactly.
    TxnConflict,
}

/// Lifetimes in milliseconds.
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    /// A challenge is fresh this long after it was issued.
    pub challenge_ms: u64,
    /// A continuous setpoint is fresh this long after it arrived (and while its challenge is).
    pub setpoint_ms: u64,
    /// The lease lasts this long after its grant or its holder's last valid envelope.
    pub lease_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Grant {
    pub lease: LeaseId,
    pub epoch: Epoch,
}

/// What an envelope claims about itself.
#[derive(Debug, Clone, Copy)]
pub struct Header {
    pub epoch: Epoch,
    pub lease: LeaseId,
    pub kind: Kind,
    pub seq: Seq,
    pub challenge: ChallengeToken,
    /// A discrete envelope's transaction.
    pub txn: Option<Txn>,
}

/// A discrete transaction: its id, and a digest of the command it carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Txn {
    pub id: TxnId,
    pub digest: u64,
}

/// The envelope kinds. Each counts its own sequence numbers per lease, so a stop command is never
/// refused because a setpoint sent after it overtook it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Liveness,
    Continuous,
    Discrete,
    /// The pilot's region for the picture: judged like liveness, never carries a command.
    Region,
}

/// The last admitted sequence number of each kind.
#[derive(Debug, Default, Clone, Copy)]
struct LastSeq {
    liveness: Option<Seq>,
    continuous: Option<Seq>,
    discrete: Option<Seq>,
    region: Option<Seq>,
}

impl LastSeq {
    fn of(&mut self, kind: Kind) -> &mut Option<Seq> {
        match kind {
            Kind::Liveness => &mut self.liveness,
            Kind::Continuous => &mut self.continuous,
            Kind::Discrete => &mut self.discrete,
            Kind::Region => &mut self.region,
        }
    }
}

/// Proof that an envelope passed [`LeaseState::admit`]; applying it needs one.
#[derive(Debug)]
pub struct Admitted {
    lease: LeaseId,
    challenge_issued_ms: u64,
    txn: Option<Txn>,
    repeated: bool,
}

impl Admitted {
    /// A retry of a transaction this lease already applied: answer it as applied, never apply it
    /// again.
    pub fn repeated(&self) -> bool {
        self.repeated
    }
}

#[derive(Debug, Clone, Copy)]
struct Lease {
    id: LeaseId,
    holder: PeerId,
    expires_ms: u64,
    last_seq: LastSeq,
}

/// The vehicle's control state for one session. `T` is a continuous setpoint.
#[derive(Debug)]
pub struct LeaseState<T> {
    epoch: Epoch,
    limits: Limits,
    /// The only peers that may ask for the lease or send envelopes.
    controllers: Vec<PeerId>,
    lease: Option<Lease>,
    leases_granted: u64,
    /// Issued challenges and when, newest last.
    challenges: VecDeque<(ChallengeToken, u64)>,
    challenges_issued: u64,
    slot: ContinuousSlot<T>,
    txns: TxnCache,
}

impl<T> LeaseState<T> {
    pub fn new(epoch: Epoch, limits: Limits, controllers: Vec<PeerId>) -> Self {
        Self {
            epoch,
            limits,
            controllers,
            lease: None,
            leases_granted: 0,
            challenges: VecDeque::with_capacity(RECENT_CHALLENGES),
            challenges_issued: 0,
            slot: ContinuousSlot::default(),
            txns: TxnCache::default(),
        }
    }

    /// Refuses a peer that is not on the list, before anything else looks at what it sent.
    pub fn controller(&self, peer: PeerId) -> Result<(), Reason> {
        if self.controllers.contains(&peer) {
            Ok(())
        } else {
            Err(Reason::NotController)
        }
    }

    pub fn epoch(&self) -> Epoch {
        self.epoch
    }

    /// The holder and lease while the lease has not expired.
    pub fn holder(&self, now_ms: u64) -> Option<(PeerId, LeaseId)> {
        self.active(now_ms).map(|l| (l.holder, l.id))
    }

    /// Grants a new lease to `peer` unless another peer holds an unexpired one. The holder asking
    /// again gets a new lease; its old one is gone, with the old challenges and setpoint.
    pub fn request(&mut self, peer: PeerId, now_ms: u64) -> Result<Grant, Reason> {
        self.controller(peer)?;
        if self.active(now_ms).is_some_and(|l| l.holder != peer) {
            return Err(Reason::LeaseHeld);
        }
        self.leases_granted += 1;
        let id = LeaseId(self.leases_granted);
        self.lease = Some(Lease {
            id,
            holder: peer,
            expires_ms: now_ms.saturating_add(self.limits.lease_ms),
            last_seq: LastSeq::default(),
        });
        self.challenges.clear();
        self.slot.clear();
        Ok(Grant {
            lease: id,
            epoch: self.epoch,
        })
    }

    /// Ends the lease if `peer` holds it as `lease`.
    pub fn release(&mut self, peer: PeerId, lease: LeaseId, now_ms: u64) -> Result<(), Reason> {
        self.controller(peer)?;
        self.current(peer, lease, now_ms)?;
        self.lease = None;
        self.challenges.clear();
        self.slot.clear();
        Ok(())
    }

    /// A new challenge for the holder; `None` without an unexpired lease.
    pub fn issue_challenge(&mut self, now_ms: u64) -> Option<ChallengeToken> {
        self.active(now_ms)?;
        self.challenges_issued += 1;
        let token = ChallengeToken(self.challenges_issued);
        if self.challenges.len() == RECENT_CHALLENGES {
            self.challenges.pop_front();
        }
        self.challenges.push_back((token, now_ms));
        Some(token)
    }

    /// Checks an envelope from `peer`: a listed controller, this session, the current lease and its holder, a fresh
    /// recent challenge, a sequence number above the last of its kind, and for a known transaction
    /// the same command. A refused envelope changes nothing; a valid one renews the lease.
    pub fn admit(
        &mut self,
        peer: PeerId,
        header: &Header,
        now_ms: u64,
    ) -> Result<Admitted, Reason> {
        self.controller(peer)?;
        if header.epoch != self.epoch {
            return Err(Reason::WrongEpoch);
        }
        self.current(peer, header.lease, now_ms)?;
        let challenge_issued_ms = self.challenge_issued(header.challenge, now_ms)?;
        let lease_ms = self.limits.lease_ms;
        let lease = self.lease.as_mut().ok_or(Reason::NoLease)?;
        let last_seq = lease.last_seq.of(header.kind);
        match *last_seq {
            Some(last) if header.seq == last => return Err(Reason::DuplicateSeq),
            Some(last) if header.seq < last => return Err(Reason::ReorderedSeq),
            _ => {}
        }
        let known = header.txn.and_then(|t| self.txns.digest(lease.id, t.id));
        let repeated = match (header.txn, known) {
            (Some(txn), Some(digest)) if txn.digest != digest => return Err(Reason::TxnConflict),
            (_, known) => known.is_some(),
        };
        *last_seq = Some(header.seq);
        lease.expires_ms = now_ms.saturating_add(lease_ms);
        Ok(Admitted {
            lease: lease.id,
            challenge_issued_ms,
            txn: header.txn,
            repeated,
        })
    }

    /// Makes `setpoint` the only one, fresh until the setpoint lifetime or its challenge's runs out.
    pub fn offer_setpoint(&mut self, admitted: &Admitted, setpoint: T, now_ms: u64) {
        let expires_ms = now_ms.saturating_add(self.limits.setpoint_ms).min(
            admitted
                .challenge_issued_ms
                .saturating_add(self.limits.challenge_ms),
        );
        self.slot.offer(setpoint, expires_ms);
    }

    /// The setpoint to keep refreshing, while it and the lease are fresh.
    pub fn setpoint(&self, now_ms: u64) -> Option<&T> {
        self.active(now_ms)?;
        self.slot.fresh(now_ms)
    }

    /// Remembers the transaction of an applied discrete envelope (or, for a retry, that it was
    /// used again).
    pub fn record(&mut self, admitted: &Admitted) {
        if let Some(txn) = admitted.txn {
            self.txns.insert(admitted.lease, txn);
        }
    }

    fn active(&self, now_ms: u64) -> Option<&Lease> {
        self.lease.as_ref().filter(|l| now_ms < l.expires_ms)
    }

    /// The lease, if `peer` holds it as `lease` and it has not expired.
    fn current(&self, peer: PeerId, lease: LeaseId, now_ms: u64) -> Result<(), Reason> {
        let Some(current) = self.lease else {
            return Err(Reason::NoLease);
        };
        if current.holder != peer {
            return Err(Reason::NotHolder);
        }
        if current.id != lease {
            return Err(Reason::WrongLease);
        }
        if now_ms >= current.expires_ms {
            return Err(Reason::LeaseExpired);
        }
        Ok(())
    }

    fn challenge_issued(&self, token: ChallengeToken, now_ms: u64) -> Result<u64, Reason> {
        let issued_ms = self
            .challenges
            .iter()
            .find(|(t, _)| *t == token)
            .map(|(_, at)| *at)
            .ok_or(Reason::UnknownChallenge)?;
        if now_ms.saturating_sub(issued_ms) > self.limits.challenge_ms {
            return Err(Reason::ChallengeExpired);
        }
        Ok(issued_ms)
    }
}

/// The newest continuous setpoint and when it stops being fresh. Older ones are gone.
#[derive(Debug)]
pub struct ContinuousSlot<T> {
    latest: Option<(T, u64)>,
}

impl<T> Default for ContinuousSlot<T> {
    fn default() -> Self {
        Self { latest: None }
    }
}

impl<T> ContinuousSlot<T> {
    pub fn offer(&mut self, setpoint: T, expires_ms: u64) {
        self.latest = Some((setpoint, expires_ms));
    }

    pub fn fresh(&self, now_ms: u64) -> Option<&T> {
        self.latest
            .as_ref()
            .filter(|(_, expires_ms)| now_ms < *expires_ms)
            .map(|(setpoint, _)| setpoint)
    }

    pub fn clear(&mut self) {
        self.latest = None;
    }
}

/// Recently applied discrete transactions per lease, with their command digests, at most
/// `TXN_CACHE`, least recently used dropped first.
#[derive(Debug, Default)]
pub struct TxnCache(VecDeque<(LeaseId, Txn)>);

impl TxnCache {
    /// The command digest of a known transaction.
    pub fn digest(&self, lease: LeaseId, id: TxnId) -> Option<u64> {
        self.0
            .iter()
            .find(|(l, t)| *l == lease && t.id == id)
            .map(|(_, t)| t.digest)
    }

    /// Remembers a transaction as the most recently used.
    pub fn insert(&mut self, lease: LeaseId, txn: Txn) {
        self.0.retain(|(l, t)| !(*l == lease && t.id == txn.id));
        if self.0.len() == TXN_CACHE {
            self.0.pop_front();
        }
        self.0.push_back((lease, txn));
    }
}

#[cfg(test)]
mod tests {
    use std::net::Ipv4Addr;

    use super::*;

    const E: Epoch = Epoch(1_000);
    const A: PeerId = PeerId(IpAddr::V4(Ipv4Addr::new(10, 77, 0, 3)));
    const B: PeerId = PeerId(IpAddr::V4(Ipv4Addr::new(10, 77, 0, 4)));
    /// An overlay peer not on the list, such as the relay.
    const C: PeerId = PeerId(IpAddr::V4(Ipv4Addr::new(10, 77, 0, 1)));
    const LIMITS: Limits = Limits {
        challenge_ms: 1_500,
        setpoint_ms: 500,
        lease_ms: 3_000,
    };

    /// Liveness, a continuous setpoint, or a discrete transaction.
    #[derive(Debug, Clone, Copy)]
    enum Envelope {
        Live,
        Set(&'static str),
        /// Transaction id and command digest.
        Txn(u64, u64),
    }

    #[derive(Debug, Clone, Copy)]
    enum Step {
        Request(PeerId),
        Release(PeerId, u64),
        Issue,
        /// From whom: epoch, lease, seq, challenge, kind.
        Send(PeerId, Epoch, u64, u64, u64, Envelope),
        Refresh,
    }

    #[derive(Debug, PartialEq, Eq)]
    enum Got {
        Granted(u64),
        Released,
        Token(Option<u64>),
        Admitted,
        /// A transaction run for the first time.
        Ran,
        Repeated,
        Refused(Reason),
        Setpoint(Option<&'static str>),
    }

    fn run(state: &mut LeaseState<&'static str>, now: u64, step: Step) -> Got {
        match step {
            Step::Request(peer) => match state.request(peer, now) {
                Ok(grant) => Got::Granted(grant.lease.0),
                Err(reason) => Got::Refused(reason),
            },
            Step::Release(peer, lease) => match state.release(peer, LeaseId(lease), now) {
                Ok(()) => Got::Released,
                Err(reason) => Got::Refused(reason),
            },
            Step::Issue => Got::Token(state.issue_challenge(now).map(|t| t.0)),
            Step::Send(peer, epoch, lease, seq, challenge, kind) => {
                let header = Header {
                    epoch,
                    lease: LeaseId(lease),
                    kind: match kind {
                        Envelope::Live => Kind::Liveness,
                        Envelope::Set(_) => Kind::Continuous,
                        Envelope::Txn(..) => Kind::Discrete,
                    },
                    seq: Seq(seq),
                    challenge: ChallengeToken(challenge),
                    txn: match kind {
                        Envelope::Txn(id, digest) => Some(Txn {
                            id: TxnId(id),
                            digest,
                        }),
                        _ => None,
                    },
                };
                let admitted = match state.admit(peer, &header, now) {
                    Ok(admitted) => admitted,
                    Err(reason) => return Got::Refused(reason),
                };
                match kind {
                    Envelope::Live => Got::Admitted,
                    Envelope::Set(setpoint) => {
                        state.offer_setpoint(&admitted, setpoint, now);
                        Got::Admitted
                    }
                    Envelope::Txn(..) => {
                        state.record(&admitted);
                        if admitted.repeated() {
                            return Got::Repeated;
                        }
                        Got::Ran
                    }
                }
            }
            Step::Refresh => Got::Setpoint(state.setpoint(now).copied()),
        }
    }

    #[test]
    fn only_fresh_ordered_envelopes_from_the_holder_count() {
        use Envelope::{Live, Set, Txn};
        use Got::{Admitted, Granted, Ran, Refused, Released, Repeated, Setpoint, Token};
        use Reason::{
            ChallengeExpired, DuplicateSeq, LeaseExpired, LeaseHeld, NoLease, NotController,
            NotHolder, ReorderedSeq, TxnConflict, UnknownChallenge, WrongEpoch, WrongLease,
        };
        use Step::{Issue, Refresh, Release, Request, Send};
        let old = Epoch(E.0 - 1);
        // (ms, step, expected): one session, in order.
        let steps = [
            // Only listed controllers; two of them, one lease.
            (0, Request(C), Refused(NotController)),
            (0, Request(A), Granted(1)),
            (0, Request(B), Refused(LeaseHeld)),
            (0, Issue, Token(Some(1))),
            (10, Send(A, E, 1, 1, 1, Live), Admitted),
            (10, Send(B, E, 1, 2, 1, Live), Refused(NotHolder)),
            (10, Send(A, old, 1, 2, 1, Live), Refused(WrongEpoch)),
            (20, Send(A, E, 1, 1, 1, Live), Refused(DuplicateSeq)),
            (20, Send(A, E, 1, 3, 1, Live), Admitted),
            (20, Send(A, E, 1, 2, 1, Live), Refused(ReorderedSeq)),
            (20, Send(A, E, 1, 4, 9, Live), Refused(UnknownChallenge)),
            // Challenge 1 was issued at 0 and lives 1500 ms.
            (1501, Send(A, E, 1, 4, 1, Live), Refused(ChallengeExpired)),
            // A refused envelope changes nothing: its seq is still free, and (at 4000 below) it
            // does not renew the lease.
            (1501, Issue, Token(Some(2))),
            (1505, Send(A, E, 1, 4, 2, Live), Admitted),
            // A retried transaction after a lost acknowledgement is answered, not run again.
            (1510, Send(A, E, 1, 5, 2, Txn(7, 1)), Ran),
            (1600, Send(A, E, 1, 6, 2, Txn(7, 1)), Repeated),
            // A known transaction with another command is refused, and changes nothing.
            (1600, Send(A, E, 1, 7, 2, Txn(7, 2)), Refused(TxnConflict)),
            (1600, Send(A, E, 1, 7, 2, Txn(8, 1)), Ran),
            // Latest only, refreshed while fresh: 500 ms after it arrived...
            (1700, Send(A, E, 1, 8, 2, Set("slow")), Admitted),
            (1710, Send(A, E, 1, 9, 2, Set("fast")), Admitted),
            // Each kind counts its own: a discrete envelope below the setpoints' seq still counts.
            (1720, Send(A, E, 1, 8, 2, Txn(9, 1)), Ran),
            (
                1720,
                Send(A, E, 1, 9, 2, Set("fast")),
                Refused(DuplicateSeq),
            ),
            (2209, Refresh, Setpoint(Some("fast"))),
            (2210, Refresh, Setpoint(None)),
            // ...or while its challenge is fresh, if that ends first.
            (2900, Send(A, E, 1, 10, 2, Set("late")), Admitted),
            (3000, Refresh, Setpoint(Some("late"))),
            (3001, Refresh, Setpoint(None)),
            (4000, Send(A, E, 1, 11, 2, Live), Refused(ChallengeExpired)),
            // The lease ends 3000 ms after the holder's last valid envelope: no challenges, no
            // envelopes, and another controller may take it.
            (5899, Issue, Token(Some(3))),
            (5900, Issue, Token(None)),
            (5900, Send(A, E, 1, 11, 3, Live), Refused(LeaseExpired)),
            (5900, Request(B), Granted(2)),
            (5900, Send(A, E, 1, 11, 3, Live), Refused(NotHolder)),
            // Release, then a new lease; the old one is dead.
            (6000, Release(A, 2), Refused(NotHolder)),
            (6000, Release(B, 2), Released),
            (6000, Send(B, E, 2, 1, 3, Live), Refused(NoLease)),
            (6000, Request(A), Granted(3)),
            (6000, Issue, Token(Some(4))),
            (6010, Send(A, E, 1, 12, 4, Live), Refused(WrongLease)),
            (6010, Send(A, E, 3, 1, 4, Live), Admitted),
            // Only the four most recent challenges count.
            (6100, Issue, Token(Some(5))),
            (6100, Issue, Token(Some(6))),
            (6100, Issue, Token(Some(7))),
            (6110, Send(A, E, 3, 2, 4, Live), Admitted),
            (6110, Issue, Token(Some(8))),
            (6120, Send(A, E, 3, 3, 4, Live), Refused(UnknownChallenge)),
            // Transactions are per lease: lease 3 has not run transaction 7.
            (6120, Send(A, E, 3, 4, 8, Txn(7, 1)), Ran),
        ];
        let mut state = LeaseState::new(E, LIMITS, vec![A, B]);
        for (now, step, expected) in steps {
            assert_eq!(run(&mut state, now, step), expected, "{now} ms: {step:?}");
        }
    }
}
