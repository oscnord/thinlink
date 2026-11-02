//! The vehicle's side of retrieval: polls the relay for a job, picks the nearest retained snapshot
//! at or before the time asked for, crops it, and uploads the result in chunks. It sends only when
//! `sat_teleop_policy::retrieval::permit` says live video and control are not paying for it, one
//! chunk at a time and paced by a token bucket at the retrieval allowance; it stops at the next
//! chunk when the conditions go, the job is cancelled or its life is over.

use std::fs;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use sat_teleop_policy::rate::{Priority, TokenBucket};
use sat_teleop_policy::retrieval::{self, Conditions, Hold};
use sat_teleop_policy::token::Token;

use crate::DetailError;
use crate::client::{Client, ClientError};
use crate::crop::{self, Cropped};
use crate::store::{Entry, Store};
use crate::wire::{
    Assignment, Complete, Failure, JobRequest, Kind, Listed, Listing, MAX_LISTED, Meta, sha256_hex,
};
use crate::wire::{CHUNK_HASH_HEADER, MAX_REASON};

/// How often a retrieval that may not send looks again.
const HOLD_POLL: Duration = Duration::from_millis(250);
/// How often it looks for tokens.
const PACE_POLL: Duration = Duration::from_millis(50);
const POLL_TIMEOUT: Duration = Duration::from_secs(10);
const UPLOAD_TIMEOUT: Duration = Duration::from_secs(15);
const RETRY_PAUSE: Duration = Duration::from_millis(500);
/// A chunk is tried this often before the job is given up.
const ATTEMPTS: u32 = 3;
/// A snapshot is kept from retention this long after a job picked it, while it is read.
const PIN_FOR: Duration = Duration::from_secs(10);

/// `[retrieval]` of `testbed/limits.toml`.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RetrievalLimits {
    /// What retrieval may send, in bytes per second on the wire. Zero switches retrieval off.
    pub allowance_bytes_per_s: u32,
}

impl RetrievalLimits {
    pub fn validate(self) -> Result<Self, DetailError> {
        if self.allowance_bytes_per_s > 1_000_000 {
            return Err(DetailError::Config(
                "[retrieval]: allowance_bytes_per_s is at most 1000000".to_owned(),
            ));
        }
        Ok(self)
    }
}

/// What the retrieval loop is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum State {
    /// Not allowed to ask for work; see `hold`.
    Held,
    /// Waiting for a job at the relay.
    Polling,
    /// Preparing or uploading a job.
    Working,
}

#[derive(Debug)]
struct Status {
    state: State,
    hold: Option<Hold>,
    /// When the current hold began.
    held_since: Option<Instant>,
}

/// Counters shared with the metrics endpoint.
#[derive(Debug)]
pub struct RetrievalStats {
    status: Mutex<Status>,
    /// The allowance reserved in the uplink budget while a job runs; zero while idle.
    reserved: AtomicU32,
    jobs_done: AtomicU64,
    jobs_failed: AtomicU64,
    jobs_cancelled: AtomicU64,
    jobs_expired: AtomicU64,
    chunks_sent: AtomicU64,
    chunk_failures: AtomicU64,
    payload_bytes: AtomicU64,
    wire_bytes: AtomicU64,
    holds: AtomicU64,
    held_ms: AtomicU64,
}

impl Default for RetrievalStats {
    fn default() -> Self {
        Self {
            status: Mutex::new(Status {
                state: State::Held,
                hold: None,
                held_since: Some(Instant::now()),
            }),
            reserved: AtomicU32::new(0),
            jobs_done: AtomicU64::new(0),
            jobs_failed: AtomicU64::new(0),
            jobs_cancelled: AtomicU64::new(0),
            jobs_expired: AtomicU64::new(0),
            chunks_sent: AtomicU64::new(0),
            chunk_failures: AtomicU64::new(0),
            payload_bytes: AtomicU64::new(0),
            wire_bytes: AtomicU64::new(0),
            holds: AtomicU64::new(0),
            held_ms: AtomicU64::new(0),
        }
    }
}

#[derive(Debug, Serialize)]
pub struct RetrievalSnapshot {
    pub state: State,
    pub hold: Option<&'static str>,
    pub jobs_done: u64,
    pub jobs_failed: u64,
    pub jobs_cancelled: u64,
    pub jobs_expired: u64,
    pub chunks_sent: u64,
    pub chunk_failures: u64,
    /// Chunk bytes the relay acknowledged.
    pub payload_bytes: u64,
    /// Everything retrieval put on the wire, estimated per request.
    pub wire_bytes: u64,
    /// Times retrieval began to hold back, and the time it spent held back (a hold in progress
    /// included).
    pub holds: u64,
    pub held_ms: u64,
}

impl RetrievalStats {
    /// What retrieval reserves in the uplink budget now.
    pub fn reserved(&self) -> u32 {
        self.reserved.load(Ordering::Relaxed)
    }

    pub fn snapshot(&self) -> RetrievalSnapshot {
        let load = |counter: &AtomicU64| counter.load(Ordering::Relaxed);
        let status = self.status();
        RetrievalSnapshot {
            state: status.state,
            hold: status.hold.map(Hold::name),
            jobs_done: load(&self.jobs_done),
            jobs_failed: load(&self.jobs_failed),
            jobs_cancelled: load(&self.jobs_cancelled),
            jobs_expired: load(&self.jobs_expired),
            chunks_sent: load(&self.chunks_sent),
            chunk_failures: load(&self.chunk_failures),
            payload_bytes: load(&self.payload_bytes),
            wire_bytes: load(&self.wire_bytes),
            holds: load(&self.holds),
            held_ms: load(&self.held_ms)
                .saturating_add(status.held_since.map_or(0, |since| millis(since.elapsed()))),
        }
    }

    fn status(&self) -> MutexGuard<'_, Status> {
        self.status.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Records what retrieval is doing; a change into or out of `Held` is counted and timed.
    fn set(&self, state: State, hold: Option<Hold>) {
        let mut status = self.status();
        let was_held = status.state == State::Held;
        let now_held = state == State::Held;
        if now_held && !was_held {
            Self::add(&self.holds, 1);
            status.held_since = Some(Instant::now());
        }
        if was_held && !now_held {
            let since = status.held_since.take();
            Self::add(
                &self.held_ms,
                since.map_or(0, |since| millis(since.elapsed())),
            );
        }
        status.state = state;
        status.hold = hold;
    }

    fn add(counter: &AtomicU64, n: u64) {
        counter.fetch_add(n, Ordering::Relaxed);
    }
}

/// How a job ended.
#[derive(Debug, PartialEq, Eq)]
enum End {
    Done,
    /// The relay no longer knows the job: cancelled, or its life ran out there.
    Cancelled,
    Expired,
    /// Given up, with the reason to tell the relay.
    Failed(String),
}

/// Works the relay's jobs until the process ends; never returns.
pub fn run(
    relay: &str,
    store: &Mutex<Store>,
    allowance: u32,
    conditions: impl Fn() -> Conditions,
    stats: &RetrievalStats,
    token: Option<&Token>,
) -> ! {
    let poller = Client::new(relay, POLL_TIMEOUT).with_token(token);
    let uploader = Client::new(relay, UPLOAD_TIMEOUT).with_token(token);
    let worker = Worker {
        uploader: &uploader,
        store,
        allowance,
        conditions: &conditions,
        stats,
    };
    loop {
        if let Err(hold) = retrieval::permit(&conditions()) {
            stats.set(State::Held, Some(hold));
            thread::sleep(HOLD_POLL);
            continue;
        }
        stats.set(State::Polling, None);
        match next(&poller, stats) {
            Ok(Some(assignment)) => worker.work(&assignment),
            Ok(None) => {}
            Err(e) => {
                eprintln!("retrieval: cannot poll the relay: {e}");
                thread::sleep(RETRY_PAUSE * 2);
            }
        }
    }
}

fn next(poller: &Client, stats: &RetrievalStats) -> Result<Option<Assignment>, ClientError> {
    let wire = retrieval::request_wire_bytes(0);
    RetrievalStats::add(&stats.wire_bytes, u64::from(wire));
    let answer = poller.send_ok("GET", "/jobs/next", &[], &[])?;
    if answer.status == 204 {
        return Ok(None);
    }
    serde_json::from_slice(&answer.body)
        .map(Some)
        .map_err(|e| ClientError::Malformed(e.to_string()))
}

struct Worker<'a, C> {
    uploader: &'a Client,
    store: &'a Mutex<Store>,
    allowance: u32,
    conditions: &'a C,
    stats: &'a RetrievalStats,
}

impl<C: Fn() -> Conditions> Worker<'_, C> {
    fn store(&self) -> MutexGuard<'_, Store> {
        self.store.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn work(&self, assignment: &Assignment) {
        let started = Instant::now();
        let deadline = started + Duration::from_millis(assignment.expires_in_ms);
        self.stats.set(State::Working, None);
        self.stats.reserved.store(self.allowance, Ordering::Relaxed);
        let end = match assignment.request.kind {
            Kind::Outage => self.list(assignment, deadline),
            Kind::Snapshot | Kind::Crop => self.send_picture(assignment, started, deadline),
        };
        let end = end.unwrap_or_else(End::Failed);
        self.stats.reserved.store(0, Ordering::Relaxed);
        self.finish(assignment, end);
    }

    /// The picture for the job, uploaded; the snapshot is released once the relay has it.
    fn send_picture(
        &self,
        assignment: &Assignment,
        started: Instant,
        deadline: Instant,
    ) -> Result<End, String> {
        let prepared = self.prepare(&assignment.request, started)?;
        let end = self.upload(assignment, &prepared, deadline)?;
        if end == End::Done {
            self.store().release(prepared.entry);
        }
        Ok(end)
    }

    /// Answers an `Outage` job with the snapshots taken out of reach since the time asked for.
    fn list(&self, assignment: &Assignment, deadline: Instant) -> Result<End, String> {
        assignment.request.validate()?;
        let snapshots = self
            .store()
            .outage_from(assignment.request.at_capture_ms, MAX_LISTED)
            .into_iter()
            .map(|e| Listed {
                capture_ms: e.capture_ms,
                frame_id: e.frame_id,
                position: e.position,
            })
            .collect();
        let body = serde_json::to_vec(&Listing { snapshots }).map_err(|e| e.to_string())?;
        let cost = retrieval::request_wire_bytes(body.len());
        let mut bucket = TokenBucket::new(self.allowance);
        if !self.wait_to_send(&mut bucket, cost, deadline) {
            return Ok(End::Expired);
        }
        RetrievalStats::add(&self.stats.wire_bytes, u64::from(cost));
        let path = format!("/jobs/{}/listing", assignment.job_id);
        match self.uploader.send("POST", &path, &[], &body) {
            Ok(answer) if answer.status == 204 => Ok(End::Done),
            Ok(answer) if matches!(answer.status, 404 | 409) => Ok(End::Cancelled),
            other => Err(format!(
                "the relay refused the listing: {}",
                describe(&other)
            )),
        }
    }

    fn finish(&self, assignment: &Assignment, end: End) {
        let stats = self.stats;
        match end {
            End::Done => RetrievalStats::add(&stats.jobs_done, 1),
            End::Cancelled => RetrievalStats::add(&stats.jobs_cancelled, 1),
            End::Expired => RetrievalStats::add(&stats.jobs_expired, 1),
            End::Failed(reason) => {
                RetrievalStats::add(&stats.jobs_failed, 1);
                eprintln!("retrieval: job {} failed: {reason}", assignment.job_id);
                self.report_failure(assignment, &reason);
            }
        }
    }

    fn report_failure(&self, assignment: &Assignment, reason: &str) {
        let reason: String = reason.chars().take(MAX_REASON).collect();
        let Ok(body) = serde_json::to_vec(&Failure { reason }) else {
            return;
        };
        let path = format!("/jobs/{}/fail", assignment.job_id);
        let _ = self.uploader.send("POST", &path, &[], &body);
    }

    /// The object for `request`: the retained snapshot as stored, or a crop of it.
    fn prepare(&self, request: &JobRequest, now: Instant) -> Result<Prepared, String> {
        request.validate()?;
        let (entry, path) = {
            let mut store = self.store();
            let entry = store
                .nearest_at_or_before(request.at_capture_ms)
                .ok_or("no retained snapshot at or before that time")?;
            store.pin(entry, now + PIN_FOR);
            (entry, store.path(entry))
        };
        let stored = fs::read(&path).map_err(|e| format!("cannot read the snapshot: {e}"))?;
        let Cropped { jpeg, source, rect } =
            crop::cut(stored, request.crop).map_err(|e| e.to_string())?;
        if jpeg.len() > usize::try_from(request.max_bytes).unwrap_or(usize::MAX) {
            return Err(format!(
                "the object is {} bytes, over max_bytes {}",
                jpeg.len(),
                request.max_bytes
            ));
        }
        let meta = Meta {
            capture_ms: entry.capture_ms,
            frame_id: entry.frame_id,
            source_width: source.0,
            source_height: source.1,
            crop: rect,
            position: entry.position,
        };
        Ok(Prepared { jpeg, meta, entry })
    }

    fn upload(
        &self,
        assignment: &Assignment,
        prepared: &Prepared,
        deadline: Instant,
    ) -> Result<End, String> {
        let size = retrieval::chunk_size(self.allowance);
        let mut bucket = TokenBucket::new(self.allowance);
        let id = &assignment.job_id;
        let mut chunks = 0_u32;
        for (index, data) in prepared.jpeg.chunks(size).enumerate() {
            chunks += 1;
            let cost = retrieval::request_wire_bytes(data.len());
            let path = format!("/jobs/{id}/chunks/{index}");
            let hash = sha256_hex(data);
            let mut attempts = 0;
            loop {
                if !self.wait_to_send(&mut bucket, cost, deadline) {
                    return Ok(End::Expired);
                }
                attempts += 1;
                RetrievalStats::add(&self.stats.wire_bytes, u64::from(cost));
                let sent =
                    self.uploader
                        .send("PUT", &path, &[(CHUNK_HASH_HEADER, hash.as_str())], data);
                match sent {
                    Ok(answer) if answer.status == 204 => break,
                    Ok(answer) if matches!(answer.status, 404 | 409) => return Ok(End::Cancelled),
                    other => {
                        RetrievalStats::add(&self.stats.chunk_failures, 1);
                        if attempts >= ATTEMPTS {
                            return Err(format!("chunk {index} failed: {}", describe(&other)));
                        }
                        thread::sleep(RETRY_PAUSE);
                    }
                }
            }
            RetrievalStats::add(&self.stats.chunks_sent, 1);
            RetrievalStats::add(&self.stats.payload_bytes, data.len() as u64);
        }
        self.complete(assignment, prepared, chunks)
    }

    /// Blocks until the conditions allow one more chunk and the bucket holds its cost; false if the
    /// job's life ran out first.
    fn wait_to_send(&self, bucket: &mut TokenBucket, cost: u32, deadline: Instant) -> bool {
        loop {
            let now = Instant::now();
            if now >= deadline {
                return false;
            }
            match retrieval::permit(&(self.conditions)()) {
                Err(hold) => {
                    self.stats.set(State::Held, Some(hold));
                    thread::sleep(HOLD_POLL);
                }
                Ok(()) if bucket.take(now, cost, Priority::Preferred) => {
                    self.stats.set(State::Working, None);
                    return true;
                }
                Ok(()) => thread::sleep(PACE_POLL),
            }
        }
    }

    fn complete(
        &self,
        assignment: &Assignment,
        prepared: &Prepared,
        chunks: u32,
    ) -> Result<End, String> {
        let done = Complete {
            chunks,
            bytes: u32::try_from(prepared.jpeg.len()).map_err(|e| e.to_string())?,
            sha256: sha256_hex(&prepared.jpeg),
            meta: prepared.meta,
        };
        let body = serde_json::to_vec(&done).map_err(|e| e.to_string())?;
        let path = format!("/jobs/{}/complete", assignment.job_id);
        RetrievalStats::add(
            &self.stats.wire_bytes,
            u64::from(retrieval::request_wire_bytes(body.len())),
        );
        match self.uploader.send("POST", &path, &[], &body) {
            Ok(answer) if answer.status == 204 => Ok(End::Done),
            Ok(answer) if answer.status == 404 => Ok(End::Cancelled),
            other => Err(format!(
                "the relay refused the object: {}",
                describe(&other)
            )),
        }
    }
}

struct Prepared {
    jpeg: Vec<u8>,
    meta: Meta,
    entry: Entry,
}

fn millis(elapsed: Duration) -> u64 {
    u64::try_from(elapsed.as_millis()).unwrap_or(u64::MAX)
}

fn describe(result: &Result<crate::client::Answer, ClientError>) -> String {
    match result {
        Ok(answer) => format!("{} {}", answer.status, answer.text()),
        Err(e) => e.to_string(),
    }
}
