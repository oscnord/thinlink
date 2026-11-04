//! The relay's job API. Two listeners share one job table: the overlay address the vehicle polls and
//! uploads on, and the ground network address the ground asks and fetches on. Each answers only its
//! own routes. Every request needs the bearer token (`thinlink_video_core::token`; without a token only
//! loopback is served) and is then checked as hostile once read: bounded
//! URL and body sizes, strict parsing, opaque ids, bounded jobs and bytes, expiry. `tiny_http` does
//! not bound the request line, the headers or a stalled client; `docs/drone.md#retained-detail` says what that
//! costs.

use std::io::Read;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Condvar, Mutex, MutexGuard, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::json;
use tiny_http::{Header, Method, Request, Response, Server};

use crate::DetailError;
use crate::assemble::Accepted;
use crate::jobs::{Counts, JobError, Jobs, Lookup};
use crate::wire::{
    CAPTURE_HEADER, CHUNK_HASH_HEADER, CROP_HEADER, Complete, Created, FRAME_HEADER, Failure,
    JobId, JobRequest, Listing, POSITION_HEADER, SOURCE_HEADER, is_sha256_hex, position_text,
};
use thinlink_policy::retrieval::MAX_CHUNK;
use thinlink_video_core::token::{Token, admit};

/// The vehicle's poll waits this long for a job.
const LONG_POLL: Duration = Duration::from_secs(5);
const MAX_URL: usize = 200;
const MAX_JSON: usize = 4096;
/// Requests handled at once on each listener.
const WORKERS: usize = 4;

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RelayConfig {
    /// Where the vehicle polls and uploads: the relay's overlay address.
    pub vehicle: SocketAddr,
    /// Where the ground asks and fetches: the relay's `ground_net` address.
    pub ground: SocketAddr,
}

#[derive(Debug, Default)]
pub struct Relay {
    jobs: Mutex<Jobs>,
    wake: Condvar,
    polling: AtomicBool,
    requests: AtomicU64,
    rejected: AtomicU64,
    created: AtomicU64,
    completed: AtomicU64,
    failed: AtomicU64,
    cancelled: AtomicU64,
    chunks: AtomicU64,
    chunk_bytes: AtomicU64,
}

#[derive(Debug, Serialize)]
pub struct RelaySnapshot {
    pub requests: u64,
    pub rejected: u64,
    pub created: u64,
    pub completed: u64,
    pub failed: u64,
    pub cancelled: u64,
    pub chunks: u64,
    pub chunk_bytes: u64,
    pub jobs: Counts,
}

impl Relay {
    pub fn snapshot(&self) -> RelaySnapshot {
        let load = |counter: &AtomicU64| counter.load(Ordering::Relaxed);
        RelaySnapshot {
            requests: load(&self.requests),
            rejected: load(&self.rejected),
            created: load(&self.created),
            completed: load(&self.completed),
            failed: load(&self.failed),
            cancelled: load(&self.cancelled),
            chunks: load(&self.chunks),
            chunk_bytes: load(&self.chunk_bytes),
            jobs: self.jobs().counts(),
        }
    }

    fn jobs(&self) -> MutexGuard<'_, Jobs> {
        self.jobs.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Surface {
    Vehicle,
    Ground,
}

/// Binds both listeners and serves until one of them fails.
pub fn serve(config: &RelayConfig, relay: &Relay, token: &Token) -> Result<(), DetailError> {
    let bind = |addr: SocketAddr| {
        Server::http(addr).map_err(|e| DetailError::Bind {
            addr: addr.to_string(),
            reason: e.to_string(),
        })
    };
    let vehicle = bind(config.vehicle)?;
    let ground = bind(config.ground)?;
    eprintln!(
        "job API: vehicle on http://{}, ground on http://{}",
        config.vehicle, config.ground
    );
    thread::scope(|scope| {
        let listeners = [(&vehicle, Surface::Vehicle), (&ground, Surface::Ground)];
        let workers: Vec<_> = listeners
            .into_iter()
            .flat_map(|listener| std::iter::repeat_n(listener, WORKERS))
            .map(|(server, surface)| scope.spawn(move || work(server, surface, relay, token)))
            .collect();
        for worker in workers {
            let _ = worker.join();
        }
    });
    Err(DetailError::Stopped)
}

fn work(server: &Server, surface: Surface, relay: &Relay, token: &Token) {
    loop {
        let mut request = match server.recv() {
            Ok(request) => request,
            Err(e) => {
                eprintln!("job API: cannot receive: {e}");
                return;
            }
        };
        relay.requests.fetch_add(1, Ordering::Relaxed);
        let access = admit(Some(token), header(&request, "Authorization"));
        let reply = match access.refusal_status() {
            Some(status) => Reply::from(Reject::new(status, "not authorized")),
            None => route(surface, &mut request, relay).unwrap_or_else(Reply::from),
        };
        if reply.status >= 400 {
            relay.rejected.fetch_add(1, Ordering::Relaxed);
        }
        if let Err(e) = request.respond(reply.into_response()) {
            eprintln!("job API: cannot answer: {e}");
        }
    }
}

struct Reply {
    status: u16,
    content_type: &'static str,
    body: Vec<u8>,
    headers: Vec<(&'static str, String)>,
}

impl Reply {
    fn json(status: u16, value: &impl Serialize) -> Self {
        let body = serde_json::to_vec(value).unwrap_or_default();
        Self {
            status,
            content_type: "application/json",
            body,
            headers: Vec::new(),
        }
    }

    fn empty(status: u16) -> Self {
        Self {
            status,
            content_type: "application/json",
            body: Vec::new(),
            headers: Vec::new(),
        }
    }

    fn into_response(self) -> Response<std::io::Cursor<Vec<u8>>> {
        let mut response = Response::from_data(self.body).with_status_code(self.status);
        let typed = [("Content-Type", self.content_type.to_owned())];
        for (name, value) in typed.into_iter().chain(self.headers) {
            if let Ok(header) = Header::from_bytes(name.as_bytes(), value.as_bytes()) {
                response.add_header(header);
            }
        }
        response
    }
}

/// A refusal: the status and what to tell the caller.
struct Reject {
    status: u16,
    message: String,
}

impl Reject {
    fn new(status: u16, message: impl Into<String>) -> Self {
        Self {
            status,
            message: message.into(),
        }
    }

    fn not_found() -> Self {
        Self::new(404, "not found")
    }
}

impl From<Reject> for Reply {
    fn from(reject: Reject) -> Self {
        Self::json(reject.status, &json!({ "error": reject.message }))
    }
}

impl From<JobError> for Reject {
    fn from(error: JobError) -> Self {
        let status = match error {
            JobError::Invalid(_) | JobError::Chunk(_) => 400,
            JobError::Busy => 429,
            JobError::Unknown => 404,
            JobError::NotAssigned => 409,
        };
        Self::new(status, error.to_string())
    }
}

fn route(surface: Surface, request: &mut Request, relay: &Relay) -> Result<Reply, Reject> {
    if request.url().len() > MAX_URL {
        return Err(Reject::new(414, "the URL is too long"));
    }
    let method = request.method().clone();
    let url = request.url().to_owned();
    let segments: Vec<&str> = url.strip_prefix('/').unwrap_or("").split('/').collect();
    match (surface, method, segments.as_slice()) {
        (Surface::Ground, Method::Post, ["jobs"]) => create(request, relay),
        (Surface::Ground, Method::Get, ["objects", id]) => object(id, relay),
        (Surface::Ground, Method::Delete, ["jobs", id]) => cancel(id, relay),
        (Surface::Vehicle, Method::Get, ["jobs", "next"]) => next(relay),
        (Surface::Vehicle, Method::Put, ["jobs", id, "chunks", index]) => {
            chunk(request, id, index, relay)
        }
        (Surface::Vehicle, Method::Post, ["jobs", id, "complete"]) => complete(request, id, relay),
        (Surface::Vehicle, Method::Post, ["jobs", id, "listing"]) => listing(request, id, relay),
        (Surface::Vehicle, Method::Post, ["jobs", id, "fail"]) => fail(request, id, relay),
        _ => Err(Reject::not_found()),
    }
}

fn parse_id(text: &str) -> Result<JobId, Reject> {
    JobId::parse(text).ok_or_else(Reject::not_found)
}

/// The body, if it is at most `limit` bytes. A declared length over the limit is refused unread.
fn read_body(request: &mut Request, limit: usize) -> Result<Vec<u8>, Reject> {
    let too_large = || Reject::new(413, format!("the body is over {limit} bytes"));
    if request.body_length().is_some_and(|length| length > limit) {
        return Err(too_large());
    }
    let mut body = Vec::new();
    let read = request
        .as_reader()
        .take(limit as u64 + 1)
        .read_to_end(&mut body);
    read.map_err(|e| Reject::new(400, format!("cannot read the body: {e}")))?;
    if body.len() > limit {
        return Err(too_large());
    }
    Ok(body)
}

fn read_json<T: for<'de> Deserialize<'de>>(request: &mut Request) -> Result<T, Reject> {
    let body = read_body(request, MAX_JSON)?;
    serde_json::from_slice(&body).map_err(|e| Reject::new(400, format!("malformed request: {e}")))
}

fn header<'a>(request: &'a Request, name: &str) -> Option<&'a str> {
    request
        .headers()
        .iter()
        .find(|h| h.field.as_str().as_str().eq_ignore_ascii_case(name))
        .map(|h| h.value.as_str())
}

fn create(request: &mut Request, relay: &Relay) -> Result<Reply, Reject> {
    let job: JobRequest = read_json(request)?;
    let id = JobId::random().map_err(|e| Reject::new(500, format!("no random id: {e}")))?;
    relay.jobs().submit(id.clone(), job, Instant::now())?;
    relay.created.fetch_add(1, Ordering::Relaxed);
    relay.wake.notify_all();
    Ok(Reply::json(201, &Created { job_id: id }))
}

fn cancel(id: &str, relay: &Relay) -> Result<Reply, Reject> {
    relay.jobs().cancel(&parse_id(id)?, Instant::now())?;
    relay.cancelled.fetch_add(1, Ordering::Relaxed);
    Ok(Reply::empty(204))
}

fn object(id: &str, relay: &Relay) -> Result<Reply, Reject> {
    let lookup = relay.jobs().lookup(&parse_id(id)?, Instant::now());
    match lookup {
        Lookup::Unknown => Err(Reject::not_found()),
        Lookup::Failed(reason) => Err(Reject::new(410, reason)),
        Lookup::Pending { state, chunks } => Ok(Reply::json(
            202,
            &json!({ "state": state, "chunks": chunks }),
        )),
        Lookup::Listed(listing) => Ok(Reply::json(200, &*listing)),
        Lookup::Ready(object) => {
            let meta = object.meta;
            let crop = meta.crop.map_or_else(
                || "none".to_owned(),
                |r| format!("{},{},{},{}", r.x, r.y, r.w, r.h),
            );
            let mut headers = vec![
                (CAPTURE_HEADER, meta.capture_ms.to_string()),
                (FRAME_HEADER, meta.frame_id.to_string()),
                (CROP_HEADER, crop),
                (
                    SOURCE_HEADER,
                    format!("{}x{}", meta.source_width, meta.source_height),
                ),
            ];
            headers.extend(meta.position.map(|p| (POSITION_HEADER, position_text(p))));
            Ok(Reply {
                status: 200,
                content_type: "image/jpeg",
                body: object.jpeg.clone(),
                headers,
            })
        }
    }
}

/// Clears the long-poll flag when the poll ends, however it ends.
struct Polling<'a>(&'a AtomicBool);

impl Drop for Polling<'_> {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

/// The vehicle's long poll: the oldest job nobody works, or nothing after `LONG_POLL`. One poll at
/// a time, so a flood of polls cannot take every worker.
fn next(relay: &Relay) -> Result<Reply, Reject> {
    if relay.polling.swap(true, Ordering::AcqRel) {
        return Err(Reject::new(429, "a poll is waiting already"));
    }
    let _polling = Polling(&relay.polling);
    let deadline = Instant::now() + LONG_POLL;
    let mut jobs = relay.jobs();
    loop {
        let now = Instant::now();
        if let Some(assignment) = jobs.next(now) {
            return Ok(Reply::json(200, &assignment));
        }
        let left = deadline.saturating_duration_since(now);
        if left.is_zero() {
            return Ok(Reply::empty(204));
        }
        jobs = relay
            .wake
            .wait_timeout(jobs, left)
            .map_or_else(|e| e.into_inner().0, |(guard, _)| guard);
    }
}

fn chunk(request: &mut Request, id: &str, index: &str, relay: &Relay) -> Result<Reply, Reject> {
    let id = parse_id(id)?;
    let index: usize = Some(index)
        .filter(|i| i.len() <= 5 && i.bytes().all(|b| b.is_ascii_digit()))
        .and_then(|i| i.parse().ok())
        .ok_or_else(|| Reject::new(400, "the chunk index is a number of up to 5 digits"))?;
    let hash = header(request, CHUNK_HASH_HEADER)
        .filter(|hash| is_sha256_hex(hash))
        .ok_or_else(|| {
            Reject::new(
                400,
                format!("{CHUNK_HASH_HEADER} is 64 lowercase hex digits"),
            )
        })?
        .to_owned();
    let data = read_body(request, MAX_CHUNK)?;
    let accepted = relay
        .jobs()
        .chunk(&id, index, &data, &hash, Instant::now())?;
    if accepted == Accepted::Stored {
        relay.chunks.fetch_add(1, Ordering::Relaxed);
        relay
            .chunk_bytes
            .fetch_add(data.len() as u64, Ordering::Relaxed);
    }
    Ok(Reply::empty(204))
}

fn complete(request: &mut Request, id: &str, relay: &Relay) -> Result<Reply, Reject> {
    let id = parse_id(id)?;
    let done: Complete = read_json(request)?;
    relay.jobs().complete(&id, &done, Instant::now())?;
    relay.completed.fetch_add(1, Ordering::Relaxed);
    Ok(Reply::empty(204))
}

fn listing(request: &mut Request, id: &str, relay: &Relay) -> Result<Reply, Reject> {
    let id = parse_id(id)?;
    let listing: Listing = read_json(request)?;
    relay.jobs().listing(&id, listing, Instant::now())?;
    relay.completed.fetch_add(1, Ordering::Relaxed);
    Ok(Reply::empty(204))
}

fn fail(request: &mut Request, id: &str, relay: &Relay) -> Result<Reply, Reject> {
    let id = parse_id(id)?;
    let failure: Failure = read_json(request)?;
    relay.jobs().fail(&id, &failure.reason, Instant::now())?;
    relay.failed.fetch_add(1, Ordering::Relaxed);
    Ok(Reply::empty(204))
}
