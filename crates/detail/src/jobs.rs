//! The relay's job table: jobs asked for by the ground, worked by the vehicle, and the objects they
//! produce, all bounded in number, bytes and lifetime. Time arrives as an argument.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::Serialize;

use crate::assemble::{Accepted, Assembly, ChunkError};
use crate::crop;
use crate::wire::{
    Assignment, Complete, JobId, JobRequest, Kind, Listing, MAX_REASON, Meta, is_sha256_hex,
};

/// Jobs the table holds, whatever their state.
pub const MAX_JOBS: usize = 4;
/// Bytes the jobs may hold between them, summed over their `max_bytes`.
pub const MAX_OUTSTANDING_BYTES: u64 = 2 * 1024 * 1024;

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum JobError {
    #[error("{0}")]
    Invalid(String),
    #[error("the relay holds {MAX_JOBS} jobs already, or as many bytes as it may")]
    Busy,
    #[error("no such job")]
    Unknown,
    #[error("the job is not being worked")]
    NotAssigned,
    #[error(transparent)]
    Chunk(#[from] ChunkError),
}

/// A finished object: the JPEG and what it shows.
#[derive(Debug)]
pub struct Object {
    pub jpeg: Vec<u8>,
    pub meta: Meta,
}

#[derive(Debug)]
enum State {
    Queued,
    Assigned(Assembly),
    Done(Arc<Object>),
    /// An `Outage` job's answer.
    Listed(Arc<Listing>),
    Failed(String),
}

#[derive(Debug)]
struct Job {
    request: JobRequest,
    expires_at: Instant,
    state: State,
}

/// What `GET /objects/{id}` finds.
#[derive(Debug)]
pub enum Lookup {
    Ready(Arc<Object>),
    Listed(Arc<Listing>),
    Pending { state: &'static str, chunks: usize },
    Failed(String),
    Unknown,
}

#[derive(Debug, Default)]
pub struct Jobs {
    jobs: BTreeMap<JobId, Job>,
    /// Ids in the order they were created, so the vehicle works the oldest first.
    queue: Vec<JobId>,
}

/// How many jobs are in each state, for the metrics.
#[derive(Debug, Default, Clone, Copy, Serialize)]
pub struct Counts {
    pub queued: usize,
    pub assigned: usize,
    pub done: usize,
    pub failed: usize,
}

impl Jobs {
    /// Forgets the jobs whose life is over, and their objects.
    pub fn reap(&mut self, now: Instant) {
        self.jobs.retain(|_, job| job.expires_at > now);
        let jobs = &self.jobs;
        self.queue.retain(|id| jobs.contains_key(id));
    }

    /// Adds a job for `request`, valid already.
    pub fn submit(&mut self, id: JobId, request: JobRequest, now: Instant) -> Result<(), JobError> {
        request.validate().map_err(JobError::Invalid)?;
        self.reap(now);
        let outstanding: u64 = self
            .jobs
            .values()
            .map(|job| u64::from(job.request.max_bytes))
            .sum();
        let over = outstanding + u64::from(request.max_bytes) > MAX_OUTSTANDING_BYTES;
        if self.jobs.len() >= MAX_JOBS || over {
            return Err(JobError::Busy);
        }
        let life = Duration::from_millis(u64::from(request.expires_in_ms));
        self.queue.push(id.clone());
        self.jobs.insert(
            id,
            Job {
                request,
                expires_at: now + life,
                state: State::Queued,
            },
        );
        Ok(())
    }

    /// Hands the vehicle the oldest job nobody works yet.
    pub fn next(&mut self, now: Instant) -> Option<Assignment> {
        self.reap(now);
        for id in &self.queue {
            let Some(job) = self.jobs.get_mut(id) else {
                continue;
            };
            if !matches!(job.state, State::Queued) {
                continue;
            }
            job.state = State::Assigned(Assembly::new(job.request.max_bytes));
            let left = job.expires_at.saturating_duration_since(now);
            return Some(Assignment {
                job_id: id.clone(),
                request: job.request,
                expires_in_ms: u64::try_from(left.as_millis()).unwrap_or(u64::MAX),
            });
        }
        None
    }

    fn assembly(&mut self, id: &JobId, now: Instant) -> Result<&mut Assembly, JobError> {
        self.reap(now);
        match self.jobs.get_mut(id).map(|job| &mut job.state) {
            None => Err(JobError::Unknown),
            Some(State::Assigned(assembly)) => Ok(assembly),
            Some(_) => Err(JobError::NotAssigned),
        }
    }

    pub fn chunk(
        &mut self,
        id: &JobId,
        index: usize,
        data: &[u8],
        sha256: &str,
        now: Instant,
    ) -> Result<Accepted, JobError> {
        Ok(self.assembly(id, now)?.put(index, data, sha256)?)
    }

    /// Ends the upload: the object becomes available if the chunks add up and are a JPEG of the
    /// size `done.meta` says.
    pub fn complete(&mut self, id: &JobId, done: &Complete, now: Instant) -> Result<(), JobError> {
        let invalid = |what: &str| JobError::Invalid(what.to_owned());
        if self.kind_of(id) == Some(Kind::Outage) {
            return Err(invalid("an outage job is answered with a listing"));
        }
        done.meta.validate().map_err(JobError::Invalid)?;
        if !is_sha256_hex(&done.sha256) {
            return Err(invalid("sha256 is 64 lowercase hex digits"));
        }
        let assembly = self.assembly(id, now)?;
        let chunks = usize::try_from(done.chunks).unwrap_or(usize::MAX);
        let bytes = usize::try_from(done.bytes).unwrap_or(usize::MAX);
        let checked = assembly
            .finish(chunks, bytes, &done.sha256)
            .map_err(JobError::from)
            .and_then(|jpeg| match crop::dimensions(&jpeg) {
                Some(size) if size == done.meta.size() => Ok(jpeg),
                _ => Err(invalid(
                    "the object is not a JPEG of the size its metadata says",
                )),
            });
        let (state, result) = match checked {
            Ok(jpeg) => {
                let object = Object {
                    jpeg,
                    meta: done.meta,
                };
                (State::Done(Arc::new(object)), Ok(()))
            }
            Err(e) => (State::Failed(e.to_string()), Err(e)),
        };
        if let Some(job) = self.jobs.get_mut(id) {
            job.state = state;
        }
        result
    }

    fn kind_of(&self, id: &JobId) -> Option<Kind> {
        self.jobs.get(id).map(|job| job.request.kind)
    }

    /// Ends an `Outage` job with the snapshots the vehicle lists.
    pub fn listing(&mut self, id: &JobId, listing: Listing, now: Instant) -> Result<(), JobError> {
        listing.validate().map_err(JobError::Invalid)?;
        self.assembly(id, now)?;
        if self.kind_of(id) != Some(Kind::Outage) {
            return Err(JobError::Invalid(
                "only an outage job is answered with a listing".to_owned(),
            ));
        }
        if let Some(job) = self.jobs.get_mut(id) {
            job.state = State::Listed(Arc::new(listing));
        }
        Ok(())
    }

    /// The vehicle gives up the job.
    pub fn fail(&mut self, id: &JobId, reason: &str, now: Instant) -> Result<(), JobError> {
        self.reap(now);
        let job = self.jobs.get_mut(id).ok_or(JobError::Unknown)?;
        let reason: String = reason.chars().take(MAX_REASON).collect();
        job.state = State::Failed(reason);
        Ok(())
    }

    /// Removes the job and whatever it holds; the vehicle's next chunk finds it unknown.
    pub fn cancel(&mut self, id: &JobId, now: Instant) -> Result<(), JobError> {
        self.reap(now);
        self.jobs.remove(id).ok_or(JobError::Unknown)?;
        self.queue.retain(|queued| queued != id);
        Ok(())
    }

    pub fn lookup(&mut self, id: &JobId, now: Instant) -> Lookup {
        self.reap(now);
        match self.jobs.get(id).map(|job| &job.state) {
            None => Lookup::Unknown,
            Some(State::Queued) => Lookup::Pending {
                state: "queued",
                chunks: 0,
            },
            Some(State::Assigned(assembly)) => Lookup::Pending {
                state: "uploading",
                chunks: assembly.chunks(),
            },
            Some(State::Done(object)) => Lookup::Ready(Arc::clone(object)),
            Some(State::Listed(listing)) => Lookup::Listed(Arc::clone(listing)),
            Some(State::Failed(reason)) => Lookup::Failed(reason.clone()),
        }
    }

    pub fn counts(&self) -> Counts {
        let mut counts = Counts::default();
        for job in self.jobs.values() {
            match job.state {
                State::Queued => counts.queued += 1,
                State::Assigned(_) => counts.assigned += 1,
                State::Done(_) | State::Listed(_) => counts.done += 1,
                State::Failed(_) => counts.failed += 1,
            }
        }
        counts
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::{Kind, Listed, MAX_LISTED};

    fn request(kind: Kind) -> JobRequest {
        JobRequest {
            kind,
            at_capture_ms: 0,
            crop: None,
            max_bytes: 4096,
            expires_in_ms: 60_000,
        }
    }

    fn done() -> Complete {
        Complete {
            chunks: 0,
            bytes: 0,
            sha256: "0".repeat(64),
            meta: Meta {
                capture_ms: 1,
                frame_id: 1,
                source_width: 1,
                source_height: 1,
                crop: None,
                position: None,
            },
        }
    }

    #[test]
    fn an_outage_job_ends_with_a_listing_and_a_picture_job_with_a_picture() {
        let now = Instant::now();
        let (outage, picture) = (JobId::random().unwrap(), JobId::random().unwrap());
        let mut jobs = Jobs::default();
        jobs.submit(outage.clone(), request(Kind::Outage), now)
            .unwrap();
        jobs.submit(picture.clone(), request(Kind::Snapshot), now)
            .unwrap();
        while jobs.next(now).is_some() {}
        let listed = |n: usize| Listing {
            snapshots: vec![
                Listed {
                    capture_ms: 1,
                    frame_id: 1,
                    position: None
                };
                n
            ],
        };

        assert!(jobs.listing(&picture, listed(1), now).is_err());
        assert!(jobs.complete(&outage, &done(), now).is_err());
        assert!(jobs.listing(&outage, listed(MAX_LISTED + 1), now).is_err());
        jobs.listing(&outage, listed(2), now).unwrap();
        assert!(matches!(jobs.lookup(&outage, now), Lookup::Listed(l) if l.snapshots.len() == 2));
    }
}
