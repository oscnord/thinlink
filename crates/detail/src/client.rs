//! HTTP against the job API: the transport the vehicle uploads with, and what the ground and the lab
//! ask and fetch with.

use std::time::{Duration, Instant};

use serde::Deserialize;

use sat_teleop_video_core::token::Token;

use crate::crop::Rect;
use crate::wire::{
    CAPTURE_HEADER, CROP_HEADER, Created, FRAME_HEADER, JobId, JobRequest, Listing, Meta,
    POSITION_HEADER, SOURCE_HEADER, parse_position,
};

/// The most a response may hold: a job's largest object, with room.
const MAX_RESPONSE: u64 = 2 * 1024 * 1024;
const FETCH_POLL: Duration = Duration::from_millis(500);

#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    #[error("{method} {url}: {reason}")]
    Transport {
        method: String,
        url: String,
        reason: String,
    },
    #[error("{method} {url} returned {status}: {body}")]
    Status {
        method: String,
        url: String,
        status: u16,
        body: String,
    },
    #[error("the answer is not what the job API sends: {0}")]
    Malformed(String),
    #[error("the job failed: {0}")]
    Failed(String),
    #[error("no such job")]
    Unknown,
    #[error("the object was not ready within {0:?}")]
    Timeout(Duration),
}

/// An answer, whatever its status.
#[derive(Debug)]
pub struct Answer {
    pub status: u16,
    pub headers: Vec<(String, String)>,
    pub body: Vec<u8>,
}

impl Answer {
    pub fn header(&self, name: &str) -> Option<&str> {
        self.headers
            .iter()
            .find(|(n, _)| n.eq_ignore_ascii_case(name))
            .map(|(_, v)| v.as_str())
    }

    pub fn text(&self) -> String {
        String::from_utf8_lossy(&self.body).into_owned()
    }
}

#[derive(Debug, Clone)]
pub struct Client {
    base: String,
    agent: ureq::Agent,
    token: Option<Token>,
}

impl Client {
    /// `base` such as `http://192.168.232.20:8080`; no request outlasts `timeout`.
    pub fn new(base: &str, timeout: Duration) -> Self {
        let agent = ureq::Agent::config_builder()
            .http_status_as_error(false)
            .timeout_global(Some(timeout))
            .build()
            .into();
        Self {
            base: base.trim_end_matches('/').to_owned(),
            agent,
            token: None,
        }
    }

    /// Sends `token` as the bearer token with every request.
    #[must_use]
    pub fn with_token(self, token: Option<&Token>) -> Self {
        Self {
            token: token.cloned(),
            ..self
        }
    }

    /// One request; any status is an answer. `path` is sent as given.
    pub fn send(
        &self,
        method: &str,
        path: &str,
        headers: &[(&str, &str)],
        body: &[u8],
    ) -> Result<Answer, ClientError> {
        let url = format!("{}{path}", self.base);
        let transport = |reason: String| ClientError::Transport {
            method: method.to_owned(),
            url: url.clone(),
            reason,
        };
        let mut builder = ureq::http::Request::builder().method(method).uri(&url);
        if let Some(token) = &self.token {
            builder = builder.header("Authorization", token.header_value());
        }
        let builder = headers.iter().fold(builder, |builder, (name, value)| {
            builder.header(*name, *value)
        });
        let request = builder
            .body(body.to_vec())
            .map_err(|e| transport(e.to_string()))?;
        let mut response = self
            .agent
            .run(request)
            .map_err(|e| transport(e.to_string()))?;
        let status = response.status().as_u16();
        let headers = response
            .headers()
            .iter()
            .map(|(n, v)| {
                (
                    n.to_string(),
                    String::from_utf8_lossy(v.as_bytes()).into_owned(),
                )
            })
            .collect();
        let body = response
            .body_mut()
            .with_config()
            .limit(MAX_RESPONSE)
            .read_to_vec()
            .map_err(|e| transport(e.to_string()))?;
        Ok(Answer {
            status,
            headers,
            body,
        })
    }

    /// As [`Self::send`], with a status outside 200-299 an error.
    pub fn send_ok(
        &self,
        method: &str,
        path: &str,
        headers: &[(&str, &str)],
        body: &[u8],
    ) -> Result<Answer, ClientError> {
        let answer = self.send(method, path, headers, body)?;
        if (200..300).contains(&answer.status) {
            return Ok(answer);
        }
        Err(ClientError::Status {
            method: method.to_owned(),
            url: format!("{}{path}", self.base),
            status: answer.status,
            body: answer.text(),
        })
    }

    /// Asks for a snapshot or crop (`POST /jobs`).
    pub fn request(&self, job: &JobRequest) -> Result<JobId, ClientError> {
        let body = serde_json::to_vec(job).map_err(|e| ClientError::Malformed(e.to_string()))?;
        let answer = self.send_ok("POST", "/jobs", &[], &body)?;
        let created: Created = serde_json::from_slice(&answer.body)
            .map_err(|e| ClientError::Malformed(e.to_string()))?;
        Ok(created.job_id)
    }

    /// Cancels a job and frees its slot (`DELETE /jobs/{id}`).
    pub fn cancel(&self, id: &JobId) -> Result<(), ClientError> {
        self.send_ok("DELETE", &format!("/jobs/{id}"), &[], &[])
            .map(drop)
    }

    /// The object of `id`, `None` while the vehicle has not finished it.
    pub fn poll(&self, id: &JobId) -> Result<Option<Fetched>, ClientError> {
        self.poll_with(id, Fetched::from_answer)
    }

    /// The listing of an `Outage` job, `None` while the vehicle has not answered.
    pub fn poll_listing(&self, id: &JobId) -> Result<Option<Listing>, ClientError> {
        self.poll_with(id, |answer| {
            serde_json::from_slice(&answer.body).map_err(|e| ClientError::Malformed(e.to_string()))
        })
    }

    fn poll_with<T>(
        &self,
        id: &JobId,
        parse: impl FnOnce(Answer) -> Result<T, ClientError>,
    ) -> Result<Option<T>, ClientError> {
        let answer = self.send("GET", &format!("/objects/{id}"), &[], &[])?;
        match answer.status {
            200 => parse(answer).map(Some),
            202 => Ok(None),
            404 => Err(ClientError::Unknown),
            410 => Err(ClientError::Failed(error_text(&answer))),
            status => Err(ClientError::Status {
                method: "GET".to_owned(),
                url: format!("{}/objects/{id}", self.base),
                status,
                body: answer.text(),
            }),
        }
    }

    /// Polls for the object until it is ready or `timeout` is over.
    pub fn fetch(&self, id: &JobId, timeout: Duration) -> Result<Fetched, ClientError> {
        wait_for(timeout, || self.poll(id))
    }

    /// Polls for the listing until it is ready or `timeout` is over.
    pub fn list(&self, id: &JobId, timeout: Duration) -> Result<Listing, ClientError> {
        wait_for(timeout, || self.poll_listing(id))
    }
}

fn wait_for<T>(
    timeout: Duration,
    mut poll: impl FnMut() -> Result<Option<T>, ClientError>,
) -> Result<T, ClientError> {
    let started = Instant::now();
    loop {
        if let Some(ready) = poll()? {
            return Ok(ready);
        }
        if started.elapsed() >= timeout {
            return Err(ClientError::Timeout(timeout));
        }
        std::thread::sleep(FETCH_POLL);
    }
}

fn error_text(answer: &Answer) -> String {
    #[derive(Deserialize)]
    struct Body {
        error: String,
    }
    serde_json::from_slice::<Body>(&answer.body).map_or_else(|_| answer.text(), |b| b.error)
}

/// A finished object, labelled with the frame it shows.
#[derive(Debug)]
pub struct Fetched {
    pub jpeg: Vec<u8>,
    pub meta: Meta,
}

impl Fetched {
    fn from_answer(answer: Answer) -> Result<Self, ClientError> {
        let bad = |what: &str| ClientError::Malformed(format!("bad {what} header"));
        let number = |name: &str| answer.header(name).and_then(|v| v.parse::<u64>().ok());
        let capture_ms = number(CAPTURE_HEADER).ok_or_else(|| bad(CAPTURE_HEADER))?;
        let frame_id = number(FRAME_HEADER)
            .and_then(|n| u32::try_from(n).ok())
            .ok_or_else(|| bad(FRAME_HEADER))?;
        let (source_width, source_height) = answer
            .header(SOURCE_HEADER)
            .and_then(|v| v.split_once('x'))
            .and_then(|(w, h)| Some((w.parse().ok()?, h.parse().ok()?)))
            .ok_or_else(|| bad(SOURCE_HEADER))?;
        let crop = match answer.header(CROP_HEADER) {
            Some("none") => None,
            Some(text) => Some(parse_rect(text).ok_or_else(|| bad(CROP_HEADER))?),
            None => return Err(bad(CROP_HEADER)),
        };
        let position = match answer.header(POSITION_HEADER) {
            Some(text) => Some(parse_position(text).ok_or_else(|| bad(POSITION_HEADER))?),
            None => None,
        };
        Ok(Self {
            meta: Meta {
                capture_ms,
                frame_id,
                source_width,
                source_height,
                crop,
                position,
            },
            jpeg: answer.body,
        })
    }
}

fn parse_rect(text: &str) -> Option<Rect> {
    let mut parts = text.split(',').map(|p| p.parse::<u32>().ok());
    let rect = Rect {
        x: parts.next()??,
        y: parts.next()??,
        w: parts.next()??,
        h: parts.next()??,
    };
    parts.next().is_none().then_some(rect)
}
