//! What crosses the job API: requests, assignments, completions and the identifiers in them.

use std::fmt;
use std::fs::File;
use std::io::{self, Read};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use sat_teleop_policy::position::Position;

use crate::crop::{self, Crop, Rect};

/// Larger than any snapshot or crop the testbed's sources produce, and what one job may hold.
pub const MAX_JOB_BYTES: u32 = 1024 * 1024;
pub const MIN_EXPIRY_MS: u32 = 1000;
pub const MAX_EXPIRY_MS: u32 = 600_000;
/// Longest reason a vehicle gives for a failed job.
pub const MAX_REASON: usize = 200;
/// Largest side of a source picture a job may name.
pub const MAX_SIDE: u32 = 16_384;

/// An opaque job identifier: 16 random bytes from the operating system, as 32 lowercase hex
/// digits. It is the only thing a request may name; no path is ever built from anything else.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct JobId(String);

impl JobId {
    pub fn random() -> io::Result<Self> {
        let mut bytes = [0_u8; 16];
        File::open("/dev/urandom")?.read_exact(&mut bytes)?;
        Ok(Self(hex(&bytes)))
    }

    /// `None` unless `text` is exactly 32 lowercase hex digits.
    pub fn parse(text: &str) -> Option<Self> {
        let valid =
            text.len() == 32 && text.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'));
        valid.then(|| Self(text.to_owned()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl TryFrom<String> for JobId {
    type Error = &'static str;

    fn try_from(text: String) -> Result<Self, Self::Error> {
        Self::parse(&text).ok_or("a job id is 32 lowercase hex digits")
    }
}

impl From<JobId> for String {
    fn from(id: JobId) -> Self {
        id.0
    }
}

impl fmt::Display for JobId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

pub fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .flat_map(|byte| [byte >> 4, byte & 0xf])
        .filter_map(|nibble| char::from_digit(u32::from(nibble), 16))
        .collect()
}

/// Whether `text` is a SHA-256 as 64 lowercase hex digits.
pub fn is_sha256_hex(text: &str) -> bool {
    text.len() == 64 && text.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
}

pub fn sha256_hex(data: &[u8]) -> String {
    hex(&Sha256::digest(data))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    /// The retained frame as stored.
    Snapshot,
    /// A rectangle of it, encoded again.
    Crop,
    /// Not a picture: the snapshots taken out of reach from `at_capture_ms` on, with positions.
    Outage,
}

/// `POST /jobs`: the frame captured at or before `at_capture_ms`, whole or cropped.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JobRequest {
    pub kind: Kind,
    pub at_capture_ms: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crop: Option<Crop>,
    /// The most the object may weigh; a bigger one fails the job.
    pub max_bytes: u32,
    /// How long the job lives, from its creation.
    pub expires_in_ms: u32,
}

impl JobRequest {
    pub fn validate(&self) -> Result<(), String> {
        if !(1..=MAX_JOB_BYTES).contains(&self.max_bytes) {
            return Err(format!("max_bytes must be 1-{MAX_JOB_BYTES}"));
        }
        if !(MIN_EXPIRY_MS..=MAX_EXPIRY_MS).contains(&self.expires_in_ms) {
            return Err(format!(
                "expires_in_ms must be {MIN_EXPIRY_MS}-{MAX_EXPIRY_MS}"
            ));
        }
        match (self.kind, self.crop) {
            (Kind::Snapshot | Kind::Outage, None) => Ok(()),
            (Kind::Snapshot | Kind::Outage, Some(_)) => Err("only a crop takes a crop".to_owned()),
            (Kind::Crop, None) => Err("a crop needs a crop rectangle".to_owned()),
            (Kind::Crop, Some(crop)) => crop::check(crop).map_err(|e| e.to_string()),
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Created {
    pub job_id: JobId,
}

/// `GET /jobs/next`: the job the vehicle works, with what is left of its life.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Assignment {
    pub job_id: JobId,
    pub request: JobRequest,
    pub expires_in_ms: u64,
}

/// What the object is: which retained frame, and which part of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Meta {
    pub capture_ms: u64,
    pub frame_id: u32,
    pub source_width: u32,
    pub source_height: u32,
    /// The rectangle of the source, in pixels; `None` for the whole frame.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crop: Option<Rect>,
    /// Where the vehicle was when it took the frame, if the autopilot's position was fresh.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position: Option<Position>,
}

impl Meta {
    /// The size the object's JPEG must have.
    pub fn size(&self) -> (u32, u32) {
        self.crop
            .map_or((self.source_width, self.source_height), |r| (r.w, r.h))
    }

    pub fn validate(&self) -> Result<(), String> {
        let sides = [self.source_width, self.source_height];
        if sides.iter().any(|side| !(1..=MAX_SIDE).contains(side)) {
            return Err(format!("the source sides must be 1-{MAX_SIDE}"));
        }
        let Some(rect) = self.crop else {
            return Ok(());
        };
        let fits = rect.w > 0
            && rect.h > 0
            && rect
                .x
                .checked_add(rect.w)
                .is_some_and(|right| right <= self.source_width)
            && rect
                .y
                .checked_add(rect.h)
                .is_some_and(|bottom| bottom <= self.source_height);
        if fits {
            Ok(())
        } else {
            Err("the crop lies outside the source".to_owned())
        }
    }
}

/// The most snapshots one list answers with, so the answer fits a small request body.
pub const MAX_LISTED: usize = 16;

/// A snapshot taken out of reach.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Listed {
    pub capture_ms: u64,
    pub frame_id: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position: Option<Position>,
}

/// `POST /jobs/{id}/listing`: the answer to an `Outage` job, oldest first.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Listing {
    pub snapshots: Vec<Listed>,
}

impl Listing {
    pub fn validate(&self) -> Result<(), String> {
        if self.snapshots.len() > MAX_LISTED {
            return Err(format!("a listing holds at most {MAX_LISTED} snapshots"));
        }
        Ok(())
    }
}

/// `POST /jobs/{id}/complete`: the chunks sent, and the whole object's size and hash.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Complete {
    pub chunks: u32,
    pub bytes: u32,
    pub sha256: String,
    #[serde(flatten)]
    pub meta: Meta,
}

/// `POST /jobs/{id}/fail`.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Failure {
    pub reason: String,
}

pub const CHUNK_HASH_HEADER: &str = "X-Chunk-Sha256";
pub const CAPTURE_HEADER: &str = "X-Capture-Ms";
pub const FRAME_HEADER: &str = "X-Frame-Id";
pub const CROP_HEADER: &str = "X-Crop";
pub const SOURCE_HEADER: &str = "X-Source-Size";
pub const POSITION_HEADER: &str = "X-Position";

/// A position as `lat_e7,lon_e7,rel_alt_mm,heading_cdeg`, the heading `-` when unknown.
pub fn position_text(position: Position) -> String {
    let heading = position
        .heading_cdeg
        .map_or_else(|| "-".to_owned(), |h| h.to_string());
    format!(
        "{},{},{},{heading}",
        position.lat_e7, position.lon_e7, position.rel_alt_mm
    )
}

/// The inverse of [`position_text`]; `None` for anything else.
pub fn parse_position(text: &str) -> Option<Position> {
    let mut parts = text.split(',');
    let lat_e7 = parts.next()?.parse().ok()?;
    let lon_e7 = parts.next()?.parse().ok()?;
    let rel_alt_mm = parts.next()?.parse().ok()?;
    let heading_cdeg = match parts.next()? {
        "-" => None,
        heading => Some(heading.parse().ok()?),
    };
    parts.next().is_none().then_some(Position {
        lat_e7,
        lon_e7,
        rel_alt_mm,
        heading_cdeg,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_position_header_reads_back_what_was_written_and_nothing_else() {
        let position = |heading_cdeg| Position {
            lat_e7: -590_000_000,
            lon_e7: 180_000_001,
            rel_alt_mm: 12_000,
            heading_cdeg,
        };
        for heading in [Some(9000), None] {
            let text = position_text(position(heading));
            assert_eq!(parse_position(&text), Some(position(heading)), "{text}");
        }
        for text in ["", "1,2,3", "1,2,3,4,5", "1,2,x,4", "1,2,3,70000", "1,2,3,"] {
            assert_eq!(parse_position(text), None, "{text:?}");
        }
    }
}
