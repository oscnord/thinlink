//! The hostile requests `lab check retrieval` sends to the job API, and the bounds it fills.

use serde_json::json;

use sat_teleop_detail::client::{Client, ClientError};
use sat_teleop_detail::wire::{JobId, JobRequest, Kind};
use sat_teleop_video_core::token::Token;

use super::{CLIENT_TIMEOUT, Settings};
use crate::check::{self, Results, Verdict, float};
use crate::client::AgentClient;

/// A request and the statuses that refuse it.
struct Hostile {
    name: &'static str,
    surface: Surface,
    method: &'static str,
    path: String,
    headers: Vec<(&'static str, String)>,
    body: Vec<u8>,
    refused: &'static [u16],
}

#[derive(Clone, Copy)]
enum Surface {
    Ground,
    Vehicle,
}

const ID: &str = "0123456789abcdef0123456789abcdef";
const JSON_JOB: &str = r#"{"kind":"crop","at_capture_ms":1,"crop":{"x":0.1,"y":0.1,"w":0.5,"h":0.5},"max_bytes":1000,"expires_in_ms":5000}"#;

fn hostile_cases() -> Vec<Hostile> {
    let post = |name, body: &str, refused| Hostile {
        name,
        surface: Surface::Ground,
        method: "POST",
        path: "/jobs".to_owned(),
        headers: Vec::new(),
        body: body.as_bytes().to_vec(),
        refused,
    };
    let get = |name, surface, path: &str, refused| Hostile {
        name,
        surface,
        method: "GET",
        path: path.to_owned(),
        headers: Vec::new(),
        body: Vec::new(),
        refused,
    };
    let chunk = |name, path: String, hash: Option<&str>, body: Vec<u8>, refused| Hostile {
        name,
        surface: Surface::Vehicle,
        method: "PUT",
        path,
        headers: hash
            .map(|h| vec![("X-Chunk-Sha256", h.to_owned())])
            .unwrap_or_default(),
        body,
        refused,
    };
    let hash = "0".repeat(64);
    let long_path = format!("/objects/{}", "a".repeat(300));
    vec![
        post("malformed JSON", "{not json", &[400]),
        post(
            "unknown field",
            &JSON_JOB.replace("\"kind\"", "\"extra\":1,\"kind\""),
            &[400],
        ),
        post("oversize body", &" ".repeat(5000), &[413]),
        post(
            "crop right of the picture",
            &JSON_JOB.replace("\"x\":0.1", "\"x\":0.8"),
            &[400],
        ),
        post(
            "crop without area",
            &JSON_JOB.replace("\"w\":0.5", "\"w\":0.0"),
            &[400],
        ),
        post(
            "crop left of the picture",
            &JSON_JOB.replace("\"x\":0.1", "\"x\":-0.1"),
            &[400],
        ),
        post(
            "crop kind without a crop",
            r#"{"kind":"crop","at_capture_ms":1,"max_bytes":1000,"expires_in_ms":5000}"#,
            &[400],
        ),
        post(
            "snapshot kind with a crop",
            &JSON_JOB.replace("\"crop\",", "\"snapshot\","),
            &[400],
        ),
        post("no bytes allowed", &JSON_JOB.replace("1000,", "0,"), &[400]),
        post(
            "more bytes than any job holds",
            &JSON_JOB.replace("1000,", "2000000,"),
            &[400],
        ),
        post(
            "a job that expires at once",
            &JSON_JOB.replace("5000", "10"),
            &[400],
        ),
        post(
            "an unknown kind",
            &JSON_JOB.replace("\"crop\",", "\"video\","),
            &[400],
        ),
        get(
            "an unknown object",
            Surface::Ground,
            &format!("/objects/{ID}"),
            &[404],
        ),
        get(
            "path traversal in an object",
            Surface::Ground,
            "/objects/..%2f..%2f..%2fetc%2fpasswd",
            &[404],
        ),
        get(
            "dot dot in an object",
            Surface::Ground,
            "/objects/../../etc/passwd",
            &[404],
        ),
        get(
            "an id with the wrong length",
            Surface::Ground,
            "/objects/abc",
            &[404],
        ),
        get(
            "an id in capitals",
            Surface::Ground,
            &format!("/objects/{}", ID.to_uppercase()),
            &[404],
        ),
        get("a very long path", Surface::Ground, &long_path, &[414]),
        get(
            "a query string",
            Surface::Ground,
            &format!("/objects/{ID}?x=1"),
            &[404],
        ),
        Hostile {
            name: "cancel an unknown job",
            surface: Surface::Ground,
            method: "DELETE",
            path: format!("/jobs/{ID}"),
            headers: Vec::new(),
            body: Vec::new(),
            refused: &[404],
        },
        Hostile {
            name: "an upload on the ground's listener",
            surface: Surface::Ground,
            method: "PUT",
            path: format!("/jobs/{ID}/chunks/0"),
            headers: vec![("X-Chunk-Sha256", hash.clone())],
            body: b"x".to_vec(),
            refused: &[404],
        },
        Hostile {
            name: "a new job on the vehicle's listener",
            surface: Surface::Vehicle,
            method: "POST",
            path: "/jobs".to_owned(),
            headers: Vec::new(),
            body: JSON_JOB.as_bytes().to_vec(),
            refused: &[404],
        },
        get(
            "an object on the vehicle's listener",
            Surface::Vehicle,
            &format!("/objects/{ID}"),
            &[404],
        ),
        chunk(
            "a chunk of an unknown job",
            format!("/jobs/{ID}/chunks/0"),
            Some(&hash),
            b"x".to_vec(),
            &[404],
        ),
        chunk(
            "path traversal in a chunk's job",
            "/jobs/..%2f..%2fx/chunks/0".to_owned(),
            Some(&hash),
            b"x".to_vec(),
            &[404],
        ),
        chunk(
            "a chunk index that is no number",
            format!("/jobs/{ID}/chunks/abc"),
            Some(&hash),
            b"x".to_vec(),
            &[400],
        ),
        chunk(
            "a chunk index beyond any job",
            format!("/jobs/{ID}/chunks/99999999"),
            Some(&hash),
            b"x".to_vec(),
            &[400],
        ),
        chunk(
            "a chunk without its hash",
            format!("/jobs/{ID}/chunks/0"),
            None,
            b"x".to_vec(),
            &[400],
        ),
        chunk(
            "a chunk over 16 KiB",
            format!("/jobs/{ID}/chunks/0"),
            Some(&hash),
            vec![0; 20_000],
            &[413],
        ),
        Hostile {
            name: "complete an unknown job",
            surface: Surface::Vehicle,
            method: "POST",
            path: format!("/jobs/{ID}/complete"),
            headers: Vec::new(),
            body: b"{}".to_vec(),
            refused: &[400, 404],
        },
    ]
}

/// A request that is valid but for its token, to each listener, with none and with the wrong one:
/// both must be refused 401. The cases that answered anything else.
fn unauthorized(settings: &Settings) -> Vec<String> {
    let wrong = Token::new("not-the-relays-token").ok();
    let bearers = [("no token", None), ("a wrong token", wrong.as_ref())];
    let listeners = [
        ("ground", &settings.ground_api, format!("/objects/{ID}")),
        ("vehicle", &settings.vehicle_api, "/jobs/next".to_owned()),
    ];
    let mut unexpected = Vec::new();
    for (listener, base, path) in &listeners {
        for (what, token) in bearers {
            let client = Client::new(base, CLIENT_TIMEOUT).with_token(token);
            let status = client
                .send("GET", path, &[], &[])
                .map(|answer| answer.status);
            if !status.as_ref().is_ok_and(|s| *s == 401) {
                unexpected.push(format!("{listener} with {what}: {status:?}"));
            }
        }
    }
    unexpected
}

/// Sends each hostile request and holds the statuses to the refusals, then fills the job table to
/// its bounds (jobs, bytes) and checks the next job is refused. The relay must count what it
/// refused.
pub(super) fn refusals(results: &mut Results, settings: &Settings, relay: &AgentClient) {
    let ground = Client::new(&settings.ground_api, CLIENT_TIMEOUT)
        .with_token(crate::client::token_for(&settings.ground_api));
    let vehicle = Client::new(&settings.vehicle_api, CLIENT_TIMEOUT)
        .with_token(crate::client::token_for(&settings.vehicle_api));
    let before = check::get_json(relay, "/metrics");
    let mut unexpected = unauthorized(settings);
    let mut readings = Vec::new();
    let cases = hostile_cases();
    for case in &cases {
        let client = match case.surface {
            Surface::Ground => &ground,
            Surface::Vehicle => &vehicle,
        };
        let headers: Vec<(&str, &str)> =
            case.headers.iter().map(|(n, v)| (*n, v.as_str())).collect();
        let status = client
            .send(case.method, &case.path, &headers, &case.body)
            .map(|answer| answer.status);
        readings.push(json!({ "case": case.name, "status": status.as_ref().ok() }));
        if !status.as_ref().is_ok_and(|s| case.refused.contains(s)) {
            unexpected.push(format!("{}: {status:?}", case.name));
        }
    }
    let bounds = job_bounds(&ground);
    let after = check::get_json(relay, "/metrics");
    let counted = check::increase(&before, &after, "/detail/rejected");
    let refused = cases.len() as u64 + u64::from(bounds.is_ok());
    results.metrics.insert("hostile_requests", float(refused));
    results
        .metrics
        .insert("hostile_accepted", float(unexpected.len() as u64));
    let (verdict, detail) = if !unexpected.is_empty() {
        (
            Verdict::Fail,
            format!("not refused as expected: {}", unexpected.join("; ")),
        )
    } else if let Err(e) = bounds {
        (Verdict::Fail, e)
    } else if counted.is_none_or(|n| n < cases.len() as u64) {
        (
            Verdict::Fail,
            format!(
                "the relay counted {counted:?} refusals of {} requests",
                cases.len()
            ),
        )
    } else {
        (
            Verdict::Pass,
            format!(
                "{} hostile requests refused, and the job table held to its bounds",
                cases.len()
            ),
        )
    };
    results.record("hostile_requests_refused", verdict, detail);
    results.readings = json!({ "hostile": readings });
}

/// Four jobs with small limits fill the table (the vehicle may fail them at once, they still hold
/// their slots); a fifth is refused with 429. Two jobs of 1 MiB fill the bytes; a third is refused.
fn job_bounds(ground: &Client) -> Result<(), String> {
    let small = |at| JobRequest {
        kind: Kind::Snapshot,
        at_capture_ms: at,
        crop: None,
        max_bytes: 1000,
        expires_in_ms: 5000,
    };
    let big = JobRequest {
        max_bytes: 1_048_576,
        ..small(1)
    };
    let mut ids = Vec::new();
    let table = bound(ground, &mut ids, &(0..4).map(small).collect::<Vec<_>>());
    free(ground, &mut ids);
    let bytes = bound(ground, &mut ids, &[big, big]);
    free(ground, &mut ids);
    table.and(bytes)
}

/// Creates every job of `fit`, all accepted, then one more which must be refused with 429.
fn bound(ground: &Client, ids: &mut Vec<JobId>, fit: &[JobRequest]) -> Result<(), String> {
    for (index, job) in fit.iter().enumerate() {
        let id = ground
            .request(job)
            .map_err(|e| format!("job {} of {} within the bounds: {e}", index + 1, fit.len()))?;
        ids.push(id);
    }
    match fit.first().map(|job| ground.request(job)) {
        Some(Err(ClientError::Status { status: 429, .. })) => Ok(()),
        other => Err(format!(
            "the job beyond the bounds ({} held) was not refused with 429: {other:?}",
            fit.len()
        )),
    }
}

fn free(ground: &Client, ids: &mut Vec<JobId>) {
    for id in ids.drain(..) {
        let _ = ground.cancel(&id);
    }
}
