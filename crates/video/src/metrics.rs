//! `GET /metrics` as JSON for the applications, on their `ctl_net` address, and the management
//! routes beside it. Every route needs the bearer token; without one the endpoint listens on loopback
//! only, whatever address it was given.

use std::fs;
use std::net::SocketAddr;
use std::os::unix::fs::MetadataExt;
use std::path::Path;

use serde::Serialize;
use serde_json::{Value, json};
use tiny_http::{Header, Method, Response, Server};

use sat_teleop_video_core::token::{Token, admit, exposed, listen_address};

use crate::MediaError;

/// The token in `path`, one line, from a file only its owner can use.
pub fn load_token(path: &Path) -> Result<Token, MediaError> {
    let unreadable = |e: std::io::Error| {
        MediaError::Config(format!("cannot read the token {}: {e}", path.display()))
    };
    let mode = fs::metadata(path).map_err(unreadable)?.mode() & 0o777;
    if exposed(mode) {
        return Err(MediaError::Config(format!(
            "the token {} can be used by others (mode {mode:o}): run chmod 600 {}",
            path.display(),
            path.display()
        )));
    }
    let text = fs::read_to_string(path).map_err(unreadable)?;
    Token::new(&text).map_err(|e| MediaError::Config(format!("{}: {e}", path.display())))
}

/// Serves `snapshot()` as JSON at `GET /metrics` until the process ends; every other request gets
/// 404. Requests are handled one at a time and their bodies are never read.
pub fn serve<T: Serialize>(
    listen: &str,
    token: Option<&Token>,
    snapshot: impl Fn() -> T,
) -> Result<(), MediaError> {
    serve_with(listen, token, snapshot, |_, _| None)
}

/// As [`serve`], with more routes: `route(method, path)` answers a request other than
/// `GET /metrics` with a status and a JSON body, or `None` for 404. A route may block; requests
/// wait their turn.
pub fn serve_with<T: Serialize>(
    listen: &str,
    token: Option<&Token>,
    snapshot: impl Fn() -> T,
    route: impl Fn(&str, &str) -> Option<(u16, Value)>,
) -> Result<(), MediaError> {
    let failed = |reason: String| MediaError::Metrics {
        addr: listen.to_owned(),
        reason,
    };
    let configured: SocketAddr = listen.parse().map_err(|e| failed(format!("{e}")))?;
    let bound = listen_address(configured, token);
    let server = Server::http(bound).map_err(|e| failed(e.to_string()))?;
    if bound == configured {
        eprintln!("metrics on http://{bound}/metrics");
    } else {
        eprintln!("metrics on http://{bound}/metrics (no token: not on {configured})");
    }
    for request in server.incoming_requests() {
        let metrics = *request.method() == Method::Get && request.url() == "/metrics";
        let authorization = request
            .headers()
            .iter()
            .find(|h| h.field.equiv("Authorization"))
            .map(|h| h.value.as_str());
        let refusal = admit(token, authorization).refusal_status();
        let (status, body) = if let Some(status) = refusal {
            (status, json!({ "error": "not authorized" }).to_string())
        } else if metrics {
            let body = serde_json::to_string(&snapshot())
                .unwrap_or_else(|e| json!({ "error": e.to_string() }).to_string());
            (200, body)
        } else {
            let (status, body) = route(request.method().as_str(), request.url())
                .unwrap_or_else(|| (404, json!({ "error": "not found" })));
            (status, body.to_string())
        };
        let mut response = Response::from_string(body).with_status_code(status);
        if let Ok(header) = Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]) {
            response.add_header(header);
        }
        if let Err(e) = request.respond(response) {
            eprintln!("cannot send metrics response: {e}");
        }
    }
    Ok(())
}
