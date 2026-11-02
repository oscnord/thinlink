//! Router agent: a small JSON-over-HTTP control plane that shapes exactly two interfaces.
//! Requests are handled one at a time on the calling thread.

use std::io::Read;

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tiny_http::{Header, Request, Response, Server};

use crate::command::{Command, ExecError};
use crate::counters::{self, CountersError};
use crate::interface::Interfaces;
use crate::nft;
use crate::profile::{Profile, ProfileError, Side};
use crate::tc;
use crate::timeline::{self, TimelineRun};

pub const MAX_BODY_BYTES: usize = 64 * 1024;
const SIDES: [Side; 2] = [Side::Uplink, Side::Downlink];

#[derive(Debug, thiserror::Error)]
pub enum AgentError {
    #[error(transparent)]
    Profile(#[from] ProfileError),
    #[error(transparent)]
    Exec(#[from] ExecError),
    #[error(transparent)]
    Counters(#[from] CountersError),
    #[error("request body: {0}")]
    BadBody(String),
    #[error("tc output does not parse: {0}")]
    TcOutput(#[from] serde_json::Error),
    #[error("no profile applied")]
    NoProfile,
    #[error("the applied profile has no timeline")]
    NoTimeline,
    #[error("not found")]
    NotFound,
    #[error("method not allowed")]
    MethodNotAllowed,
    #[error("body larger than {MAX_BODY_BYTES} bytes")]
    TooLarge,
    #[error("cannot listen on {addr}: {reason}")]
    Listen { addr: String, reason: String },
}

impl AgentError {
    fn status(&self) -> u16 {
        match self {
            Self::Profile(_) | Self::BadBody(_) => 400,
            Self::NoProfile | Self::NoTimeline => 409,
            Self::NotFound => 404,
            Self::MethodNotAllowed => 405,
            Self::TooLarge => 413,
            Self::Exec(_) | Self::Counters(_) | Self::TcOutput(_) | Self::Listen { .. } => 500,
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ForwardingRequest {
    enabled: bool,
}

#[derive(Serialize)]
struct SideCounters {
    dev: String,
    qdiscs: Vec<counters::QdiscStats>,
    net: counters::NetStats,
}

pub struct Agent {
    interfaces: Interfaces,
    profile: Option<Profile>,
    timeline: Option<TimelineRun>,
}

impl Agent {
    pub fn new(interfaces: Interfaces) -> Self {
        Self {
            interfaces,
            profile: None,
            timeline: None,
        }
    }

    pub fn handle(&mut self, method: &str, path: &str, body: &str) -> Result<Value, AgentError> {
        match (method, path) {
            ("POST", "/profile") => self.apply_profile(body),
            ("POST", "/timeline/start") => self.start_timeline(),
            ("GET", "/timeline") => Ok(self.timeline_state()),
            ("GET", "/counters") => self.counters(),
            ("POST", "/forwarding") => Self::set_forwarding(body),
            ("POST", "/reset") => self.reset(),
            (
                _,
                "/profile" | "/timeline/start" | "/timeline" | "/counters" | "/forwarding"
                | "/reset",
            ) => Err(AgentError::MethodNotAllowed),
            _ => Err(AgentError::NotFound),
        }
    }

    fn apply_profile(&mut self, body: &str) -> Result<Value, AgentError> {
        let profile = Profile::parse(body)?;
        self.stop_timeline();
        let applied = self.apply_baseline(&profile)?;
        let summary = json!({
            "profile": profile.meta.name,
            "provenance": profile.meta.provenance,
            "applied": applied,
            "tc": self.tc_state()?,
        });
        self.profile = Some(profile);
        Ok(summary)
    }

    /// Rebuilds both qdisc chains. The router is half-configured if this fails, so no profile is recorded.
    fn apply_baseline(&mut self, profile: &Profile) -> Result<Vec<String>, AgentError> {
        self.profile = None;
        let mut applied = Vec::new();
        for side in SIDES {
            let dev = self.interfaces.dev(side);
            for cmd in tc::setup(dev, profile.link(side), profile.meta.seed) {
                cmd.run()?;
                applied.push(cmd.display());
            }
        }
        Ok(applied)
    }

    fn start_timeline(&mut self) -> Result<Value, AgentError> {
        let profile = self.profile.clone().ok_or(AgentError::NoProfile)?;
        if profile.timeline.is_empty() {
            return Err(AgentError::NoTimeline);
        }
        self.stop_timeline();
        self.apply_baseline(&profile)?;
        let steps = timeline::plan(&profile, &self.interfaces);
        let count = steps.len();
        self.profile = Some(profile);
        self.timeline = Some(TimelineRun::start(steps));
        Ok(json!({ "started": true, "steps": count }))
    }

    fn timeline_state(&self) -> Value {
        match &self.timeline {
            None => json!({ "running": false, "transitions": [] }),
            Some(run) => json!({ "running": run.running(), "transitions": run.transitions() }),
        }
    }

    fn stop_timeline(&mut self) {
        if let Some(run) = self.timeline.take() {
            run.stop();
        }
    }

    fn counters(&self) -> Result<Value, AgentError> {
        let side = |side: Side| -> Result<SideCounters, AgentError> {
            let dev = self.interfaces.dev(side);
            let stats = tc::show_qdisc_stats(dev).run()?;
            Ok(SideCounters {
                dev: dev.to_owned(),
                qdiscs: counters::parse_qdisc_stats(&stats)?,
                net: counters::read_net_stats(dev)?,
            })
        };
        Ok(json!({ "uplink": side(Side::Uplink)?, "downlink": side(Side::Downlink)? }))
    }

    fn set_forwarding(body: &str) -> Result<Value, AgentError> {
        let request: ForwardingRequest =
            serde_json::from_str(body).map_err(|e| AgentError::BadBody(e.to_string()))?;
        let cmds = nft::forwarding(request.enabled);
        for cmd in &cmds {
            cmd.run()?;
        }
        let applied: Vec<String> = cmds.iter().map(Command::display).collect();
        Ok(json!({ "forwarding": request.enabled, "applied": applied }))
    }

    fn reset(&mut self) -> Result<Value, AgentError> {
        self.stop_timeline();
        self.profile = None;
        for side in SIDES {
            tc::teardown(self.interfaces.dev(side)).run()?;
        }
        for cmd in nft::forwarding(true) {
            cmd.run()?;
        }
        Ok(json!({ "reset": true, "tc": self.tc_state()? }))
    }

    fn tc_state(&self) -> Result<Value, AgentError> {
        let side = |side: Side| -> Result<Value, AgentError> {
            let dev = self.interfaces.dev(side);
            let qdisc: Value = serde_json::from_str(&tc::show_qdisc(dev).run()?)?;
            let class: Value = serde_json::from_str(&tc::show_class(dev).run()?)?;
            Ok(json!({ "dev": dev, "qdisc": qdisc, "class": class }))
        };
        Ok(json!({ "uplink": side(Side::Uplink)?, "downlink": side(Side::Downlink)? }))
    }
}

/// Serves until the process ends.
pub fn serve(listen: &str, mut agent: Agent) -> Result<(), AgentError> {
    let server = Server::http(listen).map_err(|e| AgentError::Listen {
        addr: listen.to_owned(),
        reason: e.to_string(),
    })?;
    eprintln!("router agent listening on {listen}");
    for request in server.incoming_requests() {
        respond(&mut agent, request);
    }
    Ok(())
}

fn respond(agent: &mut Agent, mut request: Request) {
    let method = request.method().to_string();
    let path = request
        .url()
        .split('?')
        .next()
        .unwrap_or_default()
        .to_owned();
    let (status, body) =
        match read_body(&mut request).and_then(|b| agent.handle(&method, &path, &b)) {
            Ok(value) => (200, value),
            Err(e) => (e.status(), json!({ "error": e.to_string() })),
        };
    eprintln!("{method} {path} -> {status}");
    let mut response = Response::from_string(body.to_string()).with_status_code(status);
    if let Ok(header) = Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]) {
        response.add_header(header);
    }
    if let Err(e) = request.respond(response) {
        eprintln!("cannot send response: {e}");
    }
}

fn read_body(request: &mut Request) -> Result<String, AgentError> {
    if request.body_length().is_some_and(|n| n > MAX_BODY_BYTES) {
        return Err(AgentError::TooLarge);
    }
    let mut body = String::new();
    request
        .as_reader()
        .take(MAX_BODY_BYTES as u64 + 1)
        .read_to_string(&mut body)
        .map_err(|e| AgentError::BadBody(e.to_string()))?;
    if body.len() > MAX_BODY_BYTES {
        return Err(AgentError::TooLarge);
    }
    Ok(body)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn agent() -> Agent {
        Agent::new(Interfaces::resolve("up0", "down0").unwrap())
    }

    fn status(method: &str, path: &str, body: &str) -> u16 {
        agent().handle(method, path, body).unwrap_err().status()
    }

    // The interfaces do not exist, so a request that reached `tc` or `nft` would be a 500, not these statuses.
    #[test]
    fn routing_and_rejections_never_reach_the_router() {
        let cases: &[(&str, &str, &str, u16)] = &[
            ("GET", "/nope", "", 404),
            ("GET", "/profile", "", 405),
            ("POST", "/counters", "", 405),
            ("POST", "/profile", "not a profile =", 400),
            ("POST", "/profile", "[profile]\nname = \"x\"\n", 400),
            ("POST", "/forwarding", "not json", 400),
            ("POST", "/forwarding", "{\"enabled\": 1}", 400),
            (
                "POST",
                "/forwarding",
                "{\"enabled\": false, \"extra\": 1}",
                400,
            ),
            ("POST", "/timeline/start", "", 409),
        ];
        for (method, path, body, expected) in cases {
            assert_eq!(
                status(method, path, body),
                *expected,
                "{method} {path} {body}"
            );
        }
    }

    #[test]
    fn timeline_state_is_idle_without_a_run() {
        let state = agent().handle("GET", "/timeline", "").unwrap();
        assert_eq!(state, json!({ "running": false, "transitions": [] }));
    }
}
