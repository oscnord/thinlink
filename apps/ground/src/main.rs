use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use sat_teleop_control::auth::Signer;
use sat_teleop_control::envelope::ControlLimits;
use sat_teleop_control::ground::{
    self, Camera, CameraCounts, CameraStream, ClockView, ControlSnapshot, Controller, GroundConfig,
    GroundSnapshot, GroundStats, TakeError,
};
use sat_teleop_media::metrics;
use sat_teleop_media::output::{self, FreshnessLimits, OutputConfig, OutputSnapshot, OutputStats};
use sat_teleop_policy::freshness::Thresholds;
use sat_teleop_policy::token::Token;

use crate::detail::Detail;

mod detail;

/// The adapter's `ctl_net` address in the testbed.
const ADAPTER: &str = "http://192.168.233.30:9100";
/// `take-control` waits up to 3 s for the vehicle; this leaves room.
const CLIENT_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Parser)]
#[command(
    name = "sat-teleop-ground",
    about = "Ground adapter: MAVLink between mavlink-router and the ground stations, their only command endpoint, and the video output that shows freshness"
)]
struct Cli {
    /// A file with the bearer token of the adapter's endpoints (the lease among them). It never
    /// leaves the ground: the CLI presents it only to the adapter. Without one the adapter's
    /// endpoints listen on 127.0.0.1 only.
    #[arg(long, global = true)]
    token_file: Option<PathBuf>,
    /// A file with the relay's job API token, which `detail` presents to the relay.
    #[arg(long, global = true)]
    job_token_file: Option<PathBuf>,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run the bridge, the controller and the video output, and serve GET /metrics and the lease
    /// endpoints.
    Run {
        /// Ground config, for example testbed/config/ground.toml.
        #[arg(long)]
        config: PathBuf,
        /// Operational limits, for example testbed/limits.toml.
        #[arg(long)]
        limits: PathBuf,
        /// Address for GET /metrics and the lease endpoints, for example 192.168.233.30:9100.
        #[arg(long)]
        metrics_addr: String,
    },
    /// Ask the running adapter to take the vehicle's control lease (POST /lease).
    TakeControl {
        /// Base URL of the running adapter.
        #[arg(long, default_value = ADAPTER)]
        adapter: String,
    },
    /// Ask the running adapter to give the lease back (DELETE /lease).
    ReleaseControl {
        #[arg(long, default_value = ADAPTER)]
        adapter: String,
    },
    /// Show the running adapter's lease (GET /status).
    Status {
        #[arg(long, default_value = ADAPTER)]
        adapter: String,
    },
    /// Ask the relay for retained snapshots and crops of frames captured earlier.
    #[command(subcommand)]
    Detail(Detail),
}

#[derive(Serialize)]
struct Metrics {
    #[serde(flatten)]
    mavlink: GroundSnapshot,
    control: ControlSnapshot,
    /// The clock bounds the video's freshness is judged by.
    clock: ClockView,
    /// `null` when no camera is announced.
    camera: Option<CameraCounts>,
    video: OutputSnapshot,
}

/// `testbed/config/ground.toml`: the `MAVLink` bridge and, under `[video]`, the output.
struct Config {
    ground: GroundConfig,
    video: OutputConfig,
}

/// What the ground reads from `testbed/limits.toml`.
#[derive(Deserialize)]
struct Limits {
    control: ControlLimits,
    freshness: FreshnessLimits,
}

fn main() -> Result<ExitCode> {
    let Cli {
        token_file,
        job_token_file,
        command,
    } = Cli::parse();
    let load = |file: Option<PathBuf>| file.as_deref().map(metrics::load_token).transpose();
    let token = load(token_file).context("token file")?;
    let job_token = load(job_token_file).context("job token file")?;
    match command {
        Command::Run {
            config,
            limits,
            metrics_addr,
        } => run(&config, &limits, &metrics_addr, token.as_ref()).map(|()| ExitCode::SUCCESS),
        Command::TakeControl { adapter } => request(&adapter, Method::Post, token.as_ref()),
        Command::ReleaseControl { adapter } => request(&adapter, Method::Delete, token.as_ref()),
        Command::Status { adapter } => request(&adapter, Method::Get, token.as_ref()),
        Command::Detail(detail) => {
            detail::run(detail, job_token.as_ref()).map(|()| ExitCode::SUCCESS)
        }
    }
}

fn run(config: &Path, limits: &Path, metrics_addr: &str, token: Option<&Token>) -> Result<()> {
    let Config { ground, video } = load(config)?;
    let (control_limits, thresholds) = load_limits(limits)?;
    let stations = ground::resolve(&ground.gcs).context("ground stations")?;
    let destinations = ground::resolve(&video.destinations).context("video destinations")?;
    sat_teleop_media::init().context("GStreamer")?;
    let camera = camera_stream(&video).map(|stream| Arc::new(Camera::new(stream)));
    let output_stats = Arc::new(OutputStats::default());
    let stats = Arc::new(GroundStats::new(ground.clock.clone()));
    let signer = Signer::load(&ground.control_key, &ground.vehicle_id).context("control key")?;
    let controller = Arc::new(Controller::new(
        ground.control,
        ground.vehicle,
        control_limits,
        signer,
    ));
    {
        let (output_stats, stats) = (Arc::clone(&output_stats), Arc::clone(&stats));
        thread::spawn(move || {
            let bound = || stats.clock_bound();
            output::run(&video, &destinations, thresholds, &output_stats, &bound);
        });
    }
    {
        let (stats, controller, camera) =
            (Arc::clone(&stats), Arc::clone(&controller), camera.clone());
        thread::spawn(move || {
            if let Err(e) = ground::run(&ground, &stations, &stats, &controller, camera.as_ref()) {
                eprintln!("mavlink bridge stopped: {e}");
                stats.listen.down(&e);
            }
        });
    }
    let snapshot = || Metrics {
        mavlink: stats.snapshot(),
        control: controller.snapshot(),
        clock: stats.clock_view(&controller),
        camera: camera.as_deref().map(Camera::counts),
        video: output_stats.snapshot(),
    };
    let route = |method: &str, path: &str| match (method, path) {
        ("POST", "/lease") => Some(take(&controller)),
        ("DELETE", "/lease") => Some((200, json!(controller.release()))),
        ("GET", "/status") => Some((200, json!(controller.status()))),
        _ => None,
    };
    metrics::serve_with(metrics_addr, token, snapshot, route).context("metrics endpoint stopped")
}

fn take(controller: &Controller) -> (u16, Value) {
    let error = match controller.take() {
        Ok(status) => return (200, json!(status)),
        Err(e) => e,
    };
    let code = match error {
        TakeError::NotReady => 503,
        TakeError::Send(_) => 502,
        TakeError::Denied(_) => 409,
        TakeError::NoAnswer | TakeError::NoChallenge => 504,
    };
    let body = json!({ "error": error.to_string(), "status": controller.status() });
    (code, body)
}

/// The camera's stream: the output's encoding at the configured URI, if there is one.
fn camera_stream(video: &OutputConfig) -> Option<CameraStream> {
    let rung = video.encoding;
    Some(CameraStream {
        width: rung.width,
        height: rung.height,
        fps: rung.fps,
        bitrate_kbit: rung.bitrate_kbit,
        uri: video.camera_uri.clone()?,
    })
}

fn load(path: &Path) -> Result<Config> {
    let text =
        fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let mut table: toml::Table =
        toml::from_str(&text).with_context(|| format!("{} is not TOML", path.display()))?;
    let video: OutputConfig = table
        .remove("video")
        .with_context(|| format!("{} has no [video] table", path.display()))?
        .try_into()
        .with_context(|| format!("{} has an invalid [video] table", path.display()))?;
    video
        .validate()
        .with_context(|| format!("{} has an invalid [video] table", path.display()))?;
    let ground = table
        .try_into()
        .with_context(|| format!("{} is not a ground config", path.display()))?;
    Ok(Config { ground, video })
}

/// `[control]` and `[freshness]` of the limits file, validated.
fn load_limits(path: &Path) -> Result<(ControlLimits, Thresholds)> {
    let text =
        fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let limits: Limits = toml::from_str(&text).with_context(|| {
        format!(
            "{} has no valid [control] and [freshness] tables",
            path.display()
        )
    })?;
    let control = limits
        .control
        .validate()
        .with_context(|| format!("{} has invalid control limits", path.display()))?;
    let freshness = limits
        .freshness
        .validate()
        .with_context(|| format!("{} has invalid freshness limits", path.display()))?;
    Ok((control, freshness))
}

#[derive(Clone, Copy)]
enum Method {
    Get,
    Post,
    Delete,
}

/// Sends one request to the running adapter and prints its JSON answer; fails on a non-2xx
/// status.
fn request(adapter: &str, method: Method, token: Option<&Token>) -> Result<ExitCode> {
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .http_status_as_error(false)
        .timeout_global(Some(CLIENT_TIMEOUT))
        .build()
        .into();
    let path = match method {
        Method::Get => "/status",
        Method::Post | Method::Delete => "/lease",
    };
    let url = format!("{}{path}", adapter.trim_end_matches('/'));
    let authorization = token.map(Token::header_value).unwrap_or_default();
    let response = match method {
        Method::Get => agent
            .get(&url)
            .header("Authorization", &authorization)
            .call(),
        Method::Post => agent
            .post(&url)
            .header("Authorization", &authorization)
            .send(""),
        Method::Delete => agent
            .delete(&url)
            .header("Authorization", &authorization)
            .call(),
    };
    let mut response = response.with_context(|| format!("cannot reach {url}"))?;
    let body = response
        .body_mut()
        .read_to_string()
        .with_context(|| format!("cannot read the answer from {url}"))?;
    println!("{body}");
    Ok(if response.status().is_success() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_config_loads() {
        let testbed = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testbed");
        load(&testbed.join("config/ground.toml")).unwrap();
        load(&testbed.join("../deploy/ground/ground.toml")).unwrap();
        load_limits(&testbed.join("limits.toml")).unwrap();
    }
}
