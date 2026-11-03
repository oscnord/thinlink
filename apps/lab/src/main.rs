use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use clap::{Args, Parser, Subcommand, ValueEnum};

use sat_teleop_control::auth::{self, VehicleId};
use sat_teleop_detail::store::StoreConfig;
use sat_teleop_lab::agent::{self, Agent};
use sat_teleop_lab::budget::Budgets;
use sat_teleop_lab::check::{self, Summary, Verdict};
use sat_teleop_lab::client::{self, AgentClient};
use sat_teleop_lab::interface::Interfaces;
use sat_teleop_lab::profile::{self, Profile};
use sat_teleop_lab::{
    adaptation, boot, control, ingest, mavlink, qgc_status, recording, regional, retrieval,
    sightings, skeleton,
};
use sat_teleop_video_core::token::Token;

const PROFILES_DIR: &str = "testbed/profiles";
const BUDGETS_PATH: &str = "perf/budgets.toml";
const LIMITS_PATH: &str = "testbed/limits.toml";
const VEHICLE_CONFIG_PATH: &str = "testbed/config/vehicle.toml";
const RESULTS_DIR: &str = "results";

#[derive(Parser)]
#[command(name = "lab", about = "sat-teleop lab runner")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run the router agent that applies impairment profiles with tc.
    Router {
        /// Address for the agent HTTP API, for example 192.168.233.254:9000.
        #[arg(long)]
        listen: String,
        /// Uplink egress interface (toward `ground_net`), or `auto` to find it by address.
        #[arg(long, default_value = "auto")]
        uplink_if: String,
        /// Downlink egress interface (toward `vehicle_net`), or `auto` to find it by address.
        #[arg(long, default_value = "auto")]
        downlink_if: String,
    },
    /// Apply an impairment profile through a running router agent.
    Impair {
        /// Base URL of the router agent, for example `http://192.168.233.254:9000`.
        #[arg(long)]
        agent: String,
        #[arg(long)]
        profile: PathBuf,
        /// Also run the profile's timeline after applying it.
        #[arg(long)]
        start_timeline: bool,
    },
    /// Write a controller key pair, `ctl-<name>.key` and `ctl-<name>.pub`, for each name into a
    /// directory, keeping keys that exist.
    Keygen {
        #[arg(long, default_value = ".")]
        dir: PathBuf,
        #[arg(required = true)]
        names: Vec<String>,
    },
    /// Run a scenario and write results/<run-id>/. Exit 0 pass, 1 fail, 2 incomplete.
    Check(Box<CheckArgs>),
}

#[derive(Args)]
struct CheckArgs {
    scenario: Scenario,
    /// Profile name, read from testbed/profiles/<name>.toml under the working directory.
    #[arg(long)]
    profile: String,
    /// Base URL of the router agent.
    #[arg(long, default_value = "http://192.168.233.254:9000")]
    agent: String,
    /// The directory with the bearer tokens, one file per service: `vehicle.token` (the vehicle's
    /// endpoints), `ground.token` (the ground adapter's) and `job.token` (the relay's, metrics and
    /// job API).
    #[arg(long, default_value = "/secrets")]
    token_dir: PathBuf,
    /// Revision recorded in summary.json, for example from `git describe --always --dirty`.
    #[arg(long)]
    git_rev: Option<String>,
    /// skeleton, adaptation: RTSP URL the headless receiver plays.
    #[arg(long, default_value = "rtsp://192.168.232.20:8554/live")]
    url: String,
    /// skeleton, qgc-status, adaptation, ingest: seconds between the ladder settling and measuring.
    #[arg(long, default_value_t = 5, value_parser = clap::value_parser!(u64).range(0..=600))]
    warmup_s: u64,
    /// skeleton, mavlink, qgc-status, adaptation, ingest: seconds of measurement (qgc-status: the healthy
    /// window; adaptation: the observation under a profile without a timeline).
    #[arg(long, default_value_t = 30, value_parser = clap::value_parser!(u64).range(1..=3600))]
    duration_s: u64,
    /// skeleton, mavlink, qgc-status, adaptation, ingest, sightings: base URL of the vehicle's metrics endpoint.
    #[arg(long, default_value = "http://192.168.233.10:9100")]
    vehicle_metrics: String,
    /// skeleton: base URL of the relay's metrics endpoint.
    #[arg(long, default_value = "http://192.168.233.20:9100")]
    relay_metrics: String,
    /// mavlink, control-freshness, sim-flight, sim-mission, sim-landing, qgc-status, sightings: where the ground adapter sends this ground station
    /// its telemetry.
    #[arg(long, default_value = "192.168.232.40:14552")]
    gcs_listen: SocketAddr,
    /// skeleton, mavlink, control-freshness, qgc-status: base URL of the ground adapter's metrics and lease
    /// endpoints.
    #[arg(long, default_value = "http://192.168.233.30:9100")]
    ground_metrics: String,
    /// control-freshness, sim-flight, sim-mission, sim-landing: where mavlink-router sends this host SITL's telemetry, on the overlay.
    #[arg(long, default_value = "10.77.0.4:14553")]
    observer_listen: SocketAddr,
    /// control-freshness: the key this host signs its envelopes with as a second controller.
    #[arg(long, default_value = "/secrets/ctl-checks.key")]
    controller_key: PathBuf,
    /// control-freshness: the vehicle's name (`vehicle_id` of its config).
    #[arg(long, default_value = "testbed")]
    vehicle_id: VehicleId,
    /// control-freshness: this host's socket as a second controller, on the overlay.
    #[arg(long, default_value = "10.77.0.4:0")]
    controller: SocketAddr,
    /// control-freshness: the vehicle's control endpoint.
    #[arg(long, default_value = "10.77.0.2:7700")]
    vehicle_control: SocketAddr,
    /// qgc-status, sim-flight, regional: where the ground adapter sends this host its video output.
    #[arg(long, default_value = "192.168.232.40:5602")]
    output_listen: SocketAddr,
    /// regional: the ladder rung (counted from the best, 0) the first two windows run at; the third
    /// is the rung below.
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u64).range(0..=3))]
    rung: u64,
    /// recording, retrieval, regional: the vehicle's snapshot directory as this host mounts it.
    #[arg(long, default_value = "/snapshots")]
    snapshot_dir: PathBuf,
    /// retrieval, sightings: the relay's job API for the ground, and for the vehicle on the overlay.
    #[arg(long, default_value = "http://192.168.232.20:8080")]
    ground_api: String,
    #[arg(long, default_value = "http://10.77.0.1:8080")]
    vehicle_api: String,
}

#[derive(Clone, Copy, ValueEnum)]
enum Scenario {
    Boot,
    Skeleton,
    Mavlink,
    ControlFreshness,
    RealMode,
    SimFlight,
    SimMission,
    Follow,
    SimLanding,
    QgcStatus,
    Adaptation,
    Recording,
    Retrieval,
    Ingest,
    Regional,
    Sightings,
}

impl Scenario {
    fn name(self) -> &'static str {
        match self {
            Self::Boot => "boot",
            Self::Skeleton => "skeleton",
            Self::Mavlink => "mavlink",
            Self::ControlFreshness => "control-freshness",
            Self::RealMode => "real-mode",
            Self::SimFlight => "sim-flight",
            Self::SimMission => "sim-mission",
            Self::Follow => "follow",
            Self::SimLanding => "sim-landing",
            Self::QgcStatus => "qgc-status",
            Self::Adaptation => "adaptation",
            Self::Recording => "recording",
            Self::Retrieval => "retrieval",
            Self::Ingest => "ingest",
            Self::Regional => "regional",
            Self::Sightings => "sightings",
        }
    }
}

fn main() -> Result<ExitCode> {
    match Cli::parse().command {
        Command::Router {
            listen,
            uplink_if,
            downlink_if,
        } => router(&listen, &uplink_if, &downlink_if).map(|()| ExitCode::SUCCESS),
        Command::Impair {
            agent,
            profile,
            start_timeline,
        } => impair(&agent, &profile, start_timeline).map(|()| ExitCode::SUCCESS),
        Command::Keygen { dir, names } => keygen(&dir, &names).map(|()| ExitCode::SUCCESS),
        Command::Check(args) => Ok(check(&args)),
    }
}

fn keygen(dir: &Path, names: &[String]) -> Result<()> {
    for name in names {
        let created = auth::generate(dir, name).with_context(|| format!("key for {name}"))?;
        println!(
            "controller key {name}: {}",
            if created { "made" } else { "kept" }
        );
    }
    Ok(())
}

fn router(listen: &str, uplink_if: &str, downlink_if: &str) -> Result<()> {
    let interfaces =
        Interfaces::resolve(uplink_if, downlink_if).context("cannot select router interfaces")?;
    agent::serve(listen, Agent::new(interfaces)).context("router agent stopped")
}

fn impair(agent: &str, profile: &Path, start_timeline: bool) -> Result<()> {
    let text = fs::read_to_string(profile)
        .with_context(|| format!("cannot read {}", profile.display()))?;
    Profile::parse(&text)
        .with_context(|| format!("{} is not a valid profile", profile.display()))?;
    let client = AgentClient::new(agent);
    println!("{}", client.post("/profile", &text)?);
    if start_timeline {
        println!("{}", client.post("/timeline/start", "")?);
    }
    Ok(())
}

/// A check that cannot run or cannot record its results is incomplete, never a pass.
fn check(args: &CheckArgs) -> ExitCode {
    let verdict = run_check(args).unwrap_or_else(|e| {
        println!(
            "{} {}: incomplete: {e:#}",
            args.scenario.name(),
            args.profile
        );
        Verdict::Incomplete
    });
    ExitCode::from(verdict.exit_code())
}

fn run_check(args: &CheckArgs) -> Result<Verdict> {
    let (scenario, profile_name) = (args.scenario, args.profile.as_str());
    client::use_tokens(load_tokens(args)?);
    let (profile, text) = load_profile(profile_name)?;
    let run_id = format!("{}-{}-{profile_name}", unix_ms()?, scenario.name());
    let agent = AgentClient::new(&args.agent);
    let results = match scenario {
        Scenario::Boot => boot::run(&agent, &profile, &text),
        Scenario::Skeleton => {
            let budgets = load_budgets()?;
            let settings = skeleton::Settings {
                url: args.url.clone(),
                warmup: Duration::from_secs(args.warmup_s),
                duration: Duration::from_secs(args.duration_s),
                vehicle_metrics: args.vehicle_metrics.clone(),
                relay_metrics: args.relay_metrics.clone(),
                ground_metrics: args.ground_metrics.clone(),
            };
            skeleton::run(
                &agent,
                &text,
                &settings,
                budgets.get("skeleton", profile_name),
            )
        }
        Scenario::Mavlink => {
            let budgets = load_budgets()?;
            let settings = mavlink::Settings {
                listen: args.gcs_listen,
                duration: Duration::from_secs(args.duration_s),
                vehicle_metrics: args.vehicle_metrics.clone(),
                ground_metrics: args.ground_metrics.clone(),
            };
            mavlink::run(
                &agent,
                &text,
                &settings,
                budgets.get("mavlink", profile_name),
            )
        }
        Scenario::ControlFreshness => {
            let budgets = load_budgets()?;
            let settings = control_settings(args)?;
            control::run(
                &agent,
                &profile,
                &text,
                &settings,
                budgets.get("control-freshness", profile_name),
            )
        }
        Scenario::RealMode => {
            let settings = control_settings(args)?;
            control::run_real_mode(&agent, &text, &settings)
        }
        Scenario::SimFlight => {
            let budgets = load_budgets()?;
            control::sim_flight::run(
                &agent,
                &text,
                &sim_settings(args)?,
                budgets.get("sim-flight", profile_name),
            )
        }
        Scenario::SimMission => {
            control::sim_flight::mission::run(&agent, &text, &sim_settings(args)?)
        }
        Scenario::Follow => control::sim_flight::follow::run(&agent, &text, &sim_settings(args)?),
        Scenario::SimLanding => {
            control::sim_flight::landing::run(&agent, &text, &sim_settings(args)?)
        }
        Scenario::QgcStatus => {
            let budgets = load_budgets()?;
            let limits = fs::read_to_string(LIMITS_PATH)
                .with_context(|| format!("cannot read {LIMITS_PATH}"))?;
            let settings = qgc_status::Settings {
                listen: args.output_listen,
                gcs_listen: args.gcs_listen,
                warmup: Duration::from_secs(args.warmup_s),
                duration: Duration::from_secs(args.duration_s),
                vehicle_metrics: args.vehicle_metrics.clone(),
                ground_metrics: args.ground_metrics.clone(),
                freshness: qgc_status::freshness(&limits)
                    .with_context(|| format!("{LIMITS_PATH} has no valid [freshness] table"))?,
            };
            qgc_status::run(
                &agent,
                &profile,
                &text,
                &settings,
                budgets.get("qgc-status", profile_name),
            )
        }
        Scenario::Adaptation => {
            let budgets = load_budgets()?;
            let settings = adaptation::Settings {
                url: args.url.clone(),
                warmup: Duration::from_secs(args.warmup_s),
                duration: Duration::from_secs(args.duration_s),
                vehicle_metrics: args.vehicle_metrics.clone(),
            };
            adaptation::run(
                &agent,
                &profile,
                &text,
                &settings,
                budgets.get("adaptation", profile_name),
            )
        }
        Scenario::Recording => {
            let budgets = load_budgets()?;
            let settings = recording::Settings {
                url: args.url.clone(),
                warmup: Duration::from_secs(args.warmup_s),
                vehicle_metrics: args.vehicle_metrics.clone(),
                dir: args.snapshot_dir.clone(),
                store: vehicle_store()?,
            };
            recording::run(
                &agent,
                &text,
                &settings,
                budgets.get("recording", profile_name),
            )
        }
        Scenario::Ingest => ingest::run(
            &agent,
            &text,
            &ingest::Settings {
                warmup: Duration::from_secs(args.warmup_s),
                duration: Duration::from_secs(args.duration_s),
                vehicle_metrics: args.vehicle_metrics.clone(),
            },
        ),
        Scenario::Regional => {
            let budgets = load_budgets()?;
            let settings = regional::Settings {
                output_listen: args.output_listen,
                gcs_listen: args.gcs_listen,
                rung: usize::try_from(args.rung).unwrap_or(0),
                snapshot_dir: args.snapshot_dir.clone(),
                warmup: Duration::from_secs(args.warmup_s),
                duration: Duration::from_secs(args.duration_s),
                vehicle_metrics: args.vehicle_metrics.clone(),
                ground_metrics: args.ground_metrics.clone(),
            };
            regional::run(&agent, &text, &settings, &budgets, profile_name)
        }
        Scenario::Sightings => sightings::run(
            &agent,
            &profile,
            &text,
            &sightings::Settings {
                gcs_listen: args.gcs_listen,
                vehicle_metrics: args.vehicle_metrics.clone(),
                ground_api: args.ground_api.clone(),
            },
        ),
        Scenario::Retrieval => {
            let budgets = load_budgets()?;
            let settings = retrieval::Settings {
                url: args.url.clone(),
                warmup: Duration::from_secs(args.warmup_s),
                duration: Duration::from_secs(args.duration_s),
                vehicle_metrics: args.vehicle_metrics.clone(),
                relay_metrics: args.relay_metrics.clone(),
                ground_api: args.ground_api.clone(),
                vehicle_api: args.vehicle_api.clone(),
                dir: args.snapshot_dir.clone(),
                allowance: retrieval::allowance(
                    &fs::read_to_string(LIMITS_PATH)
                        .with_context(|| format!("cannot read {LIMITS_PATH}"))?,
                )
                .map_err(anyhow::Error::msg)
                .with_context(|| format!("{LIMITS_PATH} has no valid [retrieval] table"))?,
            };
            retrieval::run(
                &agent,
                &profile,
                &text,
                &settings,
                budgets.get("retrieval", profile_name),
            )
        }
    };
    let verdict = results.verdict();
    let summary = Summary {
        run_id: &run_id,
        scenario: scenario.name(),
        profile: profile_name,
        verdict,
        git_revision: args.git_rev.as_deref(),
        vehicle_at_end: check::vehicle_at_end(&AgentClient::new(&args.vehicle_metrics)),
        results: &results,
    };
    let dir = Path::new(RESULTS_DIR).join(&run_id);
    check::write(&dir, &summary).with_context(|| format!("cannot write {}", dir.display()))?;
    let passed = results
        .assertions
        .iter()
        .filter(|a| a.verdict == Verdict::Pass)
        .count();
    println!(
        "{} {profile_name}: {verdict} ({passed}/{} assertions passed) {}",
        scenario.name(),
        results.assertions.len(),
        dir.display()
    );
    Ok(verdict)
}

fn control_settings(args: &CheckArgs) -> Result<control::Settings> {
    let limits =
        fs::read_to_string(LIMITS_PATH).with_context(|| format!("cannot read {LIMITS_PATH}"))?;
    Ok(control::Settings {
        gcs_listen: args.gcs_listen,
        observer_listen: args.observer_listen,
        controller: args.controller,
        controller_key: args.controller_key.clone(),
        vehicle_id: args.vehicle_id.clone(),
        vehicle: args.vehicle_control,
        vehicle_metrics: args.vehicle_metrics.clone(),
        ground_metrics: args.ground_metrics.clone(),
        limits: control::limits(&limits)
            .with_context(|| format!("{LIMITS_PATH} has no valid [control] table"))?,
    })
}

fn sim_settings(args: &CheckArgs) -> Result<control::sim_flight::Settings> {
    let limits =
        fs::read_to_string(LIMITS_PATH).with_context(|| format!("cannot read {LIMITS_PATH}"))?;
    Ok(control::sim_flight::Settings {
        gcs_listen: args.gcs_listen,
        observer_listen: args.observer_listen,
        output_listen: args.output_listen,
        vehicle_metrics: args.vehicle_metrics.clone(),
        ground_metrics: args.ground_metrics.clone(),
        limits: control::limits(&limits)
            .with_context(|| format!("{LIMITS_PATH} has no valid [control] table"))?,
    })
}

/// Each service's token against the endpoints it guards: the relay's `job.token` also guards its
/// two job API surfaces.
fn load_tokens(args: &CheckArgs) -> Result<Vec<(String, Token)>> {
    let load = |name: &str| {
        let path = args.token_dir.join(name);
        client::load_token(&path).with_context(|| format!("token {}", path.display()))
    };
    let (vehicle, ground, job) = (
        load("vehicle.token")?,
        load("ground.token")?,
        load("job.token")?,
    );
    Ok(vec![
        (args.vehicle_metrics.clone(), vehicle),
        (args.ground_metrics.clone(), ground),
        (args.relay_metrics.clone(), job.clone()),
        (args.ground_api.clone(), job.clone()),
        (args.vehicle_api.clone(), job),
    ])
}

fn load_profile(name: &str) -> Result<(Profile, String)> {
    if !profile::valid_name(name) {
        bail!("`{name}` is not a profile name (a-z, 0-9 and '-')");
    }
    let path = Path::new(PROFILES_DIR).join(format!("{name}.toml"));
    let text =
        fs::read_to_string(&path).with_context(|| format!("cannot read {}", path.display()))?;
    let profile = Profile::parse(&text)
        .with_context(|| format!("{} is not a valid profile", path.display()))?;
    if profile.meta.name != name {
        bail!("{} is named `{}`", path.display(), profile.meta.name);
    }
    Ok((profile, text))
}

fn vehicle_store() -> Result<StoreConfig> {
    let text = fs::read_to_string(VEHICLE_CONFIG_PATH)
        .with_context(|| format!("cannot read {VEHICLE_CONFIG_PATH}"))?;
    recording::store(&text)
        .map_err(anyhow::Error::msg)
        .with_context(|| format!("{VEHICLE_CONFIG_PATH} has no valid [store] table"))
}

fn load_budgets() -> Result<Budgets> {
    let text =
        fs::read_to_string(BUDGETS_PATH).with_context(|| format!("cannot read {BUDGETS_PATH}"))?;
    Budgets::parse(&text).with_context(|| format!("{BUDGETS_PATH} is invalid"))
}

fn unix_ms() -> Result<u128> {
    let since = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("system clock is before 1970")?;
    Ok(since.as_millis())
}
