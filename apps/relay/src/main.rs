use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use serde::Serialize;

use sat_teleop_detail::relay::{self, Relay, RelayConfig, RelaySnapshot};
use sat_teleop_video::feedback::{self, FeedbackConfig, FeedbackSnapshot, FeedbackStats};
use sat_teleop_video::ingress::{self, IngressConfig, IngressSnapshot, IngressStats};
use sat_teleop_video::{metrics, supervise};

#[derive(Parser)]
#[command(
    name = "sat-teleop-relay",
    about = "Relay: RTP from the vehicle in, RTSP publish to the local MediaMTX, feedback to the vehicle"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run the ingress and the feedback endpoint, restarting them after a failure, and serve
    /// GET /metrics.
    Run {
        /// Relay config, for example testbed/config/relay.toml.
        #[arg(long)]
        config: PathBuf,
        /// Address for GET /metrics, for example 192.168.233.20:9100.
        #[arg(long)]
        metrics_addr: String,
        /// A file with the bearer token the metrics endpoint and the job API need. Without one
        /// the metrics endpoint listens on 127.0.0.1 only and the job API does not start.
        #[arg(long)]
        token_file: Option<PathBuf>,
    },
}

/// Pause between a failed job API and the next try: the overlay address may not be up yet.
const DETAIL_RETRY: Duration = Duration::from_secs(1);

struct Config {
    ingress: IngressConfig,
    feedback: FeedbackConfig,
    detail: RelayConfig,
}

#[derive(Serialize)]
struct Metrics {
    #[serde(flatten)]
    ingress: IngressSnapshot,
    feedback: FeedbackSnapshot,
    detail: RelaySnapshot,
}

fn main() -> Result<()> {
    let Command::Run {
        config,
        metrics_addr,
        token_file,
    } = Cli::parse().command;
    let token = token_file
        .as_deref()
        .map(metrics::load_token)
        .transpose()
        .context("token file")?;
    let Config {
        ingress,
        feedback,
        detail,
    } = load(&config)?;
    sat_teleop_video::init().context("GStreamer")?;
    let stats = Arc::new(IngressStats::default());
    {
        let stats = Arc::clone(&stats);
        thread::spawn(move || supervise(&stats.health, || ingress::run(&ingress, &stats)));
    }
    let answers = Arc::new(FeedbackStats::default());
    {
        let (stats, answers) = (Arc::clone(&stats), Arc::clone(&answers));
        thread::spawn(move || {
            supervise(&answers.health, || {
                feedback::answer(&feedback, &stats, &answers)
            });
        });
    }
    let jobs = Arc::new(Relay::default());
    match token.clone() {
        Some(token) => {
            let jobs = Arc::clone(&jobs);
            thread::spawn(move || {
                loop {
                    let stopped = relay::serve(&detail, &jobs, &token);
                    eprintln!("job API stopped, starting again: {stopped:?}");
                    thread::sleep(DETAIL_RETRY);
                }
            });
        }
        None => eprintln!("job API off: it needs a token (--token-file)"),
    }
    let snapshot = || Metrics {
        ingress: stats.snapshot(),
        feedback: answers.snapshot(),
        detail: jobs.snapshot(),
    };
    metrics::serve(&metrics_addr, token.as_ref(), snapshot).context("metrics endpoint stopped")
}

fn load(path: &Path) -> Result<Config> {
    let text =
        fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let mut table: toml::Table =
        toml::from_str(&text).with_context(|| format!("{} is not TOML", path.display()))?;
    let feedback = table
        .remove("feedback")
        .with_context(|| format!("{} has no [feedback] table", path.display()))?
        .try_into()
        .with_context(|| format!("{} has an invalid [feedback] table", path.display()))?;
    let detail = table
        .remove("detail")
        .with_context(|| format!("{} has no [detail] table", path.display()))?
        .try_into()
        .with_context(|| format!("{} has an invalid [detail] table", path.display()))?;
    let ingress = table
        .try_into()
        .with_context(|| format!("{} is not an ingress config", path.display()))?;
    Ok(Config {
        ingress,
        feedback,
        detail,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_config_loads() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testbed/config/relay.toml");
        load(&path).unwrap();
    }
}
