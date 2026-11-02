//! `sat-teleop-ground detail`: asks the relay for a retained snapshot or a crop of the frame
//! captured some time ago, and fetches it (docs/architecture.md, "Retained detail").

use std::fs;
use std::path::PathBuf;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use clap::{Args, Subcommand};

use sat_teleop_detail::client::Client;
use sat_teleop_detail::crop::Crop;
use sat_teleop_detail::wire::{JobId, JobRequest, Kind};
use sat_teleop_policy::token::Token;

/// The relay's job API for the ground, on `ground_net`.
const RELAY: &str = "http://192.168.232.20:8080";
const CLIENT_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Subcommand)]
pub enum Detail {
    /// Ask for the retained frame at or before a time, whole or cropped; prints the job id.
    Request(RequestArgs),
    /// Wait for a job's object and write it to a file, labelled with the frame it shows.
    Fetch(FetchArgs),
    /// Cancel a job: the vehicle stops at its next chunk.
    Cancel {
        id: String,
        #[arg(long, default_value = RELAY)]
        relay: String,
    },
}

#[derive(Args)]
pub struct RequestArgs {
    /// Capture time to look for, in UNIX milliseconds.
    #[arg(long, conflicts_with = "ago_s", required_unless_present = "ago_s")]
    at_ms: Option<u64>,
    /// Seconds before now, by this machine's clock. The vehicle's capture clock must agree with
    /// it (docs/measurement.md); `--at-ms` is exact.
    #[arg(long)]
    ago_s: Option<u64>,
    /// A rectangle as fractions of the picture, `x,y,w,h` (for example `0.25,0.25,0.5,0.5`).
    /// Without it the whole frame is sent.
    #[arg(long, value_parser = parse_crop)]
    crop: Option<Crop>,
    /// The most the object may weigh, in bytes.
    #[arg(long, default_value_t = 262_144)]
    max_bytes: u32,
    /// How long the job lives, in seconds.
    #[arg(long, default_value_t = 120)]
    expires_s: u32,
    #[arg(long, default_value = RELAY)]
    relay: String,
}

#[derive(Args)]
pub struct FetchArgs {
    id: String,
    /// Where to write the JPEG.
    #[arg(short, long)]
    output: PathBuf,
    /// Give up waiting after this many seconds.
    #[arg(long, default_value_t = 120)]
    wait_s: u64,
    #[arg(long, default_value = RELAY)]
    relay: String,
}

pub fn run(detail: Detail, token: Option<&Token>) -> Result<()> {
    match detail {
        Detail::Request(args) => request(&args, token),
        Detail::Fetch(args) => fetch(&args, token),
        Detail::Cancel { id, relay } => {
            let id = parse_id(&id)?;
            Client::new(&relay, CLIENT_TIMEOUT)
                .with_token(token)
                .cancel(&id)?;
            println!("cancelled {id}");
            Ok(())
        }
    }
}

fn request(args: &RequestArgs, token: Option<&Token>) -> Result<()> {
    let at_capture_ms = match (args.at_ms, args.ago_s) {
        (Some(at), _) => at,
        (None, Some(ago)) => now_ms()?.saturating_sub(ago.saturating_mul(1000)),
        (None, None) => bail!("give --at-ms or --ago-s"),
    };
    let job = JobRequest {
        kind: if args.crop.is_some() {
            Kind::Crop
        } else {
            Kind::Snapshot
        },
        at_capture_ms,
        crop: args.crop,
        max_bytes: args.max_bytes,
        expires_in_ms: args.expires_s.saturating_mul(1000),
    };
    job.validate().map_err(anyhow::Error::msg)?;
    let id = Client::new(&args.relay, CLIENT_TIMEOUT)
        .with_token(token)
        .request(&job)?;
    println!("{id}");
    Ok(())
}

fn fetch(args: &FetchArgs, token: Option<&Token>) -> Result<()> {
    let id = parse_id(&args.id)?;
    let client = Client::new(&args.relay, CLIENT_TIMEOUT).with_token(token);
    let fetched = client
        .fetch(&id, Duration::from_secs(args.wait_s))
        .with_context(|| format!("job {id}"))?;
    fs::write(&args.output, &fetched.jpeg)
        .with_context(|| format!("cannot write {}", args.output.display()))?;
    let meta = fetched.meta;
    let age_s = now_ms()?.saturating_sub(meta.capture_ms) / 1000;
    let part = meta.crop.map_or_else(
        || "the whole frame".to_owned(),
        |r| format!("crop {},{} {}x{}", r.x, r.y, r.w, r.h),
    );
    println!(
        "{}: retained frame {} captured at {} ms UNIX ({age_s} s ago by this clock), {part} of \
         {}x{}, {} bytes; historical, not live",
        args.output.display(),
        meta.frame_id,
        meta.capture_ms,
        meta.source_width,
        meta.source_height,
        fetched.jpeg.len()
    );
    // The job's slot is free for the next request; the file is the ground's now.
    client.cancel(&id)?;
    Ok(())
}

fn parse_id(text: &str) -> Result<JobId> {
    JobId::parse(text).context("a job id is 32 lowercase hex digits")
}

fn parse_crop(text: &str) -> Result<Crop, String> {
    let numbers: Vec<f64> = text
        .split(',')
        .map(|part| part.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|e| format!("x,y,w,h as numbers: {e}"))?;
    let [x, y, w, h] = numbers[..] else {
        return Err("a crop is x,y,w,h".to_owned());
    };
    Ok(Crop { x, y, w, h })
}

fn now_ms() -> Result<u64> {
    let since = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("this machine's clock is before 1970")?;
    Ok(u64::try_from(since.as_millis()).unwrap_or(u64::MAX))
}
