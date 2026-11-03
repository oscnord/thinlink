//! While `QGroundControl` tracks a box, asks the relay for a crop of it from the newest retained
//! snapshot every few seconds and hands the newest to the freshness output (docs/architecture.md,
//! "Retained detail"). One request is in flight at a time; retrieval's own budget and pacing on the
//! vehicle apply as for any detail job.

use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};

use sat_teleop_control::ground::Camera;
use sat_teleop_detail::client::Client;
use sat_teleop_detail::crop::Crop;
use sat_teleop_detail::wire::{JobRequest, Kind};
use sat_teleop_media::inset::{DetailCell, Image};
use sat_teleop_policy::inset;
use sat_teleop_policy::region::NormRect;

use crate::detail::now_ms;

/// A crop is asked for this often while a box is tracked.
const EVERY: Duration = Duration::from_secs(4);
/// How often the tracked box is looked at.
const TICK: Duration = Duration::from_millis(250);
/// A request that has not been answered by then is given up and the next one waits its turn. A
/// crop crosses the link at the retrieval allowance, a few kB/s under `geo`.
const WAIT: Duration = Duration::from_secs(45);
const MAX_BYTES: u32 = 262_144;

/// Requests crops of the camera's tracked box until the process ends, keeping `cell` to the newest
/// crop for the box being tracked. `picture` is the output's size.
pub fn keep(camera: &Camera, cell: &DetailCell, client: &Client, picture: (usize, usize)) {
    let mut last: Option<(NormRect, Instant)> = None;
    loop {
        thread::sleep(TICK);
        let Some(rect) = camera.tracking() else {
            cell.clear();
            last = None;
            continue;
        };
        if last.is_some_and(|(asked, _)| asked != rect) {
            cell.clear();
        }
        if last.is_some_and(|(asked, at)| asked == rect && at.elapsed() < EVERY) {
            continue;
        }
        last = Some((rect, Instant::now()));
        match crop(client, rect, picture) {
            Ok((image, age_ms)) if camera.tracking() == Some(rect) => {
                cell.put(image, age_ms);
                camera.detail_received(rect, Instant::now());
            }
            Ok(_) => {}
            Err(e) => eprintln!("no crop of the pilot's box: {e:#}"),
        }
    }
}

/// The newest retained snapshot's part around `rect` and its capture age.
fn crop(client: &Client, rect: NormRect, picture: (usize, usize)) -> Result<(Image, u64)> {
    let wide = inset::widen(rect);
    let job = JobRequest {
        kind: Kind::Crop,
        at_capture_ms: now_ms()?,
        crop: Some(Crop {
            x: f64::from(wide.x0),
            y: f64::from(wide.y0),
            w: f64::from(wide.x1) - f64::from(wide.x0),
            h: f64::from(wide.y1) - f64::from(wide.y0),
        }),
        max_bytes: MAX_BYTES,
        expires_in_ms: u32::try_from(WAIT.as_millis()).unwrap_or(u32::MAX),
    };
    job.validate().map_err(anyhow::Error::msg)?;
    let id = client.request(&job)?;
    let fetched = client.fetch(&id, WAIT);
    // The job may be gone already (expired or failed); its slot is free either way.
    let _ = client.cancel(&id);
    let fetched = fetched.with_context(|| format!("job {id}"))?;
    let age_ms = now_ms()?.saturating_sub(fetched.meta.capture_ms);
    Ok((Image::from_jpeg(&fetched.jpeg, picture)?, age_ms))
}
