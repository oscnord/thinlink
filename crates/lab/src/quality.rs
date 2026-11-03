//! `lab check quality`: how good the picture is, next to how old. The vehicle plays a looping clip
//! of the regional check's test card over a terrain, scrolling down (`regional::card::loop_y4m`),
//! which the check writes to the vehicle's volume. `checks` pulls the relay's stream after the
//! ladder has settled and, for every decoded frame, finds the clip frame it shows (`best_match`:
//! the rung drops frames before they are stamped, so the stamp's frame id says nothing about the
//! clip) and scores its luma against that frame by SSIM, below the stamp's rows: at the full
//! 1280x720 (the received frame scaled up) and at the received size (the clip frame scaled down the
//! way the sender scales it). Frames that never arrived are the gaps in the stamp's frame ids.
//! `docs/testbed.md#quality-check` defines the metrics.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use sat_teleop_video::card::{self as reference_card, Luma};
use sat_teleop_video::receiver::Source;
use sat_teleop_video::{Health, supervise};
use sat_teleop_video_core::stamp::{plausible_age, state_rows_px};

use crate::adaptation;
use crate::budget::{self, Budget};
use crate::check::{self, Results, Sample, Verdict, float, percentile};
use crate::client::AgentClient;
use crate::regional::card::{self, HEIGHT, LOOP_FRAMES, WIDTH};
use crate::regional::ssim;

/// The clip's size, the vehicle's source (`testbed/config/vehicle-quality.toml`).
const SOURCE: (usize, usize) = (WIDTH, HEIGHT);
/// Each rung's frame size in `testbed/ladder.toml`: the size the relay's stream arrives at.
const RUNG_SIZE: [(u16, u16); 6] = [
    (1280, 720),
    (640, 360),
    (640, 360),
    (480, 270),
    (320, 180),
    (320, 180),
];
/// A frame whose SSIM at full resolution is below this counts as poor. A clean 720p frame scores
/// above 0.95 and a frame scaled from the 360p rung about 0.85, so the floor sits below both.
const FLOOR: f64 = 0.8;
/// The grid a frame is reduced to for matching.
const SIGNATURE: (usize, usize) = (64, 36);
/// Matching may move backwards through the loop at most this share of the frames that score above
/// the floor: a larger share means the match is not telling the frames apart.
const MAX_BACKWARD_SHARE: f64 = 0.02;
const MIN_FRAMES: usize = 30;
/// The clip frame name of the vehicle's volume (`vehicle-quality.toml`).
const CLIP_FILE: &str = "quality.y4m";

/// Requested settings of one run.
#[derive(Debug)]
pub struct Settings {
    /// RTSP URL of the relay's stream.
    pub url: String,
    /// The vehicle's snapshot volume as this host mounts it: the clip goes here.
    pub snapshot_dir: std::path::PathBuf,
    pub warmup: Duration,
    pub duration: Duration,
    pub vehicle_metrics: String,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "url": self.url,
            "warmup_s": self.warmup.as_secs_f64(),
            "duration_s": self.duration.as_secs_f64(),
            "loop_frames": LOOP_FRAMES,
            "pan_rows_per_frame": card::PAN,
            "ssim_floor": FLOOR,
            "signature": [SIGNATURE.0, SIGNATURE.1],
            "vehicle_metrics": self.vehicle_metrics,
        })
    }
}

/// The clip's frames at the full and the received size, and what matching compares.
struct Scene {
    size: (usize, usize),
    full: Vec<Vec<u8>>,
    native: Vec<Vec<u8>>,
    signatures: Vec<Vec<f32>>,
}

/// What one decoded frame scored.
#[derive(Debug, Clone, Copy)]
struct Scored {
    index: usize,
    ssim_full: f64,
    ssim_native: f64,
}

impl Scene {
    fn new(size: (usize, usize)) -> Result<Self, String> {
        let full = card::loop_luma();
        let mut native = Vec::with_capacity(full.len());
        for luma in &full {
            let mut i420 = luma.clone();
            i420.resize(SOURCE.0 * SOURCE.1 * 3 / 2, 128);
            let to = |(w, h): (usize, usize)| {
                (
                    u16::try_from(w).unwrap_or(u16::MAX),
                    u16::try_from(h).unwrap_or(u16::MAX),
                )
            };
            let scaled = reference_card::reference(&i420, to(SOURCE), to(size))
                .map_err(|e| e.to_string())?;
            native.push(scaled.pixels);
        }
        let first = state_rows_px(size.0);
        let signatures = native
            .iter()
            .map(|luma| signature(luma, size, first))
            .collect();
        Ok(Self {
            size,
            full,
            native,
            signatures,
        })
    }

    fn score(&self, received: &Luma) -> Option<Scored> {
        let size = self.size;
        if (received.width, received.height) != size {
            return None;
        }
        let first = state_rows_px(size.0);
        let (index, _) = best_match(&signature(&received.pixels, size, first), &self.signatures)?;
        let ssim_native = masked_ssim(&received.pixels, self.native.get(index)?, size, first)?;
        let scaled;
        let full = if size == SOURCE {
            &received.pixels
        } else {
            scaled = upscale(&received.pixels, size, SOURCE);
            &scaled
        };
        let ssim_full = masked_ssim(full, self.full.get(index)?, SOURCE, state_rows_px(SOURCE.0))?;
        Some(Scored {
            index,
            ssim_full,
            ssim_native,
        })
    }
}

/// The mean luma of `SIGNATURE` cells over the rows from `first` down.
#[expect(
    clippy::cast_precision_loss,
    reason = "cell sums of at most 255 * a few thousand pixels are exact in f32"
)]
fn signature(luma: &[u8], (width, height): (usize, usize), first: usize) -> Vec<f32> {
    let rows = height.saturating_sub(first);
    let mut cells = Vec::with_capacity(SIGNATURE.0 * SIGNATURE.1);
    for cy in 0..SIGNATURE.1 {
        let (y0, y1) = (
            first + cy * rows / SIGNATURE.1,
            first + (cy + 1) * rows / SIGNATURE.1,
        );
        for cx in 0..SIGNATURE.0 {
            let (x0, x1) = (cx * width / SIGNATURE.0, (cx + 1) * width / SIGNATURE.0);
            let mut sum = 0_u64;
            for y in y0..y1 {
                if let Some(row) = luma.get(y * width + x0..y * width + x1) {
                    sum += row.iter().map(|&p| u64::from(p)).sum::<u64>();
                }
            }
            let count = ((y1 - y0) * (x1 - x0)).max(1);
            cells.push(sum as f32 / count as f32);
        }
    }
    cells
}

/// The index of the reference signature closest to `signature` (mean absolute difference), and
/// that difference.
#[expect(
    clippy::cast_precision_loss,
    reason = "a signature has a few thousand cells"
)]
fn best_match(signature: &[f32], references: &[Vec<f32>]) -> Option<(usize, f32)> {
    references
        .iter()
        .enumerate()
        .map(|(index, reference)| {
            let sum: f32 = signature
                .iter()
                .zip(reference)
                .map(|(a, b)| (a - b).abs())
                .sum();
            (index, sum / signature.len().max(1) as f32)
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

/// The mean SSIM of two planes of `size` over the rows from `first` down: the stamp is drawn over
/// the rows above and is not part of the picture.
#[expect(
    clippy::cast_precision_loss,
    reason = "window counts stay far below 2^53"
)]
fn masked_ssim(
    received: &[u8],
    reference: &[u8],
    (width, height): (usize, usize),
    first: usize,
) -> Option<f64> {
    let (received, reference) = (
        received.get(first * width..)?,
        reference.get(first * width..)?,
    );
    let windows = ssim::windows(received, reference, width, height.checked_sub(first)?);
    (!windows.is_empty())
        .then(|| windows.iter().map(|w| w.ssim).sum::<f64>() / windows.len() as f64)
}

/// `luma` of `from` size scaled up to `to` by bilinear interpolation.
fn upscale(luma: &[u8], from: (usize, usize), to: (usize, usize)) -> Vec<u8> {
    let positions = |source: usize, target: usize| -> Vec<(usize, u32)> {
        (0..target)
            .map(|t| {
                let at = t * (source - 1) * 256 / (target - 1).max(1);
                (at / 256, u32::try_from(at % 256).unwrap_or(0))
            })
            .collect()
    };
    let (columns, rows) = (positions(from.0, to.0), positions(from.1, to.1));
    let pixel = |x: usize, y: usize| {
        u32::from(
            luma.get(y.min(from.1 - 1) * from.0 + x.min(from.0 - 1))
                .copied()
                .unwrap_or(0),
        )
    };
    let mut out = Vec::with_capacity(to.0 * to.1);
    for &(y, fy) in &rows {
        for &(x, fx) in &columns {
            let top = pixel(x, y) * (256 - fx) + pixel(x + 1, y) * fx;
            let bottom = pixel(x, y + 1) * (256 - fx) + pixel(x + 1, y + 1) * fx;
            let value = (top * (256 - fy) + bottom * fy) >> 16;
            out.push(u8::try_from(value).unwrap_or(u8::MAX));
        }
    }
    out
}

/// One decoded frame as the window keeps it.
#[derive(Debug, Clone, Copy)]
struct Frame {
    at: Instant,
    received_ms: u64,
    id: Option<u32>,
    age_ms: Option<f64>,
    scored: Option<Scored>,
    score_ms: f64,
}

/// The relay's stream decoded and scored on a thread of its own.
struct Scorer {
    stop: Arc<AtomicBool>,
    frames: Arc<Mutex<Vec<Frame>>>,
    thread: JoinHandle<()>,
}

impl Scorer {
    fn start(source: Source, scene: Scene) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let frames = Arc::new(Mutex::new(Vec::new()));
        let thread = {
            let (stop, frames) = (Arc::clone(&stop), Arc::clone(&frames));
            thread::spawn(move || {
                let health = Health::default();
                supervise(&health, || {
                    let size = (
                        u16::try_from(scene.size.0).unwrap_or(u16::MAX),
                        u16::try_from(scene.size.1).unwrap_or(u16::MAX),
                    );
                    reference_card::receive_luma(&source, size, &stop, |observation, luma| {
                        let started = Instant::now();
                        let scored = scene.score(&luma);
                        let frame = Frame {
                            at: observation.received_at,
                            received_ms: observation.received_ms,
                            id: observation.stamp.map(|s| s.frame_id.0),
                            age_ms: observation
                                .stamp
                                .and_then(|s| plausible_age(observation.received_ms, s))
                                .map(f64::from),
                            scored,
                            score_ms: started.elapsed().as_secs_f64() * 1000.0,
                        };
                        if let Ok(mut frames) = frames.lock() {
                            frames.push(frame);
                        }
                        true
                    })
                });
            })
        };
        Self {
            stop,
            frames,
            thread,
        }
    }

    fn between(&self, from: Instant, to: Instant) -> Vec<Frame> {
        let frames = self.frames.lock().map(|f| f.clone()).unwrap_or_default();
        frames
            .into_iter()
            .filter(|f| f.at >= from && f.at <= to)
            .collect()
    }

    fn finish(self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.thread.join();
    }
}

pub fn run(
    agent: &AgentClient,
    profile_text: &str,
    settings: &Settings,
    budget: Option<&Budget>,
) -> Results {
    let mut results = Results {
        thresholds: serde_json::to_value(budget).unwrap_or(Value::Null),
        settings: settings.to_json(),
        ..Results::default()
    };
    if !check::apply_profile(&mut results, agent, profile_text) {
        return results;
    }
    if let Err(e) = sat_teleop_video::init() {
        results.record("video_received", Verdict::Incomplete, e.to_string());
        return results;
    }
    if let Err(e) = write_clip(&settings.snapshot_dir) {
        let detail = format!(
            "cannot write the clip to {}: {e}",
            settings.snapshot_dir.display()
        );
        results.record("clip_written", Verdict::Incomplete, detail);
        return results;
    }
    let vehicle = AgentClient::new(&settings.vehicle_metrics);
    let settled = adaptation::settle_anywhere(&vehicle);
    adaptation::record_settled(&mut results, &settled);
    let Some(size) = settled
        .rung
        .and_then(|rung| RUNG_SIZE.get(usize::try_from(rung).ok()?))
        .map(|&(w, h)| (usize::from(w), usize::from(h)))
    else {
        results.record(
            "video_received",
            Verdict::Incomplete,
            "the vehicle reports no rung",
        );
        return results;
    };
    let scene = match Scene::new(size) {
        Ok(scene) => scene,
        Err(e) => {
            results.record("clip_rendered", Verdict::Incomplete, e);
            return results;
        }
    };
    let scorer = Scorer::start(Source::Rtsp(settings.url.clone()), scene);
    thread::sleep(settings.warmup);
    let (from, rung_before) = (Instant::now(), rung_of(&vehicle));
    thread::sleep(settings.duration);
    let (to, rung_after) = (Instant::now(), rung_of(&vehicle));
    let frames = scorer.between(from, to);
    scorer.finish();
    results.readings = json!({
        "frame_size": [size.0, size.1],
        "rung_before": rung_before,
        "rung_after": rung_after,
    });
    record(&mut results, &frames);
    budget::grade(&mut results, budget);
    results
}

fn rung_of(vehicle: &AgentClient) -> Option<u64> {
    check::get_json(vehicle, "/metrics")
        .ok()?
        .pointer("/adapt/rung")?
        .as_u64()
}

/// Writes the clip where the vehicle's source reads it (`/snapshots/quality.y4m` in
/// `vehicle-quality.toml`), whole or not at all: the vehicle starts over until it opens one. The
/// clip is the same every run, so one of its length that is there already is left alone: the volume
/// has no room for a second copy beside it.
fn write_clip(dir: &std::path::Path) -> std::io::Result<()> {
    let clip = card::loop_y4m();
    let path = dir.join(CLIP_FILE);
    if std::fs::metadata(&path).is_ok_and(|m| u64::try_from(clip.len()).is_ok_and(|n| m.len() == n))
    {
        return Ok(());
    }
    let part = dir.join(format!("{CLIP_FILE}.part"));
    std::fs::write(&part, clip)?;
    std::fs::rename(part, path)
}

/// The ids of the frames that never arrived, from the stamp ids of the arrived ones in order of
/// arrival: the ids from the first to the last readable one, less the frames received between
/// them, readable or not.
fn missing(frames: &[Frame]) -> usize {
    let readable: Vec<(usize, u32)> = frames
        .iter()
        .enumerate()
        .filter_map(|(position, f)| Some((position, f.id?)))
        .collect();
    let (Some(&(first_at, first_id)), Some(&(last_at, last_id))) =
        (readable.first(), readable.last())
    else {
        return 0;
    };
    let ids = usize::try_from(last_id.saturating_sub(first_id)).unwrap_or(0) + 1;
    ids.saturating_sub(last_at - first_at + 1)
}

/// Steps backwards through the loop between neighbouring frames that both score above the floor,
/// which a frame shown later cannot take: a forward step of more than half the loop is a step
/// back. A frame the link damaged scores below the floor and matches whichever clip frame it
/// resembles most, so its step says nothing about the match.
fn backward_steps(matches: &[(usize, f64)]) -> usize {
    matches
        .windows(2)
        .filter(|pair| match pair {
            [(a, sa), (b, sb)] if *sa >= FLOOR && *sb >= FLOOR => {
                (b + LOOP_FRAMES - a) % LOOP_FRAMES > LOOP_FRAMES / 2
            }
            _ => false,
        })
        .count()
}

#[expect(
    clippy::cast_precision_loss,
    reason = "frame counts stay far below 2^53"
)]
fn record(results: &mut Results, frames: &[Frame]) {
    let scored: Vec<(&Frame, Scored)> = frames
        .iter()
        .filter_map(|f| f.scored.map(|s| (f, s)))
        .collect();
    if scored.len() < MIN_FRAMES {
        let detail = format!(
            "{} of {MIN_FRAMES} frames scored in the window",
            scored.len()
        );
        results.record("video_received", Verdict::Incomplete, detail);
        return;
    }
    let sorted = |values: Vec<f64>| {
        let mut values = values;
        values.sort_by(f64::total_cmp);
        values
    };
    let full = sorted(scored.iter().map(|(_, s)| s.ssim_full).collect());
    let native = sorted(scored.iter().map(|(_, s)| s.ssim_native).collect());
    let ages = sorted(frames.iter().filter_map(|f| f.age_ms).collect());
    let times = sorted(frames.iter().map(|f| f.score_ms).collect());
    let below = full.iter().filter(|&&s| s < FLOOR).count();
    let lost = missing(frames);
    let backward = backward_steps(
        &scored
            .iter()
            .map(|(_, s)| (s.index, s.ssim_full))
            .collect::<Vec<_>>(),
    );
    let metrics = [
        ("ssim_p50", percentile(&full, 50)),
        ("ssim_p5", percentile(&full, 5)),
        (
            "ssim_below_floor_share",
            Some(below as f64 / full.len() as f64),
        ),
        ("ssim_native_p50", percentile(&native, 50)),
        ("ssim_native_p5", percentile(&native, 5)),
        ("frames_scored", Some(scored.len() as f64)),
        ("frames_missing", Some(lost as f64)),
        (
            "frames_missing_ratio",
            Some(lost as f64 / (lost + frames.len()) as f64),
        ),
        ("frame_age_p50_ms", percentile(&ages, 50)),
        ("frame_age_p95_ms", percentile(&ages, 95)),
        ("match_backward_steps", Some(backward as f64)),
        ("score_ms_p95", percentile(&times, 95)),
    ];
    for (name, value) in metrics {
        if let Some(value) = value {
            results.metrics.insert(name, value);
        }
    }
    let share = backward as f64 / scored.len() as f64;
    let verdict = if share <= MAX_BACKWARD_SHARE {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    let detail = format!(
        "{backward} steps back in the loop between good frames, over {} frames scored",
        scored.len()
    );
    results.record("matching_forward", verdict, detail);
    results.samples = samples(&scored);
}

fn samples(scored: &[(&Frame, Scored)]) -> Vec<Sample> {
    let mut out = Vec::new();
    for (frame, score) in scored {
        let Some(id) = frame.id else {
            continue;
        };
        let unix_s = float(frame.received_ms) / 1000.0;
        out.push(Sample {
            unix_s,
            series: "ssim",
            seq: id,
            value: score.ssim_full,
        });
        out.push(Sample {
            unix_s,
            series: "clip_index",
            seq: id,
            value: float(u64::try_from(score.index).unwrap_or(0)),
        });
        if let Some(age) = frame.age_ms {
            out.push(Sample {
                unix_s,
                series: "frame_age_ms",
                seq: id,
                value: age,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIDE: usize = 64;

    /// A plane of rings of rising frequency.
    fn rings() -> Vec<u8> {
        (0..SIDE * SIDE)
            .map(|i| {
                let (x, y) = (i % SIDE, i / SIDE);
                if (x * x + y * y) / 5 % 2 == 0 {
                    30
                } else {
                    220
                }
            })
            .collect()
    }

    fn frame(id: Option<u32>) -> Frame {
        Frame {
            at: Instant::now(),
            received_ms: 0,
            id,
            age_ms: None,
            scored: None,
            score_ms: 0.0,
        }
    }

    #[test]
    fn a_noisy_frame_is_matched_to_the_clip_frame_it_shows_whatever_its_stamp_says() {
        let size = (WIDTH, HEIGHT);
        let first = state_rows_px(WIDTH);
        let clip = card::loop_luma();
        let signatures: Vec<Vec<f32>> = clip.iter().map(|l| signature(l, size, first)).collect();
        for shown in [0, 7, 8, 30, LOOP_FRAMES - 1] {
            let mut received = clip[shown].clone();
            for (i, p) in received.iter_mut().enumerate() {
                *p = p.saturating_add_signed(i8::try_from(i % 7).unwrap_or(0) - 3);
            }
            let found = best_match(&signature(&received, size, first), &signatures).unwrap();
            assert_eq!(found.0, shown);
        }
    }

    #[test]
    fn the_stamp_rows_do_not_count_against_the_picture() {
        let reference = rings();
        let mut stamped = reference.clone();
        stamped[..8 * SIDE].fill(255);
        let masked = masked_ssim(&stamped, &reference, (SIDE, SIDE), 8).unwrap();
        let unmasked = masked_ssim(&stamped, &reference, (SIDE, SIDE), 0).unwrap();
        assert!((masked - 1.0).abs() < 1e-9, "{masked}");
        assert!(unmasked < 0.99, "{unmasked}");
    }

    #[test]
    fn frames_that_never_arrived_are_the_gaps_in_the_ids() {
        let ids =
            |ids: &[Option<u32>]| missing(&ids.iter().map(|&id| frame(id)).collect::<Vec<_>>());
        assert_eq!(ids(&[Some(10), Some(11), Some(12)]), 0);
        assert_eq!(ids(&[Some(10), Some(11), Some(15), Some(16)]), 3);
        assert_eq!(ids(&[Some(10), None, Some(12)]), 0);
        assert_eq!(ids(&[None, None]), 0);
    }

    #[test]
    fn a_step_back_in_the_loop_is_told_from_dropped_frames() {
        let good = |indices: &[usize]| {
            backward_steps(&indices.iter().map(|&i| (i, 0.9)).collect::<Vec<_>>())
        };
        assert_eq!(good(&[3, 5, 7, 9]), 0);
        assert_eq!(good(&[43, 1, 3]), 0);
        assert_eq!(good(&[10, 9, 11]), 1);
        assert_eq!(backward_steps(&[(10, 0.9), (24, 0.5), (12, 0.9)]), 0);
    }
}
