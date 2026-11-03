//! Following one region from frame to frame: normalised cross-correlation of a template, taken
//! from the box the pilot drew, in a small search window around where it was. Pure, on the luma
//! plane of a picture of fixed size, so the same code runs on the vehicle and in tests.
//!
//! The follow ends rather than guesses: a match under the threshold for `lost_frames` frames in a
//! row, a target that leaves the picture, or a start on a box with nothing to match.

use crate::region::NormRect;

/// The size of the picture the tracker is given, whatever the camera's.
pub const WIDTH: usize = 320;
pub const HEIGHT: usize = 180;
/// How far the target may move between two frames, in pixels of that picture. After a weak frame
/// the window is wider by this much for each weak frame in a row (at most `MAX_REACH` times), since
/// the target kept moving while the box did not.
const SEARCH_PX: usize = 16;
const MAX_REACH: usize = 3;
/// Narrowest side of a template, in pixels.
const MIN_SIDE_PX: usize = 8;
/// A template is sampled on a lattice that keeps it under this many samples, so a large box
/// (a building) costs no more than a person.
const MAX_SAMPLES: usize = 3000;
/// Standard deviation of the template under which there is nothing to follow (flat sky, a wall).
const MIN_STD_DEV: f64 = 4.0;
/// A match this good moves the template a step towards what matched, so a target that slowly
/// changes its look (a turning person, a changing angle) keeps its score. Anything worse leaves
/// the template alone, so a target that leaves or is hidden does not teach it the background.
const REFRESH_SCORE: f32 = 0.7;
/// The template takes one part in this of the new patch.
const REFRESH_WEIGHT: i32 = 8;
/// The outline of a followed box in Y, U and V and its letter: its own colour, not a kind's.
pub const YUV: [u8; 3] = [158, 140, 197];
pub const GLYPH: [u8; 7] = [
    0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000,
];
/// A detection must overlap the pilot's box by more than this (intersection over union) to take
/// its place as the start.
const SNAP_OVERLAP: f32 = 0.5;

/// A picture of `WIDTH` x `HEIGHT` luma samples, rows without padding.
#[derive(Debug, Clone, Copy)]
pub struct Luma<'a>(&'a [u8]);

impl<'a> Luma<'a> {
    pub fn new(data: &'a [u8]) -> Option<Self> {
        (data.len() == WIDTH * HEIGHT).then_some(Self(data))
    }

    /// The row `y` from column `x` to its end.
    fn row(&self, x: usize, y: usize) -> Option<&'a [u8]> {
        self.0.get(y * WIDTH + x..(y + 1) * WIDTH)
    }
}

/// What ends a follow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lost {
    /// The match stayed under the threshold.
    Score,
    /// The target went out of the picture.
    Edge,
    /// The box has nothing to follow: too small, or flat.
    Flat,
}

impl Lost {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Score => "score",
            Self::Edge => "edge",
            Self::Flat => "flat",
        }
    }
}

/// `[region]` of the limits file: when a follow ends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// Correlation (0 to 1) a match must reach.
    pub min_score: f32,
    /// Frames in a row under `min_score` that end the follow.
    pub lost_frames: u32,
}

/// What the tracker was asked to follow: the pilot's box, and which request it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Order {
    pub generation: u64,
    pub seed: NormRect,
}

/// What one frame did to the follow.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Step {
    /// Nothing is being followed.
    Idle,
    /// The target is here now (the start counts).
    Moved(NormRect, f32),
    /// A weak match: the target is not reported to have moved.
    Held(f32),
    Lost(Lost),
}

/// The tracker of the order it is given, and the frames in a row it has been unsure of.
#[derive(Debug)]
pub struct Follower {
    settings: Settings,
    generation: u64,
    tracker: Option<Tracker>,
    weak: u32,
}

impl Follower {
    pub fn new(settings: Settings) -> Self {
        Self {
            settings,
            generation: 0,
            tracker: None,
            weak: 0,
        }
    }

    /// Takes one frame. `order` is what the cell wants followed now (none: nothing), `detections`
    /// the boxes the detector has, one of which replaces a new seed it overlaps.
    pub fn step(&mut self, frame: Luma, order: Option<Order>, detections: &[NormRect]) -> Step {
        let Some(order) = order else {
            self.tracker = None;
            return Step::Idle;
        };
        if order.generation != self.generation {
            return self.start(frame, order, detections);
        }
        let Some(tracker) = &mut self.tracker else {
            return Step::Idle;
        };
        let found = tracker.update(
            frame,
            1 + usize::try_from(self.weak).unwrap_or(0).min(MAX_REACH - 1),
        );
        if found.leaving {
            self.tracker = None;
            return Step::Lost(Lost::Edge);
        }
        if self.settings.min_score > found.score {
            self.weak += 1;
            return self.weak_frame(found.score);
        }
        self.weak = 0;
        Step::Moved(found.rect, found.score)
    }

    fn start(&mut self, frame: Luma, order: Order, detections: &[NormRect]) -> Step {
        self.generation = order.generation;
        self.weak = 0;
        let seed = snap(order.seed, detections);
        self.tracker = Tracker::start(frame, seed);
        match self.tracker {
            Some(_) => Step::Moved(seed, 1.0),
            None => Step::Lost(Lost::Flat),
        }
    }

    fn weak_frame(&mut self, score: f32) -> Step {
        if self.weak < self.settings.lost_frames {
            return Step::Held(score);
        }
        self.tracker = None;
        Step::Lost(Lost::Score)
    }
}

/// The detection that overlaps `seed` by more than half and most, or `seed` itself.
pub fn snap(seed: NormRect, detections: &[NormRect]) -> NormRect {
    detections
        .iter()
        .map(|d| (seed.overlap(d), *d))
        .filter(|(overlap, _)| *overlap > SNAP_OVERLAP)
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .map_or(seed, |(_, d)| d)
}

/// A template and where it was last found, in pixels of the picture.
#[derive(Debug)]
struct Tracker {
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    /// The edges of the picture the box touched when the follow began: not leaving through them.
    at_start: [bool; 4],
    template: Template,
}

/// A template sampled on a lattice, minus nothing: the sums the correlation needs are kept.
#[derive(Debug)]
struct Template {
    /// Distance between samples, in pixels.
    step: usize,
    columns: usize,
    samples: Vec<i32>,
    sum: i32,
    /// Sum of squares minus the sum squared over the count: the template's variance times its count.
    energy: f64,
}

struct Found {
    rect: NormRect,
    score: f32,
    leaving: bool,
}

impl Tracker {
    fn start(frame: Luma, rect: NormRect) -> Option<Self> {
        let pixels = rect.pixels(WIDTH, HEIGHT);
        let (width, height) = (pixels.x1 - pixels.x0, pixels.y1 - pixels.y0);
        if width < MIN_SIDE_PX || height < MIN_SIDE_PX {
            return None;
        }
        let template = Template::take(frame, pixels.x0, pixels.y0, width, height)?;
        let count = count(template.samples.len());
        let mut tracker = Self {
            x: pixels.x0,
            y: pixels.y0,
            width,
            height,
            at_start: [false; 4],
            template,
        };
        tracker.at_start = tracker.touching((pixels.x0, pixels.y0));
        (tracker.template.energy / count >= MIN_STD_DEV * MIN_STD_DEV).then_some(tracker)
    }

    /// Which edges (left, right, top, bottom) the box touches at `at`.
    fn touching(&self, at: (usize, usize)) -> [bool; 4] {
        [
            at.0 == 0,
            at.0 + self.width == WIDTH,
            at.1 == 0,
            at.1 + self.height == HEIGHT,
        ]
    }

    fn update(&mut self, frame: Luma, reach: usize) -> Found {
        let xs = search(self.x, WIDTH - self.width, reach);
        let ys = search(self.y, HEIGHT - self.height, reach);
        let (mut best, mut at) = (f32::MIN, (self.x, self.y));
        for y in ys {
            for x in xs.clone() {
                let score = self.template.score(frame, x, y);
                if score > best {
                    (best, at) = (score, (x, y));
                }
            }
        }
        let leaving = self
            .touching(at)
            .iter()
            .zip(self.at_start)
            .any(|(now, was)| *now && !was);
        (self.x, self.y) = at;
        if best >= REFRESH_SCORE {
            self.template
                .blend(frame, at.0, at.1, self.width, self.height);
        }
        Found {
            rect: self.rect(),
            score: best,
            leaving,
        }
    }

    fn rect(&self) -> NormRect {
        NormRect {
            x0: fraction(self.x, WIDTH),
            y0: fraction(self.y, HEIGHT),
            x1: fraction(self.x + self.width, WIDTH),
            y1: fraction(self.y + self.height, HEIGHT),
        }
    }
}

/// The positions a window may take along one axis: `reach` search radii either side of `at`, and
/// inside the picture (`last` is the greatest top-left position that still fits).
fn search(at: usize, last: usize, reach: usize) -> std::ops::RangeInclusive<usize> {
    let radius = SEARCH_PX * reach;
    at.saturating_sub(radius)..=(at + radius).min(last)
}

impl Template {
    /// The `width` x `height` window at (`x`, `y`), or none if it does not lie in the picture.
    fn take(frame: Luma, x: usize, y: usize, width: usize, height: usize) -> Option<Self> {
        let step = lattice_step(width * height);
        let columns = width.div_ceil(step);
        let samples: Vec<i32> = (0..height.div_ceil(step))
            .map(|j| frame.row(x, y + j * step))
            .collect::<Option<Vec<_>>>()?
            .into_iter()
            .flat_map(|row| row.iter().step_by(step).take(columns))
            .map(|v| i32::from(*v))
            .collect();
        let sum: i32 = samples.iter().sum();
        let squares: i32 = samples.iter().map(|v| v * v).sum();
        Some(Self {
            step,
            columns,
            sum,
            energy: energy(sum, squares, samples.len()),
            samples,
        })
    }

    /// Moves the samples a step towards the window at (`x`, `y`), which has the template's size.
    fn blend(&mut self, frame: Luma, x: usize, y: usize, width: usize, height: usize) {
        let Some(patch) = Self::take(frame, x, y, width, height) else {
            return;
        };
        for (sample, new) in self.samples.iter_mut().zip(&patch.samples) {
            *sample += (new - *sample) / REFRESH_WEIGHT;
        }
        self.sum = self.samples.iter().sum();
        let squares: i32 = self.samples.iter().map(|v| v * v).sum();
        self.energy = energy(self.sum, squares, self.samples.len());
    }

    /// Normalised cross-correlation with the window at (`x`, `y`): 1 for the same picture up to
    /// brightness and contrast, near 0 for something else, 0 for a flat window.
    #[expect(
        clippy::cast_possible_truncation,
        reason = "a correlation between -1 and 1 needs no more than f32"
    )]
    fn score(&self, frame: Luma, x: usize, y: usize) -> f32 {
        let rows =
            (0..self.samples.len() / self.columns).filter_map(|j| frame.row(x, y + j * self.step));
        let (mut sum, mut squares, mut product) = (0_i32, 0_i32, 0_i32);
        for (row, template) in rows.zip(self.samples.chunks(self.columns)) {
            for (v, t) in row.iter().step_by(self.step).zip(template) {
                let v = i32::from(*v);
                sum += v;
                squares += v * v;
                product += v * t;
            }
        }
        let count = count(self.samples.len());
        let energy = energy(sum, squares, self.samples.len());
        if energy <= 1.0 {
            return 0.0;
        }
        let covariance = f64::from(product) - f64::from(sum) * f64::from(self.sum) / count;
        (covariance / (energy * self.energy).sqrt()) as f32
    }
}

fn count(samples: usize) -> f64 {
    f64::from(u32::try_from(samples).unwrap_or(u32::MAX))
}

/// Sum of squares minus the sum squared over the count: the variance times the count.
fn energy(sum: i32, squares: i32, samples: usize) -> f64 {
    f64::from(squares) - f64::from(sum).powi(2) / count(samples)
}

/// `at` pixels of `side` as a fraction of it.
fn fraction(at: usize, side: usize) -> f32 {
    let pixels = |n: usize| f32::from(u16::try_from(n).unwrap_or(u16::MAX));
    pixels(at) / pixels(side)
}

/// The lattice step that keeps `area` pixels to at most `MAX_SAMPLES` samples.
fn lattice_step(area: usize) -> usize {
    let mut step = 1;
    while area.div_ceil(step * step) > MAX_SAMPLES {
        step += 1;
    }
    step
}

#[cfg(test)]
#[expect(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::manual_midpoint,
    reason = "test pictures are small and synthetic"
)]
mod tests {
    use super::*;

    /// A picture of coarse noise with a bright blob at (`bx`, `by`).
    fn scene(bx: usize, by: usize, gain: f64) -> Vec<u8> {
        crossfade(bx, by, gain, 0.0)
    }

    /// The blob, `change` of the way (0 to 1) to a striped one in the same place.
    fn crossfade(bx: usize, by: usize, gain: f64, change: f64) -> Vec<u8> {
        let mut pixels = vec![0_u8; WIDTH * HEIGHT];
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                let cell = (x / 3) * 7919 + (y / 3) * 104_729;
                let background = 60.0 + f64::from(u8::try_from(cell % 41).unwrap());
                let (dx, dy) = (x as f64 - bx as f64, y as f64 - by as f64);
                let blob = 120.0 * (-(dx * dx / 90.0 + dy * dy / 260.0)).exp();
                let inside = dx.abs() < 12.0 && dy.abs() < 24.0;
                let stripes = if inside && (dy / 4.0).floor() % 2.0 == 0.0 {
                    120.0
                } else {
                    0.0
                };
                let value = background + (1.0 - change) * blob + change * stripes;
                pixels[y * WIDTH + x] = (value * gain).clamp(0.0, 255.0) as u8;
            }
        }
        pixels
    }

    fn settings() -> Settings {
        Settings {
            min_score: 0.6,
            lost_frames: 2,
        }
    }

    fn around(bx: usize, by: usize) -> NormRect {
        let px = |v: usize, side: usize| v as f32 / side as f32;
        NormRect::new(
            px(bx - 12, WIDTH),
            px(by - 24, HEIGHT),
            px(bx + 12, WIDTH),
            px(by + 24, HEIGHT),
        )
        .unwrap()
    }

    #[expect(clippy::unnecessary_wraps, reason = "step takes an optional order")]
    fn order(rect: NormRect) -> Option<Order> {
        Some(Order {
            generation: 1,
            seed: rect,
        })
    }

    fn centre(rect: NormRect) -> (f32, f32) {
        ((rect.x0 + rect.x1) / 2.0, (rect.y0 + rect.y1) / 2.0)
    }

    #[test]
    fn follows_a_target_that_moves_and_changes_brightness_and_then_ends_when_it_leaves() {
        let mut follower = Follower::new(settings());
        let first = scene(160, 90, 1.0);
        let start = follower.step(Luma::new(&first).unwrap(), order(around(160, 90)), &[]);
        assert!(matches!(start, Step::Moved(..)));
        // (blob x, gain, expected step); 10 px a frame is a yaw of 3 % of the picture a frame
        let frames = [(170, 1.0), (180, 0.9), (190, 1.1), (200, 1.0)];
        for (bx, gain) in frames {
            let frame = scene(bx, 90, gain);
            let step = follower.step(Luma::new(&frame).unwrap(), order(around(160, 90)), &[]);
            let Step::Moved(rect, _) = step else {
                panic!("lost at {bx}: {step:?}");
            };
            let expected = bx as f32 / WIDTH as f32;
            assert!((centre(rect).0 - expected).abs() < 0.01, "at {bx}");
        }
        let mut step = Step::Idle;
        for bx in (210..=400).step_by(10) {
            let frame = scene(bx.min(WIDTH + 40), 90, 1.0);
            step = follower.step(Luma::new(&frame).unwrap(), order(around(160, 90)), &[]);
            if matches!(step, Step::Lost(_)) {
                break;
            }
        }
        assert!(
            matches!(step, Step::Lost(Lost::Edge | Lost::Score)),
            "{step:?}"
        );
    }

    #[test]
    fn a_target_that_slowly_changes_its_look_is_still_followed() {
        let mut follower = Follower::new(settings());
        let start = scene(160, 90, 1.0);
        follower.step(Luma::new(&start).unwrap(), order(around(160, 90)), &[]);
        for k in 1..=40 {
            let frame = crossfade(160 + k, 90, 1.0, k as f64 / 40.0);
            let step = follower.step(Luma::new(&frame).unwrap(), order(around(160, 90)), &[]);
            assert!(matches!(step, Step::Moved(..)), "lost at {k}: {step:?}");
        }
    }

    #[test]
    fn a_different_picture_ends_the_follow_after_the_frames_in_a_row_and_a_good_one_resets_them() {
        let mut follower = Follower::new(settings());
        let target = scene(160, 90, 1.0);
        let elsewhere: Vec<u8> = (0..WIDTH * HEIGHT)
            .map(|i| u8::try_from((i * 7919 + i / WIDTH * 31) % 251).unwrap())
            .collect();
        let seed = order(around(160, 90));
        follower.step(Luma::new(&target).unwrap(), seed, &[]);
        let steps: Vec<Step> = [&elsewhere, &target, &elsewhere, &elsewhere]
            .iter()
            .map(|frame| follower.step(Luma::new(frame).unwrap(), seed, &[]))
            .collect();
        assert!(matches!(steps[0], Step::Held(_)));
        assert!(matches!(steps[1], Step::Moved(..)));
        assert!(matches!(steps[2], Step::Held(_)));
        assert_eq!(steps[3], Step::Lost(Lost::Score));
    }

    #[test]
    fn a_box_with_nothing_in_it_is_not_followed() {
        let mut follower = Follower::new(settings());
        let flat = vec![90_u8; WIDTH * HEIGHT];
        let step = follower.step(Luma::new(&flat).unwrap(), order(around(160, 90)), &[]);
        assert_eq!(step, Step::Lost(Lost::Flat));
    }

    #[test]
    fn a_detection_overlapping_more_than_half_replaces_the_seed() {
        let seed = NormRect::new(0.40, 0.40, 0.60, 0.80).unwrap();
        let near = NormRect::new(0.42, 0.38, 0.62, 0.82).unwrap();
        let apart = NormRect::new(0.55, 0.40, 0.75, 0.80).unwrap();
        // (name, detections, expected)
        let cases = [
            ("overlaps", vec![apart, near], near),
            ("overlaps too little", vec![apart], seed),
            ("none", vec![], seed),
        ];
        for (name, detections, expected) in cases {
            assert_eq!(snap(seed, &detections), expected, "{name}");
        }
    }
}
