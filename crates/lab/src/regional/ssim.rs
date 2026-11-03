//! Structural similarity of a received luma plane against its reference, in 8x8 windows every 4
//! pixels, and how much of the reference's local contrast survived. The windows are scored once;
//! `Region` then averages the ones inside and the ones outside a box.

use sat_teleop_policy::region::PixelRect;

const WINDOW: usize = 8;
const STEP: usize = 4;
/// SSIM's stabilising constants for 8-bit pixels: (0.01 * 255)^2 and (0.03 * 255)^2.
const C1: f64 = 6.5025;
const C2: f64 = 58.5225;

/// One window: its top-left corner, its SSIM and the standard deviation of the received and the
/// reference pixels in it.
#[derive(Debug, Clone, Copy)]
pub struct Window {
    pub x: usize,
    pub y: usize,
    pub ssim: f64,
    pub std_received: f64,
    pub std_reference: f64,
}

/// The windows of two planes of `width` x `height` pixels, rows packed; none if the planes do not
/// have that size.
#[expect(
    clippy::cast_precision_loss,
    reason = "pixel counts and sums stay far below 2^53"
)]
pub fn windows(received: &[u8], reference: &[u8], width: usize, height: usize) -> Vec<Window> {
    if received.len() != width * height || reference.len() != width * height {
        return Vec::new();
    }
    let sums = Sums::new(received, reference, width, height);
    let n = (WINDOW * WINDOW) as f64;
    let mut found = Vec::new();
    for y in (0..=height.saturating_sub(WINDOW)).step_by(STEP) {
        for x in (0..=width.saturating_sub(WINDOW)).step_by(STEP) {
            let [a, b, aa, bb, ab] = sums.over(x, y);
            let (mean_a, mean_b) = (a / n, b / n);
            let var_a = (aa / n - mean_a * mean_a).max(0.0);
            let var_b = (bb / n - mean_b * mean_b).max(0.0);
            let covariance = ab / n - mean_a * mean_b;
            let ssim = ((2.0 * mean_a * mean_b + C1) * (2.0 * covariance + C2))
                / ((mean_a * mean_a + mean_b * mean_b + C1) * (var_a + var_b + C2));
            found.push(Window {
                x,
                y,
                ssim,
                std_received: var_a.sqrt(),
                std_reference: var_b.sqrt(),
            });
        }
    }
    found
}

/// Summed-area tables of the pixels, their squares and their product, so a window costs five
/// lookups whatever its size.
struct Sums {
    stride: usize,
    tables: [Vec<f64>; 5],
}

impl Sums {
    fn new(received: &[u8], reference: &[u8], width: usize, height: usize) -> Self {
        let stride = width + 1;
        let mut tables: [Vec<f64>; 5] = std::array::from_fn(|_| vec![0.0; stride * (height + 1)]);
        for y in 0..height {
            let mut row = [0.0; 5];
            for x in 0..width {
                let a = f64::from(received.get(y * width + x).copied().unwrap_or(0));
                let b = f64::from(reference.get(y * width + x).copied().unwrap_or(0));
                for (sum, value) in row.iter_mut().zip([a, b, a * a, b * b, a * b]) {
                    *sum += value;
                }
                for (table, sum) in tables.iter_mut().zip(row) {
                    let above = table.get(y * stride + x + 1).copied().unwrap_or(0.0);
                    if let Some(cell) = table.get_mut((y + 1) * stride + x + 1) {
                        *cell = above + sum;
                    }
                }
            }
        }
        Self { stride, tables }
    }

    fn over(&self, x: usize, y: usize) -> [f64; 5] {
        let at = |table: &[f64], x: usize, y: usize| table.get(y * self.stride + x).copied();
        std::array::from_fn(|i| {
            let Some(table) = self.tables.get(i) else {
                return 0.0;
            };
            let corner = |dx, dy| at(table, x + dx, y + dy).unwrap_or(0.0);
            corner(WINDOW, WINDOW) - corner(0, WINDOW) - corner(WINDOW, 0) + corner(0, 0)
        })
    }
}

/// A box in pixels, half open, and the rows worth scoring: the stamp is drawn at the top and the
/// ground's label at the bottom.
#[derive(Debug, Clone, Copy)]
pub struct Region {
    pub x0: usize,
    pub y0: usize,
    pub x1: usize,
    pub y1: usize,
    pub rows: (usize, usize),
    /// Windows this close to the box are in neither group: the encoder and the softening blur
    /// across its edge.
    pub margin: usize,
    /// Windows touching this pixel rectangle are in neither group: the ground draws its detail
    /// inset there.
    pub excluded: Option<PixelRect>,
}

/// What a frame scored inside and outside the box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scores {
    pub inside_ssim: f64,
    pub outside_ssim: f64,
    /// The sum of the received windows' standard deviations outside the box, and of the
    /// reference's: their ratio is the share of local contrast kept.
    pub outside_std_received: f64,
    pub outside_std_reference: f64,
}

impl Region {
    /// `None` if either group has no window.
    #[expect(
        clippy::cast_precision_loss,
        reason = "window counts stay far below 2^53"
    )]
    pub fn score(&self, windows: &[Window]) -> Option<Scores> {
        let (mut inside, mut outside) = (Vec::new(), Vec::new());
        for w in windows {
            let (x1, y1) = (w.x + WINDOW, w.y + WINDOW);
            if w.y < self.rows.0 || y1 > self.rows.1 {
                continue;
            }
            if self
                .excluded
                .is_some_and(|e| w.x < e.x1 && x1 > e.x0 && w.y < e.y1 && y1 > e.y0)
            {
                continue;
            }
            if w.x >= self.x0 && x1 <= self.x1 && w.y >= self.y0 && y1 <= self.y1 {
                inside.push(w);
            } else if x1 + self.margin <= self.x0
                || w.x >= self.x1 + self.margin
                || y1 + self.margin <= self.y0
                || w.y >= self.y1 + self.margin
            {
                outside.push(w);
            }
        }
        if inside.is_empty() || outside.is_empty() {
            return None;
        }
        let mean =
            |group: &[&Window]| group.iter().map(|w| w.ssim).sum::<f64>() / group.len() as f64;
        Some(Scores {
            inside_ssim: mean(&inside),
            outside_ssim: mean(&outside),
            outside_std_received: outside.iter().map(|w| w.std_received).sum(),
            outside_std_reference: outside.iter().map(|w| w.std_reference).sum(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIDE: usize = 64;
    const BLOCK: usize = 4;

    /// Rings of rising frequency, in integers.
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

    /// Replaces every block outside the box with its average, as the sender softens.
    fn softened_outside(plane: &[u8], region: &Region) -> Vec<u8> {
        let mut out = plane.to_vec();
        for by in (0..SIDE).step_by(BLOCK) {
            for bx in (0..SIDE).step_by(BLOCK) {
                let inside = bx < region.x1
                    && bx + BLOCK > region.x0
                    && by < region.y1
                    && by + BLOCK > region.y0;
                if inside {
                    continue;
                }
                let cells: Vec<usize> = (0..BLOCK * BLOCK)
                    .map(|i| (by + i / BLOCK) * SIDE + bx + i % BLOCK)
                    .collect();
                let sum: usize = cells.iter().map(|&p| usize::from(plane[p])).sum();
                let mean = u8::try_from(sum / cells.len()).unwrap();
                for p in cells {
                    out[p] = mean;
                }
            }
        }
        out
    }

    #[test]
    fn softening_outside_the_box_costs_ssim_and_contrast_only_outside() {
        let region = Region {
            x0: 16,
            y0: 16,
            x1: 40,
            y1: 40,
            rows: (0, SIDE),
            margin: 4,
            excluded: None,
        };
        let reference = rings();
        let softened = softened_outside(&reference, &region);
        let same = region
            .score(&windows(&reference, &reference, SIDE, SIDE))
            .unwrap();
        let blurred = region
            .score(&windows(&softened, &reference, SIDE, SIDE))
            .unwrap();
        assert!((same.inside_ssim - 1.0).abs() < 1e-9 && (same.outside_ssim - 1.0).abs() < 1e-9);
        assert!((blurred.inside_ssim - 1.0).abs() < 1e-9);
        assert!(blurred.outside_ssim < 0.9, "{}", blurred.outside_ssim);
        assert!(blurred.outside_std_received < blurred.outside_std_reference);

        let covered = Region {
            excluded: Some(PixelRect {
                x0: 40,
                y0: 0,
                x1: SIDE,
                y1: SIDE,
            }),
            ..region
        };
        let windows = windows(&softened, &reference, SIDE, SIDE);
        let with = region.score(&windows).unwrap();
        let without = covered.score(&windows).unwrap();
        assert!(without.outside_std_reference < with.outside_std_reference);
        assert!((without.inside_ssim - with.inside_ssim).abs() < 1e-9);
    }
}
