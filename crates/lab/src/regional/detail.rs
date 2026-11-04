//! What the detail inset is scored against: the card's region around the tracked box at the
//! vehicle's full resolution, scaled to the size the inset is shown at. The live picture's same
//! region (the output's pixels around the box) is scaled to that size and scored against the same
//! reference, so the two differ only by what they carry.

use thinlink_detail::crop::{Crop, pixel_rect};
use thinlink_policy::inset::{self, Placement};
use thinlink_policy::region::{NormRect, PixelRect};
use thinlink_video::card::Luma;
use thinlink_video_core::freshness::State;

use super::ssim;

/// A rectangle of a plane in pixels.
#[derive(Debug, Clone, Copy)]
struct Area {
    x: usize,
    y: usize,
    w: usize,
    h: usize,
}

/// Where the inset sits in the ground's output and what to score it against.
pub struct Detail {
    placed: Placement,
    size: (usize, usize),
    reference: Vec<u8>,
    live: Area,
}

impl Detail {
    /// `card` is the card's luma at `source` size. `None` if the box has no crop or the inset has
    /// no room in the `output`.
    pub fn new(
        card: &[u8],
        tracked: [f32; 4],
        source: (usize, usize),
        output: (usize, usize),
    ) -> Option<Self> {
        let [x0, y0, x1, y1] = tracked;
        let wide = inset::widen(NormRect { x0, y0, x1, y1 });
        let crop = Crop {
            x: f64::from(wide.x0),
            y: f64::from(wide.y0),
            w: f64::from(wide.x1 - wide.x0),
            h: f64::from(wide.y1 - wide.y0),
        };
        let rect = pixel_rect(
            u32::try_from(source.0).ok()?,
            u32::try_from(source.1).ok()?,
            crop,
        )
        .ok()?;
        let area = Area {
            x: usize::try_from(rect.x).ok()?,
            y: usize::try_from(rect.y).ok()?,
            w: usize::try_from(rect.w).ok()?,
            h: usize::try_from(rect.h).ok()?,
        };
        let size = inset::fit((rect.w, rect.h), output)?;
        let placed = inset::place(State::Live, 0, size, output)?;
        let live = wide.pixels(output.0, output.1);
        Some(Self {
            placed,
            size,
            reference: resample(card, source.0, area, size),
            live: Area {
                x: live.x0,
                y: live.y0,
                w: live.x1 - live.x0,
                h: live.y1 - live.y0,
            },
        })
    }

    /// The inset with its label row and frame, as a pixel rectangle reaching to the right edge.
    pub fn covered(&self, output_width: usize) -> PixelRect {
        PixelRect {
            x0: self.placed.frame.x0,
            y0: self.placed.label_y,
            x1: output_width,
            y1: self.placed.frame.y1,
        }
    }

    /// Mean SSIM against the reference of the inset in `output`, and of the live picture's region
    /// scaled to the inset's size.
    pub fn scores(&self, output: &Luma) -> Option<(f64, f64)> {
        let image = self.placed.image;
        let shown = resample(
            &output.pixels,
            output.width,
            Area {
                x: image.x0,
                y: image.y0,
                w: image.x1 - image.x0,
                h: image.y1 - image.y0,
            },
            self.size,
        );
        let live = resample(&output.pixels, output.width, self.live, self.size);
        Some((self.mean_ssim(&shown)?, self.mean_ssim(&live)?))
    }

    #[expect(
        clippy::cast_precision_loss,
        reason = "window counts stay far below 2^53"
    )]
    fn mean_ssim(&self, received: &[u8]) -> Option<f64> {
        let windows = ssim::windows(received, &self.reference, self.size.0, self.size.1);
        (!windows.is_empty())
            .then(|| windows.iter().map(|w| w.ssim).sum::<f64>() / windows.len() as f64)
    }
}

/// `area` of a plane `stride` pixels wide, scaled to `size` by averaging the source pixels each
/// target pixel covers (at least one).
fn resample(plane: &[u8], stride: usize, area: Area, size: (usize, usize)) -> Vec<u8> {
    let (width, height) = size;
    let mut out = Vec::with_capacity(width * height);
    for ty in 0..height {
        let (y0, y1) = span(area.y, area.h, ty, height);
        for tx in 0..width {
            let (x0, x1) = span(area.x, area.w, tx, width);
            let mut sum = 0_usize;
            for y in y0..y1 {
                for x in x0..x1 {
                    sum += usize::from(plane.get(y * stride + x).copied().unwrap_or(0));
                }
            }
            let count = (x1 - x0) * (y1 - y0);
            out.push(u8::try_from(sum / count.max(1)).unwrap_or(u8::MAX));
        }
    }
    out
}

/// The source pixels `[from, to)` of an extent of `len` at `start` that target pixel `index` of
/// `count` covers.
fn span(start: usize, len: usize, index: usize, count: usize) -> (usize, usize) {
    let from = start + index * len / count;
    let to = start + ((index + 1) * len).div_ceil(count);
    (from, to.max(from + 1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resampling_averages_what_a_target_pixel_covers() {
        // A 4x2 plane of 0 | 100 | 200 | 40 columns, its right 2x2 and then a 2x1 target.
        let plane = [0, 100, 200, 40, 0, 100, 200, 40];
        let right = Area {
            x: 2,
            y: 0,
            w: 2,
            h: 2,
        };
        assert_eq!(resample(&plane, 4, right, (1, 1)), [120]);
        assert_eq!(resample(&plane, 4, right, (2, 2)), [200, 40, 200, 40]);
        let all = Area {
            x: 0,
            y: 0,
            w: 4,
            h: 2,
        };
        assert_eq!(resample(&plane, 4, all, (2, 1)), [50, 120]);
    }
}
