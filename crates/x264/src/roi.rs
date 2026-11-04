//! Regions of interest as x264's per-macroblock quantiser offsets.

use std::ops::Range;

/// A region in pixels: x, y, width, height, as `GstVideoRegionOfInterestMeta` has it.
pub type Rect = (u32, u32, u32, u32);

/// One offset per 16x16 macroblock in raster order of a `width` x `height` picture: `offset` for
/// every macroblock a region touches (so a partial macroblock counts as inside), 0 elsewhere.
/// Regions that overlap do not add up; a region outside the picture touches nothing.
pub fn quant_offsets(width: u32, height: u32, regions: &[Rect], offset: f32) -> Vec<f32> {
    let (columns, rows) = (width.div_ceil(16), height.div_ceil(16));
    let touched: Vec<_> = regions
        .iter()
        .map(|&(x, y, w, h)| (macroblocks(x, w, columns), macroblocks(y, h, rows)))
        .collect();
    (0..columns * rows)
        .map(|n| {
            let (column, row) = (n % columns, n / columns);
            let inside = touched
                .iter()
                .any(|(across, down)| across.contains(&column) && down.contains(&row));
            if inside { offset } else { 0.0 }
        })
        .collect()
}

/// The macroblocks of a row or column of `count` that the pixels `start..start + len` touch.
fn macroblocks(start: u32, len: u32, count: u32) -> Range<u32> {
    let end = start.saturating_add(len).div_ceil(16).min(count);
    (start / 16).min(end)..end
}

#[cfg(test)]
mod tests {
    use super::{Rect, quant_offsets};

    /// A 48x32 picture is 3x2 macroblocks.
    #[test]
    fn regions_become_the_macroblocks_they_touch() {
        let cases: [(&[Rect], [f32; 6]); 6] = [
            (&[], [0.; 6]),
            // Exactly the first macroblock: its edge is not the next one's.
            (&[(0, 0, 16, 16)], [-3., 0., 0., 0., 0., 0.]),
            // One pixel into the next column and row takes the whole of them.
            (&[(0, 0, 17, 17)], [-3., -3., 0., -3., -3., 0.]),
            // Starts mid-macroblock, runs past the picture's edge.
            (&[(40, 20, 100, 100)], [0., 0., 0., 0., 0., -3.]),
            // Overlap sets the offset, it does not add up.
            (
                &[(0, 0, 32, 16), (16, 0, 32, 16)],
                [-3., -3., -3., 0., 0., 0.],
            ),
            // Outside the picture, and past its edge by a huge size.
            (
                &[(48, 0, 16, 16), (0, 32, 16, 16), (60, 60, u32::MAX, 1)],
                [0.; 6],
            ),
        ];
        for (regions, expected) in cases {
            assert_eq!(
                quant_offsets(48, 32, regions, -3.0),
                expected,
                "{regions:?}"
            );
        }
    }
}
