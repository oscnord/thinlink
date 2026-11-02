use crate::{
    region::{NormRect, PixelRect},
    stamp::PlaneGeometry,
};

/// Replaces every `factor` x `factor` block of the plane that does not overlap a rectangle in
/// `keep` with its average. Blocks start at the plane's origin; a partial block at the right or
/// bottom edge averages the pixels it has. `factor` of 1 or no rectangle changes nothing.
pub fn soften_plane(plane: &mut [u8], geometry: PlaneGeometry, keep: &[PixelRect], factor: usize) {
    if factor <= 1 || keep.is_empty() || geometry.width == 0 {
        return;
    }
    let band = geometry.stride.saturating_mul(factor);
    if band == 0 {
        return;
    }
    for (band_index, band_rows) in plane
        .chunks_mut(band)
        .take(geometry.height.div_ceil(factor))
        .enumerate()
    {
        let y0 = band_index * factor;
        let y1 = (y0 + factor).min(geometry.height);
        for x0 in (0..geometry.width).step_by(factor) {
            let x1 = (x0 + factor).min(geometry.width);
            if keep.iter().any(|k| overlaps(k, x0, y0, x1, y1)) {
                continue;
            }
            average_block(band_rows, geometry.stride, x0, x1);
        }
    }
}

fn overlaps(keep: &PixelRect, x0: usize, y0: usize, x1: usize, y1: usize) -> bool {
    x0 < keep.x1 && keep.x0 < x1 && y0 < keep.y1 && keep.y0 < y1
}

/// Sets columns `x0..x1` of every row in `rows` (rows of `stride` bytes) to their mean.
fn average_block(rows: &mut [u8], stride: usize, x0: usize, x1: usize) {
    let (sum, count) = rows
        .chunks(stride)
        .filter_map(|row| row.get(x0..x1))
        .flatten()
        .fold((0usize, 0usize), |(sum, count), &p| {
            (sum + usize::from(p), count + 1)
        });
    if count == 0 {
        return;
    }
    let mean = u8::try_from((sum + count / 2) / count).unwrap_or(u8::MAX);
    for row in rows.chunks_mut(stride) {
        if let Some(block) = row.get_mut(x0..x1) {
            block.fill(mean);
        }
    }
}

/// Draws the border of `rect`, `thickness` pixels wide and inside the rectangle, clipped to the
/// plane.
pub fn outline(
    plane: &mut [u8],
    geometry: PlaneGeometry,
    rect: PixelRect,
    thickness: usize,
    value: u8,
) {
    if geometry.stride == 0 {
        return;
    }
    let right = rect.x1.min(geometry.width);
    let rows = plane
        .chunks_mut(geometry.stride)
        .enumerate()
        .take(rect.y1.min(geometry.height))
        .skip(rect.y0);
    for (y, row) in rows {
        let horizontal = y < rect.y0 + thickness || y + thickness >= rect.y1;
        if horizontal {
            fill(row, rect.x0, right, value);
            continue;
        }
        fill(row, rect.x0, (rect.x0 + thickness).min(right), value);
        fill(
            row,
            rect.x1.saturating_sub(thickness).max(rect.x0),
            right,
            value,
        );
    }
}

fn fill(row: &mut [u8], from: usize, to: usize, value: u8) {
    if let Some(span) = row.get_mut(from..to) {
        span.fill(value);
    }
}

/// Softens an I420 frame outside `regions`: `factor` on luma, half of it (at least 1) on the
/// half-size chroma planes, which keeps the softened area equally soft on screen.
pub fn soften_i420(
    planes: [&mut [u8]; 3],
    geometries: [PlaneGeometry; 3],
    regions: &[NormRect],
    factor: usize,
) {
    for (index, (plane, geometry)) in planes.into_iter().zip(geometries).enumerate() {
        soften_i420_plane(index, plane, geometry, regions, factor);
    }
}

/// [`soften_i420`] for plane `index` (0 luma, 1 and 2 chroma) alone, for a caller that can borrow
/// only one plane of a frame at a time.
pub fn soften_i420_plane(
    index: usize,
    plane: &mut [u8],
    geometry: PlaneGeometry,
    regions: &[NormRect],
    factor: usize,
) {
    let keep: Vec<PixelRect> = regions
        .iter()
        .map(|r| r.pixels(geometry.width, geometry.height))
        .collect();
    let factor = if index == 0 {
        factor
    } else {
        (factor / 2).max(1)
    };
    soften_plane(plane, geometry, &keep, factor);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geometry(width: usize, height: usize, stride: usize) -> PlaneGeometry {
        PlaneGeometry {
            width,
            height,
            stride,
        }
    }

    fn checkerboard(width: usize, height: usize) -> Vec<u8> {
        (0..width * height)
            .map(|i| {
                if (i % width + i / width).is_multiple_of(2) {
                    0
                } else {
                    200
                }
            })
            .collect()
    }

    #[test]
    fn outside_is_block_averaged_inside_is_untouched() {
        let original = checkerboard(8, 8);
        let mut plane = original.clone();
        let keep = [PixelRect {
            x0: 0,
            y0: 0,
            x1: 4,
            y1: 4,
        }];
        soften_plane(&mut plane, geometry(8, 8, 8), &keep, 2);
        for (i, (&got, &was)) in plane.iter().zip(&original).enumerate() {
            let inside = i % 8 < 4 && i / 8 < 4;
            assert_eq!(got, if inside { was } else { 100 }, "pixel {i}");
        }
    }

    #[test]
    fn factor_one_or_no_region_changes_nothing() {
        let original = checkerboard(8, 8);
        let keep = [PixelRect {
            x0: 0,
            y0: 0,
            x1: 2,
            y1: 2,
        }];
        let mut plane = original.clone();
        soften_plane(&mut plane, geometry(8, 8, 8), &keep, 1);
        soften_plane(&mut plane, geometry(8, 8, 8), &[], 2);
        assert_eq!(plane, original);
    }

    #[test]
    fn partial_edge_blocks_and_padding() {
        // 5x3 plane in a stride of 6: factor 2 leaves a 1 pixel wide and a 1 row high block.
        let mut plane = vec![
            0, 100, 0, 100, 40, 9, 100, 0, 100, 0, 80, 9, 50, 50, 50, 50, 60, 9,
        ];
        let keep = [PixelRect {
            x0: 4,
            y0: 2,
            x1: 5,
            y1: 3,
        }];
        soften_plane(&mut plane, geometry(5, 3, 6), &keep, 2);
        assert_eq!(
            plane,
            vec![
                50, 50, 50, 50, 60, 9, 50, 50, 50, 50, 60, 9, 50, 50, 50, 50, 60, 9
            ]
        );
    }

    #[test]
    fn same_region_at_two_sizes() {
        let region = NormRect {
            x0: 0.25,
            y0: 0.25,
            x1: 0.5,
            y1: 0.5,
        };
        // (width, height, unchanged columns, unchanged rows): blocks of 2 pad the rows at 36 high.
        let cases = [(64, 36, 16..32, 8..18), (128, 72, 32..64, 18..36)];
        for (w, h, columns, rows) in cases {
            let mut plane = checkerboard(w, h);
            let keep = [region.pixels(w, h)];
            soften_plane(&mut plane, geometry(w, h, w), &keep, 2);
            for (i, &p) in plane.iter().enumerate() {
                let inside = columns.contains(&(i % w)) && rows.contains(&(i / w));
                assert_eq!(p != 100, inside, "{w}x{h} pixel {i}");
            }
        }
    }

    #[test]
    fn outline_is_clipped() {
        let (width, height, stride) = (8, 8, 10);
        let mut plane = vec![0u8; stride * height];
        let rect = PixelRect {
            x0: 4,
            y0: 1,
            x1: 12,
            y1: 7,
        };
        outline(&mut plane, geometry(width, height, stride), rect, 2, 255);
        for (i, &got) in plane.iter().enumerate() {
            let (x, y) = (i % stride, i / stride);
            let drawn = x >= 4
                && x < width
                && ((1..3).contains(&y) || (5..7).contains(&y) || (3..5).contains(&y) && x < 6);
            assert_eq!(got, if drawn { 255 } else { 0 }, "pixel {x},{y}");
        }
    }
}
