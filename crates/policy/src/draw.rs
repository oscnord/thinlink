use thinlink_video_core::stamp::PlaneGeometry;

use crate::region::PixelRect;

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

/// Draws the kind's letter in a solid tab of `colour` at the top-left corner of `rect`: the glyph's
/// 5 x 7 pixels each `cell` pixels square, in `ink`, on a margin of one cell. Drawn only when the
/// tab fits inside the rectangle, so a letter never swamps a small box.
pub fn tag(
    plane: &mut [u8],
    geometry: PlaneGeometry,
    rect: PixelRect,
    glyph: [u8; 7],
    cell: usize,
    colour: u8,
    ink: u8,
) {
    let (width, height) = (7 * cell, 9 * cell);
    if geometry.stride == 0 || rect.x1 < rect.x0 + width || rect.y1 < rect.y0 + height {
        return;
    }
    let right = (rect.x0 + width).min(geometry.width);
    let rows = plane
        .chunks_mut(geometry.stride)
        .enumerate()
        .take((rect.y0 + height).min(geometry.height))
        .skip(rect.y0);
    for (y, row) in rows {
        fill(row, rect.x0, right, colour);
        let glyph_row = (y - rect.y0) / cell;
        let Some(bits) = glyph_row.checked_sub(1).and_then(|r| glyph.get(r)) else {
            continue;
        };
        for column in (0..5).filter(|c| bits >> (4 - c) & 1 == 1) {
            let from = rect.x0 + (column + 1) * cell;
            fill(row, from, (from + cell).min(right), ink);
        }
    }
}

fn fill(row: &mut [u8], from: usize, to: usize, value: u8) {
    if let Some(span) = row.get_mut(from..to) {
        span.fill(value);
    }
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
