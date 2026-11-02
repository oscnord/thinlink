//! NanoDet-Plus-m's output tensor as people in picture fractions, and the overlap filter. Pure.

use sat_teleop_policy::region::NormRect;

use crate::Detection;

/// Grid strides of the model's four feature maps, finest first.
const STRIDES: [u16; 4] = [8, 16, 32, 64];
/// A row is 80 COCO class scores (already 0..1) then four sides of `BINS` box-edge logits.
const CLASSES: usize = 80;
const BINS: usize = 8;
const SIDES: usize = 4;
const ROW: usize = CLASSES + SIDES * BINS;
/// COCO class 0.
const PERSON: usize = 0;

/// The people in `output` (the `[1, cells, 112]` tensor, flattened) of a model run on an `input`
/// pixel square picture, scoring at least `min_score`. A tensor of the wrong length gives none.
pub(crate) fn decode(output: &[f32], input: u32, min_score: f32) -> Vec<Detection> {
    let Ok(side) = u16::try_from(input) else {
        return Vec::new();
    };
    let cells: usize = STRIDES
        .iter()
        .map(|&stride| usize::from(side.div_ceil(stride)).pow(2))
        .sum();
    if side == 0 || output.len() != cells * ROW {
        return Vec::new();
    }
    let grid = STRIDES.iter().flat_map(|&stride| {
        let n = side.div_ceil(stride);
        (0..n).flat_map(move |gy| (0..n).map(move |gx| Cell { stride, gx, gy }))
    });
    output
        .as_chunks::<ROW>()
        .0
        .iter()
        .zip(grid)
        .filter_map(|(row, cell)| person(row, cell, side, min_score))
        .collect()
}

/// A grid cell of the feature map of one stride.
#[derive(Clone, Copy)]
struct Cell {
    stride: u16,
    gx: u16,
    gy: u16,
}

fn person(row: &[f32; ROW], cell: Cell, side: u16, min_score: f32) -> Option<Detection> {
    let score = *row.get(PERSON)?;
    if score.is_nan() || score < min_score {
        return None;
    }
    let sides = row.get(CLASSES..)?;
    let reach = |side_index: usize| -> Option<f32> {
        let bins = sides.get(side_index * BINS..(side_index + 1) * BINS)?;
        Some(expected_bin(bins) * f32::from(cell.stride))
    };
    let (left, top, right, bottom) = (reach(0)?, reach(1)?, reach(2)?, reach(3)?);
    let cx = f32::from(cell.gx) * f32::from(cell.stride);
    let cy = f32::from(cell.gy) * f32::from(cell.stride);
    let size = f32::from(side);
    let rect = NormRect::new(
        (cx - left) / size,
        (cy - top) / size,
        (cx + right) / size,
        (cy + bottom) / size,
    )?;
    Some(Detection { rect, score })
}

/// The softmax-weighted mean bin index of one side's logits.
fn expected_bin(logits: &[f32]) -> f32 {
    let top = logits.iter().copied().fold(f32::MIN, f32::max);
    let weights: Vec<f32> = logits.iter().map(|l| (l - top).exp()).collect();
    let total: f32 = weights.iter().sum();
    (0_u8..)
        .zip(&weights)
        .map(|(bin, w)| f32::from(bin) * w / total)
        .sum()
}

/// The detections with every one that overlaps a higher scoring one by more than `iou` removed,
/// best first.
pub fn non_max_suppression(mut found: Vec<Detection>, iou: f32) -> Vec<Detection> {
    found.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut kept: Vec<Detection> = Vec::new();
    for detection in found {
        if kept
            .iter()
            .all(|k| overlap(&k.rect, &detection.rect) <= iou)
        {
            kept.push(detection);
        }
    }
    kept
}

/// Intersection over union of two rectangles.
fn overlap(a: &NormRect, b: &NormRect) -> f32 {
    let width = (a.x1.min(b.x1) - a.x0.max(b.x0)).max(0.0);
    let height = (a.y1.min(b.y1) - a.y0.max(b.y0)).max(0.0);
    let both = width * height;
    both / (a.area() + b.area() - both)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 64 pixel picture has 8x8, 4x4, 2x2 and 1x1 cells: 85 rows.
    const SIDE: u32 = 64;
    const CELLS: usize = 85;

    fn detection(rect: NormRect, score: f32) -> Detection {
        Detection { rect, score }
    }

    fn tensor(rows: &[(usize, usize, f32)]) -> Vec<f32> {
        // (row, class, score); the sides of every scored row reach two bins out.
        let mut out = vec![0.0; CELLS * ROW];
        for &(row, class, score) in rows {
            out[row * ROW + class] = score;
            for side in 0..SIDES {
                out[row * ROW + CLASSES + side * BINS + 2] = 30.0;
            }
        }
        out
    }

    #[test]
    fn only_a_person_scoring_enough_is_decoded_in_picture_fractions() {
        // Row 19 is cell (3, 2) of the stride 8 map: centre (24, 16), reaching 2 x 8 = 16 each way.
        let output = tensor(&[(19, 0, 0.9), (20, 1, 0.95), (21, 0, 0.2)]);
        let found = decode(&output, SIDE, 0.4);
        let wanted = NormRect {
            x0: 0.125,
            y0: 0.0,
            x1: 0.625,
            y1: 0.5,
        };
        assert_eq!(found.len(), 1);
        let person = found.first().unwrap();
        assert_eq!(person.score, 0.9);
        let close = |a: f32, b: f32| (a - b).abs() < 1e-3;
        assert!(close(person.rect.x0, wanted.x0) && close(person.rect.x1, wanted.x1));
        assert!(close(person.rect.y0, wanted.y0) && close(person.rect.y1, wanted.y1));
    }

    #[test]
    fn a_tensor_of_the_wrong_size_finds_nobody() {
        assert_eq!(decode(&[0.9; ROW], SIDE, 0.4), Vec::new());
    }

    #[test]
    fn overlapping_boxes_keep_the_better_one() {
        let first = NormRect::new(0.0, 0.0, 0.5, 0.5).unwrap();
        let same_person = NormRect::new(0.0, 0.0, 0.5, 0.4).unwrap();
        let other_person = NormRect::new(0.35, 0.0, 0.85, 0.5).unwrap();
        let near = non_max_suppression(
            vec![detection(same_person, 0.6), detection(first, 0.8)],
            0.5,
        );
        assert_eq!(near, vec![detection(first, 0.8)]);
        let apart = non_max_suppression(
            vec![detection(other_person, 0.6), detection(first, 0.8)],
            0.5,
        );
        assert_eq!(apart.len(), 2);
    }
}
