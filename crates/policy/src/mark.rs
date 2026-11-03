//! Draws the predictive display into the luma plane of a frame: a hollow, dashed circle at the aim
//! point and a dashed horizon line. Both are outlines in dashes so they cannot be taken for part of
//! the scene. They stay below the stamp rows, which the lab reads back.

use crate::predict::{Aim, Horizon};
use sat_teleop_video_core::stamp::{PlaneGeometry, STATE_ROWS, StampError, validate};

const LUMA: u8 = 235;
/// The circle is this many dashes and gaps; the horizon's dash and gap are this long in pixels.
const CIRCLE_SEGMENTS: f32 = 16.0;
const LINE_DASH_PX: f32 = 12.0;
/// The ring is this far either side of the radius, so about two pixels thick.
const RING_HALF_WIDTH: f32 = 1.0;
const MIN_RADIUS: usize = 6;
/// The radius is the shorter side over this.
const RADIUS_DIVISOR: usize = 32;

/// A luma plane with the rows the stamps own set apart.
struct Canvas<'a> {
    plane: &'a mut [u8],
    geometry: PlaneGeometry,
    first_row: usize,
}

impl Canvas<'_> {
    fn put(&mut self, x: usize, y: usize) {
        let PlaneGeometry {
            width,
            height,
            stride,
        } = self.geometry;
        if x >= width || y >= height || y < self.first_row {
            return;
        }
        if let Some(pixel) = self.plane.get_mut(y * stride + x) {
            *pixel = LUMA;
        }
    }

    /// A 2 by 2 square from (`x`, `y`), the line's thickness.
    fn dot(&mut self, x: usize, y: usize) {
        for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            self.put(x + dx, y + dy);
        }
    }
}

/// Draws `aim` and, if there is one, `horizon` into `plane`.
///
/// # Errors
/// The plane cannot carry the stamps (`validate`): the mark stays below their rows.
pub fn draw(
    plane: &mut [u8],
    geometry: PlaneGeometry,
    aim: Aim,
    horizon: Option<Horizon>,
) -> Result<(), StampError> {
    let side = validate(geometry, plane.len(), STATE_ROWS)?;
    let mut canvas = Canvas {
        plane,
        geometry,
        first_row: side * STATE_ROWS,
    };
    if let Some((from, to)) = horizon {
        dashed_line(&mut canvas, from, to);
    }
    dashed_circle(&mut canvas, aim);
    Ok(())
}

fn dashed_circle(canvas: &mut Canvas, aim: Aim) {
    let PlaneGeometry { width, height, .. } = canvas.geometry;
    let radius = to_f32(width.min(height) / RADIUS_DIVISOR).max(to_f32(MIN_RADIUS));
    let reach = radius + RING_HALF_WIDTH;
    let (x0, y0) = (to_px(aim.x - reach), to_px(aim.y - reach));
    let size = to_px(2.0 * reach) + 1;
    for y in y0..=y0 + size {
        for x in x0..=x0 + size {
            let (dx, dy) = (to_f32(x) - aim.x, to_f32(y) - aim.y);
            let on_ring = (dx.hypot(dy) - radius).abs() <= RING_HALF_WIDTH;
            if on_ring && dash_on(dy.atan2(dx)) {
                canvas.put(x, y);
            }
        }
    }
}

/// Every other 1/16 of the circle.
fn dash_on(angle: f32) -> bool {
    let turn = (angle + std::f32::consts::PI) / std::f32::consts::TAU;
    to_px(turn * CIRCLE_SEGMENTS).is_multiple_of(2)
}

fn dashed_line(canvas: &mut Canvas, from: (f32, f32), to: (f32, f32)) {
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);
    let length = dx.hypot(dy);
    let mut at = 0.0;
    while at <= length {
        let (x, y) = (from.0 + dx * at / length, from.1 + dy * at / length);
        if to_px(at / LINE_DASH_PX).is_multiple_of(2) {
            canvas.dot(to_px(x), to_px(y));
        }
        at += 1.0;
    }
}

#[expect(
    clippy::cast_precision_loss,
    reason = "picture sizes are far below 2^24"
)]
fn to_f32(v: usize) -> f32 {
    v as f32
}

/// Rounds towards zero, and a negative or non-finite value to 0.
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "a float coordinate to a pixel index, saturating"
)]
fn to_px(v: f32) -> usize {
    v as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    const GEOMETRY: PlaneGeometry = PlaneGeometry {
        width: 192,
        height: 144,
        stride: 200,
    };

    fn drawn(aim: Aim, horizon: Option<Horizon>) -> Vec<u8> {
        let mut plane = vec![100_u8; GEOMETRY.stride * GEOMETRY.height];
        draw(&mut plane, GEOMETRY, aim, horizon).unwrap();
        plane
    }

    fn at(plane: &[u8], x: usize, y: usize) -> u8 {
        plane[y * GEOMETRY.stride + x]
    }

    #[test]
    fn the_aim_is_a_dashed_hollow_ring_below_the_stamps() {
        let plane = drawn(Aim { x: 96.0, y: 80.0 }, None);
        let ring = |x: usize, y: usize| {
            let r = (to_f32(x) - 96.0).hypot(to_f32(y) - 80.0);
            (r - 6.0).abs() <= 1.0
        };
        let (mut on, mut off) = (0, 0);
        for y in 60..100 {
            for x in 76..116 {
                match (ring(x, y), at(&plane, x, y) == LUMA) {
                    (true, true) => on += 1,
                    (true, false) => off += 1,
                    (false, drawn) => assert!(!drawn, "pixel outside the ring at {x},{y}"),
                }
            }
        }
        assert!(
            on > 0 && off > 0,
            "ring neither drawn nor dashed: {on} {off}"
        );
        assert_eq!(at(&plane, 96, 80), 100);
    }

    #[test]
    fn nothing_lands_on_the_stamp_rows() {
        let top = Aim { x: 4.0, y: 2.0 };
        let line = Some(((0.0, 0.0), (192.0, 20.0)));
        let plane = drawn(top, line);
        let stamp_rows = 8 * STATE_ROWS;
        for y in 0..stamp_rows {
            assert!(
                plane[y * GEOMETRY.stride..][..GEOMETRY.width]
                    .iter()
                    .all(|&p| p == 100)
            );
        }
    }

    #[test]
    fn the_horizon_is_dashed() {
        let plane = drawn(
            Aim { x: 96.0, y: 130.0 },
            Some(((0.0, 60.0), (191.0, 60.0))),
        );
        let on = (0..GEOMETRY.width)
            .filter(|&x| at(&plane, x, 60) == LUMA)
            .count();
        assert!(on > 40 && on < 150, "{on}");
    }
}
