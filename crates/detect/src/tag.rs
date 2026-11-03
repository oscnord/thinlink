//! Finds one `AprilTag` of the tag36h11 family in a luma frame, with `AprilRobotics`' reference
//! detector (BSD-2-Clause, through the `apriltag` crate).

use apriltag::{Detector, Family, Image};

use crate::DetectError;

/// Where the wanted tag's centre is, as fractions of the frame's width and height from the top left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TagFound {
    pub x: f32,
    pub y: f32,
}

/// The reference detector holds a raw pointer, so it stays on the thread that made it.
pub struct TagDetector {
    detector: Detector,
    id: usize,
}

impl TagDetector {
    /// A detector for tag36h11 number `id`. It takes no bit errors and no decimation: a tag it is not
    /// sure of is not a tag, and the frame's pixels all count.
    pub fn new(id: u16) -> Result<Self, DetectError> {
        let mut detector = Detector::builder()
            .add_family_bits(Family::tag_36h11(), 0)
            .build()
            .map_err(|e| DetectError::Tag(e.to_string()))?;
        detector.set_decimation(1.0);
        detector.set_thread_number(1);
        Ok(Self {
            detector,
            id: usize::from(id),
        })
    }

    /// The wanted tag in `luma`, a `width` x `height` frame of 1 byte a pixel. Nothing when it is not
    /// there, when the frame is not that size, or when the id is in the frame more than once: which
    /// one to land on is then not known.
    pub fn find(&mut self, luma: &[u8], width: u32, height: u32) -> Option<TagFound> {
        let (w, h) = (usize::try_from(width).ok()?, usize::try_from(height).ok()?);
        if w == 0 || h == 0 || luma.len() != w.checked_mul(h)? {
            return None;
        }
        let mut image = Image::zeros_with_stride(w, h, w).ok()?;
        image
            .as_slice_mut()
            .get_mut(..luma.len())?
            .copy_from_slice(luma);
        let mut wanted = self
            .detector
            .detect(&image)
            .into_iter()
            .filter(|d| d.id() == self.id);
        let found = wanted.next()?;
        if wanted.next().is_some() {
            return None;
        }
        let [x, y] = found.center();
        Some(TagFound {
            x: fraction(x, width),
            y: fraction(y, height),
        })
    }
}

#[expect(
    clippy::cast_possible_truncation,
    reason = "a position in a frame of at most a few thousand pixels"
)]
fn fraction(position: f64, extent: u32) -> f32 {
    (position / f64::from(extent)) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tag36h11 number 0, `#` black and `.` white, with its white margin (AprilRobotics/apriltag-imgs).
    const TAG_0: [&str; 10] = [
        "..........",
        ".########.",
        ".#..#.#.#.",
        ".##...#.#.",
        ".##..####.",
        ".#.#.####.",
        ".##.#..##.",
        ".####.###.",
        ".########.",
        "..........",
    ];
    const CELL: usize = 6;
    const FRAME: (usize, usize) = (640, 360);

    /// A grey frame with a tag 0 at each of `corners` (the top left of its margin).
    fn frame(corners: &[(usize, usize)]) -> Vec<u8> {
        let mut luma = vec![128; FRAME.0 * FRAME.1];
        for &(left, top) in corners {
            for (row, line) in TAG_0.iter().enumerate() {
                for (col, cell) in line.bytes().enumerate() {
                    let value = if cell == b'#' { 20 } else { 235 };
                    for y in 0..CELL {
                        let at = (top + row * CELL + y) * FRAME.0 + left + col * CELL;
                        luma[at..at + CELL].fill(value);
                    }
                }
            }
        }
        luma
    }

    fn find(id: u16, corners: &[(usize, usize)]) -> Option<TagFound> {
        let (w, h) = FRAME;
        TagDetector::new(id).unwrap().find(
            &frame(corners),
            u32::try_from(w).unwrap(),
            u32::try_from(h).unwrap(),
        )
    }

    #[test]
    fn only_the_wanted_tag_is_found_and_only_when_it_is_the_only_one() {
        let one = find(0, &[(200, 100)]).unwrap();
        // The margin is 10 cells wide, so the tag's centre is 30 pixels in.
        assert!((one.x - 230.0 / 640.0).abs() < 0.002, "{one:?}");
        assert!((one.y - 130.0 / 360.0).abs() < 0.002, "{one:?}");
        assert_eq!(find(1, &[(200, 100)]), None, "another id");
        assert_eq!(
            find(0, &[(200, 100), (400, 200)]),
            None,
            "two of the same id"
        );
        assert_eq!(find(0, &[]), None, "no tag");
    }
}
