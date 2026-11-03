//! Where the pilot's detail inset goes in the ground's picture, how big it is and when it shows.
//! The inset is a retained crop of the pilot's box, so it is old by construction; it appears only
//! beside a picture that claims to be current, carries its own age and a frame, and never covers
//! the stamp rows.

use crate::region::{NormRect, PixelRect};
use sat_teleop_video_core::freshness::State;
use sat_teleop_video_core::stamp::state_rows_px;

/// A crop older than this is not shown.
pub const MAX_AGE_MS: u64 = 30_000;
/// Thickness of the frame around the inset.
pub const FRAME_PX: usize = 2;
/// Gap between the inset and the picture's right edge, and below the stamp rows. Even.
pub const MARGIN_PX: usize = 8;
/// Height reserved above the frame for the label (one line of the overlay's font). Even.
pub const LABEL_PX: usize = 32;
/// The crop is at most this fraction of the picture's width, and of its height, wide and tall.
const MAX_WIDTH_DIV: usize = 3;
const MAX_HEIGHT_DIV: usize = 2;
/// How much of the box's side each side of the crop adds, so a small drift stays inside.
const WIDEN: f32 = 0.1;

/// Where the inset goes in the picture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Placement {
    /// Top of the label's row; the label is right aligned to the frame.
    pub label_y: usize,
    /// The frame, drawn around the image.
    pub frame: PixelRect,
    /// The crop's pixels. Every coordinate is even, so chroma planes line up.
    pub image: PixelRect,
}

/// `rect` grown by 10 % of its sides on each side and clamped to the picture: what the ground asks
/// the vehicle to crop.
pub fn widen(rect: NormRect) -> NormRect {
    let (dx, dy) = ((rect.x1 - rect.x0) * WIDEN, (rect.y1 - rect.y0) * WIDEN);
    NormRect {
        x0: (rect.x0 - dx).max(0.0),
        y0: (rect.y0 - dy).max(0.0),
        x1: (rect.x1 + dx).min(1.0),
        y1: (rect.y1 + dy).min(1.0),
    }
}

/// The size a `crop` of this many pixels is scaled to for a `picture`: as large as fits in a third
/// of the picture's width and half its height, in the crop's aspect, both sides even. `None` for
/// an empty crop or a picture too small to hold one.
pub fn fit(crop: (u32, u32), picture: (usize, usize)) -> Option<(usize, usize)> {
    let (cw, ch) = (usize::try_from(crop.0).ok()?, usize::try_from(crop.1).ok()?);
    let (max_w, max_h) = (picture.0 / MAX_WIDTH_DIV, picture.1 / MAX_HEIGHT_DIV);
    if cw == 0 || ch == 0 {
        return None;
    }
    // Compare max_w / cw with max_h / ch without division.
    let (w, h) = if max_w * ch <= max_h * cw {
        (max_w, (max_w * ch) / cw)
    } else {
        ((max_h * cw) / ch, max_h)
    };
    let (w, h) = (w & !1, h & !1);
    (w >= 2 && h >= 2).then_some((w, h))
}

/// The label: the crop's capture age, to a tenth of a second.
pub fn label(age_ms: u64) -> String {
    let tenths = age_ms / 100;
    format!("DETAIL {}.{} s", tenths / 10, tenths % 10)
}

/// Top of the label's row in a picture `width` wide: right below the stamp rows.
pub fn label_y(width: usize) -> usize {
    (state_rows_px(width) + MARGIN_PX).next_multiple_of(2)
}

/// Where to draw an inset of `inset` pixels in a `picture` showing `state`, or `None` when it must
/// not be drawn: the picture is not current (anything but `Live` or `LowRate`), the crop is
/// `MAX_AGE_MS` old or more, or it does not fit below the stamp rows.
pub fn place(
    state: State,
    age_ms: u64,
    inset: (usize, usize),
    picture: (usize, usize),
) -> Option<Placement> {
    if !matches!(state, State::Live | State::LowRate) || age_ms >= MAX_AGE_MS {
        return None;
    }
    let (picture_w, picture_h) = (picture.0 & !1, picture.1);
    let label_y = label_y(picture.0);
    let image_y0 = label_y + LABEL_PX + FRAME_PX;
    let image_x1 = picture_w.checked_sub(MARGIN_PX + FRAME_PX)?;
    let image = PixelRect {
        x0: image_x1.checked_sub(inset.0)?,
        y0: image_y0,
        x1: image_x1,
        y1: image_y0 + inset.1,
    };
    if inset.0 == 0 || inset.1 == 0 || image.y1 + FRAME_PX > picture_h {
        return None;
    }
    let frame = PixelRect {
        x0: image.x0 - FRAME_PX,
        y0: image.y0 - FRAME_PX,
        x1: image.x1 + FRAME_PX,
        y1: image.y1 + FRAME_PX,
    };
    Some(Placement {
        label_y,
        frame,
        image,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const PICTURE: (usize, usize) = (640, 360);
    const INSET: (usize, usize) = (200, 120);

    #[test]
    fn shows_only_beside_a_current_picture_and_a_young_crop() {
        let cases = [
            ("live", State::Live, 3_400, true),
            ("low rate", State::LowRate, 3_400, true),
            ("just under the limit", State::Live, MAX_AGE_MS - 1, true),
            ("at the limit", State::Live, MAX_AGE_MS, false),
            ("stale", State::Stale, 3_400, false),
            ("frozen", State::Frozen, 3_400, false),
            ("disconnected", State::Disconnected, 3_400, false),
            ("age unknown", State::AgeUnknown, 3_400, false),
        ];
        for (name, state, age_ms, shown) in cases {
            assert_eq!(
                place(state, age_ms, INSET, PICTURE).is_some(),
                shown,
                "{name}"
            );
        }
    }

    #[test]
    fn sits_top_right_below_the_stamp_rows_inside_the_picture() {
        let placed = place(State::Live, 0, INSET, PICTURE).unwrap();
        assert!(placed.label_y >= state_rows_px(PICTURE.0));
        assert!(placed.frame.y0 >= placed.label_y + LABEL_PX);
        assert_eq!(placed.frame.x1, PICTURE.0 - MARGIN_PX);
        assert!(placed.frame.y1 <= PICTURE.1);
        assert_eq!(placed.image.x1 - placed.image.x0, INSET.0);
        assert_eq!(placed.image.y1 - placed.image.y0, INSET.1);
        assert_eq!(placed.image.x0 - placed.frame.x0, FRAME_PX);
        for edge in [
            placed.image.x0,
            placed.image.y0,
            placed.image.x1,
            placed.image.y1,
        ] {
            assert_eq!(edge % 2, 0, "{placed:?}");
        }
        // The age label is bottom left; the inset stays on the right.
        assert!(placed.frame.x0 > PICTURE.0 / 2);
    }

    #[test]
    fn an_inset_that_does_not_fit_is_not_drawn() {
        assert_eq!(place(State::Live, 0, (200, 400), PICTURE), None);
        assert_eq!(place(State::Live, 0, (700, 100), PICTURE), None);
        assert_eq!(place(State::Live, 0, (0, 100), PICTURE), None);
    }

    #[test]
    fn fits_a_third_of_the_width_in_the_crops_aspect() {
        let cases = [
            ("wide crop", (460, 288), PICTURE, Some((212, 132))),
            (
                "tall crop, height bound",
                (100, 400),
                PICTURE,
                Some((44, 180)),
            ),
            (
                "small crop is enlarged",
                (50, 50),
                PICTURE,
                Some((180, 180)),
            ),
            ("empty", (0, 50), PICTURE, None),
            ("tiny picture", (50, 50), (4, 4), None),
        ];
        for (name, crop, picture, expected) in cases {
            assert_eq!(fit(crop, picture), expected, "{name}");
        }
    }

    #[test]
    fn widens_a_tenth_each_side_and_stays_in_the_picture() {
        let rect = |x0, y0, x1, y1| NormRect { x0, y0, x1, y1 };
        let wide = widen(rect(0.4, 0.4, 0.6, 0.8));
        assert!((wide.x0 - 0.38).abs() < 1e-6 && (wide.x1 - 0.62).abs() < 1e-6);
        assert!((wide.y0 - 0.36).abs() < 1e-6 && (wide.y1 - 0.84).abs() < 1e-6);
        assert_eq!(widen(rect(0.0, 0.0, 1.0, 1.0)), rect(0.0, 0.0, 1.0, 1.0));
    }

    #[test]
    fn labels_the_age_to_a_tenth() {
        assert_eq!(label(3_400), "DETAIL 3.4 s");
        assert_eq!(label(3_499), "DETAIL 3.4 s");
        assert_eq!(label(29_900), "DETAIL 29.9 s");
        assert_eq!(label(0), "DETAIL 0.0 s");
    }
}
