//! Cropping a retained JPEG: the rectangle a normalised request names, mapped to pixels without
//! ever leaving the picture, and the decode, crop and encode.

use std::io::Cursor;

use image::codecs::jpeg::JpegEncoder;
use image::{ImageFormat, ImageReader, Limits};
use serde::{Deserialize, Serialize};

/// Largest side of a picture the vehicle decodes, and the memory it may take to.
const MAX_SIDE: u32 = 8192;
const MAX_ALLOC: u64 = 256 * 1024 * 1024;
/// Quality of a crop's new JPEG.
const CROP_QUALITY: u8 = 80;
/// A rectangle's sum of origin and size may exceed 1 by this much, for floating-point sums such as
/// 0.7 + 0.3.
const SLACK: f64 = 1e-6;

/// A rectangle of the picture as fractions of its width and height.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Crop {
    pub x: f64,
    pub y: f64,
    pub w: f64,
    pub h: f64,
}

/// A rectangle in pixels, inside the picture.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rect {
    pub x: u32,
    pub y: u32,
    pub w: u32,
    pub h: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CropError {
    #[error("the crop rectangle has a number that is not finite")]
    NotFinite,
    #[error("the crop rectangle lies outside the picture (0 to 1)")]
    OutOfRange,
    #[error("the crop rectangle is empty")]
    Degenerate,
    #[error("the snapshot does not decode: {0}")]
    Decode(String),
    #[error("the crop does not encode: {0}")]
    Encode(String),
}

/// Whether `crop` is a rectangle inside the picture with area; what a request is held to before
/// any picture is known.
pub fn check(crop: Crop) -> Result<(), CropError> {
    let numbers = [crop.x, crop.y, crop.w, crop.h];
    if numbers.iter().any(|n| !n.is_finite()) {
        return Err(CropError::NotFinite);
    }
    if crop.x < 0.0 || crop.y < 0.0 || crop.x > 1.0 || crop.y > 1.0 {
        return Err(CropError::OutOfRange);
    }
    if crop.w <= 0.0 || crop.h <= 0.0 {
        return Err(CropError::Degenerate);
    }
    if crop.x + crop.w > 1.0 + SLACK || crop.y + crop.h > 1.0 + SLACK {
        return Err(CropError::OutOfRange);
    }
    Ok(())
}

/// The pixels of a `width` x `height` picture that `crop` names. Edges are rounded to the nearest
/// pixel and held inside the picture; a rectangle that rounds to no pixels is degenerate.
pub fn pixel_rect(width: u32, height: u32, crop: Crop) -> Result<Rect, CropError> {
    check(crop)?;
    let (left, right) = (scale(crop.x, width), scale(crop.x + crop.w, width));
    let (top, bottom) = (scale(crop.y, height), scale(crop.y + crop.h, height));
    if right <= left || bottom <= top {
        return Err(CropError::Degenerate);
    }
    Ok(Rect {
        x: left,
        y: top,
        w: right - left,
        h: bottom - top,
    })
}

#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "clamped to 0 to size first, and a size is a u32"
)]
fn scale(fraction: f64, size: u32) -> u32 {
    let edge = f64::from(size);
    (fraction * edge).round().clamp(0.0, edge) as u32
}

/// What a job sends: a retained snapshot whole, or a part of it.
pub struct Cropped {
    pub jpeg: Vec<u8>,
    /// The size of the snapshot.
    pub source: (u32, u32),
    /// The part of the snapshot `jpeg` shows; `None` for all of it.
    pub rect: Option<Rect>,
}

fn reader(jpeg: &[u8]) -> ImageReader<Cursor<&[u8]>> {
    let mut limits = Limits::default();
    limits.max_image_width = Some(MAX_SIDE);
    limits.max_image_height = Some(MAX_SIDE);
    limits.max_alloc = Some(MAX_ALLOC);
    let mut reader = ImageReader::with_format(Cursor::new(jpeg), ImageFormat::Jpeg);
    reader.limits(limits);
    reader
}

/// The size of a JPEG, from its header; `None` if it is not one.
pub fn dimensions(jpeg: &[u8]) -> Option<(u32, u32)> {
    reader(jpeg).into_dimensions().ok()
}

/// `stored` as it is without a `crop`; with one, `stored` decoded, cut and encoded again at
/// quality 80.
pub fn cut(stored: Vec<u8>, crop: Option<Crop>) -> Result<Cropped, CropError> {
    let Some(crop) = crop else {
        let source = dimensions(&stored)
            .ok_or_else(|| CropError::Decode("not a JPEG within the size limits".to_owned()))?;
        return Ok(Cropped {
            jpeg: stored,
            source,
            rect: None,
        });
    };
    check(crop)?;
    let picture = reader(&stored)
        .decode()
        .map_err(|e| CropError::Decode(e.to_string()))?;
    let source = (picture.width(), picture.height());
    let rect = pixel_rect(source.0, source.1, crop)?;
    let part = picture.crop_imm(rect.x, rect.y, rect.w, rect.h);
    let mut out = Vec::new();
    JpegEncoder::new_with_quality(&mut out, CROP_QUALITY)
        .encode_image(&part)
        .map_err(|e| CropError::Encode(e.to_string()))?;
    Ok(Cropped {
        jpeg: out,
        source,
        rect: Some(rect),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_a_crop_to_pixels_inside_the_picture() {
        let crop = |x, y, w, h| Crop { x, y, w, h };
        let rect = |x, y, w, h| Ok(Rect { x, y, w, h });
        let cases = [
            (
                "the whole picture",
                crop(0.0, 0.0, 1.0, 1.0),
                rect(0, 0, 1280, 720),
            ),
            (
                "a centred half",
                crop(0.25, 0.25, 0.5, 0.5),
                rect(320, 180, 640, 360),
            ),
            (
                "flush with the far edges",
                crop(0.75, 0.5, 0.25, 0.5),
                rect(960, 360, 320, 360),
            ),
            (
                "a sum that rounds above one",
                crop(0.7, 0.7, 0.3, 0.3),
                rect(896, 504, 384, 216),
            ),
            (
                "one pixel",
                crop(0.5, 0.5, 1.0 / 1280.0, 1.0 / 720.0),
                rect(640, 360, 1, 1),
            ),
            (
                "no width",
                crop(0.5, 0.5, 0.0, 0.5),
                Err(CropError::Degenerate),
            ),
            (
                "no height",
                crop(0.5, 0.5, 0.5, 0.0),
                Err(CropError::Degenerate),
            ),
            (
                "negative width",
                crop(0.5, 0.5, -0.1, 0.5),
                Err(CropError::Degenerate),
            ),
            (
                "under half a pixel",
                crop(0.5, 0.5, 0.0003, 0.5),
                Err(CropError::Degenerate),
            ),
            (
                "left of the picture",
                crop(-0.1, 0.0, 0.5, 0.5),
                Err(CropError::OutOfRange),
            ),
            (
                "above the picture",
                crop(0.0, -0.1, 0.5, 0.5),
                Err(CropError::OutOfRange),
            ),
            (
                "past the right edge",
                crop(0.6, 0.0, 0.5, 0.5),
                Err(CropError::OutOfRange),
            ),
            (
                "past the bottom edge",
                crop(0.0, 0.6, 0.5, 0.5),
                Err(CropError::OutOfRange),
            ),
            (
                "starting beyond the picture",
                crop(1.5, 0.0, 0.1, 0.5),
                Err(CropError::OutOfRange),
            ),
            (
                "not a number",
                crop(f64::NAN, 0.0, 0.5, 0.5),
                Err(CropError::NotFinite),
            ),
            (
                "infinite",
                crop(0.0, 0.0, f64::INFINITY, 0.5),
                Err(CropError::NotFinite),
            ),
        ];
        for (name, crop, expected) in cases {
            assert_eq!(pixel_rect(1280, 720, crop), expected, "{name}");
        }
    }
}
