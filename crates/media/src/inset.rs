//! The pilot's detail inset in the ground's picture: a retained crop decoded and scaled once to the
//! size it is shown at (`Image`), held with its age (`DetailCell`), and blitted into the planes of
//! a frame inside a frame of its own (`Image::draw`). Where and when it shows is decided by
//! `sat_teleop_policy::inset`.

use std::io::Cursor;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Instant;

use image::imageops::{self, FilterType};
use image::{ImageFormat, ImageReader, Limits};

use sat_teleop_policy::inset::{self, FRAME_PX, Placement};
use sat_teleop_policy::stamp::PlaneGeometry;

use crate::MediaError;

/// The most a crop may decode to (a retained snapshot is at most 8192 on a side).
const MAX_SIDE: u32 = 8192;
const MAX_ALLOC: u64 = 256 * 1024 * 1024;
const FRAME_LUMA: u8 = 235;
const NEUTRAL_CHROMA: u8 = 128;

/// A crop as the I420 planes it is drawn from, at the size it is shown at.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Image {
    width: usize,
    height: usize,
    planes: [Vec<u8>; 3],
}

impl Image {
    /// Decodes `jpeg` and scales it to the size `inset::fit` gives for a `picture` of this size.
    pub fn from_jpeg(jpeg: &[u8], picture: (usize, usize)) -> Result<Self, MediaError> {
        let bad = |e: &dyn std::fmt::Display| MediaError::Crop(e.to_string());
        let mut limits = Limits::default();
        limits.max_image_width = Some(MAX_SIDE);
        limits.max_image_height = Some(MAX_SIDE);
        limits.max_alloc = Some(MAX_ALLOC);
        let mut reader = ImageReader::with_format(Cursor::new(jpeg), ImageFormat::Jpeg);
        reader.limits(limits);
        let decoded = reader.decode().map_err(|e| bad(&e))?.to_rgb8();
        let (width, height) = inset::fit(decoded.dimensions(), picture)
            .ok_or_else(|| bad(&"the crop is empty or the picture too small"))?;
        let (w, h) = (
            u32::try_from(width).map_err(|e| bad(&e))?,
            u32::try_from(height).map_err(|e| bad(&e))?,
        );
        let scaled = imageops::resize(&decoded, w, h, FilterType::Triangle);
        Ok(Self::from_rgb(width, height, scaled.as_raw()))
    }

    /// `rgb` is `width` x `height` pixels, both even, three bytes each.
    fn from_rgb(width: usize, height: usize, rgb: &[u8]) -> Self {
        let pixel = |x: usize, y: usize| -> [i32; 3] {
            let at = (y * width + x) * 3;
            std::array::from_fn(|c| rgb.get(at + c).copied().map_or(0, i32::from))
        };
        let clamp = |v: i32| u8::try_from(v.clamp(0, 255)).unwrap_or(u8::MAX);
        let mut y_plane = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                let [r, g, b] = pixel(x, y);
                y_plane.push(clamp(16 + ((66 * r + 129 * g + 25 * b + 128) >> 8)));
            }
        }
        let (mut u_plane, mut v_plane) = (Vec::new(), Vec::new());
        for y in (0..height).step_by(2) {
            for x in (0..width).step_by(2) {
                let mean: [i32; 3] = std::array::from_fn(|c| {
                    let sum: i32 = [(0, 0), (1, 0), (0, 1), (1, 1)]
                        .iter()
                        .map(|(dx, dy)| pixel(x + dx, y + dy).get(c).copied().unwrap_or(0))
                        .sum();
                    (sum + 2) / 4
                });
                let [r, g, b] = mean;
                u_plane.push(clamp(128 + ((-38 * r - 74 * g + 112 * b + 128) >> 8)));
                v_plane.push(clamp(128 + ((112 * r - 94 * g - 18 * b + 128) >> 8)));
            }
        }
        Self {
            width,
            height,
            planes: [y_plane, u_plane, v_plane],
        }
    }

    pub fn size(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Draws the frame and the crop into plane `index` (0 luma, 1 and 2 chroma) of an I420 frame at
    /// `placed`, which `inset::place` made for this image's size. Outside the frame nothing changes.
    pub fn draw(&self, index: usize, data: &mut [u8], geometry: PlaneGeometry, placed: Placement) {
        let shift = usize::from(index > 0);
        let level = if index == 0 {
            FRAME_LUMA
        } else {
            NEUTRAL_CHROMA
        };
        let Some(source) = self.planes.get(index) else {
            return;
        };
        let source_width = self.width >> shift;
        let image = placed.image;
        for y in (placed.frame.y0 >> shift)..(placed.frame.y1 >> shift) {
            let inside_y = (image.y0 >> shift..image.y1 >> shift).contains(&y);
            let Some(row) = data
                .get_mut(y * geometry.stride..)
                .and_then(|r| r.get_mut(..geometry.width))
            else {
                continue;
            };
            let Some(span) = row.get_mut(placed.frame.x0 >> shift..placed.frame.x1 >> shift) else {
                continue;
            };
            span.fill(level);
            if !inside_y {
                continue;
            }
            let from = (y - (image.y0 >> shift)) * source_width;
            let (Some(pixels), Some(target)) = (
                source.get(from..from + source_width),
                span.get_mut(FRAME_PX >> shift..(FRAME_PX >> shift) + source_width),
            ) else {
                continue;
            };
            target.copy_from_slice(pixels);
        }
    }
}

/// The newest crop and how old it was when it arrived.
#[derive(Default)]
pub struct DetailCell {
    held: Mutex<Option<Held>>,
}

struct Held {
    image: Arc<Image>,
    age_ms: u64,
    at: Instant,
}

impl DetailCell {
    /// Holds `image`, captured `age_ms` ago, in place of any before.
    pub fn put(&self, image: Image, age_ms: u64) {
        *self.lock() = Some(Held {
            image: Arc::new(image),
            age_ms,
            at: Instant::now(),
        });
    }

    pub fn clear(&self) {
        *self.lock() = None;
    }

    /// The crop and its capture age at `now`: the age it arrived with plus the time held, so a
    /// step of the wall clock never makes it younger.
    pub fn current(&self, now: Instant) -> Option<(Arc<Image>, u64)> {
        let held = self.lock();
        let held = held.as_ref()?;
        let held_for = now.saturating_duration_since(held.at).as_millis();
        let age_ms = held
            .age_ms
            .saturating_add(u64::try_from(held_for).unwrap_or(u64::MAX));
        Some((Arc::clone(&held.image), age_ms))
    }

    fn lock(&self) -> MutexGuard<'_, Option<Held>> {
        self.held.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

#[cfg(test)]
mod tests {
    use sat_teleop_policy::freshness::State;

    use super::*;

    const PICTURE: (usize, usize) = (640, 360);

    fn jpeg(width: u32, height: u32) -> Vec<u8> {
        let rgb = image::RgbImage::from_fn(width, height, |x, _| {
            if x < width / 2 {
                image::Rgb([200, 40, 40])
            } else {
                image::Rgb([40, 40, 200])
            }
        });
        let mut out = Vec::new();
        rgb.write_to(&mut Cursor::new(&mut out), ImageFormat::Jpeg)
            .unwrap();
        out
    }

    #[test]
    fn draws_the_crop_inside_its_frame_and_nothing_outside() {
        let image = Image::from_jpeg(&jpeg(460, 288), PICTURE).unwrap();
        assert_eq!(image.size(), (212, 132));
        let placed = inset::place(State::Live, 0, image.size(), PICTURE).unwrap();
        let mut planes = [
            vec![0_u8; PICTURE.0 * PICTURE.1],
            vec![0_u8; PICTURE.0 * PICTURE.1 / 4],
            vec![0_u8; PICTURE.0 * PICTURE.1 / 4],
        ];
        for (index, data) in planes.iter_mut().enumerate() {
            let shift = usize::from(index > 0);
            let geometry = PlaneGeometry {
                width: PICTURE.0 >> shift,
                height: PICTURE.1 >> shift,
                stride: PICTURE.0 >> shift,
            };
            image.draw(index, data, geometry, placed);
        }
        let luma = |x: usize, y: usize| planes[0][y * PICTURE.0 + x];
        let frame = placed.frame;
        assert_eq!(luma(frame.x0, frame.y0), FRAME_LUMA);
        assert_eq!(luma(frame.x1 - 1, frame.y1 - 1), FRAME_LUMA);
        // Red on the left half, blue on the right: different luma, and not the frame's.
        let (left, right) = (
            luma(placed.image.x0 + 10, placed.image.y0 + 10),
            luma(placed.image.x1 - 10, placed.image.y0 + 10),
        );
        assert_ne!(left, right);
        assert_ne!(left, FRAME_LUMA);
        // Chroma differs between the halves too, and the picture outside the frame is untouched.
        let chroma = |x: usize, y: usize| planes[2][(y / 2) * (PICTURE.0 / 2) + x / 2];
        assert_ne!(
            chroma(placed.image.x0 + 10, placed.image.y0 + 10),
            chroma(placed.image.x1 - 10, placed.image.y0 + 10)
        );
        assert_eq!(luma(frame.x0 - 1, frame.y0), 0);
        assert_eq!(luma(frame.x0, frame.y1), 0);
        assert_eq!(luma(frame.x0, frame.y0 - 1), 0);
        let touched = planes[0].iter().filter(|&&p| p != 0).count();
        let area = (frame.x1 - frame.x0) * (frame.y1 - frame.y0);
        assert!(
            touched <= area,
            "{touched} pixels drawn, frame covers {area}"
        );
    }

    #[test]
    fn the_held_crop_ages_with_the_time_held() {
        let cell = DetailCell::default();
        let image = Image::from_jpeg(&jpeg(200, 100), PICTURE).unwrap();
        cell.put(image, 3_000);
        let now = Instant::now();
        let (_, at_once) = cell.current(now).unwrap();
        let (_, later) = cell
            .current(now + std::time::Duration::from_secs(2))
            .unwrap();
        assert!((3_000..3_100).contains(&at_once), "{at_once}");
        assert!(later >= 5_000, "{later}");
        cell.clear();
        assert!(cell.current(now).is_none());
    }
}
