use thiserror::Error;

use crate::freshness::State;

const COLUMNS: usize = 24;
const ROWS: usize = 3;
const BITS: usize = COLUMNS * ROWS;
/// The state stamp is one more row of blocks, right below the stamp.
const STATE_ROWS: usize = ROWS + 1;
/// 4 bits state, 16 bits age, 4 bits CRC-4.
const STATE_DATA_BITS: u32 = 20;
const STATE_CRC_BITS: u32 = 4;
const PAYLOAD_LEN: usize = 9;
const MIN_BLOCK_SIDE: usize = 8;
const LUMA_ONE: u8 = 235;
const LUMA_ZERO: u8 = 16;
const LUMA_THRESHOLD: u64 = 128;
const CRC8_POLY: u8 = 0x07;
/// CRC-8/I-432-1. Without it a uniform stamp area (all zero bits) would carry a valid CRC.
const CRC8_XOROUT: u8 = 0x55;
/// CRC-4/INTERLAKEN: polynomial x^4 + x + 1, initial value and final xor all ones.
const CRC4_POLY: u8 = 0x3;
const CRC4_INIT: u8 = 0xF;
const CRC4_XOROUT: u8 = 0xF;
/// Larger ages are a misread stamp (an 8-bit CRC lets about 1 in 256 corrupt reads through) or a
/// clock fault, not a measurement.
pub const MAX_PLAUSIBLE_AGE_MS: u32 = 60_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameId(pub u32);

/// UNIX epoch milliseconds modulo 2^32.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureMs(pub u32);

impl CaptureMs {
    #[expect(
        clippy::cast_possible_truncation,
        reason = "the stamp clock is UNIX milliseconds modulo 2^32"
    )]
    pub fn from_unix_ms(unix_ms: u64) -> Self {
        Self(unix_ms as u32)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stamp {
    pub frame_id: FrameId,
    pub capture_ms: CaptureMs,
}

/// An 8-bit luma plane: `stride` bytes per row, `width` of them visible.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlaneGeometry {
    pub width: usize,
    pub height: usize,
    pub stride: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum StampError {
    #[error("invalid plane geometry {width}x{height} stride {stride}")]
    InvalidGeometry {
        width: usize,
        height: usize,
        stride: usize,
    },
    #[error("frame {width}x{height} is too small for the stamp")]
    FrameTooSmall { width: usize, height: usize },
    #[error("plane buffer holds {actual} bytes, geometry needs {needed}")]
    BufferTooShort { needed: usize, actual: usize },
}

/// Side of one square block of the stamp grid.
fn block_side(width: usize) -> usize {
    (width / COLUMNS).max(MIN_BLOCK_SIDE)
}

/// Checks the geometry, with `rows` rows of blocks, against the buffer and returns the block side.
fn validate(geometry: PlaneGeometry, len: usize, rows: usize) -> Result<usize, StampError> {
    let PlaneGeometry {
        width,
        height,
        stride,
    } = geometry;
    let invalid = StampError::InvalidGeometry {
        width,
        height,
        stride,
    };
    if width == 0 || height == 0 || stride < width {
        return Err(invalid);
    }
    let needed = stride.checked_mul(height).ok_or(invalid)?;
    let side = block_side(width);
    let too_small = StampError::FrameTooSmall { width, height };
    if width < COLUMNS * side || height < rows * side {
        return Err(too_small);
    }
    if len < needed {
        return Err(StampError::BufferTooShort {
            needed,
            actual: len,
        });
    }
    Ok(side)
}

/// Byte range of row `y` of the block at (`col`, `row`).
fn block_row_range(
    geometry: PlaneGeometry,
    side: usize,
    col: usize,
    row: usize,
    y: usize,
) -> std::ops::Range<usize> {
    let start = (row * side + y) * geometry.stride + col * side;
    start..start + side
}

fn crc8(bytes: &[u8]) -> u8 {
    let crc = bytes.iter().fold(0, |crc, &byte| {
        (0..8).fold(crc ^ byte, |crc, _| {
            if crc & 0x80 == 0 {
                crc << 1
            } else {
                (crc << 1) ^ CRC8_POLY
            }
        })
    });
    crc ^ CRC8_XOROUT
}

fn encode(stamp: Stamp) -> [u8; PAYLOAD_LEN] {
    let mut payload = [0; PAYLOAD_LEN];
    payload[..4].copy_from_slice(&stamp.frame_id.0.to_be_bytes());
    payload[4..8].copy_from_slice(&stamp.capture_ms.0.to_be_bytes());
    payload[8] = crc8(&payload[..8]);
    payload
}

fn decode(payload: [u8; PAYLOAD_LEN]) -> Option<Stamp> {
    let (body, crc) = payload.split_at(8);
    if crc.first() != Some(&crc8(body)) {
        return None;
    }
    let (id, ms) = body.split_at(4);
    Some(Stamp {
        frame_id: FrameId(u32::from_be_bytes(id.try_into().ok()?)),
        capture_ms: CaptureMs(u32::from_be_bytes(ms.try_into().ok()?)),
    })
}

fn bit_at(payload: &[u8; PAYLOAD_LEN], index: usize) -> bool {
    payload
        .get(index / 8)
        .is_some_and(|byte| byte >> (7 - index % 8) & 1 == 1)
}

fn fill_block(plane: &mut [u8], geometry: PlaneGeometry, side: usize, index: usize, luma: u8) {
    let (col, row) = (index % COLUMNS, index / COLUMNS);
    for y in 0..side {
        if let Some(pixels) = plane.get_mut(block_row_range(geometry, side, col, row, y)) {
            pixels.fill(luma);
        }
    }
}

/// Mean over the centred half-size square of the block is brighter than the threshold.
fn block_is_set(plane: &[u8], geometry: PlaneGeometry, side: usize, index: usize) -> Option<bool> {
    let (col, row) = (index % COLUMNS, index / COLUMNS);
    let inset = side / 4;
    let inner = side / 2;
    let mut sum = 0u64;
    for y in inset..inset + inner {
        let range = block_row_range(geometry, side, col, row, y);
        let pixels = plane.get(range.start + inset..range.start + inset + inner)?;
        sum += pixels.iter().map(|&p| u64::from(p)).sum::<u64>();
    }
    Some(sum > LUMA_THRESHOLD * (inner * inner) as u64)
}

/// Whether a `width` x `height` frame is large enough to carry the stamp.
pub fn check_size(width: usize, height: usize) -> Result<(), StampError> {
    let geometry = PlaneGeometry {
        width,
        height,
        stride: width,
    };
    validate(geometry, usize::MAX, ROWS).map(|_| ())
}

/// Whether a `width` x `height` frame is large enough to carry the stamp and the state stamp.
pub fn check_state_size(width: usize, height: usize) -> Result<(), StampError> {
    let geometry = PlaneGeometry {
        width,
        height,
        stride: width,
    };
    validate(geometry, usize::MAX, STATE_ROWS).map(|_| ())
}

/// Draws the stamp into the top-left corner of an 8-bit luma plane.
pub fn draw(plane: &mut [u8], geometry: PlaneGeometry, stamp: Stamp) -> Result<(), StampError> {
    let side = validate(geometry, plane.len(), ROWS)?;
    let payload = encode(stamp);
    for index in 0..BITS {
        let luma = if bit_at(&payload, index) {
            LUMA_ONE
        } else {
            LUMA_ZERO
        };
        fill_block(plane, geometry, side, index, luma);
    }
    Ok(())
}

/// Reads a stamp back; `None` for bad geometry, a short buffer or a CRC mismatch.
pub fn read(plane: &[u8], geometry: PlaneGeometry) -> Option<Stamp> {
    let side = validate(geometry, plane.len(), ROWS).ok()?;
    let mut payload = [0u8; PAYLOAD_LEN];
    for index in 0..BITS {
        if block_is_set(plane, geometry, side, index)? {
            *payload.get_mut(index / 8)? |= 0x80 >> (index % 8);
        }
    }
    decode(payload)
}

/// Frame age at the receiver; wraps with the 32-bit capture clock.
pub fn age_ms(receiver_ms: u32, stamp: Stamp) -> u32 {
    receiver_ms.wrapping_sub(stamp.capture_ms.0)
}

/// The age at `receiver_unix_ms`, if it is between 0 and [`MAX_PLAUSIBLE_AGE_MS`].
pub fn plausible_age(receiver_unix_ms: u64, stamp: Stamp) -> Option<u32> {
    let age = age_ms(CaptureMs::from_unix_ms(receiver_unix_ms).0, stamp);
    (age <= MAX_PLAUSIBLE_AGE_MS).then_some(age)
}

/// What the ground output showed with a frame: its freshness state and the age of the newest
/// frame, in 10 ms units, clamped to 16 bits; all ones also when the age is unknown.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StateStamp {
    pub state: State,
    pub age_10ms: u16,
}

impl StateStamp {
    pub fn new(state: State, age_ms: Option<u64>) -> Self {
        let age_10ms = age_ms.map_or(u16::MAX, |ms| u16::try_from(ms / 10).unwrap_or(u16::MAX));
        Self { state, age_10ms }
    }

    /// The age in milliseconds; `None` when it was unknown or at least 655.35 s.
    pub fn age_ms(self) -> Option<u32> {
        (self.age_10ms != u16::MAX).then(|| u32::from(self.age_10ms) * 10)
    }

    /// 24 bits: state, age, CRC-4 over the first 20.
    fn encode(self) -> u32 {
        let data = u32::from(self.state.code()) << 16 | u32::from(self.age_10ms);
        data << STATE_CRC_BITS | u32::from(crc4(bits_of(data, STATE_DATA_BITS)))
    }

    fn decode(bits: u32) -> Option<Self> {
        let data = bits >> STATE_CRC_BITS;
        if bits & 0xF != u32::from(crc4(bits_of(data, STATE_DATA_BITS))) {
            return None;
        }
        Some(Self {
            state: State::from_code(u8::try_from(data >> 16).ok()?)?,
            age_10ms: u16::try_from(data & 0xFFFF).ok()?,
        })
    }
}

/// The low `count` bits of `value`, most significant first.
fn bits_of(value: u32, count: u32) -> impl Iterator<Item = bool> {
    (0..count).rev().map(move |i| value >> i & 1 == 1)
}

fn crc4(bits: impl IntoIterator<Item = bool>) -> u8 {
    let crc = bits.into_iter().fold(CRC4_INIT, |crc, bit| {
        let shifted = crc << 1 & 0xF;
        if (crc >> 3 == 1) == bit {
            shifted
        } else {
            shifted ^ CRC4_POLY
        }
    });
    crc ^ CRC4_XOROUT
}

/// Draws the state stamp as the row of blocks below the stamp, leaving the stamp as it is.
pub fn draw_state(
    plane: &mut [u8],
    geometry: PlaneGeometry,
    state: StateStamp,
) -> Result<(), StampError> {
    let side = validate(geometry, plane.len(), STATE_ROWS)?;
    let bits = bits_of(state.encode(), STATE_DATA_BITS + STATE_CRC_BITS);
    for (i, bit) in bits.enumerate() {
        let luma = if bit { LUMA_ONE } else { LUMA_ZERO };
        fill_block(plane, geometry, side, BITS + i, luma);
    }
    Ok(())
}

/// Reads a state stamp back; `None` for bad geometry, a short buffer, a CRC mismatch or an
/// unused state code.
pub fn read_state(plane: &[u8], geometry: PlaneGeometry) -> Option<StateStamp> {
    let side = validate(geometry, plane.len(), STATE_ROWS).ok()?;
    let mut bits = 0u32;
    for i in 0..COLUMNS {
        bits = bits << 1 | u32::from(block_is_set(plane, geometry, side, BITS + i)?);
    }
    StateStamp::decode(bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIZES: [(usize, usize); 4] = [(192, 144), (320, 240), (640, 360), (1280, 720)];
    const STAMP: Stamp = Stamp {
        frame_id: FrameId(0x0102_0304),
        capture_ms: CaptureMs(0xA1B2_C3D4),
    };

    fn plane(width: usize, height: usize, pad: usize) -> (Vec<u8>, PlaneGeometry) {
        let geometry = PlaneGeometry {
            width,
            height,
            stride: width + pad,
        };
        (vec![90; geometry.stride * height], geometry)
    }

    fn block_origin(geometry: PlaneGeometry, bit: usize) -> usize {
        let side = geometry.width / 24;
        (bit / 24) * side * geometry.stride + (bit % 24) * side
    }

    #[test]
    fn round_trips_with_stride_padding() {
        for (width, height) in SIZES {
            let (mut buf, geometry) = plane(width, height, 32);
            draw(&mut buf, geometry, STAMP).unwrap();
            assert_eq!(read(&buf, geometry), Some(STAMP), "{width}x{height}");
        }
    }

    #[test]
    fn draws_only_the_top_left_blocks() {
        let (mut buf, geometry) = plane(320, 240, 16);
        draw(&mut buf, geometry, STAMP).unwrap();
        let side = 320 / 24;
        let below = 3 * side * geometry.stride;
        assert!(buf[below..].iter().all(|&p| p == 90));
        assert!(buf[24 * side..320].iter().all(|&p| p == 90));
    }

    #[test]
    fn uniform_stamp_areas_never_validate() {
        assert_eq!(crc8(b"123456789"), 0xA1, "CRC-8/I-432-1 check value");
        let cases = [
            ([0x00; PAYLOAD_LEN], LUMA_ZERO),
            ([0xFF; PAYLOAD_LEN], LUMA_ONE),
        ];
        for (payload, luma) in cases {
            assert_eq!(decode(payload), None, "{payload:?}");
            let (mut buf, geometry) = plane(320, 240, 0);
            let side = 320 / 24;
            for row in 0..3 * side {
                buf[row * geometry.stride..][..24 * side].fill(luma);
            }
            assert_eq!(read(&buf, geometry), None, "luma {luma}");
        }
    }

    #[test]
    fn rejects_a_flipped_block() {
        let (mut buf, geometry) = plane(640, 360, 0);
        draw(&mut buf, geometry, STAMP).unwrap();
        let side = 640 / 24;
        let origin = block_origin(geometry, 40);
        let flipped = if buf[origin + side / 2 * geometry.stride + side / 2] > 128 {
            16
        } else {
            235
        };
        for y in 0..side {
            buf[origin + y * geometry.stride..][..side].fill(flipped);
        }
        assert_eq!(read(&buf, geometry), None);
    }

    #[test]
    fn survives_mild_noise() {
        let (mut buf, geometry) = plane(320, 240, 8);
        draw(&mut buf, geometry, STAMP).unwrap();
        for (i, p) in buf.iter_mut().enumerate() {
            *p = if i % 3 == 0 {
                p.saturating_add(40)
            } else {
                p.saturating_sub(40)
            };
        }
        assert_eq!(read(&buf, geometry), Some(STAMP));
    }

    #[test]
    fn rejects_unusable_geometry() {
        let cases = [
            (191, 144, 191),
            (192, 23, 192),
            (192, 144, 191),
            (usize::MAX, usize::MAX, usize::MAX),
        ];
        for (width, height, stride) in cases {
            let geometry = PlaneGeometry {
                width,
                height,
                stride,
            };
            let mut buf = vec![0; 192 * 144];
            assert!(draw(&mut buf, geometry, STAMP).is_err(), "{geometry:?}");
            assert_eq!(read(&buf, geometry), None, "{geometry:?}");
        }
        assert!(check_size(191, 144).is_err());
        assert!(check_size(192, 23).is_err());
        assert!(check_size(192, 24).is_ok());
    }

    #[test]
    fn short_buffer_is_an_error_not_a_panic() {
        let geometry = PlaneGeometry {
            width: 192,
            height: 144,
            stride: 200,
        };
        let mut buf = vec![0; 200 * 144 - 1];
        assert!(matches!(
            draw(&mut buf, geometry, STAMP),
            Err(StampError::BufferTooShort { .. })
        ));
        assert_eq!(read(&buf, geometry), None);
    }

    #[test]
    fn state_stamp_round_trips_below_the_stamp_and_rejects_a_corrupted_crc() {
        let check: Vec<bool> = b"123456789"
            .iter()
            .flat_map(|&byte| bits_of(u32::from(byte), 8))
            .collect();
        assert_eq!(crc4(check), 0xB, "CRC-4/INTERLAKEN check value");
        let geometry = PlaneGeometry {
            width: 640,
            height: 360,
            stride: 656,
        };
        let crc_block = block_origin(geometry, BITS + COLUMNS - 1);
        // (state, age in ms, block to flip, what reads back)
        let cases = [
            (State::Live, Some(337), None, Some((State::Live, Some(330)))),
            (
                State::Stale,
                Some(655_339),
                None,
                Some((State::Stale, Some(655_330))),
            ),
            (
                State::Disconnected,
                Some(900_000),
                None,
                Some((State::Disconnected, None)),
            ),
            (
                State::AgeUnknown,
                None,
                None,
                Some((State::AgeUnknown, None)),
            ),
            (
                State::LowRate,
                Some(829),
                None,
                Some((State::LowRate, Some(820))),
            ),
            (State::Frozen, Some(1234), Some(crc_block), None),
        ];
        let side = 640 / 24;
        for (state, age_ms, flip, expected) in cases {
            let mut buf = vec![90; geometry.stride * geometry.height];
            draw(&mut buf, geometry, STAMP).unwrap();
            draw_state(&mut buf, geometry, StateStamp::new(state, age_ms)).unwrap();
            if let Some(origin) = flip {
                let flipped = if buf[origin] > 128 { 16 } else { 235 };
                for y in 0..side {
                    buf[origin + y * geometry.stride..][..side].fill(flipped);
                }
            }
            assert_eq!(
                read(&buf, geometry),
                Some(STAMP),
                "{state:?}: the stamp stays"
            );
            let read = read_state(&buf, geometry).map(|s| (s.state, s.age_ms()));
            assert_eq!(read, expected, "{state:?} {age_ms:?}");
        }
    }

    #[test]
    fn age_wraps_around_the_32_bit_clock() {
        let stamp = Stamp {
            frame_id: FrameId(1),
            capture_ms: CaptureMs(u32::MAX - 9),
        };
        assert_eq!(age_ms(10, stamp), 20);
        assert_eq!(age_ms(u32::MAX - 9, stamp), 0);
        let received = CaptureMs::from_unix_ms(u64::from(u32::MAX) + 11).0;
        assert_eq!(age_ms(received, stamp), 20);
    }
}
