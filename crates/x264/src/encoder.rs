//! One libx264 encoder, opened the way the sender needs it: ultrafast and zerolatency (no
//! lookahead, no B-frames, sliced threads), average bitrate under a VBV buffer, intra refresh
//! instead of periodic IDR frames, parameter sets with every frame that starts a refresh wave.

#![expect(
    unsafe_code,
    reason = "the only FFI to libx264, each block says why it holds"
)]

use std::mem::MaybeUninit;
use std::ptr::NonNull;
use std::slice;

use crate::sys;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}")]
    Config(&'static str),
    #[error("x264 refused the settings")]
    Open,
    #[error("x264 refused the new rate")]
    Reconfig,
    #[error("x264 failed to encode a frame")]
    Encode,
    #[error("{0} plane is too small for the frame")]
    Plane(&'static str),
}

/// The settings that change while the encoder runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rate {
    pub bitrate_kbit: u32,
    /// The VBV buffer in milliseconds of `bitrate_kbit`.
    pub vbv_ms: u32,
}

impl Rate {
    fn vbv_kbit(self) -> i32 {
        let kbit = u64::from(self.bitrate_kbit) * u64::from(self.vbv_ms) / 1000;
        saturating_i32(kbit)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub width: u32,
    pub height: u32,
    pub fps_n: u32,
    pub fps_d: u32,
    /// One intra refresh wave takes this long.
    pub refresh_ms: u32,
    pub rate: Rate,
}

impl Config {
    /// The frames one refresh wave takes, at least one.
    pub fn key_int_max(&self) -> u32 {
        let frames = u64::from(self.fps_n) * u64::from(self.refresh_ms);
        let frames = frames.div_ceil(1000 * u64::from(self.fps_d.max(1))).max(1);
        u32::try_from(frames).unwrap_or(u32::MAX)
    }
}

/// An I420 picture: the planes and their row strides in bytes.
pub struct Picture<'a> {
    pub planes: [&'a [u8]; 3],
    pub strides: [usize; 3],
}

pub struct Encoded<'a> {
    pub data: &'a [u8],
    /// A whole IDR frame. Frames that start or continue a refresh wave are not.
    pub idr: bool,
}

pub struct Encoder {
    handle: NonNull<sys::x264_t>,
    param: sys::x264_param_t,
    width: usize,
    height: usize,
    key_int_max: u32,
}

// SAFETY: the handle is used from one thread at a time, as `&mut self` requires, and x264 keeps
// no thread-local state between calls.
unsafe impl Send for Encoder {}

impl Encoder {
    pub fn open(config: &Config) -> Result<Self, Error> {
        if config.width == 0 || config.height == 0 {
            return Err(Error::Config("the picture has no size"));
        }
        if config.fps_n == 0 || config.fps_d == 0 {
            return Err(Error::Config("the frame rate is not fixed"));
        }
        let mut param = default_param()?;
        param.i_csp = sys::X264_CSP_I420;
        param.i_width = saturating_i32(config.width.into());
        param.i_height = saturating_i32(config.height.into());
        param.b_vfr_input = 0;
        param.i_fps_num = config.fps_n;
        param.i_fps_den = config.fps_d;
        // Presentation times are frame counts.
        param.i_timebase_num = config.fps_d;
        param.i_timebase_den = config.fps_n;
        param.i_keyint_max = saturating_i32(config.key_int_max().into());
        param.b_intra_refresh = 1;
        param.b_repeat_headers = 1;
        param.b_annexb = 1;
        param.b_aud = 0;
        param.i_log_level = sys::X264_LOG_WARNING;
        param.rc.i_rc_method = sys::X264_RC_ABR;
        set_rate(&mut param, config.rate);
        // SAFETY: `param` is a fully initialised parameter set.
        let handle = unsafe { sys::x264_encoder_open(&raw mut param) };
        Ok(Self {
            handle: NonNull::new(handle).ok_or(Error::Open)?,
            param,
            width: usize::try_from(config.width).map_err(|_| Error::Open)?,
            height: usize::try_from(config.height).map_err(|_| Error::Open)?,
            key_int_max: config.key_int_max(),
        })
    }

    pub fn key_int_max(&self) -> u32 {
        self.key_int_max
    }

    /// Takes effect at the next frame.
    pub fn reconfig(&mut self, rate: Rate) -> Result<(), Error> {
        set_rate(&mut self.param, rate);
        // SAFETY: the handle is open and `param` is the set it was opened with, with the new rate.
        let status =
            unsafe { sys::x264_encoder_reconfig(self.handle.as_ptr(), &raw mut self.param) };
        if status < 0 {
            Err(Error::Reconfig)
        } else {
            Ok(())
        }
    }

    /// Encodes one picture. `idr` makes it a whole IDR frame, which x264 allows with intra
    /// refresh on (a forced type becomes the frame's type; the refresh wave restarts after it).
    /// `None` when x264 produced no output for the picture.
    pub fn encode(
        &mut self,
        picture: &Picture<'_>,
        pts: i64,
        idr: bool,
    ) -> Result<Option<Encoded<'_>>, Error> {
        let [luma, blue, red] = picture.planes;
        let [luma_stride, blue_stride, red_stride] = picture.strides;
        let chroma = (self.width.div_ceil(2), self.height.div_ceil(2));
        check_plane("luma", luma, luma_stride, (self.width, self.height))?;
        check_plane("blue chroma", blue, blue_stride, chroma)?;
        check_plane("red chroma", red, red_stride, chroma)?;
        let stride = |bytes| i32::try_from(bytes).map_err(|_| Error::Encode);
        let mut pic_in = MaybeUninit::<sys::x264_picture_t>::uninit();
        // SAFETY: `x264_picture_init` initialises the whole picture.
        unsafe { sys::x264_picture_init(pic_in.as_mut_ptr()) };
        // SAFETY: initialised by the call above.
        let mut pic_in = unsafe { pic_in.assume_init() };
        pic_in.img.i_csp = sys::X264_CSP_I420;
        pic_in.img.i_plane = 3;
        pic_in.img.i_stride = [
            stride(luma_stride)?,
            stride(blue_stride)?,
            stride(red_stride)?,
            0,
        ];
        // x264 only reads the input planes.
        pic_in.img.plane = [
            luma.as_ptr().cast_mut(),
            blue.as_ptr().cast_mut(),
            red.as_ptr().cast_mut(),
            std::ptr::null_mut(),
        ];
        pic_in.i_pts = pts;
        pic_in.i_type = if idr {
            sys::X264_TYPE_IDR
        } else {
            sys::X264_TYPE_AUTO
        };

        let mut pic_out = MaybeUninit::<sys::x264_picture_t>::uninit();
        // SAFETY: as for `pic_in`.
        unsafe { sys::x264_picture_init(pic_out.as_mut_ptr()) };
        // SAFETY: initialised by the call above.
        let mut pic_out = unsafe { pic_out.assume_init() };
        let mut nal: *mut sys::x264_nal_t = std::ptr::null_mut();
        let mut nal_count = 0;
        // SAFETY: the handle is open, the input planes were checked against the frame size above
        // and outlive the call, and x264 fills `nal` and `nal_count`.
        let size = unsafe {
            sys::x264_encoder_encode(
                self.handle.as_ptr(),
                &raw mut nal,
                &raw mut nal_count,
                &raw mut pic_in,
                &raw mut pic_out,
            )
        };
        let size = usize::try_from(size).map_err(|_| Error::Encode)?;
        if size == 0 || nal.is_null() {
            return Ok(None);
        }
        // SAFETY: x264 guarantees the payloads of all NALs of a frame are contiguous, starting at
        // the first one, `size` bytes in all; they stay valid until the next call, which `&mut self`
        // rules out while the borrow lives.
        let data = unsafe { slice::from_raw_parts((*nal).p_payload, size) };
        Ok(Some(Encoded {
            data,
            idr: pic_out.i_type == sys::X264_TYPE_IDR,
        }))
    }
}

impl Drop for Encoder {
    fn drop(&mut self) {
        // SAFETY: the handle is open and never used again.
        unsafe { sys::x264_encoder_close(self.handle.as_ptr()) };
    }
}

/// A plane of `size` (width, height) bytes with rows `stride` apart.
fn check_plane(
    name: &'static str,
    data: &[u8],
    stride: usize,
    (width, height): (usize, usize),
) -> Result<(), Error> {
    let needed = stride
        .checked_mul(height.saturating_sub(1))
        .and_then(|n| n.checked_add(width));
    match needed {
        Some(needed) if stride >= width && data.len() >= needed => Ok(()),
        _ => Err(Error::Plane(name)),
    }
}

fn default_param() -> Result<sys::x264_param_t, Error> {
    let mut param = MaybeUninit::<sys::x264_param_t>::uninit();
    // SAFETY: on success x264 initialises the whole parameter set; the names are valid C strings.
    let status = unsafe {
        sys::x264_param_default_preset(
            param.as_mut_ptr(),
            c"ultrafast".as_ptr(),
            c"zerolatency".as_ptr(),
        )
    };
    if status != 0 {
        return Err(Error::Config(
            "x264 has no ultrafast preset or zerolatency tune",
        ));
    }
    // SAFETY: initialised by the call above.
    Ok(unsafe { param.assume_init() })
}

fn set_rate(param: &mut sys::x264_param_t, rate: Rate) {
    let bitrate = saturating_i32(rate.bitrate_kbit.into());
    param.rc.i_bitrate = bitrate;
    param.rc.i_vbv_max_bitrate = bitrate;
    param.rc.i_vbv_buffer_size = rate.vbv_kbit();
}

fn saturating_i32(value: u64) -> i32 {
    i32::try_from(value).unwrap_or(i32::MAX)
}
