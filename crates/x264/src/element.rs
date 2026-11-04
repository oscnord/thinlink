//! The element: a `VideoEncoder` subclass around [`Encoder`].

use std::sync::OnceLock;

use gst::glib;
use gst::prelude::*;
use gstreamer as gst;
use gstreamer_video as gst_video;

use crate::encoder::{Config, Encoder, Picture, Rate};

pub const ELEMENT: &str = "satx264enc";

glib::wrapper! {
    pub struct X264Enc(ObjectSubclass<imp::X264Enc>)
        @extends gst_video::VideoEncoder, gst::Element, gst::Object;
}

/// Registers the element with `GStreamer` (no plugin file). Safe to call again; call after
/// `gst::init`.
pub fn register() -> Result<(), glib::BoolError> {
    static REGISTERED: OnceLock<Result<(), glib::BoolError>> = OnceLock::new();
    REGISTERED
        .get_or_init(|| {
            gst::Element::register(None, ELEMENT, gst::Rank::NONE, X264Enc::static_type())
        })
        .clone()
}

mod imp {
    use std::sync::{LazyLock, Mutex, MutexGuard, PoisonError};

    use gst::glib;
    use gst::prelude::*;
    use gst::subclass::prelude::*;
    use gst_video::prelude::*;
    use gst_video::subclass::prelude::*;
    use gstreamer as gst;
    use gstreamer_video as gst_video;

    use super::{Config, Encoder, Picture, Rate};
    use crate::roi::{Rect, quant_offsets};

    static CAT: LazyLock<gst::DebugCategory> = LazyLock::new(|| {
        gst::DebugCategory::new(
            super::ELEMENT,
            gst::DebugColorFlags::empty(),
            Some("libx264 encoder"),
        )
    });

    const DEFAULT_BITRATE_KBIT: u32 = 2048;
    const MAX_BITRATE_KBIT: u32 = 100_000;
    const DEFAULT_VBV_MS: u32 = 100;
    const MAX_VBV_MS: u32 = 10_000;
    const DEFAULT_REFRESH_MS: u32 = 2000;
    const MIN_REFRESH_MS: u32 = 100;
    const MAX_REFRESH_MS: u32 = 60_000;
    const MIN_ROI_QP_OFFSET: f32 = -51.0;

    #[derive(Debug, Clone, Copy)]
    struct Settings {
        rate: Rate,
        refresh_ms: u32,
        roi_qp_offset: f32,
    }

    impl Default for Settings {
        fn default() -> Self {
            Self {
                rate: Rate {
                    bitrate_kbit: DEFAULT_BITRATE_KBIT,
                    vbv_ms: DEFAULT_VBV_MS,
                },
                refresh_ms: DEFAULT_REFRESH_MS,
                roi_qp_offset: 0.0,
            }
        }
    }

    /// The open encoder for the negotiated caps.
    struct State {
        encoder: Encoder,
        info: gst_video::VideoInfo,
        /// The rate the encoder runs at, to see when a property changed it.
        rate: Rate,
        /// The next picture's presentation time to x264, in frames.
        next_pts: i64,
        /// The quantiser offset inside regions of interest the encoder was opened with.
        roi_qp_offset: f32,
    }

    #[derive(Default)]
    pub struct X264Enc {
        settings: Mutex<Settings>,
        state: Mutex<Option<State>>,
    }

    fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
        mutex.lock().unwrap_or_else(PoisonError::into_inner)
    }

    #[glib::object_subclass]
    impl ObjectSubclass for X264Enc {
        const NAME: &'static str = "SatTeleopX264Enc";
        type Type = super::X264Enc;
        type ParentType = gst_video::VideoEncoder;
    }

    impl ObjectImpl for X264Enc {
        fn properties() -> &'static [glib::ParamSpec] {
            static PROPERTIES: LazyLock<Vec<glib::ParamSpec>> = LazyLock::new(|| {
                vec![
                    glib::ParamSpecUInt::builder("bitrate")
                        .nick("Bitrate")
                        .blurb("Average bitrate in kbit/s; changes at the next frame while playing")
                        .minimum(1)
                        .maximum(MAX_BITRATE_KBIT)
                        .default_value(DEFAULT_BITRATE_KBIT)
                        .mutable_playing()
                        .build(),
                    glib::ParamSpecUInt::builder("vbv-buf-capacity")
                        .nick("VBV buffer capacity")
                        .blurb("VBV buffer in milliseconds of the bitrate; changes at the next frame while playing")
                        .minimum(1)
                        .maximum(MAX_VBV_MS)
                        .default_value(DEFAULT_VBV_MS)
                        .mutable_playing()
                        .build(),
                    glib::ParamSpecUInt::builder("refresh-ms")
                        .nick("Refresh period")
                        .blurb("Duration of one intra refresh wave; read again whenever the caps are set")
                        .minimum(MIN_REFRESH_MS)
                        .maximum(MAX_REFRESH_MS)
                        .default_value(DEFAULT_REFRESH_MS)
                        .build(),
                    glib::ParamSpecFloat::builder("roi-qp-offset")
                        .nick("Region quantiser offset")
                        .blurb("Quantiser offset inside the regions of interest on a frame's buffer, negative for finer; 0 is off. Turns adaptive quantization on, which x264 needs to honour offsets; read again whenever the caps are set")
                        .minimum(MIN_ROI_QP_OFFSET)
                        .maximum(0.0)
                        .default_value(0.0)
                        .build(),
                    glib::ParamSpecUInt::builder("key-int-max")
                        .nick("Refresh period in frames")
                        .blurb("Frames in one refresh wave of the open encoder, 0 before the caps are set")
                        .read_only()
                        .build(),
                ]
            });
            PROPERTIES.as_ref()
        }

        fn set_property(&self, _id: usize, value: &glib::Value, pspec: &glib::ParamSpec) {
            let mut settings = lock(&self.settings);
            if pspec.name() == "roi-qp-offset" {
                settings.roi_qp_offset = value.get().unwrap_or_default();
                return;
            }
            let Ok(value) = value.get::<u32>() else {
                return;
            };
            match pspec.name() {
                "bitrate" => settings.rate.bitrate_kbit = value,
                "vbv-buf-capacity" => settings.rate.vbv_ms = value,
                "refresh-ms" => settings.refresh_ms = value,
                _ => {}
            }
        }

        fn property(&self, _id: usize, pspec: &glib::ParamSpec) -> glib::Value {
            let settings = *lock(&self.settings);
            match pspec.name() {
                "bitrate" => settings.rate.bitrate_kbit.to_value(),
                "vbv-buf-capacity" => settings.rate.vbv_ms.to_value(),
                "refresh-ms" => settings.refresh_ms.to_value(),
                "roi-qp-offset" => settings.roi_qp_offset.to_value(),
                "key-int-max" => lock(&self.state)
                    .as_ref()
                    .map_or(0, |state| state.encoder.key_int_max())
                    .to_value(),
                _ => glib::Value::from_type(glib::Type::INVALID),
            }
        }
    }

    impl GstObjectImpl for X264Enc {}

    impl ElementImpl for X264Enc {
        fn metadata() -> Option<&'static gst::subclass::ElementMetadata> {
            static METADATA: LazyLock<gst::subclass::ElementMetadata> = LazyLock::new(|| {
                gst::subclass::ElementMetadata::new(
                    "libx264 encoder",
                    "Codec/Encoder/Video",
                    "H.264 on libx264 with on-demand IDR frames under intra refresh",
                    "sat-teleop",
                )
            });
            Some(&METADATA)
        }

        #[expect(clippy::expect_used, reason = "static caps and names are always valid")]
        fn pad_templates() -> &'static [gst::PadTemplate] {
            static TEMPLATES: LazyLock<Vec<gst::PadTemplate>> = LazyLock::new(|| {
                let sink = gst::Caps::builder("video/x-raw")
                    .field("format", "I420")
                    .field("width", gst::IntRange::new(2, i32::MAX))
                    .field("height", gst::IntRange::new(2, i32::MAX))
                    .field(
                        "framerate",
                        gst::FractionRange::new(
                            gst::Fraction::new(1, i32::MAX),
                            gst::Fraction::new(i32::MAX, 1),
                        ),
                    )
                    .build();
                let src = gst::Caps::builder("video/x-h264")
                    .field("stream-format", "byte-stream")
                    .field("alignment", "au")
                    .build();
                vec![
                    gst::PadTemplate::new(
                        "sink",
                        gst::PadDirection::Sink,
                        gst::PadPresence::Always,
                        &sink,
                    )
                    .expect("sink template"),
                    gst::PadTemplate::new(
                        "src",
                        gst::PadDirection::Src,
                        gst::PadPresence::Always,
                        &src,
                    )
                    .expect("src template"),
                ]
            });
            TEMPLATES.as_ref()
        }
    }

    impl VideoEncoderImpl for X264Enc {
        fn stop(&self) -> Result<(), gst::ErrorMessage> {
            *lock(&self.state) = None;
            self.parent_stop()
        }

        fn set_format(
            &self,
            input: &gst_video::VideoCodecState<'static, gst_video::video_codec_state::Readable>,
        ) -> Result<(), gst::LoggableError> {
            let info = input.info().clone();
            let settings = *lock(&self.settings);
            let config = config_for(&info, settings)
                .ok_or_else(|| gst::loggable_error!(CAT, "the caps have no fixed frame rate"))?;
            let encoder = Encoder::open(&config)
                .map_err(|e| gst::loggable_error!(CAT, "opening x264: {e}"))?;
            gst::debug!(CAT, imp = self, "opened x264: {config:?}");
            *lock(&self.state) = Some(State {
                encoder,
                info,
                rate: settings.rate,
                next_pts: 0,
                roi_qp_offset: settings.roi_qp_offset,
            });

            let caps = gst::Caps::builder("video/x-h264")
                .field("stream-format", "byte-stream")
                .field("alignment", "au")
                .build();
            let obj = self.obj();
            let output = obj
                .set_output_state(caps, Some(input))
                .map_err(|_| gst::loggable_error!(CAT, "setting the output state"))?;
            obj.set_latency(gst::ClockTime::ZERO, gst::ClockTime::ZERO);
            obj.negotiate(output)
                .map_err(|_| gst::loggable_error!(CAT, "negotiating the output"))?;
            self.parent_set_format(input)
        }

        fn handle_frame(
            &self,
            mut frame: gst_video::VideoCodecFrame,
        ) -> Result<gst::FlowSuccess, gst::FlowError> {
            let rate = lock(&self.settings).rate;
            let force = frame
                .flags()
                .contains(gst_video::VideoCodecFrameFlags::FORCE_KEYFRAME);
            let encoded = self.encode(&frame, rate, force)?;
            let Some((data, idr)) = encoded else {
                // A frame without an output buffer is counted as dropped.
                return self.obj().finish_frame(frame);
            };
            if idr {
                frame.set_flags(gst_video::VideoCodecFrameFlags::SYNC_POINT);
            } else {
                frame.unset_flags(gst_video::VideoCodecFrameFlags::SYNC_POINT);
            }
            frame.set_dts(frame.pts());
            frame.set_output_buffer(gst::Buffer::from_mut_slice(data));
            self.obj().finish_frame(frame)
        }
    }

    impl X264Enc {
        /// Encodes the frame's picture: its bytes and whether they are a whole IDR frame. The
        /// state lock is released before the frame is pushed downstream.
        fn encode(
            &self,
            frame: &gst_video::VideoCodecFrame,
            rate: Rate,
            idr: bool,
        ) -> Result<Option<(Vec<u8>, bool)>, gst::FlowError> {
            let mut guard = lock(&self.state);
            let state = guard.as_mut().ok_or(gst::FlowError::NotNegotiated)?;
            if rate != state.rate {
                state.encoder.reconfig(rate).map_err(|e| self.fail(&e))?;
                state.rate = rate;
            }
            let input = frame.input_buffer().ok_or(gst::FlowError::Error)?;
            let video = gst_video::VideoFrameRef::from_buffer_ref_readable(input, &state.info)
                .map_err(|_| gst::FlowError::Error)?;
            let plane = |n| video.plane_data(n).map_err(|_| gst::FlowError::Error);
            let stride = |n: usize| {
                usize::try_from(video.plane_stride().get(n).copied().unwrap_or(0))
                    .map_err(|_| gst::FlowError::Error)
            };
            let picture = Picture {
                planes: [plane(0)?, plane(1)?, plane(2)?],
                strides: [stride(0)?, stride(1)?, stride(2)?],
            };
            let pts = state.next_pts;
            state.next_pts += 1;
            let offsets = (state.roi_qp_offset < 0.0)
                .then(|| regions_of_interest(input))
                .flatten()
                .map(|regions| {
                    let (width, height) = (state.info.width(), state.info.height());
                    quant_offsets(width, height, &regions, state.roi_qp_offset)
                });
            let encoded = state
                .encoder
                .encode(&picture, pts, idr, offsets.as_deref())
                .map_err(|e| self.fail(&e))?;
            Ok(encoded.map(|e| (e.data.to_vec(), e.idr)))
        }

        fn fail(&self, error: &crate::encoder::Error) -> gst::FlowError {
            gst::element_imp_error!(self, gst::StreamError::Encode, ["{error}"]);
            gst::FlowError::Error
        }
    }

    fn config_for(info: &gst_video::VideoInfo, settings: Settings) -> Option<Config> {
        let fps = info.fps();
        Some(Config {
            width: info.width(),
            height: info.height(),
            fps_n: u32::try_from(fps.numer()).ok()?,
            fps_d: u32::try_from(fps.denom()).ok()?,
            refresh_ms: settings.refresh_ms,
            rate: settings.rate,
            roi_qp_offset: settings.roi_qp_offset,
        })
    }

    /// The regions of interest on the buffer, when there are any to apply.
    fn regions_of_interest(buffer: &gst::BufferRef) -> Option<Vec<Rect>> {
        let regions: Vec<Rect> = buffer
            .iter_meta::<gst_video::VideoRegionOfInterestMeta>()
            .map(|meta| meta.rect())
            .collect();
        (!regions.is_empty()).then_some(regions)
    }
}

#[cfg(test)]
mod tests {
    use gst::prelude::*;
    use gstreamer as gst;
    use gstreamer_app as gst_app;
    use gstreamer_video as gst_video;

    const WIDTH: i32 = 64;
    const HEIGHT: i32 = 48;
    const IDR: u8 = 5;

    /// appsrc -> satx264enc -> appsink, one frame in and one out at a time.
    struct Rig {
        pipeline: gst::Pipeline,
        src: gst_app::AppSrc,
        sink: gst_app::AppSink,
        encoder: gst::Element,
        pushed: u64,
    }

    impl Rig {
        fn new(fps: i32, refresh_ms: u32) -> Self {
            gst::init().unwrap();
            super::register().unwrap();
            let src = gst_app::AppSrc::builder().format(gst::Format::Time).build();
            let encoder = gst::ElementFactory::make(super::ELEMENT)
                .property("refresh-ms", refresh_ms)
                .build()
                .unwrap();
            let sink = gst_app::AppSink::builder().sync(false).build();
            let pipeline = gst::Pipeline::new();
            pipeline
                .add_many([src.upcast_ref(), &encoder, sink.upcast_ref()])
                .unwrap();
            gst::Element::link_many([src.upcast_ref(), &encoder, sink.upcast_ref()]).unwrap();
            pipeline.set_state(gst::State::Playing).unwrap();
            let rig = Self {
                pipeline,
                src,
                sink,
                encoder,
                pushed: 0,
            };
            rig.set_fps(fps);
            rig
        }

        fn set_fps(&self, fps: i32) {
            let caps = gst_video::VideoCapsBuilder::new()
                .format(gst_video::VideoFormat::I420)
                .width(WIDTH)
                .height(HEIGHT)
                .framerate(gst::Fraction::new(fps, 1))
                .build();
            self.src.set_caps(Some(&caps));
        }

        fn force_key_unit(&self) {
            let event = gst_video::UpstreamForceKeyUnitEvent::builder()
                .all_headers(true)
                .build();
            assert!(self.encoder.send_event(event));
        }

        /// Pushes a moving picture and returns the encoded frame's NAL types and whether it is
        /// marked as a delta unit.
        fn frame(&mut self) -> (Vec<u8>, bool) {
            let shift = usize::try_from(self.pushed * 3).unwrap();
            let mut data = vec![0u8; usize::try_from(WIDTH * HEIGHT * 3 / 2).unwrap()];
            for (i, byte) in data.iter_mut().enumerate() {
                *byte = u8::try_from((i + shift) * 7 % 251).unwrap();
            }
            let mut buffer = gst::Buffer::from_mut_slice(data);
            buffer
                .get_mut()
                .unwrap()
                .set_pts(gst::ClockTime::from_mseconds(self.pushed * 100));
            self.pushed += 1;
            self.src.push_buffer(buffer).unwrap();
            let sample = self
                .sink
                .try_pull_sample(gst::ClockTime::from_seconds(5))
                .unwrap();
            let buffer = sample.buffer().unwrap();
            let map = buffer.map_readable().unwrap();
            (
                nal_types(&map),
                buffer.flags().contains(gst::BufferFlags::DELTA_UNIT),
            )
        }
    }

    impl Drop for Rig {
        fn drop(&mut self) {
            let _ = self.pipeline.set_state(gst::State::Null);
        }
    }

    fn nal_types(annex_b: &[u8]) -> Vec<u8> {
        annex_b
            .windows(4)
            .filter(|w| w[..3] == [0, 0, 1])
            .map(|w| w[3] & 0x1f)
            .collect()
    }

    #[test]
    fn forced_key_unit_is_a_whole_idr_while_refresh_waves_stay_delta_units() {
        // A 5 frame refresh period, so waves start at frames 5 and 10 around the forced frame 7.
        let mut rig = Rig::new(10, 500);
        for n in 0..14 {
            if n == 7 {
                rig.force_key_unit();
            }
            let (nals, delta) = rig.frame();
            let idr = n == 0 || n == 7;
            assert_eq!(nals.contains(&IDR), idr, "frame {n}: NAL types {nals:?}");
            assert_eq!(delta, !idr, "frame {n}");
        }
    }

    #[test]
    fn refresh_period_follows_the_frame_rate_without_replacing_the_element() {
        let mut rig = Rig::new(10, 1000);
        rig.frame();
        assert_eq!(rig.encoder.property::<u32>("key-int-max"), 10);
        rig.set_fps(20);
        let (nals, _) = rig.frame();
        assert_eq!(rig.encoder.property::<u32>("key-int-max"), 20);
        assert!(
            nals.contains(&IDR),
            "a reopened encoder starts with an IDR frame"
        );
    }

    /// The size of the first frame of a noisy picture, pushed through a leaky queue as the sender
    /// does, with a region of interest over its left half or none. The rate is low enough that the quantiser
    /// is not at its floor, where an offset changes nothing.
    fn first_frame_bytes(roi: bool) -> usize {
        const W: i32 = 320;
        const H: i32 = 256;
        gst::init().unwrap();
        super::register().unwrap();
        let caps = gst_video::VideoCapsBuilder::new()
            .format(gst_video::VideoFormat::I420)
            .width(W)
            .height(H)
            .framerate(gst::Fraction::new(10, 1))
            .build();
        let src = gst_app::AppSrc::builder()
            .format(gst::Format::Time)
            .caps(&caps)
            .build();
        let queue = gst::ElementFactory::make("queue")
            .property_from_str("leaky", "downstream")
            .build()
            .unwrap();
        let encoder = gst::ElementFactory::make(super::ELEMENT)
            .property("bitrate", 200u32)
            .property("vbv-buf-capacity", 10_000u32)
            .property("roi-qp-offset", -12.0f32)
            .build()
            .unwrap();
        let sink = gst_app::AppSink::builder().sync(false).build();
        let pipeline = gst::Pipeline::new();
        let chain = [src.upcast_ref(), &queue, &encoder, sink.upcast_ref()];
        pipeline.add_many(chain).unwrap();
        gst::Element::link_many(chain).unwrap();
        pipeline.set_state(gst::State::Playing).unwrap();
        let data: Vec<u8> = (0..u32::try_from(W * H * 3 / 2).unwrap())
            .map(|i| {
                u8::try_from(120 + (i.wrapping_mul(2_654_435_761) >> 24) % 2 + (i % 320) / 4)
                    .unwrap()
            })
            .collect();
        let mut buffer = gst::Buffer::from_mut_slice(data);
        if roi {
            let buffer = buffer.get_mut().unwrap();
            gst_video::VideoRegionOfInterestMeta::add(buffer, "region", (0, 0, 160, 256));
        }
        src.push_buffer(buffer).unwrap();
        let sample = sink
            .try_pull_sample(gst::ClockTime::from_seconds(5))
            .unwrap();
        let size = sample.buffer().unwrap().size();
        pipeline.set_state(gst::State::Null).unwrap();
        size
    }

    #[test]
    fn a_region_of_interest_gets_more_bits_through_the_leaky_queue() {
        let plain = first_frame_bytes(false);
        let with_region = first_frame_bytes(true);
        assert!(
            with_region > plain + plain / 5,
            "{with_region} bytes with the region, {plain} without"
        );
    }
}
