//! External camera ingest: RTP/H.264 datagrams from a camera outside the vehicle's container are
//! read from one UDP socket, kept only when they come from an expected address, and fed to an
//! `appsrc` -> depay -> parse -> decode chain that replaces the sender's source. The decode is the
//! only extra step before the stamp; [`IngestStats`] measures it.

use std::collections::VecDeque;
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use serde::Serialize;

use crate::MediaError;

/// RTP packets are at most 1200 bytes (the camera's `pkt_size`); a datagram this big was cut.
const MAX_DATAGRAM: usize = 2048;
/// How long a read waits before the stop flag is checked again.
const POLL: Duration = Duration::from_millis(200);
/// What `appsrc` may queue before it drops the oldest packets: far more than a keyframe at the
/// highest rung, far less than a second of video.
const QUEUE_BYTES: u64 = 512 * 1024;
const RTP_CAPS: &str = "application/x-rtp,media=(string)video,clock-rate=(int)90000,\
                        encoding-name=(string)H264,payload=(int)96";
/// Access units waiting for the decoder's answer; more means the decoder drops frames.
const DECODES_IN_FLIGHT: usize = 8;

/// Whether a datagram from `peer` is taken. An IPv4-mapped IPv6 address counts as its IPv4 one.
pub fn accepts(expected: &[IpAddr], peer: IpAddr) -> bool {
    let peer = peer.to_canonical();
    expected.iter().any(|a| a.to_canonical() == peer)
}

/// Counters of the ingest, and the decoder's time per frame.
#[derive(Debug, Default)]
pub struct IngestStats {
    accepted: AtomicU64,
    /// From an unexpected address, or cut to the read buffer's size.
    refused: AtomicU64,
    decoded: AtomicU64,
    decode_us_last: AtomicU64,
    decode_us_total: AtomicU64,
    decode_us_max: AtomicU64,
    /// When each access unit reached the decoder, by presentation time, oldest first.
    waiting: Mutex<VecDeque<(gst::ClockTime, Instant)>>,
}

#[derive(Debug, Serialize)]
pub struct IngestSnapshot {
    pub packets: u64,
    pub refused: u64,
    pub frames_decoded: u64,
    /// Decoder input to decoder output, per frame.
    pub decode_ms_last: f64,
    pub decode_ms_mean: f64,
    pub decode_ms_max: f64,
}

impl IngestStats {
    pub fn snapshot(&self) -> IngestSnapshot {
        let ms = |us: u64| Duration::from_micros(us).as_secs_f64() * 1000.0;
        let decoded = self.decoded.load(Ordering::Relaxed);
        let total = self.decode_us_total.load(Ordering::Relaxed);
        IngestSnapshot {
            packets: self.accepted.load(Ordering::Relaxed),
            refused: self.refused.load(Ordering::Relaxed),
            frames_decoded: decoded,
            decode_ms_last: ms(self.decode_us_last.load(Ordering::Relaxed)),
            decode_ms_mean: ms(total.checked_div(decoded).unwrap_or(0)),
            decode_ms_max: ms(self.decode_us_max.load(Ordering::Relaxed)),
        }
    }

    fn waiting(&self) -> MutexGuard<'_, VecDeque<(gst::ClockTime, Instant)>> {
        self.waiting.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn entered(&self, pts: gst::ClockTime) {
        let mut waiting = self.waiting();
        if waiting.len() >= DECODES_IN_FLIGHT {
            waiting.pop_front();
        }
        waiting.push_back((pts, Instant::now()));
    }

    /// A frame the decoder dropped never leaves, so its entry is matched by time, not by order.
    fn left(&self, pts: gst::ClockTime) {
        let mut waiting = self.waiting();
        let Some(at) = waiting.iter().position(|(entered, _)| *entered == pts) else {
            return;
        };
        let Some((_, entered)) = waiting.remove(at) else {
            return;
        };
        drop(waiting);
        let us = u64::try_from(entered.elapsed().as_micros()).unwrap_or(u64::MAX);
        self.decoded.fetch_add(1, Ordering::Relaxed);
        self.decode_us_last.store(us, Ordering::Relaxed);
        self.decode_us_total.fetch_add(us, Ordering::Relaxed);
        self.decode_us_max.fetch_max(us, Ordering::Relaxed);
    }
}

/// The reading thread; dropping it stops the thread and releases the socket.
#[derive(Debug)]
pub struct Reader {
    stop: Arc<AtomicBool>,
}

impl Drop for Reader {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// The ingest's elements up to and including the decoder, and the reader that feeds them.
pub fn source(
    listen: SocketAddr,
    from: &[IpAddr],
    stats: &Arc<IngestStats>,
) -> Result<(Vec<gst::Element>, Reader), MediaError> {
    if from.is_empty() {
        return Err(MediaError::Config(
            "an ingest source needs at least one address in from".to_owned(),
        ));
    }
    let socket_error = |e: std::io::Error| MediaError::Socket {
        addr: listen,
        reason: e.to_string(),
    };
    let socket = UdpSocket::bind(listen).map_err(socket_error)?;
    socket.set_read_timeout(Some(POLL)).map_err(socket_error)?;
    let appsrc = gst_app::AppSrc::builder()
        .caps(&RTP_CAPS.parse::<gst::Caps>()?)
        .is_live(true)
        .format(gst::Format::Time)
        .do_timestamp(true)
        .block(false)
        .max_bytes(QUEUE_BYTES)
        .build();
    appsrc.set_property_from_str("leaky-type", "downstream");
    // Frame threading holds frames back; slices only, whatever libav guesses about the source.
    // A picture broken by loss is dropped, not stamped as fresh.
    let decoder = gst::ElementFactory::make("avdec_h264")
        .property_from_str("thread-type", "slice")
        .property("output-corrupt", false)
        .build()?;
    time_decode(&decoder, stats)?;
    let elements = vec![
        appsrc.clone().upcast(),
        gst::ElementFactory::make("rtph264depay").build()?,
        gst::ElementFactory::make("h264parse").build()?,
        decoder,
    ];
    let stop = Arc::new(AtomicBool::new(false));
    let (from, stats, flag) = (from.to_vec(), Arc::clone(stats), Arc::clone(&stop));
    thread::spawn(move || read(&socket, &appsrc, &from, &stats, &flag));
    Ok((elements, Reader { stop }))
}

/// Reads until told to stop. A push into a pipeline that is not playing yet fails and the packet
/// is lost, as on the wire.
fn read(
    socket: &UdpSocket,
    appsrc: &gst_app::AppSrc,
    from: &[IpAddr],
    stats: &IngestStats,
    stop: &AtomicBool,
) {
    let mut datagram = [0_u8; MAX_DATAGRAM];
    while !stop.load(Ordering::Relaxed) {
        let Ok((len, peer)) = socket.recv_from(&mut datagram) else {
            continue;
        };
        let Some(packet) = datagram.get(..len).filter(|_| len < MAX_DATAGRAM) else {
            stats.refused.fetch_add(1, Ordering::Relaxed);
            continue;
        };
        if !accepts(from, peer.ip()) {
            stats.refused.fetch_add(1, Ordering::Relaxed);
            continue;
        }
        stats.accepted.fetch_add(1, Ordering::Relaxed);
        let _ = appsrc.push_buffer(gst::Buffer::from_slice(packet.to_vec()));
    }
}

/// Times each frame from the decoder's input to its output, matched by presentation time.
fn time_decode(decoder: &gst::Element, stats: &Arc<IngestStats>) -> Result<(), MediaError> {
    let pad = |name| {
        decoder
            .static_pad(name)
            .ok_or(MediaError::Missing("decoder pad"))
    };
    let entering = Arc::clone(stats);
    pad("sink")?.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        if let Some(pts) = info.buffer().and_then(|buffer| buffer.pts()) {
            entering.entered(pts);
        }
        gst::PadProbeReturn::Ok
    });
    let leaving = Arc::clone(stats);
    pad("src")?.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        if let Some(pts) = info.buffer().and_then(|buffer| buffer.pts()) {
            leaving.left(pts);
        }
        gst::PadProbeReturn::Ok
    });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn takes_only_the_expected_addresses() {
        let expected: Vec<IpAddr> = vec!["192.168.234.1".parse().unwrap(), "::1".parse().unwrap()];
        let cases = [
            ("192.168.234.1", true),
            ("::ffff:192.168.234.1", true),
            ("192.168.234.6", false),
            ("10.77.0.3", false),
            ("::1", true),
            ("::2", false),
        ];
        for (peer, taken) in cases {
            assert_eq!(accepts(&expected, peer.parse().unwrap()), taken, "{peer}");
        }
        assert!(!accepts(&[], "192.168.234.1".parse().unwrap()));
    }
}
