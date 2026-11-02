//! The sender's last stage: RTP packets go through a backlog of the vehicle's own to the relay at
//! the pace the uplink measured allows (`Controller::pace_bytes_per_s`), so a video faster than
//! the uplink waits here, where it can be dropped, and not in the network's queue, where the
//! control envelopes wait behind it. Control, liveness, challenges and telemetry do not pass
//! through it.

use std::collections::VecDeque;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::Ordering;
use std::sync::{Arc, Condvar, Mutex, PoisonError, Weak};
use std::thread;
use std::time::{Duration, Instant};

use gstreamer as gst;
use gstreamer_app as gst_app;

use sat_teleop_policy::budget::DATAGRAM_OVERHEAD;
use sat_teleop_policy::pace::{Admission, Verdict};
use sat_teleop_policy::rate::{Priority, TokenBucket};

use super::{RTP_MTU, SenderStats};
use crate::MediaError;

/// The longest a packet may wait in the backlog, below the 200 ms of queue at which the control
/// envelopes were refused (`docs/architecture.md`, "Control protocol").
const MAX_AGE_MS: u64 = 150;
/// The bucket holds this much of the pace, and at least two packets.
const BURST_MS: u32 = 20;
const IDLE_WAIT: Duration = Duration::from_millis(50);
const TOKEN_WAIT: Duration = Duration::from_millis(1);

#[derive(Default)]
struct Backlog {
    packets: VecDeque<Vec<u8>>,
    /// The wire bytes of `packets`.
    bytes: u32,
    admission: Admission,
    /// The presentation time of the last packet queued or dropped: a packet with another starts a
    /// frame.
    last_pts: Option<u64>,
}

#[derive(Default)]
struct Shared {
    backlog: Mutex<Backlog>,
    ready: Condvar,
}

fn wire_bytes(len: usize) -> u32 {
    u32::try_from(len)
        .unwrap_or(u32::MAX)
        .saturating_add(DATAGRAM_OVERHEAD)
}

/// An appsink that feeds the pacer thread, which sends to `destination`. The thread ends when the
/// pipeline, and with it the sink's callback, is gone.
pub(super) fn sink(
    destination: SocketAddr,
    stats: &Arc<SenderStats>,
) -> Result<gst_app::AppSink, MediaError> {
    let socket = UdpSocket::bind(("0.0.0.0", 0)).map_err(|e| MediaError::Socket {
        addr: destination,
        reason: e.to_string(),
    })?;
    let shared = Arc::new(Shared::default());
    let weak = Arc::downgrade(&shared);
    let sender_stats = Arc::clone(stats);
    thread::spawn(move || send(&weak, &socket, destination, &sender_stats));
    let stats = Arc::clone(stats);
    let sink = gst_app::AppSink::builder().sync(false).build();
    sink.set_callbacks(
        gst_app::AppSinkCallbacks::builder()
            .new_sample(move |sink| {
                let sample = sink.pull_sample().map_err(|_| gst::FlowError::Eos)?;
                offer(&shared, &stats, &sample);
                Ok(gst::FlowSuccess::Ok)
            })
            .build(),
    );
    Ok(sink)
}

/// Queues the packets of a sample, or drops them as the admission says.
fn offer(shared: &Shared, stats: &SenderStats, sample: &gst::Sample) {
    let buffers: Vec<&gst::BufferRef> = match (sample.buffer_list(), sample.buffer()) {
        (Some(list), _) => list.iter().collect(),
        (None, Some(buffer)) => vec![buffer],
        (None, None) => return,
    };
    let rate = stats.pace.load(Ordering::Relaxed);
    let mut backlog = shared
        .backlog
        .lock()
        .unwrap_or_else(PoisonError::into_inner);
    for buffer in buffers {
        let Ok(map) = buffer.map_readable() else {
            continue;
        };
        let data = map.as_slice();
        let keyframe = !buffer.flags().contains(gst::BufferFlags::DELTA_UNIT);
        let bytes = wire_bytes(data.len());
        let pts = buffer.pts().map_or(u64::MAX, gst::ClockTime::nseconds);
        let starts_frame = backlog.last_pts != Some(pts);
        backlog.last_pts = Some(pts);
        let verdict = if rate == 0 {
            Verdict::Queue
        } else {
            let waiting = backlog.bytes;
            backlog
                .admission
                .offer(starts_frame, keyframe, waiting, rate, MAX_AGE_MS)
        };
        match verdict {
            Verdict::Queue => {
                backlog.bytes = backlog.bytes.saturating_add(bytes);
                backlog.packets.push_back(data.to_vec());
            }
            Verdict::Drop | Verdict::DropAndRecover => {
                stats.paced_dropped.fetch_add(1, Ordering::Relaxed);
                stats
                    .keyframe_wanted
                    .fetch_or(verdict == Verdict::DropAndRecover, Ordering::Relaxed);
            }
        }
    }
    shared.ready.notify_one();
}

fn send(shared: &Weak<Shared>, socket: &UdpSocket, destination: SocketAddr, stats: &SenderStats) {
    let mut bucket = None::<(u32, TokenBucket)>;
    loop {
        let Some(shared) = shared.upgrade() else {
            return;
        };
        let Some(packet) = next(&shared) else {
            continue;
        };
        let rate = stats.pace.load(Ordering::Relaxed);
        if rate > 0 {
            wait_for_tokens(&mut bucket, rate, wire_bytes(packet.len()));
        }
        if socket.send_to(&packet, destination).is_ok() {
            stats.rtp.packets.fetch_add(1, Ordering::Relaxed);
            stats
                .rtp
                .bytes
                .fetch_add(packet.len() as u64, Ordering::Relaxed);
        }
    }
}

fn next(shared: &Shared) -> Option<Vec<u8>> {
    let mut backlog = shared
        .backlog
        .lock()
        .unwrap_or_else(PoisonError::into_inner);
    if backlog.packets.is_empty() {
        backlog = shared
            .ready
            .wait_timeout(backlog, IDLE_WAIT)
            .unwrap_or_else(PoisonError::into_inner)
            .0;
    }
    let packet = backlog.packets.pop_front()?;
    backlog.bytes = backlog.bytes.saturating_sub(wire_bytes(packet.len()));
    Some(packet)
}

/// Blocks until the bucket for `rate` grants `bytes`; the bucket starts over when the rate changes.
fn wait_for_tokens(bucket: &mut Option<(u32, TokenBucket)>, rate: u32, bytes: u32) {
    let burst = (rate / 1000 * BURST_MS).max(2 * (RTP_MTU + DATAGRAM_OVERHEAD));
    if bucket.as_ref().is_none_or(|(paced, _)| *paced != rate) {
        *bucket = Some((rate, TokenBucket::with_burst(rate, burst)));
    }
    let Some((_, bucket)) = bucket else {
        return;
    };
    while !bucket.take(Instant::now(), bytes.min(burst), Priority::Preferred) {
        thread::sleep(TOKEN_WAIT);
    }
}
