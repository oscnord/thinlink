//! Relay ingress: RTP/H.264 from the vehicle over the overlay -> jitter buffer -> depayload ->
//! parse -> RTSP publish to the local `MediaMTX`.

use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use gstreamer as gst;
use gstreamer::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{DATA_PROBE, Health, HealthSnapshot, MediaError, Running, Traffic, failure};

/// Longest wait at the relay for a missing packet; one arriving later is dropped.
pub const JITTER_LATENCY_MS: u32 = 50;
/// How often the jitter buffer statistics are copied into [`IngressStats`]: often enough for the
/// loss each feedback answer reports (one every probe interval).
const STATS_EVERY: gst::ClockTime = gst::ClockTime::from_mseconds(100);

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IngressConfig {
    /// The relay's overlay address and RTP port. Binding to the overlay keeps the uplink off
    /// `ground_net`, where it would bypass `WireGuard`.
    pub listen: SocketAddr,
    /// RTSP URL of the local `MediaMTX` path to publish to.
    pub publish: String,
}

/// Counters shared between the pipeline and the metrics endpoint. Jitter buffer counts are totals
/// across pipeline restarts; `avg_jitter_us` is the current pipeline's.
#[derive(Debug, Default)]
pub struct IngressStats {
    pub health: Health,
    rtp: Traffic,
    pushed: AtomicU64,
    lost: AtomicU64,
    late: AtomicU64,
    duplicates: AtomicU64,
    avg_jitter_us: AtomicU64,
}

#[derive(Debug, Serialize)]
pub struct IngressSnapshot {
    /// RTP packets and bytes received from the vehicle, before the jitter buffer.
    pub rtp_packets: u64,
    pub rtp_bytes: u64,
    pub pushed: u64,
    pub lost: u64,
    pub late: u64,
    pub duplicates: u64,
    pub avg_jitter_us: u64,
    #[serde(flatten)]
    pub health: HealthSnapshot,
}

impl IngressStats {
    pub fn snapshot(&self) -> IngressSnapshot {
        let rtp = self.rtp.snapshot();
        IngressSnapshot {
            rtp_packets: rtp.packets,
            rtp_bytes: rtp.bytes,
            pushed: self.pushed.load(Ordering::Relaxed),
            lost: self.lost.load(Ordering::Relaxed),
            late: self.late.load(Ordering::Relaxed),
            duplicates: self.duplicates.load(Ordering::Relaxed),
            avg_jitter_us: self.avg_jitter_us.load(Ordering::Relaxed),
            health: self.health.snapshot(),
        }
    }
}

/// Jitter buffer counts of one pipeline run, added to the totals carried from earlier runs.
#[derive(Debug, Clone, Copy, Default)]
struct JitterCounts {
    pushed: u64,
    lost: u64,
    late: u64,
    duplicates: u64,
}

impl JitterCounts {
    /// The counts and the average jitter in microseconds; `None` when the element has no
    /// readable `stats` structure.
    fn read(jitterbuffer: &gst::Element) -> Option<(Self, u64)> {
        let readable = jitterbuffer
            .find_property("stats")
            .is_some_and(|pspec| pspec.flags().contains(gst::glib::ParamFlags::READABLE));
        let stats = readable
            .then(|| jitterbuffer.property_value("stats"))?
            .get::<gst::Structure>()
            .ok()?;
        let field = |name: &str| stats.get::<u64>(name).unwrap_or(0);
        let counts = Self {
            pushed: field("num-pushed"),
            lost: field("num-lost"),
            late: field("num-late"),
            duplicates: field("num-duplicates"),
        };
        Some((counts, field("avg-jitter") / 1000))
    }

    fn totals(stats: &IngressStats) -> Self {
        Self {
            pushed: stats.pushed.load(Ordering::Relaxed),
            lost: stats.lost.load(Ordering::Relaxed),
            late: stats.late.load(Ordering::Relaxed),
            duplicates: stats.duplicates.load(Ordering::Relaxed),
        }
    }

    fn store(stats: &IngressStats, base: Self, run: Self) {
        let add = |counter: &AtomicU64, base: u64, run: u64| {
            counter.store(base.saturating_add(run), Ordering::Relaxed);
        };
        add(&stats.pushed, base.pushed, run.pushed);
        add(&stats.lost, base.lost, run.lost);
        add(&stats.late, base.late, run.late);
        add(&stats.duplicates, base.duplicates, run.duplicates);
    }
}

/// Builds and plays the ingress until it fails. Never returns `Ok`; run it under
/// [`crate::supervise`].
pub fn run(config: &IngressConfig, stats: &Arc<IngressStats>) -> Result<(), MediaError> {
    let (pipeline, jitterbuffer) = build(config, stats)?;
    let running = Running(pipeline);
    running.set(gst::State::Playing)?;
    let bus = running.0.bus().ok_or(MediaError::Missing("bus"))?;
    let base = JitterCounts::totals(stats);
    let mut warned = false;
    loop {
        let watched = [gst::MessageType::Error, gst::MessageType::Eos];
        if let Some(error) = bus
            .timed_pop_filtered(STATS_EVERY, &watched)
            .and_then(|message| failure(&message))
        {
            return Err(error);
        }
        match JitterCounts::read(&jitterbuffer) {
            Some((run_counts, avg_jitter_us)) => {
                JitterCounts::store(stats, base, run_counts);
                stats.avg_jitter_us.store(avg_jitter_us, Ordering::Relaxed);
            }
            None if !warned => {
                eprintln!(
                    "rtpjitterbuffer has no readable stats; loss and jitter metrics stay put"
                );
                warned = true;
            }
            None => {}
        }
    }
}

/// The relay forwards, it does not re-time: the sinks rtspclientsink creates would otherwise hold
/// each frame until the jitter buffer's timestamp plus its latency.
fn forward_without_sync(element: &gst::Element) {
    let is_sink = element.element_flags().contains(gst::ElementFlags::SINK);
    let Some(pspec) = element.find_property("sync").filter(|_| is_sink) else {
        return;
    };
    let flags = pspec.flags();
    let settable = pspec.value_type() == bool::static_type()
        && flags.contains(gst::glib::ParamFlags::WRITABLE)
        && !flags.contains(gst::glib::ParamFlags::CONSTRUCT_ONLY);
    if !settable {
        eprintln!(
            "{}: sync is not a writable boolean, left as is",
            element.name()
        );
        return;
    }
    element.set_property("sync", false);
}

fn build(
    config: &IngressConfig,
    stats: &Arc<IngressStats>,
) -> Result<(gst::Pipeline, gst::Element), MediaError> {
    let rtp_caps = gst::Caps::builder("application/x-rtp")
        .field("media", "video")
        .field("clock-rate", 90_000i32)
        .field("encoding-name", "H264")
        .field("payload", 96i32)
        .build();
    let source = gst::ElementFactory::make("udpsrc")
        .property("address", config.listen.ip().to_string())
        .property("port", i32::from(config.listen.port()))
        .property("caps", &rtp_caps)
        .build()?;
    let jitterbuffer = gst::ElementFactory::make("rtpjitterbuffer")
        .property("latency", JITTER_LATENCY_MS)
        .property("drop-on-latency", true)
        .build()?;
    let chain = [
        source.clone(),
        jitterbuffer.clone(),
        gst::ElementFactory::make("rtph264depay").build()?,
        gst::ElementFactory::make("h264parse")
            .property("config-interval", -1i32)
            .build()?,
        gst::ElementFactory::make("rtspclientsink")
            .property("location", &config.publish)
            .property_from_str("protocols", "tcp")
            .build()?,
    ];
    let pipeline = gst::Pipeline::new();
    pipeline.connect_deep_element_added(|_, _, element| forward_without_sync(element));
    pipeline.add_many(&chain)?;
    gst::Element::link_many(&chain)?;
    let stats = Arc::clone(stats);
    source
        .static_pad("src")
        .ok_or(MediaError::Missing("src pad"))?
        .add_probe(DATA_PROBE, move |_, info| {
            stats.rtp.record(info.data.as_ref());
            gst::PadProbeReturn::Ok
        });
    Ok((pipeline, jitterbuffer))
}
