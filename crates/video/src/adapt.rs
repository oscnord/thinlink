//! The vehicle's side of adaptation: it probes the relay every probe interval, turns each answer
//! into feedback for `sat_teleop_video_core::adapt::Controller` and gives the sender the rung the controller
//! picks. The uplink the controller measured at its last cut also caps the telemetry, through the
//! caller, from the same budget (`sat_teleop_video_core::budget`), sized to what the autopilot offers.

use std::collections::VecDeque;
use std::io;
use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::sync::{Mutex, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use sat_teleop_video_core::adapt::{
    Controller, Decision, Feedback, Group, Ladder, Reason, Rung as LadderRung, Thresholds,
};
use sat_teleop_video_core::budget::{Budget, DATAGRAM_OVERHEAD, Usage, wire_bytes};

use crate::feedback::{self, MAX_DATAGRAM, Probe, Report};
use crate::sender::{Rung, SenderStats};
use crate::{MediaError, TrafficSnapshot, unix_ms};

/// Probes whose answer may still come; an answer to an older one is ignored.
const OUTSTANDING: usize = 64;
/// The relay's windows are summed over at least this much: while a queue fills, the uplink
/// delivers at its rate continuously, so half a second measures it.
const WINDOW_SPAN: Duration = Duration::from_millis(500);
/// The vehicle's own counters are read over at least this much: SITL sends its telemetry in one
/// burst a second, so a shorter span would count the burst or miss it.
const OWN_SPAN: Duration = Duration::from_secs(1);
/// What the autopilot offers is averaged over this much, so the telemetry share follows a burst
/// of replies (a parameter list) at once and lets go of it a few seconds after.
const OFFERED_SPAN: Duration = Duration::from_secs(5);
/// Counter readings kept for the rates: more than `OFFERED_SPAN` at any sane probe interval.
const READINGS: usize = 64;
const RECV_ERROR_PAUSE: Duration = Duration::from_millis(100);
/// Rung changes kept for the metrics.
const RECENT_CHANGES: usize = 16;
/// Durations in `[adapt]` stay below ten minutes.
const MAX_MS: u64 = 600_000;

/// `[adapt]` of `testbed/limits.toml`; `docs/measurement.md` derives the values.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AdaptLimits {
    pub probe_interval_ms: u64,
    pub min_rtt_window_ms: u64,
    pub queue_delay_high_ms: u64,
    pub drain_above_ms: u64,
    pub queue_delay_base_ms: u64,
    pub trend_window_ms: u64,
    pub trend_ms_per_s: u64,
    pub capacity_fit_percent: u32,
    pub loss_window_ms: u64,
    pub loss_min_packets: u32,
    pub loss_low_percent: u32,
    pub loss_high_percent: u32,
    pub stale_after_ms: u64,
    pub outage_recovery_ms: u64,
    pub dwell_ms: u64,
    pub min_change_interval_ms: u64,
    pub capacity_hold_ms: u64,
    pub capacity_hold_max_ms: u64,
}

impl AdaptLimits {
    /// The probe interval and the controller's thresholds.
    pub fn validate(self) -> Result<(Duration, Thresholds), MediaError> {
        let durations = [
            self.probe_interval_ms,
            self.min_rtt_window_ms,
            self.queue_delay_high_ms,
            self.drain_above_ms,
            self.trend_window_ms,
            self.loss_window_ms,
            self.stale_after_ms,
            self.outage_recovery_ms,
            self.dwell_ms,
            self.min_change_interval_ms,
            self.capacity_hold_ms,
            self.capacity_hold_max_ms,
        ];
        let rules = [
            (
                durations.iter().all(|ms| (1..=MAX_MS).contains(ms)),
                "durations must be 1-600000 ms",
            ),
            (
                self.queue_delay_base_ms < self.queue_delay_high_ms,
                "queue_delay_base_ms must be below queue_delay_high_ms",
            ),
            (
                self.drain_above_ms > self.queue_delay_high_ms,
                "drain_above_ms must exceed queue_delay_high_ms",
            ),
            (
                self.loss_low_percent <= self.loss_high_percent && self.loss_high_percent <= 100,
                "loss_low_percent <= loss_high_percent <= 100",
            ),
            (
                self.trend_window_ms >= 2 * self.probe_interval_ms
                    && self.stale_after_ms >= 2 * self.probe_interval_ms,
                "the trend window and the stale threshold must span two probes",
            ),
            (
                self.min_change_interval_ms <= self.dwell_ms,
                "min_change_interval_ms must not exceed dwell_ms",
            ),
            (
                self.capacity_hold_ms <= self.capacity_hold_max_ms,
                "capacity_hold_ms must not exceed capacity_hold_max_ms",
            ),
            (
                (1..100).contains(&self.capacity_fit_percent)
                    && self.trend_ms_per_s.saturating_mul(2)
                        <= u64::from(100 - self.capacity_fit_percent.min(100)) * 10,
                "capacity_fit_percent must be 1-99 and leave a drain of twice trend_ms_per_s",
            ),
        ];
        if let Some((_, rule)) = rules.iter().find(|(holds, _)| !holds) {
            return Err(MediaError::Config(format!("[adapt]: {rule}")));
        }
        let thresholds = Thresholds {
            min_rtt_window_ms: self.min_rtt_window_ms,
            queue_delay_high_ms: self.queue_delay_high_ms,
            drain_above_ms: self.drain_above_ms,
            queue_delay_base_ms: self.queue_delay_base_ms,
            trend_window_ms: self.trend_window_ms,
            trend_ms_per_s: self.trend_ms_per_s,
            capacity_fit_percent: self.capacity_fit_percent,
            loss_window_ms: self.loss_window_ms,
            loss_min_packets: self.loss_min_packets,
            loss_low_percent: self.loss_low_percent,
            loss_high_percent: self.loss_high_percent,
            stale_after_ms: self.stale_after_ms,
            outage_recovery_ms: self.outage_recovery_ms,
            dwell_ms: self.dwell_ms,
            min_change_interval_ms: self.min_change_interval_ms,
            capacity_hold_ms: self.capacity_hold_ms,
            capacity_hold_max_ms: self.capacity_hold_max_ms,
        };
        Ok((Duration::from_millis(self.probe_interval_ms), thresholds))
    }
}

/// `testbed/ladder.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LadderConfig {
    /// A keyframe this often at every rung.
    pub keyframe_interval_ms: u32,
    /// The rung the vehicle starts at, counted from the best (0).
    pub start: usize,
    /// Best first.
    pub rung: Vec<RungConfig>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RungConfig {
    pub group: GroupName,
    pub width: u16,
    pub height: u16,
    pub fps: u16,
    pub bitrate_kbit: u32,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GroupName {
    Normal,
    Constrained,
    AwarenessOnly,
}

/// The validated ladder and the sender's encoding of each rung.
#[derive(Debug, Clone)]
pub struct Rungs {
    pub ladder: Ladder,
    pub encodings: Vec<Rung>,
    pub start: usize,
}

impl Rungs {
    pub fn start_encoding(&self) -> Option<Rung> {
        self.encodings.get(self.start).copied()
    }
}

impl LadderConfig {
    pub fn validate(&self) -> Result<Rungs, MediaError> {
        let encodings: Vec<Rung> = self
            .rung
            .iter()
            .map(|r| Rung {
                width: r.width,
                height: r.height,
                fps: r.fps,
                bitrate_kbit: r.bitrate_kbit,
                keyframe_interval_ms: self.keyframe_interval_ms,
            })
            .collect();
        for (i, encoding) in encodings.iter().enumerate() {
            encoding
                .validate()
                .map_err(|e| MediaError::Config(format!("rung {i}: {e}")))?;
        }
        let rungs = self.rung.iter().map(|r| LadderRung {
            width: r.width,
            height: r.height,
            fps: r.fps,
            bitrate_kbit: r.bitrate_kbit,
            group: match r.group {
                GroupName::Normal => Group::Normal,
                GroupName::Constrained => Group::Constrained,
                GroupName::AwarenessOnly => Group::AwarenessOnly,
            },
        });
        let ladder = Ladder::new(rungs.collect()).map_err(|e| MediaError::Config(e.to_string()))?;
        if ladder.rung(self.start).is_none() {
            return Err(MediaError::Config(format!(
                "start {} is not a rung of the ladder",
                self.start
            )));
        }
        Ok(Rungs {
            ladder,
            encodings,
            start: self.start,
        })
    }
}

/// What the vehicle sent besides video and probes, counted since it started: telemetry and the
/// control endpoint's replies, in datagrams and payload bytes.
#[derive(Debug, Clone, Copy, Default)]
pub struct Uplink {
    /// Telemetry frame bytes the autopilot offered, sent or not.
    pub telemetry_offered: u64,
    pub telemetry_datagrams: u64,
    pub telemetry_bytes: u64,
    pub replies: u64,
    pub reply_bytes: u64,
}

impl Uplink {
    fn wire(self) -> u64 {
        let datagrams = self.telemetry_datagrams.saturating_add(self.replies);
        let payload = self.telemetry_bytes.saturating_add(self.reply_bytes);
        payload.saturating_add(datagrams.saturating_mul(u64::from(DATAGRAM_OVERHEAD)))
    }

    fn reply_wire(self) -> u64 {
        self.reply_bytes
            .saturating_add(self.replies.saturating_mul(u64::from(DATAGRAM_OVERHEAD)))
    }
}

/// Everything the loop needs besides its callers' counters.
#[derive(Debug, Clone)]
pub struct Setup {
    /// The relay's feedback address.
    pub relay: SocketAddr,
    pub probe_every: Duration,
    pub thresholds: Thresholds,
    pub rungs: Rungs,
    pub budget: Budget,
}

/// The largest probe on the wire, for the control reserve.
pub fn probe_wire_bytes() -> u32 {
    let largest = Probe {
        seq: u64::MAX,
        sent_mono_us: u64::MAX,
    };
    let len = serde_json::to_vec(&largest).map_or(MAX_DATAGRAM, |bytes| bytes.len());
    wire_bytes(u32::try_from(len).unwrap_or(u32::MAX))
}

#[derive(Debug, Default)]
pub struct AdaptStats {
    snapshot: Mutex<AdaptSnapshot>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct AdaptSnapshot {
    /// The current rung, counted from the best (0), and its group.
    pub rung: usize,
    pub group: Option<&'static str>,
    #[serde(skip)]
    pub group_kind: Option<Group>,
    /// When the relay last answered a probe, on the monotonic clock.
    #[serde(skip)]
    last_answer: Option<Instant>,
    pub probes: u64,
    pub send_failed: u64,
    pub answers: u64,
    /// Datagrams that were no answer to an outstanding probe, or came from anyone but the relay.
    pub refused: u64,
    pub receive_errors: u64,
    /// Rung changes, drains included.
    pub changes: u64,
    /// The newest rung changes and the video resuming after a drain, oldest first.
    pub recent_changes: VecDeque<ChangeView>,
    /// The video is paused while the uplink queue drains.
    pub draining: bool,
    pub rtt_ms: Option<u64>,
    pub min_rtt_ms: Option<u64>,
    pub queue_delay_ms: Option<u64>,
    /// Video packets lost and all video packets in the controller's loss window.
    pub loss_lost: Option<u32>,
    pub loss_packets: Option<u32>,
    /// Everything the uplink carried from the vehicle at the last answer, on the wire.
    pub uplink_bytes_per_s: Option<u32>,
    /// The uplink measured at the last cut on a rising queue, while it bounds the rungs up.
    pub capacity_bytes_per_s: Option<u32>,
    /// The telemetry cap now, in `MAVLink` frame bytes per second.
    pub telemetry_cap: Option<u32>,
    /// The uplink carries a normal rung beside the reserved flow's allowance (`Controller::reserve_fits`).
    pub reserve_fits: bool,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct ChangeView {
    pub to: usize,
    pub group: &'static str,
    pub reason: &'static str,
    pub unix_ms: u64,
    /// Since the loop started, on the monotonic clock.
    pub at_ms: u64,
}

impl AdaptStats {
    pub fn snapshot(&self) -> AdaptSnapshot {
        self.lock().clone()
    }

    /// Milliseconds since the relay last answered a probe; `None` if it never did.
    pub fn since_answer_ms(&self, now: Instant) -> Option<u64> {
        self.lock()
            .last_answer
            .map(|at| duration_ms(now.saturating_duration_since(at)))
    }

    fn update(&self, f: impl FnOnce(&mut AdaptSnapshot)) {
        f(&mut self.lock());
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, AdaptSnapshot> {
        self.snapshot.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

/// One reading of the vehicle's own counters.
#[derive(Debug, Clone, Copy)]
struct Reading {
    at: Instant,
    uplink: Uplink,
    probes: u64,
    rtp: TrafficSnapshot,
    frames: u64,
    /// The video was paused for a drain.
    paused: bool,
}

impl Reading {
    fn video_wire(&self) -> u64 {
        let headers = self
            .rtp
            .packets
            .saturating_mul(u64::from(DATAGRAM_OVERHEAD));
        self.rtp.bytes.saturating_add(headers)
    }
}

/// Rates from the relay's windows and the vehicle's counters.
#[derive(Debug)]
struct Meter {
    probe_wire: u64,
    /// The relay's windows, newest last: their length and the video's bytes on the wire.
    windows: VecDeque<(u64, u64)>,
    readings: VecDeque<Reading>,
    /// What the vehicle sent besides video, and its video, over the newest `OWN_SPAN` it sent
    /// video throughout: the mix a queue holds and lets out, also while the video is paused.
    mix: Option<(u32, u32)>,
}

impl Meter {
    fn read(&mut self, reading: Reading) {
        if self.readings.len() >= READINGS {
            self.readings.pop_front();
        }
        self.readings.push_back(reading);
        let Some((oldest, newest, seconds)) = self.span(OWN_SPAN) else {
            return;
        };
        let video = per_second(
            newest.video_wire().saturating_sub(oldest.video_wire()),
            seconds,
        );
        let others = per_second(
            self.rest(&newest).saturating_sub(self.rest(&oldest)),
            seconds,
        );
        if video > 0 && !self.paused_since(oldest.at) {
            self.mix = Some((others, video));
        }
    }

    /// The vehicle's own traffic besides video on the wire so far: telemetry, replies and probes.
    fn rest(&self, r: &Reading) -> u64 {
        r.uplink
            .wire()
            .saturating_add(r.probes.saturating_mul(self.probe_wire))
    }

    fn paused_since(&self, at: Instant) -> bool {
        self.readings.iter().any(|r| r.at >= at && r.paused)
    }

    fn window(&mut self, report: &Report) {
        let wire = report.rx_bytes.saturating_add(
            report
                .rx_packets
                .saturating_mul(u64::from(DATAGRAM_OVERHEAD)),
        );
        if self.windows.len() >= READINGS {
            self.windows.pop_front();
        }
        self.windows.push_back((report.window_ms, wire));
    }

    /// The newest reading and the newest one at least `span` older, with the seconds between.
    fn span(&self, span: Duration) -> Option<(Reading, Reading, f64)> {
        let newest = *self.readings.back()?;
        let oldest = *self
            .readings
            .iter()
            .rev()
            .find(|r| newest.at.saturating_duration_since(r.at) >= span)?;
        let seconds = newest.at.saturating_duration_since(oldest.at).as_secs_f64();
        Some((oldest, newest, seconds))
    }

    /// Per second over at least `span`: what `count` grew by; `None` until there is that much.
    fn rate(&self, span: Duration, count: impl Fn(&Reading) -> u64) -> Option<u32> {
        self.span(span).map(|(oldest, newest, seconds)| {
            per_second(count(&newest).saturating_sub(count(&oldest)), seconds)
        })
    }

    fn usage(&self) -> Usage {
        let per_frame = self.span(OWN_SPAN).map_or(1, |(oldest, newest, _)| {
            let frames = newest.frames.saturating_sub(oldest.frames);
            let packets = newest.rtp.packets.saturating_sub(oldest.rtp.packets);
            u32::try_from(packets.div_ceil(frames.max(1))).unwrap_or(u32::MAX)
        });
        Usage {
            telemetry_offered: self.rate(OFFERED_SPAN, |r| r.uplink.telemetry_offered),
            telemetry_datagrams_per_s: self
                .rate(OWN_SPAN, |r| r.uplink.telemetry_datagrams)
                .unwrap_or(0),
            control: self
                .rate(OWN_SPAN, |r| {
                    r.uplink
                        .reply_wire()
                        .saturating_add(r.probes.saturating_mul(self.probe_wire))
                })
                .unwrap_or(0),
            video_packets_per_frame: per_frame.max(1),
        }
    }

    /// The video the relay received over at least `WINDOW_SPAN` of its newest windows, plus what
    /// the vehicle sent besides as the uplink carried it, all on the wire.
    fn uplink_bytes_per_s(&self) -> Option<u32> {
        let mut span_ms = 0_u64;
        let mut wire = 0_u64;
        for (window_ms, bytes) in self.windows.iter().rev() {
            span_ms = span_ms.saturating_add(*window_ms);
            wire = wire.saturating_add(*bytes);
            if span_ms >= duration_ms(WINDOW_SPAN) {
                break;
            }
        }
        if span_ms < duration_ms(WINDOW_SPAN) {
            return None;
        }
        let video = wire.saturating_mul(1000) / span_ms;
        let (oldest, newest, seconds) = self.span(OWN_SPAN)?;
        let others = per_second(
            self.rest(&newest).saturating_sub(self.rest(&oldest)),
            seconds,
        );
        let sent = per_second(
            newest.video_wire().saturating_sub(oldest.video_wire()),
            seconds,
        );
        // A queue lets every kind of traffic out in the share it came in: when the video arrives
        // slower than it is sent, or was paused while the queue drains, the rest got through in
        // the mix that was sent.
        let queued = video < u64::from(sent) || self.paused_since(oldest.at);
        let carried = match self.mix.filter(|_| queued) {
            Some((rest, sent)) => video.saturating_mul(u64::from(rest)) / u64::from(sent.max(1)),
            None => u64::from(others),
        };
        Some(u32::try_from(video.saturating_add(carried)).unwrap_or(u32::MAX))
    }
}

#[expect(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "counts over a second or so, far below 2^52; the rate is rounded to whole bytes"
)]
fn per_second(count: u64, seconds: f64) -> u32 {
    if seconds <= 0.0 {
        return 0;
    }
    (count as f64 / seconds).round().min(f64::from(u32::MAX)) as u32
}

fn duration_ms(duration: Duration) -> u64 {
    u64::try_from(duration.as_millis()).unwrap_or(u64::MAX)
}

/// Probes the relay and adapts until the socket cannot be opened. `uplink` reads the vehicle's
/// other uplink counters; `telemetry_cap` takes the telemetry cap in frame bytes per second
/// (`Budget::telemetry_cap`); `reserved_wanted` is what the reserved flow asks to hold on the wire per
/// second now, zero while idle; `reserved_allowance` is what it would ask for with work waiting.
pub fn run(
    setup: &Setup,
    sender: &SenderStats,
    uplink: impl Fn() -> Uplink,
    telemetry_cap: impl Fn(u32),
    reserved_wanted: impl Fn() -> u32,
    reserved_allowance: u32,
    stats: &AdaptStats,
) -> Result<(), MediaError> {
    let any = SocketAddr::from((Ipv4Addr::UNSPECIFIED, 0));
    let socket = UdpSocket::bind(any).map_err(|e| MediaError::Socket {
        addr: any,
        reason: e.to_string(),
    })?;
    let rungs = &setup.rungs;
    let started = Instant::now();
    let start = sender_rung(sender, rungs);
    let controller = Controller::new(rungs.ladder.clone(), setup.thresholds, setup.budget, start)
        .map_err(|e| MediaError::Config(e.to_string()))?;
    let mut adapter = Adapter {
        setup,
        sender,
        stats,
        uplink,
        telemetry_cap,
        reserved_wanted,
        reserved_allowance,
        controller,
        prober: Prober {
            socket,
            relay: setup.relay,
            started,
            seq: 0,
            outstanding: VecDeque::new(),
        },
        meter: Meter {
            probe_wire: u64::from(probe_wire_bytes()),
            windows: VecDeque::new(),
            readings: VecDeque::new(),
            mix: None,
        },
        cap: None,
    };
    // A loop that failed during a drain left the video paused; the new controller knows of none.
    sender.set_draining(false);
    stats.update(|s| {
        show_rung(s, rungs, start);
        s.draining = false;
    });
    let mut next_probe = started;
    let mut buf = [0_u8; MAX_DATAGRAM + 1];
    loop {
        let now = Instant::now();
        if now >= next_probe {
            next_probe = (next_probe + setup.probe_every).max(now);
            adapter.probe(now);
        }
        let wait = next_probe.saturating_duration_since(Instant::now());
        match adapter
            .prober
            .receive(&mut buf, wait.max(Duration::from_millis(1)))
        {
            Ok(Some(datagram)) => adapter.answered(&datagram),
            Ok(None) => {}
            Err(_) => {
                stats.update(|s| s.receive_errors += 1);
                thread::sleep(RECV_ERROR_PAUSE);
            }
        }
    }
}

/// The loop's state between probes and answers.
struct Adapter<'a, U, C, R> {
    setup: &'a Setup,
    sender: &'a SenderStats,
    stats: &'a AdaptStats,
    uplink: U,
    telemetry_cap: C,
    reserved_wanted: R,
    reserved_allowance: u32,
    controller: Controller,
    prober: Prober,
    meter: Meter,
    /// The telemetry cap last handed out.
    cap: Option<u32>,
}

impl<U: Fn() -> Uplink, C: Fn(u32), R: Fn() -> u32> Adapter<'_, U, C, R> {
    /// Sends a probe, reads the vehicle's counters and lets the controller notice silence.
    fn probe(&mut self, now: Instant) {
        let sent = self.prober.probe();
        self.stats.update(|s| {
            s.probes += 1;
            s.send_failed += u64::from(!sent);
        });
        let (rtp, frames) = self.sender.sent();
        self.meter.read(Reading {
            at: now,
            uplink: (self.uplink)(),
            probes: self.prober.seq,
            rtp,
            frames,
            paused: self.sender.draining(),
        });
        let now_ms = self.prober.ms(now);
        let decision = self.controller.observe(None, now_ms);
        self.apply(decision, now_ms);
    }

    /// Hands an answer to the controller, applies its decision and the telemetry cap, and shows
    /// what the controller made of it.
    fn answered(&mut self, datagram: &[u8]) {
        let Some((report, feedback)) = self.prober.answer(datagram) else {
            self.stats.update(|s| s.refused += 1);
            return;
        };
        self.meter.window(&report);
        let uplink_bytes_per_s = self.meter.uplink_bytes_per_s();
        let usage = self.meter.usage();
        let feedback = Feedback {
            uplink_bytes_per_s,
            usage,
            reserved: (self.reserved_wanted)(),
            ..feedback
        };
        let now_ms = self.prober.ms(Instant::now());
        let decision = self.controller.observe(Some(feedback), now_ms);
        self.apply(decision, now_ms);
        let cap = self
            .setup
            .budget
            .telemetry_cap(self.controller.capacity(), usage);
        if self.cap != Some(cap) {
            (self.telemetry_cap)(cap);
            self.cap = Some(cap);
        }
        self.sender
            .set_pace(self.controller.pace_bytes_per_s(usage));
        let status = self.controller.status(now_ms);
        let reserve_fits = self
            .controller
            .reserve_fits(self.reserved_allowance, usage, now_ms);
        self.stats.update(|s| {
            s.reserve_fits = reserve_fits;
            s.answers += 1;
            s.last_answer = Some(Instant::now());
            s.rtt_ms = status.rtt_ms;
            s.min_rtt_ms = status.min_rtt_ms;
            s.queue_delay_ms = status.queue_delay_ms;
            s.loss_lost = status.loss.map(|(lost, _)| lost);
            s.loss_packets = status.loss.map(|(_, all)| all);
            s.uplink_bytes_per_s = uplink_bytes_per_s;
            s.capacity_bytes_per_s = status.capacity;
            s.telemetry_cap = Some(cap);
        });
    }
}

impl<U, C, R> Adapter<'_, U, C, R> {
    /// Gives the sender the controller's decision at `at_ms` and records it.
    fn apply(&self, decision: Decision, at_ms: u64) {
        match decision {
            Decision::Hold => {}
            Decision::Step {
                to,
                reason: Reason::Probe,
            } => self.pace(to, at_ms),
            Decision::Step { to, reason } => self.change(to, reason.name(), false, at_ms),
            Decision::Drain { to } => self.change(to, "drain", true, at_ms),
            Decision::LinkBack => self.sender.want_keyframe(),
            Decision::Resume => {
                self.sender.set_draining(false);
                let rungs = &self.setup.rungs;
                self.stats.update(|s| {
                    s.draining = false;
                    let to = s.rung;
                    remember(s, rungs, to, "resume", at_ms);
                });
            }
        }
    }

    /// Raises the ceiling of the rung being climbed into: not a rung change.
    fn pace(&self, rung: usize, at_ms: u64) {
        let Some(encoding) = self.encoding(rung) else {
            return;
        };
        self.sender.request(encoding);
        let rungs = &self.setup.rungs;
        self.stats
            .update(|s| remember(s, rungs, rung, Reason::Probe.name(), at_ms));
    }

    fn change(&self, to: usize, reason: &'static str, draining: bool, at_ms: u64) {
        let rungs = &self.setup.rungs;
        let Some(encoding) = self.encoding(to) else {
            return;
        };
        self.sender.set_draining(draining);
        self.sender.request(encoding);
        self.stats.update(|s| {
            show_rung(s, rungs, to);
            s.changes += 1;
            s.draining = draining;
            remember(s, rungs, to, reason, at_ms);
        });
    }
}

impl<U, C, R> Adapter<'_, U, C, R> {
    /// The rung's encoding, at the controller's ceiling while a climb into it is paced.
    fn encoding(&self, rung: usize) -> Option<Rung> {
        let encoding = *self.setup.rungs.encodings.get(rung)?;
        let kbit = self
            .controller
            .ceiling_kbit()
            .map_or(encoding.bitrate_kbit, |kbit| {
                kbit.min(encoding.bitrate_kbit)
            });
        Some(Rung {
            bitrate_kbit: kbit,
            ..encoding
        })
    }
}

fn remember(s: &mut AdaptSnapshot, rungs: &Rungs, to: usize, reason: &'static str, at_ms: u64) {
    if s.recent_changes.len() >= RECENT_CHANGES {
        s.recent_changes.pop_front();
    }
    s.recent_changes.push_back(ChangeView {
        to,
        group: group_name(rungs, to),
        reason,
        unix_ms: unix_ms(),
        at_ms,
    });
}

/// The rung the sender runs, for the controller to start from: a loop restarted after a failure
/// starts where the sender is, so the two never disagree. A sender on none of the ladder's rungs
/// is set to its start.
fn sender_rung(sender: &SenderStats, rungs: &Rungs) -> usize {
    let running = sender
        .requested()
        .and_then(|rung| rungs.encodings.iter().position(|e| *e == rung));
    if let Some(index) = running {
        return index;
    }
    if let Some(encoding) = rungs.start_encoding() {
        sender.request(encoding);
    }
    rungs.start
}

fn show_rung(s: &mut AdaptSnapshot, rungs: &Rungs, index: usize) {
    s.rung = index;
    s.group_kind = rungs.ladder.rung(index).map(|r| r.group);
    s.group = Some(group_name(rungs, index));
}

fn group_name(rungs: &Rungs, index: usize) -> &'static str {
    rungs.ladder.rung(index).map_or("", |r| r.group.name())
}

/// The probe socket and the probes still waiting for an answer.
struct Prober {
    socket: UdpSocket,
    relay: SocketAddr,
    started: Instant,
    seq: u64,
    outstanding: VecDeque<(u64, Instant, u64)>,
}

impl Prober {
    fn ms(&self, at: Instant) -> u64 {
        duration_ms(at.saturating_duration_since(self.started))
    }

    /// Sends the next probe; false when it could not be sent (`wg0` not up yet, for one).
    fn probe(&mut self) -> bool {
        self.seq += 1;
        let now = Instant::now();
        let sent_mono_us =
            u64::try_from(now.saturating_duration_since(self.started).as_micros()).unwrap_or(0);
        let probe = Probe {
            seq: self.seq,
            sent_mono_us,
        };
        if self.outstanding.len() >= OUTSTANDING {
            self.outstanding.pop_front();
        }
        self.outstanding.push_back((self.seq, now, sent_mono_us));
        serde_json::to_vec(&probe)
            .is_ok_and(|bytes| self.socket.send_to(&bytes, self.relay).is_ok())
    }

    /// The next datagram from the relay within `wait`; `None` when none came or it came from
    /// anyone else.
    fn receive(&self, buf: &mut [u8], wait: Duration) -> io::Result<Option<Vec<u8>>> {
        self.socket.set_read_timeout(Some(wait))?;
        match self.socket.recv_from(buf) {
            Ok((len, from)) if from == self.relay => {
                Ok(Some(buf.get(..len).unwrap_or_default().to_vec()))
            }
            Ok(_) => Ok(Some(Vec::new())),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                Ok(None)
            }
            Err(e) => Err(e),
        }
    }

    /// The report in `datagram` and its feedback (rates still to fill in), if it answers an
    /// outstanding probe with that probe's own send time; that probe and every older one are then
    /// done. A malformed, unknown or old answer changes nothing.
    fn answer(&mut self, datagram: &[u8]) -> Option<(Report, Feedback)> {
        let report: Report = feedback::decode(datagram)?;
        let &(seq, sent, sent_mono_us) = self
            .outstanding
            .iter()
            .find(|(seq, _, _)| *seq == report.probe_seq)?;
        if sent_mono_us != report.echo_sent_mono_us {
            return None;
        }
        self.outstanding.retain(|(s, _, _)| *s > seq);
        let clamp = |n: u64| u32::try_from(n).unwrap_or(u32::MAX);
        let feedback = Feedback {
            probe_sent_ms: self.ms(sent),
            rtt_ms: duration_ms(sent.elapsed()),
            received: clamp(report.rx_packets),
            lost: clamp(report.lost_packets),
            uplink_bytes_per_s: None,
            usage: Usage {
                telemetry_offered: None,
                telemetry_datagrams_per_s: 0,
                control: 0,
                video_packets_per_frame: 1,
            },
            reserved: 0,
        };
        Some((report, feedback))
    }
}
