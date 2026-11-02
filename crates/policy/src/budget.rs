//! The uplink budget: what the vehicle may send, split between control, telemetry, retrieval and
//! live video, in that order. Rates are bytes per second on the wire, the bytes the bottleneck
//! carries: payload, inner UDP and IPv4, `WireGuard` with its padding, outer UDP and IPv4 and the
//! Ethernet header.

/// Ethernet (14), outer IPv4 (20) and UDP (8), `WireGuard`'s data message header (16) and its
/// authentication tag (16).
const TUNNEL_BYTES: u32 = 74;
/// Inner IPv4 (20) and UDP (8).
const INNER_HEADERS: u32 = 28;
/// `WireGuard` pads the inner packet to a multiple of 16 bytes.
const PADDING_BLOCK: u32 = 16;
/// The headers around a datagram whose size is not known: tunnel and inner headers plus
/// `WireGuard`'s padding, 7.5 bytes on average, rounded up.
pub const DATAGRAM_OVERHEAD: u32 = TUNNEL_BYTES + INNER_HEADERS + 8;
/// RTP's fixed header, without CSRCs or extensions (`rtph264pay` adds none).
pub const RTP_HEADER: u32 = 12;
/// Bytes per second in one kbit/s.
const BYTES_PER_KBIT: u32 = 125;

/// Bytes on the wire for a datagram carrying `payload` bytes.
pub fn wire_bytes(payload: u32) -> u32 {
    let inner = payload
        .saturating_add(INNER_HEADERS)
        .checked_next_multiple_of(PADDING_BLOCK)
        .unwrap_or(u32::MAX);
    TUNNEL_BYTES.saturating_add(inner)
}

/// What video at `bitrate_kbit` and `fps` costs on the wire with `packets_per_frame` RTP packets.
pub fn video_bytes_per_s(bitrate_kbit: u32, fps: u16, packets_per_frame: u32) -> u32 {
    let packets = u32::from(fps).saturating_mul(packets_per_frame.max(1));
    bitrate_kbit
        .saturating_mul(BYTES_PER_KBIT)
        .saturating_add(packets.saturating_mul(DATAGRAM_OVERHEAD + RTP_HEADER))
}

/// The bitrate whose video costs `bytes_per_s` on the wire at `fps` and `packets_per_frame`; zero
/// when the headers alone cost more.
pub fn video_kbit_within(bytes_per_s: u32, fps: u16, packets_per_frame: u32) -> u32 {
    bytes_per_s.saturating_sub(video_bytes_per_s(0, fps, packets_per_frame)) / BYTES_PER_KBIT
}

/// What is reserved before video, from the configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Budget {
    /// Telemetry gets this percentage of what the autopilot offers, within the floor and the cap,
    /// all in `MAVLink` frame bytes per second.
    pub telemetry_percent: u32,
    pub telemetry_floor: u32,
    pub telemetry_cap: u32,
    /// The probes and the vehicle's challenges at their largest, on the wire.
    pub control: u32,
}

/// What the vehicle measured of its own traffic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Usage {
    /// Telemetry frame bytes a second the autopilot offered, sent or not; `None` until measured.
    pub telemetry_offered: Option<u32>,
    pub telemetry_datagrams_per_s: u32,
    /// Probes and every reply to a controller, on the wire.
    pub control: u32,
    pub video_packets_per_frame: u32,
}

/// Shares of a total, each in bytes per second on the wire. They never add up to more than it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Allocation {
    pub control: u32,
    pub telemetry: u32,
    pub retrieval: u32,
    pub video: u32,
}

impl Budget {
    /// Splits `total`: control first, the larger of the reserve and what was measured; then
    /// telemetry, what it wants with its datagrams' headers; then retrieval up to what it asks for
    /// (zero while idle), and video gets the rest. Each takes at most what is left.
    pub fn split(self, total: u32, usage: Usage, retrieval_wanted: u32) -> Allocation {
        let mut left = total;
        let mut take = |wanted: u32| {
            let granted = wanted.min(left);
            left -= granted;
            granted
        };
        let control = take(self.control.max(usage.control));
        let telemetry = take(
            self.telemetry_wanted(usage)
                .saturating_add(telemetry_headers(usage)),
        );
        let retrieval = take(retrieval_wanted);
        Allocation {
            control,
            telemetry,
            retrieval,
            video: left,
        }
    }

    /// The part of `wanted` retrieval may reserve out of `total`: all of it if the video left
    /// after it still passes `video_ok`, else none, so retrieval never pushes the video out of the
    /// rungs it must keep. A pure function of capacity, so granting and holding cannot feed back
    /// into each other.
    pub fn retrieval_reserve(
        self,
        total: u32,
        usage: Usage,
        wanted: u32,
        video_ok: impl Fn(u32) -> bool,
    ) -> u32 {
        if video_ok(self.split(total, usage, wanted).video) {
            wanted
        } else {
            0
        }
    }

    /// The telemetry cap in frame bytes: its share of the measured uplink `total`, or the
    /// configured cap while nothing is measured; never below the floor, so the vehicle's state
    /// keeps flowing however short the uplink.
    pub fn telemetry_cap(self, total: Option<u32>, usage: Usage) -> u32 {
        let Some(total) = total else {
            return self.telemetry_cap;
        };
        let share = self.split(total, usage, 0).telemetry;
        share
            .saturating_sub(telemetry_headers(usage))
            .max(self.telemetry_floor)
    }

    /// Unmeasured, the offered telemetry may be anything up to the cap.
    fn telemetry_wanted(self, usage: Usage) -> u32 {
        let cap = self.telemetry_cap.max(self.telemetry_floor);
        let Some(offered) = usage.telemetry_offered else {
            return cap;
        };
        let wanted = u64::from(offered) * u64::from(self.telemetry_percent) / 100;
        u32::try_from(wanted)
            .unwrap_or(u32::MAX)
            .clamp(self.telemetry_floor, cap)
    }
}

fn telemetry_headers(usage: Usage) -> u32 {
    usage
        .telemetry_datagrams_per_s
        .saturating_mul(DATAGRAM_OVERHEAD)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn allocation_never_exceeds_the_total() {
        let budget = Budget {
            telemetry_percent: 200,
            telemetry_floor: 200,
            telemetry_cap: 8000,
            control: 1000,
        };
        let usage = |offered, control| Usage {
            telemetry_offered: offered,
            telemetry_datagrams_per_s: 30,
            control,
            video_packets_per_frame: 7,
        };
        // (total, telemetry offered, control measured, retrieval wanted, expected shares: control,
        // telemetry, retrieval, video, and the telemetry cap in frame bytes). Telemetry wants
        // twice what is offered (the cap while that is unmeasured), at least the floor and at most
        // the cap, plus 3300 B/s of headers; its cap never falls below the floor.
        let cases = [
            (0, Some(900), 0, 0, (0, 0, 0, 0, 200)),
            (600, Some(900), 0, 5000, (600, 0, 0, 0, 200)),
            (4000, Some(900), 0, 0, (1000, 3000, 0, 0, 200)),
            (16_000, Some(900), 0, 0, (1000, 5100, 0, 9900, 1800)),
            (16_000, Some(900), 1500, 0, (1500, 5100, 0, 9400, 1800)),
            (16_000, Some(0), 0, 0, (1000, 3500, 0, 11_500, 200)),
            (16_000, None, 0, 0, (1000, 11_300, 0, 3700, 8000)),
            (16_000, Some(9000), 0, 0, (1000, 11_300, 0, 3700, 8000)),
            (16_000, Some(900), 0, 9000, (1000, 5100, 9000, 900, 1800)),
            (64_000, Some(900), 0, 2000, (1000, 5100, 2000, 55_900, 1800)),
            (
                u32::MAX,
                Some(u32::MAX),
                0,
                u32::MAX,
                (1000, 11_300, u32::MAX - 12_300, 0, 8000),
            ),
        ];
        for (total, offered, control_measured, wanted, expected) in cases {
            let usage = usage(offered, control_measured);
            let shares = budget.split(total, usage, wanted);
            let (control, telemetry, retrieval, video, cap) = expected;
            assert_eq!(
                budget.telemetry_cap(Some(total), usage),
                cap,
                "{total} B/s cap"
            );
            let expected = Allocation {
                control,
                telemetry,
                retrieval,
                video,
            };
            assert_eq!(
                shares, expected,
                "{total} B/s, {offered:?} offered, {wanted} wanted"
            );
            let sum = [shares.control, shares.telemetry, shares.retrieval]
                .into_iter()
                .fold(u64::from(shares.video), |sum, s| sum + u64::from(s));
            assert!(sum <= u64::from(total), "{total} B/s split into {sum}");
        }
        // (total, retrieval wanted, video that must be left, reserve granted): control 1000 and
        // telemetry 5100 come off first, as above.
        let reserves = [(16_000, 2000, 7000, 2000), (16_000, 2000, 8000, 0)];
        for (total, wanted, needed, expected) in reserves {
            let reserve = budget
                .retrieval_reserve(total, usage(Some(900), 0), wanted, |video| video >= needed);
            assert_eq!(
                reserve, expected,
                "{total} B/s, {wanted} wanted, {needed} needed"
            );
        }
    }
}
