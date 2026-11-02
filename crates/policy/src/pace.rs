//! Pacing the live video to the uplink: which packets join the backlog the vehicle keeps in front
//! of its send, so that an uplink slower than the video is filled by the vehicle's own queue, not
//! the network's, where the control envelopes would wait behind it.

/// The video may go out at most at this share of the bitrate it is allowed: the slack a keyframe
/// and x264's one-frame buffer need.
const ALLOWED_PERCENT: u64 = 120;
/// And, where the uplink is believed, at this share of it if that is less: a queue builds for
/// about the round trip before the controller sees it, so what it peaks at is the round trip
/// times the excess.
const UPLINK_PERCENT: u64 = 110;

/// The pace, in bytes per second on the wire, for video allowed `allowed` and an uplink believed
/// to carry `uplink`, when one is: 110 % of the uplink, never below the allowance (dropping what
/// the encoder was told to send reads as loss and cuts again) and never above 120 % of it.
pub fn rate(allowed: u32, uplink: Option<u32>) -> u32 {
    let allowed = u64::from(allowed);
    let slack = allowed * ALLOWED_PERCENT / 100;
    let held = uplink.map_or(slack, |u| {
        (u64::from(u) * UPLINK_PERCENT / 100).clamp(allowed, slack)
    });
    u32::try_from(held).unwrap_or(u32::MAX)
}

/// What happens to a packet offered to the pacer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Joins the backlog.
    Queue,
    /// Dropped: the backlog would hold it longer than the age limit, or the picture it belongs to
    /// is already broken.
    Drop,
    /// Dropped, and the first drop since the picture was last whole: ask for a keyframe.
    DropAndRecover,
}

/// The pacer's admission, decided once per frame, on the packet that starts it: a frame joins the
/// backlog when the packets ahead of it leave within `max_age_ms` at the paced rate, and then so do
/// all its packets, however long it takes to send itself (a keyframe at a slow rung is seconds of
/// packets). After a drop every delta frame is dropped too (a picture missing a frame only mends
/// at a keyframe) until a keyframe fits.
#[derive(Debug, Default)]
pub struct Admission {
    broken: bool,
    admitted: bool,
}

impl Admission {
    pub fn offer(
        &mut self,
        starts_frame: bool,
        keyframe: bool,
        backlog_bytes: u32,
        bytes_per_s: u32,
        max_age_ms: u64,
    ) -> Verdict {
        if !starts_frame {
            return if self.admitted {
                Verdict::Queue
            } else {
                Verdict::Drop
            };
        }
        let wait_ms = u64::from(backlog_bytes) * 1000 / u64::from(bytes_per_s.max(1));
        self.admitted = wait_ms <= max_age_ms && (keyframe || !self.broken);
        if self.admitted {
            self.broken = false;
            return Verdict::Queue;
        }
        let first = !self.broken;
        self.broken = true;
        if first {
            Verdict::DropAndRecover
        } else {
            Verdict::Drop
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use Verdict::{Drop, DropAndRecover, Queue};

    #[test]
    fn the_pace_follows_the_uplink_believed_between_the_allowance_and_its_slack() {
        // (allowed, uplink believed, pace) in bytes per second.
        let cases = [
            // No uplink known: the slack above the allowance only.
            (50_000, None, 60_000),
            // The allowance is far under the uplink: the slack.
            (30_000, Some(64_000), 36_000),
            // The allowance a little over the uplink: held to it, not to 120 % of it.
            (75_000, Some(64_000), 75_000),
            (60_000, Some(64_000), 70_400),
            // An uplink believed below the allowance never starves it.
            (40_000, Some(12_000), 40_000),
            // The test raised with the belief: a recovered link is not held at the old uplink.
            (75_000, Some(75_000), 82_500),
        ];
        for (allowed, uplink, expected) in cases {
            assert_eq!(rate(allowed, uplink), expected, "{allowed} {uplink:?}");
        }
    }

    #[test]
    fn a_frame_behind_a_backlog_older_than_the_limit_is_dropped_whole_until_a_keyframe_fits() {
        // 10 kB/s, 200 ms: a backlog of up to 2000 bytes ahead of a frame is fine, however big the
        // frame. (starts a frame, keyframe, backlog, verdict)
        let offers = [
            (true, false, 0, Queue),
            // The rest of the frame follows its first packet, whatever the backlog has become.
            (false, false, 9000, Queue),
            (true, false, 2000, Queue),
            (true, false, 2100, DropAndRecover),
            (false, false, 0, Drop),
            // A delta frame is dropped even when it would fit; a keyframe that does not fit is
            // dropped; one that fits mends the picture.
            (true, false, 0, Drop),
            (true, true, 3000, Drop),
            (true, true, 0, Queue),
            (false, true, 5000, Queue),
            (true, false, 0, Queue),
        ];
        let mut admission = Admission::default();
        for (starts_frame, keyframe, backlog, expected) in offers {
            assert_eq!(
                admission.offer(starts_frame, keyframe, backlog, 10_000, 200),
                expected,
                "starts {starts_frame}, keyframe {keyframe}, backlog {backlog}"
            );
        }
    }
}
