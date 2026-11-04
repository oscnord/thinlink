//! Where the camera's centre will point when a command sent now takes effect, and the horizon,
//! from the autopilot's newest attitude. Lens distortion is ignored.

use thinlink_video_core::freshness::State;

/// An attitude older than this predicts nothing.
pub const MAX_ATTITUDE_AGE_MS: u64 = 1000;

/// The autopilot's newest attitude and when the ground received it (`at_ms`, on the caller's clock).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rate_rad_s: f32,
    pub pitch_rate_rad_s: f32,
    pub at_ms: u64,
}

/// A point in the picture, in pixels from its top-left corner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aim {
    pub x: f32,
    pub y: f32,
}

/// The two points where the horizon leaves the picture.
pub type Horizon = ((f32, f32), (f32, f32));

/// What the ground knows when it draws a frame.
#[derive(Debug, Clone, Copy)]
pub struct Inputs {
    pub attitude: Attitude,
    /// Now, on the clock `attitude.at_ms` is on.
    pub now_ms: u64,
    /// How long a command takes to reach the vehicle.
    pub uplink_delay_ms: u64,
}

/// What to draw into a frame.
#[derive(Debug, Clone, Copy)]
pub struct Mark {
    pub aim: Aim,
    pub horizon: Option<Horizon>,
}

/// The mark for a frame shown in `state`, whose newest picture is `age_ms` old. Only a current
/// picture (`Live` or `LowRate`) with a known age gets one: old video is never dressed up. The
/// horizon needs `tilt_deg`, the angle of a camera fixed to the body; a gimbal that holds its own
/// angle gets none, since the body's attitude does not move its picture.
#[must_use]
pub fn mark(
    state: State,
    age_ms: Option<u64>,
    inputs: &Inputs,
    hfov_deg: f32,
    tilt_deg: Option<f32>,
    width: u32,
    height: u32,
) -> Option<Mark> {
    let current = matches!(state, State::Live | State::LowRate);
    let lead_ms = age_ms?.saturating_add(inputs.uplink_delay_ms);
    let aim = aim(
        &inputs.attitude,
        inputs.now_ms,
        lead_ms,
        hfov_deg,
        width,
        height,
    )?;
    current.then(|| Mark {
        aim,
        horizon: tilt_deg.and_then(|tilt| horizon(&inputs.attitude, hfov_deg, tilt, width, height)),
    })
}

/// Pixels per radian for a picture `width` pixels wide with a horizontal field of view of
/// `hfov_deg`, or `None` for a size or field of view that makes no picture.
/// A picture size or duration as a float; the precision lost above 2^24 is far past any picture.
#[expect(
    clippy::cast_precision_loss,
    reason = "sizes and leads are far below 2^24"
)]
fn px(v: u32) -> f32 {
    v as f32
}

fn pixels_per_rad(hfov_deg: f32, width: u32, height: u32) -> Option<f32> {
    let valid =
        hfov_deg.is_finite() && hfov_deg > 0.0 && hfov_deg < 180.0 && width > 0 && height > 0;
    valid.then(|| px(width) / hfov_deg.to_radians())
}

fn fresh(att: &Attitude, now_ms: u64) -> bool {
    now_ms.saturating_sub(att.at_ms) <= MAX_ATTITUDE_AGE_MS
}

/// The picture's centre moved by the yaw and pitch rates over `lead_ms` (the shown frame's age
/// plus the uplink delay), clamped to the picture. `None` when the attitude is stale or any input
/// is not a number.
#[must_use]
pub fn aim(
    att: &Attitude,
    now_ms: u64,
    lead_ms: u64,
    hfov_deg: f32,
    width: u32,
    height: u32,
) -> Option<Aim> {
    let ppr = pixels_per_rad(hfov_deg, width, height)?;
    let moved = px(u32::try_from(lead_ms).unwrap_or(u32::MAX)) / 1000.0 * ppr;
    let x = px(width) / 2.0 + att.yaw_rate_rad_s * moved;
    let y = px(height) / 2.0 - att.pitch_rate_rad_s * moved;
    (fresh(att, now_ms) && x.is_finite() && y.is_finite()).then(|| Aim {
        x: x.clamp(0.0, px(width)),
        y: y.clamp(0.0, px(height)),
    })
}

/// The horizon as the two points where it leaves the picture, for a camera mounted `tilt_deg`
/// below the body's nose: through the centre in level flight with a level camera, higher with the
/// camera tilted down, lower with the nose up, tilted by the roll. `None` when it is off the
/// picture or the attitude is not a number.
#[must_use]
pub fn horizon(
    att: &Attitude,
    hfov_deg: f32,
    tilt_deg: f32,
    width: u32,
    height: u32,
) -> Option<Horizon> {
    let ppr = pixels_per_rad(hfov_deg, width, height)?;
    let (sin, cos) = att.roll_rad.sin_cos();
    let offset = (att.pitch_rad - tilt_deg.to_radians()) * ppr;
    let origin = (
        px(width) / 2.0 + sin * offset,
        px(height) / 2.0 + cos * offset,
    );
    clip(origin, (cos, -sin), px(width), px(height))
}

/// The part of the line `origin + t * dir` inside the rectangle (Liang-Barsky).
fn clip(origin: (f32, f32), dir: (f32, f32), w: f32, h: f32) -> Option<Horizon> {
    let (mut t0, mut t1) = (f32::NEG_INFINITY, f32::INFINITY);
    for (o, d, max) in [(origin.0, dir.0, w), (origin.1, dir.1, h)] {
        let parallel = d.abs() < 1e-6;
        if parallel && !(0.0..=max).contains(&o) {
            return None;
        }
        if parallel {
            continue;
        }
        let (a, b) = ((0.0 - o) / d, (max - o) / d);
        t0 = t0.max(a.min(b));
        t1 = t1.min(a.max(b));
    }
    let valid = t0.is_finite() && t1.is_finite() && t0 < t1;
    valid.then_some((
        (origin.0 + t0 * dir.0, origin.1 + t0 * dir.1),
        (origin.0 + t1 * dir.0, origin.1 + t1 * dir.1),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn att(roll_deg: f32, pitch_deg: f32, yaw_rate_deg: f32, pitch_rate_deg: f32) -> Attitude {
        Attitude {
            roll_rad: roll_deg.to_radians(),
            pitch_rad: pitch_deg.to_radians(),
            yaw_rate_rad_s: yaw_rate_deg.to_radians(),
            pitch_rate_rad_s: pitch_rate_deg.to_radians(),
            at_ms: 10_000,
        }
    }

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 0.01
    }

    #[test]
    fn aim_leads_by_rate_times_lead() {
        // (attitude, now, lead, expected)
        let cases = [
            // 18 degrees of yaw at 640 px / 90 degrees
            (
                att(0.0, 0.0, 30.0, 0.0),
                10_100,
                600,
                Some((320.0 + 128.0, 240.0)),
            ),
            // a pitch rate up moves the mark up the picture
            (
                att(0.0, 0.0, 0.0, 30.0),
                10_100,
                600,
                Some((320.0, 240.0 - 128.0)),
            ),
            // no rate, no lead
            (att(0.0, 0.0, 0.0, 0.0), 10_100, 600, Some((320.0, 240.0))),
            // clamped at the edge
            (
                att(0.0, 0.0, 90.0, -90.0),
                10_100,
                2000,
                Some((640.0, 480.0)),
            ),
            // exactly one second old is still used, older is not
            (att(0.0, 0.0, 30.0, 0.0), 11_000, 600, Some((448.0, 240.0))),
            (att(0.0, 0.0, 30.0, 0.0), 11_001, 600, None),
            (att(0.0, 0.0, f32::NAN, 0.0), 10_100, 600, None),
        ];
        for (a, now, lead, want) in cases {
            let got = aim(&a, now, lead, 90.0, 640, 480).map(|p| (p.x, p.y));
            match (got, want) {
                (Some(g), Some(w)) => assert!(close(g.0, w.0) && close(g.1, w.1), "{g:?} != {w:?}"),
                (g, w) => assert_eq!(g, w),
            }
        }
        assert!(aim(&att(0.0, 0.0, 1.0, 0.0), 10_000, 100, 0.0, 640, 480).is_none());
    }

    #[test]
    fn only_a_current_picture_with_a_known_age_gets_a_mark() {
        let inputs = Inputs {
            attitude: att(0.0, 0.0, 30.0, 0.0),
            now_ms: 10_100,
            uplink_delay_ms: 300,
        };
        let mark = |state, age| mark(state, age, &inputs, 90.0, Some(0.0), 640, 480);
        // 300 ms of delay and 300 ms of age make 18 degrees of yaw
        let live = mark(State::Live, Some(300)).unwrap();
        assert!(close(live.aim.x, 448.0));
        assert!(mark(State::LowRate, Some(300)).is_some());
        let gimballed = super::mark(State::Live, Some(300), &inputs, 90.0, None, 640, 480);
        assert!(gimballed.unwrap().horizon.is_none());
        for state in [
            State::Stale,
            State::Frozen,
            State::Disconnected,
            State::AgeUnknown,
        ] {
            assert!(mark(state, Some(300)).is_none(), "{state:?}");
        }
        assert!(mark(State::Live, None).is_none());
    }

    #[test]
    fn horizon_follows_pitch_and_roll() {
        let ppr = 640.0 / 90.0;
        let ends = |a: &Attitude| horizon(a, 90.0, 0.0, 640, 480);
        let level = ends(&att(0.0, 0.0, 0.0, 0.0)).unwrap();
        assert!(close(level.0.1, 240.0) && close(level.1.1, 240.0));
        assert!(close(level.0.0.min(level.1.0), 0.0) && close(level.0.0.max(level.1.0), 640.0));

        let up = ends(&att(0.0, 10.0, 0.0, 0.0)).unwrap();
        assert!(close(up.0.1, 240.0 + 10.0 * ppr) && close(up.1.1, 240.0 + 10.0 * ppr));

        // rolled right, the right end is higher on the picture
        let (a, b) = ends(&att(20.0, 0.0, 0.0, 0.0)).unwrap();
        let (left, right) = if a.0 < b.0 { (a, b) } else { (b, a) };
        assert!(right.1 < 240.0 && left.1 > 240.0);

        // a camera tilted 10 degrees down sees the level horizon 10 degrees above the centre
        let tilted = horizon(&att(0.0, 0.0, 0.0, 0.0), 90.0, 10.0, 640, 480).unwrap();
        assert!(close(tilted.0.1, 240.0 - 10.0 * ppr) && close(tilted.1.1, 240.0 - 10.0 * ppr));

        // a steep pitch puts it off the picture
        assert!(ends(&att(0.0, 40.0, 0.0, 0.0)).is_none());
        assert!(ends(&att(0.0, f32::NAN, 0.0, 0.0)).is_none());
    }
}
