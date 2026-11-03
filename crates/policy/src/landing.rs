//! Landing on a marker: whether a sighting may become a `LANDING_TARGET` for the autopilot, and the
//! angles it carries. The vehicle only reports where it saw the marker; `ArduPilot`'s precision landing
//! decides what to do with it, and only when the pilot has chosen Land.

use std::time::{Duration, Instant};

/// How far the camera's axis may be from straight down. Beyond it the angles would be measured from
/// a picture `ArduPilot` takes for a nadir one.
pub const MAX_OFF_NADIR_DEG: f32 = 5.0;
/// How far the picture's top may be turned from the airframe's nose. `ArduPilot` takes the angles
/// for a picture whose top is the front (`retrieve_los_meas` turns every target by `PLND_YAW_ALIGN`).
pub const MAX_TURN_DEG: f32 = 5.0;
/// A mount reading is judged only against a sighting at most this far from it, before or after: a
/// camera that moved since the frame was taken is not the one that saw the marker.
pub const MOUNT_MAX_GAP: Duration = Duration::from_millis(500);
/// A sighting older than this (from the frame leaving the tee to the send) is not the current frame
/// window: it is dropped, never sent late.
pub const SIGHTING_MAX_AGE: Duration = Duration::from_millis(500);

/// How the camera is mounted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Camera {
    /// Rigid to the airframe, pitched `tilt_deg` from the horizon (-90 is straight down).
    Fixed { tilt_deg: f32 },
    /// On a gimbal whose attitude the autopilot reports.
    Gimbal,
}

/// What the vehicle needs to turn a sighting into angles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Setup {
    pub camera: Camera,
    /// Horizontal field of view of the picture the marker was found in.
    pub hfov_deg: f32,
}

/// A gimbal reading: when it arrived, how far its axis was from straight down and how far the
/// picture's top was turned from the airframe's nose (positive clockwise seen from above).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mount {
    pub at: Instant,
    pub off_nadir_deg: f32,
    pub turn_deg: f32,
}

/// The marker's centre in a picture: fractions of its width and height from the top left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sighting {
    /// When the frame left the tee, a little after its capture.
    pub at: Instant,
    pub x: f32,
    pub y: f32,
    /// Width over height of the picture.
    pub aspect: f32,
}

/// What `LANDING_TARGET` carries for the sighting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aim {
    /// Radians, positive to the right of the picture's centre.
    pub angle_x: f32,
    /// Radians, positive below the picture's centre.
    pub angle_y: f32,
    /// Metres from the camera to the marker along the line of sight.
    pub distance_m: f32,
    /// How far the picture's top is turned from the nose, in degrees: 0 for a fixed camera.
    pub turn_deg: f32,
}

/// Why nothing is sent for a sighting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Withheld {
    /// The frame was captured too long ago.
    Stale,
    /// The camera's axis is this far from straight down.
    NotDown { off_nadir_deg: f32 },
    /// The picture's top is this far turned from the airframe's nose.
    Turned { turn_deg: f32 },
    /// A gimbal camera whose attitude the autopilot has not reported near the sighting.
    MountUnknown,
    /// No fresh height above home, or not above it.
    NoHeight,
    /// The sighting or the setup is not a finite, usable number.
    Invalid,
}

impl Withheld {
    /// What the pilot is told once, or nothing for a sighting that is merely late.
    pub fn notice(self) -> Option<String> {
        match self {
            Self::Stale | Self::Invalid => None,
            Self::NotDown { off_nadir_deg } => Some(format!(
                "Landing target off: camera {off_nadir_deg:.0} deg from straight down"
            )),
            Self::Turned { turn_deg } => Some(format!(
                "Landing target off: camera picture turned {turn_deg:.0} deg from the nose"
            )),
            Self::MountUnknown => Some("Landing target off: camera angle unknown".to_owned()),
            Self::NoHeight => Some("Landing target off: no height above home".to_owned()),
        }
    }
}

/// The unit quaternion `q` (w, x, y, z) as a rotation: `None` when it is not one.
fn rotation(q: [f32; 4]) -> Option<[f32; 4]> {
    let norm = q.iter().map(|c| c * c).sum::<f32>().sqrt();
    (norm.is_finite() && (norm - 1.0).abs() <= 0.1).then(|| q.map(|c| c / norm))
}

/// How far the camera's axis, given by the mount's quaternion `q` (w, x, y, z; the camera looks along
/// its x axis), is from straight down, in degrees. The servo backend reports its output angles in
/// the body frame (`AP_Mount_Servo::get_attitude_quaternion`), so down is the airframe's own down
/// axis, the quaternion's. `None` for a quaternion that is not a rotation.
pub fn off_nadir_deg(q: [f32; 4]) -> Option<f32> {
    let [qw, qx, qy, qz] = rotation(q)?;
    // The third component of the rotated x axis.
    let down = 2.0 * (qx * qz - qw * qy);
    Some(down.clamp(-1.0, 1.0).acos().to_degrees())
}

/// How far the picture's top is turned from the airframe's nose, in degrees, positive clockwise seen
/// from above: the camera's up axis (its negative z), rotated by `q`, on the horizontal plane. The
/// mount's yaw is relative to the airframe, and `ArduPilot` takes the angles for a picture whose top
/// is the nose, so a gimbal panned 90 degrees looking down turns every angle by 90. Only
/// meaningful for a camera near straight down. `None` for a quaternion that is not a rotation.
pub fn picture_turn_deg(q: [f32; 4]) -> Option<f32> {
    let [qw, qx, qy, qz] = rotation(q)?;
    let forward = -2.0 * (qx * qz + qw * qy);
    let right = -2.0 * (qy * qz - qw * qx);
    Some(right.atan2(forward).to_degrees())
}

/// Whether the camera points down and its picture is upright, from its mount.
fn mount_pose(
    camera: Camera,
    mount: Option<Mount>,
    sighting_at: Instant,
) -> Result<(f32, f32), Withheld> {
    match camera {
        Camera::Fixed { tilt_deg } => Ok(((tilt_deg + 90.0).abs(), 0.0)),
        Camera::Gimbal => {
            let near = mount
                .filter(|m| sighting_at.max(m.at) - sighting_at.min(m.at) <= MOUNT_MAX_GAP)
                .ok_or(Withheld::MountUnknown)?;
            Ok((near.off_nadir_deg, near.turn_deg))
        }
    }
}

/// The one decision: the `LANDING_TARGET` for `sighting`, or why none is sent. `height_mm` is the
/// autopilot's height above home, fresh at `now`; a gimbal's `mount` must be within `MOUNT_MAX_GAP`
/// of the sighting.
pub fn decide(
    setup: Setup,
    sighting: Sighting,
    mount: Option<Mount>,
    height_mm: Option<i32>,
    now: Instant,
) -> Result<Aim, Withheld> {
    if now.saturating_duration_since(sighting.at) > SIGHTING_MAX_AGE {
        return Err(Withheld::Stale);
    }
    let in_picture = |f: f32| (0.0..=1.0).contains(&f);
    let usable = in_picture(sighting.x)
        && in_picture(sighting.y)
        && sighting.aspect.is_finite()
        && sighting.aspect > 0.0
        && setup.hfov_deg.is_finite()
        && (1.0..180.0).contains(&setup.hfov_deg);
    if !usable {
        return Err(Withheld::Invalid);
    }
    let (tilt, turn_deg) = mount_pose(setup.camera, mount, sighting.at)?;
    if tilt.is_nan() || tilt > MAX_OFF_NADIR_DEG {
        return Err(Withheld::NotDown {
            off_nadir_deg: tilt,
        });
    }
    if turn_deg.is_nan() || turn_deg.abs() > MAX_TURN_DEG {
        return Err(Withheld::Turned { turn_deg });
    }
    let height_m = height_mm
        .filter(|mm| *mm > 0)
        .map(millimetres_to_metres)
        .ok_or(Withheld::NoHeight)?;
    Ok(aim(setup.hfov_deg, sighting, height_m, turn_deg))
}

#[expect(
    clippy::cast_precision_loss,
    reason = "a height of a few hundred metres is far inside f32's exact integers"
)]
fn millimetres_to_metres(mm: i32) -> f32 {
    mm as f32 / 1000.0
}

/// The angles of a point in a pinhole picture, and the line-of-sight distance to a flat ground
/// `height_m` below a camera looking straight down.
fn aim(hfov_deg: f32, sighting: Sighting, height_m: f32, turn_deg: f32) -> Aim {
    let half_width = (hfov_deg.to_radians() / 2.0).tan();
    let right = (2.0 * sighting.x - 1.0) * half_width;
    let below = (2.0 * sighting.y - 1.0) * half_width / sighting.aspect;
    Aim {
        angle_x: right.atan(),
        angle_y: below.atan(),
        distance_m: height_m * (1.0 + right * right + below * below).sqrt(),
        turn_deg,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SETUP: Setup = Setup {
        camera: Camera::Gimbal,
        hfov_deg: 90.0,
    };

    fn down_mount(at: Instant, off_nadir_deg: f32) -> Mount {
        Mount {
            at,
            off_nadir_deg,
            turn_deg: 0.0,
        }
    }

    #[test]
    fn what_is_sent_and_what_is_withheld() {
        let t0 = Instant::now();
        let ms = |n| t0 + Duration::from_millis(n);
        let seen = |at, x, y| Sighting {
            at,
            x,
            y,
            aspect: 2.0,
        };
        let fixed = |tilt_deg| Setup {
            camera: Camera::Fixed { tilt_deg },
            ..SETUP
        };
        let withheld = |reason| Err(reason);
        let cases = [
            (
                "a gimbal straight down",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(down_mount(t0, 0.0)),
                Some(10_000),
                ms(100),
                Ok(Aim {
                    angle_x: 0.0,
                    angle_y: 0.0,
                    distance_m: 10.0,
                    turn_deg: 0.0,
                }),
            ),
            (
                "a fixed camera at -90",
                fixed(-90.0),
                seen(t0, 0.5, 0.5),
                None,
                Some(10_000),
                ms(100),
                Ok(Aim {
                    angle_x: 0.0,
                    angle_y: 0.0,
                    distance_m: 10.0,
                    turn_deg: 0.0,
                }),
            ),
            (
                "a fixed camera pitched 20 degrees down",
                fixed(-20.0),
                seen(t0, 0.5, 0.5),
                None,
                Some(10_000),
                ms(100),
                withheld(Withheld::NotDown {
                    off_nadir_deg: 70.0,
                }),
            ),
            (
                "a gimbal 6 degrees off",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(down_mount(t0, 6.0)),
                Some(10_000),
                ms(100),
                withheld(Withheld::NotDown { off_nadir_deg: 6.0 }),
            ),
            (
                "a gimbal 5 degrees off",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(down_mount(t0, 5.0)),
                Some(10_000),
                ms(100),
                Ok(Aim {
                    angle_x: 0.0,
                    angle_y: 0.0,
                    distance_m: 10.0,
                    turn_deg: 0.0,
                }),
            ),
            (
                "a gimbal that never reported",
                SETUP,
                seen(t0, 0.5, 0.5),
                None,
                Some(10_000),
                ms(100),
                withheld(Withheld::MountUnknown),
            ),
            (
                "a gimbal reading from 900 ms before the frame",
                SETUP,
                seen(ms(900), 0.5, 0.5),
                Some(down_mount(t0, 0.0)),
                Some(10_000),
                ms(1000),
                withheld(Withheld::MountUnknown),
            ),
            (
                "a gimbal reading from 400 ms before the frame",
                SETUP,
                seen(ms(400), 0.5, 0.5),
                Some(down_mount(t0, 0.0)),
                Some(10_000),
                ms(450),
                Ok(Aim {
                    angle_x: 0.0,
                    angle_y: 0.0,
                    distance_m: 10.0,
                    turn_deg: 0.0,
                }),
            ),
            (
                "a gimbal reading from after the frame",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(down_mount(ms(300), 0.0)),
                Some(10_000),
                ms(400),
                Ok(Aim {
                    angle_x: 0.0,
                    angle_y: 0.0,
                    distance_m: 10.0,
                    turn_deg: 0.0,
                }),
            ),
            (
                "a gimbal panned 90 degrees, looking straight down",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(Mount {
                    turn_deg: 90.0,
                    ..down_mount(t0, 0.0)
                }),
                Some(10_000),
                ms(100),
                withheld(Withheld::Turned { turn_deg: 90.0 }),
            ),
            (
                "a gimbal panned 5 degrees",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(Mount {
                    turn_deg: -5.0,
                    ..down_mount(t0, 0.0)
                }),
                Some(10_000),
                ms(100),
                Ok(Aim {
                    angle_x: 0.0,
                    angle_y: 0.0,
                    distance_m: 10.0,
                    turn_deg: -5.0,
                }),
            ),
            (
                "a gimbal reading that is not a number",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(down_mount(t0, f32::NAN)),
                Some(10_000),
                ms(100),
                withheld(Withheld::NotDown {
                    off_nadir_deg: f32::NAN,
                }),
            ),
            (
                "a frame captured 600 ms ago",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(down_mount(ms(550), 0.0)),
                Some(10_000),
                ms(600),
                withheld(Withheld::Stale),
            ),
            (
                "no height",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(down_mount(t0, 0.0)),
                None,
                ms(100),
                withheld(Withheld::NoHeight),
            ),
            (
                "not above home",
                SETUP,
                seen(t0, 0.5, 0.5),
                Some(down_mount(t0, 0.0)),
                Some(0),
                ms(100),
                withheld(Withheld::NoHeight),
            ),
            (
                "a centre outside the picture",
                SETUP,
                seen(t0, 1.2, 0.5),
                Some(down_mount(t0, 0.0)),
                Some(10_000),
                ms(100),
                withheld(Withheld::Invalid),
            ),
        ];
        for (name, setup, sighting, mount, height, now, expected) in cases {
            let got = decide(setup, sighting, mount, height, now);
            match (got, expected) {
                (Ok(a), Ok(b)) => {
                    assert!(
                        (a.angle_x - b.angle_x).abs() < 1e-6
                            && (a.angle_y - b.angle_y).abs() < 1e-6
                            && (a.distance_m - b.distance_m).abs() < 1e-4,
                        "{name}: {a:?}"
                    );
                }
                (
                    Err(Withheld::NotDown { off_nadir_deg: a }),
                    Err(Withheld::NotDown { off_nadir_deg: b }),
                )
                | (Err(Withheld::Turned { turn_deg: a }), Err(Withheld::Turned { turn_deg: b })) => {
                    assert!(a.is_nan() && b.is_nan() || (a - b).abs() < 1e-3, "{name}");
                }
                (got, expected) => assert_eq!(got, expected, "{name}"),
            }
        }
    }

    /// With a 90 degree field of view the picture's edge is 45 degrees from its centre. Right of
    /// the centre is positive x, below it positive y (the autopilot reads them as body right and
    /// back), and the line of sight is longer than the height by the same factor.
    #[test]
    fn angles_follow_the_picture() {
        let t0 = Instant::now();
        let seen = |x, y| Sighting {
            at: t0,
            x,
            y,
            aspect: 2.0,
        };
        let right_edge = aim(90.0, seen(1.0, 0.5), 10.0, 0.0);
        assert!((right_edge.angle_x.to_degrees() - 45.0).abs() < 1e-3);
        assert!(right_edge.angle_y.abs() < 1e-6);
        assert!((right_edge.distance_m - 10.0 * 2.0_f32.sqrt()).abs() < 1e-3);
        // The picture is twice as wide as it is high, so its bottom edge is 26.6 degrees away.
        let bottom = aim(90.0, seen(0.5, 1.0), 10.0, 0.0);
        assert!((bottom.angle_y.to_degrees() - 26.565).abs() < 1e-2);
        assert!(bottom.angle_x.abs() < 1e-6);
    }

    /// The mount's attitude as the autopilot reports it: roll 0, pitch, then yaw about the
    /// airframe's down axis (a Z-Y-X rotation), all in degrees.
    fn mount_quaternion(pitch: f32, yaw: f32) -> [f32; 4] {
        let (sp, cp) = (pitch.to_radians() / 2.0).sin_cos();
        let (sy, cy) = (yaw.to_radians() / 2.0).sin_cos();
        [cy * cp, -sy * sp, cy * sp, sy * cp]
    }

    #[test]
    fn the_axis_is_measured_from_down() {
        for (pitch, expected) in [(-90.0, 0.0), (-85.0, 5.0), (-20.0, 70.0), (0.0, 90.0)] {
            let got = off_nadir_deg(mount_quaternion(pitch, 0.0)).unwrap();
            assert!((got - expected).abs() < 1e-2, "pitch {pitch}: {got}");
        }
        // Rolled 90 degrees, looking along the body's x axis: level, not down.
        let (s, c) = (45.0_f32.to_radians()).sin_cos();
        assert!((off_nadir_deg([c, s, 0.0, 0.0]).unwrap() - 90.0).abs() < 1e-2);
        assert_eq!(off_nadir_deg([f32::NAN, 0.0, 0.0, 0.0]), None);
        assert_eq!(off_nadir_deg([0.0; 4]), None);
    }

    /// Looking straight down, the picture follows the mount's pan: its top points where the pan
    /// points, whatever the axis says (a pan does not change `off_nadir_deg`).
    #[test]
    fn the_picture_turns_with_the_pan() {
        for yaw in [0.0, 5.0, -90.0, 90.0, 180.0] {
            let q = mount_quaternion(-90.0, yaw);
            let turn = picture_turn_deg(q).unwrap();
            let apart = (turn - yaw + 180.0).rem_euclid(360.0) - 180.0;
            assert!(apart.abs() < 1e-2, "pan {yaw}: {turn}");
            assert!(off_nadir_deg(q).unwrap() < 1e-2, "pan {yaw}");
        }
        assert_eq!(picture_turn_deg([0.0; 4]), None);
    }
}
