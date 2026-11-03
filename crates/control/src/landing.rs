//! The vehicle's one path from a sighting of the landing marker to the autopilot: a `LANDING_TARGET`
//! for `ArduPilot`'s precision landing, which uses it only while the pilot has the copter in Land. The
//! vehicle never moves the copter itself.
//!
//! The pinned source (`fb773567896014da007bca58358704d4a0d8516f`) reads the message in
//! `libraries/AC_PrecLand/AC_PrecLand_MAVLink.cpp`, `handle_msg` (`PLND_TYPE` 1 is that backend). A
//! frame of `MAV_FRAME_BODY_FRD` or `MAV_FRAME_LOCAL_FRD` is accepted; with `position_valid` 0 the unit
//! vector is `(-tan(angle_y), tan(angle_x), 1)`, body forward, right and down, so a picture whose top
//! is the vehicle's front gives `angle_x` to the right and `angle_y` below the centre. A positive
//! `distance` is the range to the target (`AC_PrecLand.cpp`, `construct_pos_meas_using_rangefinder`: without one the
//! autopilot needs a rangefinder); a measurement older than 1 s is dropped, and `AC_PrecLand` treats
//! the target as lost after `LANDING_TARGET_TIMEOUT_MS` (2000). Nothing is sent without a sighting
//! in the current frame window, so a lost marker ends the messages and the autopilot falls back to its
//! own lost-target behaviour (`PLND_STRICT`, `docs/real-vehicle.md`).

use std::collections::BTreeMap;
use std::mem::{Discriminant, discriminant};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::MessageData;
use mavlink::dialects::ardupilotmega::{
    GIMBAL_DEVICE_ATTITUDE_STATUS_DATA, LANDING_TARGET_DATA, LandingTargetType, MavFrame,
    MavMessage, MavSeverity,
};
use serde::Serialize;

use sat_teleop_policy::landing::{self, Aim, Camera, Mount, Setup, Sighting, Withheld};

use crate::frame::Frame;
use crate::identity::{self, ControlTarget};
use crate::sighting::status_frame;
use crate::telemetry::RateCap;
use crate::unix_ms;

/// How often a gimbal camera's attitude is asked for while none is fresh.
const MOUNT_ASK_EVERY: Duration = Duration::from_secs(5);
/// The attitude stream's interval, well inside `landing::MOUNT_MAX_GAP`.
pub const MOUNT_INTERVAL: Duration = Duration::from_millis(200);
/// The `gimbal_device_id` of the first mount: `AP_Mount_Backend::send_gimbal_device_attitude_status`
/// sends `_instance + 1`, and the first backend's instance is 0. Every mount's reading comes from
/// the autopilot's address, so the id is what tells the camera's from another mount's.
const FIRST_MOUNT: u8 = 1;

/// The landing target switch and everything between a sighting and the wire. Off (the default)
/// it holds no setup, takes no sighting and sends nothing.
#[derive(Debug)]
pub struct Landing {
    setup: Option<Setup>,
    /// Zero of the message's `time_usec`: the vehicle's own monotonic clock from here.
    epoch: Instant,
    /// The newest sighting, taken when it is judged: a sighting is sent once or never.
    newest: Mutex<Option<Sighting>>,
    mount: Mutex<Option<Mount>>,
    notice: Mutex<Notice>,
    sent: AtomicU64,
    last: Mutex<Option<(Instant, Instant, Aim)>>,
    withheld: Mutex<BTreeMap<&'static str, u64>>,
    notice_sequence: Mutex<u8>,
    last_mount_ask: Mutex<Option<Instant>>,
}

impl Default for Landing {
    fn default() -> Self {
        Self::new(None)
    }
}

/// What the pilot has been told about why nothing is sent, so a reason is said once until a target
/// goes out again.
#[derive(Debug, Default)]
struct Notice {
    told: Option<Discriminant<Withheld>>,
    pending: Option<String>,
}

#[derive(Debug, Serialize)]
pub struct LandingSnapshot {
    pub enabled: bool,
    /// `LANDING_TARGET` messages written to the autopilot.
    pub sent: u64,
    /// Sightings not sent, by reason.
    pub withheld: BTreeMap<&'static str, u64>,
    /// The target sent last, so a check can compare it with where the marker truly is.
    pub last: Option<LastTarget>,
}

/// The newest `LANDING_TARGET` sent, in degrees.
#[derive(Debug, Serialize)]
pub struct LastTarget {
    pub angle_x_deg: f32,
    pub angle_y_deg: f32,
    pub distance_m: f32,
    /// How far the picture's top was turned from the nose.
    pub turn_deg: f32,
    pub age_ms: u64,
    /// When the sighting's frame left the tee (UNIX ms), which identifies it across polls.
    pub sighted_unix_ms: u64,
    /// From the frame leaving the tee to the send.
    pub to_send_ms: u64,
}

impl Landing {
    /// `setup` is `None` when the switch is off.
    pub fn new(setup: Option<Setup>) -> Self {
        Self {
            setup,
            epoch: Instant::now(),
            newest: Mutex::default(),
            mount: Mutex::default(),
            notice: Mutex::default(),
            sent: AtomicU64::default(),
            last: Mutex::default(),
            withheld: Mutex::default(),
            notice_sequence: Mutex::default(),
            last_mount_ask: Mutex::default(),
        }
    }

    /// Offers a sighting, replacing one not yet judged. Ignored when the switch is off.
    pub fn see(&self, sighting: Sighting) {
        if self.setup.is_some() {
            *lock(&self.newest) = Some(sighting);
        }
    }

    /// The `LANDING_TARGET` for the newest sighting, if the interlocks hold and the policy allows it.
    /// The sighting is consumed either way. `height_mm` is the autopilot's fresh height above home.
    pub fn next_frame(
        &self,
        now: Instant,
        target: ControlTarget,
        height_mm: Option<i32>,
    ) -> Option<Frame> {
        let setup = self.setup?;
        let sighting = lock(&self.newest).take()?;
        if target != ControlTarget::Autopilot {
            self.count("interlocks");
            return None;
        }
        let mount = *lock(&self.mount);
        match landing::decide(setup, sighting, mount, height_mm, now) {
            Ok(aim) => {
                lock(&self.notice).told = None;
                let frame = target_frame(&aim, self.usec(sighting.at))?;
                self.sent.fetch_add(1, Ordering::Relaxed);
                *lock(&self.last) = Some((now, sighting.at, aim));
                Some(frame)
            }
            Err(reason) => {
                self.withhold(reason);
                None
            }
        }
    }

    /// Microseconds from `epoch` to `at`.
    fn usec(&self, at: Instant) -> u64 {
        u64::try_from(at.saturating_duration_since(self.epoch).as_micros()).unwrap_or(u64::MAX)
    }

    fn withhold(&self, reason: Withheld) {
        self.count(reason_key(reason));
        let mut notice = lock(&self.notice);
        let Some(text) = reason.notice() else {
            return;
        };
        if notice.told == Some(discriminant(&reason)) {
            return;
        }
        notice.told = Some(discriminant(&reason));
        notice.pending = Some(text);
    }

    fn count(&self, key: &'static str) {
        *lock(&self.withheld).entry(key).or_default() += 1;
    }

    /// The pilot's notice, as a `STATUSTEXT` from the autopilot's address, once the budget has room.
    pub fn notice_frame(
        &self,
        now: Instant,
        autopilot: Option<(u8, u8)>,
        cap: &mut RateCap,
    ) -> Option<Frame> {
        let (system, component) = autopilot?;
        let mut notice = lock(&self.notice);
        let text = notice.pending.as_deref()?;
        let mut sequence = lock(&self.notice_sequence);
        let header = MavHeader {
            system_id: system.into(),
            component_id: component,
            sequence: *sequence,
        };
        let frame = status_frame(header, MavSeverity::MAV_SEVERITY_WARNING, text)
            .filter(|f| cap.take_bulk(now, f.size()))?;
        notice.pending = None;
        *sequence = sequence.wrapping_add(1);
        Some(frame)
    }

    /// Keeps the camera's attitude `frame` carries, if it is the autopilot's
    /// `GIMBAL_DEVICE_ATTITUDE_STATUS` for the first mount (`libraries/AP_Mount/AP_Mount_Backend.cpp`,
    /// `send_gimbal_device_attitude_status`; the first mount's quaternion is the servo backend's
    /// body-frame output angles, so against the airframe).
    pub fn note_mount(&self, frame: &Frame, autopilot: Option<(u8, u8)>, now: Instant) {
        if self.setup.map(|s| s.camera) != Some(Camera::Gimbal)
            || frame.message_id() != GIMBAL_DEVICE_ATTITUDE_STATUS_DATA::ID
        {
            return;
        }
        let from = u8::try_from(frame.system_id())
            .ok()
            .zip(Some(frame.component_id()));
        if from.is_none() || from != autopilot {
            return;
        }
        let Ok(MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(m)) = frame.decode() else {
            return;
        };
        if m.gimbal_device_id != FIRST_MOUNT {
            return;
        }
        let reading = landing::off_nadir_deg(m.q)
            .zip(landing::picture_turn_deg(m.q))
            .map(|(off_nadir_deg, turn_deg)| Mount {
                at: now,
                off_nadir_deg,
                turn_deg,
            });
        *lock(&self.mount) = reading;
    }

    /// Whether the attitude of a gimbal camera should be asked for now: it is needed and none is
    /// fresh. Asking again waits `MOUNT_ASK_EVERY`.
    pub fn wants_mount(&self, now: Instant) -> bool {
        if self.setup.map(|s| s.camera) != Some(Camera::Gimbal) {
            return false;
        }
        let fresh = lock(&self.mount)
            .is_some_and(|m| now.saturating_duration_since(m.at) <= landing::MOUNT_MAX_GAP);
        let mut last = lock(&self.last_mount_ask);
        let due = last.is_none_or(|at| now.saturating_duration_since(at) >= MOUNT_ASK_EVERY);
        if fresh || !due {
            return false;
        }
        *last = Some(now);
        true
    }

    /// The autopilot is gone: its last word on the camera and any sighting go with it.
    pub fn forget(&self) {
        *lock(&self.mount) = None;
        *lock(&self.newest) = None;
    }

    pub fn snapshot(&self) -> LandingSnapshot {
        let last = lock(&self.last).map(|(at, sighted, aim)| LastTarget {
            angle_x_deg: aim.angle_x.to_degrees(),
            angle_y_deg: aim.angle_y.to_degrees(),
            distance_m: aim.distance_m,
            turn_deg: aim.turn_deg,
            age_ms: u64::try_from(at.elapsed().as_millis()).unwrap_or(u64::MAX),
            to_send_ms: u64::try_from(at.saturating_duration_since(sighted).as_millis())
                .unwrap_or(u64::MAX),
            sighted_unix_ms: unix_ms()
                .saturating_sub(u64::try_from(sighted.elapsed().as_millis()).unwrap_or(u64::MAX)),
        });
        LandingSnapshot {
            enabled: self.setup.is_some(),
            sent: self.sent.load(Ordering::Relaxed),
            withheld: lock(&self.withheld).clone(),
            last,
        }
    }
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

fn reason_key(reason: Withheld) -> &'static str {
    match reason {
        Withheld::Stale => "stale",
        Withheld::NotDown { .. } => "not_down",
        Withheld::Turned { .. } => "turned",
        Withheld::MountUnknown => "mount_unknown",
        Withheld::NoHeight => "no_height",
        Withheld::Invalid => "invalid",
    }
}

/// The message for `aim`, from the vehicle's own identity. The angles are of the target from the
/// centre of a picture whose top is the vehicle's front, so `ArduPilot` takes the vector from them
/// (`position_valid` 0); `x`, `y` and the quaternion are unused then. The frame is the body's for a
/// fixed camera and for a mount, whose reading is the camera's attitude against the airframe: the
/// servo backend reports its body-frame output angles (`AP_Mount_Servo::get_attitude_quaternion`) and
/// does not stabilise in neutral, so the camera tilts with the airframe, which `AC_PrecLand` undoes by
/// the whole attitude. `MAV_FRAME_LOCAL_FRD` is turned by the heading only and made that tilt look
/// like the target moving.
fn target_frame(aim: &Aim, time_usec: u64) -> Option<Frame> {
    let message = MavMessage::LANDING_TARGET(LANDING_TARGET_DATA {
        time_usec,
        angle_x: aim.angle_x,
        angle_y: aim.angle_y,
        distance: aim.distance_m,
        frame: MavFrame::MAV_FRAME_BODY_FRD,
        q: [1.0, 0.0, 0.0, 0.0],
        mavtype: LandingTargetType::LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        ..LANDING_TARGET_DATA::default()
    });
    identity::own_frame(&message)
}

/// The request for the camera's attitude.
pub fn mount_request(system: u8, component: u8) -> Option<Frame> {
    identity::stream_request(
        system,
        component,
        GIMBAL_DEVICE_ATTITUDE_STATUS_DATA::ID,
        MOUNT_INTERVAL,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::encoded;
    use crate::telemetry::TelemetryLimits;

    const AUTOPILOT: (u8, u8) = (1, 1);

    fn setup(camera: Camera) -> Setup {
        Setup {
            camera,
            hfov_deg: 90.0,
        }
    }

    fn seen(at: Instant) -> Sighting {
        Sighting {
            at,
            x: 1.0,
            y: 0.5,
            aspect: 2.0,
        }
    }

    /// The mount `device` reports pitch, then pan about the airframe's down axis.
    fn mount_attitude(device: u8, pitch_deg: f32, yaw_deg: f32) -> Frame {
        let (sp, cp) = (pitch_deg.to_radians() / 2.0).sin_cos();
        let (sy, cy) = (yaw_deg.to_radians() / 2.0).sin_cos();
        let message =
            MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(GIMBAL_DEVICE_ATTITUDE_STATUS_DATA {
                q: [cy * cp, -sy * sp, cy * sp, sy * cp],
                gimbal_device_id: device,
                ..GIMBAL_DEVICE_ATTITUDE_STATUS_DATA::default()
            });
        // The frame names the autopilot's system and component, as `encoded` does for ids 1/1.
        encoded(&message)
    }

    fn attitude(pitch_deg: f32) -> Frame {
        mount_attitude(FIRST_MOUNT, pitch_deg, 0.0)
    }

    fn decoded(frame: &Frame) -> LANDING_TARGET_DATA {
        let Ok(MavMessage::LANDING_TARGET(m)) = frame.decode() else {
            panic!("not a LANDING_TARGET");
        };
        m
    }

    /// What reaches the autopilot, and what only the pilot hears, in one flight's worth of events.
    #[test]
    fn a_sighting_is_sent_once_and_only_when_the_camera_points_down() {
        let t0 = Instant::now();
        let landing = Landing::new(Some(setup(Camera::Gimbal)));
        let go = ControlTarget::Autopilot;
        let height = Some(10_000);

        // No attitude yet: nothing, and the pilot is told.
        landing.see(seen(t0));
        assert!(landing.next_frame(t0, go, height).is_none());
        let mut cap = RateCap::new(TelemetryLimits {
            bytes_per_s: 10_000,
            offered_percent: 200,
            essential_bytes_per_s: 200,
        });
        let said = landing.notice_frame(t0, Some(AUTOPILOT), &mut cap).unwrap();
        assert!(matches!(said.decode(), Ok(MavMessage::STATUSTEXT(_))));

        // Looking 20 degrees down: nothing, and it is a different reason, so it is told again.
        landing.note_mount(&attitude(-20.0), Some(AUTOPILOT), t0);
        landing.see(seen(t0));
        assert!(landing.next_frame(t0, go, height).is_none());
        assert!(
            landing
                .notice_frame(t0, Some(AUTOPILOT), &mut cap)
                .is_some()
        );
        // The same reason again is not told twice.
        landing.see(seen(t0));
        assert!(landing.next_frame(t0, go, height).is_none());
        assert!(
            landing
                .notice_frame(t0, Some(AUTOPILOT), &mut cap)
                .is_none()
        );

        // Straight down but panned 90 degrees: the picture is turned, a reason of its own.
        landing.note_mount(
            &mount_attitude(FIRST_MOUNT, -90.0, 90.0),
            Some(AUTOPILOT),
            t0,
        );
        landing.see(seen(t0));
        assert!(landing.next_frame(t0, go, height).is_none());
        assert!(
            landing
                .notice_frame(t0, Some(AUTOPILOT), &mut cap)
                .is_some()
        );

        // Straight down: sent once, with the angles of a marker at the right edge.
        landing.note_mount(&attitude(-90.0), Some(AUTOPILOT), t0);
        landing.see(seen(t0));
        let frame = landing.next_frame(t0, go, height).unwrap();
        let m = decoded(&frame);
        assert!((m.angle_x.to_degrees() - 45.0).abs() < 1e-3);
        assert_eq!(m.frame, MavFrame::MAV_FRAME_BODY_FRD);
        assert!(!m.position_valid.as_bool());
        assert!(landing.next_frame(t0, go, height).is_none(), "no repeat");

        // Control goes to the logging sink: the sighting is dropped, not held for later.
        landing.see(seen(t0));
        assert!(
            landing
                .next_frame(t0, ControlTarget::LoggingSink, height)
                .is_none()
        );
        assert!(landing.next_frame(t0, go, height).is_none());

        let counts = landing.snapshot();
        assert_eq!(counts.sent, 1);
        let last = counts.last.unwrap();
        assert!((last.angle_x_deg - 45.0).abs() < 1e-3 && last.angle_y_deg.abs() < 1e-3);
        assert_eq!(counts.withheld.get("not_down"), Some(&2));
        assert_eq!(counts.withheld.get("turned"), Some(&1));
    }

    #[test]
    fn with_the_switch_off_nothing_is_taken_or_sent() {
        let t0 = Instant::now();
        let landing = Landing::default();
        landing.see(seen(t0));
        landing.note_mount(&attitude(-90.0), Some(AUTOPILOT), t0);
        assert!(
            landing
                .next_frame(t0, ControlTarget::Autopilot, Some(10_000))
                .is_none()
        );
        assert!(!landing.wants_mount(t0));
        assert!(!landing.snapshot().enabled);
    }

    #[test]
    fn only_the_first_mount_of_the_autopilot_is_the_camera() {
        let t0 = Instant::now();
        let landing = Landing::new(Some(setup(Camera::Gimbal)));
        let go = ControlTarget::Autopilot;
        // Another address, and the second mount at the autopilot's address.
        landing.note_mount(&attitude(-90.0), Some((1, 99)), t0);
        landing.note_mount(&mount_attitude(2, -90.0, 0.0), Some(AUTOPILOT), t0);
        landing.see(seen(t0));
        assert!(landing.next_frame(t0, go, Some(10_000)).is_none());
        // The second mount looking elsewhere does not unseat the first's reading.
        landing.note_mount(&attitude(-90.0), Some(AUTOPILOT), t0);
        landing.note_mount(&mount_attitude(2, -20.0, 0.0), Some(AUTOPILOT), t0);
        landing.see(seen(t0));
        assert!(landing.next_frame(t0, go, Some(10_000)).is_some());
    }
}
