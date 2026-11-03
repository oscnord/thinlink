# Flying a real vehicle

sat-teleop is meant for people who hold a drone pilot licence and want to fly a real vehicle over a satellite or cellular link. Real mode is how the software lets control reach a real flight controller. It is never the default: the testbed and every shipped example run in SITL mode, where control reaches only a simulated autopilot. Real mode has never flown a vehicle. Whether, where and how you may fly, and what the link is good enough for, are your decisions and your regulator's.

## What real mode is

`mode = "real"` in the `[mavlink]` table of `vehicle.toml`. The vehicle then sends control (the ground station's heartbeat, sticks, arm, mode and takeoff commands) to the autopilot only while every interlock below holds. When one does not, control goes to a logging sink instead: the commands are refused to the pilot (QGroundControl shows them denied), nothing is written to the autopilot, and the vehicle says which interlock failed in its log and in its metrics (`mavlink.control.interlocks`, each with a reason). The vehicle loads the keys once, when it starts. Changing, replacing or removing a key file takes effect at the next restart, and a vehicle that started with a readable key keeps verifying with it. Once they hold, the ground station's heartbeat reaches the autopilot as it does for SITL, so `FS_GCS_ENABLE` keeps its meaning.

## The interlocks

| Interlock | Holds when | Otherwise |
|---|---|---|
| `identity` | The autopilot sent a heartbeat of a vehicle type the gate knows (copter or rover) and it is not SITL: it ignored three probes for the simulator-only parameter `SIM_WIND_SPD` (`docs/drone.md#control-protocol`). In SITL mode the rule is the reverse. | Control is refused. A real autopilot is never taken for SITL, and SITL is refused in real mode. |
| `controller_keys` | Every controller in the `controllers` list had a public key file that loaded when the vehicle started. | That controller cannot be verified, and control is refused for everyone until a restart finds the key. |
| `failsafe_params` | Every parameter of `require` was read back from the autopilot's own system and component within 30 s, in the configured type, and equals the configured value. | A missing, stale or different parameter, or an empty `require`, refuses control. |
| `clock_bound` | The vehicle had a fresh bound on its clock's error from chrony before it first sent control. | No fresh bound at the start, no control. `clock = "shared"` and `"fixed"` are refused in real mode. This interlock gates the start only (below). |

The clock bound is the one interlock that gates the start only. Once the vehicle has had a fresh bound, a stale one (chrony lost its sources, restarted, or `chronyc` failed) turns the ground's label to `AGE UNKNOWN` and shows in the vehicle's metrics (`clock.bound_ms` is null) and its log, and never withdraws control or the ground station's heartbeat: the bound only labels the video, and a display input must not trip a failsafe. The keys cannot go stale, and the failsafe parameters keep gating in flight (next section).

Independent of the interlocks, every control envelope must be signed (below), and the autopilot's own failsafes are what stops the vehicle when the link goes; the interlocks exist so that you know they are what you configured.

## Setup

1. **Keys.** `just deploy-keys <relay endpoint>` writes, besides the WireGuard keys, `deploy/secrets/ctl-ground.key` and `.pub` (the ground adapter's Ed25519 signing key, mode 600, never committed) and three bearer tokens (`vehicle.token`, `ground.token`, `job.token`, mode 600). `just vehicle-release` puts the public key, `vehicle.token` and `job.token` in `dist/vehicle`, `just ground-release` the private key, `ground.token` and `job.token` in `dist/ground`, `just relay-release` only `job.token` in `dist/relay`. A private key or token file that its group or others can access is refused when the program starts, with the file's name and `chmod 600`. Rotate by deleting `deploy/secrets/ctl-ground.*` and releasing again.
2. **Vehicle config.** Give the vehicle a name, `vehicle_id` in `/etc/sat-teleop/vehicle.toml`, and the same string as `vehicle_id` in the ground's `ground.toml` (every signature is made for it; the ground's messages are refused until they match). In `/etc/sat-teleop/vehicle.toml` set `mode = "real"`, `clock = "chrony"`, and `require` (below). Leave `debug_hooks` and `sitl_in_real_mode` out: they exist only in the testbed's builds (the `testbed` cargo feature), and a release build refuses to start on a config that names either. The release recipes (`vehicle-release`, `image-for`, `relay-release`, `ground-release`) build without the feature.
3. **Chrony.** `provision.sh` installs chrony on the Pi. The vehicle derives its bound from `chronyc -c tracking` (`docs/hardware.md#clock-sync`). Check with `chronyc tracking`. The ground needs a bound too (`clock` in `ground.toml`, see "Clocks" below).
4. **Parameters.** `require` lists parameter names, the type the autopilot reports each in (`type`: `uint8`, `int8`, `uint16`, `int16`, `uint32`, `int32` or `real32`, as QGroundControl's parameter editor shows it) and the values the autopilot must report. A reading in another type, or from another component, is not the parameter you named. The vehicle only compares; it never writes a parameter. Set them in the autopilot first (Mission Planner, QGroundControl, or a `.parm` file) and put the same values in `require`.
5. **Tokens.** Each service has a token of its own (compared in constant time): `vehicle.token` guards the vehicle's metrics, `ground.token` the ground's lease endpoints and `job.token` the relay's metrics and job API, which the vehicle and the ground present to the relay. A compromised relay, which holds only the job API's token, cannot take the lease through the ground's endpoints or read the vehicle's. Without a token an endpoint listens on 127.0.0.1 only, whatever address it was given, and the relay does not start its job API; the shipped deployments bind the metrics to loopback and pass the token.

## Clocks

The picture is `LIVE` only when the newest frame's age plus a bound on how far the two machines' clocks can differ is small, so without a fresh bound on both sides the label reads `AGE UNKNOWN`, never `LIVE`. How the bound is made is in `docs/video.md#frame-age-and-clocks`. `clock` in `ground.toml` says where the ground's comes from:

- **A Linux ground with chrony.** The adapter runs in a container, and the host's chronyd is not on the container's loopback. Install `chrony-ground.conf` (shipped with the ground release) on the host and set `clock = { chrony_at = "host.containers.internal" }`: the container's `chronyc` reads `tracking` over chronyd's command port. That port answers read-only commands to the addresses `cmdallow` names, with no privilege, and refuses everything that would change the clock. Mounting chronyd's socket does not work: it is read-write for the whole directory (`chronyc` makes its own socket beside it), and it is chronyd's privileged channel, so a compromised container could step the host's clock. This was tried in two unprivileged containers with chrony 4.6.1 (one running `chronyd -x`): `chronyc -h <host> tracking` answered as root and as an unprivileged user, and `makestep` was refused. The host's chronyd with the shipped compose file has not been tried. `chrony-ground.conf` assumes Podman's default bridge (gateway 10.88.0.1, subnet 10.88.0.0/16). With another network (a compose project network, rootless networking, Docker) change both addresses to what `getent hosts host.containers.internal` shows in the container, or chrony is not reachable. On a rootful host chronyd may fail to bind 10.88.0.1 at boot because the bridge does not exist yet: restart chrony (`sudo systemctl restart chrony`) after the first container has started. Until then, and whenever the container cannot read chrony, the pilot sees `AGE UNKNOWN`, never `LIVE`.
- **A Mac.** The adapter runs in the Podman VM, so the clock it sees is the VM's, which has no chrony and stops while the laptop sleeps. Measure your Mac's clock error and set `clock = { fixed = <ms> }` with margin. A fixed bound holds until the VM's wall clock steps against its monotonic clock by more than that number (checked every 2 s), and then the label reads `AGE UNKNOWN` until the container restarts. After the laptop wakes the pilot sees this: the VM's clock is behind by the length of the sleep, so frames carry an age that is not plausible and the label reads `AGE UNKNOWN`; when the VM corrects its clock the step drops the fixed bound, and the label stays `AGE UNKNOWN`. Restart the ground container (`podman compose restart ground`) after every wake, once `podman machine ssh date` agrees with the Mac's clock. A Mac ground is best-effort for the clock bound: the step rule catches a clock that jumps, not a VM clock that stays behind after sleep without stepping, which a fixed bound cannot see. This has not been tried across a sleep on a Mac.
- **The Pi** uses chrony (`provision.sh`, `clock = "chrony"` in `vehicle.toml`).

## Required autopilot parameters

The list below is the set to start from for an ArduCopter 4.x; the parameter names and meanings are in ArduPilot's parameter reference (`https://ardupilot.org/copter/docs/parameters.html`), and the behaviour of the GCS failsafe in its failsafe page (`https://ardupilot.org/copter/docs/gcs-failsafe.html`). The values are examples chosen for a small multicopter on a link that can go quiet for seconds. The types are from ArduPilot's parameter definitions and have not been checked against a flight controller: confirm each in the parameter editor. Choose your own values for your airframe and operation, test each failsafe on the bench with the props off, then require exactly those:

```toml
require = [
    { name = "FS_GCS_ENABLE", type = "int8", value = 5 },    # GCS failsafe: Land when the ground station's heartbeat stops
    { name = "FS_GCS_TIMEOUT", type = "real32", value = 5 },   # seconds without it (default 5; the simulated flight uses 2)
    { name = "FS_THR_ENABLE", type = "int8", value = 7 },    # radio failsafe: Brake or Land
    { name = "FS_THR_VALUE", type = "int16", value = 975 },   # the throttle PWM below which the radio failsafe fires
    { name = "RC_OVERRIDE_TIME", type = "real32", value = 0.5 }, # seconds a MANUAL_CONTROL override lasts without a new one
    { name = "FENCE_ENABLE", type = "int8", value = 1 },     # geofence on
    { name = "FENCE_TYPE", type = "int8", value = 7 },       # maximum altitude, circle and polygon
    { name = "FENCE_ACTION", type = "int8", value = 1 },     # RTL or Land on a breach
    { name = "FENCE_ALT_MAX", type = "real32", value = 100 },  # metres
    { name = "FENCE_RADIUS", type = "real32", value = 300 },   # metres
]
```

Why these: `FS_GCS_ENABLE`/`FS_GCS_TIMEOUT` are what lands the vehicle if the lease, the adapter or the link is lost. `RC_OVERRIDE_TIME` makes the sticks stop mattering shortly after the setpoints do (the vehicle refreshes a fresh setpoint every 200 ms and lets it lapse after 0.5 s). The radio failsafe covers the throttle the sticks stream drives. The geofence is the autopilot's own bound on where the vehicle can go whatever the ground sends. The testbed's rover set and its derivation are in `testbed/sat-teleop.parm` (`docs/drone.md#sitl`); the copter's simulated-flight values and measurements are in `docs/drone.md#copter-failsafes` and `perf/budgets.toml`.

## Missions flown out of reach

By default nothing here changes the core promise: when commands stop, the drone stops. `allow_missions = true` in the `[mavlink]` table of `vehicle.toml` makes one exception, for a copter in Auto. It is off by default and without it the vehicle refuses everything below.

With it on, the lease holder can use QGroundControl's Plan view: the vehicle admits `MISSION_COUNT`, `MISSION_ITEM_INT` and `MISSION_CLEAR_ALL` (the upload), `MAV_CMD_MISSION_START` (how QGroundControl 5.1.5 starts a mission from the ground) and a change into Auto (copter mode 3). They travel only in signed discrete envelopes from the lease holder, like arming, and are refused otherwise: with the switch off, for a rover, and for any mission type but the waypoint mission, so the fence and the rally points cannot be rewritten over the link. The autopilot's requests, acknowledgement and progress come back as telemetry. The vehicle also looks at the items (below).

A copter that loses the link inside its geofence then finishes the mission by the autopilot's own failsafe. A lost link stops the sticks as well as the heartbeat, so both failsafes must let a mission in Auto carry on: `FS_OPTIONS` bit 0 (`RC_CONTINUE_IF_AUTO`, the radio failsafe) and bit 1 (`GCS_CONTINUE_IF_AUTO`, the GCS failsafe), and bit 4 (`GCS_CONTINUE_IF_PILOT_CONTROL`) must stay clear, or the GCS failsafe would be skipped in the stick modes (`ArduCopter/Copter.h`, `FailsafeOption`, and `ArduCopter/events.cpp`, ArduPilot `fb773567`, the commit the simulation is built from). The geofence ends the mission with its action on a breach. The gate forbids fence uploads, so the fence must be one that needs none: the maximum altitude and the circle. Real mode with the switch on therefore refuses to start unless `require` holds, besides the entries above:

```toml
allow_missions = true
require = [
    # ... the entries above, and:
    { name = "FS_OPTIONS", type = "int32", value = 3, compare = "bits_set" },    # continue in Auto on the radio and the GCS failsafe
    { name = "FS_OPTIONS", type = "int32", value = 16, compare = "bits_clear" }, # the other bits are yours
    { name = "FENCE_ENABLE", type = "int8", value = 1 },
    { name = "FENCE_TYPE", type = "int8", value = 3, compare = "bits_set" },     # maximum altitude and circle (7 adds the polygon)
    { name = "FENCE_ALT_MAX", type = "real32", value = 100, compare = "at_most" }, # at most 120 m, the height the gate allows
    { name = "FENCE_ACTION", type = "int8", value = 1 },                         # RTL or Land (4 would brake and hold after a breach)
    { name = "BATT_FS_LOW_ACT", type = "int8", value = 1, compare = "at_least" }, # a battery failsafe, not "none" (2 is RTL)
]
```

A `compare` of `bits_set` holds when every bit of `value` is set in what the autopilot reports and the others are free, `bits_clear` when every bit of `value` is clear in it, `at_most` when the report is no higher than `value`; the default is equality. `FS_OPTIONS` is an `AP_Int32`, so `int32`. If the autopilot reports another value, real mode refuses control as for any other entry. `at_least` holds when the report is no lower than `value`. `FENCE_RADIUS` stays your choice: size the circle to the site. The battery failsafe (`BATT_FS_LOW_ACT` of 1 or more, with `BATT_LOW_VOLT` or `BATT_LOW_MAH` set for the pack) is what ends a mission that runs long: a mission out of reach has no other bound on its duration than the battery and the 300 s hold caps below. Guided, Loiter, Brake and the stick modes still land or brake when control stops, whatever the switch says.

A mission upload is held to what the fence cannot contain: at most 48 items (every item is a transaction and the vehicle remembers 64), and each item one of waypoint, takeoff, land, return to launch, a timed or counted loiter, change speed or delay, with a location relative to home and between 0 and 120 m (a landing may be at 0). Anything else (the fence, a mode, a servo, a relay, a gripper, a parachute) is refused. Item 0, the home placeholder, is not read by the autopilot (`AP_Mission::replace_cmd`, `read_cmd_from_storage`). A mission flown out of reach is a sequence that ends: there is no unlimited loiter and no jump, a waypoint's hold, a timed loiter and a delay last at most 300 s and a counted loiter at most 10 turns. An item marked `current` 2 or 3 (ArduPilot's Guided fly-to-here and altitude change, which it handles outside the mission) is refused.

Know what this means before you switch it on. A mission in Auto keeps flying when the ground station goes quiet, because that is its purpose: the pilot has no control of it until the link is back, apart from a separate radio. If a required parameter changes in flight, the vehicle withdraws control as described below, and the mission carries on or not as the autopilot's own parameters then say. Whether a mission may be flown out of reach where you fly is the pilot's and the regulator's decision.

## Landing on a marker

Off by default. With `[landing] target = true` in `vehicle.toml` the vehicle looks for a printed AprilTag (family tag36h11, the number `tag_id`, 0 unless set) in the camera's picture and tells ArduPilot where it is, so that ArduPilot's precision landing can steer the final descent onto it. It only reports. The vehicle never moves the copter, never changes a mode and never writes a parameter; the pilot chooses Land (QGroundControl: Land, with the copter over the tag, `docs/drone.md#landing-on-a-marker`), and ArduPilot does the rest. The pilot may take over at any time as for any Land.

```toml
[camera]
hfov_deg = 60        # the picture's horizontal field of view, from the lens and the sensor's crop
tilt_deg = -90       # a fixed camera, which must be exactly -90 (straight down); leave it out for a gimbal on the first mount (simulation only)
[landing]
target = true
tag_id = 0
```

What the vehicle sends: `LANDING_TARGET` (`MAV_FRAME_BODY_FRD`, for a fixed camera and for a mount whose camera moves with the airframe; `position_valid` 0) with the angles of the tag's centre from the picture's centre, from the vehicle's own identity (system 254, component 191), once per detection, at the rate the detector gives (`tag_fps`, 10 by default, up to 15: while ArduPilot's target estimator starts, it gives up after 500 ms without a reading it accepts, `EKF_INIT_SENSOR_MIN_UPDATE_MS` in `AC_PrecLand.cpp`). The fields and what ArduPilot does with them are in `libraries/AC_PrecLand/AC_PrecLand_MAVLink.cpp`, `handle_msg`, at the commit the simulation is built from (`fb773567`). The angles assume the picture's top is the front of the vehicle (`PLND_YAW_ALIGN` stays 0: `retrieve_los_meas` rotates every target by it); `distance` is the line of sight to a flat ground at the autopilot's height above home, because without a rangefinder ArduPilot has no other height (`AC_PrecLand.cpp`, `construct_pos_meas_using_rangefinder`). On sloping ground, or with a barometer that has drifted, that height is wrong and so is the offset ArduPilot corrects. Not measured on hardware, and not tried with a rangefinder (a positive `distance` replaces it).

Nothing is sent that was not seen in the current frame window: no prediction, no repeat of the last position. A sighting older than 500 ms by the time it would go out, counted from when the frame left the sender's tee (a little after the camera captured it), is dropped, and each one is sent once. When the tag leaves the picture the messages stop, and ArduPilot treats the target as lost 2 s later (`LANDING_TARGET_TIMEOUT_MS`).

### The camera must point down

The angles are only right for a camera that looks straight down with its picture's top towards the nose, so the vehicle sends nothing otherwise. A fixed camera must be configured at exactly `tilt_deg = -90`; the config is refused otherwise, because a tilt is not corrected. A gimbal camera is the camera on the first mount, and counts when the autopilot's own report of that mount's attitude (`GIMBAL_DEVICE_ATTITUDE_STATUS`, which ArduPilot sends from `AP_Mount_Backend::send_gimbal_device_attitude_status`, with `gimbal_device_id` `_instance + 1`, so 1 for the first mount; the quaternion is the servo backend's demanded body-frame angles, `AP_Mount_Servo::get_attitude_quaternion`, with no feedback from the mount) puts two things right: its axis within 5 degrees of straight down, and the picture's top (the camera's up axis, turned by the quaternion, on the horizontal plane) within 5 degrees of the nose. The second matters because a gimbal that looks down but is panned gives a picture turned by the pan. The target goes out in the body frame, so the rule holds only for a mount whose camera moves with the airframe: the servo backend does not stabilise in neutral (`AP_Mount_Servo::update_angle_outputs`), and its report is the demanded angle, not a measurement. A mount that stabilises its camera, or a backend whose quaternion means something else, has not been tried, so real mode refuses the landing target with a gimbal and accepts it only with a fixed camera; a gimbal is for the simulation. To bring a gimbal to real mode, trace it first (the sent angle's error against the copter's pitch and roll should be about 1 degree per degree for a camera that follows the airframe, 0 for one held level). Readings of other mounts come from the same autopilot address and are ignored. The vehicle asks for the attitude with `SET_MESSAGE_INTERVAL` every 5 s while none is fresh and the autopilot then streams it every 200 ms; a sighting is judged only against a reading at most 500 ms from it, before or after. Otherwise, with a tag in the picture, the pilot is told once in QGroundControl's message list, as a warning from the autopilot's address, and told again only after a target has gone out in between:

| Message | Why nothing is sent |
|---|---|
| `Landing target off: camera 70 deg from straight down` | The camera is further than 5 degrees from down. |
| `Landing target off: camera picture turned 90 deg from the nose` | The picture's top is more than 5 degrees from the nose (a panned gimbal). |
| `Landing target off: camera angle unknown` | The first mount's attitude has not been reported within 500 ms of the sighting. |
| `Landing target off: no height above home` | No fresh `GLOBAL_POSITION_INT`, or the copter is not above home. |

The decision is `sat_teleop_policy::landing::decide`, a pure function with a table test, and `sat_teleop_control::landing::Landing::next_frame` is the only place a `LANDING_TARGET` is made; it is written under the connection's lock like any control frame, only while `mavlink.control.target` is `autopilot`. The vehicle's metrics show `mavlink.landing` (`sent`, `withheld` by reason, and `last`: the newest target's angles in degrees, distance, the picture's turn and its age) and `tag` (the detector thread's state, runs, finds and time per run).

### Real mode

In real mode with the switch on, the vehicle refuses to start unless `require` holds the entries below, as `allow_missions` does for its parameters, and then, like every entry, compares them with what the autopilot reports:

```toml
require = [
    # ... the entries above, and:
    { name = "PLND_ENABLED", type = "int8", value = 1 },     # precision landing on
    { name = "PLND_TYPE", type = "int8", value = 1 },        # the MAVLink backend: a companion computer's LANDING_TARGET
    { name = "PLND_YAW_ALIGN", type = "real32", value = 0 }, # the picture's top is the nose
    { name = "PLND_STRICT", type = "int8", value = 1 },      # a lost target: retry, then pause and land straight down
    { name = "PLND_ALT_MIN", type = "real32", value = 0 },   # no altitude window (below)
    { name = "PLND_ALT_MAX", type = "real32", value = 0 },
]
```

The meanings are those of `libraries/AC_PrecLand/AC_PrecLand.cpp` and `AC_PrecLand_StateMachine.cpp` at `fb773567`. `PLND_TYPE` 1 is called "MAVLink" there. `PLND_YAW_ALIGN` (centidegrees, 0 by default) turns every target by that yaw (`retrieve_los_meas`), so it must be 0 for angles of a picture whose top is the nose. `PLND_STRICT` is 0 "Land Vertically (Not strict)", 1 "Retry Landing (Normal Strictness)" and 2 "Do not land (just Hover)". Real mode requires 1: 0 lands at once whether the tag is in view or not, and 2 (`VERY_STRICT`) returns `HOLD_POS` from `get_failsafe_actions` with no end, so a Land that has no tag in view would hover until the battery or the pilot ends it, including a failsafe Land and the descent at the end of RTL. The altitude window must be off because ArduPilot calls a target out of range, and then simply descends whatever `PLND_STRICT` says, below `PLND_ALT_MIN` or above `PLND_ALT_MAX` and whenever there is no valid rangefinder (`check_target_status`, `check_if_sensor_in_range`); both zero removes the window, so a lost target is judged lost at any height. The types are from the parameter definitions and not checked against a flight controller: confirm them in the parameter editor.

**`PLND_LAG` is yours to set, and is not checked.** `AC_PrecLand` pairs each sighting with the vehicle's attitude and motion `PLND_LAG` seconds back (default 0.02, range 0.02 to 0.25, needs a reboot) and uses the message's `time_usec` only for its 1 s validity (`GCS_MAVLINK::handle_landing_target`, `correct_offboard_timestamp_usec_to_ms`; `AC_PrecLand::init`, `construct_pos_meas_using_rangefinder`). So `PLND_LAG` should match the delay of the setup from the camera seeing the tag to ArduPilot receiving the message: exposure, encoding of the picture the detector reads, the detector and the link to the flight controller. A value that is too small pairs a sighting with a pose that is too new, and the copter overshoots the tag. Measure the delay for the camera in use; the vehicle neither measures nor checks it. The simulation's way: fly with `mavlink.landing.last` polled, record the copter's position and attitude beside each sent target's sighting time (`lab check sim-landing` writes them to `timeseries.csv`), compute the angles a body-fixed camera would see and find the time shift that best aligns them with the sent angles, then add the time from the sighting to the send (`to_send_ms`). A value measured that way still has to be flown: in the simulation 0.06 (the measured age) landed 2.8 m from the tag where 0.15 landed 0.01 m from it, one run each.

**What the pilot gets with the switch on.** `PLND_ENABLED` is not specific to the pilot's Land: a failsafe Land (`FS_GCS_ENABLE` 5, `FS_THR_ENABLE`, a fence breach), a mission's Land item and the descent of RTL run the same code (`Mode::land_run_normal_or_precland`, `Mode::precland_run`), so all of them behave as follows, whether or not the vehicle ever sends a target:

- **No tag was ever seen.** The state machine reports `TARGET_NEVER_SEEN` and `update` returns `FAILSAFE`; `get_failsafe_actions` says `HOLD_POS` for `FAILSAFE_INIT_TIMEOUT_MS` (7 s: "vehicle is completely stopped to give user the time to take over", with `PrecLand: Failsafe Measures` in the message list) and then `DESCEND`, a vertical landing where the copter is. Every such Land pauses about 7 s before it descends.
- **The tag is lost on the way down.** ArduPilot reports `PrecLand: Target Lost` 2 s after the last message (`LANDING_TARGET_TIMEOUT_MS`), keeps descending for `PLND_TIMEOUT` (4 s by default), flies back above where it last saw the tag and retries (up to `PLND_RET_MAX`, 4), and when a retry does not find the tag again, or those are used up, takes the same failsafe path (`retry_landing`): hold 7 s, then land vertically.
- **The tag stays in view.** The descent is steered onto it.

So with the switch on the copter is never steered down by a guess, and it always ends on the ground. The cost is the pause, and a vertical landing where the copter stands when the tag was not found: check that is acceptable over the site you fly, and that your failsafes allow the extra 7 s. With the switch off nothing here is required, but a `PLND_ENABLED` 1 left in the autopilot has the same pause.

### Checking it

Props off, with the real camera: point the camera forward and show it the tag; the pilot's message list says the camera is not down and `mavlink.landing.sent` stays 0. Point it down: `sent` counts up at about `tag_fps` and stops within a frame of covering the tag. Then, only on a test site and with a spare pilot ready to take over: hover 2 m off the tag, choose Land, cover the tag once the copter is descending, and see it keep descending for a few seconds, return above the tag, pause for about 7 s and land straight down.

## What is authenticated, and what is not

- **Control envelopes, lease requests and lease releases** are signed end to end with the controller's key. A relay, which holds no controller key, can drop or delay them, not forge or change them, and not use one for another kind of message (each signature starts with a constant for its kind) or for another vehicle (it carries the vehicle's id, `vehicle_id`, which `vehicle.toml` and `ground.toml` must share). The vehicle's challenge, sequence, epoch and nonce rules cover replay (`docs/drone.md#control-protocol`); a ground whose clock steps backwards has its lease requests refused (`replayed_request`) until it passes the last nonce the vehicle accepted.
- **What the vehicle says to the ground is not signed**: telemetry, video, challenges, command results and the vehicle's clock bound. A compromised relay can make the picture, the telemetry and the bound lie to the pilot. Treat the relay as trusted for what you see, and untrusted for what the vehicle does.
- The video stamp, the bound and the label are only as good as the clocks. A stuck V4L2 camera is detected (a repeated frame keeps its old capture time, `docs/hardware.md#a-stuck-camera`); a driver that stamps a repeated image as new, and any other camera source, is not.

## When a parameter changes in flight

The vehicle reads each required parameter again every 10 s and trusts a reading for 30 s, so a change shows within about 10 s and at the latest 30 s after the autopilot stops answering. When one no longer matches, `failsafe_params` fails and the vehicle withdraws control: the ground station's heartbeat, the sticks and every command go to the logging sink. The vehicle and the pilot then see:

- QGroundControl shows every command denied, and `mavlink.control.interlocks` and the log name the parameter.
- The autopilot stops hearing a ground-station heartbeat, so its GCS failsafe acts after `FS_GCS_TIMEOUT`, with the action `FS_GCS_ENABLE` has.
- If the parameter that changed is `FS_GCS_ENABLE` itself (set to 0, say), the failsafe the withdrawal relies on is the one that was switched off: nothing lands the vehicle. The autopilot keeps flying what it was doing with no ground station and no input, until a battery, geofence or radio failsafe or someone with a separate radio takes over. Do not fly without that radio.
- Putting the parameter back restores control by itself, within 10 s, with the lease and the challenge as they were if they have not expired.

Bench-test each of these before flying (checklist 4 and 5).

## Over a GEO link

Real mode has not been tried on a satellite link. On the emulated `geo` link the vehicle paces its video to the uplink it measured, and in the simulated flights the uplink queue stays at about 70 ms or less after the first seconds, with no envelope refused (`docs/drone.md#a-filled-uplink-under-geo`; `docs/video.md#climb-and-packet-pacing`). What can still fill the queue is the first try of a video quality before any uplink is measured (at the start, or after a route change) and a real sudden drop of the uplink. The vehicle then refuses envelopes as `challenge_expired`, and a copter can brake early or land through its GCS failsafe (`docs/drone.md#a-filled-uplink-under-geo`); expect that in the first seconds after the video starts and after a route change. Also unmeasured on hardware: the camera, the hardware encoder, the serial link, the power budget and every real link.

## Pre-flight checklist

Props off for items 1 to 8; the first flight is yours, not this list's.

1. The vehicle is on `mode = "real"` and `systemctl status sat-teleop-vehicle` is active; `journalctl -u sat-teleop-vehicle` shows no `logging sink` line.
2. `curl -s -H "Authorization: Bearer $(sudo cat /etc/sat-teleop/vehicle.token)" http://127.0.0.1:9100/metrics` shows `mavlink.control.mode` `real`, `interlocks` empty, `target` `autopilot`, `key_errors` empty and `clock.bound_ms` a number.
3. `chronyc tracking` on the Pi and on the ground machine is synchronised; the picture reads `LIVE` and the ground's metrics `clock.effective_ms` is a plausible few tens of milliseconds. On a ground with a fixed bound, put the laptop to sleep and wake it: expect `AGE UNKNOWN` until you restart the ground container, then `LIVE` again.
4. Change one required parameter in the autopilot (for example `RC_OVERRIDE_TIME`) and see `failsafe_params` fail and QGroundControl deny an arm; change it back. Then, props off and the vehicle armed on the bench, set `FS_GCS_ENABLE` to the value that disables it and watch what the autopilot does when the vehicle withdraws the heartbeat (nothing, by design: know it); set it back.
5. Stop chrony on the Pi (`sudo systemctl stop chrony`): the label goes to `AGE UNKNOWN` within 15 s, `clock.bound_ms` is null, `interlocks` stays empty and control continues. Start it again. Restart the vehicle with chrony stopped: control is refused with `clock_bound` until chrony is synchronised.
6. Cover and uncover the camera lens, and stop the camera's feed if you can: a V4L2 camera that repeats its frame should read `FROZEN` (this has not been tried on hardware).
7. Take the lease, arm with the props off, move the sticks and watch the autopilot's inputs; release the lease and confirm the GCS failsafe fires after `FS_GCS_TIMEOUT` (disarm first).
8. Pull the link (the Pi's network) with the lease held and confirm the autopilot's failsafe acts as configured.
9. Check the geofence and return-to-launch settings in the autopilot against the site, the weather and your licence conditions.
10. Only with `allow_missions`: with the props off, upload a short mission from QGroundControl's Plan view, arm, start it, pull the link, and confirm the autopilot reports `Radio Failsafe - Continuing Auto` and `GCS Failsafe - Continuing Auto Mode` (STATUSTEXT) and keeps the mission running. Then change `FS_OPTIONS` to clear bit 1 and see `failsafe_params` fail.
