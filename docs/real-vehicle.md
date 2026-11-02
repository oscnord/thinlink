# Flying a real vehicle

sat-teleop is meant for people who hold a drone pilot licence and want to fly a real vehicle over a satellite or cellular link. Real mode is how the software lets control reach a real flight controller. It is never the default: the testbed and every shipped example run in SITL mode, where control reaches only a simulated autopilot. Real mode has never flown a vehicle. Whether, where and how you may fly, and what the link is good enough for, are your decisions and your regulator's.

## What real mode is

`mode = "real"` in the `[mavlink]` table of `vehicle.toml`. The vehicle then sends control (the ground station's heartbeat, sticks, arm, mode and takeoff commands) to the autopilot only while every interlock below holds. When one does not, control goes to a logging sink instead: the commands are refused to the pilot (QGroundControl shows them denied), nothing is written to the autopilot, and the vehicle says which interlock failed in its log and in its metrics (`mavlink.control.interlocks`, each with a reason). The vehicle loads the keys once, when it starts. Changing, replacing or removing a key file takes effect at the next restart, and a vehicle that started with a readable key keeps verifying with it. Once they hold, the ground station's heartbeat reaches the autopilot as it does for SITL, so `FS_GCS_ENABLE` keeps its meaning.

## The interlocks

| Interlock | Holds when | Otherwise |
|---|---|---|
| `identity` | The autopilot sent a heartbeat of a vehicle type the gate knows (copter or rover) and it is not SITL: it ignored three probes for the simulator-only parameter `SIM_WIND_SPD` (`docs/architecture.md`, "Control protocol"). In SITL mode the rule is the reverse. | Control is refused. A real autopilot is never taken for SITL, and SITL is refused in real mode. |
| `controller_keys` | Every controller in the `controllers` list had a public key file that loaded when the vehicle started. | That controller cannot be verified, and control is refused for everyone until a restart finds the key. |
| `failsafe_params` | Every parameter of `require` was read back from the autopilot's own system and component within 30 s, in the configured type, and equals the configured value. | A missing, stale or different parameter, or an empty `require`, refuses control. |
| `clock_bound` | The vehicle had a fresh bound on its clock's error from chrony before it first sent control. | No fresh bound at the start, no control. `clock = "shared"` and `"fixed"` are refused in real mode. This interlock gates the start only (below). |

The clock bound is the one interlock that gates the start only. Once the vehicle has had a fresh bound, a stale one (chrony lost its sources, restarted, or `chronyc` failed) turns the ground's label to `AGE UNKNOWN` and shows in the vehicle's metrics (`clock.bound_ms` is null) and its log, and never withdraws control or the ground station's heartbeat: the bound only labels the video, and a display input must not trip a failsafe. The keys cannot go stale, and the failsafe parameters keep gating in flight (next section).

Independent of the interlocks, every control envelope must be signed (below), and the autopilot's own failsafes are what stops the vehicle when the link goes; the interlocks exist so that you know they are what you configured.

## Setup

1. **Keys.** `just deploy-keys <relay endpoint>` writes, besides the WireGuard keys, `deploy/secrets/ctl-ground.key` and `.pub` (the ground adapter's Ed25519 signing key, mode 600, never committed) and three bearer tokens (`vehicle.token`, `ground.token`, `job.token`, mode 600). `just vehicle-release` puts the public key, `vehicle.token` and `job.token` in `dist/vehicle`, `just ground-release` the private key, `ground.token` and `job.token` in `dist/ground`, `just relay-release` only `job.token` in `dist/relay`. A private key or token file that its group or others can access is refused when the program starts, with the file's name and `chmod 600`. Rotate by deleting `deploy/secrets/ctl-ground.*` and releasing again.
2. **Vehicle config.** Give the vehicle a name, `vehicle_id` in `/etc/sat-teleop/vehicle.toml`, and the same string as `vehicle_id` in the ground's `ground.toml` (every signature is made for it; the ground's messages are refused until they match). In `/etc/sat-teleop/vehicle.toml` set `mode = "real"`, `clock = "chrony"`, and `require` (below). Leave `debug_hooks` and `sitl_in_real_mode` out: they exist only in the testbed's builds (the `testbed` cargo feature), and a release build refuses to start on a config that names either. The release recipes (`vehicle-release`, `image-for`, `relay-release`, `ground-release`) build without the feature.
3. **Chrony.** `provision.sh` installs chrony on the Pi. The vehicle derives its bound from `chronyc -c tracking` (`docs/pi.md`, "Clock sync"). Check with `chronyc tracking`. The ground needs a bound too (`clock` in `ground.toml`, see "Clocks" below).
4. **Parameters.** `require` lists parameter names, the type the autopilot reports each in (`type`: `uint8`, `int8`, `uint16`, `int16`, `uint32`, `int32` or `real32`, as QGroundControl's parameter editor shows it) and the values the autopilot must report. A reading in another type, or from another component, is not the parameter you named. The vehicle only compares; it never writes a parameter. Set them in the autopilot first (Mission Planner, QGroundControl, or a `.parm` file) and put the same values in `require`.
5. **Tokens.** Each service has a token of its own (compared in constant time): `vehicle.token` guards the vehicle's metrics, `ground.token` the ground's lease endpoints and `job.token` the relay's metrics and job API, which the vehicle and the ground present to the relay. A compromised relay, which holds only the job API's token, cannot take the lease through the ground's endpoints or read the vehicle's. Without a token an endpoint listens on 127.0.0.1 only, whatever address it was given, and the relay does not start its job API; the shipped deployments bind the metrics to loopback and pass the token.

## Clocks

The picture is `LIVE` only when the newest frame's age plus a bound on how far the two machines' clocks can differ is small, so without a fresh bound on both sides the label reads `AGE UNKNOWN`, never `LIVE`. How the bound is made is in `docs/measurement.md`, "Clock assumption". `clock` in `ground.toml` says where the ground's comes from:

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

Why these: `FS_GCS_ENABLE`/`FS_GCS_TIMEOUT` are what lands the vehicle if the lease, the adapter or the link is lost. `RC_OVERRIDE_TIME` makes the sticks stop mattering shortly after the setpoints do (the vehicle refreshes a fresh setpoint every 200 ms and lets it lapse after 0.5 s). The radio failsafe covers the throttle the sticks stream drives. The geofence is the autopilot's own bound on where the vehicle can go whatever the ground sends. The testbed's rover set and its derivation are in `testbed/sat-teleop.parm` (`docs/testbed.md`, "SITL"); the copter's simulated-flight values and measurements are in `docs/simulation.md`.

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

## What is authenticated, and what is not

- **Control envelopes, lease requests and lease releases** are signed end to end with the controller's key. A relay, which holds no controller key, can drop or delay them, not forge or change them, and not use one for another kind of message (each signature starts with a constant for its kind) or for another vehicle (it carries the vehicle's id, `vehicle_id`, which `vehicle.toml` and `ground.toml` must share). The vehicle's challenge, sequence, epoch and nonce rules cover replay (`docs/architecture.md`, "Control protocol"); a ground whose clock steps backwards has its lease requests refused (`replayed_request`) until it passes the last nonce the vehicle accepted.
- **What the vehicle says to the ground is not signed**: telemetry, video, challenges, command results and the vehicle's clock bound. A compromised relay can make the picture, the telemetry and the bound lie to the pilot. Treat the relay as trusted for what you see, and untrusted for what the vehicle does.
- The video stamp, the bound and the label are only as good as the clocks. A stuck V4L2 camera is detected (a repeated frame keeps its old capture time, `docs/pi.md`); a driver that stamps a repeated image as new, and any other camera source, is not.

## When a parameter changes in flight

The vehicle reads each required parameter again every 10 s and trusts a reading for 30 s, so a change shows within about 10 s and at the latest 30 s after the autopilot stops answering. When one no longer matches, `failsafe_params` fails and the vehicle withdraws control: the ground station's heartbeat, the sticks and every command go to the logging sink. The vehicle and the pilot then see:

- QGroundControl shows every command denied, and `mavlink.control.interlocks` and the log name the parameter.
- The autopilot stops hearing a ground-station heartbeat, so its GCS failsafe acts after `FS_GCS_TIMEOUT`, with the action `FS_GCS_ENABLE` has.
- If the parameter that changed is `FS_GCS_ENABLE` itself (set to 0, say), the failsafe the withdrawal relies on is the one that was switched off: nothing lands the vehicle. The autopilot keeps flying what it was doing with no ground station and no input, until a battery, geofence or radio failsafe or someone with a separate radio takes over. Do not fly without that radio.
- Putting the parameter back restores control by itself, within 10 s, with the lease and the challenge as they were if they have not expired.

Bench-test each of these before flying (checklist 4 and 5).

## Over a GEO link

Real mode has not been tried on a satellite link. On the emulated `geo` link the vehicle paces its video to the uplink it measured, and in the simulated flights the uplink queue stays at about 70 ms or less after the first seconds, with no envelope refused (`docs/simulation.md`, "A filled uplink under geo"; `docs/architecture.md`, "The pacer"). What can still fill the queue is the first try of a video quality before any uplink is measured (at the start, or after a route change) and a real sudden drop of the uplink. The vehicle then refuses envelopes as `challenge_expired`, and a copter can brake early or land through its GCS failsafe (`docs/architecture.md`, "Control under a filled uplink"); expect that in the first seconds after the video starts and after a route change. Also unmeasured on hardware: the camera, the hardware encoder, the serial link, the power budget and every real link.

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
