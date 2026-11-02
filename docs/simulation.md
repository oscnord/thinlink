# Simulated flight

`just sim-up` brings up the testbed with a simulated quadcopter in place of the rover. Gazebo renders the copter's camera, the picture goes through the emulated link like any other video, and you fly it from QGroundControl. Control reaches ArduCopter SITL only: nothing here commands a real vehicle.

## Run it

```sh
just sim-up              # builds the images the first time (long), then starts the testbed
just impair geo          # optional: the link of a profile (lan is the default)
just take-control        # the ground adapter takes the control lease
```

Start QGroundControl (nothing to set up, `docs/qgc.md`). About two minutes after `sim-up` the copter answers and the camera's picture arrives with its age label. `just lab-up` brings the default rover testbed back.

The images are an ArduCopter 4.7.2 SITL (`testbed/Containerfile.sitl-copter`, the same ArduPilot commit as the rover's) and Gazebo Harmonic with a camera (`testbed/Containerfile.sim`). `testbed/compose.sim.yml` adds them to the testbed; the topology, the link and every other service are the default testbed's.

`just regional=1 sim-up` starts the vehicle with the person detector and softening (`soften_factor = 2`) on (`testbed/config/vehicle-sim-regional.toml`). The world has two standing people (Gazebo Fuel's "Standing person" and "Casual female" by OpenRobotics, CC0 1.0, fetched by version and hash in `testbed/Containerfile.sim`) and a white landing pad with an H, all in front of the copter's camera at the take-off point. At rest the camera sits on the runway and sees the people standing 5 and 6.5 m ahead, and the pad only as a sliver on the horizon; it shows from the air. The detector found both people in every run (100 runs, 200 people, one run 30.7 ms at p50 and 52.8 ms at p95 on the build container's CPU) and outlines them in amber, and a box drawn on the pad in QGC (`docs/qgc.md`, "Regional video: drawing a box") keeps it sharp.

## Flying it from QGC

The vehicle accepts QGC's joystick (`MANUAL_CONTROL`) only from the lease holder. Take the lease first (`just take-control`; `just release-control` gives it back; `docs/qgc.md`, "Taking control").

**The joystick must be sending before you arm, and while you fly in a mode that uses the sticks.** The copter treats the end of the stream as a failsafe (below), and it refuses to arm without one: "Throttle below failsafe". Either way works:

- **Virtual joystick**: turn on the virtual joystick in QGC's Application Settings (its `virtualJoystick` setting, 5.1.5). Its throttle returns to the middle when released, which is what the copter wants.
- **Gamepad**: plug it in, open Vehicle Setup, Joystick, tick Enabled, run the Calibration (move every stick to its ends when asked) and leave "Center stick is zero throttle" off. QGC then sends the throttle stick's middle as hold altitude, and a spring-centred stick holds the copter's height in AltHold and Loiter. The default axis rate, 25 Hz, is what the vehicle's refresh expects.

Then, in the Fly view: Takeoff (it switches to Guided, arms and climbs to the altitude you choose), or arm and fly by hand. The vehicle's command gate allows Takeoff, arming and the mode changes behind Land, Return and Pause (Brake).

### What the command gate allows

QGC 5.1.5 sends a mode change to ArduPilot as `COMMAND_LONG` `MAV_CMD_DO_SET_MODE` (`docs/qgc.md`, "What QGC 5.1.5 sends when it connects"). Neither `COMMAND_INT` nor `SET_MODE` passes the gate.

Before any of this, the vehicle sends control to the copter only after it has identified it as a simulator (`docs/architecture.md`, "Control protocol"; the `sim-check lan` run is in `docs/measurement.md`, "Results summary").

The vehicle learns what it is from the autopilot's heartbeat (`MAV_TYPE`) and refuses every mode change while it does not know, and any mode outside its type's list:

| Vehicle | Modes passed | Evidence for the list |
|---|---|---|
| Copter | Stabilize 0, AltHold 2, Guided 4, Loiter 5, RTL 6, Land 9, PosHold 16, Brake 17 | `ArduCopter/mode.h` `Mode::Number`; Guided, Land, RTL and Brake are what the Fly view's Takeoff, Land, Return and Pause use, Stabilize, AltHold, Loiter and PosHold are the stick modes suggested above |
| Rover | Manual 0, Hold 4, RTL 11 | `Rover/mode.h` `Mode::Number`; Manual is the mode of the rover checks and of `docs/qgc.md` ("Manual for driving"), Hold of the `control-freshness` stop command and of the rover's GCS failsafe (`testbed/sat-teleop.parm`), RTL is added |

Acro, Auto, Flip, Throw, Drift, Sport, AutoTune and the rest are denied. Auto (mode 3) is passed for a copter only when the vehicle has `allow_missions` on ("Missions" below). A mode change into Stabilize, AltHold, Loiter or PosHold also needs the joystick sending: with no fresh `MANUAL_CONTROL` from the lease holder (the freshness the vehicle applies to every continuous setpoint) it is denied, so a braked copter stays braked until the stream is back. A takeoff is denied unless its altitude is a positive number up to 120 m above home. The vehicle counts each refusal by reason (`mode_not_allowed`, `no_fresh_sticks`, `takeoff_altitude`, `mission_not_allowed` under `mavlink.control.rejected` in its metrics) and QGC sees the command denied.

### Which modes per profile

| Profile | Suggested |
|---|---|
| `lan` | Stabilize, AltHold, Loiter: the sticks answer within a frame or two. |
| `lte-good`, `lte-bad` | Loiter or PosHold: the position hold forgives a late stick. |
| `geo` | Guided takeoff, then short stick inputs in Loiter or PosHold, and Brake, Land or Return. Everything is about a third of a second late each way, so keep inputs short and let the copter settle. |

Guided go-to is not allowed through the gate. A mission upload is, but only with `allow_missions` on ("Missions" below); otherwise there are no waypoints. Pulling the throttle to the bottom in flight is a radio failsafe (the copter brakes); land with the Land button.

## Failsafes

The copter never falls when control goes away. The parameters are in `testbed/sat-teleop-copter.parm`, with their ArduPilot sources.

| What is lost | What the copter does | Measured (`lan`, `geo`) |
|---|---|---|
| The setpoint stream, lease and adapter still there (joystick off, link blip) | Brake: it stops where it is and holds its height. The pilot sends the stream again and changes mode to fly on; until then the gate denies a change into a mode that flies on the sticks. | Brake 0.92 to 0.97 s (`lan`), 1.5 s (`geo`) after the last setpoint, a sag of 0.02 m at most |
| The lease or the ground adapter | Land, after the GCS failsafe's 2 s. | Land 2.4 to 2.6 s (`lan`), 2.6 to 2.8 s (`geo`) after the last setpoint, down and disarmed after 30 to 31 s, never faster than 0.7 m/s |

The simulator has a radio the autopilot always sees, parked at throttle 1000 us. With ArduPilot's default failsafe threshold a Loiter copter descended at full rate into the ground when the stream stopped (measured). The defaults file moves the failsafe threshold above the parked throttle, which makes the end of the stream a radio failsafe, and re-ranges the throttle channel so that the stick's middle stays the middle.

A guided flight or a landing ignores the radio failsafe (so Takeoff and Land need no stick); the GCS failsafe still lands them when the lease goes.

ArduPilot itself also guards a braked copter: in the radio failsafe it refuses a change into Stabilize ("in RC failsafe", `Copter::set_mode`), and AltHold, Loiter and PosHold hold the height there (zero climb rate while the failsafe is on). Three seconds after each of AltHold, PosHold and Loiter was accepted with the stream off, the braked copter had kept its height (run 1793584280458).

### A filled uplink under `geo`

Under `geo` a video queue at the uplink makes the vehicle refuse control envelopes (`challenge_expired`), which brakes or lands the copter early (`docs/architecture.md`, "Control under a filled uplink"). Before the vehicle paced its climbs and its video (`docs/architecture.md`, "Pacing a climb"), two `sim-flight geo` runs (1793616684672, 1793616794347) had 29 and 1 refused envelopes, a queue delay of 929 and 590 ms, and the copter braked and landed early (incomplete, fail). With pacing five runs (1793619172418, 1793619282827, 1793619392544, 1793619502337, 1793619613808) refused none beyond the check's own refused mode change, with a queue delay of at most 71 ms (p95 40 to 49 ms), the video's age p50 318 to 320 ms and p95 330 to 335 ms as before, and Live over 70 to 100 % of the leg's frames. With the pace bounded by the uplink believed (`docs/architecture.md`, "The pacer"), three `sim-flight geo` runs on a fresh stack (1793628787473, 1793628925988, 1793629038177) passed every grade with no refused envelope beyond the check's own, a queue delay of 65 and 69 ms after the first seconds (611 ms in the first run's first seconds, the round trip jumping from the `lan` to the `geo` delay, not a queue) and the video's age p50 320 to 321 ms. The simulation's real-time factor moves every time in this section (brake, land, on the ground): the runs above were 0.97 to 1.01, and runs at 0.51 to 0.88 on a loaded host failed the Brake, Land and on-ground budgets by the same factor with a clean link. The simulated camera's top quality now runs at about the uplink, not at its nominal 1200 kbit/s, until the link has carried more.

A run that starts right after `just impair geo` on a stack that was on `lan` still shows the adaptation draining at its start: the round trip jumps from about 1 ms to 600 ms, which it reads as a queue, and takes the cut back as a base shift. Nothing is queued then and no envelope is refused.

What can still fill the queue on `geo` is the first try of a quality with no uplink measured and a real fall of the uplink (`docs/architecture.md`, "Control under a filled uplink"); refusals in that time can brake or land the copter, which is the failsafe working. `sim-flight` records the refusals per reason and phase, the queue delay, the adaptation steps and every mode change with the autopilot's own words in `summary.json` (`readings`), and the vehicle's latest steps, refusals and pacer in `vehicle_at_end`, so a run that goes wrong says why.

## Missions

Off by default. `just sim-mission <profile> on|off` brings the simulation up with `testbed/config/vehicle-sim-missions.toml` (`allow_missions = true`) or `vehicle-sim.toml` and runs `lab check sim-mission` through the emulated link of the profile. Fly one by hand with the switch on: `just sim-up vehicle-sim-missions.toml`, `just take-control`, build a mission in QGC's Plan view (`docs/qgc.md`, "Missions"), upload it and start it from the Fly view.

`testbed/sat-teleop-copter.parm` sets what a mission out of reach relies on: `FS_OPTIONS` 15 (bits 0 and 1, continue in Auto on the radio and the GCS failsafe, with the bits it had; ArduPilot `fb773567`, `ArduCopter/Copter.h`, `FailsafeOption`), `FENCE_ENABLE` 1, `FENCE_ACTION` 1 (RTL or Land on a breach), `FENCE_TYPE` 3 (maximum altitude and circle), `FENCE_ALT_MAX` 100 m, the default circle of 300 m and `BATT_FS_LOW_ACT` 2 (RTL; the simulated battery sets no low threshold, so it never fires in the checks). Nothing else changes: Guided, Loiter, Brake and the stick modes behave as above.

What `sim-mission` does and what it measured (`geo`, `1793656806455` with the switch on, `1793657044906` with it off):

| | Switch on | Switch off |
|---|---|---|
| Upload of 6 items (home placeholder, takeoff, three waypoints 20 m apart to the north at 10 m, landing) | accepted in 4.5 s: one `MISSION_COUNT`, six items, no repeat | `MISSION_COUNT` answered `MISSION_ACK` denied |
| Start | Guided, arm, `MAV_CMD_MISSION_START` accepted, Auto 33.5 s to the first waypoint | `MISSION_START` and a change into Auto denied; the vehicle counted `mission_not_allowed` 2 and `mode_not_allowed` 1 |
| Link cut (adapter killed) at the first waypoint | the autopilot said "Radio Failsafe - Continuing Auto" and "GCS Failsafe - Continuing Auto Mode"; items 3 and 4 reached, the landing item flown, always in Auto, on the ground and disarmed 42.1 s after the cut | |

The upload takes about seven round trips (`geo` is about 0.6 s each way and back): QGC waits 1.5 s for each next request, so it never sent `MISSION_COUNT` a second time. ArduPilot asks for the items with `MISSION_REQUEST`, not `MISSION_REQUEST_INT` (`MissionItemProtocol::queued_request_send`, and QGC 5.1.5 handles both); the first run read only `MISSION_REQUEST_INT` and so saw no request in 9 s, with six `MISSION_COUNT` written to the autopilot.

With the fence and `FS_OPTIONS` values in the parameter file, `sim-flight geo` passed 16 of 16 (`1793658892958`, real-time factor 1.00); an earlier run on the same stack (`1793658691410`) at a real-time factor of 0.62 on a loaded host failed the Brake, Land and Live budgets by about that factor, as in "A filled uplink under `geo`". Later `sim-mission geo` runs with the stricter item and parameter rules: `1793658459813` (on) and `1793658636373` (off) passed.

The check cuts the link by killing the ground adapter, as `sim-flight` does, so the lease lapses and the autopilot's GCS heartbeat stops; the radio failsafe fires too, because the setpoint stream stops, and `FS_OPTIONS` bit 0 lets Auto carry on through that as well. A cut inside the fence is the only case it covers; a breach of the fence was not flown.

## Sightings out of reach

`just regional=1 sim-up` and then `just lab-check sightings outage-30s` runs `lab check sightings` (`docs/architecture.md`, "Sightings"). The `outage-30s` profile is `lan` plus a 30 s blackout of both directions from 5 s after the timeline starts, while the copter, still on the runway, has the two standing people in view. The check is a ground station and grades: no sighting arrives during the cut; after the restore, sightings arrive as `STATUSTEXT` (`person -35.36326 149.16524 0m 23:33:51`: the copter's position at SITL's home, altitude above home, the UTC time first seen), with positions that agree and times of day inside the cut give or take 3 s; the job API lists as many (all marked told, each with a snapshot within 1.5 s); and the first listed snapshot can be fetched. `vehicle-sim-regional.toml` keeps 4 MiB of snapshots (the others 512 KiB), because a 30 s outage at one snapshot a second does not fit half of 512 KiB and the oldest outage snapshots, the ones the first sightings are in, would go first. Run `1793662423232-sightings-outage-30s` passed 9 of 9: two sightings (one per person, both told), listed with their snapshots, one fetched (13 350 bytes). An earlier run on the 512 KiB store failed only `snapshot_named`: the snapshots of the first seconds of the cut were gone, which is the store's rule that the newest outage snapshots win.

## The camera

The copter's gimbal camera is rendered headless by Gazebo at 640x360 and 15 frames per second. It looks forward and 20 degrees down: the defaults file holds the gimbal in Neutral mode (`MNT1_DEFLT_MODE 1`, `MNT1_NEUTRAL_Y -20`), because the simulator's parked radio would otherwise drive it to the limits of the stock gimbal parameters, and the picture shows the runway, the grass and the scenery. The `GstCameraPlugin` of ardupilot_gazebo encodes it to H.264 with x264 (`bitrate` 800 kbit/s, `speed-preset` 6 medium, `tune` 4 zero latency, `key-int-max` 10 frames: `src/GstCameraPlugin.cc` lines 549 to 551 at the pinned commit) and sends RTP to the vehicle's camera ingest (`testbed/config/vehicle-sim.toml`), which decodes it (about 1 ms), stamps it and hands it to the sender. So the frame's age, as the label shows it, starts when the picture reaches the vehicle software: it does not include the render and the plugin's encode, which were not measured separately.

Gazebo runs in the vehicle's network namespace and the ingest accepts only the vehicle's own address, so the picture cannot reach the ground except through the link.

## Resources

The Podman machine has 2 GiB and no GPU, so Gazebo renders in software (Mesa llvmpipe) and the world is kept lean: no shadows, no GUI services, 21 models, one grass texture. Measured next to the testbed:

| | Measured |
|---|---|
| Gazebo, memory | the container's cgroup peaked at 1064 MB while it started (page cache included; the resident part measured earlier at 860 to 970 MB for a few seconds); 676 to 702 MB afterwards (the container is capped at 1100 MB) |
| Gazebo, CPU | 146 % of one core (`podman stats`, 40 samples in 160 s while `sim-flight` ran, run 1793584798866) |
| ArduCopter SITL under qemu | 27 % of a core, 18 to 22 MB (the same samples) |
| Machine memory in use | about 880 MB with the default testbed, 1511 to 1526 MB with the simulation (`free` in the machine, 413 to 428 MB available): the start peak of the Gazebo container comes on top of what the rest of the testbed uses, which leaves little for anything else |
| Real-time factor | 1.04 to 1.06 (simulated seconds per second, from the autopilot's own clock; the world's cap is 1.25), in the eight `sim-flight` runs of this section and the budgets |

The x86_64 SITL keeps up with Gazebo's lockstep under qemu, so SITL is not built for arm64. The world's step is 1 ms with a real-time cap of 1.25: at exactly 1.0 the factor had settled at 0.88.

Starting the simulation with other heavy containers running can run the machine out of memory; Gazebo is the process that gets killed.

## What the check grades

`just sim-mission` runs `lab check sim-mission` ("Missions" above); it has no budget. `just sim-check <profile>` runs `lab check sim-flight` (arm, takeoff, a leg at about 4 m/s in Loiter within a 3 m height band, both failsafes, the refused mode change, the video Live at the ground) and `lab check qgc-status`. Results are in `results/<run>/`. The Brake and Land times, the sag, the time to the ground and the minimum Live share are graded against `[sim-flight.lan]` and `[sim-flight.geo]` in `perf/budgets.toml`, which cite the three runs each is derived from; Land's descent must stay under 1.875 m/s, 1.25 times ArduCopter 4.7.2's default `WP_SPD_DN` of 1.5 m/s (`LAND_SPD_HIGH_MS` is 0, so Land uses `WP_SPD_DN`: `ArduCopter/mode.cpp`, `land_run_vertical_control`; `AC_WPNav.cpp`; no pinned parameter file or `testbed/sat-teleop-copter.parm` sets either). Runs on `lan` and `geo`:

| Run | Result |
|---|---|
| `sim-flight lan` 1793585480173 | pass, 16 of 16; Brake 924 ms, Land 2583 ms, down 30123 ms, video 100 % Live, age p50 4 / p95 5 ms, 16.1 new frames/s, real-time factor 1.06 |
| `sim-flight geo` 1793585642746 | pass, 16 of 16; Brake 1571 ms, Land 2574 ms, down 30503 ms, video 100 % Live, age p50 317 / p95 330 ms, 8.0 new frames/s, real-time factor 1.06 |

Label shares and frame ages of the simulated camera and of the test pattern, for the content-dependent encoding data of the latency campaign, from `qgc-status` (30 s healthy window):

| Camera | Profile | Run | Live share | Age p50 / p95 | New frames per s |
|---|---|---|---|---|---|
| Test pattern | `lan` | 1793580641660 | 100 % | 11 / 14 ms | 15.0 |
| Simulation | `lan` | 1793585580178 | 100 % | 5 / 7 ms | 14.5 |
| Test pattern | `geo` | 1793580688475 | 100 % | 323 / 338 ms | 15.0 |
| Simulation | `geo` | 1793585752565 | 79.5 % | 319 / 330 ms | 11.5 |

The test pattern's rows are from before the camera showed the world; the pattern does not depend on it. Under `geo` the simulated camera is Live 80 % of the time (the rest Low Rate) where the test pattern is Live all the time, its frame rate falls to 11.5 per second and the adaptation settles at rung 1 (its second video quality step, after the 1280x720/15 fps top one), so `ladder_settled` reads incomplete; every other assertion and budget of that run passed. The three `sim-flight geo` runs behind the budget read Live 83 %, 64 % and 100 % of the leg.

## Provenance and licences

Everything is baked into the images at build time; nothing is fetched when it runs.

- ArduCopter 4.7.2 SITL binary and `copter.parm`, `gazebo-iris.parm`: ArduPilot (GPL-3.0), git `fb773567896014da007bca58358704d4a0d8516f`, fetched by URL and SHA-256.
- Gazebo Harmonic (gz-sim 8.15.0 and its libraries, Apache-2.0) from packages.osrfoundation.org for Ubuntu 22.04, pinned by version; the archive key is checked by hash.
- ardupilot_gazebo (LGPL-3.0): commit `f116b03552d05c8d0b0bed5ed67033a1183121d1` from a checksummed tarball. The plugin, the iris and gimbal models, `gazebo-iris-gimbal.parm` and the runway model come from it. Its model files name their authors but state no licence of their own, except `parachute_small` (Apache-2.0), which is not used. The runway's grass texture is `Grass004_2K_Color.jpg`; the name matches a CC0 texture set of that name at ambientCG, which the repository does not say. Verify its licence, or replace the texture, before any image is shared.
- The world (`testbed/sim/world.sdf`) is derived from ardupilot_gazebo's `iris_runway.sdf`; the scenery in it is boxes and cylinders written for this project. `testbed/sim/models/field` re-uses the runway model's meshes and textures with one map.
- `testbed/Containerfile.sim` patches the pinned sources in two places, each checked by the build: the camera plugin starts streaming with the first frame, and the gimbal camera's size, rate and destination are set.
- The Ubuntu packages `testbed/Containerfile.sim` installs with `apt` float with the base image (only the Gazebo packages are pinned by version), which is acceptable for a bench image.
