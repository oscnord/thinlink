# Drone and operator use

The drone adds MAVLink control and retained detail to the [video path](video.md). QGroundControl uses the ground adapter. The vehicle owns the lease and validates commands. [Hardware](hardware.md) covers deployment; [real mode](real-mode.md) covers its configuration.

## QGroundControl

QGroundControl 5.1.5 runs natively on macOS.

1. Run `just lab-up` and start QGC. Enable Application Settings, General, AutoConnect to UDP. UDP 14550 must be free.
2. If AutoConnect is off, add a UDP link in Application Settings, Comm Links, port 14550, with no server address.
3. Keep the video UDP URL at `0.0.0.0:5600`. Enable Low Latency Mode in Application Settings, Video; otherwise QGC adds an 80 ms jitter buffer and presentation-time waits.

The adapter sends telemetry to `host.containers.internal:14550` and video to `host.containers.internal:5600`. Podman's gateway forwards these to the Mac. [Video freshness](video.md) defines the labels. [Age boundaries](video.md#frame-age-and-clocks) depend on the configured source.

If the adapter dies, QGC holds its last picture and label. Its UDP watchdog shows "WAITING FOR VIDEO" after about 4 to 5 s. Reconnect delays grow from 1 s to 30 s. An output-only failure restarts that pipeline after 1 s.

### Camera discovery

The adapter announces component 100 as camera stream 1, "overview", using `[video] camera_uri = "udp://0.0.0.0:5600"` in `testbed/config/ground.toml`. QGC selects "UDP h.264 Video Stream". Camera information, stream information/status and tracking commands stay at the adapter. Other camera commands receive `MAV_RESULT_UNSUPPORTED`.

Camera heartbeats follow vehicle heartbeats. Camera replies stop 1.2 s after the vehicle's last message. QGC reports Communication Lost after 3.5 s without any vehicle-system message and removes a silent camera after 5 s.

### Direct stream viewing

For RTSP in QGC, remove `camera_uri` from `testbed/config/ground.toml`, run `just lab-up`, then select "RTSP Video Stream", `rtsp://127.0.0.1:8554/live`, and Low Latency Mode. The direct stream has no freshness labels. QGC's default RTSP Video Timeout is 8 s.

Mac RTSP uses TCP. QGC tries UDP first, so startup can take several seconds.

```sh
ffprobe -v error -rtsp_transport tcp -show_streams -select_streams v rtsp://127.0.0.1:8554/live
ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/live
```

MediaMTX's WebRTC player is `http://127.0.0.1:8889/live`, with media on `127.0.0.1:8189/udp`. Use Chrome Canary with an isolated profile.

### Checking the display

| Action | Expected display |
|---|---|
| `just lab-up` | `LIVE`, about 0.01 s age on `lan`. |
| `just video-hook pause` | `STALE` after about 0.6 s; `NO VIDEO` after about 2 s. |
| `just video-hook freeze` | `FROZEN` after about 0.5 s. |
| `just video-hook resume` | New frames restore the label. |
| `just impair geo` | Healthy age about 0.33 s. |

Regional tracking and marker landing lack native QGC display measurements. Container checks and optical glass-to-glass measurements cover different paths.

## Taking control

1. Run `just lab-up` and wait for the vehicle in QGC.
2. Run `just take-control`. Another holder returns `409` with `lease_held`. Use `just control-status` to inspect the lease.
3. For the rover, choose Manual and arm. Enable a joystick or virtual thumbsticks. QGC sends `MANUAL_CONTROL` at 25 Hz; the rover uses y for steering and z for throttle.
4. Disarm and run `just release-control`.

A lease expires after 3 s without an accepted envelope. Reconnect requires another `just take-control`; commands are not replayed. Copter setup is under [Flying from QGC](#flying-from-qgc).

## What QGC can and cannot do

Read-only traffic needs no lease. State changes require the lease and [command gate](#command-gate). Missing permission returns `MAV_RESULT_DENIED`; unsupported commands return `MAV_RESULT_UNSUPPORTED`. Heartbeats and stick messages without a lease are dropped. Missions also require `allow_missions`.

Parameter downloads work through `PARAM_REQUEST_LIST`. MAVLink FTP is dropped, so QGC waits through its FTP retries before downloading parameters. Parameter writes, fence/rally uploads and guided go-to are refused.

## Control protocol

Default `mode = "sitl"` sends control only after a matching vehicle heartbeat and `SIM_WIND_SPD` identify SITL. Other destinations use the logging sink: setpoints count as `sunk`, and discrete commands return `no_destination`. Explicit `mode = "real"` uses [real-mode configuration](real-mode.md#interlocks).

### Identity and authentication

| Configuration | Meaning |
|---|---|
| Vehicle `[mavlink] controllers` | Controller `address` and `public_key`; the lease belongs to the source address. |
| `vehicle_id` | Must match on vehicle and ground. |
| Ground `control_key` | Signs lease requests, releases and envelopes. |
| `clock` | Clock source; testbed uses `"shared"`. |
| `vehicle.token`, `ground.token`, `job.token` | Bearer tokens for metrics, the lease endpoint and retained jobs. |

Controller keys load at startup. Bad or missing signatures return `bad_signature` or `unsigned`. Replies, telemetry and video are unsigned. `POST /lease`, `DELETE /lease` and `GET /status` use the ground token. Endpoints without a configured token bind to loopback; the job API requires a token.

### Envelope protocol

JSON messages occupy one UDP datagram, at most 1200 bytes.

| Message/kind | Purpose |
|---|---|
| `lease_request { peer, nonce, sig }` | Ask for a lease; `peer` is informational. |
| `lease_release { lease, sig }` | Release the holder's lease. |
| `envelope { epoch, lease, seq, challenge, kind, mavlink, region, sent_unix_ms, sig }` | Signed command; `mavlink` contains one base64 MAVLink 2 frame. |
| `liveness` | GCS heartbeat or lease renewal without a frame. |
| `continuous` | Latest `MANUAL_CONTROL`. |
| `{ discrete: { txn } }` | Allowed command or mission transaction. |
| `region` | Box selection, without a MAVLink frame. |
| `lease_grant`, `lease_denied`, `challenge` | Lease answer and freshness challenge. |
| `command_result`, `rejected` | Applied, repeated, unsupported or rejected command. |
| `clock_query { nonce }`, `clock { nonce, bound_ms }` | Clock-bound query and reply. |

Accepted envelopes need the current epoch, holder, lease and challenge, plus increasing sequence numbers per kind. Challenge age uses the vehicle's monotonic clock. `sent_unix_ms` is measurement only. Discrete transactions apply once per lease; identical retries return `repeated`. A changed command under the same transaction returns `txn_conflict`.

### Control lifetimes

Values come from `[control]` in `testbed/limits.toml`.

| Key | ms | Behavior |
|---|---:|---|
| `challenge_lifetime_ms` | 1500 | Maximum challenge age; challenges arrive every 500 ms. |
| `setpoint_lifetime_ms` | 500 | Maximum setpoint refresh life, also capped by challenge expiry; refresh every 200 ms. |
| `liveness_interval_ms` | 1000 | Empty renewal when no other envelope is sent. |
| `ack_window_ms` | 4000 | Window for matching station command retries. |
| `lease_lifetime_ms` | 3000 | Lease expiry since the last accepted envelope. |
| `throttle_release_deadline_ms` | 1825 | Rover check deadline for zero throttle. |

Rover `RC_OVERRIDE_TIME 0.5` ends override after the last refresh. `testbed/thinlink.parm` selects Hold after 3 s without GCS heartbeat or `MANUAL_CONTROL`; the rover coasts without brakes.

### A filled uplink under `geo`

Video and control share the 512 kbit/s FIFO. Propagation and challenge phase consume 600 to 1100 ms of the 1500 ms lifetime. Queue delay can cause `challenge_expired`, especially during initial capacity measurement or a sudden capacity drop. Refused commands are not queued for later execution.

## MAVLink path

```text
SITL 127.0.0.1:5760 <-> companion <-> mavlink-router 10.77.0.1:14550
  <-> ground 10.77.0.3:14551 <-> QGC host.containers.internal:14550
                            <-> checks 192.168.232.40:14552
```

`testbed/config/vehicle.toml` configures `[mavlink] autopilot`, `relay` and `control`. Ground `listen`, `relay`, `gcs`, `control` and `vehicle` are in `testbed/config/ground.toml`. Control envelopes use vehicle `10.77.0.2:7700` and ground `10.77.0.3:7701`.

Raw traffic permits parameter/mission downloads, stream-rate requests, time sync and information requests. Commands use envelopes. Telemetry uses `ardupilotmega`, one frame per datagram. Reconnect backoff is 0.5 to 5 s; 5 s without data triggers reconnect and clears queued replies.

`[telemetry] bytes_per_s = 8000`, `offered_percent = 250` and `essential_bytes_per_s = 200` in `testbed/limits.toml` govern the shared uplink allocation. Heartbeats always pass. Replies wait in a 64 KiB queue for at most 10 s; bulk traffic yields to replies.

## Predictive display

The hollow dashed ring predicts camera-centre motion from attitude rates, frame age and half the clock-query round trip. That delay is an estimate. Ground `[camera] hfov_deg` supplies horizontal field of view; `tilt_deg` adds a horizon for a fixed camera. Without `[camera]`, nothing is drawn.

Drawing needs `LIVE` or `LOW RATE`, known age, fresh delay and attitude at most 1 s old. The simulated camera uses `hfov_deg = 114.6`. Real-flight prediction accuracy is unmeasured.

## Retained detail

The vehicle retains full-resolution JPEGs. Retrieval selects the nearest snapshot at or before the requested capture time. Output labels identify the historical frame and its capture time.

### Snapshot store

| Vehicle config | Testbed value | Meaning |
|---|---|---|
| `[snapshot] fps`, `quality` | 1, 90 | Snapshot rate and JPEG quality. |
| `[store] dir` | `"/snapshots"` | Snapshot directory. |
| `max_bytes`, `max_age_s` | 524288, 120 | Ordinary retention bounds. |
| `min_free_bytes`, `gap_after_ms` | 4194304, 1500 | Free-space floor and reported capture-gap threshold. |
| `outage_share_percent`, `outage_keep_s` | 50, 3600 | Default outage retention share and lifetime. |

The two-item writer queue drops excess snapshots. Oldest unprotected snapshots are evicted first. A snapshot-branch error restarts the live pipeline. Pi JPEG cost is unmeasured.

### Positions and outage snapshots

Snapshots include drone position only when telemetry is at most 2 s old. Missing relay feedback marks outage snapshots. The newest marked snapshots remain protected within the outage share and lifetime; successful retrieval releases their mark.

### Listing outage snapshots

`just detail-outage` lists up to 16 marked snapshots oldest first. `detail outage --from-ms <unix-ms>` sets the capture-time boundary. Fetched images report position through `X-Position` when available.

### Sightings

Detections during a link outage record kind, time, drone position, box and score. Position must be fresh. Same-kind observations merge within 2 s and 0.15 picture fraction; the newest 256 entries remain.

On reconnection, QGC receives informational notifications, at most one per 500 ms. Coordinates locate the drone. `just sightings` lists up to 16 entries, including announced ones. `detail sightings --from-ms <unix-ms>` sets the first-seen boundary. Each can name a retained snapshot within 1.5 s of capture; fetch it with `detail request --at-ms`.

### Job API

| Field/bound | Limit |
|---|---|
| `kind` | `snapshot`, `crop`, `outage`, `sightings` |
| `at_capture_ms` | Requested capture time. |
| `crop: {x,y,w,h}` | Picture fractions inside 0 to 1, with positive area. |
| `max_bytes` | 1 B to 1 MiB; 2 MiB across at most 4 jobs. |
| `expires_in_ms` | 1 s to 10 min. |
| JSON / chunk | 4 KiB / 16 KiB. |
| `[retrieval] allowance_bytes_per_s` | 4000 in `testbed/limits.toml`; 0 disables sending. |

`POST /jobs` returns `{job_id}`. `GET /objects/{id}` returns 202 while pending, 410 on failure, or the object. `DELETE /jobs/{id}` cancels it and frees its slot. Retrieval sends one chunk at a time, with at most three attempts per chunk. It waits during drains, stale feedback, excess queue delay or non-`Normal` rungs. A crashed worker's assigned job occupies its slot until cancellation or expiry.

### Requesting detail

```sh
just detail-request 10
just detail-request-at <capture-ms>
just detail-fetch <job-id> frame.jpg
just detail-outage
just sightings
```

CLI: `thinlink-ground detail request (--ago-s <n> | --at-ms <unix-ms>) [--crop x,y,w,h] [--max-bytes <n>] [--expires-s <n>] [--relay <url>]`. `detail fetch <id> -o <file>` waits, labels the JPEG and frees the job; `detail cancel <id>` cancels it. Default relay: `http://192.168.232.20:8080`; token: `--job-token-file`. `--ago-s` requires agreement with the vehicle's capture clock.

## Regional video

Pilot and detector regions receive encoder quantiser offsets. Outlines identify detections; following tracks a box without moving the drone. Boxes use picture fractions across resolution changes.

| Setting | Value/limit |
|---|---|
| Vehicle `[region] roi_qp_offset` | -51 to 0; default 0, testbed regional configs -6. Pilot regions require a negative value. |
| Limits `[region] pilot_refresh_ms`, `pilot_ttl_ms` | 500, 3000 ms. Lease changes clear the box. |
| `detection_ttl_ms` | 1000 ms. |
| `max_regions`, `max_area_percent` | 4 regions; none selected above 50 % coverage. |
| `detector_fps`, `detector_min_score` | 2 fps, 0.4. |
| `follow_fps`, `follow_min_score`, `follow_lost_frames` | 10 fps, 0.5, 5 frames. |

Limits are in `testbed/limits.toml`.

### Detector

`just model` downloads and verifies `models/person.onnx`. Vehicle `[region] detector` enables NanoDet-Plus-m 320. Its `model`, `input` and `kinds` configure inference. `input` accepts 32-1280 pixels. `kinds` must be nonempty, defaults to `["person"]` and accepts person, bicycle, car, motorcycle, bus, truck or boat. Slow inference drops detector frames. Load failure reports `detector.state = "failed: <cause>"` while video continues.

Outlines remain with offsets off: amber P, green B, cyan C, magenta M, red U, white T and deep blue S. Following uses pink F.

### Following a target

Set `[region] follow = true`, then draw a box. It snaps to a detection with over half overlap and tracks the selected pixels. Updates reach QGC at most four times a second. A low score for `follow_lost_frames`, an edge exit, or an unsuitable starting box ends tracking. Draw again to restart.

### Detail inset

Tracking requests a crop every 4 s, one job at a time, widened 10 % per side. It appears top-right below the stamp rows, at most one-third picture width, labelled `DETAIL <age> s`. Display requires `LIVE` or `LOW RATE` and crop age under 30 s. A new box, stop or tracking loss clears it.

### Drawing a box

Run `just take-control`, then draw a rectangle in QGC video. `MAV_CMD_CAMERA_TRACK_RECTANGLE` and `MAV_CMD_CAMERA_TRACK_POINT` select; `MAV_CMD_CAMERA_STOP_TRACKING` stops. Selections below 1 % of either dimension are ignored. Ground needs `--job-token-file` for crops.

The red box becomes active with a crop or a confirmed encoder region. Confirmation expires after 3 s; without confirmation or a crop for 30 s, tracking becomes idle. A busy link can delay or prevent crops.

## Landing on a marker

Vehicle `[landing] target = true` enables tag36h11 detection; default is off. Duplicate IDs in one frame and bit errors are rejected.

| Key | Default | Limit |
|---|---:|---|
| `tag_id` | 0 | Marker ID to detect. |
| `tag_fps` | 10 | 1-15 frames/s. |
| `tag_width` | 640 | 160-1920 pixels. |

Set `[camera] hfov_deg` to at least 1 and below 180 degrees. For a fixed camera set `tilt_deg = -90`, with picture top toward the nose. A gimbal requires downward orientation and picture rotation within 5 degrees, reported within 500 ms of the sighting. Targets expire 500 ms after capture. Distance uses fresh positive height above home and a flat-ground assumption. `PLND_LAG` must match the unmeasured camera-to-message delay.

In QGC, select Land over the tag. ArduPilot controls the descent. Lost or incorrectly oriented sightings stop target messages. Mode changes take over. [Real mode](real-mode.md#landing-on-a-marker) adds configuration requirements. Hardware behavior is unmeasured.

## Missions

Set vehicle `[mavlink] allow_missions = true` and run `just take-control`. In QGC Plan, create the mission and Upload; in Fly, select Start. Progress returns as telemetry. Upload, Start and Auto are denied without the switch or lease.

| Limit | Value |
|---|---|
| Vehicle / mission type | Copter / waypoint mission; fence and rally uploads refused. |
| Items | At most 48. |
| Commands | Waypoint, takeoff, land, RTL, timed/counted loiter, speed change, delay. |
| Location | Relative to home, above 0 through 120 m; landing permits 0. |
| Item 0 | Waypoint home placeholder; unrestricted frame/altitude. |
| Hold / turns | At most 300 s / 10 turns. |
| `current` | At most 1; `MISSION_SET_CURRENT` is dropped. |

QGC waits 1500 ms per request or acknowledgement. A six-item mission takes about seven round trips. See [mission checks](testbed.md#mission-check).

## SITL

The rover uses model `rover`, instance 0, `--speedup 1`, at 59.3293 N, 18.0686 E, 10 m, heading 0. System/component ids are 1. It waits on TCP 5760. Parameters and logs live in `/var/lib/sitl`; each `lab-up` starts with defaults.

## Simulated flight

`just sim-up` runs ArduCopter 4.7.2 in Gazebo Harmonic. Camera and controls traverse the emulated link.

```sh
just sim-up
just impair geo
just take-control
```

QGC usually discovers the copter and camera after about two minutes. Complete its vehicle Preferences dialog if shown. `just lab-up` starts the rover testbed. To build simulation images alone, run `just lab-down`, then `just sim-image`.

### Flying from QGC

Enable `virtualJoystick`, or configure and calibrate a gamepad in Vehicle Setup, Joystick. Leave "Center stick is zero throttle" off and use the default 25 Hz rate. Without stick input, arming reports "Throttle below failsafe".

Takeoff in Fly selects Guided, arms and climbs. Land, Return and Pause use mode changes. Throttle at the bottom in flight triggers the radio failsafe, which brakes the copter; Land descends. `just release-control` ends the lease.

### Command gate

| Vehicle | Admitted modes |
|---|---|
| Copter | Stabilize 0, AltHold 2, Guided 4, Loiter 5, RTL 6, Land 9, PosHold 16, Brake 17; Auto 3 with `allow_missions`. |
| Rover | Manual 0, Hold 4, RTL 11. |

Mode changes use `COMMAND_LONG` `MAV_CMD_DO_SET_MODE`; `SET_MODE` and state-changing `COMMAND_INT` are refused. Stabilize, AltHold, Loiter and PosHold require fresh sticks. Takeoff altitude must be finite, above 0 and at most 120 m. Refusals appear under `mavlink.control.rejected`.

### Control loss

`testbed/thinlink-copter.parm` defines behavior. Lost stick input in stick modes selects Brake. Guided and Land ignore stick loss. Resume input and change mode to continue. Lost lease or adapter selects Land after the 2 s GCS timeout. Auto uses `FS_OPTIONS 15` to continue through link loss. Timing depends on simulation speed; [simulated-flight checks](testbed.md#simulated-flight-check) grade it.

### Flying missions

```sh
just sim-up vehicle-sim-missions.toml
just take-control
```

Use QGC Plan, Upload, then Fly, Start. `just sim-mission <profile> on|off` runs the mission check. Continuation is measured for link cuts inside the configured fence; fence breaches are unmeasured.

### Regional simulation

`just regional=1 sim-up` loads `testbed/config/vehicle-sim-regional.toml`: `kinds = ["person", "car"]`, `roi_qp_offset = -6`, `follow = true`, and a 4 MiB snapshot store. The scene contains people, a parked car and a landing pad.

### Simulated camera

The gimbal renders 640x360 at 15 fps, looking 20 degrees down. `MNT1_DEFLT_MODE 1` and `MNT1_NEUTRAL_Y -20` set that position. Plugin x264 uses 800 kbit/s and keyframes every 10 frames. The vehicle ingests, decodes and stamps it; label age excludes rendering and plugin encoding.

`just sim-landing <on|off|forward> <profile>` selects downward targets, targets disabled, or forward-facing targets. Files: `testbed/thinlink-copter-camera-down.parm`, `testbed/thinlink-copter-precland.parm`; simulated `PLND_LAG` is 0.15.

### Simulation resources

Gazebo uses CPU rendering. The simulation uses roughly 1.5 GB; Gazebo's `mem_limit` is 1100 MB. Stop it with `just lab-down` before building or running other heavy containers. A loaded host can slow simulation timing.

### Simulation provenance and licences

Everything is baked into the images; nothing is fetched at runtime.

- ArduCopter 4.7.2, `copter.parm` and `gazebo-iris.parm` come from ArduPilot (GPL-3.0), commit `fb773567896014da007bca58358704d4a0d8516f`, with URL and SHA-256 pins.
- Gazebo Harmonic, gz-sim 8.15.0 and its libraries (Apache-2.0), come from packages.osrfoundation.org for Ubuntu 22.04. Versions and the archive key's hash are pinned.
- ardupilot_gazebo (LGPL-3.0) is commit `f116b03552d05c8d0b0bed5ed67033a1183121d1` from a checksummed tarball. It supplies the plugin, iris and gimbal models, `gazebo-iris-gimbal.parm` and runway. Those model files name authors but state no separate licence, except the unused `parachute_small` (Apache-2.0).
- The world in `testbed/sim/world.sdf` derives from `iris_runway.sdf`. Project scenery uses boxes and cylinders. `testbed/sim/models/field` reuses the airfield mesh with a plain green material. The runway is the only texture used from ardupilot_gazebo.
- Standing person, Casual female and Hatchback version 3 are OpenRobotics Gazebo Fuel models under CC0 1.0, fetched by version and hash in `testbed/Containerfile.sim`.
- The container build applies two checked patches: start camera streaming on the first frame, and set the gimbal camera's size, rate and destination.
- Ubuntu packages installed with `apt` float with the base image; only Gazebo packages are version-pinned. This is a bench image.

## Mac camera demo

`just demo <profile>` sends the Mac camera through the emulated link to QGC. The autopilot is the independent simulated rover.

### Running the demo

Install ffmpeg with `brew install ffmpeg` and use the [development tools](testbed.md#development-setup).

```sh
just demo-cameras
just demo lan
just demo geo 1
just demo-stop
```

Camera defaults to 0; `CAMERA=1 just demo geo` also selects 1. Profiles include `geo`, `lte-good`, `lte-bad`, `geo-degraded` and `outage-10s`. UDP 5700 must be free. A capture failure leaves the testbed up; `just demo-stop` stops both. `just regional=1 demo geo` enables person detection and offset -6.

### Camera permission on macOS

Allow camera access for the terminal. Change it in System Settings, Privacy & Security, Camera. A refusal or camera held by another app produces a no-frames error with ffmpeg's message.

### Watching the demo

Start QGC with Low Latency Mode. Expect a few milliseconds of stamp age on `lan`, about 0.33 s on `geo`. Impairment can lower quality or produce `LOW RATE`, `STALE` and `NO VIDEO`.

### Showing each label

Use [display hooks](#checking-the-display). Covering the lens stays live; ingest cannot identify repeated unstamped images. `kill $(cat .cache/demo-camera.pid)` stops capture alone. `just impair geo` changes the running profile.

### Driving with the demo

Follow [Taking control](#taking-control) to operate the independent rover. Its observed response under `geo` is roughly 0.6 s later.

### Camera ingest

Mac ffmpeg captures 1280x720 at 30 fps and sends RTP/H.264 to `127.0.0.1:5700`. Encoding uses x264 `ultrafast`, `zerolatency`, 3 Mbit/s, no B-frames, 1200-byte packets and keyframes every 0.5 s. Capture and encoding precede the stamp. `just ingest-check lan` exercises the port with a test pattern.

### Other ingest senders

Vehicle `[source] from` lists accepted sender addresses. Send RTP/H.264 payload type 96 without B-frames to `127.0.0.1:5700`. Ingest has no jitter buffer or keyframe requests. After packet loss it drops damaged pictures until the next keyframe. Port mapping is in `testbed/compose.yml`; filtering is in `testbed/scripts/ingest.nft`.
