# Drone and operator use

The drone adds MAVLink control, retained detail, regions, detections and landing targets to the [video path](video.md). QGroundControl is the ground station. The ground adapter is its only command endpoint. The vehicle owns the control lease and validates commands.

The default testbed commands ArduRover or ArduCopter SITL only. Real mode is never the default, has never flown and requires the interlocks in [Real vehicle](real-vehicle.md). No test or demonstration authorizes commands to real actuators. Deployment and camera hardware are covered in [Hardware](hardware.md).

## QGroundControl

QGroundControl 5.1.5 runs natively on the Mac. With AutoConnect to UDP on (Application Settings, General, on by default), it listens on UDP 14550 and answers the MAVLink sender. No Comm Link or published MAVLink port is needed. Nothing else on the Mac may hold UDP 14550.

The ground adapter sends telemetry to `host.containers.internal:14550` and reads replies on the same socket. In a Podman machine this address is `192.168.127.254`, gvproxy's gateway. It forwards datagrams to the Mac's `127.0.0.1:14550` and maps replies back. The adapter sees the replies from `192.168.127.254:14550`.

```text
QGC :14550 <-> gvproxy <-> ground adapter <-> 10.77.0.3:14551
  <-> mavlink-router 10.77.0.1 <-> companion (gate, telemetry budget) <-> SITL
ground adapter 10.77.0.3:7701 -- control envelopes --> companion 10.77.0.2:7700
```

If AutoConnect is off, add a UDP link in Application Settings, Comm Links, port 14550, no server address, then connect it.

The adapter announces its freshness output as the vehicle's camera. QGC selects "UDP h.264 Video Stream" and plays it. Keep the UDP URL at `0.0.0.0:5600`. Turn on Low Latency Mode in Application Settings, Video. Otherwise QGC adds an RTP jitter buffer (80 ms by default) and waits for presentation times at its video sink. The adapter runs next to QGC, so this local stream does not need that buffer.

The output goes to `host.containers.internal:5600`, delivered to the Mac's `127.0.0.1:5600`. Its labels and their limits are defined in [Video freshness](video.md). A stamp measures age from the vehicle's stamp point. It does not measure camera capture or encoding before that point.

If the adapter or its output dies, QGC holds the last picture and its last label until its own watchdog stops the receiver. QGC 5.1.5's UDP timeout is 3 s, checked once a second in whole seconds. It shows "WAITING FOR VIDEO" about 4 to 5 s after the last frame and reconnects after 1, 2, 4 and up to 30 s. A dead adapter also stops MAVLink. If only the output pipeline fails, the adapter restarts it after a second. These behaviors come from QGC's `VideoManager.cc`, `GstVideoReceiver.cc`, `FlightDisplayViewVideo.qml` and `Video.SettingsGroup.json` at tag v5.1.5.

### Camera discovery

The adapter uses the vehicle's system id and component 100 (`MAV_COMP_ID_CAMERA`). QGC discovers camera components 100 to 105 after the vehicle's initial connection. It asks for `CAMERA_INFORMATION` with `MAV_CMD_REQUEST_MESSAGE`, alternating with `MAV_CMD_REQUEST_CAMERA_INFORMATION` on retries. With `CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM`, it asks a second later for `VIDEO_STREAM_INFORMATION` of all streams (id 0). It also asks for `CAMERA_SETTINGS`, `STORAGE_INFORMATION`, `CAMERA_CAPTURE_STATUS` and `VIDEO_STREAM_STATUS`, giving up after five retries without showing errors.

The adapter advertises `CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM`, `CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE` and `CAMERA_CAP_FLAGS_HAS_TRACKING_POINT`. It answers camera information, stream information and stream status with `COMMAND_ACK` `ACCEPTED` and the requested message. Stream 1 is "overview", `VIDEO_STREAM_TYPE_RTPUDP`, running, with the output's resolution, frame rate and bitrate, and URI `udp://0.0.0.0:5600`. Tracking commands and status are described under [Regional video](#regional-video). Other camera commands get `MAV_RESULT_UNSUPPORTED`. These messages stay at the adapter and never reach the vehicle.

For an RTP/UDP stream whose encoding is not H.265, QGC accepts a URI containing `udp://`, or builds `udp://0.0.0.0:<uri>` from a port alone. The `mavlink` crate 0.21 definitions omit the encoding field, so it arrives as 0 and QGC uses H.264. QGC uses type, URI, stream id and count, flags and resolution. It sets Video Source to "UDP h.264 Video Stream" when it discovers the camera, then uses its UDP URL setting. It prefers the camera again whenever it reconsiders the source. The adapter never sets `VIDEO_STREAM_STATUS_FLAGS_THERMAL`: QGC uses that flag for a thermal overlay, not its main video.

Any message from the vehicle's system id keeps QGC's link alive, regardless of component. It declares Communication Lost after 3.5 s without one. The camera heartbeat therefore follows each vehicle heartbeat, and the camera answers only within 1.2 s of the vehicle's last message. It can delay Communication Lost by at most that interval. QGC drops a camera silent for 5 s. The UDP video source stays selected and the camera is rediscovered when the vehicle returns. These discovery and liveness rules are in QGC 5.1.5's `QGCCameraManager.cc`, `VehicleCameraControl.cc`, `VideoManager.cc` and `VehicleLinkManager.h`.

### Direct stream viewing

MediaMTX's direct RTSP stream skips the adapter's decode and re-encode and has no freshness indication. A stalled picture stays visible until QGC's RTSP watchdog expires (Video Timeout, 8 s by default). A repeating stream can look live.

To use it in QGC, remove `camera_uri` from `testbed/config/ground.toml`, run `just lab-up`, then select "RTSP Video Stream" with `rtsp://127.0.0.1:8554/live` and Low Latency Mode on. Removing the URI stops the camera announcement; the freshness output still runs.

RTSP from the Mac must use TCP. MediaMTX otherwise sends UDP media to Podman's port forwarder, where it cannot reach the Mac. QGC's GStreamer source tries UDP first and switches to TCP after its timeout, so the picture can take a few seconds to appear.

```sh
ffprobe -v error -rtsp_transport tcp -show_streams -select_streams v rtsp://127.0.0.1:8554/live
ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/live
```

With `-rtsp_transport udp`, the session opens but no frame arrives. The optional browser view is MediaMTX's WebRTC player at `http://127.0.0.1:8889/live`. Use Chrome Canary with an isolated profile. Its ICE media uses published `127.0.0.1:8189/udp`; `testbed/mediamtx.yml` offers `127.0.0.1` as a candidate.

### Checking the display

1. Run `just lab-up`, start QGC and wait for the vehicle. The ball pattern should have a `LIVE` label and an age near 0.01 s on `lan`.
2. `just video-hook pause` stops the vehicle's video. Expect `STALE` after about 0.6 s, then `NO VIDEO` after about 2 s, with the last picture kept. `just video-hook resume` restores it.
3. `just video-hook freeze` repeats one stamped frame. Expect `FROZEN` after about 0.5 s. Resume afterwards.
4. `just impair geo` applies the GEO-like profile. Healthy frame age is about 0.33 s.

The rover, webcam and simulated camera can render in QGC 5.1.5 on macOS. Switching between rover and copter can show its first-run Preferences dialog for firmware, vehicle and units; answer it once. Regional tracking and marker landing have not been checked in QGC on screen. Container checks, native QGC rendering and optical glass-to-glass measurements remain separate evidence.

## Taking control

1. Run `just lab-up`, start QGC and wait for the vehicle to load.
2. `just take-control` asks the adapter for the lease. It prints its status, or `409` with `lease_held` if another controller holds it. `just control-status` shows the status at any time.
3. Choose Manual for the rover and arm. QGC sends `MAV_CMD_DO_SET_MODE` with param 1 `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` and param 2 the vehicle mode number. `MAV_CMD_COMPONENT_ARM_DISARM` uses param 1 = 1 to arm or 0 to disarm; QGC Force Arm sets param 2 = 2989. For copter modes and joystick setup, see [Flying from QGC](#flying-from-qgc).
4. Enable a joystick or virtual thumbsticks. QGC sends `MANUAL_CONTROL` at 25 Hz by default. ArduRover uses y for steering and z for throttle, as RC override from ground-station system id 255.
5. Disarm, then `just release-control`.

A link loss longer than the 3 s lease lifetime ends the lease. Commands are denied and nothing is replayed after reconnect. Take control again explicitly. [Control lifetimes](#control-lifetimes) defines setpoint expiry and the rover's stop behavior.

## What QGC can and cannot do

The adapter is QGC's only command endpoint. Read-only traffic passes without a lease. `SYSTEM_TIME` is the exception that writes the autopilot's real-time clock, but changes no flight state.

State changes need the lease and the vehicle's command gate. Without the lease, commands receive `MAV_RESULT_DENIED`; heartbeats and `MANUAL_CONTROL` are dropped and counted. Unsupported commands receive `MAV_RESULT_UNSUPPORTED`. The vehicle's own acknowledgement returns as telemetry after an applied command. [Missions](#missions) also require `allow_missions`. Camera commands stay at the adapter.

## What QGC 5.1.5 sends when it connects

Read from QGC 5.1.5's source (`InitialConnectStateMachine`, `ParameterManager`, `Vehicle`, `APMFirmwarePlugin`, `FTPManager`):

- Admitted raw: `REQUEST_MESSAGE` for `AUTOPILOT_VERSION`, `AVAILABLE_MODES` and component metadata; `MISSION_REQUEST_LIST` for the mission, fence and rally points, then `MISSION_REQUEST_INT` and `MISSION_ACK`; `PARAM_REQUEST_LIST` and `PARAM_REQUEST_READ`; `REQUEST_DATA_STREAM`; `SYSTEM_TIME`; `TIMESYNC`. `HEARTBEAT` needs the lease and travels as liveness in an envelope.
- Dropped: `FILE_TRANSFER_PROTOCOL`. For ArduPilot, QGC first asks for the parameter file `@PARAM/param.pck?withdefaults=1` over MAVLink FTP, retries once, then falls back to `PARAM_REQUEST_LIST`. FTP stays dropped: the same message also creates, writes, renames and removes files on the vehicle, and QGC loads every parameter without it. The cost is QGC's FTP timeouts before the parameter download starts.
- Sent only on a user action or when the vehicle asks. With the lease they travel in envelopes: flight-mode changes (for ArduPilot always `MAV_CMD_DO_SET_MODE`, because its firmware plugin says it supports the command; the `SET_MODE` message only for firmware that does not), arming and disarming (`MAV_CMD_COMPONENT_ARM_DISARM` as `COMMAND_LONG`, never retried: QGC retries only `REQUEST_MESSAGE`, `REQUEST_AUTOPILOT_CAPABILITIES`, `PREFLIGHT_STORAGE` and `RUN_PREARM_CHECKS`, and increments `confirmation` when it does), and `MANUAL_CONTROL` from a joystick or the virtual thumbsticks. Dropped with or without it because they change the vehicle: `PARAM_SET`, `MISSION_SET_CURRENT`, fence and rally uploads (mission uploads travel in envelopes with the switch on, [Missions](#missions)), `RC_CHANNELS_OVERRIDE` (only for extra joystick axes, if so configured), `SET_GPS_GLOBAL_ORIGIN`, guided-mode targets, `TERRAIN_DATA` (answers to the vehicle's `TERRAIN_REQUEST`, which loads terrain into it) and Remote ID (`OPEN_DRONE_ID_*`) for a vehicle that has it. QGC also answers a vehicle's `PING`, which would be dropped too.

Read from QGC 5.1.5's source: `Vehicle::setFlightMode`, `Vehicle::setArmed` and `Vehicle::forceArm` (`src/Vehicle/Vehicle.cc`), `APMFirmwarePlugin::MAV_CMD_DO_SET_MODE_is_supported` (`src/FirmwarePlugin/APM/APMFirmwarePlugin.h`), `MavCommandQueue::_shouldRetry` and the `confirmation` field (`src/Vehicle/MavCommandQueue.cc`), `Vehicle::sendJoystickDataThreadSafe` and `axisFrequencyHz` (default 25, `src/Settings/Joystick.SettingsGroup.json`). QGC accepts a `COMMAND_ACK` only from the vehicle's system id and matches it by the sender's component and the command, so the adapter's own answers come from the command's target system and component.

## Control protocol

Default `mode = "sitl"` sends control only to identified ArduRover or ArduCopter SITL. Otherwise frames go to the logging sink. Explicit `mode = "real"` requires [separate interlocks](real-vehicle.md) and has never flown. `identity::failures`, wrapped by `identity::control_target`, is the one destination decision.

SITL identification needs a vehicle-type heartbeat and `SIM_WIND_SPD` from that heartbeat's system and component. ArduPilot 4.7.2 registers the `SIM_` group under `AP_SIM_ENABLED`, enabled for SITL and ChibiOS builds linked with simulation. The bridge asks once a second until answered. A disconnect or different heartbeat address/type clears the identity.

Without identification, liveness and setpoints are logged and counted as `sunk`. A discrete command gets `Rejected` with `NoDestination`; the lease stays held and QGC receives a denied `COMMAND_ACK`. Read-only traffic still passes.

### Identity and authentication

One controller at a time holds the lease. Its identity is its overlay source address; `peer` in a request is informational. The vehicle accepts requests only from `controllers`. In `testbed/config/vehicle.toml` these are the ground adapter at 10.77.0.3 and lab runner at 10.77.0.4. Other sources get `not_controller` before their message is read. Every envelope must come from the holder's address or gets `not_holder`.

The relay attests those addresses. The vehicle's only WireGuard peer is the relay, with `AllowedIPs = 10.77.0.0/24`; the relay checks each client against its own `/32`. A compromised relay can pose as the ground. Every envelope, lease request and release therefore needs the controller's Ed25519 signature (`ed25519-dalek`). The vehicle holds one public key per controller address. The relay holds neither public nor private control keys. Public keys are read at startup; changes take effect after restart.

Signed data starts with its message-kind constant (`sat-teleop/envelope/v1`, `sat-teleop/lease-request/v1` or `sat-teleop/lease-release/v1`) and `vehicle_id`. The vehicle and ground must configure the same id. Signatures cannot be reused for another vehicle or message kind. An envelope signs every field except the signature itself, including challenge and sequence. A request signs its nonce; a release signs the vehicle epoch and lease.

Missing signatures get `unsigned`. A wrong key, changed field, wrong vehicle id or unreadable controller key gets `bad_signature`. A request returns `lease_denied`; a release returns `rejected`. Signature verification precedes policy checks and lease changes. A request nonce is the ground's millisecond clock and must exceed the last accepted from that controller, or gets `replayed_request`. This remembered nonce is cleared by a vehicle restart.

Challenge tokens are a per-session counter and prove freshness only. Vehicle replies, telemetry and video are unsigned. A compromised relay can forge them, but they do not authorize vehicle motion. Any listed controller can ask for the clock bound without a lease. The ground does so every 2 s with `ClockQuery`, answered by `Clock`. A forged answer could make the ground believe a small clock bound, just as a forged video could mislead it. See [Clock bounds](video.md#frame-age-and-clocks).

The adapter's `POST /lease`, `DELETE /lease`, `GET /status`, and every application's `GET /metrics` require bearer authentication when a token is configured. Tokens are compared in constant time by `sat_teleop_video_core::token`. The service files are `vehicle.token`, `ground.token` and `job.token`. The ground token stays on the ground; the relay holds only the job token. Without a token an endpoint binds to 127.0.0.1 on the configured port. Deployment metrics also bind to loopback. The job API requires its token to start.

### Command paths

Raw traffic crosses mavlink-router and the vehicle's read-only allowlist, lease or not. The allowlist is [below](#mavlink-path). All state changes use signed envelopes from the lease holder:

- Liveness carries a GCS `HEARTBEAT`, or no frame to renew only the lease. The adapter never invents a station heartbeat.
- Continuous carries the latest `MANUAL_CONTROL` setpoint.
- Discrete carries a transaction id and an allowed `COMMAND_LONG`: `MAV_CMD_COMPONENT_ARM_DISARM`, `MAV_CMD_DO_SET_MODE`, `MAV_CMD_NAV_TAKEOFF` or `MAV_CMD_REQUEST_MESSAGE`. Missions additionally admit `MISSION_COUNT`, `MISSION_ITEM_INT`, `MISSION_CLEAR_ALL`, `MAV_CMD_MISSION_START` and copter Auto, under [their switch and gate](#missions).
- Region carries a box, never MAVLink or an autopilot command.

### Envelope protocol

Messages are JSON, one per UDP datagram of at most 1200 bytes (larger ones are refused unread):

| Direction | Message | Meaning |
|---|---|---|
| ground to vehicle | `lease_request { peer, nonce, sig }` | Ask for the lease. Granted unless another controller holds an unexpired one; asking again gives a new lease. |
| | `lease_release { lease, sig }` | End it (holder only). |
| | `envelope { epoch, lease, seq, challenge, kind, mavlink, region, sent_unix_ms, sig }` | `kind` is `liveness`, `continuous`, `{ discrete: { txn } }` or `region`, and `seq` counts per kind; `mavlink` base64 of one MAVLink 2 frame when present; a region envelope carries only its optional box; `sent_unix_ms` for measurement only. |
| vehicle to ground | `lease_grant { nonce, lease, epoch }`, `lease_denied { nonce, reason }` | The answer to a request. |
| | `challenge { epoch, token, lifetime_ms }` | To the holder at once after a grant, then every 500 ms while the lease lasts. |
| | `command_result { txn, outcome }` | For every discrete envelope: `applied` (written to SITL; SITL's `COMMAND_ACK` follows in the telemetry), `repeated` (a retry of a transaction already applied, not written again), `unsupported` or `{ rejected: { reason } }`. |
| | `rejected { seq, reason }` | Any other refused envelope, release or unreadable datagram. |

The first failed check refuses an envelope without applying it. Checks run in this order:

1. Datagram size, JSON, signature (`unsigned`, `bad_signature`), frame (`malformed`) and kind's allowlist (`not_allowlisted`, or discrete `unsupported`).
2. Session epoch (`wrong_epoch`), lease and holder (`no_lease`, `not_holder`, `wrong_lease`, `lease_expired`). The epoch is the vehicle's start time.
3. One of the four newest challenges (`unknown_challenge`), within its lifetime (`challenge_expired`).
4. Strictly increasing sequence per lease and kind (`duplicate_seq`, `reordered_seq`). Separate counters let a stop command pass even if a later setpoint overtakes it.
5. Discrete transaction identity (`txn_conflict` if a known id names a different command).

Freshness uses challenge age on the vehicle's monotonic clock, never a ground timestamp. Admitted envelopes renew the lease. Permitted MAVLink goes to the selected autopilot at once; a region updates only its region state. The newest continuous setpoint repeats every 200 ms without a replacement, until expiry.

A discrete transaction runs at most once per lease. An identical retry gets `repeated` without another write, and the adapter answers QGC `ACCEPTED`. A station resend with MAVLink `confirmation` above 0 reuses the first transaction within `ack_window_ms`. Lease changes clear that memory. Stale commands are refused, never queued or replayed after a reconnect, restart or blackout.

The adapter returns `DENIED` without a lease or fresh challenge, and on a vehicle rejection. A command no envelope permits gets `UNSUPPORTED`. Applied commands are acknowledged by the autopilot. The adapter never acquires a lease automatically, including after a restart.

### Control lifetimes

`[control]` in `testbed/limits.toml` is validated by `sat-teleop-control`. The configured GEO round trip is about 600 ms.

| Limit | Value | Purpose |
|---|---|---|
| Challenge interval | 500 ms, fixed | Bound the gap between fresh challenges. |
| `challenge_lifetime_ms` | 1500 | Challenge interval + GEO round trip + 400 ms jitter/stall margin. |
| `setpoint_lifetime_ms` | 500 | Bound the 200 ms refreshes; also capped by challenge expiry. |
| `liveness_interval_ms` | 1000 | Send empty liveness if nothing else was sent; a station need not remain connected to hold the lease. |
| `ack_window_ms` | 4000 | Cover QGC's retried commands at 1200 ms intervals, three tries, plus GEO RTT and margin. |
| `lease_lifetime_ms` | 3000 | Two missed liveness intervals and margin; reacquisition is explicit. |
| `throttle_release_deadline_ms` | 1825 | Rover check deadline, described in [Control freshness check](testbed.md#control-freshness-check). |

`testbed/sat-teleop.parm` loads after `rover.parm`. `RC_OVERRIDE_TIME 0.5` ends override 0.5 s after the last refresh. `FS_GCS_ENABLE 1`, `FS_GCS_TIMEOUT 2`, `FS_TIMEOUT 1` and `FS_ACTION 2` select Hold after no GCS heartbeat or `MANUAL_CONTROL` for 2 + 1 s. The vehicle must stop refreshes by the setpoint lifetime plus one 20 ms tick. The rover then coasts without brakes. ArduPilot parameter names and ranges are pinned to commit `fb773567896014da007bca58358704d4a0d8516f`.

### A filled uplink under `geo`

Control and video share the uplink's FIFO (512 kbit/s under `geo`). A queued packet delays control too. Challenge age at the vehicle includes 300 ms down, up to 500 ms waiting for the next challenge, 300 ms up, queue delay and jitter. That is 600 to 1100 ms before the queue. At the worst phase, about 400 ms of queue consumes the 1500 ms lifetime. Refusals can appear with a sampled queue near 200 ms because probes sample every 250 ms and miss a peak.

Queue time counts in command age. The vehicle accepts only a challenge at most `challenge_lifetime_ms` old. [Pacing](video.md) keeps excess video on the vehicle. Initial probing without measured capacity, a route change or a real capacity drop can still build a queue before feedback arrives. Those envelopes get `challenge_expired` and are never delayed for later execution. More than 0.5 s of refused setpoints brakes the copter; 2 s without liveness triggers Land.

### Control timing

Envelopes carry `sent_unix_ms`, the ground's wall clock; the vehicle records it with its own arrival and write times for the last continuous and discrete envelope, and `lab check control-freshness` reports discrete acknowledgement times and continuous setpoint delay per profile. The testbed's containers share one clock ([Video timing](video.md)); across machines this needs a clock mapping.

`clock_query { nonce }` asks for the clock bound; `clock { nonce, bound_ms }` answers it. `region { region }` confirms the applied box and `lost { region_seq }` reports a lost follow. These vehicle replies are unsigned. A missing `region` clears the pilot box.

## MAVLink path

`mavlink-router` v4 runs in the relay namespace with `testbed/mavlink-router.conf`. Its `vehicle` UDP server binds 10.77.0.1:14550 and replies to its latest sender. Its `ground` client sends to 10.77.0.3:14551 from its own port. In v4 that client accepts any sender and follows the latest for 5 s. Its `checks` client sends telemetry to 10.77.0.4:14553 so checks can observe SITL without the adapter. TCP service is off and message filters are at their defaults, none. The service restarts if it cannot bind before `wg0` is ready.

```text
SITL 127.0.0.1:5760 <-> companion <-> mavlink-router 10.77.0.1:14550
  <-> ground 10.77.0.3:14551 <-> QGC host.containers.internal:14550
                            <-> checks 192.168.232.40:14552
```

`companion` connects to SITL on TCP 5760, as configured in `[mavlink]` of `testbed/config/vehicle.toml`. Reconnect backoff doubles from 0.5 to 5 s. Five seconds without data counts as a hang. Telemetry uses one MAVLink frame per datagram and the `ardupilotmega` dialect, which includes `common`. An id outside the dialect fails CRC validation and is not forwarded; compare `sitl_bytes` with `telemetry.bytes_in`.

The [shared uplink budget](video.md#shared-uplink-budget) sets the telemetry cap from `[telemetry]` in `testbed/limits.toml`. The one-second bucket always admits autopilot heartbeats, spending tokens when available. State (`SYS_STATUS`, `GLOBAL_POSITION_INT`, `ATTITUDE`, `COMMAND_ACK`) can use the whole bucket. Replies and bulk use only its upper half. Replies (`AUTOPILOT_VERSION`, `PARAM_VALUE`, mission download) wait in a 64 KiB, 10 s queue; bulk drops while replies wait. ArduPilot's 1256 parameters arrive as about 46 KB in 0.6 s, and take about 6 s to forward. Reconnect clears queued replies.

The vehicle accepts raw datagrams only from 10.77.0.1:14550. Its allowlist is parameter and mission downloads, stream-rate requests, time sync, and `COMMAND_LONG`/`COMMAND_INT` with `REQUEST_MESSAGE`, `SET_MESSAGE_INTERVAL`, `REQUEST_PROTOCOL_VERSION` or `REQUEST_AUTOPILOT_CAPABILITIES`. Every other raw message, including GCS heartbeats, is dropped and counted in `mavlink.gcs_refused`. Commands use [the separate envelope endpoint](#control-protocol), with no reconnect replay.

The ground binds 10.77.0.3:14551, retrying each second until available. It accepts telemetry only from mavlink-router's address and learns its sending port. Each configured `gcs` station gets its own UDP socket, accepting replies only from that station. Camera messages stay local. Read-only traffic goes to mavlink-router; the rest goes through the controller on 10.77.0.3:7701.

`testbed/config/ground.toml` configures the adapter. `testbed/limits.toml` supplies operational limits; unknown keys in a table an application reads are rejected. `just take-control`, `just release-control` and `just control-status` call `sat-teleop-ground take-control | release-control | status` in the ground container, against its `ctl_net` HTTP endpoint.

## Predictive display

The ground draws a hollow white dashed ring where the camera centre is predicted to point when a command sent now takes effect. It uses the newest `ATTITUDE` yaw/pitch rates, shown frame age and estimated uplink delay: half the latest clock-query round trip. This is not a direct command-delay measurement. `sat_teleop_policy::predict` maps the rates times age-plus-delay into pixels using `[camera] hfov_deg`.

For a fixed camera, `[camera] tilt_deg` also enables a dashed horizon from roll, pitch and mount tilt. A gimbal holding its own angle gets no horizon. The simulated camera uses `hfov_deg = 114.6` (2.0 rad). Without `[camera]`, nothing is drawn.

Drawing requires `LIVE` or `LOW RATE`, known frame age, fresh uplink delay and attitude at most 1 s old. QGC requests `ATTITUDE` on connection; the autopilot may send none without a request. Marks are clamped, ignore lens distortion and stay below stamp rows, before the label. The freshness label and age remain unchanged. Prediction accuracy in real flight is unmeasured.

## Retained detail

The vehicle keeps what the operator may want to look at again: a JPEG of the source's frame, at the source's full size, every second. An operator who saw something in the live picture asks for the frame captured some seconds ago, whole or a rectangle of it, and gets it labelled as historical: the retained frame's id and capture time, never the live one's.

```text
sender (sat-teleop-video)                      vehicle app                      relay                    ground
 source -> convert -> tee -> scale/rate -> ... live
                        \-> queue(leaky,1) -> videorate 1 fps -> jpegenc q90 -> appsink(drop,2)
                                                |  try_send (queue of 2: full = snapshot dropped, counted)
                                                v
                                       writer thread -> store: <capture_ms>-<frame_id>.jpg  (retention: policy)
                                                                  ^ nearest at or before, crop (image), q80
   poll GET /jobs/next (5 s)  <-- overlay 10.77.0.1:8080 --> job table (4 jobs, 2 MiB)  <-- 192.168.232.20:8080 -- POST /jobs, GET /objects/{id}, DELETE /jobs/{id}
   PUT /jobs/{id}/chunks/{n}, POST /jobs/{id}/complete|fail --->
```

### Snapshot store

Snapshots are `<capture_ms>-<frame_id>.jpg` on a separate volume. Writes use `.tmp`, sync and rename. Startup removes leftover `.tmp` and orphan `.pos` files, indexes valid names, leaves unrelated files alone and trims to retention limits. Gaps greater than `gap_after_ms`, including restart gaps, appear in metrics.

`sat-teleop-policy::retention` enforces byte quota, maximum age and a free-space floor, evicting oldest first. Retrieval pins a snapshot until the pin expires. Capture time is the wall clock as the frame passes the snapshot branch. Its id is the next live frame id at that moment, within one live frame. The snapshot has no live stamp.

### Positions and outage snapshots

The bridge keeps the newest `GLOBAL_POSITION_INT` with its receive time. A snapshot gets its latitude, longitude, height above home and heading only if that reading is at most 2 s old (`position::fresh`). While position is stale, the bridge requests a 500 ms `SET_MESSAGE_INTERVAL`, retrying every 5 s without a fresh reading, even without a ground station.

Position and outage status live in `<capture_ms>-<frame_id>.pos`, written before the JPEG. A JPEG without this file has neither. A snapshot is marked for an outage when no relay probe answer arrived within `stale_after_ms`, the same condition that holds retrieval.

Outage protection keeps the newest marked snapshots beyond `max_age_s` and the byte quota, within `outage_share_percent` of `max_bytes` (default 50 %) and `outage_keep_s` from capture (default 3600 s). When that share fills, older snapshots lose protection first. Completing retrieval releases the sent snapshot's mark. Reading position does not command the autopilot.

### Listing outage snapshots

`detail outage` (`just detail-outage`) posts a job of kind `outage`: the vehicle answers with the marked snapshots captured at or after a requested time, oldest first, at most 16, with their positions, as JSON to `POST /jobs/{id}/listing` (the ground reads it from `GET /objects/{id}`). A fetched snapshot carries its position in the `X-Position` header, and `detail fetch` prints it.

### Sightings

The detector records sightings only while the ground is not heard and position is fresh. Each entry keeps kind, capture time, drone position, height, heading, picture-fraction box and score. Capture time subtracts the frame's monotonic age from the wall clock.

A same-kind detection within 2 s of the last observation and 0.15 picture fraction of its box centre extends that entry's time range and moves its box. Two boxes in one frame never merge. The newest 256 entries are kept; oldest entries drop. A continuing target stays one entry. The detector holds the list lock briefly and never waits on the link.

After reconnection, the bridge announces unsent sightings oldest first, at most one per 500 ms. `STATUSTEXT` comes from the autopilot's system and component, severity INFO, at most 50 characters. For example, `car 59.32931 18.06862 12m 14:02:31` gives kind, latitude/longitude to five places, whole metres above home and first-seen UTC. It spends only the shared telemetry bucket's upper half and waits behind replies (`RateCap::take_bulk`). An entry is marked sent only after its frame goes out, and remains listable.

`detail sightings` (`just sightings`) requests kind `sightings`, first seen at or after `at_capture_ms`, at most 16, oldest first. The vehicle posts a `sightings` listing to `POST /jobs/{id}/listing`. Each entry names the nearest retained snapshot within 1.5 s, on either side of capture time, if one still exists. Fetch it with `detail request --at-ms`. Nothing here writes to the autopilot.

### Job API

The relay starts the API only with a bearer token. Missing or wrong tokens get `401`. One token is shared with vehicle and ground; it excludes strangers but does not distinguish those roles. Separate listeners and firewall rules separate routes. The vehicle listener binds only the overlay, the ground listener only `ground_net`.

| Bound | Limit |
|---|---|
| URL | 200 bytes, checked after the HTTP library reads it |
| JSON | 4 KiB, unknown fields refused |
| Chunk | 16 KiB; excessive declared body length refused unread |
| Job id | 16 OS-random bytes as 32 lowercase hex digits; no client filesystem path |
| `max_bytes` | 1 B to 1 MiB per job; 2 MiB across at most 4 jobs, in any state |
| `expires_in_ms` | 1 s to 10 min |
| Chunk index | Below `max_bytes / 512` |
| Long poll | One at a time, 5 s |

SHA-256 is checked per chunk and whole object. Completion checks object size and JPEG dimensions against metadata. Duplicate bytes are a retry; conflicting bytes fail.

Ground routes are `POST /jobs {kind: snapshot|crop|outage|sightings, at_capture_ms, crop?: {x,y,w,h}, max_bytes, expires_in_ms}`, `GET /objects/{id}` and `DELETE /jobs/{id}`. Creation returns `{job_id}`. Fetch returns 202 while pending, 410 with failure reason, or the JPEG/listing. JPEG headers are `X-Capture-Ms`, `X-Frame-Id`, `X-Crop` (source pixels or `none`), `X-Source-Size` and optional `X-Position`. Delete cancels and frees the slot. Crops use picture fractions.

Vehicle routes are `GET /jobs/next`, `PUT /jobs/{id}/chunks/{n}`, and `POST /jobs/{id}/complete`, `/fail` or `/listing`. Each listener answers only its own routes.

`tiny_http` 0.12 does not bound request-line length, header length/count or connection threads. It has no read/write timeout. A stalled header holds a connection thread; a stalled body or slow reader holds one of four listener workers. The API shares the live ingress and RTSP relay process, so a flood can exhaust that process. Before exposure to an untrusted network, isolate it in its own process or use a proxy that bounds sizes and times.

### Retrieval worker

It takes the oldest queued job, validates it again (it never trusts the relay: a rectangle outside 0 to 1 or without area fails the job, and `crop::pixel_rect` keeps every crop inside the picture), picks the nearest retained snapshot at or before `at_capture_ms`, pins it, and sends it as stored or decoded, cropped and encoded again at quality 80. The upload is one chunk at a time, each hashed and retried at most three times, paced by a token bucket at the allowance; the chunk is what the allowance carries in half a second (512 B to 16 KiB). A job ends done, cancelled (the relay has removed it: the next chunk finds out), expired (its life, counted from when it was taken) or failed (with a reason the ground can read).

### Live video first

`retrieval::permit` requires nonzero allowance, a `Normal` rung, no drain, fresh relay feedback and queue delay at most `queue_delay_base_ms`. It checks before each chunk. One chunk can be in flight; otherwise retrieval waits for permission or expiry.

Adaptation reserves the allowance in wire bytes per second while a job waits to send. It fits video to measured capacity after control, telemetry and retrieval. The reserve is granted only if a `Normal` rung still fits; otherwise retrieval gets zero. This decision uses measured capacity, so stopping retrieval cannot create a repeating reserve-and-withdraw cycle.

The snapshot branch uses a leaky queue, dropping appsink and separate writer. Its two-item writer queue drops and counts excess snapshots instead of delaying live capture. The [retrieval check](testbed.md#retrieval-check) grades the effect on live video.

### Failure behavior

An error in the snapshot branch (`jpegenc` or the appsink) propagates through the tee and restarts the live pipeline, so a misconfigured snapshot quality or rate can trigger it. A vehicle that crashes mid-job leaves the job Assigned on the relay until it expires (at most its `expires_in_ms`, and it holds one of the 4 slots meanwhile); the ground's `DELETE` frees it. While retrieval is held, the vehicle learns that a job was cancelled only at its next chunk or when the job expires.

On a Pi, `jpegenc` at full resolution once a second runs next to the H.264 encoder and costs CPU there. Its cost on a Pi is unmeasured; see [Hardware](hardware.md). Frames are copied before the branch queue so JPEG encoding does not retain the camera buffer pool.

### Requesting detail

```sh
just detail-request 10
just detail-request-at <capture-ms>
just detail-fetch <job-id> frame.jpg
just detail-outage
just sightings
```

The underlying CLI is `sat-teleop-ground detail request (--ago-s <n> | --at-ms <unix-ms>) [--crop x,y,w,h] [--max-bytes <n>] [--expires-s <n>] [--relay <url>]`. `detail fetch <id> -o <file>` waits, writes the JPEG with its historical label and frees the job. `detail cancel <id>` cancels it. The default relay URL is `http://192.168.232.20:8080` and `--job-token-file` supplies its token. `--ago-s` needs agreement between this machine's clock and the vehicle's capture clock; `--at-ms` names the capture time directly. The output identifies the retained frame, capture time, age, crop, source size and position when available. It never labels the object as live.

Sightings appear in QGC's Fly-view notification area under the vehicle, oldest first, at informational severity. Their coordinates locate the drone, not the detected person or vehicle. `just sightings` lists the boxes and retained snapshots, including sightings already announced, within the newest 256 entries. No outline means no detection was reported; it does not prove an area is clear. The detector provides no obstacle avoidance or obstacle-clear decision.

## Regional video

Regional video gives the pilot's box and detector regions a negative quantiser offset in `satx264enc`. The offsets leave the picture's pixels unchanged. `RegionCell` alone selects the regions and the pilot box confirmed to the ground. Boxes use picture fractions and keep their location across rung changes.

```text
QGC TRACK_RECTANGLE -> ground -> signed Region envelope -> vehicle RegionCell
source tee -> scale/rate -> region hook -> stamp -> ROI metadata -> encoder
           -> leaky RGB tap -> detector -> RegionCell detections
           -> leaky luma tap -> tracker -> RegionCell followed box
```

The adapter sends the box every 500 ms. A region envelope has its own sequence and uses lease/challenge freshness. It carries no MAVLink; a frame in it gets `not_allowlisted`. Lease release, expiry or regrant clears the box. Without following, confirmation requires the box to be within `pilot_ttl_ms` and selected under the region limits.

`[region] roi_qp_offset` accepts -51 to 0 and defaults to 0 (off). The testbed's regional configs use -6. A pilot box is kept only with a negative offset. The frame hook returns the selected regions; the sender attaches them as `GstVideoRegionOfInterestMeta` for the encoder. Outlines are drawn before the stamp. At most `max_regions` regions are retained; if their coverage exceeds `max_area_percent`, none are selected.

On the lab card with `PI=pi4`, `geo`, pilot only and offset -6, the box's SSIM gain is +0.018 at rung 0 and -0.026 at rung 2. At rung 0, outside SSIM is -0.041 against the rung below. These are one run each, with run-to-run spread about 0.02 and no measurable encode cost. They use emulated board limits, not Pi hardware.

### Detector

The detector is off without `[region] detector` in the vehicle config. NanoDet-Plus-m 320 runs through `tract-onnx` at `detector_fps` on a leaky one-buffer RGB 320x320 tap, every 500 ms at the testbed setting. A busy detector drops its own frames. A model load failure reports `detector.state = "failed: <cause>"` without stopping video.

`just model` downloads `models/person.onnx` and verifies its pinned SHA-256. An existing file is checked without downloading it again.

`detector.kinds` selects person, bicycle, car, motorcycle, bus, truck or boat (COCO 0, 1, 2, 3, 5, 7, 8), default `["person"]`. Unknown names fail configuration loading. Each cell takes its best-scoring wanted class; overlapping boxes retain the better score regardless of kind. Detections stay valid for `detection_ttl_ms` and are outlined even with offsets off. Each outline uses its kind's colour (people amber), with a one-letter tag in a solid tab at the box's top-left: P, B, C, M, U for bus, T, S for boat. Outline and tab size scale with picture width; a box too small for the tab gets the outline alone.

### Following a target

`[region] follow = true` turns the drawn box into a starting selection. It snaps to a detection overlapped by more than half. A leaky 320x180 luma tap feeds the newest frame at `follow_fps` (10/s in the testbed). A slow tracker never holds live video.

`sat_teleop_policy::follow` matches the starting pixels by normalised cross-correlation near the previous location. Brightness and contrast changes do not affect that score. A match of 0.7 or better moves the template slightly towards the new pixels; weaker matches never teach it. The followed box becomes the selected region and the vehicle's confirmed box. Updates reach the lease holder at most four times a second; QGC's box and crop requests follow them.

Follow ends when the score stays below `follow_min_score` for `follow_lost_frames`, the target reaches an edge, or the initial box is too small or flat. The outline disappears. `ToGround::Lost { region_seq }` repeats on the 250 ms report tick for 3 s unless a new box is followed. The ground clears only a box whose first sent sequence is no newer than that loss, so a delayed loss cannot clear a new or unsent box. Nothing resumes until the pilot draws again. Following never moves the drone.

### Detail inset

While tracking, `apps/ground/src/box_detail.rs` requests the newest retained snapshot's crop every 4 s, one job at a time. The box is widened 10 % per side and clamped (`inset::widen`). Requests use the same allowance, pacing and permit rules as all retrieval. A new box, STOP_TRACKING or loss clears `DetailCell`; a crop for an obsolete box is dropped.

`inset::place` permits the inset only on `Live` or `LowRate` frames and while its capture age is under 30 s. The age at arrival increases by monotonic time held, so a wall-clock step cannot make it younger. The inset is drawn after the predictive ring and before labels, never over stamp rows. It covers one corner and never replaces live video or changes the live label.

### Drawing a box

Take the lease with `just take-control`, then draw a rectangle in QGC's video. QGC sends `MAV_CMD_CAMERA_TRACK_RECTANGLE` or `MAV_CMD_CAMERA_TRACK_POINT`; `MAV_CMD_CAMERA_STOP_TRACKING` stops it. A point or a rectangle smaller than 1 % of either picture dimension is ignored and the previous box stays. QGC requests `CAMERA_TRACKING_IMAGE_STATUS` with `MAV_CMD_SET_MESSAGE_INTERVAL` and draws the red box from that status.

The newest crop appears in the top-right corner below the stamp rows, at most a third of the picture's width, in a 2 px frame. Its `DETAIL <age> s` label uses the live label's colour. The ground needs `--job-token-file`, as passed by the testbed compose file. A full link may deliver no crop; a narrow link may take seconds. A crop failure is logged and retried by a new request after 4 s.

Tracking becomes active when a crop arrives, or while the vehicle confirms the box is selected for a quantiser offset. It becomes idle after STOP_TRACKING, after 30 s without a crop when no confirmation remains, or when follow reports loss. Confirmations expire after 3 s. With offsets off, crops keep the red box active. With `follow = true`, the red box and crop requests move with the followed target. Nothing here moves the drone.

Detector outlines use amber P for person, green B for bicycle, cyan C for car, magenta M for motorcycle, red U for bus, white T for truck and deep blue S for boat. A box too small for its tab has only an outline. The followed box is pink with an F tab. Regional behavior is exercised by the [regional check](testbed.md#regional-check) and [follow check](testbed.md#follow-check); its QGC rendering is not checked on screen.

## Landing on a marker

`[landing] target = true` enables AprilTag tag36h11 detection; it is off by default. The pilot chooses Land. Reporting the tag alone never commands motion. The leaky greyscale tap uses `tag_fps` and `tag_width` and drops its own frames if the detector is slow. The detector is made in its own thread because its C implementation is not `Send`. It accepts only `tag_id`, no bit errors or decimation, and ignores a frame containing that id twice.

The bridge sends at most one `LANDING_TARGET` per fresh sighting. Lost, stale or badly oriented sightings produce no target. The first mount's attitude is requested every 5 s until fresh, then streamed every 200 ms. The fixed-camera/gimbal geometry, 500 ms age limits, height requirements, `MAV_FRAME_BODY_FRD`, distance and monotonic timestamp semantics are specified in [Landing on a marker](real-vehicle.md#landing-on-a-marker). Real mode permits targets only after its interlocks and only with a fixed camera. Hardware behavior is unmeasured.

In QGC, hover over the tag and choose Land in the mode list or Land button. ArduPilot steers the descent. If the camera is not down, QGC receives `Landing target off: camera N deg from straight down` once. The copter pauses about 7 s and lands where it is with `PrecLand: Failsafe Measures`. A lost tag produces `PrecLand: Target Lost`; ArduPilot retries over its last position, pauses and lands straight down. Change mode to take over. This has not been tried in QGC on screen; [the landing check](testbed.md#landing-check) acts as the station.

## Missions

Off by default. With `allow_missions = true` in the vehicle's `[mavlink]` table ([Real vehicle](real-vehicle.md#missions-flown-out-of-reach)) and the lease taken (`just take-control`), the Plan view works against a copter as it does against any ArduPilot: build the mission, press Upload, then Start in the Fly view. QGC 5.1.5 sends `MISSION_COUNT`, answers each `MISSION_REQUEST_INT` with a `MISSION_ITEM_INT` (item 0 is the home placeholder it sends for ArduPilot), and waits for the autopilot's `MISSION_ACK`. To start a copter on the ground it sets Guided, arms and sends `MAV_CMD_MISSION_START`; in the air it sets Auto. Items it reached and the current item come back as telemetry, so the Fly view shows progress. All of these travel in signed envelopes, like arming.

Without the switch, or without the lease, the Plan view's upload fails at once with QGC's normal error for a refused upload ("Mission denied", from a `MISSION_ACK` the ground adapter answers in the vehicle's name), Start is denied like any command, and so is a change into Auto. Fence and rally uploads are always refused, and `MISSION_SET_CURRENT` (QGC sends it only to jump to an item) is dropped.

QGC waits `1500 ms` for each next request or acknowledgement and sends `MISSION_COUNT` again, up to five times, only while the autopilot has not asked for an item (`PlanManager.h`, QGC 5.1.5). Over `geo`, with a round trip of about 0.6 s, an upload needs no repeat; a link with a round trip over 1.5 s would fail it. The [mission check](testbed.md#mission-check) exercises this path.

Mission item, count, altitude and hold limits are defined in [Missions flown out of reach](real-vehicle.md#missions-flown-out-of-reach). The vehicle applies them in every mode; enabling uploads does not bypass the gate.

## SITL

The rover runs model `rover`, instance 0, with `--speedup 1`, at 59.3293 N, 18.0686 E, 10 m, heading 0. Its system and component ids are 1. It waits for its companion on TCP 5760 before opening its other simulator ports, all kept on loopback by `sitl.nft`. Parameters and logs live in `/var/lib/sitl` in the container, so every `lab-up` starts from defaults. Its neutral throttle returns after RC override expires. The model has no brakes and coasts with a 2 s time constant. Image pins are in [Testbed images](testbed.md#images).

## Simulated flight

`just sim-up` runs an ArduCopter 4.7.2 quadcopter in Gazebo Harmonic in place of the rover. The camera and controls traverse the same emulated link and services as the default testbed. Control reaches SITL only.

```sh
just sim-up
just impair geo
just take-control
```

Start QGC. The copter and camera usually appear about two minutes after `sim-up`. Images take longer to build the first time. `testbed/Containerfile.sitl-copter` pins the same ArduPilot commit as the rover; `testbed/Containerfile.sim` supplies Gazebo and `testbed/compose.sim.yml` adds the services. `just lab-up` restores the rover testbed.

To build only the simulation images, stop the testbed with `just lab-down`, then run `just sim-image`.

### Flying from QGC

Enable the joystick before arming and keep it sending in stick modes. Without it the copter refuses arming with "Throttle below failsafe".

- For virtual controls, enable QGC's `virtualJoystick` setting. The throttle returns to the centre when released.
- For a gamepad, open Vehicle Setup, Joystick, enable it and calibrate every stick to its limits. Leave "Center stick is zero throttle" off. A spring-centred throttle then holds height in AltHold and Loiter. Keep the default axis rate of 25 Hz.

Use Takeoff in the Fly view to switch to Guided, arm and climb to the chosen altitude, or arm and fly manually. Land, Return and Pause use mode changes through the same gate. `just release-control` releases the lease.

### Command gate

QGC 5.1.5 sends ArduPilot mode changes as `COMMAND_LONG` `MAV_CMD_DO_SET_MODE`. State changes through `COMMAND_INT` or `SET_MODE` are not admitted. The vehicle learns its type from the autopilot heartbeat and denies all mode changes until that type is known.

| Vehicle | Modes admitted |
|---|---|
| Copter | Stabilize 0, AltHold 2, Guided 4, Loiter 5, RTL 6, Land 9, PosHold 16, Brake 17 |
| Rover | Manual 0, Hold 4, RTL 11 |

The numbers are ArduPilot's `Mode::Number` in `ArduCopter/mode.h` and `Rover/mode.h`. Copter Auto 3 also requires `allow_missions`. Acro, Flip, Throw, Drift, Sport, AutoTune and other modes are denied. Stabilize, AltHold, Loiter and PosHold also require a fresh lease-holder `MANUAL_CONTROL` setpoint, so a braked copter stays braked until sticks return. Takeoff altitude must be finite, positive and at most 120 m above home. Refusals appear as `mode_not_allowed`, `no_fresh_sticks`, `takeoff_altitude` or `mission_not_allowed` under `mavlink.control.rejected`, and QGC sees a denied command.

### Modes by link profile

| Profile | Suggested modes |
|---|---|
| `lan` | Stabilize, AltHold or Loiter; sticks answer within a frame or two. |
| `lte-good`, `lte-bad` | Loiter or PosHold; position hold tolerates late sticks. |
| `geo` | Guided takeoff, short inputs in Loiter or PosHold, then Brake, Land or Return. Allow about a third of a second each way and let the copter settle. |

Guided go-to is refused. Waypoints require `allow_missions`. Pulling the throttle to the bottom in flight triggers the radio failsafe and brakes the copter. Use Land to descend.

### Copter failsafes

`testbed/sat-teleop-copter.parm` defines the failsafes and cites their ArduPilot sources. Losing only the setpoint stream, with lease and adapter present, triggers Brake and holds position and height. Resume the stream and change mode to fly again. Losing the lease or adapter triggers Land after the 2 s GCS timeout. Guided flight and landing ignore the radio failsafe; the GCS failsafe still lands them when the lease goes. Auto has the mission exceptions below.

The simulated radio stays at throttle 1000 us. `FS_THR_VALUE 1085` puts that below the failsafe threshold, `FS_THR_ENABLE 7` selects Brake or Land, and `RC3_MIN 1100` with `RC3_MAX 1900` keeps the stick's centre at hold altitude. ArduPilot also refuses Stabilize during RC failsafe; AltHold, Loiter and PosHold request zero climb while that failsafe is active.

A filled uplink can expire control envelopes and trigger these same failsafes; see [Control lifetimes](#control-lifetimes). A `lan` to `geo` change can briefly look like a queue while adaptation identifies the changed base RTT. Initial capacity probing and a real capacity drop can still fill a queue. A slow simulation changes Brake, Land and descent times even on a clean link. [Simulated-flight grading](testbed.md#simulated-flight-check) separates those measurements from hardware claims.

### Flying missions

Missions are off by default. To fly one manually:

```sh
just sim-up vehicle-sim-missions.toml
just take-control
```

Build the mission in QGC's Plan view, Upload, then Start in the Fly view. `vehicle-sim-missions.toml` sets `[mavlink] allow_missions = true`. `just sim-mission <profile> on|off` runs the [mission check](testbed.md#mission-check) with the switch enabled or disabled.

The copter parameters set `FS_OPTIONS 15`: bits 0 and 1 continue Auto through radio and GCS failsafes, with the other configured bits retained. The fence uses `FENCE_ENABLE 1`, `FENCE_ACTION 1` (RTL or Land), `FENCE_TYPE 3` (altitude and circle), `FENCE_ALT_MAX 100` m and a 300 m default radius. `BATT_FS_LOW_ACT 2` selects RTL, but the simulated battery has no low threshold and does not trigger it in checks. Guided, Loiter, Brake and stick modes keep their normal failsafes.

ArduPilot requests uploaded items with `MISSION_REQUEST`; QGC handles that and `MISSION_REQUEST_INT`. A six-item mission needs about seven round trips. Mission continuation is checked only for a link cut inside the fence. A fence breach has not been flown.

### Regional simulation

`just regional=1 sim-up` uses `testbed/config/vehicle-sim-regional.toml`: `kinds = ["person", "car"]`, `roi_qp_offset = -6` and `follow = true`. It outlines detected people and the car. Selected regions get a quantiser offset, and the tracker follows the box drawn in QGC. The world contains two standing people, 5 and 6.5 m ahead, a parked hatchback 9 m ahead and 5 m left, and a white H landing pad. From the runway the pad is a sliver near the horizon; it becomes visible in flight.

This config keeps 4 MiB of snapshots, compared with 512 KiB in the other configs. That retains more outage snapshots across the 30 s cut while respecting the store's newest-first outage share. Use [the follow check](testbed.md#follow-check) and [the sightings check](testbed.md#sightings-check) for automated exercises.

### Simulated camera

Gazebo renders its gimbal camera headlessly at 640x360 and 15 fps. It looks forward and 20 degrees down. `MNT1_DEFLT_MODE 1` selects Neutral and `MNT1_NEUTRAL_Y -20` holds the mount there instead of letting the parked radio drive it to its limits.

The pinned ardupilot_gazebo `GstCameraPlugin` encodes with x264: 800 kbit/s, `speed-preset` 6 (medium), `tune` 4 (zero latency), and `key-int-max` 10 frames. It sends RTP to the vehicle's ingest in `testbed/config/vehicle-sim.toml`. The vehicle decodes, stamps and sends it. Label age starts at that stamp; the render and plugin encode time are not included and are not measured separately. Gazebo shares the vehicle's network namespace and ingest accepts only the vehicle's own address, so video cannot bypass the emulated link.

The world has an AprilTag 6 m east and 3 m south of takeoff. `just sim-landing <on|off|forward> <profile>` exercises [marker landing](testbed.md#landing-check). `on` enables targets and points the camera down using `testbed/sat-teleop-copter-camera-down.parm`. `off` disables targets. `forward` enables them with the camera 20 degrees below the horizon. `testbed/sat-teleop-copter-precland.parm` supplies the precision-landing values required in real mode, plus `PLND_LAG 0.15` for this simulated camera path.

### Simulation resources

Gazebo uses Mesa llvmpipe without a GPU. The world has no shadows or GUI services, 21 models and one runway texture. The field uses a plain green material. On the 2 GiB Podman machine its cgroup peaks near 1064 MB at startup and settles around 676 to 702 MB; `mem_limit` is 1100 MB. Do not raise it. The whole simulation uses roughly 1.5 GB, leaving little room for other work.

Gazebo uses about 146 % of one core and the emulated ArduCopter about 27 %, with 18 to 22 MB for SITL. The x86_64 binary keeps up with Gazebo's lockstep under qemu. The world uses a 1 ms step and real-time cap 1.25. A loaded host can slow the simulation enough to fail timing budgets despite a clean link.

Never build, run `just check` or start another heavy container while the simulation is up. Run `just lab-down` first. Starting alongside heavy containers can exhaust the machine and kill Gazebo.

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

`just demo <profile>` sends the Mac's camera through the testbed's delay, jitter, loss and bandwidth profile to QGC. Frames are stamped and adapted on the vehicle. Label age excludes capture and encoding before the stamp. The autopilot remains ArduRover SITL; no real vehicle is controlled.

### Running the demo

Use the [development tools](testbed.md#development-setup), QGC and ffmpeg (`brew install ffmpeg`).

```sh
just demo-cameras        # list camera indexes; default 0
just demo lan            # geo, lte-good, lte-bad, geo-degraded or outage-10s also work
just demo geo 1          # CAMERA=1 just demo geo also works
just demo-stop           # stop camera and testbed
```

The recipe starts `vehicle-ingest.toml`, applies the profile, starts the camera and waits for its first frame. With built images this takes about a minute. Failure after capture starts stops the camera but leaves the testbed up; use `just lab-down` or `just demo-stop`. Otherwise capture continues until stopped. Another process holding Mac UDP 5700 prevents `lab-up` from publishing it. Plain `just lab-up` restores the test pattern.

`just regional=1 demo geo` uses `vehicle-ingest-regional.toml`, enabling person detection and `roi_qp_offset = -6`. Selected regions get a quantiser offset. Detected people are outlined amber. Use `just take-control`, then [draw a box](#drawing-a-box).

### Camera permission on macOS

Allow the terminal app to use the camera when macOS asks. If refused, no prompt appears, or another app holds the camera, the recipe reports no frames and prints ffmpeg's error. Enable the terminal in System Settings, Privacy & Security, Camera. The recipe does not bypass a refusal.

### Watching the demo

Start QGC and enable Low Latency Mode; [camera discovery](#qgroundcontrol) supplies the stream. Expect `LIVE` with a few milliseconds of stamp age on `lan`, about 0.33 s on `geo`. Other profiles add their configured delay. `lte-bad`, `geo-degraded` and `outage-10s` can lower quality or pause video, with `LOW RATE`, `STALE` or `NO VIDEO` labels.

### Showing each label

- Covering the lens gives a dark but live picture. Ingest stamps arriving frames; it cannot detect a camera repeating an unstamped image.
- `just video-hook pause`, `resume` and `freeze` exercise the labels as in [Checking the display](#checking-the-display).
- To stop only capture, use `kill $(cat .cache/demo-camera.pid)`. `just demo-stop` also stops the testbed. The no-camera case follows source-loss rules but has not been watched on screen.
- `just demo geo-degraded` starts a worse link; `just impair geo` changes a running demo's profile.

### Driving with the demo

The webcam picture and simulated rover are independent. Use [Taking control](#taking-control): acquire the lease, choose Manual, arm, drive, disarm and release. Under `geo`, the observed rover response is roughly 0.6 s later.

### Camera ingest

Mac ffmpeg captures avfoundation at 1280x720, 30 fps, without input buffering. It encodes x264 `ultrafast`, `zerolatency`, no B-frames, 3 Mbit/s, with a keyframe and repeated parameter sets every 15 frames (0.5 s). RTP/H.264 packets of 1200 bytes go to `127.0.0.1:5700`, published only to the vehicle's separate ingest network.

The vehicle accepts only the configured host address, decodes with slice threading and no jitter buffer, then stamps the picture. Decode adds about 1 ms before the stamp. Camera capture latency is unmeasured without an optical test; Mac x264 costs about 3 ms CPU per frame. Neither is included in label age. `just ingest-check lan` exercises the same port with a test-pattern feeder.

### Other ingest senders

The ingest accepts RTP/H.264 (payload type 96, no B-frames) from any sender whose address is listed in `from` of the vehicle's `[source]`, so a different camera or encoder on the host needs only to send that stream to `127.0.0.1:5700`. Port, addresses and the single-port filter are in `testbed/compose.yml` and `testbed/scripts/ingest.nft`. `from` trusts a source address only, and nothing proves the sender is the camera: on Podman, any process on the Mac can send from the gateway address that `from` lists. After a lost packet the picture is broken until the next keyframe, so the ingest drops pictures until then rather than showing them as fresh, for at most the camera's keyframe interval (half a second with `just demo`). It has no jitter buffer and does not ask the camera for keyframes.
