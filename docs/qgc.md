# QGroundControl

QGroundControl (QGC) 5.1.5 runs natively on the Mac and reaches the testbed through the ground adapter: MAVLink over UDP, and video over UDP from the adapter's freshness output, which QGC finds as the vehicle's camera. Video can also come straight from MediaMTX over RTSP (`docs/testbed.md`, "Viewing the stream"). This page covers the MAVLink link, the video and taking control of the vehicle (ArduRover SITL only).

## MAVLink link: nothing to configure

QGC's defaults work. With AutoConnect to UDP on (Application Settings, General; on by default), QGC listens on UDP 14550 on all addresses and answers whoever sends it MAVLink.

The ground adapter (`ground`, `sat-teleop-ground` in the ground namespace) sends telemetry to `host.containers.internal:14550` from a UDP socket of its own and reads QGC's replies on that same socket. On a Podman machine `host.containers.internal` is 192.168.127.254, gvproxy's gateway, which forwards the datagrams to the Mac's 127.0.0.1:14550 and maps the replies back: the adapter sees them coming from 192.168.127.254:14550. No port is published for MAVLink and no Comm Link needs adding.

```text
QGC :14550 <-> gvproxy (Mac 127.0.0.1) <-> ground adapter (own socket) <-> 10.77.0.3:14551
  <-> mavlink-router 10.77.0.1 <-> companion (gate, telemetry budget) <-> SITL
                  ground adapter 10.77.0.3:7701 -- control envelopes --> 10.77.0.2:7700 companion
```

If AutoConnect to UDP is off, add a link in Application Settings, Comm Links: type UDP, port 14550, no server address (QGC answers the sender), then connect it. Nothing else on the Mac may hold UDP port 14550.

## Video: the freshness output

The ground adapter (`sat-teleop-ground`) decodes MediaMTX's stream, marks every frame with its freshness and sends it as RTP/H.264 to `host.containers.internal:5600`, which gvproxy delivers to the Mac's 127.0.0.1:5600, QGC's default UDP video port. A new frame goes out as soon as it is decoded; when none came for two frame periods (2/15 s), or earlier when the state is about to change, the adapter sends the last one again with its state updated, so the picture keeps coming during an outage and says how old it is. Bottom left it reads `LIVE 0.33 s` in white while the video is live, `LOW RATE 0.82 s` in amber while it is current but comes at a low frame rate (frames more than 0.15 s apart, so it gets up to a second older between frames; the vehicle's slow rungs), and in red `STALE 1.20 s` (the newest frame is older than 0.6 s), `FROZEN 2.10 s` (frames arrive but none carries a new stamp), `NO VIDEO 3.50 s` (nothing arrived for 2 s, or the adapter's connection to MediaMTX is down; a black picture before the first frame) or `AGE UNKNOWN` (stamps do not read back, or no clock bound is configured); the time is the age of the newest frame. The same state is drawn as a row of blocks under the stamp for `lab check qgc-status`. Thresholds and their derivation: `docs/measurement.md`, "Freshness output".

What the states cannot see: except on a V4L2 camera, the stamp is drawn on the vehicle after capture, so a camera that keeps delivering the same image still gets a fresh stamp on every frame and shows `LIVE`. `FROZEN` catches only a repeated stamp: the same stamped frame sent again (the test hook does that) or a path stuck somewhere between the stamp and the ground. On a V4L2 camera a repeated frame reads `FROZEN` and then `STALE`; other sources are not covered (`docs/pi.md`, "A stuck camera").

If the adapter or its output pipeline dies, nothing marks the picture any more: QGC keeps showing the last frame with its last label until its own watchdog gives up. For a UDP stream QGC 5.1.5 starts the receiver with a 3 s timeout (`src/VideoManager/VideoManager.cc:845`), checks it every second (`src/VideoManager/VideoReceiver/GStreamer/GstVideoReceiver.cc:299`) in whole seconds since the last frame from its source (`:705-756`, `:1244-1246`), so 4 to 5 s after the last frame it stops the receiver (`:304-446`, which ends decoding, `:1318-1370`), shows "WAITING FOR VIDEO" over its no-video background instead of the picture (`src/FlyView/FlightDisplayViewVideo.qml:26`, `:43-67`) and reconnects after 1, 2, 4 ... up to 30 s (`GstVideoReceiver.cc:758-803`). MAVLink goes through the same adapter, so when the adapter dies QGC also shows Communication Lost 3.5 s after the vehicle's last message (below, "Liveness"). If only the output pipeline fails, the adapter restarts it after a second.

Nothing needs configuring in QGC: the adapter announces the stream as the vehicle's camera (below), and QGC plays it. Turn on Low Latency Mode (Application Settings, Video). Without it QGC 5.1.5 puts an RTP jitter buffer in front of the decoder, which holds packets for up to its RTP jitter latency setting (80 ms by default), and its video sink waits for each frame's presentation time (`src/VideoManager/VideoReceiver/GStreamer/GstVideoReceiver.cc:119`, `:220-228`, `:1148`; `lowLatencyMode` and `rtpJitterLatencyMs` in `src/Settings/Video.SettingsGroup.json`). The adapter runs on the same computer as QGC, so its stream has no jitter for the buffer to absorb. Leave the UDP URL setting at its default, `0.0.0.0:5600`.

The direct RTSP stream from MediaMTX (`rtsp://127.0.0.1:8554/live`, `docs/testbed.md`) stays available as the lower-latency option: it skips the adapter's decode and re-encode (a few milliseconds, `docs/measurement.md`), but has no indication at all: when the video stalls QGC shows the last picture until its watchdog gives up (as above, but after the RTSP Video Timeout setting, 8 s by default, `src/Settings/Video.SettingsGroup.json:119-127`, `src/VideoManager/VideoManager.cc:845`), and a repeating stream looks live. QGC 5.1.5 prefers a camera's stream, though: it switches to it when it finds the camera and again whenever it reconsiders its video source, even after Video Source was set by hand. To watch RTSP, remove `camera_uri` from `testbed/config/ground.toml` (the output keeps running; only the camera announcement stops), run `just lab-up` and set Video Source to "RTSP Video Stream".

### Camera discovery

Read from QGC 5.1.5's source (tag v5.1.5):

- The adapter is the vehicle's camera component: system id the vehicle's, component 100 (`MAV_COMP_ID_CAMERA`). QGC treats heartbeats from components 100-105 of the vehicle's system as cameras, once the vehicle's initial connection is complete (`src/Camera/QGCCameraManager.cc:155-211`), and asks a new one for `CAMERA_INFORMATION` with `MAV_CMD_REQUEST_MESSAGE`, alternating with the deprecated `MAV_CMD_REQUEST_CAMERA_INFORMATION` on retries (`:568-591`). With `CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM` set, it asks for `VIDEO_STREAM_INFORMATION` of all streams (stream id 0) a second later (`src/Camera/VehicleCameraControl.cc:2332-2343`, `:1903-1925`), and also for `CAMERA_SETTINGS`, `STORAGE_INFORMATION`, `CAMERA_CAPTURE_STATUS` and `VIDEO_STREAM_STATUS` (`:1518-1576`, `:734-753`, `:1927-1949`) without showing errors; it gives up on those after five retries.
- The adapter answers `CAMERA_INFORMATION` (only `CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM`, the output's resolution), `VIDEO_STREAM_INFORMATION` and `VIDEO_STREAM_STATUS` (stream 1, "overview", `VIDEO_STREAM_TYPE_RTPUDP`, running, the output's resolution, frame rate and bitrate, URI `udp://0.0.0.0:5600`) with `COMMAND_ACK` `ACCEPTED` and the message, and every other command to the camera with `MAV_RESULT_UNSUPPORTED`. None of it reaches the vehicle or travels in an envelope (`crates/control/AGENTS.md`).
- The URI: for an RTP/UDP stream whose encoding is not H.265, QGC plays the URI as given when it contains `udp://`, and `udp://0.0.0.0:<uri>` otherwise, so `udp://0.0.0.0:5600` and `5600` both work (`src/VideoManager/VideoManager.cc:611-618`). The `encoding` field is not in the MAVLink definitions the `mavlink` crate 0.21 generates, so it arrives as 0 and QGC takes H.264. QGC uses the type, the URI, the stream id and count, the flags and the resolution (for the aspect ratio).
- The Video Source setting: none needs choosing. Once a camera's stream is known, QGC sets Video Source to "UDP h.264 Video Stream" itself, whatever it was (`VideoManager.cc:589-641`, `:634`; a configured auto stream also counts as video configured, `src/Settings/VideoSettings.cc:232-238`), and then plays `udp://` plus its UDP URL setting (`VideoManager.cc:693-697`), whose default is `0.0.0.0:5600` (`src/Settings/Video.SettingsGroup.json:19`). That is why the UDP URL must stay at its default.
- Thermal: a stream flagged `VIDEO_STREAM_STATUS_FLAGS_THERMAL` is not listed among the camera's streams but used as the thermal overlay (`VehicleCameraControl.cc:1705-1712`, `src/Camera/QGCVideoStreamInfo.h:46`), and a thermal stream never counts as the main video (`VideoManager.cc:577-587`). The adapter never sets the flag.
- Liveness: QGC counts any message from the vehicle's system id, from any component, as a sign of the link (`src/Vehicle/Vehicle.cc:525-535`, `src/Vehicle/VehicleLinkManager.cc:35-53`) and declares communication lost after 3.5 s without one (`src/Vehicle/VehicleLinkManager.h:85`). A camera made up by the adapter must never keep a dead link looking alive, so it speaks only while the vehicle does: its heartbeat follows each of the vehicle's, and it answers only within 1.2 s of the vehicle's last message, so it can delay QGC's Communication Lost by at most that. QGC drops a camera silent for 5 s (`QGCCameraManager.cc:360-411`); the video keeps playing, because Video Source stays "UDP h.264 Video Stream" on the same URL, and the camera is found again when the vehicle comes back.

### Checking it in QGC

1. `just lab-up`, then start QGC and wait until the vehicle is loaded. Within a few seconds the video inset shows the ball pattern with `LIVE` and an age of about 0.01 s (`lan`); Application Settings, Video shows Video Source "UDP h.264 Video Stream".
2. `just video-hook pause` stops the vehicle's video (a test-only hook, `docs/testbed.md`): `STALE` after about 0.6 s, `NO VIDEO` after about 2 s, the last picture kept. `just video-hook resume`: `LIVE` again at once.
3. `just video-hook freeze` makes the vehicle send one frame again and again: `FROZEN` after about 0.5 s, the picture still. `just video-hook resume`.
4. Without the hooks, `lab impair` with `testbed/profiles/geo.toml` (`docs/testbed.md`) shows an age of about 0.33 s, still `LIVE`.

## What QGC can and cannot do

The ground adapter is QGC's only command endpoint. QGC's read-only traffic goes raw to the vehicle, lease or not, so QGC can always load the vehicle: parameters, mission download, autopilot version and stream rates. It may also set the vehicle's clock (`SYSTEM_TIME`, which ArduPilot can take its real-time clock from), the one raw message that writes something, though no flight state.

Without the control lease, QGC's arm, disarm and mode changes (and every other state-changing command) are answered by the adapter at once with `MAV_RESULT_DENIED`, which QGC shows as "command denied"; its heartbeat and joystick `MANUAL_CONTROL` are dropped and counted (`control` in the adapter's metrics). With the lease, the adapter wraps arm, disarm and mode changes, QGC's heartbeat and `MANUAL_CONTROL` into control envelopes, and the vehicle applies them while they are fresh (`docs/architecture.md`, "Control protocol"); SITL's own acknowledgement comes back through the telemetry. Every other state-changing message is dropped with or without the lease (the list is under "What QGC 5.1.5 sends when it connects"); a command no envelope may carry gets `MAV_RESULT_UNSUPPORTED`.

## Taking control

1. Start the testbed (`just lab-up`) and QGC; wait until QGC shows the vehicle loaded.
2. `just take-control` asks the adapter to take the vehicle's lease. It prints the adapter's status, for example `{"lease":"held","id":1,...}`, or why not (`409` with `lease_held` while another controller holds it). `just control-status` shows it at any time.
3. Set the flight mode in QGC (Manual for driving) and arm. QGC 5.1.5 sends exactly the commands the vehicle allows for these: `COMMAND_LONG` `MAV_CMD_DO_SET_MODE` (param 1 `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`, param 2 the ArduRover mode) and `MAV_CMD_COMPONENT_ARM_DISARM` (param 1 1 to arm, 0 to disarm; Force Arm adds param 2 2989). The vehicle passes a mode change only for a mode of its own type, which it learns from the autopilot's heartbeat: Manual, Hold and RTL for a rover (the other ArduRover modes are denied), and for a copter Stabilize, AltHold, Guided, Loiter, RTL, Land, PosHold and Brake (`docs/simulation.md`). A refused mode change is denied to QGC like any other refused command.
4. Drive with a joystick (enable it for the vehicle in QGC) or the virtual thumbsticks: QGC sends `MANUAL_CONTROL` (y steering, z throttle) at 25 Hz by default, and ArduRover takes it as an RC override from its ground station's system id (255).
5. Disarm, then `just release-control`. If refreshing stops instead (link, adapter or QGC gone), SITL's RC override ends 0.5 s later and its GCS failsafe holds it after about 3 s (`docs/architecture.md`, "Control protocol").

If the link drops for longer than the lease lifetime (3 s), the lease is gone: QGC's commands are denied again and nothing it sent meanwhile runs later. Take control again with `just take-control`.

Telemetry is limited to 8000 MAVLink bytes per second (`testbed/limits.toml`). SITL's default streams (every message once a second) use about 890 B/s; if QGC asks for faster streams than the budget allows, bulk messages are dropped, never the heartbeat, system status, position or attitude. ArduPilot sends the whole parameter list at once; it waits in the vehicle's reply queue and arrives complete in about 6 s.

The vehicle reads and forwards MAVLink with the `ardupilotmega` dialect, which includes `common`, so ArduPilot's own messages (EKF status, for example) reach QGC too. With the `common` dialect alone, about a fifth to a quarter of SITL's bytes never left the vehicle.

## What QGC 5.1.5 sends when it connects

Read from QGC 5.1.5's source (`InitialConnectStateMachine`, `ParameterManager`, `Vehicle`, `APMFirmwarePlugin`, `FTPManager`) and checked against the vehicle's `gcs_refused` counts:

- Admitted: `HEARTBEAT`; `REQUEST_MESSAGE` for `AUTOPILOT_VERSION`, `AVAILABLE_MODES` and component metadata; `MISSION_REQUEST_LIST` for the mission, fence and rally points, then `MISSION_REQUEST_INT` and `MISSION_ACK`; `PARAM_REQUEST_LIST` and `PARAM_REQUEST_READ`; `REQUEST_DATA_STREAM`; `SYSTEM_TIME`; `TIMESYNC`.
- Dropped: `FILE_TRANSFER_PROTOCOL`. For ArduPilot, QGC first asks for the parameter file `@PARAM/param.pck?withdefaults=1` over MAVLink FTP, retries once, then falls back to `PARAM_REQUEST_LIST`. These are the two refusals a QGC session showed (`not_allowlisted: 2`), and a stand-in sending the same `OpenFileRO` twice was counted as `FILE_TRANSFER_PROTOCOL: 2` with no reply. FTP stays dropped: the same message also creates, writes, renames and removes files on the vehicle, and QGC loads every parameter without it. The cost is QGC's FTP timeouts before the parameter download starts.
- Sent only on a user action or when the vehicle asks. With the lease they travel in envelopes: flight-mode changes (for ArduPilot always `MAV_CMD_DO_SET_MODE`, because its firmware plugin says it supports the command; the `SET_MODE` message only for firmware that does not), arming and disarming (`MAV_CMD_COMPONENT_ARM_DISARM` as `COMMAND_LONG`, never retried: QGC retries only `REQUEST_MESSAGE`, `REQUEST_AUTOPILOT_CAPABILITIES`, `PREFLIGHT_STORAGE` and `RUN_PREARM_CHECKS`, and increments `confirmation` when it does), and `MANUAL_CONTROL` from a joystick or the virtual thumbsticks. Dropped with or without it because they change the vehicle: `PARAM_SET`, mission uploads and `MISSION_SET_CURRENT`, `RC_CHANNELS_OVERRIDE` (only for extra joystick axes, if so configured), `SET_GPS_GLOBAL_ORIGIN`, guided-mode targets, `TERRAIN_DATA` (answers to the vehicle's `TERRAIN_REQUEST`, which loads terrain into it) and Remote ID (`OPEN_DRONE_ID_*`) for a vehicle that has it. QGC also answers a vehicle's `PING`, which would be dropped too; SITL sent none.

Read from QGC 5.1.5's source: `Vehicle::setFlightMode`, `Vehicle::setArmed` and `Vehicle::forceArm` (`src/Vehicle/Vehicle.cc`), `APMFirmwarePlugin::MAV_CMD_DO_SET_MODE_is_supported` (`src/FirmwarePlugin/APM/APMFirmwarePlugin.h`), `MavCommandQueue::_shouldRetry` and the `confirmation` field (`src/Vehicle/MavCommandQueue.cc`), `Vehicle::sendJoystickDataThreadSafe` and `axisFrequencyHz` (default 25, `src/Settings/Joystick.SettingsGroup.json`). QGC accepts a `COMMAND_ACK` only from the vehicle's system id and matches it by the sender's component and the command, so the adapter's own answers come from the command's target system and component.

## Verified without QGC

QGC is a GUI, so the link was checked on 2026-11-01, before the control lease (when the vehicle still admitted a ground station's heartbeat raw), with a stand-in for its default link: a Python script with plain UDP sockets bound to 127.0.0.1:14550 that answers the address it hears from with a GCS `HEARTBEAT` every second and `COMMAND_LONG` `REQUEST_MESSAGE(AUTOPILOT_VERSION)`.

- Telemetry arrived from 127.0.0.1 on a gvproxy port (a new one for each new socket), each message about once a second, vehicle disarmed: with `ardupilotmega` also ArduPilot's `MEMINFO`, `AHRS`, `SIMSTATE`, `AHRS2` and `EKF_STATUS_REPORT`, which the `common` dialect did not forward.
- Under `lan`, `COMMAND_ACK` (accepted) came back 21-24 ms after the request, followed by `AUTOPILOT_VERSION`; under `geo`, 612-642 ms.
- The adapter counted every message from the stand-in as coming from the expected address (no `unknown_source` drops) and the vehicle wrote them to SITL.
- `PARAM_REQUEST_LIST` brought all 1256 parameters in 5.5 s under `lan` and 6.6 s under `geo`, with a heartbeat every second throughout (longest gap 1.2 s).
- SITL's clock (`SYSTEM_TIME.time_boot_ms`) advanced 29000 ms in 29042 ms of wall time: it keeps real time under qemu.

Checked with QGC 5.1.5 on screen (2026-11-01, before the `ardupilotmega` change): the vehicle loaded (Ready, Manual, GPS with 10 satellites, parameters complete, video live) and QGC showed Comms Lost when the testbed went down; the vehicle had refused 2 of 134 messages from it.

## Checked on screen

Each of these was watched in QGC 5.1.5 on macOS, with the testbed in containers. Nothing here involved hardware.

**The rover on `lan`** (2026-11-01, the images at `a798131`). The video inset showed `LIVE 0.00 s` and QGC showed the camera controls, so it had discovered the camera. Pausing the video (`just video-hook pause`) gave `STALE 1.00 s`, then `NO VIDEO 3.74 s`, and resuming gave `LIVE`. Freezing it (`just video-hook freeze`) gave `FROZEN 1.53 s`, and resuming gave `LIVE`. Stopping only SITL made QGC show Comms Lost within 7 s while the adapter's camera kept running.

**The webcam demo** (2026-11-01, `ca825c9`, `docs/demo.md`). On `lan` the label read `LIVE 0.00 s` with the camera controls, on `geo` `LIVE 0.32 s`. After a pause and resume under `geo` the picture showed a corrupted band and `AGE UNKNOWN 3.96 s`, then `LOW RATE 0.32-0.44 s` for ten seconds or more. `just demo-stop` gave Comms Lost and WAITING FOR VIDEO.

**The simulated flight** (2026-11-02, `docs/simulation.md`), after the camera fix. QGC loaded the copter with no setup: Stabilize mode, the Takeoff button and the gimbal widget were there. It followed Flying, Land and Not Ready while `sim-flight` flew, and labelled the rendered camera `LIVE`. Switching from the rover to the copter made QGC show its first-run Preferences dialog (firmware, vehicle and units); answering it once is all it needs.

