# Hardware

The vehicle runs natively on a Raspberry Pi. The relay runs on a public server, and the ground adapter runs beside QGroundControl on the operator's laptop.

Deployment on hardware is unverified. The provisioning scripts, units, camera, serial link and modem have not run together on hardware. Follow the bring-up checklist in order and stop at the first failure. The vehicle defaults to the logging sink for a real autopilot; real control requires the interlocks in [real-mode.md](real-mode.md).

## The machines

| Machine | What it runs | Needs |
|---|---|---|
| Vehicle | `thinlink-vehicle`, a native binary under systemd | A Raspberry Pi (Zero 2 W minimum, Pi 4 recommended; [the Pi](#boards)), Raspberry Pi OS Lite 64-bit (Debian 13), a USB webcam or a Camera Module, a Pixhawk-class ArduPilot flight controller over USB or UART, a modem |
| Relay | `thinlink-relay`, MediaMTX and mavlink-router, containers with host networking | A public Ubuntu LTS server with an IPv4 address, 1 vCPU and 512 MB to 1 GB. The relay never transcodes. Nothing about its CPU or memory is measured |
| Ground | `thinlink-ground` in a container, QGroundControl natively | A Mac or a Linux laptop with Podman or Docker |

The vehicle and the ground machine dial out to the relay over WireGuard (UDP 51820), so the vehicle can sit behind the carrier NAT of a cellular or satellite modem and the ground behind a home router. Everything else travels inside the tunnel on `10.77.0.0/24`: relay `.1`, vehicle `.2`, ground `.3`.

| Port (on the relay's overlay address) | Who | What |
|---|---|---|
| UDP 5004 | vehicle | RTP/H.264 video |
| UDP 5006 | vehicle | Adaptation probes and the relay's answers |
| UDP 14550 | vehicle | MAVLink telemetry, to mavlink-router |
| TCP 8080 | vehicle | Job API for retained detail |
| TCP 8081 | ground | The same job API, for the ground's requests |
| TCP 8554 | ground | RTSP from MediaMTX |
| UDP 7700 on the vehicle, 7701 on the ground | each other | Control envelopes, direct between the two |

The relay's firewall (`deploy/relay/nftables.conf`) opens SSH and UDP 51820 to the Internet and lets each peer reach only its own ports through the tunnel. The relay also forwards the control envelopes between the vehicle and the ground (IP forwarding on, and the firewall's forward rules pass only those two UDP ports), so the vehicle's control endpoint is reached through it. The relay's Docker runs with its own iptables rules off (`provision.sh` writes `/etc/docker/daemon.json`), because every container uses host networking and Docker's default FORWARD drop would block that forwarding. The forwarding has never run on a real relay server: the testbed's relay has its own rules. Bring-up step 5 proves it. On a server where Docker already ran with its iptables management on, the FORWARD drop it set stays after the option is turned off: check with `sudo iptables -S FORWARD` (a `-P FORWARD DROP` line, or a `DOCKER-USER` jump) and clear it with `sudo iptables -P FORWARD ACCEPT` or a reboot.

## Images and binaries

The release recipes build locally. The runtime image carries all four programs; the vehicle's binary is taken out of the arm64 one, so `just vehicle-release` needs an arm64 build host (Apple Silicon or arm64 Linux) and stops if the binary it packs is not aarch64. All four programs link libx264, so their binaries are distributed under the GPL.

```sh
just deploy-keys 203.0.113.7:51820   # your relay's public address; keys and configs in deploy/secrets/
just vehicle-release                 # dist/vehicle: the arm64 binary, its checksum, configs, scripts
just relay-release                   # dist/relay: the amd64 image as a tar, its checksum, configs
just ground-release                  # dist/ground: compose file and configs, for the image built here
```

The recipes take the test setup down first (a build and the testbed do not fit the Podman machine together). `deploy/secrets/` and `dist/` are gitignored: they hold private keys, the WireGuard keys, the ground adapter's control signing key (`ctl-ground.key`, which only the ground release carries; the vehicle gets the public half and refuses every control message not signed with it) and three bearer tokens, one per service (`vehicle.token`, `ground.token`, `job.token`): the vehicle release carries `vehicle.token` and `job.token`, the ground release `ground.token` and `job.token`, the relay release only `job.token`. The Pi checks the binary and the server the image against `SHA256SUMS` before installing them; nothing is piped from the network into a shell.

`just image-for arm64|amd64` builds one architecture alone. The release recipes build without the vehicle's `testbed` cargo feature, so their binaries have no test hooks and refuse a config that names `debug_hooks` or `sitl_in_real_mode`; only `just image runtime` (the testbed's image) builds with it. `ground-release` builds the release image for this machine's architecture and writes it to `dist/ground/.env`. Builds run with one cargo job under a 1 GiB memory cap ([build limits](testbed.md#builds-and-gates)). An amd64 image is cross-built for `x86_64-unknown-linux-gnu` on the native architecture with the Debian cross toolchain and amd64 GStreamer headers. Only the runtime stage runs emulated. The amd64 image has not run on an amd64 machine.

## Relay

1. Create the server (Ubuntu 24.04 LTS, public IPv4, SSH access) and note its address. Open UDP 51820 in the provider's firewall if it has one.
2. Copy `dist/relay/` to it (`scp -r`) and run `sudo ./provision.sh` there. It writes `/etc/docker/daemon.json` with Docker's iptables rules off (it stops and asks if the file exists with other content), installs Docker, WireGuard and nftables, checks and loads the image, checks the firewall file (`nft -c`) before it replaces the live rules, installs `wg0` and starts the three containers with `restart: unless-stopped`.
3. Check: `sudo wg show` (a handshake appears once the vehicle and the ground are up), `docker compose -f /opt/thinlink/compose.yml ps` (three running), `curl -s -H "Authorization: Bearer $(sudo cat /opt/thinlink/job.token)" http://127.0.0.1:9100/metrics` (`rtp_packets` grows once the vehicle sends).

The relay is the only machine with a public address, and its RTSP endpoint has no authentication of its own (the tunnel and the per-peer firewall are the boundary), and its job API needs the bearer token `job.token` ([retained detail](drone.md#retained-detail) and [control](drone.md#control-protocol)). The relay is trusted for video and telemetry. Signed control envelopes prevent it from forging commands or taking the lease; see [authentication limits](real-mode.md#what-is-authenticated-and-what-is-not).

## Ground

On the operator's laptop, with QGroundControl on the same machine:

```sh
just ground-release            # builds the release runtime image here if needed
cd dist/ground
podman compose up -d           # or: docker compose -f compose.yml -f compose.docker.yml up -d
```

One container holds the WireGuard tunnel (`NET_ADMIN`, no other privilege) and the adapter runs in its network namespace. Start QGroundControl: it connects on its default UDP port 14550 and finds the video as the vehicle's camera, as in [QGroundControl](drone.md#qgroundcontrol). `THINLINK_IMAGE` (set in the release's `.env`) names the runtime image, `localhost/thinlink-runtime:arm64` or `:amd64`.

The lease and the retained-detail commands run inside the adapter's container:

```sh
podman compose exec ground thinlink-ground --token-file /secrets/ground.token take-control --adapter http://127.0.0.1:9100
podman compose exec ground thinlink-ground --token-file /secrets/ground.token status --adapter http://127.0.0.1:9100
podman compose exec ground thinlink-ground --job-token-file /secrets/job.token detail request --ago-s 10 --relay http://10.77.0.1:8081
```

Until both machines have fresh clock bounds, the picture reads `AGE UNKNOWN`. Configure the ground's clock as described in [clock sync](#clock-sync); real-mode settings are in [setup](real-mode.md#setup).

## Boards

| Board | Role | Hardware H.264 encoder |
|---|---|---|
| Pi Zero 2 W (512 MB) | The documented minimum target | Yes, as far as the documentation says (below); not verified here |
| Pi 4 | Recommended | Yes |
| Pi 5 | Software encoding only | No |

Raspberry Pi's camera documentation gives `v4l2h264enc` as the encoder for "a Raspberry Pi 4B or earlier device" and `x264enc` for a Pi 5, and says "Raspberry Pi 5 uses software video encoders" ([Camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html), sections "GStreamer", "`libcamerasrc` GStreamer element" and "Low latency video with Raspberry Pi 5"). The page does not name the Zero 2 W; "Pi 4B or earlier" is the reading that puts it with the Pi 4. On a Pi 5 the config's `encoder = "hardware"` falls back to `thinlinkx264enc` and says so in the log; whether a Pi 5's cores keep up at the rungs of `testbed/ladder.toml` is unmeasured.

The camera paths need raw pixels for the in-picture stamp. MJPEG webcam frames pass through a decoder; YUYV and libcamera frames need no decode. A camera that sends H.264 cannot use these V4L2 or libcamera paths. The separate [RTP ingest](drone.md#mac-camera-demo) decodes H.264 before stamping.

## Install

1. Flash Raspberry Pi OS Lite, 64-bit, the release based on Debian 13 (trixie). The binary is built on Debian trixie (`testbed/Containerfile`, GStreamer 1.26), so an older base would lack its libraries. Check with `cat /etc/os-release`.
2. On the Mac: `just deploy-keys <relay public host:port>` and `just vehicle-release` ([release packaging](#images-and-binaries)).
3. Copy `dist/vehicle/` to the Pi and run `sudo ./provision.sh` in it. It installs the GStreamer packages, `v4l-utils`, chrony and WireGuard, makes the `thinlink` user (groups `video` and `dialout`), checks the binary against `SHA256SUMS` and installs it, the configs, the tunnel and the systemd unit.
4. Edit `/etc/thinlink/vehicle.toml` (template: `deploy/vehicle/vehicle.toml`) for the camera and the flight controller, then `sudo systemctl restart thinlink-vehicle`.

The unit restarts the vehicle two seconds after any exit, waits for the tunnel and chrony, caps its memory at 300 MB and keeps retained snapshots in `/var/lib/thinlink`. `journalctl -u thinlink-vehicle` is the log, `curl -s -H "Authorization: Bearer $(sudo cat /etc/thinlink/vehicle.token)" http://127.0.0.1:9100/metrics` the metrics (every route of the endpoint needs the token). The tunnel dials out with a keepalive every 15 seconds, so a modem behind NAT needs no open port.

## Camera

`thinlink-vehicle probe-camera` lists every video source GStreamer's device monitor finds with the formats and sizes each offers; `v4l2-ctl --list-devices` and `v4l2-ctl -d /dev/video0 --list-formats-ext` show the same from the driver.

USB webcam (UVC), `kind = "v4l2"`: `v4l2src device=...` asked for the configured `format` (`mjpeg` or `yuyv`), `width`, `height` and `fps` by caps, so the camera is asked for exactly that mode. MJPEG goes through `jpegdec`, YUYV only through the sender's `videoconvert`. A webcam's MJPEG mode usually offers more frames per second at 720p than its YUYV mode over USB 2.0; the decode costs CPU the YUYV path does not. Which is better on a Pi is unmeasured.

Camera Module through libcamera, `kind = "libcamera"`: `libcamerasrc` from the `gstreamer1.0-libcamera` package (installed by `provision.sh` when the archive has it), on Raspberry Pi OS Bookworm or later, the releases that use the libcamera stack ([Camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html): "From Raspberry Pi OS Bookworm onwards, the camera capture applications are named `rpicam-*`"). Test the camera with `rpicam-hello` first. Unverified: the source is built from the documented element name and caps, and has not seen a camera module.

`v4l2src` takes its buffers from a small pool (four by default). The snapshot branch drops frames not due for its rate at the queue's sink pad. It copies kept frames before the queue and videorate, so the branch does not retain a camera buffer during JPEG encoding. The copy happens at the snapshot rate; the live branch's own queues hold at most two frames. The copy cost and buffer lifetime are unverified on a Pi. With the camera attached and snapshots on, watch the ground's frame age and `frames_stamped` for several minutes. Growing age or a stall can indicate an exhausted camera buffer pool.

## Encoder

`encoder = "hardware"` asks for `v4l2h264enc`. GStreamer registers that element only on a board with the encoder, so its presence is the check; without it the vehicle uses `thinlinkx264enc` and logs `no v4l2h264enc on this board`. The vehicle gives the encoder, as `extra-controls`, `video_bitrate` (the same reduced bitrate target as `thinlinkx264enc`), `repeat_sequence_header=1` and `h264_i_frame_period` (the rung's keyframe distance), and constrains its output to `video/x-h264,profile=main,level=4` with a capsfilter after it, so the level and profile are negotiated at stream startup. Raspberry Pi's own example passes `extra-controls` the same way.

Unverified, and the likeliest to need a change on first contact: that the hardware encoder keeps to that bitrate (V4L2's controls here set no VBV buffer size, so its own rate control decides how far one second can exceed it; compare the relay's measured uplink rate with the rung's on a narrow link), that these control names and the caps are accepted by the Pi's driver, that a changed bitrate takes effect while playing (the vehicle sets `extra-controls` again on every rung change), that a size change renegotiates, and that a forced keyframe is honoured. The code that sets them is `make_encoder` and `set_bitrate` in `crates/video/src/sender.rs`.

## Flight controller

The default is a Pixhawk-class ArduPilot board.

- USB: `autopilot = "serial:/dev/ttyACM0:115200"`. The baud rate is ignored by a USB serial device.
- The Pi's UART: `autopilot = "serial:/dev/serial0:921600"`. On the Pi, enable the serial port hardware and disable the login shell on it (`raspi-config`, Interface Options, Serial Port). On the flight controller, set the TELEM port's `SERIALn_PROTOCOL` to 2 (MAVLink 2) and `SERIALn_BAUD` to 921 (921600 baud), with TX, RX and ground wired crossed; the Pi's pins are 3.3 V.

The `thinlink` user is in the `dialout` group, which owns both devices.

## What the Pi can control

By default nothing: in `mode = "sitl"` control frames reach the autopilot only after it has identified itself as a simulator ([control](drone.md#control-protocol)). A normal flight controller build does not, so on a Pi the vehicle logs the control frames it admits, counts them as `sunk` under `mavlink.control` in the metrics and writes none of them. What does work on a real autopilot is the read-only traffic: QGroundControl loads parameters and missions, shows telemetry and video, and an arm or mode command is refused (QGroundControl shows it as denied, the lease stays) and stick commands are only logged. Real-mode control is a separate, explicit mode behind interlocks: `docs/real-mode.md`.

## Clock sync

`provision.sh` installs chrony with `makestep 1 3` (a Pi has no battery-backed clock) and four pool sources. With `clock = "chrony"` in `vehicle.toml` the vehicle runs `chronyc -c tracking` every 2 s and derives its bound from it, which is `none` while chrony is not synchronised; the derivation and how the ground adds it to its own are in [clock bounds](video.md#frame-age-and-clocks). In real mode the bound also gates the start of control (`docs/real-mode.md`).

On Linux, install `deploy/ground/chrony-ground.conf` on the host and set the ground's `clock = { chrony_at = "host.containers.internal" }`. The file assumes bridge address `10.88.0.1` and subnet `10.88.0.0/16`; adjust both for the container network. If chrony starts before the bridge exists, restart it after the first container starts. Without access to chrony, the label stays `AGE UNKNOWN`.

On Mac, set `clock = { fixed = <ms> }` from a measured bound on the Podman VM's clock error. A clock step beyond that bound invalidates it until restart; a clock that remains wrong without stepping is not detected. After waking the laptop, wait for the VM clock to agree with the Mac, then restart the ground container.

## A stuck camera

The [capture-time rules](video.md#frame-age-and-clocks) define which repeated V4L2 frames can be detected. `kind = "libcamera"`, the test pattern, the file and the ingest sources can still read `LIVE` with a stuck camera.

What stays invisible: a driver that stamps a repeated image with a new timestamp and a new sequence number (some USB cameras do when their sensor stalls), and a camera whose lens is covered, which is a live camera of a dark scene. Only comparing images would catch those. The check has not run against a stalled camera (the testbed has no V4L2 device): the arithmetic is tested, the probe on `v4l2src` is not. Unplug the camera's sensor feed and watch the label; covering the lens alone should leave it live.

## Snapshot cost

The retained-snapshot branch encodes a full-size JPEG every second. Its CPU and memory cost on a Pi is unmeasured. The knobs are `[snapshot]` `fps` and `quality`, a lower resolution at the source, or removing the `[snapshot]` and `[store]` tables to switch the branch off. The hardware JPEG encoder is not used.

## Bring-up checklist

Write down each result. Stop at the first failure and fix it before going on.

1. Link up.
   - `wg show` on the relay lists a recent handshake for both peers; from the vehicle `ping -c 3 10.77.0.1` and from the ground container the same.
   - The vehicle's `systemctl status thinlink-vehicle` is active and `curl -s -H "Authorization: Bearer $(sudo cat /etc/thinlink/vehicle.token)" http://127.0.0.1:9100/metrics` has `mavlink.sitl.up` true once the flight controller is connected.
2. Clocks. `chronyc tracking` on the Pi reports a synchronised clock; verify the ground's configured chrony or measured fixed bound; the vehicle reports its own bound (`mavlink.control.clock` in its metrics) and the ground shows the sum (`clock.effective_ms` in its metrics); note both over a few minutes.
3. Video.
   - `thinlink-vehicle probe-camera` lists the camera and the mode in `vehicle.toml`; `frames_stamped` and `frames_encoded` grow in the vehicle's metrics, with `stamp_failures` at 0.
   - The relay's `rtp_packets` grows; the ground's picture appears in QGroundControl with `LIVE` or `LOW RATE` when its age and both clock bounds permit, or `AGE UNKNOWN` without a usable bound.
   - Cover the lens, unplug the camera and plug it in again; the vehicle's `last_error` shows the failure and the pipeline restarts within seconds.
4. Telemetry. QGroundControl shows the vehicle, its parameters load, the attitude follows tilting the flight controller by hand.
5. A lease and a refused command.
   - Control reaches the vehicle through the relay. `take-control` returns and `status` shows the lease held and a `challenge_age_ms` under 1000: the request, the grant and the challenges crossed the relay's forward rules in both directions. On the vehicle `mavlink.control.grants` is at least 1 and `challenges` grows. If `take-control` times out while `wg show` has handshakes, the relay is not forwarding: on the relay check `sudo nft list ruleset` for a `drop` policy in another table's forward chain (`sudo iptables -S FORWARD`) and that `/etc/docker/daemon.json` has both keys off and Docker was restarted.
   - Without the lease, try to arm in QGroundControl: it shows "command denied" at once.
   - `take-control`; `status` shows the lease held. Arm in QGroundControl: it reports the command as denied, nothing happens on the flight controller, the vehicle's `mavlink.control.target` reads `logging_sink`, `sunk` grows and the journal says `went to the logging sink`. A flight controller that arms here means the interlock failed: stop.
   - The flight controller's own GCS failsafe sees no ground-station heartbeat, because the vehicle does not forward one to a real autopilot. Expect it to trigger on its own if `FS_GCS_ENABLE` is on; that is a property of the flight controller's settings, not of this software.
6. Failsafes on the bench.
   - `sudo systemctl stop thinlink-vehicle`: QGroundControl shows "Comms Lost" within about 7 s and the picture goes `STALE`, then `NO VIDEO`. Start it again; both come back.
   - Cut the modem's uplink (or the relay's WireGuard port) for 10 s: the label goes `STALE` then `NO VIDEO`, and recovers by itself after the link returns, without anyone touching the vehicle.
   - Throttle the uplink on the modem or with the carrier's plan (or `tc` on the Pi's WAN interface): the vehicle's `adapt.rung` steps down and the picture stays current at a lower size or rate.
   - Reboot the Pi with the power cut: the tunnel, the clock and the vehicle come back on their own.
7. Measure, as in "What to measure first" below.

## What to measure first

After the [bring-up checklist](#bring-up-checklist), measure:

1. Glass to glass. Film a stopwatch or a millisecond display beside the ground's picture with a phone at 120 or 240 frames per second; the difference between the stopwatch and its image on the ground screen is the real latency. The stamp and stage measurements have narrower boundaries; see [video age](video.md#measurement-method). The optical measurement includes the camera and the ground display.
2. Encoder latency. The time between a frame passing the stamp and its first RTP packet leaving, with `v4l2h264enc` and with `thinlinkx264enc` at the same rung. Compare the ground's age with `encoder = "hardware"` and `encoder = "x264"`.
3. Uplink rate. The vehicle's `adapt.uplink_bytes_per_s` and `rtp_bytes` on the real link, against what the modem and the plan deliver, and the rung the adaptation settles at.
4. CPU and memory of the vehicle per rung (`top`, or `systemctl show -p MemoryCurrent thinlink-vehicle`), on the board that will fly.
5. Regional video's cost. With `[region] detector` on, the tee's detector branch copies a source frame in the streaming thread `detector_fps` times a second (as the snapshot branch does) and the detector runs next to the encoder. Compare the vehicle's CPU and the frame age with the detector off and on, and read `detector.ms` and `stages.encode` in the metrics; the testbed's figures are emulated limits, not a Pi ([the regional check](testbed.md#regional-check)).

Then measure the relay's CPU and memory with video running and the ground adapter's CPU per output resolution. Hardware capacity is unmeasured.

## Limits of a deployment

- Video age is only as good as the [clock bound](#clock-sync).
- On a narrow link such as satellite, the vehicle paces its video to the uplink it measured, so control is rarely refused. It can still be in the first seconds of a rung selected before the uplink is measured (at the start, or after a route change) and when the uplink drops suddenly: video delays the vehicle's challenges past their lifetime and the vehicle refuses the commands that carry an old one ([control under a filled uplink](drone.md#a-filled-uplink-under-geo)). Expect refused commands and a label of `LOW RATE` or `AGE UNKNOWN` for a second or two then, and plan which flight modes you use on such a link.
- A stuck camera is detected only on V4L2, and the check has not met a real stalled one ([camera stalls](#a-stuck-camera)).
- The relay's job API reads requests with no limit on a header's length or count and starts a thread per connection ([retained detail](drone.md#retained-detail)). Keep it behind the tunnel and the firewall.
