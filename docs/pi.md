# The vehicle on a Raspberry Pi

The vehicle runs on a Raspberry Pi as a native binary under systemd: a USB webcam or a Pi camera module for video, the board's hardware H.264 encoder, and a flight controller over USB or the Pi's UART. `docs/deploy.md` has the whole deployment (relay, ground and the bench checklist); this page is the Pi's part.

**Nothing on this page has run on hardware.** The code paths exist and the pure parts are tested, the container tests cover the rest of the vehicle, and every statement about a board below comes from documentation, not from a measurement. Each section says what is unverified.

## Boards

| Board | Role | Hardware H.264 encoder |
|---|---|---|
| Pi Zero 2 W (512 MB) | The documented minimum target | Yes, as far as the documentation says (below); not verified here |
| Pi 4 | Recommended | Yes |
| Pi 5 | Software encoding only | No |

Raspberry Pi's camera documentation gives `v4l2h264enc` as the encoder for "a Raspberry Pi 4B or earlier device" and `x264enc` for a Pi 5, and says "Raspberry Pi 5 uses software video encoders" with "a longer latency than the old hardware encoders" ([Camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html), sections "GStreamer", "`libcamerasrc` GStreamer element" and "Low latency video with Raspberry Pi 5"). The page does not name the Zero 2 W; "Pi 4B or earlier" is the reading that puts it with the Pi 4. On a Pi 5 the config's `encoder = "hardware"` falls back to `x264enc` and says so in the log; whether a Pi 5's cores keep up at the rungs of `testbed/ladder.toml` is unmeasured.

The vehicle never decodes H.264 and encodes it again. It decodes one thing: an MJPEG webcam's frames, because the in-picture stamp is drawn into raw pixels (a YUYV webcam or a libcamera source needs no decode at all). A camera that sends H.264 itself cannot be used for the same reason.

## Install

1. Flash Raspberry Pi OS Lite, 64-bit, the release based on Debian 13 (trixie). The binary is built on Debian trixie (`testbed/Containerfile`, GStreamer 1.26), so an older base would lack its libraries. Check with `cat /etc/os-release`.
2. On the Mac: `just deploy-keys <relay public host:port>` and `just vehicle-release` (`docs/deploy.md`, "Images and binaries").
3. Copy `dist/vehicle/` to the Pi and run `sudo ./provision.sh` in it. It installs the GStreamer packages, `v4l-utils`, chrony and WireGuard, makes the `sat-teleop` user (groups `video` and `dialout`), checks the binary against `SHA256SUMS` and installs it, the configs, the tunnel and the systemd unit.
4. Edit `/etc/sat-teleop/vehicle.toml` (template: `deploy/vehicle/vehicle.toml`) for the camera and the flight controller, then `sudo systemctl restart sat-teleop-vehicle`.

The unit restarts the vehicle two seconds after any exit, waits for the tunnel and chrony, caps its memory at 300 MB and keeps retained snapshots in `/var/lib/sat-teleop`. `journalctl -u sat-teleop-vehicle` is the log, `curl -s -H "Authorization: Bearer $(sudo cat /etc/sat-teleop/vehicle.token)" http://127.0.0.1:9100/metrics` the metrics (every route of the endpoint needs the token). The tunnel dials out with a keepalive every 15 seconds, so a modem behind NAT needs no open port.

## Camera

`sat-teleop-vehicle probe-camera` lists every video source GStreamer's device monitor finds with the formats and sizes each offers; `v4l2-ctl --list-devices` and `v4l2-ctl -d /dev/video0 --list-formats-ext` show the same from the driver.

**USB webcam (UVC)**, `kind = "v4l2"`: `v4l2src device=...` asked for the configured `format` (`mjpeg` or `yuyv`), `width`, `height` and `fps` by caps, so the camera is asked for exactly that mode. MJPEG goes through `jpegdec`, YUYV only through the sender's `videoconvert`. A webcam's MJPEG mode usually offers more frames per second at 720p than its YUYV mode over USB 2.0; the decode costs CPU the YUYV path does not. Which is better on a Pi is unmeasured.

**Camera Module through libcamera**, `kind = "libcamera"`: `libcamerasrc` from the `gstreamer1.0-libcamera` package (installed by `provision.sh` when the archive has it), on Raspberry Pi OS Bookworm or later, the releases that use the libcamera stack ([Camera software](https://www.raspberrypi.com/documentation/computers/camera_software.html): "From Raspberry Pi OS Bookworm onwards, the camera capture applications are named `rpicam-*`"). Test the camera with `rpicam-hello` first. Unverified: the source is built from the documented element name and caps, and has not seen a camera module.

**Buffers.** `v4l2src` takes its buffers from a small pool (four by default). The snapshot branch keeps only the frames due for the snapshot rate (a probe on its queue's sink pad drops the rest) and copies those as they enter, before its queue and its videorate, so nothing in it holds a camera buffer past the current frame and the copy happens about once a second, not at the frame rate; the live branch's own queues hold at most two. The copy of a 1280x720 frame took about 40 microseconds in the testbed (Apple Silicon container, a test buffer of that size); a Pi's memory is slower, and that cost is unmeasured. That no camera buffer is held cannot be shown without a camera, so it is a bench check: with the camera attached and snapshots on, the live frame age (the label at the ground, `frames_stamped` in the metrics) must stay level over several minutes and the video must not stall; a stall or a growing age means the pool ran dry.

## Encoder

`encoder = "hardware"` asks for `v4l2h264enc`. GStreamer registers that element only on a board with the encoder, so its presence is the check; without it the vehicle uses `x264enc` and logs `no v4l2h264enc on this board`. The vehicle gives the encoder, as `extra-controls`, `video_bitrate` (the rung's bitrate scaled down for the one-frame VBV allowance, as for `x264enc`, so any second stays within the rung), `repeat_sequence_header=1` and `h264_i_frame_period` (the rung's keyframe distance), and constrains its output to `video/x-h264,profile=main,level=4` with a capsfilter after it, so the level and profile are negotiated before the stream starts. Raspberry Pi's own example passes `extra-controls` the same way.

Unverified, and the likeliest to need a change on first contact: that the hardware encoder keeps to that bitrate (V4L2's controls here set no VBV buffer size, so its own rate control decides how far one second can exceed it; compare the relay's measured uplink rate with the rung's on a narrow link), that these control names and the caps are accepted by the Pi's driver, that a changed bitrate takes effect while playing (the vehicle sets `extra-controls` again on every rung change), that a size change renegotiates, and that a forced keyframe is honoured. The code that sets them is `make_encoder` and `set_bitrate` in `crates/media/src/sender.rs`.

## Flight controller

The default is a Pixhawk-class ArduPilot board.

- **USB**: `autopilot = "serial:/dev/ttyACM0:115200"`. The baud rate is ignored by a USB serial device.
- **The Pi's UART**: `autopilot = "serial:/dev/serial0:921600"`. On the Pi, enable the serial port hardware and disable the login shell on it (`raspi-config`, Interface Options, Serial Port). On the flight controller, set the TELEM port's `SERIALn_PROTOCOL` to 2 (MAVLink 2) and `SERIALn_BAUD` to 921 (921600 baud), with TX, RX and ground wired crossed; the Pi's pins are 3.3 V.

The `sat-teleop` user is in the `dialout` group, which owns both devices.

## What the Pi can control

By default nothing: in `mode = "sitl"` control frames reach the autopilot only after it has identified itself as a simulator (`docs/architecture.md`, "Control protocol"). A normal flight controller build does not, so on a Pi the vehicle logs the control frames it admits, counts them as `sunk` under `mavlink.control` in the metrics and writes none of them. What does work on a real autopilot is the read-only traffic: QGroundControl loads parameters and missions, shows telemetry and video, and an arm or mode command is refused (QGroundControl shows it as denied, the lease stays) and stick commands are only logged. Real-vehicle control is a separate, explicit mode behind interlocks: `docs/real-vehicle.md`.

## Clock sync

The ground labels video `LIVE` only when it knows how far the vehicle's clock can be from its own. Without a bound it labels the picture `AGE UNKNOWN`, never `LIVE` (`docs/measurement.md`, "Clock assumption").

`provision.sh` installs chrony with `makestep 1 3` (a Pi has no battery-backed clock) and four pool sources. With `clock = "chrony"` in `vehicle.toml` the vehicle runs `chronyc -c tracking` every 2 s and derives its bound from it, which is `none` while chrony is not synchronised; the derivation and how the ground adds it to its own are in `docs/measurement.md`, "Clock assumption". In real mode the bound also gates the start of control (`docs/real-vehicle.md`).

## A stuck camera

With a V4L2 camera (`kind = "v4l2"`) the stamp's capture time is the capture buffer's own: its age is the pipeline's running time minus the buffer's timestamp (`v4l2src` sets it from the driver's capture timestamp), taken before the frame is decoded, and a buffer that comes with the sequence number of the one before (its offset, which `v4l2src` sets from the driver's sequence number) is that frame again and keeps its old capture time. A camera that stalls and repeats therefore still sends frames, but their stamps stop advancing, and the ground reads `FROZEN` after 0.5 s and `STALE` once the picture is old enough. `kind = "libcamera"`, the test pattern, the file and the ingest sources keep stamping the time the frame reaches the stamp, so a stuck camera on those still reads `LIVE`.

What stays invisible: a driver that stamps a repeated image with a new timestamp and a new sequence number (some USB cameras do when their sensor stalls), and a camera whose lens is covered, which is a live camera of a dark scene. Only comparing images would catch those. The check has not run against a stalled camera (the testbed has no V4L2 device): the arithmetic is tested, the probe on `v4l2src` is not, and the first bench run should unplug the camera's sensor feed, or cover and uncover it, and watch the label (`docs/real-vehicle.md`, "Pre-flight checklist").

## Snapshot cost

The retained-snapshot branch encodes a full-size JPEG every second. In the testbed (Apple Silicon container, 1280x720 ball pattern, quality 90) it cost about one percentage point of one core, inside the runs' own spread (`docs/measurement.md`, "Retained detail"). A real scene's JPEG is ten to thirty times larger than the ball's, and a Pi's cores are slower: the cost on a Pi is unmeasured. The knobs are `[snapshot]` `fps` and `quality`, a lower resolution at the source, or removing the `[snapshot]` and `[store]` tables to switch the branch off. The hardware JPEG encoder is not used.

## What to measure first

On the bench, with props off, before anything else (`docs/deploy.md`, "Bring-up checklist"):

1. **Glass to glass.** Film a stopwatch or a millisecond display beside the ground's picture with a phone at 120 or 240 frames per second; the difference between the stopwatch and its image on the ground screen is the real latency. The stamp's age starts after capture and stops at the ground's output, so it leaves out the camera's own delay, the encoder's output buffering and QGroundControl's decode; this is the only number that includes them.
2. **Encoder latency.** The time between a frame passing the stamp and its first RTP packet leaving, with `v4l2h264enc` and with `x264enc` at the same rung. Compare the ground's age with the encoder swapped (`encoder = "x264"`).
3. **Uplink rate.** The vehicle's `adapt.uplink_bytes_per_s` and `rtp_bytes` on the real link, against what the modem and the plan deliver, and the rung the adaptation settles at.
4. **CPU and memory** of the vehicle per rung (`top`, or `systemctl show -p MemoryCurrent sat-teleop-vehicle`), on the board that will fly.
5. **Regional video's cost.** With `[region] detector` on, the tee's detector branch copies a source frame in the streaming thread `detector_fps` times a second (as the snapshot branch does) and the detector runs next to the encoder. Compare the vehicle's CPU and the frame age with the detector off and on, and read `detector.ms` and `region.soften` in the metrics; the testbed's figures are emulated limits, not a Pi (`docs/measurement.md`, "Regional video check").
