# thinlink

Low-latency H.264 video from a drone over satellite and cellular links. The drone adapts resolution, frame rate and bitrate to the uplink it measures, keeping queue delay down before picture quality. QGroundControl is the ground station. Written in Rust on GStreamer.

It is a prototype. A container testbed emulates the link (delay, jitter, loss, rate limits and outages, with `tc` netem and HTB) and ArduPilot SITL stands in for the drone, so everything runs on one machine. The drone side targets a Raspberry Pi.

## Video path

```text
drone:  camera -> stamp -> x264 -> RTP -> pacer --UDP in WireGuard--> relay: jitter buffer -> MediaMTX (RTSP)
ground: RTSP -> decode -> read stamp -> label -> H.264 -> RTP/UDP -> QGroundControl
drone:  probe every 250 ms --> relay: what it received --> drone: pick a rung
```

- **Adaptation.** The drone times probes to the relay. A queue delay over 150 ms that keeps rising is a cut: the drone drops to the best rung that fits 75 % of the rate the relay measured. Over 300 ms it also pauses the video until the queue drains. Loss is read in GCC's bands. It climbs one rung after 10 s of healthy answers. Six rungs, from 1280x720 at 15 fps and 1.2 Mbit/s down to 320x180 at 1 fps and 25 kbit/s (`testbed/ladder.toml`).
- **Pacer.** Packets leave through a token bucket at the rung's rate. A frame that would wait more than 150 ms is dropped whole, together with the frames that depend on it. The sender requests a whole IDR picture at the next frame. Control and telemetry use their own sockets and a reserved share of the uplink.
- **Retained detail.** The drone keeps a full-size JPEG every second. You can fetch a whole one or a crop from the last few seconds through the relay's job API; it is sent only when live video and control leave room. Snapshots taken while the link was down are kept until fetched (`just detail-outage` lists them).
- **Sources and encoders.** Test patterns, y4m clips, V4L2 and libcamera cameras, or an RTP camera; `thinlinkx264enc` (`zerolatency`, with intra refresh and IDRs on request) or the Pi's `v4l2h264enc`. `thinlinkx264enc` supports quantiser offsets for selected regions. The ground output uses GStreamer's `x264enc`.
- **Stamp.** Each frame gets its id and capture time (two 32-bit values and a CRC-8) drawn as blocks into the luma plane, so the stamp survives encoding, transcoding and any player.
- **Freshness.** The ground compares the capture time with its own clock (both synced by chrony) and labels each frame `LIVE`, `LOW RATE`, `STALE`, `FROZEN`, `NO VIDEO` or `AGE UNKNOWN` with its age. The camera's own delay is not included.

`crates/video-core` (pure logic: stamp, freshness, pacing, budget, adaptation) and `crates/video` (the GStreamer pipelines) contain no drone-specific code.

## Drone features

- An aim ring, predicted from the autopilot's newest attitude, showing where the camera will point by the time a command sent now takes effect.
- MAVLink telemetry to QGroundControl through mavlink-router, within a byte budget.
- Control as Ed25519-signed JSON envelopes under a lease: one controller at a time, each command fresh and applied at most once. When commands stop, the drone stops.
- Waypoint missions that continue on the autopilot's failsafe when the link is lost (off by default, `docs/real-vehicle.md`).
- A box drawn in QGroundControl is fetched as a full-resolution crop and shown as an inset. The drone can follow it with a correlation tracker. An on-board detector (NanoDet, ONNX through tract) marks people and vehicles and reports what it saw while out of reach (`just sightings` lists them).
- Precision landing on a printed AprilTag through ArduPilot's `LANDING_TARGET` (off by default).

## Numbers

| | `lan` | `geo` (300 ms each way, 512 kbit/s up) |
|---|---|---|
| Video age at the ground | 7 ms | 320 ms |
| Command acknowledged | 18 ms | 620 ms |

Paused video shows `STALE` within 0.65 s. When the uplink drops from 512 to 128 kbit/s, the oldest frame is 0.57 s old and the age is back to normal within 7.6 s. Method: [video measurement](docs/video.md#measurement-method). Budgets and evidence: `perf/budgets.toml`.

## Crates

| Crate | What |
|---|---|
| `video-core` | Stamp, freshness, pacing, uplink budget, adaptation controller. Pure. |
| `x264` | GStreamer `thinlinkx264enc` element, linked to libx264. |
| `video` | GStreamer sender, relay ingress, receiver, ground output, feedback, metrics. |
| `policy` | Control lease, regions, follow, sightings, landing, prediction. Pure. |
| `media` | What the drone adds to the video: overlays, detector, tracker and tag taps. |
| `control` | MAVLink bridge, signed envelopes, telemetry budget, camera component. |
| `detect`, `detail` | The on-board detector; the snapshot store and the job API that carries snapshots to the ground. |
| `lab` | Link profiles, `tc` translation, the graded checks. |
| `apps/*` | The `vehicle`, `relay`, `ground` and `lab` binaries. |

## Quickstart

You need Rust (through rustup; `rust-toolchain.toml` pins the version), Podman with a compose provider, `just`, and QGroundControl. Docker works too with `CONTAINER_ENGINE=docker`.

```sh
just lab-up                    # build the images, make keys, start the testbed
just lab-check boot geo        # check the emulated link; results go to results/<run-id>/
just lab-check skeleton geo    # the live video under geo
just lab-check qgc-status geo  # the labelled video and the camera under geo
just lab-down                  # stop and remove it
```

`just check` runs format, lint and tests in a container. `just gate-full` runs every scenario.

## QGroundControl

Start QGroundControl on the same machine while the testbed runs; it connects by itself. Turn on Low Latency Mode in Application Settings, Video.

To arm and drive, take the control lease first:

```sh
just take-control
just control-status
just release-control
```

To watch without the labels:

- In QGroundControl: turn the camera component off (`docs/drone.md#qgroundcontrol`), run `just lab-up`, set Video Source to "RTSP Video Stream" and use `rtsp://127.0.0.1:8554/live`.
- With any player: `ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/live`.

## Demo with the Mac's camera

Your own camera through an emulated link:

```sh
just demo lan      # or geo, lte-good, ...
just demo-stop
```

Needs `ffmpeg` (`brew install ffmpeg`); the first run asks for camera access. The picture appears in QGroundControl with its age, for example `LIVE 0.33 s` under `geo`. `just video-hook pause` shows how stale video is marked. More in `docs/drone.md#mac-camera-demo`.

## Simulated drone

A quadcopter in a simulated world, its camera coming through the emulated link:

```sh
just sim-up          # the first run builds large images
just impair geo      # optional: pick the link
just take-control    # then fly it in QGroundControl, with a gamepad or the virtual joystick
```

If control stops, the drone brakes and holds; if the ground side goes away, it lands. It needs about 1.5 GB of memory in the Podman machine. `just lab-up` brings the rover back. The first time QGroundControl sees the quadcopter after the rover, fill in its first-run Preferences dialog once. Gamepad setup and flight modes per link: `docs/drone.md#simulated-flight`.

## Retained detail

```sh
just lab-up
id=$(just detail-request 10 0.25,0.25,0.5,0.5)   # 10 seconds ago, the middle quarter
just detail-fetch "$id" crop.jpg                 # saves the JPEG and prints its capture time and age
```

How it works: `docs/drone.md#retained-detail`.

## Real hardware

`deploy/` has the files for a Raspberry Pi as the drone, a small public server as the relay and a laptop as the ground station. `docs/hardware.md` walks through the setup and a bench checklist (props off). A Pi 4 is recommended; a Pi Zero 2 W is the smallest target.

Control reaches a real flight controller only in real mode, which is off by default, and only while its checks hold: the flight controller is not a simulator, every command is signed by your ground computer's key, the failsafe and geofence parameters match what you listed, and the drone's clock is accurate. If a check fails, the drone refuses control and says which. `docs/real-vehicle.md` has the setup and a pre-flight checklist.

## Limitations

- Right after start, or when the uplink suddenly narrows, video can fill it for about a second and late commands are refused. The drone may then brake or land by itself, and the picture reads `LOW RATE` or `AGE UNKNOWN`. See `docs/drone.md#a-filled-uplink-under-geo`.
- Video, telemetry and the drone's replies are not signed. A compromised relay can show a wrong picture or delay and drop commands, but cannot forge them or take the lease.
- The age needs synchronised clocks. The drone reports its clock accuracy from chrony; if either side's is unknown, the label reads `AGE UNKNOWN`. Real mode checks the clock only before it starts sending control.
- A USB camera that repeats its last frame shows as frozen. Other stuck cameras can still look live.

## Documentation

- [docs/video.md](docs/video.md): the video path, adaptation, freshness and latency measurement.
- [docs/testbed.md](docs/testbed.md): toolchain, builds, lab setup, checks and metrics.
- [docs/drone.md](docs/drone.md): drone features, QGroundControl, simulation and the Mac camera demo.
- [docs/hardware.md](docs/hardware.md): the Pi, relay, ground machine and bench bring-up.
- [docs/real-vehicle.md](docs/real-vehicle.md): safety interlocks and the pre-flight checklist.
- [AGENTS.md](AGENTS.md): rules for contributors and coding agents.

## Licence

The code is MIT or Apache-2.0, at your option ([LICENSE-MIT](LICENSE-MIT), [LICENSE-APACHE](LICENSE-APACHE)). The exception is `crates/x264`, which links libx264 and is GPL-2.0-or-later ([LICENSE-GPL-2.0](crates/x264/LICENSE-GPL-2.0)). The vehicle, relay, ground and lab binaries all link it through `thinlink-video`, so they are distributed under the GPL.
