# sat-teleop

Low-latency video from a drone over satellite and cellular links, shown in QGroundControl with the age of each frame.

It is a prototype. Everything runs on one machine: a container testbed emulates the link (delay, jitter, loss, narrow bandwidth, outages) and ArduPilot SITL stands in for the drone. The drone side targets a Raspberry Pi.

## Video

- The drone stamps each frame. The ground labels it `LIVE`, `LOW RATE`, `STALE`, `FROZEN`, `NO VIDEO` or `AGE UNKNOWN`, with its age. The camera's own delay is not included.
- When delay builds or the link narrows, the drone lowers the resolution or frame rate, and raises them again when the link recovers. Control and telemetry keep a share of the link.
- The drone keeps a full-size snapshot every second. You can fetch one from the last few seconds, whole or cropped, when the link has room. Snapshots taken while the link was down are kept until fetched (`just detail-outage` lists them).
- QGroundControl finds the video as the drone's camera. Any RTSP player works too.

The video code is in `crates/video-core` and `crates/video`, with nothing drone-specific in it.

## Drone

- A dashed ring shows where the camera will point by the time a command sent now takes effect.
- Telemetry reaches QGroundControl within a fixed share of the link.
- One controller at a time holds a lease. Commands must be fresh and run at most once; when they stop, the drone stops.
- Waypoint missions can continue on the autopilot's failsafe when the link is lost (off by default, see `docs/real-vehicle.md`).
- Draw a box in QGroundControl to get a full-resolution crop of it as an inset. The drone can follow what you boxed. An on-board detector marks people and vehicles and reports what it saw while out of reach (`just sightings` lists them).
- Precision landing on a printed AprilTag (off by default).

## Numbers

| | `lan` | `geo` |
|---|---|---|
| Video age at the ground | 7 ms | 320 ms |
| Command acknowledged | 18 ms | 620 ms |

Paused video shows `STALE` within 0.65 s. When the uplink drops from 512 to 128 kbit/s, the oldest frame is 1.1 s old (0.32 s of that is the link's own delay) and the age is back to normal within 7.4 s. Method and all numbers: `docs/measurement.md`.

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

- In QGroundControl: turn the camera component off (`docs/qgc.md`), run `just lab-up`, set Video Source to "RTSP Video Stream" and use `rtsp://127.0.0.1:8554/live`.
- With any player: `ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/live`.

## Demo with the Mac's camera

Your own camera through an emulated link:

```sh
just demo lan      # or geo, lte-good, ...
just demo-stop
```

Needs `ffmpeg` (`brew install ffmpeg`); the first run asks for camera access. The picture appears in QGroundControl with its age, for example `LIVE 0.33 s` under `geo`. `just video-hook pause` shows how stale video is marked. More in `docs/demo.md`.

## Simulated drone

A quadcopter in a simulated world, its camera coming through the emulated link:

```sh
just sim-up          # the first run builds large images
just impair geo      # optional: pick the link
just take-control    # then fly it in QGroundControl, with a gamepad or the virtual joystick
```

If control stops, the drone brakes and holds; if the ground side goes away, it lands. It needs about 1.5 GB of memory in the Podman machine. `just lab-up` brings the rover back. The first time QGroundControl sees the quadcopter after the rover, fill in its first-run Preferences dialog once. Gamepad setup and flight modes per link: `docs/simulation.md`.

## Retained detail

```sh
just lab-up
id=$(just detail-request 10 0.25,0.25,0.5,0.5)   # 10 seconds ago, the middle quarter
just detail-fetch "$id" crop.jpg                 # saves the JPEG and prints its capture time and age
```

How it works: `docs/architecture.md`, "Retained detail".

## Real hardware

`deploy/` has the files for a Raspberry Pi as the drone, a small public server as the relay and a laptop as the ground station. `docs/deploy.md` and `docs/pi.md` walk through the setup and a bench checklist (props off). A Pi 4 is recommended; a Pi Zero 2 W is the smallest target.

Control reaches a real flight controller only in real mode, which is off by default, and only while its checks hold: the flight controller is not a simulator, every command is signed by your ground computer's key, the failsafe and geofence parameters match what you listed, and the drone's clock is accurate. If a check fails, the drone refuses control and says which. `docs/real-vehicle.md` has the setup and a pre-flight checklist.

## Limitations

- Right after start, or when the uplink suddenly narrows, video can fill it for about a second and late commands are refused. The drone may then brake or land by itself, and the picture reads `LOW RATE` or `AGE UNKNOWN`. See `docs/simulation.md`, "A filled uplink under geo".
- Video, telemetry and the drone's replies are not signed. A compromised relay can show a wrong picture or delay and drop commands, but cannot forge them or take the lease.
- The age needs synchronised clocks. The drone reports its clock accuracy from chrony; if either side's is unknown, the label reads `AGE UNKNOWN`. Real mode checks the clock only before it starts sending control.
- A USB camera that repeats its last frame shows as frozen. Other stuck cameras can still look live.

## Documentation

- [docs/latency.md](docs/latency.md): picture and command age per link, where the time goes, settings for the lowest delay.
- [docs/architecture.md](docs/architecture.md): how it is built and why.
- [docs/measurement.md](docs/measurement.md): metrics, budgets and measured numbers.
- [docs/testbed.md](docs/testbed.md): the testbed, link profiles and checks.
- [docs/qgc.md](docs/qgc.md): QGroundControl's connection, video, camera and control.
- [docs/demo.md](docs/demo.md): the Mac's camera over an emulated link.
- [docs/simulation.md](docs/simulation.md): the simulated flight.
- [docs/real-vehicle.md](docs/real-vehicle.md): real mode, its checks, keys, clocks, parameters and pre-flight checklist.
- [docs/deploy.md](docs/deploy.md): setting up the relay, the drone and the ground.
- [docs/pi.md](docs/pi.md): the Raspberry Pi.
- [docs/development.md](docs/development.md): toolchain, builds, gates and code layout.
- [AGENTS.md](AGENTS.md): rules for contributors and coding agents.

## Licence

MIT or Apache-2.0, at your option ([LICENSE-MIT](LICENSE-MIT), [LICENSE-APACHE](LICENSE-APACHE)).
