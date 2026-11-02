# sat-teleop

Low-latency video and control for flying drones remotely over satellite and cellular links, with QGroundControl as the ground station.

A bench prototype. You can run all of it on one computer: a test setup emulates the link between the drone and the ground (satellite-like delay, jitter, loss, narrow bandwidth, outages) and a simulated autopilot stands in for the drone. The drone side is written for a Raspberry Pi (see "Running it on real hardware").

## What it does

- **Live video in QGroundControl, labelled with its age.** The drone stamps every frame into the picture as its software receives it, so the age is known all the way to the screen (the camera's own delay is not included). Each frame reads `LIVE`, `LOW RATE`, `STALE`, `FROZEN`, `NO VIDEO` or `AGE UNKNOWN` with that age, so old, frozen or unverifiable video never reads as live. QGroundControl finds the video as the drone's camera without setup.
- **Adaptive video.** The drone watches the link. When delay builds or the link narrows, it makes the picture smaller or slower, and pauses briefly to empty a backlog. Control and telemetry keep their share of the link.
- **Telemetry** from the autopilot to QGroundControl through the link, within a fixed budget.
- **Control under a lease.** Only one listed controller may command the drone at a time. Every command must be fresh and runs at most once. When commands stop, the drone stops.
- **Retained detail.** The drone keeps a full-size snapshot every second. Ask for one from a few seconds ago, whole or cropped: it is sent only while live video and control leave room, and is labelled as historical.
- **Graded checks.** Each scenario measures one of these under a link profile and grades it against a stored budget.

Measured in the test setup:

| | `lan` | `geo` |
|---|---|---|
| Video age at the ground | 7 ms | 320 ms |
| Command acknowledged | 18 ms | 620 ms |

Paused video reads `STALE` within 0.65 s. When the uplink drops from 512 to 128 kbit/s, the oldest frame was 1.1 s old (0.32 s of that is the link's own delay) and the picture was back to its normal age within 7.4 s. `docs/measurement.md` has the method and every number.

## Quickstart

You need Rust (through rustup; `rust-toolchain.toml` pins the version), Podman with a compose provider, `just`, and QGroundControl to watch and fly. Docker works too: set `CONTAINER_ENGINE=docker`.

```sh
just lab-up                    # build the images, make keys, start the test setup
just lab-check boot geo        # check the emulated link; results go to results/<run-id>/
just lab-check skeleton geo    # the live video under geo
just lab-check qgc-status geo  # the labelled video and the camera under geo
just lab-down                  # stop and remove it
```

For development, `just check` runs format, lint and tests in a container, and `just gate-full` runs every scenario.

## Watching and flying in QGroundControl

Start QGroundControl on the same computer while the test setup runs; it connects to the simulated drone by itself. Turn on Low Latency Mode in Application Settings, Video.

To arm and drive from QGroundControl, give it the control lease first:

```sh
just take-control
just control-status
just release-control
```

Two ways to watch without the labels, for a lower delay or a quick look:

- Direct RTSP: turn the camera component off as described in `docs/qgc.md`, run `just lab-up`, set Video Source to "RTSP Video Stream" in QGroundControl and use `rtsp://127.0.0.1:8554/live`.
- Any player: `ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/live`.

`docs/qgc.md` has the details and what QGroundControl can and cannot do here.

## Demo with the Mac's camera

See your own camera in QGroundControl after the delay, loss and limited bandwidth of a simulated link:

```sh
just demo lan      # or geo, lte-good, ...
just demo-stop
```

You need `ffmpeg` (`brew install ffmpeg`), and the first run asks to let your terminal use the camera. Start QGroundControl and the picture appears with its age, for example `LIVE 0.33 s` under `geo`. `just video-hook pause` shows how stale video is marked. `docs/demo.md` has the rest.

## Fly a simulated drone

Fly a quadcopter in a simulated world from QGroundControl, with its camera coming through the emulated link:

```sh
just sim-up          # the first run builds large images
just impair geo      # optional: pick the link
just take-control    # then fly it in QGroundControl, with a gamepad or the virtual joystick
```

If the control stops the drone brakes and holds; if the ground side goes away it lands. It needs about 1.5 GB of memory in the Podman machine. `just lab-up` brings the rover back. `docs/simulation.md` covers gamepad setup, which flight modes suit which link, and how it was measured. The first time QGroundControl meets the quadcopter after the rover it shows its first-run Preferences dialog (firmware, vehicle and units); fill it in once.

## Retained detail

To look at something you saw a few seconds ago:

```sh
just lab-up
id=$(just detail-request 10 0.25,0.25,0.5,0.5)   # 10 seconds ago, the middle quarter
just detail-fetch "$id" crop.jpg                 # saves the JPEG and says which frame it is
```

`detail-fetch` prints the frame's capture time and age. `docs/architecture.md` ("Retained detail") explains how it works.

## Running it on real hardware

The files to set up a Raspberry Pi as the drone, a small public server as the relay and a laptop as the ground station are in `deploy/`, with a step-by-step guide and a checklist for the first bench tests (props off) in `docs/deploy.md` and `docs/pi.md`. A Pi Zero 2 W is the smallest target and a Pi 4 the recommended one. Follow the checklist on the first run.

Control reaches a real flight controller only in real mode, which is off by default, and only while its safety checks hold: the flight controller is not a simulator, every control command is signed by your ground computer's key (the relay server cannot make one), the failsafe and geofence settings you listed are what the flight controller reports, and the drone's clock is known to be accurate. If any check fails the drone refuses control and says which. Whether and where you may fly is your decision and your regulator's. `docs/real-vehicle.md` has the setup, the settings and a pre-flight checklist.

## Limitations

- **A sudden drop in the uplink can briefly refuse control.** Before the drone has measured the uplink, and when a narrow link such as `geo` suddenly carries much less, video can fill the uplink for about a second and the drone refuses commands that arrive late. The picture then says `LOW RATE` or `AGE UNKNOWN`, and the drone can brake or land by itself. See `docs/simulation.md` ("A filled uplink under geo") for what was measured.
- **What you see is not authenticated.** Control commands are signed and the status and picture requests need a token, but the video, the telemetry and the drone's answers are not signed: a compromised relay server could show you a wrong picture or wrong readings, and could delay or drop your commands (it cannot forge them or take the control lease away from you). Keep the relay server secure.
- **Clocks.** Video age needs both computers' clocks to be synchronised. The drone measures how accurate its clock is from chrony and tells the ground; the ground adds its own. If either is unknown the label reads `AGE UNKNOWN`. Real mode checks the clock only before it starts sending control; a clock lost later only changes the label.
- **A stuck camera is only partly detected.** A USB camera that repeats its last frame is shown as frozen. Other cameras, and a driver that makes a repeated picture look new, still look live.

## Documentation

- [docs/real-vehicle.md](docs/real-vehicle.md): real mode, its safety checks, the keys, clocks and parameters, and the pre-flight checklist.
- [docs/deploy.md](docs/deploy.md): setting up the relay, the drone and the ground, and the bench checklist.
- [docs/pi.md](docs/pi.md): the Raspberry Pi in detail.
- [docs/qgc.md](docs/qgc.md): QGroundControl's connection, video, camera discovery and control.
- [docs/demo.md](docs/demo.md): the Mac's camera over an emulated link.
- [docs/simulation.md](docs/simulation.md): the simulated flight.
- [docs/testbed.md](docs/testbed.md): the test setup, link profiles and checks.
- [docs/measurement.md](docs/measurement.md): metrics, budgets and every measured number.
- [docs/architecture.md](docs/architecture.md): how it is built and why.
- [docs/development.md](docs/development.md): toolchain, builds, gates, code layout and adding a package.
- [AGENTS.md](AGENTS.md): rules for contributors and coding agents.
