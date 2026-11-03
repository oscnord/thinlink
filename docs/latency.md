# Latency

How old the picture is when the pilot sees it, how long a command takes to reach the drone, and where each millisecond goes. A satellite link's propagation (about 300 ms each way on `geo`) cannot be removed; everything the system adds on top of it is measured here, stage by stage. `docs/measurement.md` defines the checks and every metric.

All numbers come from the compose testbed on the development Mac (Podman machine, 5 CPUs), with the headless receiver. None is a real link, a Pi or QGroundControl on screen; the glass-to-glass measurement on hardware is in `docs/pi.md`, "What to measure first".

## Method

Each process times every frame through its own stages and serves the p50, p95 and maximum of its last 1024 frames in its metrics (`crates/video/src/stages.rs`). `lab check skeleton` keeps them all in `summary.json`, and `just stages <run>` prints them:

```sh
just lab-check skeleton geo --warmup-s 20 --duration-s 60
just stages 1793649688711-skeleton-geo
```

The window is the last 1024 frames, about 68 s at 15 frames per second, not the check's own measurement window: with a 20 s warm-up and 60 s of measurement it holds only this run's frames, while a shorter run after another profile still counts some of that profile's.

The stages are pad probes keyed by the frame's timestamp. GStreamer's own `latency` tracer would be the obvious tool, but in the runtime image's GStreamer 1.26.2 it leaves every pipeline stuck in PREROLLING.

The end-to-end number is the stamp's age at the headless receiver, which reads MediaMTX directly: vehicle, link, relay, MediaMTX and a decode. QGroundControl's picture goes one step further, through the ground adapter's decode, freshness label and re-encode; the ground's two stages below measure that step, and `lab check qgc-status` measures the age at the adapter's output.

`testbed/config/vehicle-snow.toml` is the vehicle with a noise test pattern (`VEHICLE_CONFIG=vehicle-snow.toml just lab-up`). The encoder cannot compress noise, so every rung is filled to its bitrate: the worst case for the uplink. At the low rungs it also blurs the stamp past reading, so the lossy profiles' video is measured with the normal ball pattern.

## Per stage

Snow source at the top rung, a fresh testbed, runs 1793649597475 (`lan`) and 1793649688711 (`geo`), built from `e3fb134` (x264 intra refresh on, since reverted). Milliseconds, p50 / p95 / max; percentiles of separate stages do not add up.

| Stage | `lan` | `geo` |
|---|---|---|
| Source to the stamp (before the age starts) | 0.2 / 0.4 / 0.6 | 0.3 / 0.5 / 6.8 |
| Encoder queue | 0.1 / 0.3 / 0.9 | 0.1 / 0.3 / 0.8 |
| Encode | 2.2 / 3.7 / 5.8 | 1.9 / 3.5 / 8.1 |
| Stamp to the frame's last packet sent (queue, encode and pacer) | 14.4 / 22.9 / 30.0 | 2.3 / 7.9 / 17.8 |
| The link, one way | 0 | 300 ± 10 |
| Relay: jitter buffer | 0.0 / 0.1 / 0.3 | 11.1 / 30.1 / 59.0 |
| Relay: depayload and parse | 0.0 / 0.1 / 0.2 | 0.0 / 0.0 / 0.3 |
| Headless receiver: decode | 0.5 / 0.9 / 2.6 | 0.4 / 0.8 / 3.4 |
| **Frame age at the headless receiver** | **16 / 25 / 33** | **317 / 330 / 343** |
| Ground adapter: decode | 0.5 / 0.9 / 2.2 | 0.4 / 0.8 / 2.3 |
| Ground adapter: freshness label and re-encode | 1.2 / 1.9 / 3.6 | 1.0 / 2.0 / 5.1 |

- **On `geo` the system adds about 17 ms at the median and 30 ms at p95** to the link's 300 ms. Most of it is the relay's jitter buffer: the profile's ±10 ms jitter delivers a frame's packets out of order, and the buffer waits for the missing one. The final build with the ball pattern measured 318 / 329 / 346 ms (run 1793651985136).
- **On `lan` the vehicle's pacer is the largest stage.** A noise frame fills its rung's 1.2 Mbit/s and the pacer sends at most 120 % of that with 20 ms of burst, so a frame's last packet leaves about 14 ms after its stamp at the median. That cap keeps video from queueing ahead of the control messages on a narrow link; on a fast one it is the price. On `geo` the encoder is already held to the uplink, so the pacer rarely holds a frame. A camera's frames are usually smaller than noise: the ball pattern's arrive 7 ms old on `lan` (`docs/measurement.md`).
- **MediaMTX and the hops between processes** are what the stages leave over: 1 to 3 ms.
- **QGroundControl's picture** also passes the ground adapter, about 1.5 ms at the median, and QGroundControl's own decode, which is not measured here.

## The operator's loop

What the pilot waits for: how old the picture is, and how long a command takes to reach the drone. Ball pattern for the video; the control check's ground station for the commands. The system's share is the frame age minus the link's one-way delay; with jitter it is approximate.

| Profile | Link one way, up / down | Video age p50 / p95 | System's share, p50 | Command answered, p50 (round trip) | Setpoint delay, p50 (one way) | Supports |
|---|---|---|---|---|---|---|
| `lan` | 0 / 0 | 7 / 11 ms | 7 ms | 20 ms | 1 ms | direct |
| `lte-good` | 40 ± 5 / 40 ± 5 ms | 51 / 58 ms | 11 ms | 97 ms | 39 ms | direct, at its limit |
| `lte-bad` | 60 ± 30 / 60 ± 30 ms, bursty loss | 86 / 159 ms, 3.7 new frames/s | 26 ms | 107 ms | not measured | assisted |
| `geo` | 300 ± 10 / 300 ± 10 ms | 318 / 329 ms | 18 ms | 613 ms | 299 ms | assisted, move and wait |
| `geo-degraded` | 300 ± 25 / 300 ± 10 ms, loss, 128 kbit/s up | 322 / 520 ms, 0.8 new frames/s | 22 ms | 595 ms | not measured | stop, land, return |

Runs: video `lan` 1793592510178 (`docs/measurement.md`), `lte-good` 1793651328813, `lte-bad` 1793651419489, `geo` 1793651985136, `geo-degraded` 1793651626193; commands 1793643621351, 1793643771485, 1793651831283, 1793644143516 and 1793644435204, in the same order. A command is answered when SITL's `COMMAND_ACK` is back at the ground, so its floor is both directions' delay; `lte-bad` comes in under its nominal 120 ms because its jitter goes both ways. On the lossy profiles the check's ground station sends each command once, so a lost packet is a command without an answer (1 of 7 on `lte-bad`) and the setpoint phase did not run; QGroundControl resends a command that is not answered. Their few new frames per second come from loss: a lost packet breaks the picture until the next keyframe ("What was tried").

The thresholds behind the last column: remote driving work aims at about 100 ms for direct control and about 300 ms for indirect control ([arXiv 2503.14186](https://arxiv.org/html/2503.14186v1)), and operators switch to moving and waiting from a delay of 0.25 s, with task time growing linearly with the delay ([NASA TM-85653](https://ntrs.nasa.gov/api/citations/19830027375/downloads/19830027375.pdf)). The loop is the picture's age plus the command's way out: about 8 ms on `lan`, 90 ms on `lte-good`, 140 ms on `lte-bad` and 620 ms on both `geo` profiles.

- **Direct** (Stabilize, AltHold: the sticks fly the drone) fits `lan`, and `lte-good` just.
- **Assisted** (Loiter, PosHold: the drone holds its position and the sticks move it) forgives a late input and fits the LTE profiles, and `geo` with short inputs and a pause to watch each one land.
- **Targets the drone flies to by itself** would suit `geo` best, but the vehicle refuses mission upload and Guided go-to (`docs/simulation.md`, "Which modes per profile"); on `geo-degraded`, with under a frame a second, Brake, Land and Return are what is left.

## Small boards

`PI=pi4 just lab-up` or `PI=pi-zero-2w just lab-up` caps the vehicle's companion container to roughly that board (`testbed/compose.pi4.yml`, `testbed/compose.pi-zero-2w.yml`; set the same `PI` for `just lab-check`). It is a quota of CPU time, not a slower core: a Pi 4's Cortex-A72 core at 1.5 GHz does roughly an eighth of what an Apple M-series core does, so its four cores get about half a host core, and a Pi Zero 2 W's four Cortex-A53 cores at 1 GHz about a quarter. Memory bandwidth, caches and the hardware encoder are not emulated: the testbed encodes with x264, which a Pi 4 or Zero 2 W leaves to its hardware encoder, so these runs are the pessimistic case for those boards.

Snow source, one 60 s `skeleton` run each, CPU as a share of one host core averaged over the whole run (podman's CPU time over the run's wall time), memory in use at the end. Runs 1793649597475 to 1793650946169, built from `e3fb134`, which had x264's intra refresh on (since reverted).

Only the vehicle is capped; the relay and the ground run as usual and show their own cost per rung.

| Cap | Link, rung | Frame age p50 / p95 / max | New frames/s | Encode p50 / p95 | Vehicle CPU, memory | Relay | Ground |
|---|---|---|---|---|---|---|---|
| none | `lan`, top | 16 / 25 / 33 ms | 15.0 | 2.2 / 3.7 ms | 15.0 %, 23 MB | 0.9 %, 13 MB | 4.3 %, 33 MB |
| none | `geo`, top at the uplink's rate | 317 / 330 / 343 ms | 14.6 | 1.9 / 3.5 ms | 12.1 %, 26 MB | 0.9 %, 13 MB | 3.3 %, 34 MB |
| none | `geo-degraded`, bottom | 330 / 343 / 357 ms | 0.5 | 0.8 / 2.8 ms | 3.3 %, 29 MB | 0.3 %, 13 MB | 2.3 %, 34 MB |
| Pi 4 | `lan`, top | 17 / 25 / 38 ms | 15.0 | 2.2 / 4.2 ms | 15.3 %, 23 MB | 0.9 %, 12 MB | 4.2 %, 33 MB |
| Pi 4 | `geo`, top at the uplink's rate | 317 / 330 / 386 ms | 14.6 | 1.9 / 3.9 ms | 12.4 %, 28 MB | 0.9 %, 11 MB | 3.3 %, 33 MB |
| Pi 4 | `geo-degraded`, rung 3 | 324 / 516 / 516 ms | 0.1 | 1.6 / 4.7 ms | 5.0 %, 29 MB | 0.5 %, 11 MB | 3.5 %, 34 MB |
| Pi Zero 2 W | `lan`, top | 17 / 32 / 74 ms | 15.0 | 2.2 / 8.7 ms | 13.8 %, 24 MB | 0.8 %, 18 MB | 4.0 %, 39 MB |
| Pi Zero 2 W | `geo`, top at the uplink's rate | 317 / 331 / 1350 ms | 13.3 | 1.9 / 5.9 ms | 11.6 %, 33 MB | 0.9 %, 18 MB | 3.3 %, 46 MB |
| Pi Zero 2 W | `geo-degraded`, rung 3 | no readable stamp | 0 | 0.9 / 2.3 ms | 3.7 %, 34 MB | 0.3 %, 18 MB | 2.3 %, 40 MB |

At the low rungs of `geo-degraded` the encoder blurs the noise, and with it the stamp, past reading, so those rows count few or no frames; their CPU and memory still hold.

Memory is no constraint anywhere: no process used more than 50 MB, against the 512 MB of a Pi Zero 2 W and the vehicle unit's cap of 300 MB. CPU is no constraint for the average either: the vehicle used about 15 % of a host core at the top rung with or without a cap. The Pi Zero 2 W's quarter core shows in the slowest frames instead: the quota runs out in the middle of a frame's encode and the encoder waits for the next period, so the encode's p95 doubled and single frames came late.

Minimum hardware, from these runs:

| Board | Top rung (1280x720, 15 fps, 1.2 Mbit/s) | Notes |
|---|---|---|
| Pi 4 | Fits with x264, so with the hardware encoder too | The recommended board |
| Pi Zero 2 W | Fits on average with x264; single frames late | Use the hardware encoder (`encoder = "hardware"`, the deployment default) |
| Pi 5 | Not emulated | No hardware encoder; its cores are faster than a Pi 4's, so x264 should keep up, but this is inferred, not measured |

## What was tried

**Mending the picture after loss.** When a frame is lost on the way, every frame after it is broken until the next keyframe, every 2 s. Under `geo` with the snow source, four ways to mend it, two 60 s runs each, alternated (new frames per second / unreadable stamps / longest gap without a frame):

| Configuration | Round 1 | Round 2 |
|---|---|---|
| A keyframe every 2 s (what the vehicle does) | 5.0 / 66 % / 8.6 s | 12.1 / 19 % / 3.5 s |
| A keyframe asked for by the relay when it sees loss | 7.2 / 51 % / 8.8 s | 9.6 / 33 % / 4.5 s |
| x264's intra refresh (the picture refreshed a column at a time, no keyframes) | 14.5 / 3 % / 0.8 s | 12.8 / 14 % / 6.3 s |
| A keyframe every 500 ms | 3.9 / 24 % / 1.9 s | 7.5 / 50 % / 4.0 s |

Runs 1793641433786 to 1793642472162. The runs vary a lot, but intra refresh was best in both rounds, and frame age was the same in all (p50 316 to 320 ms): mending decides how much of the video is usable, not how old it is. Intra refresh is still not used: after the first frame the encoder never sends a whole keyframe again, so a viewer that joins mid-stream cannot start decoding. The adaptation check's second viewer got no frame in 8 s (run 1793642676089), a ground adapter that restarts would stay without a picture, and recovery after the skeleton check's cut took 5.0 to 5.3 s (runs 1793649688711, 1793650771209). It becomes usable once the vehicle sends a keyframe whenever a viewer joins.

**Encoder settings.** Nothing to change; see "Recommended settings".

## Recommended settings

- **Vehicle:** `encoder = "hardware"` on a Pi 4 or Zero 2 W (`deploy/vehicle/vehicle.toml`). x264 is already set for the lowest latency: `tune=zerolatency` (no lookahead, sliced threads), `speed-preset=ultrafast`, no B-frames and a VBV buffer of one frame. Its encode took 2 ms at the median and 4 ms at p95 for a 720p frame, so there is nothing left to gain from encoder settings.
- **Relay:** nothing to set. Its jitter buffer passes packets that arrive in order straight on and waits up to 50 ms only when one is missing or out of order (`JITTER_LATENCY_MS` in `crates/media/src/ingress.rs`).
- **QGroundControl:** turn on Low Latency Mode (`docs/qgc.md`, "Video"); without it QGroundControl buffers packets for up to 80 ms and syncs its video sink.
