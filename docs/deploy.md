# Deploying sat-teleop

How to set up the three machines of a real deployment, and the bench checklist to run before anything flies. **None of this has run on hardware.** The scripts are checked for syntax and the firewall file for validity, the vehicle, relay and ground programs run in the container testbed, and the deployment's glue (provisioning, units, the real camera, the serial link, the modem) is untested. The first run is yours; follow the checklist in order and stop at the first step that fails.

**The deployed vehicle starts in SITL mode, which does not control a real vehicle.** Its control frames go to a logging sink (`docs/pi.md`, "What the Pi can control"). You get video, telemetry and the read-only parts of QGroundControl, and you can see every command it would have sent. Real mode, behind its own interlocks and never flown, is `docs/real-vehicle.md`.

## The machines

| Machine | What it runs | Needs |
|---|---|---|
| Vehicle | `sat-teleop-vehicle`, a native binary under systemd | A Raspberry Pi (Zero 2 W minimum, Pi 4 recommended; `docs/pi.md`), Raspberry Pi OS Lite 64-bit (Debian 13), a USB webcam or a Camera Module, a Pixhawk-class ArduPilot flight controller over USB or UART, a modem |
| Relay | `sat-teleop-relay`, MediaMTX and mavlink-router, containers with host networking | A public Ubuntu LTS server with an IPv4 address, 1 vCPU and 512 MB to 1 GB. The relay never transcodes. Nothing about its CPU or memory is measured |
| Ground | `sat-teleop-ground` in a container, QGroundControl natively | A Mac or a Linux laptop with Podman or Docker |

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

The relay's firewall (`deploy/relay/nftables.conf`) opens SSH and UDP 51820 to the Internet and lets each peer reach only its own ports through the tunnel. The relay also forwards the control envelopes between the vehicle and the ground (IP forwarding on, and the firewall's forward rules pass only those two UDP ports), so the vehicle's control endpoint is reached through it. The relay's Docker runs with its own iptables rules off (`provision.sh` writes `/etc/docker/daemon.json`), because every container uses host networking and Docker's default FORWARD drop would block that forwarding. **The forwarding has never run on a real relay server**: the testbed's relay has its own rules. Bring-up step 5 proves it. On a server where Docker already ran with its iptables management on, the FORWARD drop it set stays after the option is turned off: check with `sudo iptables -S FORWARD` (a `-P FORWARD DROP` line, or a `DOCKER-USER` jump) and clear it with `sudo iptables -P FORWARD ACCEPT` or a reboot.

## Images and binaries

Everything is built on the Mac, nothing needs CI. The runtime image carries all four programs; the vehicle's binary is taken out of the arm64 one, so `just vehicle-release` needs an arm64 build host (Apple Silicon or arm64 Linux) and stops if the binary it packs is not aarch64.

```sh
just deploy-keys 203.0.113.7:51820   # your relay's public address; keys and configs in deploy/secrets/
just vehicle-release                 # dist/vehicle: the arm64 binary, its checksum, configs, scripts
just relay-release                   # dist/relay: the amd64 image as a tar, its checksum, configs
just ground-release                  # dist/ground: compose file and configs, for the image built here
```

The recipes take the test setup down first (a build and the testbed do not fit the Podman machine together). `deploy/secrets/` and `dist/` are gitignored: they hold private keys, the WireGuard ones the ground adapter's control signing key (`ctl-ground.key`, which only the ground release carries; the vehicle gets the public half and refuses every control message not signed with it) and three bearer tokens, one per service (`vehicle.token`, `ground.token`, `job.token`): the vehicle release carries `vehicle.token` and `job.token`, the ground release `ground.token` and `job.token`, the relay release only `job.token`. The Pi checks the binary and the server the image against `SHA256SUMS` before installing them; nothing is piped from the network into a shell.

`just image-for arm64|amd64` builds one architecture alone. The release recipes build without the vehicle's `testbed` cargo feature, so their binaries have no test hooks and refuse a config that names `debug_hooks` or `sitl_in_real_mode`; only `just image runtime` (the testbed's image) builds with it. `ground-release` builds the release image for this machine's architecture and writes it to `dist/ground/.env`. Builds run with one cargo job under a 1 GiB memory cap (`docs/development.md`). An amd64 image is cross-built: `rustc` itself segfaults when run emulated (`qemu: uncaught target signal 11` in `rustc -vV`, measured on 2026-11-02 in the Podman machine on Apple Silicon), so the build stage compiles for `x86_64-unknown-linux-gnu` on the native architecture with the Debian cross toolchain and amd64 GStreamer headers, and only the runtime stage (apt, the mavlink-router download) runs emulated.

Measured on the development Mac (Podman machine with 5 CPUs and 2 GiB, caches warm for the compile unless noted):

| Build | Time | Peak machine memory used |
|---|---|---|
| arm64 runtime image, incremental (`just image runtime`, 2026-11-02) | 88 s | 1559 MiB |
| amd64 runtime image, emulated build (first attempt) | failed after 317 s: `rustc` SIGSEGV under qemu | 1092 MiB |
| amd64 runtime image, cross-built, first run (cross toolchain and a cold compile of the workspace; stopped at a Containerfile error after the compile) | 175 s | 1504 MiB |
| amd64 runtime image, cross-built, second run (incremental compile after a source change, then the emulated runtime stage) | 246 s | 1536 MiB |

The machine's memory figure is `free` in the machine sampled every 3 s, so it includes the baseline of about 850 MiB that the machine uses with no testbed running (another project's database among it), not only the build. No build was killed for memory. The cross-built image runs under emulation: `sat-teleop-vehicle probe-camera` and `sat-teleop-relay --help` start from it. The image is 984 MB; it has not run on an amd64 machine.

## Relay

1. Create the server (Ubuntu 24.04 LTS, public IPv4, SSH access) and note its address. Open UDP 51820 in the provider's firewall if it has one.
2. Copy `dist/relay/` to it (`scp -r`) and run `sudo ./provision.sh` there. It writes `/etc/docker/daemon.json` with Docker's iptables rules off (it stops and asks if the file exists with other content), installs Docker, WireGuard and nftables, checks and loads the image, checks the firewall file (`nft -c`) before it replaces the live rules, installs `wg0` and starts the three containers with `restart: unless-stopped`.
3. Check: `sudo wg show` (a handshake appears once the vehicle and the ground are up), `docker compose -f /opt/sat-teleop/compose.yml ps` (three running), `curl -s -H "Authorization: Bearer $(sudo cat /opt/sat-teleop/job.token)" http://127.0.0.1:9100/metrics` (`rtp_packets` grows once the vehicle sends).

The relay is the only machine with a public address, and its RTSP endpoint has no authentication of its own (the tunnel and the per-peer firewall are the boundary), and its job API needs the bearer token `job.token` (`docs/architecture.md`, "Retained detail" and "Control protocol"). The relay is trusted: a compromised relay could pose as the ground station.

## Vehicle

`docs/pi.md` has the Pi's installation, camera, encoder, flight controller, clock and measurements. In short: `just vehicle-release`, copy `dist/vehicle/` to the Pi, `sudo ./provision.sh`, edit `/etc/sat-teleop/vehicle.toml`, restart the service.

## Ground

On the operator's laptop, with QGroundControl on the same machine:

```sh
just ground-release            # builds the release runtime image here if needed
cd dist/ground
podman compose up -d           # or: docker compose -f compose.yml -f compose.docker.yml up -d
```

One container holds the WireGuard tunnel (`NET_ADMIN`, no other privilege) and the adapter runs in its network namespace. Start QGroundControl: it connects on its default UDP port 14550 and finds the video as the vehicle's camera, as in `docs/qgc.md`. `SAT_TELEOP_IMAGE` (set in the release's `.env`) names the runtime image, `localhost/sat-teleop-runtime:arm64` or `:amd64`.

The lease and the retained-detail commands run inside the adapter's container:

```sh
podman compose exec ground sat-teleop-ground take-control --adapter http://127.0.0.1:9100
podman compose exec ground sat-teleop-ground status --adapter http://127.0.0.1:9100
podman compose exec ground sat-teleop-ground detail request --ago-s 10 --relay http://10.77.0.1:8081
```

Until the ground has a fresh clock bound of its own and the vehicle has reported its, the picture reads `AGE UNKNOWN`, never `LIVE`. That is the intended first state. On Linux with chrony, install `chrony-ground.conf` on the host (it lets the container read `chronyc tracking` over chronyd's command port) and keep `clock = { chrony_at = "host.containers.internal" }`. On a Mac, whose container runs in the Podman VM with no chrony and a clock that stops over sleep, set `clock = { fixed = <ms> }` from your own measurement and restart the ground container after every wake. The bridge addresses `chrony-ground.conf` assumes, restarting chrony on a rootful host and the limits of a Mac ground are in `docs/real-vehicle.md`, "Clocks".

## Bring-up checklist

Run it on the bench, **props off and the flight controller powered from USB or a bench supply, never a battery with props on**. Write down each result. Stop at the first failure and fix it before going on.

1. **Link up.**
   - `wg show` on the relay lists a recent handshake for both peers; from the vehicle `ping -c 3 10.77.0.1` and from the ground container the same.
   - The vehicle's `systemctl status sat-teleop-vehicle` is active and `curl -s -H "Authorization: Bearer $(sudo cat /etc/sat-teleop/vehicle.token)" http://127.0.0.1:9100/metrics` has `mavlink.sitl.up` true once the flight controller is connected.
2. **Clocks.** `chronyc tracking` on the Pi and the laptop reports a synchronised clock; the vehicle reports its own bound (`mavlink.control.clock` in its metrics) and the ground shows the sum (`clock.effective_ms` in its metrics); note both over a few minutes.
3. **Video.**
   - `sat-teleop-vehicle probe-camera` lists the camera and the mode in `vehicle.toml`; `frames_stamped` and `frames_encoded` grow in the vehicle's metrics, with `stamp_failures` at 0.
   - The relay's `rtp_packets` grows; the ground's picture appears in QGroundControl with the label `AGE UNKNOWN` (no bound yet) and its age.
   - Cover the lens, unplug the camera and plug it in again; the vehicle's `last_error` shows the failure and the pipeline restarts within seconds.
4. **Telemetry.** QGroundControl shows the vehicle, its parameters load, the attitude follows tilting the flight controller by hand.
5. **A lease and a refused command.**
   - **Control reaches the vehicle through the relay.** `take-control` returns and `status` shows the lease held and a `challenge_age_ms` under 1000: the request, the grant and the challenges crossed the relay's forward rules in both directions. On the vehicle `mavlink.control.grants` is at least 1 and `challenges` grows. If `take-control` times out while `wg show` has handshakes, the relay is not forwarding: on the relay check `sudo nft list ruleset` for a `drop` policy in another table's forward chain (`sudo iptables -S FORWARD`) and that `/etc/docker/daemon.json` has both keys off and Docker was restarted.
   - Without the lease, try to arm in QGroundControl: it shows "command denied" at once.
   - `take-control`; `status` shows the lease held. Arm in QGroundControl: it reports the command as denied, nothing happens on the flight controller, the vehicle's `mavlink.control.target` reads `logging_sink`, `sunk` grows and the journal says `went to the logging sink`. A flight controller that arms here means the interlock failed: stop.
   - The flight controller's own GCS failsafe sees no ground-station heartbeat, because the vehicle does not forward one to a real autopilot. Expect it to trigger on its own if `FS_GCS_ENABLE` is on; that is a property of the flight controller's settings, not of this software.
6. **Failsafes on the bench.**
   - `sudo systemctl stop sat-teleop-vehicle`: QGroundControl shows "Comms Lost" within about 7 s and the picture goes `STALE`, then `NO VIDEO`. Start it again; both come back.
   - Cut the modem's uplink (or the relay's WireGuard port) for 10 s: the label goes `STALE` then `NO VIDEO`, and recovers by itself after the link returns, without anyone touching the vehicle.
   - Throttle the uplink on the modem or with the carrier's plan (or `tc` on the Pi's WAN interface): the vehicle's `adapt.rung` steps down and the picture stays current at a lower size or rate.
   - Reboot the Pi with the power cut: the tunnel, the clock and the vehicle come back on their own.
7. **Measure**, as in "What to measure first" below.

## What to measure first

The vehicle's four measurements, in order because each depends on the one before, are in `docs/pi.md`, "What to measure first". After them: the relay's CPU and memory with the video running and the ground adapter's CPU per output resolution. None of these is measured on real hardware, and the weak-hardware table (what the Pi Zero 2 W and a 1 vCPU relay can carry) does not exist until they are.

## Limits of a deployment

- Video age is only as good as the clock bound you set (`docs/pi.md`).
- On a narrow link such as satellite, the vehicle paces its video to the uplink it measured, so control is rarely refused. It can still be in the first seconds of a video quality tried before any uplink is measured (at the start, or after a route change) and when the uplink drops suddenly: video delays the vehicle's challenges past their lifetime and the vehicle refuses the commands that carry an old one (`docs/architecture.md`, "Control under a filled uplink"). Expect refused commands and a label of `LOW RATE` or `AGE UNKNOWN` for a second or two then, and plan which flight modes you use on such a link.
- A stuck camera is detected only on V4L2, and the check has not met a real stalled one (`docs/pi.md`, "A stuck camera").
- The relay's job API reads requests with no limit on a header's length or count and starts a thread per connection (`docs/architecture.md`, "Retained detail"). Keep it behind the tunnel and the firewall.
