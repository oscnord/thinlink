# Testbed

Build the workspace, run the container lab and interpret its checks here. The video path is in [video.md](video.md). Ground-station operation, control and simulated flight are in [drone.md](drone.md). Deployment is in [hardware.md](hardware.md). Real-vehicle interlocks remain in [real-vehicle.md](real-vehicle.md).

The lab controls SITL only. A container check is not hardware, real-link or QGroundControl rendering evidence.

## Development setup

The tool list, workspace layers, lint policy, package rules and commit rules are in [AGENTS.md](../AGENTS.md). Application entry points are in `apps/`, libraries in `crates/`, container setup in `testbed/`, deployment files in `deploy/`, verified fixture provenance in `fixtures/manifest.toml` and acceptance budgets in `perf/budgets.toml`.

## Toolchain

`rust-toolchain.toml` pins Rust 1.99.0 with rustfmt and Clippy. The workspace uses edition 2024 and resolver 3. Container builds use `docker.io/library/rust:1.99.0-trixie`, with `docker.io/library/debian:trixie-slim` as the runtime base.

The build image includes `libx264-dev` and `libclang-dev`. `crates/x264/build.rs` uses pkg-config to find libx264 and bindgen to generate bindings from the system `x264.h`.

The pin only takes effect when `cargo` is a rustup proxy. On some machines `~/.cargo/bin/cargo` is a plain symlink to another toolchain, so the pin is ignored and builds fail on `rust-version = "1.99"`. Check with:

```sh
cargo --version
rustup which cargo
```

Either run through rustup explicitly:

```sh
rustup toolchain install 1.99.0 --component rustfmt --component clippy
rustup run 1.99.0 cargo test -p sat-teleop-video-core --locked
```

or put the toolchain's `bin` directory first on `PATH`. On an Apple Silicon Mac:

```sh
PATH=$HOME/.rustup/toolchains/1.99.0-aarch64-apple-darwin/bin:$PATH cargo test -p sat-teleop-video-core --locked
```

Do not rewrite your global toolchain setup just for this project.

## Builds and gates

Pure crates build and test natively. GStreamer-linked packages (`sat-teleop-x264`, `sat-teleop-video`, `sat-teleop-media`, `sat-teleop-lab` and the applications) build in the build container. The host does not need GStreamer.

| Recipe | Purpose |
|---|---|
| `just check` | Full workspace fmt, Clippy, tests, layer checks and the unsafe-code boundary check in `localhost/sat-teleop-build:dev`, including control and vehicle checks with the `testbed` feature. |
| `just check-native` | Host gates for packages without GStreamer, including control's `testbed` feature. Needs `jq` and Bash and uses the pinned toolchain even when `cargo` is not a rustup proxy. |
| `just fmt` | Format the workspace on the host. |
| `just image <target>` | Build `runtime` or `build` from `testbed/Containerfile`. The runtime includes vehicle test hooks. |
| `just sitl-image` | Build the linux/amd64 ArduRover SITL image. |
| `just image-for <arch>` | Build an `arm64` or `amd64` release runtime without the `testbed` feature. See [hardware.md](hardware.md). |
| `just gate-fast <scenario> <profile> [args]` | Rebuild if inputs or running services changed, then run one check. |
| `just gate-full` | Bring up a fresh testbed and run the full default scenario set below. |

Container builds and `just check` use `BUILD_JOBS` cargo jobs under the `BUILD_MEMORY` cap, 1 and `1g` by default for a 2 GiB Podman machine. On a larger machine, use for example `BUILD_MEMORY=4g BUILD_JOBS=4 just check`. Stop the testbed before a build or `just check`. Gazebo must also be down. `lab-up` and `gate-fast` stop it before rebuilding.

`gate-fast` hashes the Cargo files, `crates`, `apps`, `xtask`, `fixtures` and `testbed/Containerfile`. It compares the hash, running images and service count with `.cache/gate-fast.inputs`. A difference runs `lab-up`; otherwise it reuses the testbed.

`gate-full` runs every scenario even if one fails, then exits 1 if any did not pass: `boot lan`, `skeleton lan`, `skeleton geo`, `mavlink lan`, `control-freshness lan`, `qgc-status lan`, `qgc-status geo`, `qgc-status outage-10s`, `adaptation bandwidth-steps`, `recording lan`, `retrieval geo`, `real-mode lan`, then `ingest-isolation`. The optional simulation, regional and picture-quality checks are separate. Use targeted checks for a focused change; gate selection is in [AGENTS.md](../AGENTS.md#8-build-test-and-lint).

Builds use a registry cache and a separate cargo target cache for each checkout and build architecture. Retire a checkout's build output with `just forget-checkout <folder>` from another checkout.

The workspace sets `unsafe_code = "deny"`. `just check` rejects any `.rs` file under `crates`, `apps` or `xtask` that mentions `unsafe_code` outside `crates/x264`. The permitted FFI scope is in [AGENTS.md](../AGENTS.md#unsafe-code).

## Image cache

`just image runtime` tags the build stage so its dependency layers remain reachable. Apt installs use cache mounts. Podman reads the build context through the VM's file share, which needs free 512 KiB memory blocks. If fewer than 16 are available, the image recipes drop clean page cache and compact memory before building. This avoids `readdirent: cannot allocate memory` from a fragmented VM; other containers reread clean cached files from disk.

## CI

[.github/workflows/ci.yml](../.github/workflows/ci.yml) defines pull-request, `main` push and manual runs. It skips documentation-only changes, cancels superseded runs and has a 15-minute job timeout. The workflow runs fmt, Clippy, tests and layer checks. Repository settings determine whether GitHub Actions executes it.

## xtask

`cargo xtask <command>` is an alias for `cargo run -q -p sat-teleop-xtask --` (see `.cargo/config.toml`).

- `cargo xtask layers` loads the workspace with `cargo metadata` and checks it against the registry in `xtask/src/layers.rs`. It reports every violation and exits 1 if there are any: unregistered members, missing `[lints] workspace = true`, upward or sideways dependencies, dependencies on apps or tooling, non-dev use of `sat-teleop-testkit`, forbidden families in `sat-teleop-policy`, GStreamer outside `sat-teleop-x264`, `sat-teleop-video`, `sat-teleop-media` and apps. It checks dependencies as declared, including optional and target-specific ones. `sat-teleop-x264` sits at L0 beside `sat-teleop-video-core`; `sat-teleop-video` depends on both.
- `cargo xtask fixtures` reads `fixtures/manifest.toml`, downloads each missing fixture into `.cache/fixtures/` and verifies it. `cargo xtask fixtures --check` only verifies files already present and fails on a missing one.

## Fixtures

`fixtures/manifest.toml` lists test media. Every entry requires `name`, `url`, `sha256`, `size_bytes`, `license`, `source` and `notes`; unknown fields are rejected. A download is capped at `size_bytes + 1` bytes, written to a `.part` file, checked against both size and SHA-256, then renamed. Any failure removes the partial file and exits non-zero. Downloaded files stay in `.cache/` and are never committed.

To add a fixture, download it once, compute its SHA-256 and size yourself, record the real source and license wording, then run `cargo xtask fixtures` and `--check`. Required fixtures that are missing or corrupt must fail a check, never skip it.

The fixture is `foreman_cif.y4m` (352x288, 300 frames, from the Xiph.org derf collection).

## Small-board limits

`PI=pi4 just lab-up` and `PI=pi-zero-2w just lab-up` select `testbed/compose.pi4.yml` and `testbed/compose.pi-zero-2w.yml`. Set the same `PI` for `just lab-check`. Only the companion is capped; relay and ground run normally.

| Overlay | CPU quota | Memory limit |
|---|---|---|
| `pi4` | 0.5 host core | 4 GiB |
| `pi-zero-2w` | 0.25 host core | 512 MiB |

These are CPU-time quotas on host cores, not emulated Pi cores. Memory bandwidth, caches and the hardware encoder are not emulated. The testbed uses x264; physical-board performance requires [hardware measurements](hardware.md).

## Lab setup

The testbed is compose project `sat-teleop` (`testbed/compose.yml`). A router container replaces the cellular or GEO-like link: it forwards and NATs the vehicle network onto the ground network and impairs each direction with `tc` through the router agent (`lab router`). Every node joins a WireGuard overlay whose server is the relay, so vehicle traffic reaches the relay inside WireGuard through the router, as it would over a carrier network.

Applications share a sidecar's network namespace (`network_mode: service:<sidecar>`). The services table below gives each namespace.

All values in the shipped profiles are synthetic (`provenance = "synthetic"`).

## Topology

```text
 vehicle_net 192.168.231.0/24 (internal)            ground_net 192.168.232.0/24
+----------------------------+                     +--------------------------------+
| vehicle-net .10            |       router        | relay-net .20                  |
|  wg0 10.77.0.2             |    .254  |  .254    |  wg0 10.77.0.1, :51820/udp     |
|  default via .254 ---------+--> forward, NAT, ---+-> forwards between peers       |
|  companion: RTP -> :5004   |    tc per direction |  relay :5004 -> mediamtx :8554 |
+----------------------------+                     | ground-net .30   wg0 10.77.0.3 |
                                                   | checks .40       wg0 10.77.0.4 |
                                                   +--------------------------------+
 ctl_net 192.168.233.0/24 (internal, orchestration and metrics, never data):
   router .254 (agent :9000), vehicle-net .10, relay-net .20, ground-net .30, checks .40
```

The router is the only container on both data networks. It masquerades `vehicle_net` onto its `ground_net` address and drops everything else it would forward, so ground hosts can only answer flows the vehicle opened (carrier-grade NAT) and `ctl_net` is never forwarded. The rules are in `testbed/scripts/router.nft`.

`ctl_net` is one shared network, so without a filter a ground-side container could reach the vehicle's `ctl_net` address directly, unimpaired, and silently invalidate every latency measurement.

Every sidecar and `checks` therefore loads `testbed/scripts/ctl.nft` on its `ctl_net` interface (found by address, passed to `nft -D ctl_if=...`): only TCP to the router agent (9000) and to app metrics (9100) and their replies are allowed; everything else, UDP and ICMP included, is dropped, and nothing is forwarded to or from `ctl_net`. `checks` may additionally send anything from its own `ctl_net` address so the boot check can prove the filter holds on the vehicle.

The router itself is not filtered on `ctl_net`; its forward policy already drops that traffic.

Local DNS lookups are unaffected: queries to the `ctl_net` gateway fail at once and the resolver moves on to the next server.

SITL listens on every address of the vehicle namespace: MAVLink on tcp 5760 (serial 0), 5762 and 5763 (serial 1 and 2), and RC input on udp 5501. Without a filter, any overlay peer (ground or `checks`) could reach the autopilot directly, past everything the companion enforces. The `vehicle-net` sidecar therefore also loads `testbed/scripts/sitl.nft` (named by `EXTRA_NFT`), which drops those ports on every interface but loopback.

A camera outside the containers (the Mac's webcam, `drone.md`) reaches the vehicle through a fourth network, `ingest_net`, and its video then takes the impaired link like any frame. It must not become a way around the link, so it carries one thing: the sidecar loads `testbed/scripts/ingest.nft` (found by address like `ctl_if`) on its interface, which accepts only UDP to port 5700 and drops everything else arriving on it, everything leaving through it and everything forwarded to or from it. The sidecar also drops the default route the network adds, so the vehicle's traffic only leaves through the router. Port 5700 is published on host loopback only.

The vehicle (`testbed/config/vehicle-ingest.toml`) then accepts datagrams from the addresses listed in its source's `from` only: a Podman machine's host gateway (192.168.127.1, where the Mac's traffic comes from through the published port), the network's gateway (192.168.234.1, a Linux host) and the feeder. `just ingest-isolation` proves it (below).

| Network | Subnet | Internal | Members |
|---|---|---|---|
| `vehicle_net` | 192.168.231.0/24 | yes | router .254, vehicle-net .10 |
| `ground_net` | 192.168.232.0/24 | no (host access, port publishing) | router .254, relay-net .20, ground-net .30, checks .40 |
| `ctl_net` | 192.168.233.0/24 | yes | router .254, vehicle-net .10, relay-net .20, ground-net .30, checks .40 |
| `ingest_net` | 192.168.234.0/24 | no (port publishing) | vehicle-net .10, feeder .5 (only while started) |

| Service | Addresses | Overlay | Privileges | Runs |
|---|---|---|---|---|
| `router` | .231.254, .232.254, .233.254 | none | `NET_ADMIN`, `net.ipv4.ip_forward=1`, `init` | `router.sh`: checks forwarding, loads `router.nft`, then `exec lab router --listen 192.168.233.254:9000` |
| `vehicle-net` | .231.10, .233.10, .234.10 | 10.77.0.2/24 | `NET_ADMIN` | `sidecar.sh` (route, `ctl.nft`, `sitl.nft`, `ingest.nft`, wg0) with `ROUTE="default via 192.168.231.254"` and `EXTRA_NFT="/scripts/sitl.nft /scripts/ingest.nft"`; publishes 127.0.0.1:5700/udp |
| `relay-net` | .232.20, .233.20 | 10.77.0.1/24, listens on 51820/udp | `NET_ADMIN`, `net.ipv4.ip_forward=1` | `sidecar.sh` |
| `ground-net` | .232.30, .233.30 | 10.77.0.3/24 | `NET_ADMIN` | `sidecar.sh` |
| `checks` | .232.40, .233.40 | 10.77.0.4/24 | `NET_ADMIN` | `sidecar.sh` with `ROUTE="192.168.231.0/24 via 192.168.232.254"`; `lab check` runs here |
| `feeder` | .234.5 (and .234.6, an alias) | none | `NET_ADMIN`, profile `feeder` | `feeder.sh`: a 720p30 test pattern as RTP/H.264 to the ingest port, from .5 (accepted) and a trickle from .6 (refused). Started by `just ingest-check`, not by `lab-up` |
| `companion` | vehicle-net's | vehicle-net's | none | `sat-teleop-vehicle run --config /config/vehicle.toml --limits /config/limits.toml --ladder /config/ladder.toml --metrics-addr 192.168.233.10:9100 --token-file /secrets/vehicle.token --job-token-file /secrets/job.token` (`VEHICLE_CONFIG=vehicle-ingest.toml` mounts the ingest config as `vehicle.toml`) |
| `relay` | relay-net's | relay-net's | none | `sat-teleop-relay run --config /config/relay.toml --metrics-addr 192.168.233.20:9100 --token-file /secrets/job.token` |
| `mediamtx` | relay-net's | relay-net's | none | `bluenviron/mediamtx:1.21.3` with `testbed/mediamtx.yml` |
| `mavlink-router` | relay-net's | relay-net's | none, `init`, `restart: on-failure` | `mavlink-routerd -c /config/mavlink-router.conf` (`testbed/mavlink-router.conf`) |
| `ground` | ground-net's | ground-net's | none | `sat-teleop-ground run --config /config/ground.toml --limits /config/limits.toml --metrics-addr 192.168.233.30:9100 --token-file /secrets/ground.token --job-token-file /secrets/job.token` |
| `sitl` | vehicle-net's | vehicle-net's | none, `init` | `localhost/sat-teleop-sitl:dev` (linux/amd64): `ardurover --model rover --defaults /sitl/rover.parm,/sitl/sat-teleop.parm --speedup 1 -I0 --home 59.3293,18.0686,10,0` |

| Port | Where | Purpose |
|---|---|---|
| 51820/udp | 192.168.232.20 | WireGuard server on the relay |
| 9000/tcp | 192.168.233.254 | Router agent, `ctl_net` only |
| 9100/tcp | 192.168.233.10, 192.168.233.20, 192.168.233.30 | `GET /metrics` of sat-teleop-vehicle, sat-teleop-relay and sat-teleop-ground, and sat-teleop-ground's lease endpoints; the only other port `ctl.nft` admits |
| 5004/udp | 10.77.0.1 | RTP/H.264 from the vehicle; the relay binds the overlay address only |
| 5006/udp | 10.77.0.1 | The relay's feedback to the vehicle's probes ([video path](video.md)); overlay address only |
| 5700/udp | 192.168.234.10, published on host 127.0.0.1:5700 | RTP/H.264 from an external camera into the vehicle's ingest source (`vehicle-ingest.toml` only), from the addresses in `from` only |
| 5760/tcp | vehicle namespace, loopback only (`sitl.nft`) | SITL serial 0, MAVLink for the companion |
| 14550/udp | 10.77.0.1 | mavlink-router's server endpoint for the vehicle, overlay address only |
| 14551/udp | 10.77.0.3 | sat-teleop-ground's overlay socket; mavlink-router's client endpoint sends here |
| 14552/udp | 192.168.232.40 | `lab check mavlink` and `control-freshness` in `checks`, a ground station of sat-teleop-ground |
| 14553/udp | 10.77.0.4 | `lab check control-freshness` in `checks`: SITL's telemetry straight from mavlink-router |
| 7700/udp | 10.77.0.2 | The vehicle's control endpoint (control envelopes, lease, challenges) |
| 7701/udp | 10.77.0.3 | sat-teleop-ground's controller, the lease holder's socket |
| 14550/udp | the Mac, as `host.containers.internal` | QGC's default UDP link; sat-teleop-ground sends telemetry there and hears QGC's replies on the same socket, so nothing is published; see [drone.md](drone.md) |
| 5600/udp | the Mac, as `host.containers.internal` | QGC's default UDP video port; sat-teleop-ground's freshness output (RTP/H.264) goes there through gvproxy; see [drone.md](drone.md) |
| 5602/udp | 192.168.232.40 | `lab check qgc-status` in `checks`: the same output |
| 8080/tcp | 10.77.0.1 | The relay's job API for the vehicle (`GET /jobs/next`, chunk uploads, `complete`, `fail`): overlay address only, bearer token required ([retained detail](drone.md)) |
| 8080/tcp | 192.168.232.20 | The relay's job API for the ground and `checks` (`POST /jobs`, `GET /objects/{id}`, `DELETE /jobs/{id}`): its `ground_net` address only, bearer token required |
| 8554/tcp | relay-net, published on host 127.0.0.1:8554 | MediaMTX RTSP, path `live` |
| 8000-8001/udp | 192.168.232.20 | MediaMTX RTP/RTCP to RTSP readers over UDP |
| 8889/tcp | relay-net, published on host 127.0.0.1:8889 | MediaMTX WebRTC page and WHEP |
| 8189/udp | relay-net, published on host 127.0.0.1:8189 | MediaMTX WebRTC media (ICE); `127.0.0.1` is offered as a candidate |

Published ports always bind to `127.0.0.1`, through the `relay-net` sidecar because MediaMTX shares its namespace. The job API's port 8080 is not published.

The vehicle's snapshot store is the volume `snapshots` (a 96 MiB tmpfs, `testbed/compose.yml`) mounted at `/snapshots` in `companion` and, to look at the files, in `checks`. `lab-down` leaves the volume; the files are gone once no container mounts it.

## Privileges

No container is privileged. `NET_ADMIN` is granted only to the router, the sidecars and `checks`; application containers will inherit a sidecar's namespace without it. Forwarding is enabled with compose `sysctls` on the router and the relay (which routes between overlay peers); `router.sh` refuses to start if it is off. Sidecars set `net.ipv4.ping_group_range` to `0 0` so root can ping over ICMP datagram sockets instead of holding `NET_RAW` (Podman's default disables them). The router runs under `init: true` so `lab router` stops on `SIGTERM`; the sidecar script traps it itself.

## WireGuard overlay and keys

The VM kernel provides WireGuard, so the sidecars use kernel WireGuard through `wg-quick`; nothing runs in userspace. Each client peer (vehicle, ground, checks) points at `192.168.232.20:51820` with `AllowedIPs = 10.77.0.0/24` and `PersistentKeepalive = 15`. The relay lists each client with its own `/32` and forwards between them, so ground and checks reach the vehicle through the relay.

The vehicle's tunnel crosses the router, which masquerades it: the relay sees the vehicle at `192.168.232.254`.

`just lab-keys` runs `testbed/scripts/keys.sh` in a runtime container. It writes `testbed/secrets/<node>.key`, `.pub` and a wg-quick `<node>.conf` for `relay`, `vehicle`, `ground` and `checks` (mode 600), one bearer token per service, mode 600 (`vehicle.token`: the vehicle's endpoints, `ground.token`: the adapter's, `job.token`: the relay's metrics and job API, which the vehicle and the adapter also present; each application mounts only what it serves or presents and `checks` all three), and the control signing keys `ctl-ground.key`/`.pub` and `ctl-checks.key`/`.pub` (`lab keygen`): the ground adapter and the lab runner sign their envelopes, the companion mounts the two public keys.

Existing keys are kept and the configs are rewritten from them, so the recipe is idempotent. `testbed/secrets/` is gitignored and never committed. Each sidecar mounts only its own config, read-only, as `/secrets/wg0.conf`.

To rotate keys, delete `testbed/secrets/` and run `just lab-up` again.

## Recipes

```sh
just lab-keys                 # WireGuard keys and configs into testbed/secrets/
just lab-up                   # stop the testbed, build images and keys, then start
just lab-check boot lan       # lab check boot --profile lan in the checks container
just lab-check skeleton geo   # the live video path under geo
just lab-check mavlink geo    # the MAVLink path under geo
just lab-check control-freshness geo  # the control lease and command freshness under geo
just lab-check qgc-status geo # the ground's freshness output and camera under geo
just lab-check adaptation bandwidth-steps            # the vehicle's adaptation to uplink rate steps
just lab-check adaptation lte-bad --duration-s 120   # further arguments go to `lab check`
just ingest-check lan         # the camera ingest path with a test-pattern feeder in place of a webcam
just video-hook pause         # the vehicle's test-only video hooks: pause, freeze, resume
just lab-down                 # remove the sat-teleop containers and networks
```

`just lab-up` first runs `just lab-down`, then the image builds (`runtime` and `sitl-image`); cached layers make them quick when nothing changed, and source changes rebuild the binaries.

The testbed goes down first because services share sidecar network namespaces, and Podman cannot recreate a sidecar (for a new image) while services still use it, and because a build never starts beside a running testbed.

On a Podman machine it then makes sure forwarding is on for `ground_net`'s bridge (see "macOS and Podman notes"). `just lab-check <scenario> <profile> [args]` runs `lab check` in `checks` with the working directory `/sat-teleop`, where `testbed/profiles/` is mounted read-only and `results/` is the repository's `results/` directory. It passes `git describe --always --dirty` as the revision and exits with the check's code.

Only the host can kill a container, so when a check prints the line `lab: kill ground` on stdout (`control-freshness` does, mid-drive), the recipe runs `compose kill ground` and `compose start ground` and passes every other line through; the adapter restarts at once, without a lease.

Cleanup: `just lab-down` runs `compose down --remove-orphans`, which removes only the `sat-teleop` containers and networks, and on a Podman machine the leaked netavark drop-ins ("macOS and Podman notes"). Images (`localhost/sat-teleop-*`), `testbed/secrets/` and `results/` stay. Remove them explicitly if wanted, for example `podman rmi localhost/sat-teleop-runtime:dev`; never prune the shared machine.

To apply a profile by hand, go through `checks` (the agent is on `ctl_net` only):

```sh
podman compose -p sat-teleop -f testbed/compose.yml exec checks \
  lab impair --agent http://192.168.233.254:9000 --profile testbed/profiles/geo.toml
```

## Test hooks and metrics

`testbed/config/vehicle.toml` sets `debug_hooks = true`, which only a build with the `testbed` cargo feature accepts (`just image runtime` passes it; the release images do not, and their vehicle refuses a config that names it): the vehicle's metrics endpoint then takes `POST /video/pause` (frames are dropped before the encoder), `/video/freeze` (the next frame is sent again and again, stamp and all) and `/video/resume`, which `lab check qgc-status` and `just video-hook <action>` use; without it they answer 403.

All three applications restart a failed pipeline after one second (for example until `wg0` is up) and report counters and health (`runs`, `running`, `last_error`) on `GET /metrics` (the vehicle's also carries its MAVLink counters under `mavlink`, the ground's its video under `video`):

```sh
podman compose -p sat-teleop -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/vehicle.token)" http://192.168.233.10:9100/metrics'
podman compose -p sat-teleop -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/job.token)" http://192.168.233.20:9100/metrics'
podman compose -p sat-teleop -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/ground.token)" http://192.168.233.30:9100/metrics'
```

`sat-teleop-ground` reports its overlay socket (`listen`), where mavlink-router was last heard from (`relay_endpoint`), per direction (`telemetry` towards the stations, `gcs` from them) messages and bytes in and out and drops by reason, its camera's counts (`camera`) and its video output (`video`: both pipelines' health, frames decoded, new and unreadable, frames sent by state, the current state); `apps/ground/AGENTS.md` lists every field.

## Checks

Run checks through `just lab-check <scenario> <profile> [args]`. Each section below defines what the check does and grades. [Result files](#result-files) hold the measurements and effective settings; [verdicts and budgets](#verdicts-and-budgets) define their interpretation.

### Boot check

`lab check boot --profile <name>` applies the profile and checks that the topology works. It runs entirely inside `checks`, which is a ground host, sits on `ctl_net` and joins the overlay as 10.77.0.4. Running everything there gives one runner, one `summary.json` and one exit code, with no assertions in shell. Facts that `checks` cannot see directly are proven by behaviour instead: a reply from 10.77.0.2 means the vehicle and relay have a WireGuard session, and a forwarding cut that silences 10.77.0.2 means the vehicle's only path to `ground_net` is the router.

| Assertion | Pass when |
|---|---|
| `agent_reachable` | `GET /timeline` on `192.168.233.254:9000` answers. |
| `profile_applied` | `POST /profile` succeeds; its report is stored as `effective_profile`. |
| `agent_only_on_ctl_net` | Nothing accepts a connection on `192.168.232.254:9000`. |
| `overlay_ground` | 10.77.0.3 replies within 30 s (handshakes retry every 5 s after a fresh start). |
| `overlay_vehicle` | 10.77.0.2 replies within 30 s. |
| `overlay_rtt` | At least 5 of 10 pings to 10.77.0.2 are answered (otherwise `incomplete`) and their median lies within the configured round trip (uplink plus downlink `delay_ms`) minus 50 ms to plus 100 ms. |
| `cgnat` | None of 10 pings to 192.168.231.10 is answered. `checks` routes `vehicle_net` via the router, so this exercises the router's filter, not just Podman's internal-network isolation. |
| `ctl_net_carries_no_data` | 192.168.233.254 (the unfiltered router) answers a ping from `checks`, proving the probe path, and then none of 10 pings to the vehicle's `ctl_net` address 192.168.233.10 is answered. |
| `forwarding_cut` | After `POST /forwarding {"enabled": false}`, none of 10 pings to 10.77.0.2 is answered. |
| `forwarding_recovery` | After `POST /forwarding {"enabled": true}`, 10.77.0.2 replies within 25 s (one keepalive plus margin). |

Probes are ICMP echo with a 2 s reply timeout. Waiting for a reply (`overlay_*` and `forwarding_recovery`) sends one probe at a time, at most every 0.5 s, until the first reply or the deadline, so a lost probe, an ICMP error or a NAT mapping that only comes back with the next keepalive cannot end the wait early. Silence (`cgnat`, `ctl_net_carries_no_data`, `forwarding_cut`) is a stream of 10 probes 0.2 s apart that must all go unanswered; a stream that could not be sent in full is `incomplete`. A few lost probes on a lossy profile therefore cannot make an open path look silent.

The ping from `checks` to 10.77.0.2 travels to the relay unimpaired, then through the router both ways, so its RTT covers both impaired directions. If the vehicle does not answer, `cgnat`, `ctl_net_carries_no_data` and the cut assertions are `incomplete` rather than passed, because silence from a dead vehicle proves nothing. Forwarding is restored even when the cut step fails. A short cut recovers within one round trip because the router's NAT mapping survives it (keepalives keep it fresh); the 25 s deadline covers a keepalive re-creating the mapping after a longer one.

| Metric | Definition |
|---|---|
| `overlay_rtt_median_ms` | Median RTT of the answered overlay probes. |
| `forwarding_recovery_ms` | From restoring forwarding to the first overlay reply. |

### Skeleton check

`lab check skeleton --profile <name>` measures the live video path from `checks`: it applies the profile, plays `rtsp://192.168.232.20:8554/live` in a headless receiver configured like QGC (RTSP over UDP, 80 ms latency, `avdec_h264`, non-syncing sink) and reads the stamp of every decoded frame. Options: `--warmup-s` (default 5), `--duration-s` (default 30), `--url`, `--vehicle-metrics` and `--relay-metrics` (the metrics base URLs).

| Assertion | Pass when |
|---|---|
| `profile_applied` | The router agent applied the profile. |
| `ladder_settled` | The vehicle's ladder settled before measuring ([adaptation check](#adaptation-check)); `incomplete` otherwise: the run would measure a transient, or a rung the profile does not hold it at. |
| `video_received` | At least one new frame (readable, plausible stamp, unseen frame id) arrived in the window. |
| `forwarding_cut` | After `POST /forwarding {"enabled": false}` and a drain time, no frame arrives for 2 s. `incomplete` without video before the cut. |
| `video_recovery` | After restoring forwarding, a fresh frame (captured after the cut began) arrives within 15 s. |
| `budget`, `budget.<metric>` | The run is graded against `perf/budgets.toml`: `incomplete` in calibration or without a budget, otherwise one assertion per limit. |

The receiver restarts after a pipeline error, one second apart. Router and app counters are read around the measurement window. The cut drain is the largest frame age in the window plus 0.5 s, capped at 10 s. Recovery requires a readable stamp captured after the cut began and received after the restore.

Window metrics use observations received inside the measurement window. A new frame is an observation with a readable stamp, a plausible age (0 to 60 s; anything else is a misread stamp or a clock fault) and a frame id not seen before in the window. Only new frames count towards age and cadence; a repeated display of the same frame is not fresh video.

| Metric | Definition |
|---|---|
| `frame_age_p50_ms`, `frame_age_p95_ms`, `frame_age_max_ms` | Nearest-rank percentiles of the age of new frames. |
| `new_frames_per_s` | New frames divided by the window length. |
| `max_frame_gap_ms` | Longest wait for a new frame, counted from the window start to the first frame and from the last frame to the window end, so silence shows as one long gap. |
| `unreadable_stamps`, `unreadable_ratio` | Observations whose stamp does not read back, and their share of all observations. |
| `implausible_stamps` | Readable stamps with an age outside 0-60 s: a corrupt read that passed the 8-bit CRC by chance (about 1 in 256), or a clock fault. |
| `repeated_frames` | Observations of a frame id already seen. |
| `skipped_frame_ids` | Ids missing between consecutive new frames: frames lost anywhere between the stamp and the decoder (encoder queue, network, jitter buffers, decoder). |
| `uplink_bytes_per_s` | Router `tx_bytes` increase on the uplink interface (towards `ground_net`) per second: on-wire bytes including WireGuard, UDP, IP and Ethernet headers and keepalives. |
| `uplink_drops` | Packets dropped by the uplink qdiscs during the window. |
| `vehicle_frames_per_s` | Frames the vehicle's encoder produced per second (vehicle metrics). |
| `relay_rtp_bytes_per_s`, `relay_lost_packets` | RTP bytes per second received by the relay, and packets its jitter buffer declared lost (relay metrics). |
| `cut_drain_ms` | The drain time allowed after the cut. |
| `cut_last_frame_ms` | When the last frame arrived after the cut, relative to it: how long queued video kept coming. |
| `video_recovery_ms` | From the restore to the first fresh frame. |
| `recovery_frame_age_ms` | Age of that first fresh frame. |
| `receiver_runs`, `observations_dropped` | Receiver pipeline starts up to the end of the window (before the cut), and observations not stored because the in-memory cap (250 000) was reached. |

Rates are only computed when both readings of a source succeeded; a failed reading leaves the metric out (and an enforced budget on it is `incomplete`).

### MAVLink check

`lab check mavlink --profile <name>` makes `checks` a ground station: it listens on 192.168.232.40:14552, where `ground` sends it telemetry from a socket of its own, and answers that socket with a GCS heartbeat every second and its requests, exactly as QGC does on the Mac. Options: `--duration-s` (default 30), `--gcs-listen`, `--vehicle-metrics` and `--ground-metrics`.

| Assertion | Pass when |
|---|---|
| `profile_applied` | As for the skeleton check. |
| `telemetry_received` | A vehicle heartbeat arrives within 60 s. Without one, every other assertion is `incomplete`. |
| `request_acked` | At least one `REQUEST_MESSAGE(AUTOPILOT_VERSION)` in the window is acknowledged as accepted. |
| `requested_message` | `AUTOPILOT_VERSION` arrives for at least one request. |
| `arm_refused` | Three forced `ARM_DISARM` commands (param 2 = 21196, which overrides ArduPilot's pre-arm checks) sent through the adapter, which holds no lease, are answered `MAV_RESULT_DENIED` by the adapter and leave every vehicle heartbeat disarmed, while heartbeats keep coming and the adapter's `denied` count rises. An armed heartbeat or any other acknowledgement fails; no heartbeat, no counted denial or no `DENIED` arriving is `incomplete`. |
| `sitl_ports_closed` | 10.77.0.2 answers pings over the overlay, and its tcp 5760, 5762 and 5763 accept no connection (`sitl.nft`). `incomplete` without a ping reply. |
| `forwarding_cut` | A vehicle message arrives in the second before the cut or during the drain (otherwise `incomplete`), then after `POST /forwarding {"enabled": false}` and the drain no vehicle message arrives for 3 s. |
| `telemetry_recovery` | After restoring forwarding, a vehicle message arrives within 25 s. |
| `budget`, `budget.<metric>` | The run is graded against `perf/budgets.toml` as for the skeleton check. |

During the window, send `REQUEST_MESSAGE(AUTOPILOT_VERSION)` to system 1 every 2 s, one at a time, waiting at most 5 s for its acknowledgement and message. Forced-arm attempts are 1 s apart, followed by 4 s of observation. Read queued datagrams after blocking metric reads and port probes before timing a cut. Its drain is the largest acknowledgement round trip plus 0.5 s, capped at 10 s.

| Metric | Definition |
|---|---|
| `heartbeats_per_s`, `heartbeat_gap_max_ms` | Vehicle heartbeats in the window per second, and the longest wait for one, counted from the window start and up to its end. |
| `request_ack_rtt_p50_ms`, `request_ack_rtt_max_ms` | From sending `REQUEST_MESSAGE` to the first `COMMAND_ACK` for it from the vehicle: the round trip through adapter, mavlink-router, both router directions, the vehicle gate and SITL. |
| `requested_message_p50_ms`, `requested_message_max_ms` | From the request to the first `AUTOPILOT_VERSION`. ArduPilot sends it right before the acknowledgement. |
| `requests_unanswered` | Requests without both answers within 5 s. |
| `received_bytes_per_s` | Frame bytes from the vehicle that reached `checks` per second. |
| `telemetry_bytes_per_s` | Frame bytes the vehicle sent to the relay per second (vehicle metrics): the telemetry on the uplink, without UDP, IP and WireGuard headers. |
| `sitl_bytes_per_s` | Bytes the vehicle read from SITL per second, including anything that formed no frame of the dialect (not forwarded). |
| `telemetry_rate_limited`, `telemetry_expired` | Frames the vehicle dropped over budget, and replies that waited too long, during the window. |
| `arm_refused` | Increase of the adapter's `denied` count over the arm attempts. |
| `cut_drain_ms`, `cut_last_message_ms` | The drain time allowed after the cut, and when the last vehicle message arrived after it. |
| `telemetry_recovery_ms` | From the restore to the first vehicle message. SITL's default streams send once a second, so this includes up to a second of waiting for the next one. |
| `messages_dropped`, `undecodable` | Messages not stored because the in-memory cap (500 000) was reached, and CRC-valid frames the dialect could not decode. |

### Control freshness check

`lab check control-freshness --profile <name>` exercises the control lease and command freshness ([control protocol](drone.md)), run through `just lab-check` so the adapter can be killed. `checks` plays three parts: a second controller on the overlay (10.77.0.4) that sends envelopes itself, a ground station of the adapter at 192.168.232.40:14552 that commands and drives as QGC does, and an observer of SITL's telemetry straight from mavlink-router at 10.77.0.4:14553. Each run first force-disarms SITL through the adapter, asks for `VFR_HUD` and `GPS_RAW_INT` at 10 Hz and frees the lease, and ends the same way with the default rates.

| Assertion | Pass when |
|---|---|
| `profile_applied`, `telemetry_received` | As for the MAVLink check; the heartbeat must arrive both through the adapter and from mavlink-router. |
| `invalid_refused` | `checks` sends the invalid envelopes and releases listed below, each carrying a forced arm or full throttle; the vehicle refuses each for that rule (an answer of any other kind fails), applies none (its `applied` discrete and continuous counts rise by the valid envelopes only) and stays disarmed. Its liveness count must have risen by exactly the valid liveness envelopes, or nothing applied proves nothing (`incomplete`). |
| `one_lease` | After `checks` releases, the adapter takes the lease (`POST /lease`) and `checks`, asking again, is denied `lease_held`. |
| `discrete_acked` | Through the adapter: manual mode, then arm and disarm three times, each acknowledged `MAV_RESULT_ACCEPTED` by SITL. |
| `resend_not_reapplied` | The last disarm resent with `confirmation` 1 within the ACK window is acknowledged `ACCEPTED` by the adapter, the vehicle counts it `repeated` and applies nothing. |
| `stop_command_applied` | Armed in manual mode with `MANUAL_CONTROL` streaming at 25 Hz through the adapter, a `DO_SET_MODE` Hold is accepted by SITL while setpoints keep being applied around it. |
| `setpoints_applied` | Under a profile without a blackout: armed in manual mode, `MANUAL_CONTROL` at 25 Hz through the adapter drives SITL above 1 m/s while the vehicle's applied setpoints are sampled. |
| `refresh_bounded` | After the adapter is killed, the vehicle's last refresh of the setpoint comes at most the setpoint lifetime plus one 20 ms tick after its last setpoint. |
| `stopped_after_kill` | The adapter is killed mid-drive (the vehicle stops getting setpoints while `checks` still sends them); SITL's throttle is zero within `throttle_release_deadline_ms` of the vehicle's last setpoint and SITL then reaches Hold. Its coast to a standstill is recorded, not graded. No drive or no kill is `incomplete`. |
| `restarted_without_lease` | The restarted adapter answers without a lease and the vehicle has ended the old one. |
| `outage_nothing_applied` | Under a profile whose timeline blacks out both directions (`outage-10s`): the adapter held the lease just before; `checks` sends a forced arm 1 s into the blackout and streams throttle until 6 s after the restore; the vehicle applies nothing in that time and stays disarmed, heard before the blackout and after the restore. |
| `reacquisition_required` | After that blackout the adapter reports the lease lost, denies an arm, takes the lease again, and arm and disarm are accepted. |
| `budget`, `budget.<metric>` | Graded against `perf/budgets.toml` as for the other checks. |

The observer timestamps every telemetry message on its own thread. Speed is `GPS_RAW_INT` ground speed, independent of the EKF; throttle is `VFR_HUD`.

The invalid-envelope phase tests: no lease; an old epoch; duplicate and reordered sequences; reuse of a discrete transaction id; an expired challenge; a body over 1200 bytes; an unsigned envelope; another signing key; another vehicle; a signed neutral frame swapped for full throttle; unsigned and wrong-key lease releases; and a former holder after release. Valid liveness envelopes between cases keep the lease alive. Valid neutral setpoints and requests establish the sequences and transaction ids. Read applied counters before releasing the test lease, so a restarted controller cannot affect the count.

The driving phase streams 20 % throttle at 25 Hz for at least 5 s, samples `last_continuous` every 200 ms, then asks the host to kill the adapter. It keeps sending, waits for applied setpoints to stop for 1 s and watches SITL for up to 12 s.

| Metric | Definition |
|---|---|
| `invalid_applied` | Discrete and continuous applications during invalid-envelope testing, after subtracting the valid probe applications from the vehicle counters. Must be 0. |
| `stop_command_ack_ms` | The acknowledgement time of the Hold sent amid the setpoint stream. |
| `discrete_ack_p50_ms`, `discrete_ack_max_ms` | From sending a command to the adapter to its `COMMAND_ACK` from SITL: adapter, envelope over the overlay (downlink), the vehicle's checks, SITL, and the acknowledgement back through the telemetry (uplink, mavlink-router, adapter). |
| `setpoint_delay_p50_ms`, `setpoint_delay_max_ms` | For sampled continuous envelopes, the vehicle's write to SITL minus the envelope's `sent_unix_ms` (the adapter's clock; the containers share one): the adapter, the overlay downlink and the vehicle's checks. `checks` to the adapter is local and not included. |
| `setpoint_apply_max_ms` | The vehicle's write to SITL minus its arrival: the vehicle's own processing. |
| `top_speed_m_s` | Fastest GPS ground speed while driving. |
| `refresh_last_ms` | The vehicle's last refresh after its last setpoint: how long it kept the setpoint alive. Asserted (`refresh_bounded`) to be at most the setpoint lifetime plus one 20 ms tick. |
| `throttle_released_ms` | First `VFR_HUD` with zero throttle after the last setpoint: SITL stops driving (setpoint lifetime, `RC_OVERRIDE_TIME`, throttle slew, telemetry delay). Graded against `throttle_release_deadline_ms` in `testbed/limits.toml`. |
| `hold_ms` | First heartbeat in Hold after the last setpoint: SITL's GCS failsafe. |
| `speed_zero_ms` | First GPS ground speed at or below 0.2 m/s after the last setpoint: SITL's rover coasting without brakes. Recorded, not graded. |
| `outage_applied` | Discrete and continuous envelopes the vehicle applied from 1 s before the blackout to 6 s after the restore. Must be 0. |
| `reacquired_arm_ack_ms` | The acknowledgement time of the first arm after taking the lease again. |

`timeseries.csv` gets `discrete_ack` per command (`seq` its index), `setpoint_delay` per sampled setpoint and, from the kill on, `ground_speed` (m/s) and `throttle` (%) per message.

### Real-mode check

`just lab-check real-mode lan` runs the vehicle in real mode (`testbed/config/vehicle-real.toml`) against the ArduRover SITL, which real mode accepts only because of the test-only `sitl_in_real_mode` (testbed builds only; the loader refuses it unless `debug_hooks` is set, and refuses `debug_hooks` in real mode without it). `checks` asks the recipe to start the companion on that config (`lab: vehicle-config <file>` on stdout; the recipe runs `compose up -d --no-deps companion` with `VEHICLE_CONFIG` set), and starts it on `vehicle.toml` again at the end, so it expects the default testbed. Assertions ([real-vehicle.md](real-vehicle.md) explains each interlock):

| Assertion | What it shows |
|---|---|
| `real_mode_up`, `interlocks_hold` | The vehicle reports mode `real` and no failing interlock, so control goes to the autopilot (identity, keys, the five failsafe parameters read back, the clock bound). |
| `control_reaches_the_autopilot` | A mode change, an arm and a disarm through the adapter are accepted by SITL and counted as applied. |
| `missing_key_refuses`, `wrong_key_refuses`, `forged_envelope_refuses`, `other_vehicle_refuses`, `unsigned_release_refuses`, `wrong_key_release_refuses` | A second controller sends an envelope with no signature, one signed with another key, a signed setpoint whose frame was swapped for full throttle, an envelope signed with its own key for another vehicle, and lease releases with no signature and with another key's; each is refused (`unsigned`, `bad_signature`) with nothing applied and the lease still held. |
| `clock_bound_reaches_the_ground` | A bound injected into the vehicle (`POST /clock/inject/300`) shows in the ground's metrics as the vehicle's bound, and the sum is 311 ms. |
| `missing_bound_reads_age_unknown` | Without a vehicle bound the ground's label reads `age_unknown`, and `live` again once it is back. |
| `wrong_failsafe_param_refuses`, `sitl_refused` | One interlock at a time is broken through the vehicle's hooks (`/real/require/FS_GCS_ENABLE/0`, `/real/allow-sitl/off`): the vehicle reports it failing, a forced arm through the adapter is denied, counted as sunk, nothing is applied and SITL stays disarmed; the hook is undone. |
| `stale_bound_keeps_control` | With the vehicle's bound taken away (`/clock/inject/missing`) the ground's label reads `age_unknown` and a mode change, an arm and a disarm still reach the autopilot with no interlock failing. |
| `missing_key_at_the_vehicle_refuses` | The vehicle started on `vehicle-real-nokey.toml`, where a listed controller has no public key file, reports `controller_keys` failing and denies the arm. |
| `no_bound_at_start_refuses` | The vehicle started on `vehicle-real-noclock.toml`, whose clock is chrony, with no chronyd running in the testbed, reports `clock_bound` failing and denies the arm. |
| `testbed_restored` | The vehicle is back on `vehicle.toml`. |

### QGC status check

`lab check qgc-status --profile <name>` checks the ground's freshness output from `checks`: it receives the output on 192.168.232.40:5602 as QGC receives it (RTP/UDP, no jitter buffer), reads every frame's stamp and state stamp, asks the adapter's camera what QGC asks it, and stalls the source with the vehicle's test-only hooks. Options: `--warmup-s` (default 5), `--duration-s` (the healthy window, default 30), `--output-listen`, `--gcs-listen`, `--vehicle-metrics` and `--ground-metrics`. It reads `[freshness]` of `testbed/limits.toml`.

| Assertion | Pass when |
|---|---|
| `profile_applied` | As for the skeleton check. |
| `ladder_settled` | As for the skeleton check. |
| `camera_answered` | The camera answers what QGC asks of it, and the vehicle receives nothing from the ground meanwhile (the method below); otherwise fail. A QGC that is still loading the vehicle sends raw requests of its own and would fail this, an idle one does not. No heartbeat or no vehicle reading is `incomplete`. |
| `output_received` | A frame with a state stamp arrives within 30 s after the warm-up. |
| `healthy_live` | At least one frame of the healthy window reads Live; the share is graded by the budget. |
| `pause_indicated`, `freeze_indicated`, `blackout_indicated` | After the vehicle stops its video (pause), repeats one frame (freeze) or the profile's timeline blacks out the link (`outage-10s` only): Stale and then Disconnected show (pause, blackout) or Frozen shows (freeze), and no frame reads Live or LowRate after the first one that read Stale, Frozen or Disconnected until the release (AgeUnknown, which follows lost packets, says nothing about the picture's age). A hook that cannot be used is `incomplete`. |
| `pause_recovered`, `freeze_recovered`, `blackout_recovered` | A Live or LowRate frame within 15 s of the release (the detail gives when the first Live one came). |
| `never_falsely_live` | No frame of the run reads Live while its own stamp, as received, is older than `stale_after_ms` plus 100 ms, or LowRate while older than `low_rate_stale_after_ms` plus 100 ms. |
| `budget`, `budget.<metric>` | Graded against `perf/budgets.toml` as for the other checks. |

Resume the video before measuring, then request `CAMERA_INFORMATION`, `VIDEO_STREAM_INFORMATION` (stream 0) and `VIDEO_STREAM_STATUS` (stream 1) from system 1, component 100. Require `ACCEPTED` and the messages for an RTP/UDP, non-thermal stream. Check `mavlink.gcs.messages_in` through 1 s after the answers to detect leaked camera requests.

Each pause or freeze watches until the expected final state, at most the disconnect threshold plus 5 s, then 1 s more. It resumes and waits up to 15 s for Live or LowRate. A blackout uses the profile's planned transition times. Pausing before the encoder isolates video from MAVLink and the lease, and preserves the compressed reference chain.

| Metric | Definition |
|---|---|
| `output_frames_per_s`, `output_new_frames_per_s` | Frames received from the output in the healthy window per second, and those with a new stamp. Repeats (nothing new for two output periods, or a change of state due) count in the first. |
| `live_ratio`, `state_unreadable` | Share of the window's frames whose state stamp read Live, and frames whose state stamp did not read back. |
| `output_age_p50_ms`, `output_age_p95_ms`, `output_age_max_ms`, `output_age_mean_ms` | Age of the new frames when they arrived here, on `ground_net`: the output's frame age. QGC on the Mac adds gvproxy's hop and its own rendering. |
| `decode_age_mean_ms` | Mean age of the new frames when the adapter decoded them (its `decode_age_ms_total` and `new_frames` over the window). |
| `output_added_ms` | `output_age_mean_ms` minus `decode_age_mean_ms`: the copy and state stamp, the overlay, the encoder, RTP, the local hop and this receiver's decoding; the adapter's own decoding is in both means and drops out. It is a proxy for re-encoding against passthrough: on the direct path a station decodes once, and the adapter's decode stands in for that one. Frames the output's one-frame leaky queue drops never reach `checks`, and they are the ones that waited longest, so it reads slightly low. |
| `pause_stale_ms`, `pause_disconnected_ms`, `freeze_frozen_ms`, `blackout_stale_ms`, `blackout_disconnected_ms` | From the stall (the hook request, or the planned blackout) to the first frame showing that state. |
| `pause_recovery_ms`, `freeze_recovery_ms`, `blackout_recovery_ms` | From the release (the resume request, or the planned restore) to the first Live or LowRate frame. |
| `pause_live_ms`, `freeze_live_ms`, `blackout_live_ms` | From the release to the first Live frame, separately from recovery at LowRate. |
| `pause_gap_max_ms`, `freeze_gap_max_ms`, `blackout_gap_max_ms` | Longest wait for an output frame during the stall: the output keeps coming. |
| `live_age_max_ms` | Oldest frame, by its own stamp as received here, that read Live in the whole run. |
| `observations_dropped` | Frames not stored because the in-memory cap (250 000) was reached. |

### Adaptation check

`lab check adaptation --profile <name>` checks the vehicle's adaptation ([adaptation](video.md)) from `checks`: it plays the relay's stream like the skeleton check, reads the vehicle's ladder and the router's counters, and runs the profile's timeline. `bandwidth-steps` is the graded profile: its uplink steps from 512 to 128, 64, 256 and 512 kbit/s, 15 s apart. Profiles without a timeline (`lte-bad`, `geo-degraded`) are watched for `--duration-s` (default 30). Options: `--warmup-s`, `--duration-s`, `--url` and `--vehicle-metrics` as for the skeleton check.

| Assertion | Pass when |
|---|---|
| `profile_applied` | As for the skeleton check. |
| `ladder_settled` | As for the skeleton check. |
| `second_viewer` | What the vehicle offers the uplink (the router's bytes from `vehicle_net`) while a second receiver plays the stream too is measured against the mean with one receiver before and after (`viewer_change_percent`, which the budget grades). `incomplete` if the rung changed meanwhile or the second receiver got nothing. |
| `video_received` | New frames arrived during the observation. |
| `step_<n>_offered_below_cap` | After the n-th downward step of the uplink rate, the vehicle offers less than the new cap by the end of the phase (the detail gives the settling time); fail if it is still above it. |
| `step_<n>_age_recovered` | After that step, frames are back within 150 ms of the phase's final median age and come again at the phase's final pace (the detail gives when, and how far that level lies above the base); `incomplete` without frames to judge by. |
| `budget`, `budget.<metric>` | Graded against `perf/budgets.toml` as for the other checks: the settling time, windows over the cap after the settling deadline, what is left of a backlog, how long it took to drain, how long a drain paused the video, rung changes per minute and the second viewer's change. |

Wait at most 120 s for a steady rung: adaptation waits 15 s without a rung change, at the top for a timeline profile and anywhere otherwise. Skeleton and QGC status use 6 s at the top; their window is longer than `stale_after_ms` plus `min_change_interval_ms` plus a poll. A failed settle is incomplete.

Compare the offered uplink over 6 s with one viewer, 6 s with two and 6 s with one again. The second window is compared with the mean of the others to cancel encoder drift. Measure base frame age over the last 5 s. Read vehicle metrics every 0.5 s and router counters every second. A timeline observation lasts until 20 s after its final step; the router's `GET /timeline` supplies actual step times.

Downward-step offered rates use 2 s windows sliding by 1 s, beginning after the step. They include one intra-refresh period and two once-per-second telemetry bursts. Router readings are timed when read. Frame-age recovery uses the phase's final median age and gap, because a lower rate also increases serialization.

| Metric | Definition |
|---|---|
| `ladder_settle_ms`, `settled_rung` | How long the initial settle waited, and the rung the ladder settled at (the skeleton and QGC status checks record them too). |
| `viewer_one_bytes_per_s`, `viewer_two_bytes_per_s`, `viewer_again_bytes_per_s` | The offered uplink with one receiver, with two, and with one again. |
| `viewer_change_percent` | How far the offered uplink with two receivers lies from the mean with one before and after, in per cent of that mean. |
| `base_age_ms` | The median frame age of the 5 s before the timeline. |
| `frame_age_p50_ms`, `frame_age_p95_ms`, `frame_age_max_ms`, `new_frames_per_s` | Over the observation, new frames counted as for the skeleton check. |
| `settle_max_ms` | The longest, over the downward steps, from the step to the end of the last 2 s window in which the vehicle offered more than the new cap; 0 when none did. |
| `over_cap_windows` | 2 s windows above the cap that began more than the settling deadline (6 s) after a downward step and before the next step. |
| `age_over_base_max_ms` | The largest, over the downward steps, of the median frame age in the last 5 s before the next step, minus the base: what is left of a backlog. Serialization at the lower rate counts in it too. |
| `age_recovery_max_ms` | The longest, over the downward steps, from the step to the last new frame that came late: more than 150 ms older than its phase's final median, or after a wait for a new frame more than 150 ms longer than the phase's final median wait (a drain pauses the video, so its queue shows as a wait, not as old frames): how long the queue took to drain. |
| `transitions`, `transitions_per_min` | The vehicle's rung changes during the observation, drains included. |
| `drains`, `drain_max_ms` | Drains the vehicle began during the observation (not among the changes it listed at the first reading), and the longest from a drain to the change after it (the resume, or a fall to the floor), on the vehicle's monotonic clock; 0 when none did. |
| `time_normal_s`, `time_constrained_s`, `time_awareness_only_s` | Time spent at each rung group, from the vehicle's readings. |
| `apply_max_ms` | The longest from a requested rung to the first frame stamped in it (the vehicle's `after_request_ms`, on its monotonic clock), as seen in the readings; a drain's pause counts, because no frame is stamped while the video is paused. |

### Ingest check

`just ingest-check <profile>` (default `lan`) grades the camera ingest without a camera. It brings the testbed up with `VEHICLE_CONFIG=vehicle-ingest.toml`, starts the `feeder` container, runs `lab check ingest` and `lab check skeleton` (the video through the link, graded by the skeleton budgets), then `just ingest-isolation`, and stops the feeder. `lab check ingest --profile <name>` applies the profile, waits `--warmup-s`, reads the vehicle's metrics, waits `--duration-s` and reads them again. Options: `--warmup-s`, `--duration-s` and `--vehicle-metrics`; the recipe passes 10 s for both.

| Assertion | Pass when |
|---|---|
| `profile_applied` | As for the other checks. |
| `feeder_decoded` | The ingest decoded at least 80 % of the feeder's 30 frames per second over the window. |
| `foreign_address_refused` | Datagrams from the feeder's second address (192.168.234.6, not in `from`) were dropped and counted (`ingest.refused` rose). |
| `decode_cost` | The mean time the decoder took per frame, over the vehicle lifetime, is at most 20 ms. The mean and the maximum are recorded as `ingest_decode_ms_mean` and `ingest_decode_ms_max`. |

`just ingest-isolation` is a recipe, not a check, because it needs a container on `ingest_net` (the feeder, or a one-off one). It runs in `just gate-full` and `ingest-check` against any testbed. It requires the vehicle sidecar to publish exactly `127.0.0.1:5700/udp`, then reads the counter of each drop rule in `ingest.nft` before and after a probe and requires an increase: a TCP connection from the ingest side to the sidecar (input), the companion reaching the host on the ingest side (output), and packets routed through the sidecar from the ingest side to the router and from the router to the ingest side (forward, one rule each). A timeout would prove nothing, because replies leave through the router either way. Deleting any one drop rule makes it fail.

### Regional check

`lab check regional --profile <name>` measures what regional video ([drone.md](drone.md)) gains and costs. It needs the vehicle on `testbed/config/vehicle-regional.toml` (a one-frame clip of the check's test card as the source, a small snapshot store the pilot's box crops are cut from, `soften_factor = 2` and `debug_hooks` on; `vehicle-regional-detector.toml` also runs the person detector): `VEHICLE_CONFIG=vehicle-regional.toml just lab-up`, then `just lab-check regional geo --warmup-s 8 --duration-s 20` (`--rung <n>` pins the first two windows to ladder rung n, the third is the rung below), with the same `PI=pi4` or `PI=pi-zero-2w` for both to run on a small board's budget ([small-board limits](#small-board-limits)). It is not in `gate-full`. The check writes the clip (one 1280x720 frame, which the vehicle's `still` file source shows for good) to the vehicle's snapshot volume, which the vehicle's sender opens, and plays the ground's output on 192.168.232.40:5602 and scores every third frame against the card scaled to the output's size (SSIM in 8x8 windows), then, as `QGroundControl` does:

1. asks the adapter's camera for its tracking status (`SET_MESSAGE_INTERVAL`), takes the lease, tracks a box (`CAMERA_TRACK_RECTANGLE`) and waits for the status to go active with that box;
2. scores one window with the box, one after `CAMERA_STOP_TRACKING`, a detail window and one with the plain picture one rung down (`POST /video/rung/1` on the vehicle). The detail window tracks the box again and waits (at most 90 s) for the ground to draw the inset of a crop it fetched for the box through the relay's job API (`video.inset_frames` in the ground's metrics), then scores the inset's place in the output against the card's region around the box at full resolution, scaled to the inset's size, and the live picture's same region against the same reference. The inset's area is left out of every window's scoring. Before the first window the check waits for the ladder to settle (`ladder_settled`).

| Assertion | Pass when |
|---|---|
| `tracking_status_requested`, `tracking_active`, `tracking_stopped` | The status arrives idle after the request, goes active with the box once the vehicle confirmed it keeps it sharp, and idle again after the stop. |
| `frames_with`, `frames_without`, `frames_lower` | At least 10 frames were scored in the window. |
| `region_ssim_gain` | The box's SSIM with regional video is above its SSIM without. Graded only on a pilot-only vehicle (`vehicle-regional.toml`): with the detector on its outlines are drawn inside the box and count against the card, so the assertion is `incomplete` and the detector-on run is for CPU and memory. |
| `outside_ssim_vs_lower_rung` | The SSIM outside the box is at least that of the plain picture one rung down. |
| `test_object_contrast_ratio` | The sum of the local standard deviations outside the box, relative to the reference's, is at least that of the rung below (fine detail kept). |
| `ladder_settled` | The vehicle's ladder did not change for 6 s at the top rung before the first window. |
| `detail_shown`, `detail_stopped`, `frames_detail` | The ground drew a crop of the box as an inset within 90 s, the status went idle after the stop, and at least 10 frames were scored. |
| `inset_drawn_throughout` | At least 90 % of the detail window's frames had the inset (`inset_frames` over `pushed` of the ground's metrics). |
| `detail_ssim_gain` | The inset's SSIM against the card's region at full resolution is at least 0.05 above the live picture's in the same region. |
| `frame_age_within_skeleton_budget`, `detail_frame_age_within_skeleton_budget` | Frame age p95 with the box, and in the detail window, is within `skeleton.<profile>`'s limit plus 100 ms for the output's hops. |
| `budget` | `regional.<profile>` in `perf/budgets.toml`. |

The card (`crates/lab/src/regional/card.rs`) is 1280x720 on a plain background with a mild 4x4-cell texture. Inside the tracked box: a landing-pad mark and a person-sized figure. Outside it: a second figure, thin vertical lines 1, 2 and 4 px wide in dark and light, and 8 px checkerboard blocks at a high and a low contrast. It is rendered by the lab, written as a one-frame y4m clip to the vehicle's volume and held by the vehicle's `still` file source; the reference is the same bytes scaled by `videoscale` to the output's 640x360, like the video. SSIM is computed on luma in 8x8 windows every 4 pixels; rows under the vehicle's stamp and over the ground's label are left out, and so are windows within 16 pixels of the box's edge.

| Metric | Definition |
|---|---|
| `region_ssim_with`, `region_ssim_without` | Mean SSIM of the windows inside the tracked box (35-65 % of the width, 30-70 % of the height) with the box tracked and after tracking stopped, at the top rung (1280x720, 15 fps, 1200 kbit/s). |
| `region_ssim_gain` | The first minus the second. |
| `outside_ssim_vs_lower_rung` | Mean SSIM outside the box with the box tracked, minus the same for the plain picture at the rung below (640x360, 15 fps, 500 kbit/s). |
| `test_object_contrast_ratio` | Fine-detail contrast kept outside the box: the sum of the received windows' standard deviations over the reference's, with the box tracked, divided by the same for the rung below. |
| `detail_ssim_inset`, `detail_ssim_live`, `detail_ssim_gain` | With the box tracked again, the ground fetches crops of it (the box widened 10 % each side, from the newest retained snapshot, every 4 s) and draws the newest as an inset. The inset's pixels in the output are scored against the card's region around the box at 1280x720, scaled by area averaging to the inset's size (212x132 on the 640x360 output); the live picture's pixels of the same region are scaled to that size and scored against the same reference. The gain is the difference. The inset's area (label row and frame included) is left out of the scoring of every window. |
| `inset_share` | The share of the detail window's frames that had the inset (the ground's `inset_frames` over `pushed`). |
| `detail_frame_age_p95_ms` | The live frame age p95 in the detail window, held to the same limit as `frame_age_p95_ms`. |
| `vehicle_cpu_percent`, `vehicle_rss_mb`, `soften_ms_p95`, `detector_ms_p95` | The vehicle process's CPU time over the first window as a share of one host core, its resident memory, and the p95 of softening a frame and of one detector run. |

### Quality check

`lab check quality --profile <name>` grades the picture's quality next to its age ([video quality](video.md#measurement-method)). It needs the vehicle on `testbed/config/vehicle-quality.toml` (a looping clip of a scrolling test scene as the source): `VEHICLE_CONFIG=vehicle-quality.toml just lab-up`, then `just lab-check quality geo --warmup-s 8 --duration-s 30`, with `PI=pi4` or `PI=pi-zero-2w` for both to run on a small board's budget. The check writes the clip (62 MB) to the vehicle's snapshot volume, waits for the ladder to settle wherever the profile lets it, then scores every frame the relay's stream delivers against the clip frame it shows. It is not in `gate-full`.

The scene is the regional check's card (`crates/lab/src/regional/card.rs`) over a terrain of smooth noise with fine grain, 1280x720. It scrolls down 16 rows a frame and wraps after 45 frames, a loop of 3 s at 15 fps. The terrain has the same period as the image height, so the loop has no seam. The lab writes the loop as a y4m clip of 62 MB to the vehicle's snapshot volume (96 MiB). The vehicle plays it with the `file` source of `testbed/config/vehicle-quality.toml`, which loops it in real time. A clip of that length already on the volume is not written again.

The check applies the profile and lets the ladder settle at whatever rung the profile allows (no change for 15 s), then waits `--warmup-s` and measures for `--duration-s`. The receiver is the one the skeleton check uses: the relay's RTSP stream, decoded at the size of the settled rung (`testbed/ladder.toml`).

For every decoded frame the check finds the clip frame it shows. The rung drops frames before they are stamped, so the stamp's frame id does not say which clip frame it is. The frame and each clip frame are reduced to a 64x36 grid of mean luma, below the stamp's rows, and the clip frame with the smallest mean absolute difference wins. The clip frames are scaled to the received size with `videoscale`, as the sender scales them.

The matched clip frame is the reference for two SSIM scores, each the mean over the 8x8 windows of `crates/lab/src/regional/ssim.rs`, on luma, leaving out the rows the stamp is drawn in (`state_rows_px`). The relay's stream has no label. `ssim` is the received frame scaled up to 1280x720 by bilinear interpolation, against the clip frame at 1280x720. `ssim_native` is the received frame against the clip frame at the received size. At the top rung the two are the same score. Below it they differ in how much of the picture an 8x8 window covers, and neither is always the higher one (`geo-degraded`).

Frames that never arrived are counted from the stamps' frame ids: the ids from the first to the last readable one, less the frames received between them. A frame whose stamp does not read still counts as received.

| Metric | Definition |
|---|---|
| `ssim_p50`, `ssim_p5` | Median and 5th percentile of `ssim` over the window's frames. |
| `ssim_below_floor_share` | The share of frames with `ssim` under 0.8. |
| `ssim_native_p50`, `ssim_native_p5` | The same for `ssim_native`. |
| `frames_scored`, `frames_missing`, `frames_missing_ratio` | Frames scored, frames that never arrived, and the missing over missing plus received. |
| `frame_age_p50_ms`, `frame_age_p95_ms` | Frame age at the receiver, from the stamp. |
| `match_backward_steps` | Steps back through the loop between two neighbouring frames that both score at least 0.8. |
| `score_ms_p95` | The time to match and score one frame, p95. |

`matching_forward` fails if more than 2 % of the frames take such a step. A frame shown later cannot be earlier in the loop, so a step back between two good frames means the match mistook one frame for another. A frame the link damaged scores under 0.8 and matches whichever clip frame it resembles most, often the same one, so its steps are left out. Scoring a damaged frame against the clip frame closest to it can only raise its SSIM, so the numbers err towards a better picture.

### Recording check

`lab check recording --profile lan` checks the vehicle's retained snapshots ([retained detail](drone.md)). `checks` sees the store's directory. The check is run through `just lab-check recording lan`: it needs the host to restart `companion`, which it asks for by printing `lab: restart companion`; run any other way it is `incomplete` at that step. Options: `--warmup-s`, `--url`, `--vehicle-metrics` and `--snapshot-dir` (default `/snapshots`). The store's limits come from `testbed/config/vehicle.toml` (`[store]`).

| Assertion | Pass when |
|---|---|
| `profile_applied`, `ladder_settled` | As for the other checks. |
| `recording_started` | The store wrote three snapshots within 30 s. |
| `quota_held` | The quota was reached (the store evicted), and for 20 s after neither the directory nor the store's own count passed the quota and the oldest snapshot only got newer. `incomplete` if the quota was never reached. |
| `snapshot_cadence` | The median interval between retained snapshots is within 10 % of the configured second. |
| `restart_recovery` | After `companion` is restarted with a half-written `.tmp` and two names that are no snapshots planted: the `.tmp` is deleted, the other names are left alone and counted as ignored, the snapshots on disk are all indexed (within two) and the store writes again. |
| `gap_reported` | Gaps left by slow writes appear in the directory and agree with the store's `gaps` count, read before and after the directory listing. |
| `slow_storage_drops` | With every write made to take 3 s (a vehicle hook, `debug_hooks`), snapshots are dropped and counted (`snapshots.dropped`). |
| `slow_storage_recovers` | After the hook is lifted the store writes again. |
| `live_unaffected` | Live frames kept arriving during slow storage; the frame age and pace are graded by the budget against the healthy stretch before. |
| `budget`, `budget.<metric>` | Graded against `perf/budgets.toml`. |

The healthy window is the 20 s quota observation. The slow-write window also lasts 20 s, with each write delayed by 3 s. Frames before and after the restart are counted separately because the process starts frame ids at zero.

| Metric | Definition |
|---|---|
| `store_over_quota_bytes` | Largest excess of the directory or store count over `max_bytes`. |
| `snapshot_interval_median_ms` | Median capture interval between retained snapshots. |
| `restart_resume_ms` | From the restart request to the first new snapshot. |
| `slow_dropped` | Snapshots dropped while storage was slow. |
| `slow_encoded_per_s`, `slow_new_frames_per_s` | Vehicle encoded frames and receiver new frames per second in the slow-write window. |
| `healthy_frame_age_p95_ms`, `slow_frame_age_p95_ms`, `slow_age_p95_delta_ms` | Healthy and slow-write frame ages and their difference. |

### Retrieval check

`lab check retrieval --profile <geo|bandwidth-steps|outage-10s>` checks retrieval of a snapshot or a crop of a frame captured seconds ago ([retained detail](drone.md)) from `checks`, which plays the ground (the job API on `ground_net`) and the stream's viewer. Options: `--warmup-s`, `--duration-s` (a pass under a profile without a timeline), `--url`, `--vehicle-metrics`, `--relay-metrics`, `--snapshot-dir`, `--ground-api` and `--vehicle-api`. The allowance comes from `[retrieval]` of `testbed/limits.toml`.

| Assertion | Pass when |
|---|---|
| `profile_applied`, `ladder_settled` | As for the other checks. |
| `crop_matches_retained` | A crop (the middle half) of the frame 8 s ago comes back as the newest retained snapshot at or before that time (the file `checks` read from the volume): same capture time and frame id, the source size of the file, the rectangle `crop::pixel_rect` gives for the fractions asked, and a JPEG of exactly that size. |
| `snapshot_matches_retained` | A snapshot of the same time comes back byte for byte as the retained file. |
| `hostile_requests_refused` | Each of the hostile requests in `crates/lab/src/retrieval.rs` is refused with its status (malformed or unknown-field JSON, an oversize body, crops outside 0 to 1 or without area, bad `max_bytes` and expiry, unknown ids, path traversal and dots in an id, a long URL, a query string, the other listener's routes, a bad chunk index, a missing hash, a chunk over 16 KiB), a fifth job and a third 1 MiB job are refused with 429, and the relay counted the refusals. |
| `cancel_stops_transfer` | A snapshot cancelled after two chunks stops: the vehicle notices within 30 s (`cancel_stop_ms`), the relay receives no chunk for 4 s after, and the job is unknown. `incomplete` if the transfer finished before the cancel. |
| `live_unaffected` | Two passes under the profile (re-applied, the ladder settled at the top before each), one without retrieval and one with crop jobs back to back: the frame age's p50 and p95 with and without retrieval and their difference are metrics the budget grades. |
| `retrieval_within_allowance` | The vehicle's retrieval wire bytes in the pass did not pass the allowance times the pass plus the bucket's first second. |
| `retrieval_yields` | Under a profile with a timeline only: retrieval held back at some point (the ladder left its normal rungs, a drain, stale feedback) and the vehicle's metrics, read every 100 ms, never show a chunk sent between two readings that both show it held. `incomplete` if it was never held. |
| `budget`, `budget.<metric>` | Graded against `perf/budgets.toml`. |

Wait for 12 retained snapshots before requesting the objects. A comparison pass lasts `--duration-s` without a timeline, or until 20 s after the last timeline step. Each pass reapplies the profile and waits for the top rung to stay steady for 15 s.

| Metric | Definition |
|---|---|
| `crop_completion_s`, `snapshot_completion_s` | From each request until its object is fetched. |
| `crop_bytes`, `snapshot_bytes` | Retrieved object sizes. |
| `hostile_requests`, `hostile_accepted` | Hostile requests sent and those accepted. |
| `cancel_stop_ms` | From cancellation until the vehicle's `jobs_cancelled` rises. |
| `age_p50_without_ms`, `age_p95_without_ms`, `age_p50_with_ms`, `age_p95_with_ms` | Live frame-age percentiles in the paired passes. |
| `age_p50_delta_ms`, `age_p95_delta_ms` | With-retrieval minus without-retrieval ages. |
| `retrieval_wire_bytes_per_s`, `retrieval_over_allowance_bytes` | Retrieval wire rate and bytes beyond the allowance times the pass plus one initial bucket second. |
| `retrieval_jobs_done`, `retrieval_chunks` | Jobs and chunks completed in the retrieval pass. |
| `retrieval_holds`, `retrieval_held_s` | Times retrieval began holding back and total time held. |
| `retrieval_sent_while_held` | Chunks sent between two consecutive readings that both show held, sampled every 100 ms during a timeline. |
| `job_completion_p50_s` | Median job completion time. |
| `offered_uplink_delta_bytes_per_s` | Offered uplink with retrieval minus without. Recorded, not graded, because the video's variation is included. |

The allowance is `allowance_bytes_per_s` in `[retrieval]` of `testbed/limits.toml`, in wire bytes per second.

### Simulated-flight check

`just sim-check <profile>` runs `lab check sim-flight` and the [QGC status check](#qgc-status-check). Start and operate the optional simulation as described in [drone.md](drone.md).

`sim-flight` takes the lease, retries a plain arm while pre-arm checks refuse it, takes off and flies a Loiter leg at about 4 m/s within a 3 m height band. It stops setpoints and requires Brake and hold. It requests Loiter without setpoints and requires refusal with the copter remaining in Brake (`no_drop_after_brake`). It then asks the host to kill the ground adapter and requires Land, touchdown and disarm.

`brake_verdict` and `land_verdict` use the copter's reports. The descent limit comes from ArduCopter 4.7.2's default `WP_SPD_DN`, with `LAND_SPD_HIGH_MS = 0`; see `land_verdict` in `crates/lab/src/control/sim_flight.rs`. A Brake that started before the check stopped setpoints is incomplete.

| Metric | Definition |
|---|---|
| `brake_ms`, `brake_sag_m` | Time from the last setpoint to Brake, and altitude lost in the Brake phase. |
| `land_ms`, `land_sag_m`, `on_ground_ms` | Time from the last setpoint to Land, altitude lost before Land and time from that setpoint to touchdown. |
| `video_live_ratio` | Live share during the flight leg. `video_live` uses the minimum from the same budget. |

These metrics are graded by `[sim-flight.<profile>]` in `perf/budgets.toml`. Video age, frame rate and simulation real-time factor are recorded but ungraded. Readings include refused envelopes, lease losses, uplink queue delay, adaptation steps, mode changes and autopilot `STATUSTEXT` per phase. A slow simulator can change wall-clock failsafe timings; it is not real flight evidence.

### Mission check

`just sim-mission <profile> on|off` runs `lab check sim-mission` with `allow_missions` enabled or disabled. It has no performance budget. The check reads the switch from the vehicle and grades accordingly.

| Phase | Switch on | Switch off |
|---|---|---|
| Upload | Accept six items: home placeholder, takeoff, three waypoints 20 m apart to the north at 10 m, landing. Wait 1500 ms for each next request, repeating `MISSION_COUNT` only before the first. Handle `MISSION_REQUEST` and `MISSION_REQUEST_INT`. | Deny `MISSION_COUNT` with `MISSION_ACK`. |
| Start | Guided, plain arm, accept `MAV_CMD_MISSION_START` and enter Auto. | Deny `MISSION_START` and Auto, counted as `mission_not_allowed` and `mode_not_allowed`. |
| Adapter loss | Kill the adapter at the first waypoint. Require later waypoints reached in Auto, then touchdown and disarm. | No mission flight. |

Upload duration and the `MISSION_COUNT` count are recorded but ungraded. Killing the adapter stops setpoints and the GCS heartbeat, so both radio and GCS failsafes apply. The check covers a cut inside the fence. It does not fly a fence breach.

### Follow check

With the [simulation's detector and follow enabled](drone.md), run `PI=pi4 SIM=1 just lab-check follow lan`. The check takes the copter to a 3 m hover in Loiter, picks the right-most person in `/detector/boxes` and sends `CAMERA_TRACK_RECTANGLE`. It yaws in 0.5 s turns at stick value 200 of 1000, holding for 2 s between turns.

At each hold, `/follow/target` must lie within 0.08 of the picture width and 0.10 of its height of that person in the same metrics read. The person must cross at least 0.2 of the width. Turns continue until follow ends lost with a reason, the box is gone and tracking status is idle; then the check lands. It does not itself prove the person left the picture.

### Sightings check

With the [simulation's detector enabled](drone.md), run `SIM=1 just lab-check sightings outage-30s`. The profile is `lan` with a 30 s blackout beginning 5 s after the timeline starts. The copter stays on the runway with two people and a car in view.

No sighting may arrive during the cut. After restore, `STATUSTEXT` sightings must have matching positions and first-seen times within the cut, allowing 3 s. The job API must list the same number, all marked told and each with a snapshot captured within 1.5 s. The first snapshot must be fetchable. `vehicle-sim-regional.toml` keeps 4 MiB of snapshots to retain the early outage frames.

### Landing check

`just sim-landing <on|off|forward> <profile>` runs `lab check sim-landing`. It takes off, steers with `MANUAL_CONTROL` to a point 2.3 m from above the tag, holds, watches the vehicle for 4 s and switches to Land. See [drone.md](drone.md) for the camera and precision-landing configuration.

With the target enabled and camera down, `aim_matches_geometry` requires the last target angles to agree within 3 degrees with the tag's angles from the copter's position, height and heading. Require counted `LANDING_TARGET` messages and landing within 1 m of the tag centre. Otherwise require no target sent, a warning to the pilot when enabled, and landing more than 1 m away after the autopilot's pause. `descent_begins_s` records that pause.

## Result files

Every run writes `results/<unix-ms>-<scenario>-<profile>/`:

- `summary.json`: `run_id`, `scenario`, `profile`, `verdict`, `git_revision`, `assertions` (name, verdict, detail), `metrics`, `sample_counts`, `thresholds` (the budget used, or `null`), `effective_profile` (what the router agent applied), and for the skeleton check `settings` (receiver URL, protocols, latency, decoder, warm-up, duration, cut timings, metrics URLs) and `readings` (router counters, both apps' metrics and the receiver's health at the window start and end, and the receiver's health when the run finished). The MAVLink check records its settings and the vehicle and ground metrics at the window start and end and around the arm attempts; the control freshness check its settings (addresses and the `[control]` limits) and the vehicle metrics before and after the second controller's phase; the QGC status check its settings (receiver, the `[freshness]` limits, timings), the adapter's metrics at the window start and end and at the finish, and its camera counts before and after the requests.
- `timeseries.csv`: `unix_s,series,seq,value`. Boot: one row per ping reply, `series` the probe, `seq` the ICMP sequence, `value` the RTT in ms. Skeleton: one row per decoded frame over the whole run (warm-up, window, cut and recovery), `series` `frame_age` (new frame, `seq` frame id, `value` age in ms), `repeated`, `implausible` (raw age) or `unreadable` (`value` NaN). MAVLink: `heartbeat_gap` (each vehicle heartbeat, the gap from the previous one in ms) and, per request (`seq` its index, `unix_s` when it was sent), `ack_rtt` and `requested_message` in ms. Control freshness: as listed in its section. Adaptation: `frame_age` per new frame (`seq` its id), `offered` and `cap` per 1 s router window in bytes per second (`seq` the window), and `rung` and `queue_delay` (the vehicle's, in ms) per reading of the vehicle (`seq` the reading). QGC status: one row per frame received from the output over the whole run, `series` the state its state stamp read (`live`, `stale`, `frozen`, `disconnected`, `age_unknown`, `low_rate`, or `no_state`), `seq` its frame id (0 without a stamp), `value` its age in ms (NaN without a plausible stamp).

Image ids are not recorded.

## Verdicts and budgets

Each assertion is `pass`, `fail` or `incomplete`. A run fails if any assertion fails; otherwise it is incomplete if any assertion is incomplete or there are none; only then does it pass. Exit codes: 0 pass, 1 fail, 2 incomplete. A run that cannot read its profile or budgets, or cannot write its results, is incomplete. Missing evidence is never a pass.

Budgets live in `perf/budgets.toml`, one table per scenario and profile. Each table has `status` and `rationale`; enforced tables also have `min` and/or `max` metric limits.

- `calibration`: no limits allowed. Metrics are recorded and the `budget` assertion is incomplete.
- `enforced`: at least one limit and a rationale are required. Each limit produces `budget.<metric>`; an unmeasured metric is incomplete.
- No budget for a budgeted scenario and profile: incomplete.

Limits come from measurements with an explicit margin. The file holds their values, derivation and enforcement status. They are never inferred from the impairment a run applies. Checks without a performance budget still grade their behavioral assertions.

## Directions

- Uplink: router egress toward `ground_net` (the interface holding `192.168.232.254`). Vehicle-to-relay traffic.
- Downlink: router egress toward `vehicle_net` (the interface holding `192.168.231.254`). Relay-to-vehicle traffic.

Each direction is impaired independently. RTT is the sum of the two directions' delays.

## Profile format

TOML, one file per profile in `testbed/profiles/`. Unknown fields are rejected. A profile is validated completely before the router is touched.

```toml
[profile]
name = "geo"                 # a-z, 0-9, '-' ; matches the file name
description = "..."
provenance = "synthetic"     # "synthetic" | "measured" ("measured" requires a non-empty source)
source = "..."               # where the numbers come from
seed = 42                    # optional, makes netem loss and jitter repeatable

[uplink]                     # and [downlink], same keys
delay_ms = 300               # 0..=5000
jitter_ms = 10               # default 0, must not exceed delay_ms
jitter_distribution = "normal"   # default; "normal" | "pareto" | "uniform"
rate_kbit = 512              # 8..=1_000_000
burst_bytes = 6000           # default 16384, 1514..=10_000_000
queue_packets = 1000         # default 1000, 1..=100_000
ordering = "preserve"        # default; "preserve" | "allow-reorder"

[uplink.loss]                # optional, per direction
model = "random"
percent = 1.5                # 0..=100

[downlink.loss]
model = "gilbert-elliott"
p = 5                        # percent, good -> bad
r = 20                       # percent, bad -> good
one_minus_h = 80             # percent loss in the bad state
one_minus_k = 0              # percent loss in the good state

[[timeline]]                 # optional, in time order
at_ms = 10000                # offset from timeline start
action = "blackout"          # "set-rate" | "blackout" | "restore"
direction = "both"           # "uplink" | "downlink" | "both"
rate_kbit = 128              # only with set-rate, 8..=1_000_000
```

Loss lives inside each direction (`[uplink.loss]`, `[downlink.loss]`) so loss can be asymmetric.

All percentages must be finite and within 0..=100. `delay_ms`, `rate_kbit` are required in each direction; everything else has a default or is optional.

## Profiles

| Profile | Uplink | Downlink | Notes |
|---|---|---|---|
| `lan` | 1 Gbit, no delay | same | No impairment (rate cap only). |
| `lte-good` | 40 ms, 5 ms jitter, 5 Mbit | 40 ms, 5 ms jitter, 20 Mbit | |
| `lte-bad` | 60 ms, 30 ms pareto jitter, 800 kbit, G-E loss | 60 ms, 30 ms jitter, 3 Mbit, G-E loss | About 5 % mean loss (p / (p + r) = 1.34 / 21.34 = 6.3 % of time in the bad state, times 80 % loss there), bad bursts of 1 / r = 5 packets. |
| `geo` | 300 ms, 10 ms jitter, 512 kbit | 300 ms, 10 ms jitter, 2048 kbit | 600 ms RTT target. |
| `geo-degraded` | 300 ms, 25 ms jitter, 128 kbit, G-E loss | 300 ms, 10 ms jitter, 1024 kbit, 1 % random loss | Asymmetric. |
| `outage-30s` | `lan` | `lan` | Timeline: blackout both at 5 s, restore both at 35 s. |
| `outage-10s` | `geo` | `geo` | Timeline: blackout both at 10 s, restore both at 20 s. |
| `bandwidth-steps` | `geo` | `geo` | Timeline: uplink 512 (initial), 128 at 15 s, 64 at 30 s, 256 at 45 s, 512 at 60 s. |

## tc translation

Per interface, `lab` builds this chain (handles are fixed):

```text
htb root 1: (default 10) -> class 1:10 (rate = ceil = rate_kbit) -> netem 10: -> pfifo 11: (ordering = preserve)
```

For `geo` uplink on `eth1`:

```sh
tc qdisc del dev eth1 root                       # failure tolerated (nothing to delete)
tc qdisc add dev eth1 root handle 1: htb default 10
tc class add dev eth1 parent 1: classid 1:10 htb rate 512kbit ceil 512kbit burst 6000 cburst 6000 quantum 1514
tc qdisc add dev eth1 parent 1:10 handle 10: netem delay 300ms 10ms distribution normal limit 1000
tc qdisc add dev eth1 parent 10: handle 11: pfifo limit 1000
```

Live changes never delete qdiscs: `set-rate` is `tc class change` on `1:10`, `blackout` is `tc qdisc change` on netem with `loss random 100%`, `restore` is the same change with the profile's own loss. A blackout drops new packets at netem; packets already queued still drain, so a deep `queue_packets` under heavy overload delays the cut.

`ordering = "preserve"` adds a FIFO child but does not guarantee that jitter cannot reorder packets.

## Router agent

```sh
lab router --listen 192.168.233.254:9000 --uplink-if auto --downlink-if auto
```

`auto` finds each interface by its router address using `ip -j addr`; explicit names are validated and `lo` is refused. The agent only ever touches those two interfaces. It needs `NET_ADMIN`, and `tc` and `nft` on the path. Requests are handled one at a time and bodies are capped at 64 KiB. Every response is JSON; errors are `{"error": "..."}`.

| Request | Effect |
|---|---|
| `POST /profile` (TOML body) | Validate, rebuild both qdisc chains, return the commands applied and the effective `tc -j qdisc/class show` state. Stops a running timeline. 400 on an invalid profile. |
| `POST /timeline/start` | Re-apply the current profile's baseline and run its timeline on a thread. 409 without a profile or timeline. |
| `GET /timeline` | `running` and the recorded transitions (`planned_ms`, `actual_ms`, interface, command, error). |
| `GET /counters` | Per direction: `tc -s` qdisc counters (bytes, packets, drops, overlimits, backlog) and `/sys/class/net` statistics. |
| `POST /forwarding` `{"enabled": bool}` | `false` installs an nft table `inet sat_teleop_cut` that drops forwarded traffic; `true` removes it. |
| `POST /reset` | Stop the timeline, delete the root qdiscs, remove the nft table. |

`inet sat_teleop_cut` hooks forward at priority -100, ahead of the router's own `ip sat_teleop_router` table, and only the agent touches it.

The CLI wraps this for interactive use and validates the file locally first:

```sh
lab impair --agent http://192.168.233.254:9000 --profile testbed/profiles/geo.toml
lab impair --agent http://192.168.233.254:9000 --profile testbed/profiles/outage-10s.toml --start-timeline
```

For HTTP diagnostics inside a container use `curl` (in the runtime image), for example `curl -s http://192.168.233.254:9000/counters` from `checks`.

## Images

`testbed/Containerfile` has these stages, built with the repository root as context (`.dockerignore` keeps `target/`, `.git/`, caches, results and secrets out):

- `build`: the pinned Rust image ([toolchain](#toolchain)) with GStreamer development packages (`localhost/sat-teleop-build:dev`, used by `just check`). `just image runtime` tags it too, so its layers, apt install included, are not left to a dangling release image that `podman image prune` removes.
- `bins-native`: from `build`, runs `cargo build --release --locked` with cache mounts for the registry and `target/` (one `target/` cache per build architecture, so a second architecture never empties the first's). `bins-cross-amd64`: from `build` too, cross-compiles the same binaries for `x86_64-unknown-linux-gnu` with the Debian cross toolchain and amd64 GStreamer headers, because `rustc` segfaults under qemu emulation on Apple Silicon. `build` always runs on the build host's own architecture.
- `localhost/sat-teleop-sitl:dev` comes from `testbed/Containerfile.sitl` instead.
- `runtime-base`, `runtime` and `runtime-amd64`: Debian trixie slim (pinned by index digest) with GStreamer runtime plugins (`gstreamer1.0-x` for Pango's `textoverlay`), `iproute2`, `nftables`, `wireguard-tools`, `iputils-ping`, `tcpdump`, `procps`, `curl`, `gstreamer1.0-rtsp` (for `rtspclientsink`), mavlink-router v4 (`mavlink-routerd-glibc-aarch64` or `-x86_64` by `TARGETARCH`, pinned by SHA-256 from the v4 GitHub release), and the release binaries `lab`, `sat-teleop-vehicle`, `sat-teleop-relay` and `sat-teleop-ground` in `/usr/local/bin`: `runtime` from `bins-native` (`localhost/sat-teleop-runtime:dev`), `runtime-amd64` from `bins-cross-amd64` (`just image-for amd64`, `localhost/sat-teleop-runtime:amd64`, [hardware.md](hardware.md)).

`testbed/Containerfile.sitl` builds `localhost/sat-teleop-sitl:dev` (`just sitl-image`, `--platform linux/amd64`) from Debian trixie slim (pinned by index digest) and two downloads pinned by SHA-256 with `ADD --checksum`: the official ArduRover 4.7.2 SITL binary (statically linked x86_64, ArduPilot git `fb773567896014da007bca58358704d4a0d8516f`, 6845664 bytes, `f453fd8217b9d798c873e5c424e139a2a3597b52e31658f76b2c177b523391af`) and `rover.parm` from the same commit (606 bytes, `6479879d3e625854a6c32cb4d3fe231bab81d935e07904eac8a411fd6f971bc4`).

Both apt installs use cache mounts (`/var/cache/apt`, `/var/lib/apt`) with Debian's `docker-clean` hook removed, so a layer rebuild reuses downloaded packages.

## macOS and Podman notes

- The testbed runs in the Podman machine VM. Follow the [build memory limits](#builds-and-gates) and [shared-machine rules](../AGENTS.md#9-testbed-commands). Do not restart or reconfigure the machine.
- The VM needs kernel WireGuard, `netem`, `htb` and nftables NAT. Containers receive `NET_ADMIN` only where required.
- Repository bind mounts pass through the VM's file share. Files written by containers appear as your user on the Mac.
- SITL is x86_64 and uses the VM's qemu binfmt on Apple Silicon. The runtime and SITL Debian bases use a multi-platform index digest, so a build for another platform does not retag a base shared by the VM. Each build checks the registry and reuses local layers.
- Without `ping_group_range`, ping fails in Podman containers with `socket: Operation not permitted`.
- Keep probes inside containers and use `curl` or `lab` for HTTP. Follow [endpoint security](../AGENTS.md#endpoint-security).
- For [netavark's leaked forwarding settings](https://github.com/containers/netavark/issues/1497), `lab-down` removes the drop-ins belonging to this project's internal networks once their bridges are gone. `lab-up` checks forwarding on `ground_net` after startup and enables it for that bridge if needed. A stale `/run/sysctl.d/10-netavark-<bridge>.conf` can otherwise leave published ports accepting connections without carrying data.

## Docker equivalence

The recipes take the engine from `CONTAINER_ENGINE` (default `podman`; `just engine=docker ...` works too): `CONTAINER_ENGINE=docker just lab-up`, `CONTAINER_ENGINE=docker just lab-check boot lan` and `CONTAINER_ENGINE=docker just lab-down` run `docker compose -p sat-teleop -f testbed/compose.yml -f testbed/compose.docker.yml ...`.

The second file maps `host.containers.internal` (Podman's name for the host, which the ground adapter sends QGroundControl's telemetry and video to) with `host-gateway`. `lab-keys` swaps `--userns=keep-id` for `--user $(id -u):$(id -g)`. Docker already allows unprivileged ping, so the sysctl is redundant there but harmless.

The Docker path has not been run end to end here; it needs a host or VM kernel with WireGuard, `netem` and nftables NAT.
