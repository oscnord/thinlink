# Testbed

The testbed is compose project `sat-teleop` (`testbed/compose.yml`). A router container replaces the cellular or GEO-like link: it forwards and NATs the vehicle network onto the ground network and impairs each direction with `tc` through the router agent (`lab router`). Every node joins a WireGuard overlay whose server is the relay, so vehicle traffic reaches the relay inside WireGuard through the router, as it would over a carrier network.

Applications share a sidecar's network namespace (`network_mode: service:<sidecar>`). The services table below says which namespace each runs in.

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

`ctl_net` is one shared network, so without a filter a ground-side container could reach the vehicle's `ctl_net` address directly, unimpaired, and silently invalidate every latency measurement. Every sidecar and `checks` therefore loads `testbed/scripts/ctl.nft` on its `ctl_net` interface (found by address, passed to `nft -D ctl_if=...`): only TCP to the router agent (9000) and to app metrics (9100) and their replies are allowed; everything else, UDP and ICMP included, is dropped, and nothing is forwarded to or from `ctl_net`. `checks` may additionally send anything from its own `ctl_net` address so the boot check can prove the filter holds on the vehicle. The router itself is not filtered on `ctl_net`; its forward policy already drops that traffic. Local DNS lookups are unaffected: queries to the `ctl_net` gateway fail at once and the resolver moves on to the next server.

SITL listens on every address of the vehicle namespace: MAVLink on tcp 5760 (serial 0), 5762 and 5763 (serial 1 and 2), and RC input on udp 5501. Without a filter, any overlay peer (ground or `checks`) could reach the autopilot directly, past everything the companion enforces. The `vehicle-net` sidecar therefore also loads `testbed/scripts/sitl.nft` (named by `EXTRA_NFT`), which drops those ports on every interface but loopback.

A camera outside the containers (the Mac's webcam, `docs/demo.md`) reaches the vehicle through a fourth network, `ingest_net`, and its video then takes the impaired link like any frame. It must not become a way around the link, so it carries one thing: the sidecar loads `testbed/scripts/ingest.nft` (found by address like `ctl_if`) on its interface, which accepts only UDP to port 5700 and drops everything else arriving on it, everything leaving through it and everything forwarded to or from it. The sidecar also drops the default route the network adds, so the vehicle's traffic only leaves through the router. Port 5700 is published on host loopback only. The vehicle (`testbed/config/vehicle-ingest.toml`) then accepts datagrams from the addresses listed in its source's `from` only: a Podman machine's host gateway (192.168.127.1, where the Mac's traffic comes from through the published port), the network's gateway (192.168.234.1, a Linux host) and the feeder. `just ingest-isolation` proves it (below).

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
| 5006/udp | 10.77.0.1 | The relay's feedback to the vehicle's probes ("Live video path"); overlay address only |
| 5700/udp | 192.168.234.10, published on host 127.0.0.1:5700 | RTP/H.264 from an external camera into the vehicle's ingest source (`vehicle-ingest.toml` only), from the addresses in `from` only |
| 5760/tcp | vehicle namespace, loopback only (`sitl.nft`) | SITL serial 0, MAVLink for the companion |
| 14550/udp | 10.77.0.1 | mavlink-router's server endpoint for the vehicle, overlay address only |
| 14551/udp | 10.77.0.3 | sat-teleop-ground's overlay socket; mavlink-router's client endpoint sends here |
| 14552/udp | 192.168.232.40 | `lab check mavlink` and `control-freshness` in `checks`, a ground station of sat-teleop-ground |
| 14553/udp | 10.77.0.4 | `lab check control-freshness` in `checks`: SITL's telemetry straight from mavlink-router |
| 7700/udp | 10.77.0.2 | The vehicle's control endpoint (control envelopes, lease, challenges) |
| 7701/udp | 10.77.0.3 | sat-teleop-ground's controller, the lease holder's socket |
| 14550/udp | the Mac, as `host.containers.internal` | QGC's default UDP link; sat-teleop-ground sends telemetry there and hears QGC's replies on the same socket, so nothing is published (`docs/qgc.md`) |
| 5600/udp | the Mac, as `host.containers.internal` | QGC's default UDP video port; sat-teleop-ground's freshness output (RTP/H.264) goes there through gvproxy (`docs/qgc.md`) |
| 5602/udp | 192.168.232.40 | `lab check qgc-status` in `checks`: the same output |
| 8080/tcp | 10.77.0.1 | The relay's job API for the vehicle (`GET /jobs/next`, chunk uploads, `complete`, `fail`): overlay address only, bearer token required (`docs/architecture.md`, "Retained detail") |
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

The VM kernel provides WireGuard, so the sidecars use kernel WireGuard through `wg-quick`; nothing runs in userspace. Each client peer (vehicle, ground, checks) points at `192.168.232.20:51820` with `AllowedIPs = 10.77.0.0/24` and `PersistentKeepalive = 15`. The relay lists each client with its own `/32` and forwards between them, so ground and checks reach the vehicle through the relay. The vehicle's tunnel crosses the router, which masquerades it: the relay sees the vehicle at `192.168.232.254`.

`just lab-keys` runs `testbed/scripts/keys.sh` in a runtime container. It writes `testbed/secrets/<node>.key`, `.pub` and a wg-quick `<node>.conf` for `relay`, `vehicle`, `ground` and `checks` (mode 600), one bearer token per service, mode 600 (`vehicle.token`: the vehicle's endpoints, `ground.token`: the adapter's, `job.token`: the relay's metrics and job API, which the vehicle and the adapter also present; each application mounts only what it serves or presents and `checks` all three), and the control signing keys `ctl-ground.key`/`.pub` and `ctl-checks.key`/`.pub` (`lab keygen`): the ground adapter and the lab runner sign their envelopes, the companion mounts the two public keys. Existing keys are kept and the configs are rewritten from them, so the recipe is idempotent. `testbed/secrets/` is gitignored and never committed. Each sidecar mounts only its own config, read-only, as `/secrets/wg0.conf`. To rotate keys, delete `testbed/secrets/` and run `just lab-up` again.

## Recipes

```sh
just lab-keys                 # WireGuard keys and configs into testbed/secrets/
just lab-up                   # build images, keys, then compose down and up -d
just lab-check boot lan       # lab check boot --profile lan in the checks container
just lab-check skeleton geo   # the live video path under geo (docs/measurement.md)
just lab-check mavlink geo    # the MAVLink path under geo (docs/measurement.md)
just lab-check control-freshness geo  # the control lease and command freshness under geo
just lab-check qgc-status geo # the ground's freshness output and camera under geo
just lab-check adaptation bandwidth-steps            # the vehicle's adaptation to uplink rate steps
just lab-check adaptation lte-bad --duration-s 120   # further arguments go to `lab check`
just ingest-check lan         # the camera ingest path with a test-pattern feeder in place of a webcam
just detail-request 10        # ask for the retained frame from 10 s ago; just detail-fetch <id> file.jpg saves it
just detail-outage            # list the snapshots taken while the ground could not hear the vehicle, with positions
just take-control             # the ground adapter takes the vehicle's control lease (docs/qgc.md)
just control-status           # its lease
just release-control          # give it back
just video-hook pause         # the vehicle's test-only video hooks: pause, freeze, resume
just lab-down                 # remove the sat-teleop containers and networks
```

`just lab-up` first runs `just lab-down`, then the image builds (`runtime` and `sitl-image`); cached layers make them quick when nothing changed, and source changes rebuild the binaries. The testbed goes down first because services share sidecar network namespaces, and Podman cannot recreate a sidecar (for a new image) while services still use it, and because a build never starts beside a running testbed (`docs/development.md`). On a Podman machine it then makes sure forwarding is on for `ground_net`'s bridge (see "macOS and Podman notes"). `just lab-check <scenario> <profile> [args]` runs `lab check` in `checks` with the working directory `/sat-teleop`, where `testbed/profiles/` is mounted read-only and `results/` is the repository's `results/` directory. It passes `git describe --always --dirty` as the revision and exits with the check's code. Only the host can kill a container, so when a check prints the line `lab: kill ground` on stdout (`control-freshness` does, mid-drive), the recipe runs `compose kill ground` and `compose start ground` and passes every other line through; the adapter restarts at once, without a lease.

Cleanup: `just lab-down` runs `compose down --remove-orphans`, which removes only the `sat-teleop` containers and networks, and on a Podman machine the leaked netavark drop-ins ("macOS and Podman notes"). Images (`localhost/sat-teleop-*`), `testbed/secrets/` and `results/` stay. Remove them explicitly if wanted, for example `podman rmi localhost/sat-teleop-runtime:dev`; never prune the shared machine.

To apply a profile by hand, go through `checks` (the agent is on `ctl_net` only):

```sh
podman compose -p sat-teleop -f testbed/compose.yml exec checks \
  lab impair --agent http://192.168.233.254:9000 --profile testbed/profiles/geo.toml
```

## SITL

`testbed/Containerfile.sitl` builds `localhost/sat-teleop-sitl:dev` (`just sitl-image`, `--platform linux/amd64`) from Debian trixie slim (pinned by index digest, see "macOS and Podman notes") and two downloads pinned by SHA-256 with `ADD --checksum`: the official ArduRover 4.7.2 SITL binary (statically linked x86_64, ArduPilot git `fb773567896014da007bca58358704d4a0d8516f`, 6845664 bytes, `f453fd8217b9d798c873e5c424e139a2a3597b52e31658f76b2c177b523391af`) and `rover.parm` from the same commit (606 bytes, `6479879d3e625854a6c32cb4d3fe231bab81d935e07904eac8a411fd6f971bc4`). It runs the `rover` model in real time (`--speedup 1`), instance 0, at a fixed home in a field near Stockholm (59.3293 N, 18.0686 E, 10 m, heading 0). Serial 0 is MAVLink on tcp 5760; SITL waits there until a client connects, then also opens the other ports named in "Topology", all kept to loopback by `sitl.nft`. Parameters and logs live in the container's `/var/lib/sitl`, so every `lab-up` starts from the defaults. The vehicle is system id 1, component 1. The vehicle sends control to it only once it has identified it as SITL (`docs/architecture.md`, "Control protocol"); every check passes with it on.

`testbed/sat-teleop.parm` (copied into the image as `/sitl/sat-teleop.parm` and loaded after `rover.parm`, which stays as pinned) makes SITL stop on its own when the vehicle stops refreshing control: `RC_OVERRIDE_TIME 0.5` (s; a `MANUAL_CONTROL` override ends 0.5 s after the last one, default 3), `FS_GCS_ENABLE 1`, `FS_GCS_TIMEOUT 2` (s, range 2-120, default 5), `FS_TIMEOUT 1` (s before the action, range 1-100, default 1.5) and `FS_ACTION 2` (Hold, the default). Names, units and ranges were checked in ArduPilot `fb773567896014da007bca58358704d4a0d8516f` (`Rover/Parameters.cpp`, `libraries/RC_Channel/RC_Channels_VarInfo.h`; behaviour in `Rover/Rover.cpp` `gcs_failsafe_check` and `Rover/failsafe.cpp`), and SITL prints no "Ignored unknown param" for the file. With no RC transmitter, SITL's rover reads a neutral throttle once the override ends, and its model has no brakes: speed decays with a 2 s time constant.

Measured on the development Mac (2026-11-01, Podman machine with 5 CPUs and 2 GiB, the whole testbed running): under qemu SITL takes about a third of one core (`podman stats --no-stream` 32.7 %, the container's cgroup 33 % over 20 s) and 13 MB (`podman stats`; the emulated process has an RSS of 19 MB). It keeps real time (`docs/qgc.md`, "Verified without QGC"). For comparison, `companion` takes 9 %, `mavlink-router` and `ground` under 0.1 %.

The simulated flight swaps this rover for an ArduCopter flown in Gazebo through `testbed/compose.sim.yml`: `docs/simulation.md`.

## MAVLink path

`mavlink-router` (v4, the release binary in the runtime image) runs in the relay namespace with `testbed/mavlink-router.conf`: a UDP server endpoint `vehicle` bound to the overlay address 10.77.0.1:14550, which replies to whoever last sent to it, and UDP client endpoints `ground`, which sends to 10.77.0.3:14551 from a port it picks itself and reads whatever arrives on that port (in v4 from any sender, and replies follow the latest sender for 5 s), and `checks`, which sends the same telemetry to 10.77.0.4:14553 so `lab check control-freshness` can watch SITL while the ground adapter is down. Its TCP server is off and message filters stay at their defaults (none): it only routes. Binding 10.77.0.1 fails until `wg0` is up, and mavlink-routerd then exits, so the service restarts on failure.

```text
SITL tcp 127.0.0.1:5760 <-> companion (sat-teleop-vehicle) <-> 10.77.0.1:14550 mavlink-router <-> 10.77.0.3:14551
                            budget up, gate down          (WireGuard through the router)
10.77.0.3:14551 ground (sat-teleop-ground) <-> one socket per ground station <-> QGC (host.containers.internal:14550)
                                                                       <-> lab in checks (192.168.232.40:14552)
```

`companion` connects to SITL at `127.0.0.1:5760` (`[mavlink]` in `testbed/config/vehicle.toml`), reconnecting with backoff (0.5 s doubling to 5 s; 5 s without data counts as a hang). It sends SITL's telemetry to mavlink-router one MAVLink frame per datagram, within the budget `[telemetry] bytes_per_s` in `testbed/limits.toml` (8000 frame bytes per second; UDP, IP and WireGuard headers not counted) until the vehicle has measured its uplink, then within the uplink budget's telemetry share: two and a half times what SITL offers, at least 200 B/s for the vehicle state, at most 8000 (`docs/architecture.md`, "Adaptation"). Over budget it drops whole messages, never the vehicle state first: the autopilot's `HEARTBEAT` always goes out (spending the bucket when it can), state (`SYS_STATUS`, `GLOBAL_POSITION_INT`, `ATTITUDE`, `COMMAND_ACK`) may use the whole one-second bucket, other messages only its upper half. Replies to admitted requests (`AUTOPILOT_VERSION`, `PARAM_VALUE`, mission download) wait in a bounded queue (64 KiB, 10 s) instead of being dropped, because ArduPilot answers a parameter request with its whole list in about 0.6 s (1256 values, about 46 KB, ten times the budget); bulk messages are dropped while replies wait. Frames are read with the `mavlink` crate's `ardupilotmega` dialect (which includes `common`); a message id outside it fails the CRC check and is not forwarded, and `sitl_bytes` against `telemetry.bytes_in` shows how much that is.

From mavlink-router it reads only datagrams from 10.77.0.1:14550 and writes to SITL only the read-only traffic QGroundControl needs to load a vehicle (parameter and mission downloads, stream rates, time sync, and `COMMAND_LONG`/`COMMAND_INT` with `REQUEST_MESSAGE`, `SET_MESSAGE_INTERVAL`, `REQUEST_PROTOCOL_VERSION` or `REQUEST_AUTOPILOT_CAPABILITIES`), lease or not. Everything else, a ground station's heartbeat, arming, mode changes, parameter and mission writes, manual control and RC overrides included, is dropped and counted by reason; `mavlink.gcs_refused` in the vehicle's metrics counts each refused message (and command) by name. State changes reach SITL only in control envelopes on the vehicle's control endpoint, 10.77.0.2:7700, from the lease holder (`docs/architecture.md`, "Control protocol"). The lists are in `crates/control/AGENTS.md`. Nothing is queued on the way to SITL, so a reconnect never replays a command.

`testbed/limits.toml` holds operational limits: the telemetry budget, the control lifetimes (`[control]`), the ground output's freshness thresholds (`[freshness]`) and the vehicle's adaptation thresholds (`[adapt]`). Each application reads only its own tables (the vehicle `[telemetry]`, `[control]` and `[adapt]`, the ground `[control]` and `[freshness]`, the lab `[control]` and `[freshness]`) and accepts the others; unknown keys inside a table it reads are rejected.

`ground` runs `sat-teleop-ground` with `testbed/config/ground.toml`. It binds the overlay address 10.77.0.3:14551 (retrying every second until `wg0` is up), accepts telemetry only from mavlink-router's address 10.77.0.1 and remembers the port it sends from. Every ground station in `gcs` gets the telemetry from a UDP socket of its own, and hears what it sends back on that socket (only from its own address). Messages addressed to the camera component (100) are answered by the adapter itself and never forwarded (`docs/qgc.md`, "Camera discovery"). Read-only frames go raw to mavlink-router; the rest goes to the adapter's controller on 10.77.0.3:7701 (`docs/qgc.md`, "What QGC can and cannot do"). `just take-control`, `just release-control` and `just control-status` run `sat-teleop-ground take-control | release-control | status` in the `ground` container against its `ctl_net` endpoint.

## Boot check

`lab check boot --profile <name>` applies the profile and checks that the topology works. It runs entirely inside `checks`, which is a ground host, sits on `ctl_net` and joins the overlay as 10.77.0.4. Running everything there gives one runner, one `summary.json` and one exit code, with no assertions in shell. Facts that `checks` cannot see directly are proven by behaviour instead: a reply from 10.77.0.2 means the vehicle and relay have a WireGuard session, and a forwarding cut that silences 10.77.0.2 means the vehicle's only path to `ground_net` is the router.

| Assertion | Pass when |
|---|---|
| `agent_reachable` | `GET /timeline` on `192.168.233.254:9000` answers. |
| `profile_applied` | `POST /profile` succeeds; its report is stored as `effective_profile`. |
| `agent_only_on_ctl_net` | Nothing accepts a connection on `192.168.232.254:9000`. |
| `overlay_ground` | 10.77.0.3 replies within 30 s (handshakes retry every 5 s after a fresh start). |
| `overlay_vehicle` | 10.77.0.2 replies within 30 s. |
| `overlay_rtt` | At least 5 of 10 pings to 10.77.0.2 are answered (otherwise `incomplete`) and their median lies within the configured round trip (uplink plus downlink `delay_ms`) minus 50 ms to plus 100 ms. For `geo` that is the 550-700 ms calibration target; for `lan`, 0-100 ms. |
| `cgnat` | None of 10 pings to 192.168.231.10 is answered. `checks` routes `vehicle_net` via the router, so this exercises the router's filter, not just Podman's internal-network isolation. |
| `ctl_net_carries_no_data` | 192.168.233.254 (the unfiltered router) answers a ping from `checks`, proving the probe path, and then none of 10 pings to the vehicle's `ctl_net` address 192.168.233.10 is answered. |
| `forwarding_cut` | After `POST /forwarding {"enabled": false}`, none of 10 pings to 10.77.0.2 is answered. |
| `forwarding_recovery` | After `POST /forwarding {"enabled": true}`, 10.77.0.2 replies within 25 s (one keepalive plus margin). |

Probes are ICMP echo with a 2 s reply timeout. Waiting for a reply (`overlay_*` and `forwarding_recovery`) sends one probe at a time, at most every 0.5 s, until the first reply or the deadline, so a lost probe, an ICMP error or a NAT mapping that only comes back with the next keepalive cannot end the wait early. Silence (`cgnat`, `ctl_net_carries_no_data`, `forwarding_cut`) is a stream of 10 probes 0.2 s apart that must all go unanswered; a stream that could not be sent in full is `incomplete`. A few lost probes on a lossy profile therefore cannot make an open path look silent.

The ping from `checks` to 10.77.0.2 travels to the relay unimpaired, then through the router both ways, so its RTT covers both impaired directions. If the vehicle does not answer, `cgnat`, `ctl_net_carries_no_data` and the cut assertions are `incomplete` rather than passed, because silence from a dead vehicle proves nothing. Forwarding is restored even when the cut step fails. A short cut recovers within one round trip because the router's NAT mapping survives it (keepalives keep it fresh); the 25 s deadline covers a keepalive re-creating the mapping after a longer one.

Results go to `results/<unix-ms>-boot-<profile>/`: `summary.json` (verdict, each assertion with detail, metrics `overlay_rtt_median_ms` and `forwarding_recovery_ms`, reply counts per probe, the thresholds used, the agent's effective profile and the git revision) and `timeseries.csv` (`unix_s,series,seq,value`: one row per reply, `series` the probe, `seq` the ICMP sequence, `value` the RTT in ms). The last line printed is the verdict. Exit code 0 is pass, 1 fail, 2 incomplete; a check that cannot read its profile or write its results is incomplete. Image ids are not recorded yet. The measured round trips are in `docs/measurement.md`, "Boot check".

## Live video path

`companion` runs `sat-teleop-vehicle` with `testbed/config/vehicle.toml`: a live `videotestsrc` ball pattern at the rung the vehicle's adaptation picks from `testbed/ladder.toml` (it starts at the top rung, `--start-rung 0` in `testbed/compose.yml`, 1280x720, 15 fps, 1200 kbit/s; a keyframe every 2 s), stamped, encoded by x264 (`tune=zerolatency`, `speed-preset=ultrafast`, no B-frames) and sent as RTP to 10.77.0.1:5004 inside WireGuard. It probes the relay on 10.77.0.1:5006 every 250 ms and changes rung from the answers (`docs/architecture.md`, "Adaptation"); under `lan` and `geo` it stays at the top rung. For a clip instead, set `kind = "file"` and `path` under `[source]` and mount the y4m file into `companion`; it is paced to real time and loops. `relay` runs `sat-teleop-relay` with `testbed/config/relay.toml`: it receives on the overlay address only, passes a 50 ms jitter buffer (the longest wait for a missing packet; in-order frames are never held) and publishes to MediaMTX at `rtsp://127.0.0.1:8554/live`; on 10.77.0.1:5006 it answers each of the vehicle's probes with what it received since its previous answer (`[feedback]` in `testbed/config/relay.toml`). Neither the overlay nor the sidecars filter that port: WireGuard's `AllowedIPs` let the vehicle reach 10.77.0.1 and route the answers back to 10.77.0.2, and the nftables rules only guard `ctl_net` and SITL's ports. MediaMTX (`testbed/mediamtx.yml`) accepts publishing to `live` from localhost only, lets anyone who reaches it read, serves RTSP (UDP and TCP) and WebRTC, and has RTMP, HLS, SRT and MoQ off. Pipelines and their invariants are in `crates/media/AGENTS.md`.

`testbed/config/vehicle.toml` sets `debug_hooks = true`, which only a build with the `testbed` cargo feature accepts (`just image runtime` passes it; the release images do not, and their vehicle refuses a config that names it): the vehicle's metrics endpoint then takes `POST /video/pause` (frames are dropped before the encoder), `/video/freeze` (the next frame is sent again and again, stamp and all) and `/video/resume`, which `lab check qgc-status` and `just video-hook <action>` use; without it they answer 403.

`ground` runs the freshness output (`[video]` in `testbed/config/ground.toml`, `docs/qgc.md`): it pulls `rtsp://192.168.232.20:8554/live` over TCP, decodes it, marks every frame with its freshness (thresholds in `[freshness]` of `testbed/limits.toml`) and sends it as RTP/H.264, 640x360 at up to 2000 kbit/s, to QGC and to `checks` (the 5600 and 5602 rows of the port table). It announces the stream to QGC as the vehicle's camera.

All three applications restart a failed pipeline after one second (for example until `wg0` is up) and report counters and health (`runs`, `running`, `last_error`) on `GET /metrics` (the vehicle's also carries its MAVLink counters under `mavlink`, the ground's its video under `video`):

```sh
podman compose -p sat-teleop -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/vehicle.token)" http://192.168.233.10:9100/metrics'
podman compose -p sat-teleop -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/job.token)" http://192.168.233.20:9100/metrics'
podman compose -p sat-teleop -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/ground.token)" http://192.168.233.30:9100/metrics'
```

`sat-teleop-ground` reports its overlay socket (`listen`), where mavlink-router was last heard from (`relay_endpoint`), per direction (`telemetry` towards the stations, `gcs` from them) messages and bytes in and out and drops by reason, its camera's counts (`camera`) and its video output (`video`: both pipelines' health, frames decoded, new and unreadable, frames sent by state, the current state); `apps/ground/AGENTS.md` lists every field.

### Viewing the stream

QGroundControl finds the ground adapter's freshness output on its own (`docs/qgc.md`). The direct RTSP stream below is lower in latency and shows nothing of freshness; to use it, follow `docs/qgc.md`, "Video: the freshness output" (remove `camera_uri`), then in QGC open Application Settings, Video and set:

- Video Source: "RTSP Video Stream"
- RTSP URL: `rtsp://127.0.0.1:8554/live`
- Low Latency Mode: on

RTSP must run over TCP from the Mac: MediaMTX sends RTP over UDP to the address the RTSP connection came from, which is Podman's port forwarder, so UDP media never reaches the Mac. GStreamer's RTSP source, which QGC uses, tries UDP first and switches to TCP when no UDP data arrives within its timeout, so the picture can take a few seconds to appear. Checked on 2026-11-01 with QGroundControl 5.1.5 on macOS: with these settings the stamped test picture appears in the video inset and advances. From a terminal on the Mac:

```sh
ffprobe -v error -rtsp_transport tcp -show_streams -select_streams v rtsp://127.0.0.1:8554/live
# codec_name=h264  profile=Constrained Baseline  width=640  height=360  r_frame_rate=15/1
ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/live
```

With `-rtsp_transport udp` the session is set up but no frame arrives.

Optional browser view: MediaMTX's WebRTC player at `http://127.0.0.1:8889/live` in Chrome Canary. Its media uses ICE over UDP to MediaMTX's port 8189, published on host `127.0.0.1:8189/udp`, and `testbed/mediamtx.yml` offers `127.0.0.1` as a candidate (MediaMTX's documented setup behind a NAT or container). Checked on 2026-11-01 in Chrome Canary: the player shows the 640x360 picture and it advances.

## Skeleton check

`lab check skeleton --profile <name>` measures the live video path from `checks`: it applies the profile, plays `rtsp://192.168.232.20:8554/live` in a headless receiver configured like QGC (RTSP over UDP, 80 ms latency, `avdec_h264`, non-syncing sink) and reads the stamp of every decoded frame. Options: `--warmup-s` (default 5), `--duration-s` (default 30), `--url`, `--vehicle-metrics` and `--relay-metrics` (the metrics base URLs). `docs/measurement.md` defines the phases, every metric, the result files and how budgets grade them.

| Assertion | Pass when |
|---|---|
| `profile_applied` | The router agent applied the profile. |
| `ladder_settled` | The vehicle's ladder settled before measuring (`docs/measurement.md`, "Adaptation check", step 1); `incomplete` otherwise: the run would measure a transient, or a rung the profile does not hold it at. |
| `video_received` | At least one new frame (readable, plausible stamp, unseen frame id) arrived in the window. |
| `forwarding_cut` | After `POST /forwarding {"enabled": false}` and a drain time, no frame arrives for 2 s. `incomplete` without video before the cut. |
| `video_recovery` | After restoring forwarding, a fresh frame (captured after the cut began) arrives within 15 s. |
| `budget`, `budget.<metric>` | The run is graded against `perf/budgets.toml`: `incomplete` in calibration or without a budget, otherwise one assertion per limit. |

Results go to `results/<unix-ms>-skeleton-<profile>/`.

## MAVLink check

`lab check mavlink --profile <name>` makes `checks` a ground station: it listens on 192.168.232.40:14552, where `ground` sends it telemetry from a socket of its own, and answers that socket with a GCS heartbeat every second and its requests, exactly as QGC does on the Mac. Options: `--duration-s` (default 30), `--gcs-listen`, `--vehicle-metrics` and `--ground-metrics`. `docs/measurement.md` defines the phases and metrics.

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

Results go to `results/<unix-ms>-mavlink-<profile>/`.

## Control freshness check

`lab check control-freshness --profile <name>` exercises the control lease and command freshness (`docs/architecture.md`, "Control protocol"), run through `just lab-check` so the adapter can be killed. `checks` plays three parts: a second controller on the overlay (10.77.0.4) that sends envelopes itself, a ground station of the adapter at 192.168.232.40:14552 that commands and drives as QGC does, and an observer of SITL's telemetry straight from mavlink-router at 10.77.0.4:14553. Each run first force-disarms SITL through the adapter, asks for `VFR_HUD` and `GPS_RAW_INT` at 10 Hz and frees the lease, and ends the same way with the default rates. `docs/measurement.md` defines the phases and metrics.

| Assertion | Pass when |
|---|---|
| `profile_applied`, `telemetry_received` | As for the MAVLink check; the heartbeat must arrive both through the adapter and from mavlink-router. |
| `invalid_refused` | `checks` sends the invalid envelopes and releases listed in `docs/measurement.md`, "Control freshness check", step 2, each carrying a forced arm or full throttle; the vehicle refuses each for that rule (an answer of any other kind fails), applies none (its `applied` discrete and continuous counts rise by the valid envelopes only) and stays disarmed. Its liveness count must have risen by exactly the valid liveness envelopes, or nothing applied proves nothing (`incomplete`). |
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

Results go to `results/<unix-ms>-control-freshness-<profile>/`.

## Real-mode check

`just lab-check real-mode lan` runs the vehicle in real mode (`testbed/config/vehicle-real.toml`) against the ArduRover SITL, which real mode accepts only because of the test-only `sitl_in_real_mode` (testbed builds only; the loader refuses it unless `debug_hooks` is set, and refuses `debug_hooks` in real mode without it). `checks` asks the recipe to start the companion on that config (`lab: vehicle-config <file>` on stdout; the recipe runs `compose up -d --no-deps companion` with `VEHICLE_CONFIG` set), and starts it on `vehicle.toml` again at the end, so it expects the default testbed. Assertions (`docs/real-vehicle.md` explains each interlock):

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

## QGC status check

`lab check qgc-status --profile <name>` checks the ground's freshness output from `checks`: it receives the output on 192.168.232.40:5602 as QGC receives it (RTP/UDP, no jitter buffer), reads every frame's stamp and state stamp, asks the adapter's camera what QGC asks it, and stalls the source with the vehicle's test-only hooks. Options: `--warmup-s` (default 5), `--duration-s` (the healthy window, default 30), `--output-listen`, `--gcs-listen`, `--vehicle-metrics` and `--ground-metrics`. It reads `[freshness]` of `testbed/limits.toml`. `docs/measurement.md` defines the phases and metrics.

| Assertion | Pass when |
|---|---|
| `profile_applied` | As for the skeleton check. |
| `ladder_settled` | As for the skeleton check. |
| `camera_answered` | The camera answers what QGC asks of it, and the vehicle receives nothing from the ground meanwhile (`docs/measurement.md`, "QGC status check", step 2); otherwise fail. A QGC that is still loading the vehicle sends raw requests of its own and would fail this, an idle one does not. No heartbeat or no vehicle reading is `incomplete`. |
| `output_received` | A frame with a state stamp arrives within 30 s after the warm-up. |
| `healthy_live` | At least one frame of the healthy window reads Live; the share is graded by the budget. |
| `pause_indicated`, `freeze_indicated`, `blackout_indicated` | After the vehicle stops its video (pause), repeats one frame (freeze) or the profile's timeline blacks out the link (`outage-10s` only): Stale and then Disconnected show (pause, blackout) or Frozen shows (freeze), and no frame reads Live or LowRate after the first one that read Stale, Frozen or Disconnected until the release (AgeUnknown, which follows lost packets, says nothing about the picture's age). A hook that cannot be used is `incomplete`. |
| `pause_recovered`, `freeze_recovered`, `blackout_recovered` | A Live or LowRate frame within 15 s of the release (the detail gives when the first Live one came). |
| `never_falsely_live` | No frame of the run reads Live while its own stamp, as received, is older than `stale_after_ms` plus 100 ms, or LowRate while older than `low_rate_stale_after_ms` plus 100 ms. |
| `budget`, `budget.<metric>` | Graded against `perf/budgets.toml` as for the other checks. |

Results go to `results/<unix-ms>-qgc-status-<profile>/`. QGC itself is checked by hand (`docs/qgc.md`, "Checking it in QGC").

## Adaptation check

`lab check adaptation --profile <name>` checks the vehicle's adaptation (`docs/architecture.md`, "Adaptation") from `checks`: it plays the relay's stream like the skeleton check, reads the vehicle's ladder and the router's counters, and runs the profile's timeline. `bandwidth-steps` is the graded profile: its uplink steps from 512 to 128, 64, 256 and 512 kbit/s, 15 s apart. Profiles without a timeline (`lte-bad`, `geo-degraded`) are watched for `--duration-s` (default 30; 120 for the recorded runs). Options: `--warmup-s`, `--duration-s`, `--url` and `--vehicle-metrics` as for the skeleton check. `docs/measurement.md` defines the phases and metrics.

| Assertion | Pass when |
|---|---|
| `profile_applied` | As for the skeleton check. |
| `ladder_settled` | As for the skeleton check. |
| `second_viewer` | What the vehicle offers the uplink (the router's bytes from `vehicle_net`) while a second receiver plays the stream too is measured against the mean with one receiver before and after (`viewer_change_percent`, which the budget grades). `incomplete` if the rung changed meanwhile or the second receiver got nothing. |
| `video_received` | New frames arrived during the observation. |
| `step_<n>_offered_below_cap` | After the n-th downward step of the uplink rate, the vehicle offers less than the new cap by the end of the phase (the detail gives the settling time); fail if it is still above it. |
| `step_<n>_age_recovered` | After that step, frames are back within 150 ms of the phase's final median age and come again at the phase's final pace (the detail gives when, and how far that level lies above the base); `incomplete` without frames to judge by. |
| `budget`, `budget.<metric>` | Graded against `perf/budgets.toml` as for the other checks: the settling time, windows over the cap after the settling deadline, what is left of a backlog, how long it took to drain, how long a drain paused the video, rung changes per minute and the second viewer's change. |

Results go to `results/<unix-ms>-adaptation-<profile>/`.

## Ingest check

`just ingest-check <profile>` (default `lan`) grades the camera ingest without a camera. It brings the testbed up with `VEHICLE_CONFIG=vehicle-ingest.toml`, starts the `feeder` container, runs `lab check ingest` and `lab check skeleton` (the video through the link, graded by the skeleton budgets), then `just ingest-isolation`, and stops the feeder. `lab check ingest --profile <name>` applies the profile, waits `--warmup-s`, reads the vehicle's metrics, waits `--duration-s` and reads them again. Options: `--warmup-s`, `--duration-s` and `--vehicle-metrics`; the recipe passes 10 s for both.

| Assertion | Pass when |
|---|---|
| `profile_applied` | As for the other checks. |
| `feeder_decoded` | The ingest decoded at least 80 % of the feeder's 30 frames per second over the window. |
| `foreign_address_refused` | Datagrams from the feeder's second address (192.168.234.6, not in `from`) were dropped and counted (`ingest.refused` rose). |
| `decode_cost` | The mean time the decoder took per frame, since the vehicle started, is at most 20 ms. The mean and the maximum are recorded as `ingest_decode_ms_mean` and `ingest_decode_ms_max`. |

`just ingest-isolation` is a recipe, not a check, because it needs a container on `ingest_net` (the feeder, or a one-off one). It runs in `just gate-full` and `ingest-check` against any testbed. It requires the vehicle sidecar to publish exactly `127.0.0.1:5700/udp`, then reads the counter of each drop rule in `ingest.nft` before and after a probe and requires an increase: a TCP connection from the ingest side to the sidecar (input), the companion reaching the host on the ingest side (output), and packets routed through the sidecar from the ingest side to the router and from the router to the ingest side (forward, one rule each). A timeout would prove nothing, because replies leave through the router either way. Deleting any one drop rule makes it fail. `lab check ingest` results go to `results/<unix-ms>-ingest-<profile>/`.

## Regional check

`lab check regional --profile <name>` measures what regional video (`docs/architecture.md`) gains and costs. It needs the vehicle on `testbed/config/vehicle-regional.toml` (a one-frame clip of the check's test card as the source, a small snapshot store the pilot's box crops are cut from, `soften_factor = 2` and `debug_hooks` on; `vehicle-regional-detector.toml` also runs the person detector): `VEHICLE_CONFIG=vehicle-regional.toml just lab-up`, then `just lab-check regional geo --warmup-s 8 --duration-s 20` (`--rung <n>` pins the first two windows to ladder rung n, the third is the rung below), with the same `PI=pi4` or `PI=pi-zero-2w` for both to run on a small board's budget (`docs/latency.md`). It is not in `gate-full`. The check writes the clip (one 1280x720 frame, which the vehicle's `still` file source shows for good) to the vehicle's snapshot volume, which the vehicle's sender opens, and plays the ground's output on 192.168.232.40:5602 and scores every third frame against the card scaled to the output's size (SSIM in 8x8 windows), then, as `QGroundControl` does:

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
| `budget` | `regional.<profile>` in `perf/budgets.toml`; calibration today. |

The metrics are `region_ssim_with`, `region_ssim_without`, `region_ssim_gain`, `outside_ssim_vs_lower_rung`, `test_object_contrast_ratio`, `frame_age_p95_ms`, and from the vehicle's metrics over the first window `vehicle_cpu_percent` (its process CPU time over the window, of one host core), `vehicle_rss_mb`, `soften_ms_p95` and `detector_ms_p95` (when the detector ran). Results go to `results/<unix-ms>-regional-<profile>/`.

## Quality check

`lab check quality --profile <name>` grades the picture's quality next to its age (`docs/measurement.md`, "Picture quality"). It needs the vehicle on `testbed/config/vehicle-quality.toml` (a looping clip of a scrolling test scene as the source): `VEHICLE_CONFIG=vehicle-quality.toml just lab-up`, then `just lab-check quality geo --warmup-s 8 --duration-s 30`, with `PI=pi4` or `PI=pi-zero-2w` for both to run on a small board's budget. The check writes the clip (62 MB) to the vehicle's snapshot volume, waits for the ladder to settle wherever the profile lets it, then scores every frame the relay's stream delivers against the clip frame it shows. It is not in `gate-full`.

| Assertion | Pass when |
|---|---|
| `ladder_settled` | The vehicle's ladder did not change for 15 s. |
| `matching_forward` | At most 2 % of the frames step back in the loop between two frames that score at least 0.8. |
| `budget` | `quality.<profile>` in `perf/budgets.toml`. |

Results go to `results/<unix-ms>-quality-<profile>/`.

## Recording check

`lab check recording --profile lan` checks the vehicle's retained snapshots (`docs/architecture.md`, "Retained detail"). `checks` sees the store's directory. The check is run through `just lab-check recording lan`: it needs the host to restart `companion`, which it asks for by printing `lab: restart companion`; run any other way it is `incomplete` at that step. Options: `--warmup-s`, `--url`, `--vehicle-metrics` and `--snapshot-dir` (default `/snapshots`). The store's limits come from `testbed/config/vehicle.toml` (`[store]`).

| Assertion | Pass when |
|---|---|
| `profile_applied`, `ladder_settled` | As for the other checks. |
| `recording_started` | The store wrote three snapshots within 30 s. |
| `quota_held` | The quota was reached (the store evicted), and for 20 s after neither the directory nor the store's own count passed the quota and the oldest snapshot only got newer. `incomplete` if the quota was never reached. |
| `snapshot_cadence` | The median interval between retained snapshots is within 10 % of the configured second. |
| `restart_recovery` | After `companion` is restarted with a half-written `.tmp` and two names that are no snapshots planted: the `.tmp` is deleted, the other names are left alone and counted as ignored, the snapshots on disk are all indexed (within two) and the store writes again. |
| `gap_reported` | A gap around the restart shows in the directory and in the store's `gaps`. |
| `slow_storage_drops` | With every write made to take 3 s (a vehicle hook, `debug_hooks`), snapshots are dropped and counted (`snapshots.dropped`). |
| `slow_storage_recovers` | After the hook is lifted the store writes again. |
| `live_unaffected` | Live frames kept arriving during slow storage; the frame age and pace are graded by the budget against the healthy stretch before. |
| `budget`, `budget.<metric>` | Graded against `perf/budgets.toml`. |

Results go to `results/<unix-ms>-recording-<profile>/`.

## Retrieval check

`lab check retrieval --profile <geo|bandwidth-steps|outage-10s>` checks retrieval of a snapshot or a crop of a frame captured seconds ago (`docs/architecture.md`, "Retained detail") from `checks`, which plays the ground (the job API on `ground_net`) and the stream's viewer. Options: `--warmup-s`, `--duration-s` (a pass under a profile without a timeline), `--url`, `--vehicle-metrics`, `--relay-metrics`, `--snapshot-dir`, `--ground-api` and `--vehicle-api`. The allowance comes from `[retrieval]` of `testbed/limits.toml`.

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

Under `bandwidth-steps` and `outage-10s` the passes run the profile's timeline. Results go to `results/<unix-ms>-retrieval-<profile>/`.

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

Loss lives inside each direction (`[uplink.loss]`, `[downlink.loss]`) so loss can be asymmetric. All percentages must be finite and within 0..=100. `delay_ms`, `rate_kbit` are required in each direction; everything else has a default or is optional.

## Profiles

| Profile | Uplink | Downlink | Notes |
|---|---|---|---|
| `lan` | 1 Gbit, no delay | same | No impairment (rate cap only). |
| `lte-good` | 40 ms, 5 ms jitter, 5 Mbit | 40 ms, 5 ms jitter, 20 Mbit | |
| `lte-bad` | 60 ms, 30 ms pareto jitter, 800 kbit, G-E loss | 60 ms, 30 ms jitter, 3 Mbit, G-E loss | About 5 % mean loss (p / (p + r) = 1.34 / 21.34 = 6.3 % of time in the bad state, times 80 % loss there), bad bursts of 1 / r = 5 packets. |
| `geo` | 300 ms, 10 ms jitter, 512 kbit | 300 ms, 10 ms jitter, 2048 kbit | 600 ms RTT target (550-700 ms measured). |
| `geo-degraded` | 300 ms, 25 ms jitter, 128 kbit, G-E loss | 300 ms, 10 ms jitter, 1024 kbit, 1 % random loss | Asymmetric. |
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

`ordering = "preserve"` does not prevent reordering under jitter: under `geo` (10 ms) the relay receives some video packets after later ones, now and then more than 50 ms behind (`docs/measurement.md`).

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

- `build`: the pinned Rust image (`docs/development.md`, "Toolchain") with GStreamer development packages (`localhost/sat-teleop-build:dev`, used by `just check`). `just image runtime` tags it too, so its layers, apt install included, are not left to a dangling release image that `podman image prune` removes.
Both apt installs use cache mounts (`/var/cache/apt`, `/var/lib/apt`) with Debian's `docker-clean` hook removed, so a real layer rebuild reuses downloaded packages.
- `bins-native`: from `build`, runs `cargo build --release --locked` with cache mounts for the registry and `target/` (one `target/` cache per build architecture, so a second architecture never empties the first's). `bins-cross-amd64`: from `build` too, cross-compiles the same binaries for `x86_64-unknown-linux-gnu` with the Debian cross toolchain and amd64 GStreamer headers, because `rustc` segfaults under qemu emulation on Apple Silicon. `build` always runs on the build host's own architecture.
- `localhost/sat-teleop-sitl:dev` comes from `testbed/Containerfile.sitl` instead (see "SITL").
- `runtime-base`, `runtime` and `runtime-amd64`: Debian trixie slim (pinned by index digest) with GStreamer runtime plugins (`gstreamer1.0-x` for Pango's `textoverlay`), `iproute2`, `nftables`, `wireguard-tools`, `iputils-ping`, `tcpdump`, `procps`, `curl`, `gstreamer1.0-rtsp` (for `rtspclientsink`), mavlink-router v4 (`mavlink-routerd-glibc-aarch64` or `-x86_64` by `TARGETARCH`, pinned by SHA-256 from the v4 GitHub release), and the release binaries `lab`, `sat-teleop-vehicle`, `sat-teleop-relay` and `sat-teleop-ground` in `/usr/local/bin`: `runtime` from `bins-native` (`localhost/sat-teleop-runtime:dev`), `runtime-amd64` from `bins-cross-amd64` (`just image-for amd64`, `localhost/sat-teleop-runtime:amd64`, `docs/deploy.md`).

## macOS and Podman notes

- The testbed runs inside the Podman machine VM; on the development Mac that is libkrun with 5 CPUs and 2 GiB of memory, shared with other projects. Container builds and `just check` therefore run with one cargo job in a cgroup capped at 1 GiB (`docs/development.md`): at two jobs a cold build or a test link ran the machine out of memory and the kernel's OOM killer fired. Touch only compose project `sat-teleop` and images `localhost/sat-teleop-*`; never restart or reconfigure the machine.
- Verified in that VM (Fedora kernel 7.1, rootful machine, netavark): kernel WireGuard, `netem`, `htb` and nftables NAT work from containers with `NET_ADMIN`.
- Repository bind mounts (`scripts`, `profiles`, `results`, secrets) go through the VM's file share; files written by containers appear as your user on the Mac.
- The SITL image is x86_64 and runs under the VM's qemu binfmt on Apple Silicon. Both it and the runtime image name their Debian base by the multi-platform index digest of `debian:trixie-slim` (`sha256:f6b08b09a05725f9431502d3b5714f5c4576e82c1a84fa1debe0c24a7e0c3ec4`, resolved 2026-11-01), never by tag: building for another platform from a tag would retag the shared local `debian:trixie-slim`, which belongs to the whole VM. Each build checks the digest with the registry, then reuses the local layers.
- Without the `ping_group_range` sysctl, ping fails in Podman containers with `socket: Operation not permitted`.
- Endpoint protection on the Mac may kill the session on reverse-shell-like command lines; keep probes inside containers and use `curl` or `lab` for HTTP (see `AGENTS.md`).
- Published ports and netavark 2.0.0 ([containers/netavark#1497](https://github.com/containers/netavark/issues/1497), fixed on netavark main, not in 2.1.0). Netavark writes `/run/sysctl.d/10-netavark-<bridge>.conf` in the VM and on teardown deletes it only for non-internal networks, so each internal network leaks a drop-in with `forwarding = 0` for its bridge. When a later non-internal network reuses that bridge name, the stale file wins and forwarding is off on the new bridge; for `ground_net` that means published ports accept connections but carry no data. On a Podman machine, `just lab-down` therefore removes the drop-ins of sat-teleop's internal networks (`vehicle_net`, `ctl_net`) once their bridges are gone, and `just lab-up` checks forwarding on `ground_net`'s bridge after starting and, if it is off, turns it on for that bridge only and prints a line naming the issue. Nothing else in the VM is changed.

## Docker equivalence

The recipes take the engine from `CONTAINER_ENGINE` (default `podman`; `just engine=docker ...` works too): `CONTAINER_ENGINE=docker just lab-up`, `just lab-check boot lan` and `just lab-down` run `docker compose -p sat-teleop -f testbed/compose.yml -f testbed/compose.docker.yml ...`. The second file maps `host.containers.internal` (Podman's name for the host, which the ground adapter sends QGroundControl's telemetry and video to) with `host-gateway`. `lab-keys` swaps `--userns=keep-id` for `--user $(id -u):$(id -g)`. Docker already allows unprivileged ping, so the sysctl is redundant there but harmless. The Docker path has not been run end to end here; it needs a host or VM kernel with WireGuard, `netem` and nftables NAT.
