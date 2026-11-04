# Testbed

Build the workspace, run the container lab and read its results here. See [video.md](video.md) for video, [drone.md](drone.md) for operation and [hardware.md](hardware.md) for deployment. Container results do not measure hardware, real links or QGroundControl rendering.

## Development setup

Tool requirements and package rules are in [AGENTS.md](../AGENTS.md). Code lives in `apps/` and `crates/`; the lab in `testbed/`; deployment in `deploy/`; fixture metadata in `fixtures/manifest.toml`; budgets in `perf/budgets.toml`.

## Toolchain

`rust-toolchain.toml` pins Rust 1.99.0, rustfmt and Clippy. The workspace uses edition 2024 and resolver 3. Container bases are `docker.io/library/rust:1.99.0-trixie` and `docker.io/library/debian:trixie-slim`. `libx264-dev` and `libclang-dev` supply dependencies for `crates/x264/build.rs`.

The pin requires a rustup proxy on `PATH`. Check it, or select the toolchain explicitly:

```sh
cargo --version
rustup which cargo
rustup toolchain install 1.99.0 --component rustfmt --component clippy
rustup run 1.99.0 cargo test -p thinlink-video-core --locked
```

On Apple Silicon:

```sh
PATH=$HOME/.rustup/toolchains/1.99.0-aarch64-apple-darwin/bin:$PATH cargo test -p thinlink-video-core --locked
```

## Builds and gates

Pure crates build natively. GStreamer-linked packages build in the container; the host needs no GStreamer.

| Recipe | Runs |
|---|---|
| `just check` | Workspace fmt, Clippy, tests, layers and unsafe-code boundary checks, including `testbed` feature checks. |
| `just check-native` | Pure-package host gates; needs `jq` and Bash; selects pinned toolchain. |
| `just fmt` | Host formatting. |
| `just image <target>` | `runtime` or `build` from `testbed/Containerfile`. |
| `just sitl-image` | linux/amd64 ArduRover SITL image. |
| `just image-for <arch>` | `arm64` or `amd64` release runtime without `testbed`. |
| `just gate-fast <scenario> <profile> [args]` | Rebuild on changed inputs or services; run one check. |
| `just gate-full` | Fresh testbed and default scenarios. |

`BUILD_JOBS=1` and `BUILD_MEMORY=1g` suit a 2 GiB VM. A larger VM can use `BUILD_MEMORY=4g BUILD_JOBS=4 just check`. Stop the lab and Gazebo before building. `lab-up` and `gate-fast` do this before rebuilding.

`gate-fast` hashes Cargo files, `crates`, `apps`, `xtask`, `fixtures` and `testbed/Containerfile`. It compares `.cache/gate-fast.inputs`, running images and service count.

`gate-full` runs `boot lan`, `skeleton lan`, `skeleton geo`, `mavlink lan`, `control-freshness lan`, `qgc-status lan`, `qgc-status geo`, `qgc-status outage-10s`, `adaptation bandwidth-steps`, `recording lan`, `retrieval geo`, `real-mode lan`, then `ingest-isolation`. It completes the set and exits 1 if any check did not pass. Simulation, regional and quality checks are separate.

`unsafe_code = "deny"` applies to the workspace. The FFI boundary is in [AGENTS.md](../AGENTS.md#unsafe-code).

## Image cache

The registry cache is shared. Cargo target caches persist per checkout and build architecture. Run `just forget-checkout <folder>` from another checkout to remove retired output. `just image runtime` tags the build stage; apt uses cache mounts.

Podman recipes drop clean page cache and compact VM memory when fewer than sixteen 512 KiB blocks are free. This addresses `readdirent: cannot allocate memory` while reading the build context.

## CI

[.github/workflows/ci.yml](../.github/workflows/ci.yml) defines PR, `main` push and manual runs. It skips documentation-only changes, cancels superseded runs and limits jobs to 15 minutes. It runs fmt, Clippy, tests and layers; repository settings determine execution.

## xtask

`cargo xtask <command>` aliases `cargo run -q -p thinlink-xtask --` in `.cargo/config.toml`.

| Command | Checks |
|---|---|
| `cargo xtask layers` | Registration, inherited lints and dependency boundaries against `xtask/src/layers.rs`. |
| `cargo xtask fixtures` | Download missing fixtures and verify size and SHA-256. |
| `cargo xtask fixtures --check` | Verify existing fixtures; missing files fail. |

## Fixtures

`fixtures/manifest.toml` supplies metadata. Downloads use `.cache/fixtures/`, a `size_bytes + 1` cap and `.part` files. Size and SHA-256 must match before rename; failure removes the partial file. Missing or corrupt required fixtures fail checks. The fixture is `foreman_cif.y4m`, 352x288, 300 frames, from Xiph.org's derf collection.

## Small-board limits

Set the same `PI` for `just lab-up` and `just lab-check`.

| `PI` | Overlay | Companion CPU / memory |
|---|---|---|
| `pi4` | `testbed/compose.pi4.yml` | 0.5 host core / 4 GiB |
| `pi-zero-2w` | `testbed/compose.pi-zero-2w.yml` | 0.25 host core / 512 MiB |

These quotas use host cores and x264. They do not emulate Pi hardware.

## Lab setup

Compose project `thinlink` uses `testbed/compose.yml`. The router forwards and NATs the vehicle network onto the ground network. `lab router` applies `tc` impairment per direction. Shipped profiles have `provenance = "synthetic"`.

## Topology

Applications share sidecar namespaces through `network_mode: service:<sidecar>`. WireGuard traffic crosses the impaired router to the relay, which forwards between overlay peers.

| Network | Subnet | Members |
|---|---|---|
| `vehicle_net` | 192.168.231.0/24 | router .254, vehicle-net .10; internal |
| `ground_net` | 192.168.232.0/24 | router .254, relay-net .20, ground-net .30, checks .40 |
| `ctl_net` | 192.168.233.0/24 | router .254, vehicle-net .10, relay-net .20, ground-net .30, checks .40; internal |
| `ingest_net` | 192.168.234.0/24 | vehicle-net .10, optional feeder .5 and .6 |

| Namespace | Overlay | Applications |
|---|---|---|
| `vehicle-net` | 10.77.0.2 | `companion`, `sitl` |
| `relay-net` | 10.77.0.1 | `relay`, `mediamtx`, `mavlink-router` |
| `ground-net` | 10.77.0.3 | `ground` |
| `checks` | 10.77.0.4 | `lab check` |

| Port | Address | Use |
|---|---|---|
| 51820/udp | 192.168.232.20 | WireGuard server |
| 9000/tcp | 192.168.233.254 | Router agent |
| 9100/tcp | 192.168.233.10, 192.168.233.20, 192.168.233.30 | Metrics; ground lease endpoints |
| 5004/udp, 5006/udp | 10.77.0.1 | RTP/H.264; probe feedback |
| 5700/udp | 192.168.234.10; host 127.0.0.1:5700 | Camera ingest |
| 5760/tcp | Vehicle loopback | SITL MAVLink |
| 14550/udp | 10.77.0.1 | mavlink-router vehicle endpoint |
| 14551/udp | 10.77.0.3 | Ground overlay socket |
| 14552/udp, 14553/udp | 192.168.232.40, 10.77.0.4 | Checks' ground station; direct observer |
| 7700/udp, 7701/udp | 10.77.0.2, 10.77.0.3 | Vehicle control; ground controller |
| 14550/udp, 5600/udp | `host.containers.internal` | QGC telemetry; freshness video |
| 5602/udp | 192.168.232.40 | Checks' freshness receiver |
| 8080/tcp | 10.77.0.1, 192.168.232.20 | Vehicle and ground job APIs |
| 8554/tcp | Host 127.0.0.1:8554 | MediaMTX RTSP, path `live` |
| 8000-8001/udp | 192.168.232.20 | MediaMTX RTP/RTCP |
| 8889/tcp, 8189/udp | Host 127.0.0.1 | WebRTC page/WHEP; ICE |

`testbed/scripts/router.nft` permits replies to vehicle-originated flows. `testbed/scripts/ctl.nft` limits `ctl_net` to router and metrics TCP traffic; checks can send probes. `testbed/scripts/sitl.nft` restricts SITL ports 5760, 5762, 5763 and 5501 to loopback. `testbed/scripts/ingest.nft` admits only incoming UDP 5700; the ingest default route is removed. These filters keep measurements on the impaired path.

`vehicle-ingest.toml` accepts source `from` addresses 192.168.127.1, 192.168.234.1 and the feeder. The `snapshots` volume is a 96 MiB tmpfs at `/snapshots` in `companion` and `checks`; files disappear when no container mounts it.

## Privileges

No container is privileged. Router, sidecars and `checks` have `NET_ADMIN`. Router and relay enable `net.ipv4.ip_forward=1`. Sidecars set `net.ipv4.ping_group_range` to `0 0`. Application containers inherit namespaces without these capabilities.

## WireGuard overlay and keys

Kernel WireGuard uses `wg-quick`. Clients point to `192.168.232.20:51820`, with `AllowedIPs = 10.77.0.0/24` and `PersistentKeepalive = 15`; the relay assigns each a `/32`.

`just lab-keys` writes mode-600 files in `testbed/secrets/`: `<node>.key`, `.pub`, `.conf` for `relay`, `vehicle`, `ground`, `checks`; `vehicle.token`, `ground.token`, `job.token`; `ctl-ground.key`/`.pub`, `ctl-checks.key`/`.pub`. Existing keys persist; configs regenerate. Sidecars mount `/secrets/wg0.conf`. Delete `testbed/secrets/` and run `just lab-up` to rotate.

## Recipes

| Command | Action |
|---|---|
| `just lab-keys` | Generate keys and configs. |
| `just lab-up` | Stop lab, build images and keys, start topology. |
| `just lab-check <scenario> <profile> [args]` | Run in `checks`, preserving exit code. |
| `just video-hook pause` | Pause video; also accepts `freeze` and `resume`. |
| `just lab-down` | Remove project containers/networks; keep images, keys and results. |

Checks run in `/thinlink`; `results/` maps to the checkout. The recipe passes `git describe --always --dirty`. Host requests on stdout restart services: `lab: kill ground`, `lab: restart companion`, `lab: vehicle-config <file>`. Use the recipe for checks requiring these actions.

```sh
podman compose -p thinlink -f testbed/compose.yml exec checks \
  lab impair --agent http://192.168.233.254:9000 --profile testbed/profiles/geo.toml
```

## Test hooks and metrics

`debug_hooks = true` in `testbed/config/vehicle.toml` requires the `testbed` Cargo feature. Release builds reject it. `POST /video/pause`, `/video/freeze`, `/video/resume` return 403 when disabled.

`GET /metrics` reports pipeline `runs`, `running`, `last_error`; failed pipelines restart after one second. Vehicle control counters are under `mavlink`; ground video under `video`.

```sh
podman compose -p thinlink -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/vehicle.token)" http://192.168.233.10:9100/metrics'
podman compose -p thinlink -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/job.token)" http://192.168.233.20:9100/metrics'
podman compose -p thinlink -f testbed/compose.yml exec checks sh -c 'curl -s -H "Authorization: Bearer $(cat /secrets/ground.token)" http://192.168.233.30:9100/metrics'
```

## Checks

Tables distinguish assertions from recorded metrics. Shared names apply where emitted by the check:

| Name | Meaning |
|---|---|
| `profile_applied` | Router accepted profile. |
| `ladder_settled` | Rung steady within 120 s deadline. |
| `ladder_settle_ms`, `settled_rung` | Settle duration and resulting rung. |
| `budget`, `budget.<metric>` | Budget availability and each enforced limit. |

Adaptation waits 15 s at a steady rung; skeleton and QGC status wait 6 s at the top. Timeline adaptation and retrieval require the top rung. Skeleton and adaptation count new frames with readable stamps, age 0-60 s and unseen frame IDs toward age and cadence.

### Boot check

`just lab-check boot lan` checks overlay reachability and network paths.

| Assertion | Pass condition |
|---|---|
| `agent_reachable`, `agent_only_on_ctl_net` | Router answers on `ctl_net` only. |
| `overlay_ground`, `overlay_vehicle` | Peers reply within 30 s. |
| `overlay_rtt` | At least 5/10 replies; median within configured RTT minus 50 to plus 100 ms. |
| `cgnat` | No direct vehicle-network replies. |
| `ctl_net_carries_no_data` | Router answers; vehicle's control-network address stays silent. |
| `forwarding_cut`, `forwarding_recovery` | Cut silences overlay; restore yields reply within 25 s. |

Silence needs ten sent probes and evidence of vehicle reachability; missing evidence is incomplete.

| Metric | Meaning |
|---|---|
| `overlay_rtt_median_ms`, `forwarding_recovery_ms` | Probe RTT; restore-to-reply time. |

### Skeleton check

`just lab-check skeleton geo` measures `rtsp://192.168.232.20:8554/live` using RTSP/UDP, 80 ms latency and `avdec_h264`. Options: `--warmup-s` (5), `--duration-s` (30), `--url`, `--vehicle-metrics`, `--relay-metrics`.

| Assertion | Pass condition |
|---|---|
| `video_received` | New frames arrived. |
| `forwarding_cut` | No frames for 2 s after cut/drain. |
| `video_recovery` | Fresh frame within 15 s of restore. |

| Metric | Meaning |
|---|---|
| `frame_age_p50_ms`, `frame_age_p95_ms`, `frame_age_max_ms` | New-frame ages. |
| `new_frames_per_s`, `max_frame_gap_ms` | Cadence; longest gap including window edges. |
| `unreadable_stamps`, `unreadable_ratio`, `implausible_stamps` | Unreadable count/share; ages outside 0-60 s. |
| `repeated_frames`, `skipped_frame_ids` | Repeated IDs; missing intervening IDs. |
| `uplink_bytes_per_s`, `uplink_drops` | Router wire rate; qdisc drops. |
| `vehicle_frames_per_s`, `relay_rtp_bytes_per_s`, `relay_lost_packets` | Encoder cadence; relay RTP rate/losses. |
| `cut_drain_ms`, `cut_last_frame_ms` | Allowed drain; final frame after cut. |
| `video_recovery_ms`, `recovery_frame_age_ms` | Restore-to-fresh-frame time; frame age. |
| `receiver_runs`, `observations_dropped` | Pipeline starts; observations exceeding 250 000 cap. |

### MAVLink check

`just lab-check mavlink geo` requests `AUTOPILOT_VERSION`, attempts arm without a lease, then cuts forwarding. Options: `--duration-s` (30), `--gcs-listen`, `--vehicle-metrics`, `--ground-metrics`.

| Assertion | Pass condition |
|---|---|
| `telemetry_received` | Heartbeat within 60 s. |
| `request_acked`, `requested_message` | Accepted acknowledgement and requested message arrive. |
| `arm_refused` | Adapter denial counted/received; heartbeats remain disarmed. |
| `sitl_ports_closed` | Overlay answers; TCP 5760, 5762, 5763 refuse connections. |
| `forwarding_cut`, `telemetry_recovery` | Three seconds' silence after drain; recovery within 25 s. |

| Metric | Meaning |
|---|---|
| `heartbeats_per_s`, `heartbeat_gap_max_ms` | Heartbeat cadence and longest gap. |
| `request_ack_rtt_p50_ms`, `request_ack_rtt_max_ms` | Request-to-acknowledgement times. |
| `requested_message_p50_ms`, `requested_message_max_ms`, `requests_unanswered` | Reply times; missing either answer within 5 s. |
| `received_bytes_per_s`, `telemetry_bytes_per_s`, `sitl_bytes_per_s` | Ground receipts; telemetry; SITL read rates. |
| `telemetry_rate_limited`, `telemetry_expired`, `arm_refused` | Budget drops; expired replies; adapter denials. |
| `cut_drain_ms`, `cut_last_message_ms`, `telemetry_recovery_ms` | Drain; final message; restore-to-message time. |
| `messages_dropped`, `undecodable` | Messages exceeding 500 000 cap; undecodable CRC-valid frames. |

### Control freshness check

`just lab-check control-freshness geo` exercises envelopes/leases, drives SITL, then kills the adapter. `outage-10s` adds blackout recovery.

| Assertion | Pass condition |
|---|---|
| `telemetry_received` | Adapter and direct observer receive heartbeats. |
| `invalid_refused` | Invalid envelopes/releases refused; only valid probes applied. |
| `one_lease` | Second controller denied while adapter holds lease. |
| `discrete_acked`, `resend_not_reapplied` | Commands accepted; retry acknowledged without reapplication. |
| `stop_command_applied`, `setpoints_applied` | Hold accepted amid setpoints; driving exceeds 1 m/s. |
| `refresh_bounded` | Refresh ends within setpoint lifetime plus 20 ms. |
| `stopped_after_kill` | Zero throttle within `throttle_release_deadline_ms`, then Hold. |
| `restarted_without_lease` | Restarted adapter has no lease. |
| `outage_nothing_applied`, `reacquisition_required` | Blackout commands unapplied; reacquisition required. |

| Metric | Meaning |
|---|---|
| `invalid_applied`, `outage_applied` | Applications during invalid-input/blackout phases; required zero. |
| `stop_command_ack_ms`, `reacquired_arm_ack_ms` | Hold and reacquired-arm acknowledgement times. |
| `discrete_ack_p50_ms`, `discrete_ack_max_ms` | End-to-end acknowledgement times. |
| `setpoint_delay_p50_ms`, `setpoint_delay_max_ms`, `setpoint_apply_max_ms` | Send-to-apply delay; vehicle processing maximum. |
| `top_speed_m_s`, `refresh_last_ms` | Peak speed; final refresh after last setpoint. |
| `throttle_released_ms`, `hold_ms`, `speed_zero_ms` | Release, Hold and standstill after last setpoint; standstill ungraded. |

### Real-mode check

`just lab-check real-mode lan` switches the default testbed among `vehicle-real.toml`, `vehicle-real-nokey.toml`, `vehicle-real-noclock.toml` and `vehicle.toml`. It uses test-only `sitl_in_real_mode` and hooks. See [real-mode.md](real-mode.md).

| Assertion | Pass condition |
|---|---|
| `real_mode_up`, `interlocks_hold` | Mode `real`; no interlock failures. |
| `control_reaches_the_autopilot`, `stale_bound_keeps_control` | Commands apply, including after clock-bound removal. |
| `missing_key_refuses`, `wrong_key_refuses`, `forged_envelope_refuses`, `other_vehicle_refuses` | Invalid signatures/identities refused without applications. |
| `unsigned_release_refuses`, `wrong_key_release_refuses` | Invalid releases leave lease held. |
| `clock_bound_reaches_the_ground` | Injected 300 ms bound produces 311 ms combined bound. |
| `missing_bound_reads_age_unknown` | Missing bound gives `age_unknown`; restoration gives `live`. |
| `wrong_failsafe_param_refuses`, `sitl_refused` | Broken parameter/identity interlock denies arm. |
| `missing_key_at_the_vehicle_refuses`, `no_bound_at_start_refuses` | Startup without keys or clock bound denies arm. |
| `testbed_restored` | Vehicle returns to `vehicle.toml`. |

### QGC status check

`just lab-check qgc-status geo` receives freshness output at 192.168.232.40:5602, queries the camera and pauses/freezes video. `outage-10s` adds blackout. Options: `--warmup-s` (5), `--duration-s` (30), `--output-listen`, `--gcs-listen`, `--vehicle-metrics`, `--ground-metrics`.

| Assertion | Pass condition |
|---|---|
| `camera_answered` | Camera answers locally; no vehicle-bound requests. |
| `output_received`, `healthy_live` | State-stamped output arrives; healthy window contains Live frames. |
| `pause_indicated`, `freeze_indicated`, `blackout_indicated` | Expected stall states appear; no return to Live/LowRate before release. |
| `pause_recovered`, `freeze_recovered`, `blackout_recovered` | Live/LowRate within 15 s of release. |
| `never_falsely_live` | Live/LowRate ages within configured thresholds plus 100 ms. |

| Metric | Meaning |
|---|---|
| `output_frames_per_s`, `output_new_frames_per_s`, `live_ratio`, `state_unreadable` | Output/fresh cadence, Live share, unreadable states. |
| `output_age_p50_ms`, `output_age_p95_ms`, `output_age_max_ms`, `output_age_mean_ms` | New-frame ages at checks. |
| `decode_age_mean_ms`, `output_added_ms` | Adapter decode age; output mean minus decode mean. |
| `pause_stale_ms`, `pause_disconnected_ms`, `freeze_frozen_ms`, `blackout_stale_ms`, `blackout_disconnected_ms` | Stall-to-state times. |
| `pause_recovery_ms`, `freeze_recovery_ms`, `blackout_recovery_ms` | Release-to-Live/LowRate times. |
| `pause_live_ms`, `freeze_live_ms`, `blackout_live_ms` | Release-to-Live times. |
| `pause_gap_max_ms`, `freeze_gap_max_ms`, `blackout_gap_max_ms` | Longest output gaps during stalls. |
| `live_age_max_ms`, `observations_dropped` | Oldest Live frame; observation-cap drops. |

### Adaptation check

`just lab-check adaptation bandwidth-steps` measures rate adaptation and a second viewer. Without a timeline: `just lab-check adaptation lte-bad --duration-s 120`. Options: `--warmup-s`, `--duration-s` (30), `--url`, `--vehicle-metrics`.

| Assertion | Pass condition |
|---|---|
| `second_viewer` | Two-viewer rate measured at unchanged rung with received video. |
| `video_received` | New frames during observation. |
| `timeline` | Records failure to start the profile timeline. |
| `step_<n>_offered_below_cap` | Offered rate settles below downward step's cap. |
| `step_<n>_age_recovered` | Frame age and inter-frame gaps return within 150 ms of their final phase medians. |

| Metric | Meaning |
|---|---|
| `viewer_one_bytes_per_s`, `viewer_two_bytes_per_s`, `viewer_again_bytes_per_s`, `viewer_change_percent` | Paired viewer rates and percentage difference. |
| `base_age_ms`, `frame_age_p50_ms`, `frame_age_p95_ms`, `frame_age_max_ms`, `new_frames_per_s` | Baseline age; observation ages/cadence. |
| `settle_max_ms`, `over_cap_windows` | Worst settling time; 2 s over-cap windows after 6 s deadline. |
| `age_over_base_max_ms`, `age_recovery_max_ms` | Final age excess; longest age/cadence recovery. |
| `transitions`, `transitions_per_min`, `drains`, `drain_max_ms` | Rung changes and drains begun during observation. |
| `time_normal_s`, `time_constrained_s`, `time_awareness_only_s` | Time in rung groups. |
| `apply_max_ms` | Longest request-to-first-frame rung application. |

### Ingest check

`just ingest-check <profile>` defaults to `lan`. It starts `vehicle-ingest.toml` and feeder, runs ingest, skeleton and isolation checks, then stops the feeder. Warmup and measurement windows are 10 s each.

| Assertion / metric | Meaning |
|---|---|
| `feeder_decoded` | At least 80 % of feeder's 30 fps decoded. |
| `foreign_address_refused` | Datagrams from 192.168.234.6 counted as refused. |
| `decode_cost` | Lifetime mean decode time at most 20 ms. |
| `ingest_decode_ms_mean`, `ingest_decode_ms_max` | Mean/maximum decode times. |

`just ingest-isolation` checks exact `127.0.0.1:5700/udp` publication and counter increases on each `ingest.nft` input, output and forwarding drop rule.

### Regional check

Start `VEHICLE_CONFIG=vehicle-regional.toml just lab-up`; run `just lab-check regional geo --warmup-s 8 --duration-s 20`. Use matching `PI` settings; `--rung <n>` pins comparison. `vehicle-regional-detector.toml` adds detection. Compare tracked, untracked, inset and lower-rung pictures against a test card.

| Assertion | Pass condition |
|---|---|
| `tracking_status_requested`, `tracking_active`, `tracking_stopped` | Requested, active and stopped tracking states reported. |
| `card_written`, `card_rendered`, `inset_placed` | Card file, reference image and inset placement available; otherwise incomplete. |
| `output_received`, `lease_taken`, `camera_heard` | Output, lease and camera available; otherwise incomplete. |
| `rung_pinned`, `lower_rung` | Requested comparison rung applied; otherwise incomplete. |
| `frames_with`, `frames_without`, `frames_lower`, `frames_detail` | At least ten scored frames per window. |
| `region_ssim_gain` | Positive gain; detector-on assertion incomplete. |
| `outside_ssim_vs_lower_rung`, `test_object_contrast_ratio` | Outside SSIM/contrast match or exceed lower rung. |
| `detail_shown`, `detail_stopped`, `inset_drawn_throughout` | Inset within 90 s, tracking stops, inset in at least 90 % of frames. |
| `detail_ssim_gain` | Inset SSIM gain at least 0.05. |
| `frame_age_within_skeleton_budget`, `detail_frame_age_within_skeleton_budget` | Age p95 within skeleton budget plus 100 ms. |

| Metric | Meaning |
|---|---|
| `region_ssim_with`, `region_ssim_without`, `region_ssim_gain` | Tracked/untracked SSIM and difference. |
| `outside_ssim_vs_lower_rung`, `test_object_contrast_ratio` | Outside SSIM difference and relative contrast. |
| `detail_ssim_inset`, `detail_ssim_live`, `detail_ssim_gain`, `inset_share` | Inset/live SSIM, difference and displayed share. |
| `frame_age_p95_ms`, `detail_frame_age_p95_ms` | Tracked/inset-window ages. |
| `vehicle_cpu_percent`, `vehicle_rss_mb`, `encode_ms_p95`, `detector_ms_p95` | Host-core CPU share, memory, encode/detection times. |

### Quality check

Start `VEHICLE_CONFIG=vehicle-quality.toml just lab-up`; run `just lab-check quality geo --warmup-s 8 --duration-s 30`. Use matching `PI` settings. The 62 MB scrolling clip lives in the snapshot volume. Match frames by luma; score at 1280x720 and native size, excluding stamp rows. Closest-frame matching can overestimate damaged-frame quality.

| Assertion / metric | Meaning |
|---|---|
| `clip_written`, `clip_rendered`, `video_received` | Clip, reference and decoded frames available. |
| `matching_forward` | At most 2 % backward matches between frames scoring at least 0.8. |
| `ssim_p50`, `ssim_p5`, `ssim_below_floor_share` | Full-size SSIM percentiles and share below 0.8. |
| `ssim_native_p50`, `ssim_native_p5` | Native-size SSIM percentiles. |
| `frames_scored`, `frames_missing`, `frames_missing_ratio` | Scored/missing counts and missing share. |
| `frame_age_p50_ms`, `frame_age_p95_ms` | Receiver ages. |
| `match_backward_steps`, `score_ms_p95` | Backward matches; match-and-score duration. |

### Recording check

`just lab-check recording lan` fills quota, restarts `companion` and delays writes by 3 s. Options: `--warmup-s`, `--url`, `--vehicle-metrics`, `--snapshot-dir` (`/snapshots`). Limits: `[store]` in `testbed/config/vehicle.toml`.

| Assertion | Pass condition |
|---|---|
| `recording_started`, `quota_held` | Three snapshots within 30 s; quota respected after eviction. |
| `video_received` | Video initialization succeeds; otherwise incomplete. |
| `snapshot_cadence` | Median interval within 10 % of configured second. |
| `restart_recovery` | Partial file removed, unrelated files retained, snapshots indexed, writing resumes. |
| `gap_reported` | Directory gaps agree with store count. |
| `slow_storage_drops`, `slow_storage_recovers` | Slow writes drop snapshots; normal writes resume. |
| `live_unaffected` | Live frames continue; budget grades age/cadence. |

| Metric | Meaning |
|---|---|
| `store_over_quota_bytes`, `snapshot_interval_median_ms`, `restart_resume_ms` | Quota excess; capture interval; restart-to-snapshot time. |
| `store_bytes_max`, `store_count` | Maximum stored bytes; retained snapshot count. |
| `slow_dropped`, `slow_encoded_per_s`, `slow_new_frames_per_s` | Drops and frame rates during slow writes. |
| `healthy_frame_age_p95_ms`, `slow_frame_age_p95_ms`, `slow_age_p95_delta_ms` | Healthy/slow-write ages and difference. |

### Retrieval check

`just lab-check retrieval geo` compares crops/snapshots with retained files, cancels transfers and measures live-video impact. Also accepts `bandwidth-steps`, `outage-10s`. Options: `--warmup-s`, `--duration-s`, `--url`, `--vehicle-metrics`, `--relay-metrics`, `--snapshot-dir`, `--ground-api`, `--vehicle-api`.

| Assertion | Pass condition |
|---|---|
| `crop_matches_retained`, `snapshot_matches_retained` | Crop identity/dimensions and snapshot bytes match source. |
| `video_received`, `history` | Video initialized; at least twelve snapshots retained; otherwise incomplete. |
| `hostile_requests_refused` | Invalid requests/excess jobs refused and counted. |
| `cancel_stops_transfer` | Cancellation noticed within 30 s; no later chunks for 4 s. |
| `live_unaffected` | Paired passes provide age comparisons for budgets. |
| `retrieval_within_allowance` | Wire bytes fit allowance plus initial bucket second. |
| `retrieval_yields` | Timeline causes holds; no chunks between consecutive held readings. |

| Metric | Meaning |
|---|---|
| `crop_completion_s`, `snapshot_completion_s`, `crop_bytes`, `snapshot_bytes` | Completion times and sizes. |
| `hostile_requests`, `hostile_accepted`, `cancel_stop_ms` | Invalid requests, acceptances, cancellation delay. |
| `age_p50_without_ms`, `age_p95_without_ms`, `age_p50_with_ms`, `age_p95_with_ms` | Ages without/with retrieval. |
| `age_p50_delta_ms`, `age_p95_delta_ms` | Age differences. |
| `retrieval_wire_bytes_per_s`, `retrieval_over_allowance_bytes` | Wire rate and allowance excess. |
| `retrieval_jobs_done`, `retrieval_chunks`, `job_completion_p50_s` | Completed jobs/chunks; median completion. |
| `retrieval_holds`, `retrieval_held_s`, `retrieval_sent_while_held` | Holds, duration and chunks during holds. |
| `offered_uplink_delta_bytes_per_s` | Offered-rate difference; ungraded. |

Allowance: `[retrieval] allowance_bytes_per_s` in `testbed/limits.toml`.

### Simulated-flight check

`just sim-check <profile>` runs `sim-flight` and QGC status against the [simulation](drone.md#simulated-flight). It takes off to 10 m, flies an 8 s Loiter leg, stops setpoints, then kills the adapter.

| Assertion | Pass condition |
|---|---|
| `telemetry_received`, `armed`, `takeoff` | Telemetry, accepted arm and climb available. |
| `video_received` | State-stamped output arrives within 60 s. |
| `leg_flown` | Movement above 1 m/s within 3 m altitude band. |
| `brake_on_lost_setpoints`, `no_drop_after_brake` | Brake holds; Loiter without setpoints refused. |
| `land_on_lost_ground` | Adapter loss leads to Land, touchdown and disarm. |
| `video_live` | Live share meets budget. |

| Metric | Meaning |
|---|---|
| `brake_ms`, `brake_sag_m` | Last-setpoint-to-Brake time and altitude loss. |
| `takeoff_s`, `leg_top_speed_m_s`, `leg_altitude_deviation_m` | Climb time, peak leg speed and maximum altitude deviation. |
| `land_ms`, `land_sag_m`, `on_ground_ms` | Land time, pre-Land loss, touchdown time. |
| `video_live_ratio` | Live share during leg. |
| `video_age_p50_ms`, `video_age_p95_ms`, `video_new_frames_per_s`, `sim_real_time_factor` | Ungraded video ages/cadence and simulation pace. |

`land_verdict` caps descent at 1.25 times ArduCopter's default `WP_SPD_DN`. Brake preceding stopped setpoints is incomplete.

### Mission check

`just sim-mission <profile> on|off` tests `allow_missions`. Enabled: upload home, takeoff, three waypoints and landing; kill adapter at first waypoint. Disabled: request upload, start and Auto. No performance budget.

| Assertion / metric | Meaning |
|---|---|
| `telemetry_received`, `switch_known`, `armed` | Required telemetry, switch reading and arm. |
| `upload_refused`, `start_and_auto_refused` | Disabled operations denied and counted. |
| `mission_uploaded`, `mission_started`, `first_waypoint_reached` | Upload, Auto start and progress. |
| `link_cut`, `mission_finished_out_of_reach` | Adapter stopped; remaining mission completes in Auto, landing/disarming. |
| `upload_ms`, `upload_mission_counts`, `upload_items_sent` | Upload duration and counts. |
| `first_waypoint_s`, `finish_s`, `waypoints_after_cut` | Progress times and post-cut waypoint count. |

### Follow check

With simulated detection/follow enabled, run `PI=pi4 SIM=1 just lab-check follow lan`. It hovers at 3 m, tracks the right-most person and yaws in 0.5 s turns with 2 s holds.

| Assertion | Pass condition |
|---|---|
| `telemetry_received`, `armed`, `hovering`, `person_picked` | Hover established; selected person's tracking active. |
| `box_stays_on_the_person` | Box within 0.08 width / 0.10 height; person crosses 0.2 width. |
| `lost_when_it_leaves`, `status_goes_idle` | Lost reason reported, target removed, tracking idle. |

The check does not prove the person left the picture.

### Sightings check

With simulated detection enabled, run `SIM=1 just lab-check sightings outage-30s`. The copter stays on the runway during a 30 s blackout.

| Assertion | Pass condition |
|---|---|
| `profile_cuts_the_link`, `link_cut`, `vehicle_heard`, `detector_sees_people` | Blackout, telemetry and detection prerequisites available. |
| `none_during_the_cut`, `sightings_arrive` | Sightings arrive only after restoration. |
| `positions_agree`, `seen_in_the_cut` | Positions agree; first-seen times inside cut within 3 s. |
| `listing_received`, `listing_matches_told`, `listed_as_told` | Listing matches reported sightings and marks them told. |
| `snapshot_named`, `snapshot_fetched` | Snapshots within 1.5 s; first fetch matches. |

### Landing check

`just sim-landing <on|off|forward> <profile>` hovers 2.3 m from the tag, observes for 4 s, then requests Land.

| Assertion / metric | Meaning |
|---|---|
| `telemetry_received`, `armed`, `takeoff`, `hover_reached` | Flight setup completed. |
| `target_sent_as_expected` | Targets counted when enabled/camera down; otherwise none, with notice when enabled. |
| `aim_matches_geometry` | Fresh target angles within 3 degrees of geometry. |
| `landed_on_tag`, `landed_away_from_tag` | Enabled/down lands within 1 m; other cases beyond 1 m. |
| `camera_off_nadir_deg`, `camera_turn_deg`, `target_messages_in_4s` | Camera orientation and target count. |
| `aim_error_deg`, `aim_turn_deg` | Target-angle error and turn. |
| `landing_error_m`, `final_altitude_m`, `land_s`, `descent_begins_s` | Final position, landing time and descent onset. |
| `tag_distance_max_after_10s_m` | Largest tag distance after settling. |

## Result files

Each run writes `results/<unix-ms>-<scenario>-<profile>/`.

| File | Contents |
|---|---|
| `summary.json` | `run_id`, `scenario`, `profile`, `verdict`, `git_revision`, `assertions`, `metrics`, `sample_counts`, `thresholds`, `effective_profile`, scenario `settings` and `readings`. |
| `timeseries.csv` | `unix_s,series,seq,value`: timestamped probes, frames or scenario observations. |

Settings record requested configuration; readings preserve counters/health. Thresholds are the budget used or `null`. Image IDs are not recorded.

## Verdicts and budgets

Any failed assertion fails the run. Otherwise, missing assertions or incomplete evidence make it incomplete. Only all-pass assertions pass. Exit codes: 0 pass, 1 fail, 2 incomplete. Profile/budget read failures and result-write failures are incomplete.

`perf/budgets.toml` has scenario/profile tables:

| `status` | Rule |
|---|---|
| `calibration` | No limits; measurements recorded; `budget` incomplete. |
| `enforced` | Nonempty `rationale` and `min`/`max` limits; each creates `budget.<metric>`. |

Missing budgets or required metrics are incomplete. Limits use measured margins, not impairment settings. Checks without performance budgets grade behavioral assertions.

## Directions

Uplink is router egress toward `ground_net`, address `192.168.232.254`. Downlink is egress toward `vehicle_net`, address `192.168.231.254`. RTT includes both delays.

## Profile format

Profiles are TOML in `testbed/profiles/`. Unknown fields fail; validation precedes application.

| Keys | Values / limits |
|---|---|
| `[profile] name`, `description`, `provenance`, `source`, `seed` | Name matches filename; provenance `synthetic` or `measured`; measured requires source; optional seed. |
| `[uplink]`, `[downlink]`: `delay_ms`, `rate_kbit` | Required; 0..=5000 ms, 8..=1_000_000 kbit/s. |
| `jitter_ms`, `jitter_distribution` | Default 0; at most delay; `normal`, `pareto`, `uniform`. |
| `burst_bytes`, `queue_packets`, `ordering` | Defaults 16384, 1000, `preserve`; ranges 1514..=10_000_000, 1..=100_000; also `allow-reorder`. |
| `[uplink.loss]`, `[downlink.loss]`: `model` | `random`: `percent`; `gilbert-elliott`: `p`, `r`, `one_minus_h`, `one_minus_k`; finite percentages 0..=100. |
| `[[timeline]]`: `at_ms`, `action`, `direction`, `rate_kbit` | Ordered offsets; `set-rate`, `blackout`, `restore`; `uplink`, `downlink`, `both`; rate only for `set-rate`. |

## Profiles

| Profile | Uplink | Downlink / timeline |
|---|---|---|
| `lan` | 1 Gbit, zero delay | Same |
| `lte-good` | 40 ms, 5 ms jitter, 5 Mbit | 40 ms, 5 ms jitter, 20 Mbit |
| `lte-bad` | 60 ms, 30 ms pareto jitter, 800 kbit, G-E loss | 60 ms, 30 ms jitter, 3 Mbit, G-E loss |
| `geo` | 300 ms, 10 ms jitter, 512 kbit | 300 ms, 10 ms jitter, 2048 kbit |
| `geo-degraded` | 300 ms, 25 ms jitter, 128 kbit, G-E loss | 300 ms, 10 ms jitter, 1024 kbit, 1 % random loss |
| `outage-30s` | `lan` | Blackout both at 5 s; restore at 35 s |
| `outage-10s` | `geo` | Blackout both at 10 s; restore at 20 s |
| `bandwidth-steps` | `geo` | Uplink 128 at 15 s, 64 at 30 s, 256 at 45 s, 512 kbit/s at 60 s |

## tc translation

Chain: `htb root 1:` → class `1:10` → `netem 10:` → `pfifo 11:` when `ordering = "preserve"`. Rate and ceil equal `rate_kbit`. Live rate changes use `tc class change`; blackout/restore change netem loss. Queued packets drain during blackout. FIFO does not guarantee jitter preserves ordering.

## Router agent

`lab router --listen 192.168.233.254:9000 --uplink-if auto --downlink-if auto` resolves interfaces by router address. It requires `NET_ADMIN`, `tc`, `nft`; handles requests serially; caps bodies at 64 KiB; returns JSON.

| Request | Action |
|---|---|
| `POST /profile` | Validate TOML, rebuild qdiscs, stop timeline; return effective state. |
| `POST /timeline/start`, `GET /timeline` | Reapply baseline/start; read transitions/timing. |
| `GET /counters` | Qdisc counters and interface statistics. |
| `POST /forwarding` `{"enabled": bool}` | Add/remove `inet thinlink_cut`. |
| `POST /reset` | Stop timeline, remove qdiscs and cut table. |

```sh
lab impair --agent http://192.168.233.254:9000 --profile testbed/profiles/geo.toml
lab impair --agent http://192.168.233.254:9000 --profile testbed/profiles/outage-10s.toml --start-timeline
```

## Images

| Source / stage | Contents |
|---|---|
| `testbed/Containerfile`: `build` | Rust/native development dependencies; `localhost/thinlink-build:dev`. |
| `bins-native`, `bins-cross-amd64` | Release binaries; amd64 cross-compiles on Apple Silicon. |
| `runtime-base`, `runtime`, `runtime-amd64` | Debian, GStreamer, network tools, mavlink-router v4, `lab`, `thinlink-vehicle`, `thinlink-relay`, `thinlink-ground`. |
| `testbed/Containerfile.sitl` | Pinned ArduRover 4.7.2 binary/parameters; `localhost/thinlink-sitl:dev`. |

Build context is the repository root; `.dockerignore` excludes local artifacts. Containerfiles hold base digests and download checksums.

## macOS and Podman notes

The VM needs kernel WireGuard, `netem`, `htb` and nftables NAT. Bind mounts cross its file share. SITL uses x86_64 qemu on Apple Silicon. Without `ping_group_range`, ping reports `socket: Operation not permitted`.

`lab-down` removes this project's leaked netavark forwarding drop-ins after bridges disappear. `lab-up` enables forwarding on `ground_net`'s bridge when needed. A stale `/run/sysctl.d/10-netavark-<bridge>.conf` can leave published ports connected without carrying data.

## Docker equivalence

Use `CONTAINER_ENGINE=docker just lab-up`, `CONTAINER_ENGINE=docker just lab-check boot lan`, `CONTAINER_ENGINE=docker just lab-down`; `just engine=docker ...` also works. Recipes use `docker compose -p thinlink -f testbed/compose.yml -f testbed/compose.docker.yml ...`.

The overlay maps `host.containers.internal` through `host-gateway`; `lab-keys` uses `--user $(id -u):$(id -g)`. The Docker path has no end-to-end measurement here. Its kernel needs WireGuard, `netem` and nftables NAT.
