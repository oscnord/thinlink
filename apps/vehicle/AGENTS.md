# AGENTS.md: apps/vehicle

`thinlink-vehicle` (L4) wires the sender, adaptation and drone services. Keep pipelines in `thinlink-video`, drone media in `thinlink-media`, and MAVLink in `thinlink-control`.

## Code map

| File | Main pieces |
|---|---|
| `src/main.rs` | `Config`, `Limits`, `Retained`, `run`, config loading and worker wiring |
| `src/landing.rs` | `CameraConfig`, `LandingConfig`, `resolve`, `find_tags`, `TagStats` |
| `src/hooks.rs` | `Hooks`, compiled with `testbed` |
| `src/process.rs` | `ProcessSnapshot`, process readings and parsing |

## Rules

- Validate config, limits and ladder before starting workers. Reject unknown keys. `--start-rung` counts from the best rung.
- Use `thinlink_video::supervise` for the sender and adaptation loop. Report bridge failure through its health state. A metrics bind failure exits the process.
- Keep `--token-file` local to metrics and hooks. Use `--job-token-file` for the relay's job API.
- Keep snapshot writes on the bounded writer queue. A full queue drops the snapshot; it must not block capture. `[snapshot]` requires `[store]`.
- Share `bridge.control.region` with the sender and detector. Do not create another `RegionCell` for their selections.
- Load the model before starting the sender. A load failure leaves out the detector branch and reports failure without stopping video. Detector and tracker workers consume their own frame feeds.
- Use the same `heard` decision for sightings, bridge reporting and snapshot tagging. `note_sightings` requires a fresh position.
- Route tag observations through `VehicleStats::landing`. The detector thread never writes directly to the autopilot.
- Keep telemetry and retrieval reservations in adaptation's shared uplink budget. Retrieval uses `thinlink_media::retrieval::conditions`; do not duplicate its admission logic here.
- Keep `debug_hooks` and `sitl_in_real_mode` behind the `testbed` feature. A build without it rejects either config key. Never enable that feature for release artifacts.

## Checks and references

Use the build container for this GStreamer-linked app. Run `just check` and the lab scenario that covers the changed wiring under the root verification rules. Config tests live in `src/main.rs`; process parsing tests live in `src/process.rs`. `thinlink-vehicle probe-camera` inspects sources on the target machine.

| Reference | Content |
|---|---|
| `testbed/config/vehicle.toml` | Vehicle config |
| `testbed/limits.toml`, `testbed/ladder.toml` | Limits and encoding ladder |
| `docs/testbed.md` | Commands, hooks and metrics |
| `docs/hardware.md#boards`, `deploy/vehicle/vehicle.toml` | Board configuration |
| `crates/control/AGENTS.md`, `crates/media/AGENTS.md` | Control and media boundaries |
