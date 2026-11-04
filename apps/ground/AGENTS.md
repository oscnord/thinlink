# AGENTS.md: apps/ground

`thinlink-ground` (L4) wires the ground adapter and its CLI. Keep MAVLink behavior in `thinlink-control`, video output in `thinlink-video`, and drawing in `thinlink-media`.

## Code map

| File | Main pieces |
|---|---|
| `src/main.rs` | `Config`, `Limits`, `Tokens`, `run`, lease routes and CLI requests |
| `src/detail.rs` | `Detail`, retained-object requests, fetching, outage snapshots and sightings |
| `src/box_detail.rs` | `keep`, tracked-box crops and `DetailCell` updates |

## Rules

- Keep the adapter token and relay job token separate. `--token-file` belongs to the adapter; `--job-token-file` belongs to the relay.
- Serve lease routes through `thinlink_video::metrics::serve_with`. A metrics bind failure exits the process.
- A lease grant without its first challenge is not success. Preserve `TakeError::NoChallenge` as HTTP 504 in `take`.
- Startup does not acquire a lease. Keep lease acquisition explicit.
- Resolve station and output destinations before starting workers. Report worker failure through its health state.
- Keep one crop job in flight. Free its slot after fetching, including fetch failure. Discard a result if the tracked box changed during the request.
- Use `Camera::target` for the reported target position, with the drawn box as fallback. Keep crop widening in `thinlink_policy::inset`.
- Label fetched detail with its source identity and capture time. `--ago-s` depends on this machine's clock; see `docs/video.md#frame-age-and-clocks`.

## Checks and references

Use the build container for this GStreamer-linked app. Run `just check` and the affected `qgc-status` or `retrieval` lab check under the root verification rules. Config tests live in `src/main.rs`.

Config: `testbed/config/ground.toml`. Limits: `testbed/limits.toml`. Commands and display behavior: `docs/testbed.md` and `docs/drone.md#qgroundcontrol`.
