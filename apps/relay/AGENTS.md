# AGENTS.md: apps/relay

`thinlink-relay` (L4) wires ingress and feedback from `thinlink-video` with the job API from `thinlink-detail`. Keep their behavior in those crates.

## Code and rules

- `src/main.rs` owns `Config`, `Metrics`, `load` and worker startup. Config comes from `testbed/config/relay.toml`.
- Run ingress and feedback under `thinlink_video::supervise`. Overlay binds may fail until `wg0` is available; preserve retry behavior and health reporting.
- Keep ingress and feedback on their configured overlay addresses. The job API uses separate vehicle and ground listeners, each with its own routes.
- Start the job API only with `--token-file`. Pass that token to the API and metrics service. Retry a stopped API using the same shared `Relay`.
- A metrics bind failure exits the process. Reject unknown config keys.

## Checks

Use the build container for this GStreamer-linked app. Run `just check` and the affected `skeleton`, `adaptation` or `retrieval` lab check under the root verification rules. Config tests live in `src/main.rs`.

See `docs/testbed.md` and `docs/drone.md#retained-detail`.
