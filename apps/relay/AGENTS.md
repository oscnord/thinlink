# AGENTS.md: apps/relay

`sat-teleop-relay` (L4) is the relay's entry point. It stays thin: config loading and wiring over `sat-teleop-video`.

- `sat-teleop-relay run --config <toml> --metrics-addr <addr>` runs the ingress and the feedback endpoint under `sat_teleop_video::supervise` on threads and serves `GET /metrics` on the main thread. If the metrics endpoint cannot bind, the process exits.
- Config: `testbed/config/relay.toml` (`listen` on the overlay address, `publish` RTSP URL of the local MediaMTX, and `[feedback]`: `listen`, the overlay address and port probes arrive on, and `vehicle`, the only address whose probes are answered; and `[detail]`: `vehicle`, the overlay address the vehicle polls and uploads on, and `ground`, the `ground_net` address the ground asks and fetches on); unknown keys are rejected. In the testbed, metrics listen on `192.168.233.20:9100`.
- The ingress binds `udpsrc` to the overlay address only, so RTP cannot arrive outside WireGuard, and so does the feedback endpoint. Until `wg0` is up the binds fail and each restarts every second.
- The job API (`sat_teleop_detail::relay`, `docs/drone.md#retained-detail`) binds exactly those two addresses, never every address, on threads restarted every second until they bind (the overlay address is not up at start). Every request, and `GET /metrics`, needs the bearer token of `--token-file` (without one the metrics endpoint listens on 127.0.0.1 only and the job API does not start); the vehicle's listener answers only the vehicle's routes and the ground's only the ground's.
- Metrics JSON: `rtp_packets`, `rtp_bytes` (received, before the jitter buffer), jitter buffer `pushed`, `lost`, `late`, `duplicates` (totals across restarts), `avg_jitter_us`, pipeline health (`runs`, `running`, `last_error`), and `feedback`: `answered`, `refused` (not a probe, or not from the vehicle), `send_failed` and its health; `detail`: the job API's `requests`, `rejected` (status 400 and up), `created`, `completed`, `failed`, `cancelled`, `chunks`, `chunk_bytes` and the jobs held by state.

Use `anyhow` with `.context()`. Pipelines belong in `sat-teleop-video`. Documented in `docs/testbed.md`.
