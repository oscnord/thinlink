# AGENTS.md: apps/lab

`sat-teleop-lab-cli` builds the `lab` executable (L3). It stays thin: argument parsing and wiring over `sat-teleop-lab`.

- `lab router --listen <addr> --uplink-if <name|auto> --downlink-if <name|auto>` runs the router agent.
- `lab impair --agent <url> --profile <path> [--start-timeline]` validates a profile locally, then applies it through the agent.
- `lab check <boot|skeleton|mavlink|control-freshness|qgc-status|adaptation> --profile <name> [--agent <url>] [--git-rev <rev>]` reads `testbed/profiles/<name>.toml` and writes `results/<run-id>/`, both relative to the working directory. Exit 0 pass, 1 fail, 2 incomplete; a check that cannot run or record its results exits 2.
- `mavlink` also reads `perf/budgets.toml` and takes `--duration-s` (as above), `--gcs-listen` (default `192.168.232.40:14552`), `--vehicle-metrics` and `--ground-metrics` (default `http://192.168.233.30:9100`).
- `control-freshness` also reads `perf/budgets.toml` and `[control]` of `testbed/limits.toml`, and takes `--gcs-listen`, `--vehicle-metrics`, `--ground-metrics` (as for `mavlink`), `--observer-listen` (default `10.77.0.4:14553`), `--controller` (default `10.77.0.4:0`) and `--vehicle-control` (default `10.77.0.2:7700`). It prints `lab: kill ground` on stdout when the adapter should be killed; `just lab-check` does that.
- `skeleton` also reads `perf/budgets.toml` and takes `--warmup-s` (0-600, default 5), `--duration-s` (1-3600, default 30), `--url` (default `rtsp://192.168.232.20:8554/live`), `--vehicle-metrics` and `--relay-metrics` (metrics base URLs on `ctl_net`).
- `qgc-status` also reads `perf/budgets.toml` and `[freshness]` of `testbed/limits.toml`, and takes `--warmup-s`, `--duration-s` (the healthy window), `--gcs-listen`, `--vehicle-metrics`, `--ground-metrics` (as above) and `--output-listen` (default `192.168.232.40:5602`, where the adapter sends its video output).

- `adaptation` also reads `perf/budgets.toml`, and takes `--warmup-s`, `--duration-s` (the observation under a profile without a timeline), `--url` and `--vehicle-metrics` (as for `skeleton`).

Use `anyhow` with `.context()`. Behaviour belongs in `sat-teleop-lab`. Documented in `docs/testbed.md`.
