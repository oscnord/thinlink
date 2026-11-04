# AGENTS.md: apps/lab

`thinlink-lab-cli` (L4) builds `lab`. Keep argument parsing and wiring here. Scenario behavior belongs in `thinlink-lab`.

## Code and rules

- `src/main.rs` owns `Command`, `CheckArgs`, `Scenario`, token loading and scenario dispatch.
- `run_check` reads `testbed/profiles/<name>.toml` and writes `results/<run-id>/`, relative to the working directory. Keep scenario names aligned with budget lookup and output labels.
- `check` maps a failure to run or record results to `Verdict::Incomplete`. Preserve exit codes: 0 pass, 1 fail, 2 incomplete.
- Keep router operations behind `thinlink-lab`'s agent. `impair` validates the profile before sending it.
- Host actions belong in the justfile. Preserve the runner's stdout markers used by `just lab-check`.

## Checks

This app links GStreamer through `thinlink-lab`; use the build container. Run `just check` and `just gate-fast <scenario> <profile>` for changed dispatch. Commands and scenario options are in `docs/testbed.md`.
