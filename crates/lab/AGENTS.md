# AGENTS.md: crates/lab

`thinlink-lab` (L3) owns impairment profiles, router operations and scenario measurements. It grades results against `perf/budgets.toml`. Scenario procedures are in `docs/testbed.md#checks`; simulation setup is in `docs/drone.md#simulated-flight`.

## Code map

Paths are relative to `src/`.

| Files | Main pieces |
|---|---|
| `profile.rs`, `interface.rs` | `Profile`, input validation and interface resolution |
| `tc.rs`, `nft.rs`, `ping.rs`, `command.rs` | Command construction, parsing and execution |
| `agent.rs`, `timeline.rs`, `client.rs` | `Agent`, `TimelineRun`, `AgentClient` |
| `check.rs`, `budget.rs` | `Results`, `Verdict`, `Budgets`, grading and result files |
| `skeleton.rs`, `adaptation.rs` | `Capture`, frame analysis and ladder settling |
| `mavlink/gcs.rs`, `control/` | Shared station socket and control scenarios |
| `regional/`, `quality.rs` | Reference cards, SSIM and clip matching |

Other scenario entry points are named for their checks in `src/`. `qgc_status.rs` implements `qgc-status`; `control.rs` implements `control-freshness`.

## Rules

- Validate profiles before touching the router. Keep `tc`, `nft` and `ping` command construction pure and table-tested. Execute processes through `command.rs`.
- Shape only the two resolved interfaces. Reject invalid interface names before constructing argv.
- Live changes use `tc class change` or `tc qdisc change`. Only setup and teardown delete qdiscs.
- Stop and join `TimelineRun` before another operation runs `tc`. Use monotonic time for scheduled steps.
- Keep HTTP bodies capped at 64 KiB and requests serial.
- A failure makes the run fail. Missing observations, missing thresholds or no assertions make it `incomplete`. Never convert absence of evidence into a pass.
- Silence assertions require proof that the target answered before the cut and a full stream of unanswered probes. Retry individual probes until the deadline; do not rely on a single ping or iputils `-w`.
- Restore forwarding and release video stalls on error paths. Restore each scenario's modified state before returning. Confirm host actions from observed behavior; stdout markers alone are not evidence.
- Count new source frames for age and cadence. Keep observation storage bounded. Use `Capture` and `thinlink-video`'s receiver; never depend directly on GStreamer.
- Keep verdict calculations pure and test their evidence requirements. Invalidate comparisons when a rung change disrupts their measurement window.
- Derive enforced limits from measured runs. Calibration is ungraded. Never widen a budget or lifetime to make a run pass.
- Read MAVLink through `thinlink_control::frame` and write it with `mavlink::write_v2_msg`. Keep crafted envelope cases in `control/intruder.rs` using `thinlink_control::envelope`.

## Checks

This crate builds in the build container. Run `just check` once per change set and `just gate-fast <scenario> <profile>` for affected behavior. See `docs/testbed.md#builds-and-gates` for gate selection and `docs/testbed.md` for scenario requirements.

Keep pure tests beside their verdict and parser functions. Inspect `summary.json` and `timeseries.csv` for measurements and effective settings. Runtime scenarios need the compose testbed's `NET_ADMIN` and Linux `tc`/`nft`.
