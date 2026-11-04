# AGENTS.md: thinlink

Rust on GStreamer for low-latency H.264 video over cellular and satellite links. Drone features build on the video core. QGroundControl is the primary ground station.

## Read first

Read the GitHub issue, root instructions and relevant package instructions before editing. Plan non-trivial changes. Search for existing helpers before adding code. If assumptions fail, reassess the plan.

| Reference | Purpose |
|---|---|
| `README.md` | Overview and quickstart for people |
| `docs/video.md` | Video ownership, adaptation and age measurement |
| `docs/drone.md` | Control, regional video and retained detail |
| `docs/real-mode.md` | Real-mode configuration and command admission |
| `docs/hardware.md`, `deploy/` | Hardware setup and installation files |
| `docs/testbed.md#development-setup` | Code layout and development setup |
| `perf/budgets.toml` | Measurement budgets and evidence |
| `fixtures/manifest.toml` | Fixture pins and checksums |
| `log/devlog.md` | Gitignored local resume notes, if present |

## Workspace and per-package instruction index

Directory links open each package's `AGENTS.md`.

| Layer | Package | Directory | Owns |
|---|---|---|---|
| L0 | `thinlink-video-core` | [crates/video-core](crates/video-core/AGENTS.md) | Pure stamps, freshness, pacing and adaptation |
| L0 | `thinlink-x264` | [crates/x264](crates/x264/AGENTS.md) | `thinlinkx264enc` and libx264 wrapper |
| L1 | `thinlink-video` | [crates/video](crates/video/AGENTS.md) | GStreamer video and extension hooks |
| L1 | `thinlink-policy` | [crates/policy](crates/policy/AGENTS.md) | Pure drone and retention decisions |
| L2 | `thinlink-media` | [crates/media](crates/media/AGENTS.md) | Drone video hooks and decoration |
| L2 | `thinlink-control` | [crates/control](crates/control/AGENTS.md) | MAVLink bridges and command admission |
| L2 | `thinlink-detect` | [crates/detect](crates/detect/AGENTS.md) | ONNX detection and AprilTag detection |
| L2 | `thinlink-detail` | [crates/detail](crates/detail/AGENTS.md) | Snapshot store and retrieval jobs |
| L3 | `thinlink-lab` | [crates/lab](crates/lab/AGENTS.md) | Impairments, scenarios and verdicts |
| L4 | `thinlink-vehicle` | [apps/vehicle](apps/vehicle/AGENTS.md) | Vehicle wiring |
| L4 | `thinlink-relay` | [apps/relay](apps/relay/AGENTS.md) | Relay wiring |
| L4 | `thinlink-ground` | [apps/ground](apps/ground/AGENTS.md) | Ground adapter and CLI |
| L4 | `thinlink-lab-cli` | [apps/lab](apps/lab/AGENTS.md) | `lab` CLI |
| Tooling | `thinlink-xtask` | [xtask](xtask/AGENTS.md) | Layer checks and fixtures |

`cargo xtask layers` enforces downward dependencies, including build and dev dependencies. Same-layer dependencies are forbidden. Applications stay thin; no package depends on an app or tool. GStreamer dependencies belong only in `x264`, `video`, `media` and apps.

Video crates know nothing about drones. Drone integration uses the sender's frame hook and taps and the output's decoration. Pure decision functions receive time and observations; they do not read clocks or perform I/O.

`thinlink-testkit` is registered in `xtask/src/layers.rs` but has no package directory. Create it only when two crates share a test helper. It has no workspace dependencies and is a dev-dependency only. Tooling also has no workspace dependencies.

Each new package needs a distinct dependency or pure-policy boundary. Register it in `xtask/src/layers.rs`, inherit workspace dependencies and `[lints] workspace = true`, add local instructions and update this index. Do not add placeholder packages or generic plugin frameworks.

## Coding rules

- Return typed errors for invalid input. Libraries use `thiserror`; binaries add `anyhow::Context` at application boundaries.
- No production `unwrap()`, input-dependent `expect()`, `panic!`, `unreachable!`, `todo!` or `unimplemented!`.
- Check input-derived indexing and arithmetic. Validate finite numbers and conversions. Cap allocations before allocating.
- Bound queues, retries and storage. Use synchronous threads and bounded `std::sync::mpsc::sync_channel` queues. An async runtime needs a demonstrated requirement.
- Keep continuous commands latest-only; reconnect must not replay them. Recording and retrieval must not block live capture or control.
- Use `std::time::Instant` for local deadlines. Cross-machine age needs a clock mapping and uncertainty. Keep unknown age explicit.
- Report worker failure. Never present stale or unverifiably aged video as live.
- Reuse protocol implementations. Never write a codec, RTP stack, MAVLink serializer or cryptography.
- Prefer small functions and early returns. Do not nest `if` statements. Extract non-trivial domain decisions into named pure functions.
- Reuse compound conditions and payload builders. Avoid traits with one implementation. Comments explain constraints, not execution steps.
- Group imports: standard library, external crates, workspace crates, then local imports. Separate groups with blank lines.
- Fix Clippy findings. Use narrow `#[expect(clippy::..., reason = "...")]` only with justification. Never suppress an input-bound violation.

### Unsafe-code boundary

Workspace Rust uses `unsafe_code = "deny"`. Exceptions belong only in `crates/x264/src/encoder.rs` and generated module `sys` in `crates/x264/src/lib.rs`. `crates/x264/build.rs` generates bindings from the system header.

Expose a safe API, isolate unsafe blocks and explain each with a `SAFETY:` comment. Further exceptions require explicit review of the named file boundary and verification plan. `just check` rejects `unsafe_code` mentions in Rust files under `crates`, `apps` and `xtask` outside `crates/x264`.

## Build and verification

Use Rust 1.99.0 through rustup, pinned in `rust-toolchain.toml`. See `docs/testbed.md#toolchain` for PATH handling. Verify installed tools and record versions for measurements.

GStreamer-linked builds and Linux checks run in the build container. Pure crates also run natively without GStreamer or hardware. Do not install GStreamer on the host for workspace builds. Containers use Podman with a compose provider; `CONTAINER_ENGINE=docker` selects Docker. Other tools: `just`, `jq`, `gh`, Git and `rg`.

| Command | Use |
|---|---|
| `just check-native` | Host gates for pure crates |
| `just check` | Full container gates, including `testbed` feature checks |
| `just fmt` | Format workspace |
| `just image <target>` | Build `localhost/thinlink-runtime:dev` or `localhost/thinlink-build:dev` |
| `just gate-fast <scenario> <profile>` | Rebuild changed inputs and run the affected scenario |
| `cargo xtask fixtures` | Fetch and verify pinned fixtures |
| `just lab-keys` | Generate testbed keys |
| `just lab-up` | Build and start the testbed |
| `just lab-check <scenario> <profile>` | Run a scenario |
| `just lab-down` | Remove the thinlink topology |

Workspace gates:

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo xtask layers
```

- During iteration, run focused package tests. Before each commit, run native fmt, Clippy and tests for touched non-GStreamer crates.
- Before reporting a change set, run `just check` once and the affected lab checks. Inspect `results/<run-id>/summary.json` and `timeseries.csv`.
- Reserve `just gate-full` for shared behavior without targeted coverage or budget measurement. Do not repeat passing checks for confidence. For flaky checks, add a pure regression test and run the check about three times.
- Tests protect product behavior and domain invariants through public inputs and observable outputs. Do not test declarative configuration, mocked return values or dependency behavior. Do not add UI tests.
- Add a regression test for each fixed input-triggered panic or limit violation.
- Scenario assertions belong in `lab check`, not recipe shell. Preserve the selected runtime and profile. Missing measurements or thresholds are not passes: exit 0 means pass, 1 fail, 2 incomplete or ungraded.
- Keep requested settings, applied settings and measurements distinct. Label native QGC, headless decoder and optical evidence separately. Hardware claims require hardware measurements.

Never build while the testbed or simulation is up. Run `just lab-down` first. Keep `BUILD_JOBS` and `BUILD_MEMORY` within VM resources; defaults are `1` and `1g`. Build one architecture at a time. Preserve simulation `mem_limit`. Simulation uses `just sim-up` and `just sim-check <profile>`; it is separate from default gates. New copter commands need a row in `docs/drone.md#command-gate`.

## Command security

Never construct reverse-shell syntax in commands, searches or files. Do not use shell network pseudo-files or command-executing network utility modes. Never connect shells to sockets through redirections, named pipes or pseudo-terminals. Never execute downloaded content through a shell pipeline. Use file-editing tools for writes. Use `curl` or `lab` for HTTP.

Keep listeners and packet tools inside containers. Use `podman compose -p thinlink -f testbed/compose.yml exec <service> ...`. Never shape the host management connection. Publish development ports on loopback. Keep secrets out of logs and URLs.

Manage only resources in compose project `thinlink` or images tagged `localhost/thinlink-*`. Never restart or reconfigure the Podman machine.

## Collaboration and delivery

Delegate bounded work with explicit file ownership. Give agents separate `CARGO_TARGET_DIR=target/agent-<name>` directories and coordinate shared files. Use isolated compose projects and ports for concurrent labs. Review every agent diff personally and report their findings. Remove only owned temporary resources; `just forget-checkout <folder>` removes a retired checkout's build output.

Review the full diff before finishing. Report passed, failed and skipped checks. Keep changes focused; separate unrelated mechanical cleanup.

- Commit and push only with session authorization or an explicit standing rule. Never commit to `main`.
- Use `type(scope): imperative lowercase subject` for commits and PR titles. Use `!` for breaking changes; no trailing period.
- Never force-push, amend or rebase pushed commits. Use follow-up commits. Do not add AI footers or co-author trailers.
- PR descriptions stay short: problem, final changes, follow-ups and verification. Rewrite them when scope changes.
- Docs describe current code, without history, design-change narratives or safety claims. Omit legal and regulatory content. Use plain short sentences, tables and short lists. Avoid repetition, slogans, bold lead-ins, rhythmic triples and em dashes. Keep commands, config keys, metric names and paths exact.
- `README.md` is for people. `AGENTS.md` files are contributor rules and pointers. Architecture and behavior belong in `docs/`; update affected documentation with behavior changes.
- Keep plans and acceptance discussion in GitHub issues. Do not commit plans, task lists, ledgers or roadmaps. Local resume notes belong in ignored `log/devlog.md`.
- Never commit `.superpowers/`, `results/`, `log/`, `.cache/`, `testbed/secrets/`, `deploy/secrets/`, recordings or machine-specific state.
