# AGENTS.md: sat-teleop

A hobby Rust bench prototype for adaptive video and test-only control over cellular and GEO-like IP links.

QGroundControl is the primary ground station. The vehicle sends one H.264 RTP/UDP uplink inside WireGuard to a relay. Live video, retained recording and historical-detail retrieval have separate bounded work queues and share one network budget.

No milestone in this repository authorizes commands to real actuators. Real mode (`docs/real-vehicle.md`) exists behind interlocks, has never flown and is never the default.

## 1. Setup

Required development tools:

- Rust 1.99.0 through rustup. `rust-toolchain.toml` pins it with rustfmt and Clippy. If `cargo` on your PATH is not a rustup proxy, see `docs/development.md`.
- Podman and a compose provider (`podman compose`), for the container testbed and the build container. `CONTAINER_ENGINE=docker` runs the lab under Docker instead (the Podman machine workarounds are skipped).
- `just`, `jq`, `gh`, Git and `rg`.
- GStreamer is only needed inside the containers. Do not install it on the host to build the workspace.
- Linux `tc`/`iproute2`, WireGuard and packet-capture tools live in the testbed containers, not on the host.
- The native macOS QGC artifact for manual ground-station checks.
- V4L2 inspection tools on the Pi when working on hardware integration.

Verify installed tools. Do not assume a tool exists because a reference repository used it. Record tested versions and artifact identities.

Run full native-dependency and Linux checks in the build container. Pure policy checks must also run without GStreamer, network privileges or hardware.

Use Chrome Canary with an isolated profile for optional browser testing and debugging.

## 2. Orientation

| Read | Purpose |
|---|---|
| `README.md` | Plain-language overview for people: what it does, quickstart, limitations (no implementation detail) |
| Issue [#2](https://github.com/oscnord/sat-teleop/issues/2) | The plan and current status (spec of record) |
| `docs/architecture.md` | Ownership, dependency layers, data/control paths, current state |
| `docs/development.md` | Toolchain, build, gates, xtask, fixtures, adding a package |
| Relevant per-package `AGENTS.md` | Local invariants, dependencies and checks |
| `log/devlog.md` | Gitignored local resume notes (may not exist) |

`docs/pi.md` and `docs/deploy.md` cover the Raspberry Pi vehicle, the relay server, the ground machine and the bench checklist; `deploy/` holds their scripts, units and config templates. `docs/qgc.md` covers connecting QGroundControl, its video and camera discovery. `docs/testbed.md` covers the compose topology, WireGuard overlay, recipes, the live video path, the boot and skeleton checks, profiles and the router agent. `docs/simulation.md` covers the opt-in simulated flight (ArduCopter in Gazebo, `just sim-up`): failsafes, camera path, resources and licences. `docs/measurement.md` defines the video metrics, result files and budget grading; `perf/budgets.toml` holds the budgets.

Read the root and relevant local instructions before editing. The GitHub issue identifies the active work.

## 3. Workspace map

Packages that exist today:

| Layer | Package | Dir | Responsibility |
|---|---|---|---|
| L0 | `sat-teleop-policy` | `crates/policy` | Pure decisions; today the in-picture stamp and the state stamp, the landing target decision, the byte-rate token bucket, the uplink budget, the adaptation ladder and controller, the control lease, video freshness, snapshot retention and when retrieval may send |
| L1 | `sat-teleop-media` | `crates/media` | Sender with live rung changes and its snapshot branch, relay ingress and its feedback, the vehicle's adaptation loop, headless receiver and the ground's freshness output, `GET /metrics` endpoint |
| L1 | `sat-teleop-control` | `crates/control` | MAVLink: the vehicle bridge with its command gate, telemetry budget and control endpoint, the ground bridge with its controller and camera component, the control envelope |
| L1 | `sat-teleop-detect` | `crates/detect` | The detector: loads an ONNX model with `tract-onnx` and returns the people and vehicles (the configured kinds) in a frame as regions in picture fractions, and finds one AprilTag in a luma frame |
| L1 | `sat-teleop-detail` | `crates/detail` | The vehicle's snapshot store, the relay's bounded job API and the vehicle's lower-priority retrieval of a snapshot or a crop |
| L2 | `sat-teleop-lab` | `crates/lab` | Impairment profiles, `tc` translation, router agent, budgets, `lab check` results and the boot, skeleton, mavlink, control-freshness, qgc-status, adaptation, recording and retrieval scenarios |
| L3 | `sat-teleop-vehicle`, `sat-teleop-relay` | `apps/vehicle`, `apps/relay` | Run the sender, its adaptation, the MAVLink bridge, the snapshot store and retrieval, and the relay ingress with its feedback and job API |
| L3 | `sat-teleop-ground` | `apps/ground` | The ground adapter: MAVLink between mavlink-router and the ground stations, their command endpoint, the camera component, the video freshness output, the lease CLI and the detail request and fetch CLI |
| L3 | `sat-teleop-lab-cli` | `apps/lab` | The `lab` executable (`router`, `impair`, `check`) |
| Tooling | `sat-teleop-xtask` | `xtask` | Layers check and pinned fixtures |

`sat-teleop-testkit` (test support, `crates/testkit`) is registered in `xtask/src/layers.rs` and created only when two crates reuse a helper; it is a dev-dependency only.

Do not create speculative modules, placeholder crates or frameworks. Each new package must justify a distinct dependency boundary or pure policy boundary, register in `xtask/src/layers.rs`, inherit workspace dependencies and lints (`[lints] workspace = true`), add its own `AGENTS.md` and add a line to the index in section 14.

`cargo xtask layers` enforces downward dependencies. L1 siblings do not depend on each other. Applications remain thin. Testkit has no workspace dependencies and is never imported by runtime code.

## 4. Golden rules

### Never panic on input

Network packets, MAVLink messages, files, configuration and requests are untrusted.

- Return typed errors for malformed, unsupported, missing or excessive input.
- No production `unwrap()`, input-dependent `expect()`, `panic!`, `unreachable!`, `todo!` or `unimplemented!`. Tests may use them.
- Use checked indexing (`get()`) and checked arithmetic for input-derived offsets, dimensions, counts and lengths.
- Validate finite numeric values and conversions. Guard division and size calculations.
- Cap allocations before allocating, including decoded dimensions and declared message lengths.
- Handle worker/thread failure explicitly. Do not hide a failed worker behind a healthy status.
- A regression test is required for a fixed input-triggered panic or limit violation.

### Regional video

- `RegionCell` (`sat-teleop-policy`) is the one place that says which regions are kept sharp. The sender softens from `regions()` and the control loop confirms `pilot_in_use()` to the ground; never compute either anywhere else.
- A region envelope never reaches the autopilot and never carries a MAVLink frame. Softening happens before the stamp is drawn, never after.
- The detector never blocks the video: its branch is leaky and drops its own frames, and a model that does not load is shown in the metrics, not fatal. It is off unless the vehicle config asks for it. Softening is off too (`soften_factor` in `[region]` of the vehicle config); with it off the pilot's box is neither kept nor confirmed, and detections are still outlined.
- No AGPL model or code (Ultralytics YOLO is excluded); weights and code Apache-2.0, MIT or BSD.

### Bound work and preserve freshness

- Bound queues by bytes/items and oldest-item age where relevant. Use bounded `std::sync::mpsc::sync_channel` queues and synchronous threads; no async runtime unless a task proves it necessary.
- Bound retries, concurrent jobs, outstanding transfer bytes, recording pins and storage use.
- Continuous commands use latest-only state. Reconnect must not replay old commands.
- Drop obsolete raw frames before encoding. Respect compressed-frame dependencies and recover at a keyframe when needed.
- Reliable transport is not permission to accumulate stale work.
- Recording and detail retrieval must not block control or live capture.

### Time and evidence

- Use monotonic time (`std::time::Instant`) for local deadlines, leases, expiry and elapsed durations.
- Pass time and observations into pure policy functions; policy does not read clocks.
- Cross-machine frame age requires a clock mapping and uncertainty.
- Unknown age is explicit. RTP timestamps alone are not capture wall time.
- Count new source observations, not repeated display frames.
- Report requested settings, applied settings and measured results separately.
- Missing measurements or thresholds never count as passing. Lab exit codes: 0 pass, 1 fail, 2 incomplete or ungraded; never report 2 as a pass.

### Safety and security

- Never send commands to real actuators in the testbed, in tests or by default. Control reaches ArduPilot SITL, or a real flight controller only in `mode = "real"`, and the logging sink otherwise, decided in one place, `sat_teleop_control::identity::failures` (`control_target` is its wrapper). In SITL mode only an autopilot that has reported the SITL-only `SIM_WIND_SPD` parameter gets control frames; in real mode only one that has not, with signed envelopes' keys loaded, the required failsafe parameters read back and equal, and a fresh clock bound. Never bypass, default on or configure around these checks. `sitl_in_real_mode` and the `/real` and `/clock/inject` hooks exist only in builds with the `testbed` cargo feature (the testbed's images); a release build refuses a config that names `debug_hooks` or `sitl_in_real_mode`. Never build a release artifact with the feature. Never add a second decision.
- Vehicle-side policy owns the control lease and command validation.
- Every QGC command must traverse the ground adapter and vehicle gate.
- Stale, frozen, disconnected or unverifiably aged video must never appear live.
- Historical detail must remain labelled with source identity, capture time and uncertainty.
- Use existing transport encryption and protocol implementations. Never write a codec, RTP stack, MAVLink serializer or crypto.
- Authenticate peers and authorize each command or retained-object request.
- Never put secrets in logs or URLs. Do not log sensitive payloads by default. Never commit `testbed/secrets/`.
- Use opaque retained-object IDs, never client-selected filesystem paths.
- Publish development ports on host loopback only.

### Endpoint security

Developer Macs may run endpoint protection (here Microsoft Defender for Endpoint) that kills the shell and the whole agent session when a host command line looks like a reverse shell. It has happened once already.

- On the host, never type, write, heredoc, echo or search for reverse-shell idioms: the shell's built-in network pseudo-files (the tcp and udp device paths), read-write redirection of a numbered file descriptor to a socket, netcat or ncat with an execute option, named pipes combined with netcat, interactive shells redirected to sockets, socat exec or system addresses, Python pty spawning, or piping downloaded content into a shell. This includes files you create, such as scripts in `testbed/scripts/`.
- For HTTP use `curl` (in the runtime image) or the `lab` CLI, never shell socket tricks.
- Keep listeners, pings and packet tools inside containers (`podman compose -p sat-teleop -f testbed/compose.yml exec <service> ...`).
- Prefer the file-editing tools over shell heredocs for writing files.

### Unsafe code

Workspace Rust code is under `unsafe_code = "forbid"`. Safe bindings to native dependencies do not imply those dependencies contain no unsafe code.

A future V4L2/FFI requirement must first document the exact named crate/file boundary, why existing safe bindings are insufficient, and the verification plan. Changing this rule requires an explicit reviewed exception. No exception is granted by this file, and no empty FFI crate should be created in anticipation.

Any accepted exception must expose a safe API, isolate unsafe blocks, explain each block with a `SAFETY:` comment and keep all other code under the prohibition.

## 5. Workflow and picking work

1. Read issue #2 and the relevant docs.
2. Plan non-trivial changes before implementation.
3. Choose the next small vertical slice with a measurable outcome.
4. Search for existing helpers before writing another utility or abstraction.
5. Keep the tree building and inspect actual artifacts as the slice develops.
6. If investigation diverges or assumptions fail, stop, reassess and revise the plan.

Keep changes focused. Fix root causes and remove stale casts, branches, comments, documentation and assumptions invalidated by the change. Separate unrelated mechanical cleanup.

Do not add traits with one implementation, generic plugin systems or configuration for values that never vary. Reuse GStreamer, MediaMTX, mavlink-router, WireGuard and ArduPilot SITL.

## 6. Code style

### Imports

Use four groups separated by blank lines:

1. Standard library.
2. External crates.
3. Workspace crates.
4. Local `crate::` and `super::` imports.

Use scoped imports for items genuinely local to one function.

### Errors and types

- Libraries use `thiserror` enums with useful typed errors.
- Binaries use `anyhow` with `.context()` at the application boundary.
- Prefer concrete types over unnecessary trait objects.
- Use newtypes when identical representations have different meanings, such as session IDs, frame IDs, bytes and rates.
- Use named fields for records that leave a function.
- Use enums instead of boolean parameters with unclear meaning.
- Add `Debug`, `Clone` and `Copy` only where appropriate.
- Make ambiguous `.parse()`, `.into()` and long-chain result types explicit.

### Readability

- Prefer small focused functions and early returns.
- Do not nest an `if` inside another `if`; use guards, named decisions or move bookkeeping to its state owner.
- Name and reuse repeated compound conditions.
- Extract non-trivial domain decisions into pure functions.
- Build repeated records or payloads in one named builder.
- Hoist fixed tables and constant arrays.
- Split functions by the decisions they make.
- Keep iterator chains readable; name intermediate steps when necessary.
- Comments explain non-obvious reasons or constraints, not the code's narration.
- Before deleting or gating behavior, check whether it also owns an unrelated responsibility.

## 7. Clippy policy

Every package opts into workspace lints (`[lints] workspace = true`). CI treats warnings as errors.

The workspace uses pedantic linting with deliberate exceptions listed in the root `Cargo.toml`. `clippy.toml` permits `unwrap`, `expect`, `panic` and indexing in tests only. Fix findings first. When suppression is justified, use the narrowest `#[expect(clippy::..., reason = "...")]` rather than a blanket allow.

Do not suppress a panic, unsafe operation or input-bound violation to make a check green.

## 8. Build, test and lint

Gates before reporting work done:

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo xtask layers
```

`just check` runs the full workspace gates in the build container; `just check-native` runs the pure-crate gates on the host. `just fmt` formats. `just image <target>` builds `localhost/sat-teleop-runtime:dev` or `localhost/sat-teleop-build:dev`. During iteration, run focused package tests first.

Two tiers of gate, so a fix iteration takes minutes:

- Per commit: host-native `cargo fmt`, `cargo clippy` and `cargo test` on the crates the commit touches that do not link GStreamer (seconds). `just check` does not run before every commit.
- Per fix: `just gate-fast <scenario> <profile>` rebuilds what changed, brings the testbed up only if the images or build inputs changed, and runs the one check that failed or covers the change.
- Per change set, once, before review or merge: `just check`, plus the lab checks that exercise what changed (`just gate-fast`, or runs already made on the same build). Run `just check` once before reporting a change set or fix round, not per commit.

Keep verification proportional, then move on:

- `just gate-full` (about 13 minutes) is not a default step. Run it only when a change touches shared behaviour that no targeted check covers, or to refresh budgets. A fix round runs only the checks that cover the code it touched, once.
- Verification must not dwarf the work: a half-hour change does not get hours of gates.
- Do not rerun a passing check for confidence, and do not ask for "N runs" or "N in a row". The exception is work on a flaky check: there, add a pure test that fails without the fix, then run the check about three times.
- Budget derivation follows the house rule's run count in `perf/budgets.toml`; that is measurement, not a gate.

Never run a container build while the testbed is up: the Podman machine's 2 GiB can run out of memory. `just lab-up` and `just gate-fast` take it down first.

GStreamer-linked crates build only in the build container. Pure crates also build natively.

Keep features small and compatible. If a real platform constraint requires a feature matrix, document and run it explicitly instead of silently omitting a target.

Tests must protect product behavior, domain invariants or realistic regressions, and must fail without the code under test. Prefer public inputs and observable outputs. Use table-driven cases for genuinely parallel scenarios.

Do not add UI tests, tests that restate configuration, tests of mocked return values, or tests of behavior already supplied by a dependency.

## 9. Testbed commands

The justfile owns compose wrappers. The Rust lab runner (`lab`) owns scenarios and impairment changes. Compose project name is `sat-teleop`, file `testbed/compose.yml`.

```sh
just lab-keys                       # generate WireGuard keys into testbed/secrets/ (gitignored)
just lab-up                         # build images, keys, start the topology
just lab-check <scenario> <profile> # e.g. boot lan; lab check in the checks container
just lab-down                       # remove only the sat-teleop containers and networks
```

Scenario assertions belong in `lab check`, not in recipe shell. Use the documented Docker equivalent when applicable. Do not silently substitute another runtime or profile.

`lab impair` applies a validated profile or scheduled transition through the router's local control path. Never shape the host's management connection.

On a shared Podman machine, never stop, remove or prune containers, images, volumes or networks that do not belong to compose project `sat-teleop` or images tagged `localhost/sat-teleop-*`. Never restart or reconfigure the Podman machine.

Fetch fixtures with `cargo xtask fixtures`. SHA-256 is verified before use. Missing required fixtures fail the check.

`lab check` writes `results/<run-id>/summary.json` and `timeseries.csv`. Inspect them for measurements, effective configuration and captures. Calibration without thresholds is ungraded, not passed.

QGC rendering, adapter-failure visibility and optional browser presentation need manual checks. Keep native QGC, headless decoder and optical glass-to-glass evidence separately labelled.

## 10. Before finishing

- Review the complete diff, including every subagent change.
- Run the gates in section 8: `just check` once, and the lab checks that cover the change.
- Run affected lab scenarios and inspect their artifacts.
- Recheck error paths, bounded queues, expiry and reconnect behavior.
- Confirm no input-derived panic path or command bypass was introduced.
- Confirm new code uses existing helpers and avoids unnecessary nesting.
- Update affected docs, CLI examples and package instructions, including the current-state list in `docs/architecture.md`. Docs describe the final state only: no task numbers, plans or roadmaps. README stays plain language for people and AGENTS.md files stay short rules for agents.
- Record budget evidence without silently weakening thresholds.
- State which checks passed, failed, were skipped or require hardware.
- Append a terse local `log/devlog.md` entry (ignored by git): change, commands/results, remaining work.

Never mark an unmeasured hardware or real-link claim complete because a container test passed.

## 11. Commits

Use Conventional Commits:

`type(scope): imperative lowercase subject`

Use `!` before the colon for a breaking change. No trailing period.

- Commits are atomic: one logical change each.
- Commit or push only when authorized in the current session or by an explicit repository standing rule.
- Never commit to `main`; work on a feature branch.
- Never force-push. Never amend or rebase pushed commits. Correct a remote branch with a follow-up commit.
- Never add AI-generated footers or co-author trailers.
- Keep PR descriptions short: problem, final changes, follow-ups and verification.
- Rewrite descriptions when scope changes; describe the final branch state.
- Do not use em dashes anywhere in committed files, commit messages or PR text.

Before committing, run the per-commit gate in section 8 (host-native fmt, clippy and test on the touched non-GStreamer crates); `just check` runs once before reporting a change set or fix round. Do not bypass failing checks to conceal failures.

## 12. Parallel agents

Delegate bounded research or implementation when ownership is clear.

- Give each agent explicit files and acceptance checks.
- Use a separate `CARGO_TARGET_DIR=target/agent-<name>` per agent to avoid Cargo build locks.
- Budget VM memory and disk space before concurrent builds; container builds use `BUILD_JOBS` cargo jobs under a `BUILD_MEMORY` cap (defaults 1 and `1g`, sized for a 2 GiB machine; raise both only on a larger machine).
- Coordinate edits to manifests, lockfiles, the layer registry and shared docs.
- Re-read shared files before editing. Do not overwrite another agent's work.
- Keep manifests valid throughout edits.
- Use isolated compose projects and ports for concurrent lab runs.
- The primary agent personally reviews every code change before accepting, committing or pushing it.
- Report which agents were used and what they found.
- Clean up only owned temporary resources after coordinating with active work, and delete a retired worktree's build output. `just check` and the images build into a target directory per checkout, named after its folder; remove a retired worktree's with `podman run --rm -v sat-teleop-target:/t localhost/sat-teleop-build:dev rm -rf /t/<folder>`.

Subagent output is draft work until reviewed.

## 13. Documentation and tracked state

- GitHub issues: scope, decisions, acceptance, status and discussion. Issue #2 is the plan and status.
- `docs/`: authoritative architecture, setup, testbed and measurement behavior.
- `perf/budgets.toml`: acceptance budgets, enforcement status and evidence references.
- `fixtures/manifest.toml`: fixture provenance, immutable pins and checksums.
- `results/`: ignored raw artifacts.
- `log/devlog.md`: ignored local resume notes, never the sole record of a decision.

Never commit plans, task lists, ledgers, devlogs or implementation notes. Do not add a roadmap file. Never commit `.superpowers/`, `results/`, `log/`, `.cache/` or `testbed/secrets/`.

Update docs in the same change as behavior, commands, defaults, ownership or measurement changes. Keep proposed, implemented, measured and hardware-unverified states distinct. Do not invent readiness percentages.

Do not commit recordings, downloaded clips, secrets or machine-specific state.

### Deployment files

- `deploy/` holds what a real deployment installs: provisioning scripts, the systemd unit, compose files, firewall and config templates for the vehicle (a Pi, native binary), the relay (a VPS) and the ground machine. `just deploy-keys`, `vehicle-release`, `relay-release` and `ground-release` package them into the gitignored `dist/`; `deploy/secrets/` holds keys and is never committed.
- Provisioning scripts download nothing but distribution packages: binaries and images arrive by copy and are verified against `SHA256SUMS` before they are installed. Never pipe downloaded content into a shell, and never write the reverse-shell idioms in "Endpoint security" into a script.
- Everything under `deploy/` and `docs/pi.md` is unverified on hardware. Keep that said wherever a statement depends on a board, a camera, a flight controller or a real link; never claim a hardware result without a measurement.
- A Pi config keeps the logging sink by default. Control reaches a real autopilot only with `mode = "real"` and every interlock in `docs/real-vehicle.md`; never add another path to it.
- amd64 images are cross-built (`rustc` segfaults emulated on Apple Silicon); under the build cap, one architecture at a time.

### Simulated flight

- The simulation (`just sim-up`, `testbed/compose.sim.yml`) is opt-in. The default testbed and every default check stay as they are; `just sim-check <profile>` is its gate and is not in `gate-full`.
- Gazebo peaks near 1 GB and settles near 650 MB in a 2 GiB machine. Never build, run `just check` or start another heavy container while it is up: `just lab-down` first. Keep its `mem_limit`; do not raise it.
- Control reaches SITL only. A new copter command needs a gate table row and a reason in `docs/simulation.md`. Mission upload and Auto are admitted only behind `allow_missions` (`crates/control/AGENTS.md`). Failsafe values live in `testbed/sat-teleop-copter.parm` with their ArduPilot source. The `sim-flight` bounds (Brake and Land times, sag, time to the ground, Live share) are measured and live in `perf/budgets.toml` under `[sim-flight.<profile>]`; only the descent limit after Land begins comes from ArduPilot's parameter defaults (`land_verdict`).
- The camera enters through the existing ingest, never a new source kind.

## 14. Per-package instruction index

- [xtask](xtask/AGENTS.md): layer registry, fixture verification and repeatable checks.
- [sat-teleop-policy](crates/policy/AGENTS.md): pure decisions, the in-picture stamp and the state stamp, the byte-rate token bucket, the uplink budget, the adaptation ladder and controller, the control lease and command freshness, video freshness.
- [sat-teleop-media](crates/media/AGENTS.md): GStreamer pipelines of the live video path (test, file, ingest, V4L2 and libcamera sources, x264 or the hardware encoder) and the ground's freshness output, media feedback and the vehicle's adaptation loop, metrics endpoint.
- [sat-teleop-control](crates/control/AGENTS.md): MAVLink frames, the autopilot link (SITL TCP or serial) and SITL identification, the vehicle bridge with its command gate, telemetry budget and control endpoint, the ground bridge, controller and camera component, the control envelope.
- [sat-teleop-detect](crates/detect/AGENTS.md): the detector, its model and output layout, and how it is fetched.
- [sat-teleop-detail](crates/detail/AGENTS.md): the snapshot store, the job API (relay and vehicle sides) and its client, crop mapping and chunk assembly.
- [sat-teleop-lab](crates/lab/AGENTS.md): impairment profiles, tc translation, router agent, budgets, checks.
- [sat-teleop-vehicle](apps/vehicle/AGENTS.md): the vehicle executable (sender, adaptation, MAVLink bridge, `probe-camera`).
- [sat-teleop-relay](apps/relay/AGENTS.md): the relay ingress and feedback executable.
- [sat-teleop-ground](apps/ground/AGENTS.md): the ground adapter executable (MAVLink bridge, controller, camera, freshness output).
- [sat-teleop-lab-cli](apps/lab/AGENTS.md): the `lab` executable.

Each new package adds its own `AGENTS.md` focused on local invariants and commands, and one line here. Do not copy this root file into it.
