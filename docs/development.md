# Development

Toolchain, builds, gates and the `just` recipes. The plan is [issue #2](https://github.com/oscnord/sat-teleop/issues/2).

## Code layout

| Path | Contents |
|---|---|
| `crates/policy/` | Pure decisions: stamps, freshness, the control lease, the uplink budget and the adaptation ladder |
| `crates/video-core/` | The pure core of the video path: stamp, freshness, pacing, the uplink budget, the adaptation controller |
| `crates/video/` | GStreamer pipelines: sender (test, file, ingest, V4L2 and libcamera sources, x264 or the hardware encoder), relay ingress, receivers and the labelled ground output |
| `crates/media/` | What the drone adds to the video: regional video, detector, tracker and tag feeds, aim ring and detail inset |
| `crates/control/` | MAVLink: vehicle bridge (SITL over TCP or a flight controller over serial), SITL identification and the command gate, telemetry budget, the ground's camera component |
| `crates/detail/` | Retained snapshots: the vehicle's store, the relay's job API, retrieval at low priority |
| `crates/detect/` | The person detector: a small ONNX model run with `tract-onnx` (pure Rust), fetched by `just model` |
| `crates/lab/`, `apps/lab/` | The testbed runner: impairment profiles, the router agent and `lab check` |
| `apps/vehicle/`, `apps/relay/`, `apps/ground/` | `sat-teleop-vehicle`, `sat-teleop-relay` and `sat-teleop-ground` |
| `xtask/` | `cargo xtask layers` and `cargo xtask fixtures` |
| `testbed/` | Containerfiles, compose topology, configs, limits and link profiles |
| `deploy/` | Provisioning scripts, systemd unit, compose files and config templates for a Pi, a relay server and a ground machine |
| `perf/budgets.toml` | What each `lab check` must meet, with the runs it was derived from |

## Toolchain

`rust-toolchain.toml` pins Rust 1.99.0 with rustfmt and Clippy. The workspace uses edition 2024 and resolver 3. Container builds use `docker.io/library/rust:1.99.0-trixie`, with `docker.io/library/debian:trixie-slim` as the runtime base.

The pin only takes effect when `cargo` is a rustup proxy. On some machines `~/.cargo/bin/cargo` is a plain symlink to another toolchain (for example stable 1.95), so the pin is ignored and builds fail on `rust-version = "1.99"`. Check with:

```sh
cargo --version
rustup which cargo
```

Either run through rustup explicitly:

```sh
rustup toolchain install 1.99.0 --component rustfmt --component clippy
rustup run 1.99.0 cargo test --workspace --locked
```

or put the toolchain's `bin` directory first on `PATH`. On an Apple Silicon Mac:

```sh
PATH=$HOME/.rustup/toolchains/1.99.0-aarch64-apple-darwin/bin:$PATH cargo test --workspace --locked
```

Do not rewrite your global toolchain setup just for this project.

## Native and container builds

Pure crates (`sat-teleop-video-core`, `sat-teleop-policy`, `sat-teleop-control`, `sat-teleop-detect`, `sat-teleop-xtask`, and other crates without native dependencies) build and test natively. Crates that link GStreamer (`sat-teleop-video`, `sat-teleop-media`, `sat-teleop-lab`, whose skeleton check runs `sat-teleop-video`'s headless receiver, and the applications) build only in the build container, so the host never needs GStreamer.

Recipes:

| Recipe | Purpose |
|---|---|
| `just check` | Authoritative gate: fmt check, clippy `-D warnings`, tests and `cargo xtask layers` in `localhost/sat-teleop-build:dev` |
| `just check-native` | Needs `jq` and `bash` on the host. Same gates on the host through the toolchain pinned in `rust-toolchain.toml` (works when `cargo` is not a rustup proxy); skips the GStreamer-linked packages |
| `just image <target>` | Build `localhost/sat-teleop-runtime:dev` or `localhost/sat-teleop-build:dev` from `testbed/Containerfile`, with the repository root as context (`.dockerignore` trims it). The runtime image carries the release binaries, built with the `testbed` cargo feature of the vehicle (its test hooks and its SITL allowance in real mode). `image runtime` also tags the build stage (see "Image cache") |
| `just sitl-image` | Build `localhost/sat-teleop-sitl:dev` (ArduRover SITL, linux/amd64) from `testbed/Containerfile.sitl` |
| `just image-for <arch>` | The runtime image for `arm64` or `amd64`, tagged with the architecture; the other architecture runs under emulation. Built without the `testbed` feature: this is the release image |
| `just deploy-keys <endpoint>`, `vehicle-release`, `relay-release`, `ground-release` | WireGuard keys for a deployment and the directories to copy to a Pi, a relay server and the ground machine (`docs/deploy.md`) |
| `just fmt` | Format the workspace (host `cargo fmt`) |
| `just lab-keys`, `lab-up`, `lab-check <scenario> <profile>`, `lab-down` | The compose testbed; see `docs/testbed.md` |
| `just sim-image`, `sim-up`, `sim-check <profile>` | The simulated flight's images, the testbed with an ArduCopter flown in Gazebo, and its checks; see `docs/simulation.md` |

Container builds and `just check` run with `BUILD_JOBS` cargo jobs under a `BUILD_MEMORY` cgroup cap, 1 and `1g` by default so they fit a 2 GiB Podman machine. On a larger machine raise both, for example `BUILD_MEMORY=4g BUILD_JOBS=4 just check`.
| `just demo <profile>`, `demo-stop`, `demo-cameras`, `ingest-check <profile>` | The Mac's camera through the testbed, and the ingest path graded with a test-pattern container; see `docs/demo.md` |
| `just gate-fast <scenario> <profile> [args]` hashes the build inputs (the Cargo files, `crates`, `apps`, `xtask`, `fixtures` and `testbed/Containerfile`) and compares the hash, the running containers' image and the number of running services with what the testbed was last brought up with (`.cache/gate-fast.inputs`). If anything differs it runs `just lab-up`: the testbed goes down, the images rebuild and it comes up again. Otherwise the running testbed is reused untouched. Then it runs `just lab-check`. A build never starts beside a running testbed: the Podman machine has 2 GiB.

`just gate-full` runs `just lab-up` (a fresh testbed on the current images) and then every check a task is reviewed on, all of them even if one fails, and exits 1 if any did not pass: `boot lan`, `skeleton lan`, `skeleton geo`, `mavlink lan`, `control-freshness lan`, `qgc-status lan`, `qgc-status geo`, `qgc-status outage-10s`, `adaptation bandwidth-steps`, `recording lan`, `retrieval geo` and `real-mode lan` (which restarts the vehicle on two other configs and back, `docs/testbed.md`), then `just ingest-isolation` (the ingest network's filter, `docs/testbed.md`). Profiles that exist only for calibration stay out (`retrieval bandwidth-steps` and `retrieval outage-10s` are run by hand). The images are release builds, as budgets need.

## Iteration speed

Measured on 2026-11-01 (Podman machine with 5 CPUs and 2 GiB, `CARGO_BUILD_JOBS=2`, warm caches, testbed down for builds):

| Step | Time |
|---|---|
| `just image runtime`, nothing changed | 3 to 8 s |
| `just image runtime` after touching one file in `apps/vehicle` | 11 s (cargo 3 s) |
| `just image runtime` after touching `crates/policy` (everything rebuilds) | 23 s (cargo 15 s) |
| `just image build` with a cold apt layer (cache mounts empty) | 58 s |
| `just lab-up`, images built | 10 s |
| `just gate-fast skeleton lan` after a source change (build, `lab-up`, check) | 1 min 50 s the first time, 35 s for `boot lan` after a one-line change |
| `just gate-fast qgc-status lan`, nothing changed | 53 s |

The release build's `target/` is a build cache mount, so a code change recompiles only what it touches; a lab build profile with more codegen units or incremental compilation is not needed (a rebuild is far under 2 minutes).

Per check under `lan` (wall time, of which the ladder settle): `boot` 14 s (none), `skeleton` 56 s (16.6 s, now 6 to 7 s), `mavlink` 50 s (none), `control-freshness` 49 s (none), `qgc-status` 56 s (15 s, now 6 s). Most of a check is its measurement: the warm-up, the 30 s window and the cut and recovery. Under `geo` and `outage-10s` the settle after a profile switch or a cut was 15 to 32 s before and is 6 to 21 s now; the 21 s of the first `skeleton geo` after `lan` is the ladder itself moving (drain, base shift) and is not shortened. The series `gate-full` runs took about 9.5 minutes before (the sum of its checks) and 8.5 minutes including the testbed bring-up after.

## Image cache

A runtime rebuild once sat 20+ minutes in the build stage's `apt-get install` with an unchanged `testbed/Containerfile`. The layer is only reachable from untagged release images, and Podman's event log shows 376 images removed in one prune at 14:41 that day. Repeating the usual flows (a no-change build, a source change, `sitl-image` pulling the other platform of the pinned Debian digest, `lab-up`) always hits the cache, so the prune is the likely cause rather than a proven one. Mitigations: `just image runtime` tags the build stage, both bases are pinned by index digest, and the apt installs use cache mounts so a real rebuild takes about a minute. The 2 GiB machine has no swap, so a build beside the testbed can run out of memory.

Podman 6 reads the build context through the machine's file share instead of uploading it, and that read needs free 512 KiB blocks of the machine's memory. Page cache fragments them within minutes of running the testbed, and `COPY . .` then fails with `readdirent: cannot allocate memory` although about 1.1 GiB is available (2026-11-01: no free block of 512 KiB or more, three failures in a row; after dropping clean page cache and compacting, 189 such blocks and the build passed). `just image` and `just sitl-image` therefore check the machine first and drop its clean page cache when fewer than 16 such blocks are free. Nothing is lost; other containers reread their files from disk.

## CI

CI is disabled on GitHub, and `just` runs the lab under Docker instead of Podman when `CONTAINER_ENGINE=docker` is set (the Podman machine workarounds apply only to Podman), so another machine or a later CI can run it. If it is re-enabled, `.github/workflows/ci.yml` runs on pull requests and on pushes to `main` (plus manual dispatch), skips documentation-only changes (`**/*.md`, `docs/**`), cancels superseded runs and stops a job after 15 minutes. Before, a push to `feat/**` with an open pull request ran the `rust` job twice, about 1.5 minutes each (3 minutes a push); now it runs once (1.5 minutes), and a docs-only push costs nothing.

## xtask

`cargo xtask <command>` is an alias for `cargo run -q -p sat-teleop-xtask --` (see `.cargo/config.toml`).

- `cargo xtask layers` loads the workspace with `cargo metadata` and checks it against the registry in `xtask/src/layers.rs`. It reports every violation and exits 1 if there are any: unregistered members, missing `[lints] workspace = true`, upward or sideways dependencies, dependencies on apps or tooling, non-dev use of `sat-teleop-testkit`, forbidden families in `sat-teleop-policy`, GStreamer outside `sat-teleop-video`, `sat-teleop-media` and apps. It checks dependencies as declared, including optional and target-specific ones.
- `cargo xtask fixtures` reads `fixtures/manifest.toml`, downloads each missing fixture into `.cache/fixtures/` and verifies it. `cargo xtask fixtures --check` only verifies files already present and fails on a missing one.

## Fixtures

`fixtures/manifest.toml` lists test media. Every entry requires `name`, `url`, `sha256`, `size_bytes`, `license`, `source` and `notes`; unknown fields are rejected. A download is capped at `size_bytes + 1` bytes, written to a `.part` file, checked against both size and SHA-256, then renamed. Any failure removes the partial file and exits non-zero. Downloaded files stay in `.cache/` and are never committed.

To add a fixture, download it once, compute its SHA-256 and size yourself, record the real source and license wording, then run `cargo xtask fixtures` and `--check`. Required fixtures that are missing or corrupt must fail a check, never skip it.

Today there is one fixture: `foreman_cif.y4m` (352x288, 300 frames, from the Xiph.org derf collection).

## Lint policy

The root `Cargo.toml` sets `unsafe_code = "forbid"` and Clippy pedantic as warnings. Denied: `unwrap_used`, `expect_used`, `panic`, `todo`, `unimplemented`, `unreachable`, `dbg_macro`. Warned: `indexing_slicing`. Allowed pedantic exceptions: `module_name_repetitions`, `must_use_candidate`, `missing_errors_doc`, `missing_panics_doc`, `similar_names`, `too_many_lines`.

`clippy.toml` allows `unwrap`, `expect`, `panic` and indexing inside tests only.

Fix findings first. If suppression is justified, use the narrowest `#[expect(clippy::..., reason = "...")]`. Never suppress a panic, unsafe or input-bound finding to get green. Unsafe code needs a reviewed exception first; see `AGENTS.md`.

## Adding a package

1. Add the package only when the slice that needs it lands, with a distinct dependency or pure-policy boundary.
2. Add it to `members` in the root `Cargo.toml`.
3. Add its name to the registry in `xtask/src/layers.rs` with the right layer. Every planned name is already listed; confirm the layer still fits.
4. Inherit metadata and dependencies from the workspace (`version.workspace = true`, `dep.workspace = true`) and add `[lints] workspace = true`.
5. Add a focused `AGENTS.md` in the package and a line in the index at the end of the root `AGENTS.md`.
6. Update `README.md` and the current-state section of `docs/architecture.md`.
7. Run the gates, including `cargo xtask layers`.

## Commits

Conventional Commits, `type(scope): imperative lowercase subject`, atomic, no AI footers. Plans, task lists, devlogs and implementation notes are never committed; status lives in issue #2. Details are in `AGENTS.md`.
