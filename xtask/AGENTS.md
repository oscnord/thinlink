# AGENTS.md: xtask

`thinlink-xtask` owns layer checks and fixture verification. `.cargo/config.toml` exposes it as `cargo xtask <command>`.

## Code and rules

- `src/main.rs` dispatches commands. Keep testbed orchestration in the justfile and `thinlink-lab`.
- `src/layers.rs` owns the package registry and pure `check(&Graph)`. Keep metadata loading separate. Every workspace member needs a registry entry and workspace lints.
- Check normal, build and dev dependencies, including optional and target-specific declarations. Do not relax layer rules to admit a dependency. Tools and testkit have no workspace dependencies; testkit is dev-only.
- `thinlink-policy` forbids `mavlink*`, `tiny_http`, `ureq`, `rusqlite` and `tokio`. Only `thinlink-x264`, `thinlink-video`, `thinlink-media` and apps may depend directly on `gstreamer*`.
- `src/fixtures.rs` fetches entries from `fixtures/manifest.toml` into `.cache/fixtures/`. Keep names to plain filenames. Cap downloads at `size_bytes + 1`, verify SHA-256 before renaming the `.part` file, and remove partial files on failure. Never skip verification or invent checksums.

## Checks

Run natively under the root verification rules:

```sh
cargo test -p thinlink-xtask --locked
cargo xtask layers
cargo xtask fixtures --check
```

`cargo xtask fixtures` fetches missing files; `--check` only verifies local files. Layer violations exit 1. Keep layer tests table-driven.
