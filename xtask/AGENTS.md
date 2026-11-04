# AGENTS.md: xtask

`thinlink-xtask` is repository tooling, run as `cargo xtask <command>` through the alias in `.cargo/config.toml`. See the root `AGENTS.md` for global rules.

## Commands

- `cargo xtask layers`: checks the workspace against the layer registry. Exit code 1 lists every violation.
- `cargo xtask fixtures [--check]`: downloads fixtures from `fixtures/manifest.toml` into `.cache/fixtures/` and verifies them. `--check` only verifies files already present.

## Invariants

- `xtask/src/layers.rs` is the architecture map. Every workspace member needs a registry line; an unregistered member fails the check. Adding a package means adding its line in the same change.
- The check covers normal, build and dev dependencies as declared in the manifests (including optional and target-specific ones). Dependencies point downward only, packages of one layer never depend on each other, `thinlink-testkit` is dev-only, `thinlink-policy` may not use `mavlink*`, `tiny_http`, `ureq`, `rusqlite` or `tokio`, only `thinlink-x264`, `thinlink-video`, `thinlink-media` and apps may use `gstreamer*`, and every package opts into workspace lints. Do not loosen a rule to make a change pass; fix the dependency.
- `thinlink-x264` is L0 alongside `thinlink-video-core`. `thinlink-video` depends on both.
- The check logic is the pure `check(&Graph)` function and is table-tested. Keep metadata loading separate from the rules.
- Fixtures are pinned. Each manifest entry requires `url`, `sha256`, `size_bytes`, `license`, `source` and `notes`; unknown fields are rejected. Downloads are capped at `size_bytes + 1`, written to a `.part` file, verified, then renamed, and the `.part` file is removed on any failure. Never skip or relax verification, and never add a fixture without recording its real SHA-256 and provenance.
- xtask does not orchestrate the testbed. Compose wrappers belong in the justfile and scenarios in the Rust lab runner.
- Library-style rules from the root apply: `anyhow` with `.context()`, no `unwrap` outside tests, no new dependency for something a few lines can do.

## Checks

```sh
cargo test -p thinlink-xtask --locked
cargo xtask layers
cargo xtask fixtures --check
```
