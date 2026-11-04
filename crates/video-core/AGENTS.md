# AGENTS.md: crates/video-core

`thinlink-video-core` (L0) owns pure video decisions. It depends on `std`, `thiserror` and `subtle`. Read the root `AGENTS.md`.

## Code map

| File | Main pieces |
|---|---|
| `src/stamp.rs` | `Stamp`, `StateStamp`, `FrameId`, `CaptureMs`, `PlaneGeometry`, drawing and reading |
| `src/freshness.rs` | `classify`, `cadence_ms`, `next_change_ms`, `LabelDeadline` |
| `src/pace.rs` | `Admission`, `rate`, whole-frame admission |
| `src/rate.rs` | `TokenBucket`, `Priority` |
| `src/budget.rs` | `Budget`, `Usage`, wire-rate accounting |
| `src/adapt.rs` | `Ladder`, `Controller`, `Feedback`, `Decision` |
| `src/token.rs` | `Token`, `admit`, `listen_address`, `exposed` |

| Read | Subject |
|---|---|
| `docs/video.md#frame-age-and-clocks` | Stamp age and clock uncertainty |
| `docs/video.md#freshness-thresholds` | `[freshness]` in `testbed/limits.toml` |
| `docs/video.md#adaptation-thresholds` | `[adapt]` in `testbed/limits.toml` |

## Invariants

- No I/O or clock reads. Callers supply time and observations. Keep GStreamer and drone concepts outside this crate.
- Keep `reserved` an unnamed secondary flow. The budget and controller must not know its consumer.
- Reuse stamp geometry validation for drawing and reading. Reject overflowing dimensions and short planes before indexing.
- A valid CRC does not establish a plausible capture time. Readers must apply `plausible_age`; cross-machine age needs a clock mapping and uncertainty.
- Interpret `StateStamp` only on the ground output. Arbitrary image rows can pass its CRC.
- Freshness distinguishes arriving duplicates from missing frames. Derive cadence from advancing source identities, not display repeats or receive counts.
- Keep `classify` and `next_change_ms` consistent. `LabelDeadline` must reject every packet of a frame whose first packet misses its deadline.
- Pass time into `TokenBucket`. Backward time must not refill it.
- Keep wire-byte rates distinct from encoded bitrate and telemetry frame-byte rates. Budget allocations must not exceed the available total.
- Call `Controller::observe` without feedback as well as with it, so silence affects the decision. Feedback from before a rung change must not trigger another change before that change is observed.
- Pacing must not fall below the encoder's allowance. `Admission` drops whole frames and waits for a sync point after a drop.
- Preserve constant-time token comparison. An endpoint without a token is bound to loopback by `listen_address`.

## Checks

Run natively; no GStreamer is required.

```sh
cargo test -p thinlink-video-core --locked
```

Use table-driven tests for domain outcomes. Cover stamp rejection and wrap-around, freshness deadlines, bounded allocations and adaptation under changing observations. Avoid tests that only restate thresholds.
