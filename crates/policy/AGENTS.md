# AGENTS.md: crates/policy

`thinlink-policy` (L1) owns drone decisions over `thinlink-video-core`. Follow the root `AGENTS.md`. Do not add GStreamer, MAVLink, HTTP or filesystem access. Pure decisions take time as an argument. `RegionCell` owns the shared monotonic origin used by its callers.

## Main pieces

Paths are relative to this crate.

| File | Ownership |
|---|---|
| `src/control.rs` | `LeaseState<T>`, `ContinuousSlot`, `TxnCache` |
| `src/clock.rs` | Clock bounds and `fixed_holds` |
| `src/region.rs` | `NormRect`, `RegionCell`, `active` |
| `src/follow.rs` | `Follower`, `Order` |
| `src/draw.rs`, `src/kind.rs` | Box drawing and `Kind` vocabulary |
| `src/predict.rs`, `src/mark.rs` | Aim, horizon and mark drawing |
| `src/inset.rs` | Detail placement and label |
| `src/landing.rs` | `decide`, camera geometry |
| `src/position.rs`, `src/sighting.rs` | `Position`, `Sightings` |
| `src/retention.rs`, `src/retrieval.rs` | `evict`, `permit`, chunk budgeting |

## Rules

- Drive `LeaseState` with vehicle monotonic milliseconds, never ground timestamps. Refused admission changes no state. Applying a setpoint requires `Admitted`.
- Keep sequence numbers separate by envelope kind. Record discrete transactions after application. A repeated transaction is answered without applying it again; the same ID with another digest is refused.
- Keep only the newest setpoint. Its expiry is bounded by its challenge and lease as well as its own lifetime.
- `RegionCell::regions` selects encoder regions; `RegionCell::confirmed` selects the box reported to the ground. Do not duplicate selection outside this crate. All `Timed::at_ms` values use the cell's origin.
- Follow answers carry their generation. Ignore answers for another generation. Repeating the same seed must not restart a lost follow; `clear` forgets it.
- Keep `Follower` search allocation-free. Do not learn from low-score matches. Bound pixel reads and leave row padding untouched in drawing loops.
- Draw predictive marks only for `Live` or `LowRate` with known age. Keep the ring hollow and dashed, and keep marks below the stamp rows. Insets also require a current state and an age within their limit.
- `landing::Sighting.at` is the frame's tee time, not capture time. Keep geometry decisions independent of clocks and autopilot I/O.
- Retention uses capture wall time for object age and `Instant` for pins. Honor active pins and the bounded outage share. Do not turn a pinned over-quota store into an eviction permission.
- Retrieval requires `permit` before each chunk. Unknown feedback is not permission. Size chunks with `chunk_size` and account for request overhead with `request_wire_bytes`.
- Mark a sighting sent only when its sender accepts it. Keep the sightings collection bounded.

Behavior is documented in `docs/drone.md#regional-video`, `docs/drone.md#predictive-display`, `docs/drone.md#retained-detail` and `docs/drone.md#landing-on-a-marker`.

## Checks

```sh
cargo test -p thinlink-policy --locked
```

Keep tests pure and table-driven. Exercise lease expiry and duplicate transactions, follow generations, drawing boundaries, retention pins and retrieval admission. Use public inputs and outcomes. Video freshness, adaptation and pacing tests belong in `thinlink-video-core`.
