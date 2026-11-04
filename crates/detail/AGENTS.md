# AGENTS.md: crates/detail

`thinlink-detail` (L2) owns retained JPEG snapshots and their HTTP retrieval jobs. It depends on `thinlink-policy` and `thinlink-video-core`. It builds natively. GStreamer stays outside this crate; callers supply JPEG bytes. Follow the root `AGENTS.md`.

## Main pieces

Paths are relative to this crate.

| File | Ownership |
|---|---|
| `src/store.rs` | `Store`, `Entry`, `Tag`, recovery and snapshot pins |
| `src/wire.rs` | `JobId`, `JobRequest`, `Kind`, `Meta`, `Listing` |
| `src/crop.rs` | `pixel_rect`, `cut`, bounded JPEG decoding |
| `src/assemble.rs` | `Assembly`, chunk validation |
| `src/jobs.rs` | `Jobs`, expiry and outstanding-byte limits |
| `src/relay.rs` | `Relay`, separate vehicle and ground routes |
| `src/vehicle.rs` | `run`, retrieval pacing and cancellation |
| `src/client.rs` | `Client`, HTTP access for the ground and lab |

## Rules

- Build store paths from validated entry numbers. Requests name opaque `JobId`s, never filesystem paths.
- Preserve atomic snapshot writes: `.tmp`, `sync_data`, rename. Write `.pos` metadata before the JPEG. Recovery must ignore unrelated filenames, remove orphan metadata and never index a `.tmp` file.
- Keep snapshots without `.pos` metadata readable. Snapshot pins use monotonic time; capture timestamps name objects. Delegate eviction to `thinlink_policy::retention`.
- Store writes run on the vehicle worker, never the video pipeline. Retrieval yields to live video and control.
- Revalidate relay requests on the vehicle. Check `retrieval::permit` before polling and before each chunk. Use `chunk_size` and the allowance's token bucket. Stop work when the job expires or is cancelled.
- Bound jobs by both count and requested bytes, including completed jobs until expiry. Reap their objects with them.
- Validate per-chunk hashes and whole-object hashes. An identical duplicate is a retry; a different duplicate is a conflict. Enforce allocation limits before decoding or collecting bodies.
- Keep listing completion separate from JPEG completion through `Kind::is_listing`. JPEG metadata must match the decoded size.
- Check the bearer token before routing. Keep vehicle and ground routes on their own listeners. Do not claim application body limits bound HTTP headers or connection lifetimes; see `docs/hardware.md#limits-of-a-deployment`.
- Clear a snapshot's outage mark only after successful retrieval.

Store and protocol details: `docs/drone.md#retained-detail`. Lab acceptance: `docs/testbed.md#retrieval-check`.

## Checks

```sh
cargo test -p thinlink-detail --locked
```

Cover store recovery, persisted tags, nearest-snapshot selection and outage release. Exercise assembly conflicts and byte limits, crop bounds and listing completion. Retention decisions are tested in `thinlink-policy`. Use `lab check recording` and `lab check retrieval` for HTTP wiring and store integration.
