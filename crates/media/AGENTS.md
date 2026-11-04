# AGENTS.md: crates/media

`thinlink-media` (L2) owns the drone's use of video frames. It connects `thinlink-policy` decisions to the hooks and taps in `thinlink-video`. Apps wire the pipelines and run detector workers.

Read the root `AGENTS.md`, `crates/video/AGENTS.md`, `docs/testbed.md` and `docs/drone.md#regional-video`.

## Code map

| File | Main pieces |
|---|---|
| `src/hooks.rs` | `sender_hooks`, builds sender `Hooks` |
| `src/region.rs` | `Regional`, `RegionSettings`, `DetectorFrame`, `FollowFrame` |
| `src/tags.rs` | `TagFeed`, `TagFrame`, row-padding removal |
| `src/decorate.rs` | `draw`, `NOTE`, `Predictive`, `InsetCount` |
| `src/inset.rs` | `Image`, `DetailCell`, bounded crop decoding |
| `src/retrieval.rs` | `conditions`, adaptation inputs for retrieval policy |

## Invariants

- Keep detection and tracking workers in `thinlink-vehicle`. Do not depend on sibling `thinlink-detect` from this crate.
- `RegionCell` owns region selection. `Regional` converts `regions()` to encoder rectangles; the control loop sends `confirmed()`. Never duplicate that selection here.
- Validate `[region] roi_qp_offset` with `validate_roi_qp_offset`. Zero disables encoder offsets and pilot-box retention; detection outlines still draw. Keep pixel drawing separate from quantiser offsets.
- Draw hook content before the stamp. Draw ground decoration after the state stamp and before labels. Never draw over stamp rows.
- Keep frame hooks cheap and taps nonblocking. Slow consumers skip to the newest frame. Hook failures count and return no regions without stopping video.
- Honour tap stride. `TagFeed::next_frame` removes row padding; `next_follow_frame` rejects a buffer with the wrong picture size. Preserve the tee time when deriving frame age.
- A detector model load failure must leave video running. Build its tap only when the model loads.
- `inset::Image` checks decode limits before allocation. `DetailCell` ages the retained image by monotonic elapsed time.
- Use `thinlink_policy::inset::place` for inset eligibility. Draw an inset only on `Live` or `LowRate` frames, outside stamp rows and with its age label.
- Keep the decoration note tied to its frame. `decorate::NOTE` supplies label text; `thinlink-video` owns the overlay element.
- `retrieval::conditions` adapts `AdaptStats` into policy inputs. Keep retrieval-specific decisions out of the video crate.

## Checks

Run in the build container:

```sh
just check
just lab-up && just lab-check skeleton lan
just regional=1 sim-up && just lab-check follow lan
```

Follow the root resource rules between lab and simulation runs. Test region geometry and frame-layout rejection through observable outputs.
