# AGENTS.md: crates/x264

`thinlink-x264` (L0) provides the `thinlinkx264enc` GStreamer `VideoEncoder`. It has no workspace dependencies. Read the root `AGENTS.md` for the unsafe-code boundary.

| File | Responsibility |
|---|---|
| `build.rs`, `src/lib.rs` | Bindgen from system `x264.h`, generated `sys` module |
| `src/encoder.rs` | `Encoder`, libx264 wrapper |
| `src/element.rs` | GStreamer element and `register()` |
| `src/roi.rs` | `quant_offsets`, macroblock mapping |

## Invariants

- Call `register()` after GStreamer initialisation; registration is in-process.
- Keep FFI in `src/encoder.rs` and `sys`. Every unsafe block needs a `SAFETY:` comment.
- Caps reopen the encoder. Derive `i_keyint_max` from their frame rate and `refresh-ms`. Apply bitrate and VBV changes at the next frame.
- A forced key unit makes the next picture a whole IDR. Only IDRs are sync points; refresh-wave starts remain delta units.
- `roi-qp-offset` is read at caps setup. Regions come from `GstVideoRegionOfInterestMeta`; overlapping rectangles do not accumulate offsets or change input pixels.
- Negative offsets require variance adaptive quantization at strength 0.01. Strength 0 clears offsets in x264.

## Checks

Run in the build container with GStreamer and libx264 development packages:

```sh
cargo test -p thinlink-x264 --locked
```

Check forced-IDR NALs and delta flags, refresh periods after frame-rate changes, and ROI macroblock coverage. Preserve the element test that passes ROI metadata through a leaky queue.
