# AGENTS.md: crates/x264

`sat-teleop-x264` (L0) provides `satx264enc`, a GStreamer `VideoEncoder` over libx264. It has no workspace dependencies and is GPL-2.0-or-later because it links libx264. See the root `AGENTS.md` for the unsafe-code exception.

## Invariants

- Call `register()` after GStreamer initialisation. Registration is in-process; there is no plugin file.
- `build.rs` generates bindings from the system `x264.h` with bindgen. Keep FFI inside the safe wrapper in `src/encoder.rs` and the generated `sys` module in `src/lib.rs`. Every unsafe block needs a `SAFETY:` comment.
- Caps reopen the encoder. Derive `i_keyint_max` from their frame rate and `refresh-ms`; apply bitrate and VBV changes at the next frame.
- A forced key unit makes the next picture a whole IDR. Only IDRs are sync points. Refresh-wave starts remain delta units.
- `roi-qp-offset` is -51 to 0, with 0 off, and is read when caps are set. `GstVideoRegionOfInterestMeta` on the input buffer supplies pixel rectangles. `roi::quant_offsets` applies the offset to every 16x16 macroblock a rectangle touches; overlaps do not add up. The input pixels are unchanged.
- A negative offset enables variance adaptive quantization at strength 0.01. x264 honours offsets only with adaptive quantization on and clears it at strength 0.

## Checks

Build and test in the build container with GStreamer and libx264 development packages:

```sh
cargo test -p sat-teleop-x264 --locked
```

Tests inspect NAL type 5 and delta flags for forced IDRs and refresh waves. They also check the refresh period after a frame-rate change. ROI tests cover touched macroblocks and overlaps. An element test checks that a frame with ROI metadata uses more bits after passing through a leaky queue.
