# AGENTS.md: crates/detect

`thinlink-detect` (L2) finds configured object kinds and AprilTags in frames. It depends on `thinlink-policy` for `NormRect` and `Kind`. Keep GStreamer outside this crate. Follow the root `AGENTS.md`.

## Main pieces

Paths are relative to this crate.

| File | Ownership |
|---|---|
| `src/lib.rs` | `Detector`, `Detection`, model loading and input conversion |
| `src/decode.rs` | Output decoding and `non_max_suppression` |
| `src/tag.rs` | `TagDetector`, `TagFound` |

## Rules

- Callers supply RGB frames at the model's input size. Keep BGR normalization inside `Detector`. Invalid frame dimensions or a failed inference return no detections.
- Keep model revision and SHA-256 pins aligned across `just model` and the runtime image. Model paths are `models/person.onnx` and `/usr/share/thinlink/person.onnx`. Model behavior is in `docs/drone.md#detector`.
- Use `tract-onnx` for inference. Keep configured kinds attached to detections through decoding and overlap filtering.
- Construct `TagDetector` on its worker thread; it is not `Send`. Its `apriltag` dependency builds the C library with `cmake`, supplied by `testbed/Containerfile`.
- Keep tag detection unambiguous: a duplicate wanted ID returns nothing. Validate luma dimensions before constructing the image.

## Checks

```sh
cargo test -p thinlink-detect --locked
```

Run natively with the required native dependencies, or in the build container. Tests use constructed output tensors and generated tag images. Keep a real-model run labelled as a manual check.
