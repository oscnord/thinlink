# AGENTS.md: crates/detect

`sat-teleop-detect` (L2) finds people and vehicles in a frame. It depends on `sat-teleop-policy` (`NormRect`, `Kind`) and on no GStreamer: the sender scales a frame to the model's input and hands over RGB bytes. It builds natively. See the root `AGENTS.md`.

## Model

NanoDet-Plus-m 320, Apache-2.0, run with `tract-onnx` (pure Rust, nothing native to cross-build). `just model` fetches it to `models/person.onnx` (ignored by git) and the runtime image to `/usr/share/sat-teleop/person.onnx`, both pinned by revision and SHA-256; a mismatch fails. Never use a model or code under AGPL or similar: Ultralytics YOLO is excluded.

- Input: `[1, 3, 320, 320]` f32, BGR, `(value - mean) / std` per channel. The shapes are fixed in the model: `Detector::load` fails for another input size, naming the path and the cause.
- Output: `[1, 2125, 112]`: the cells of the stride 8, 16, 32 and 64 maps, then per cell 80 class scores (already 0..1; person is class 0, bicycle 1, car 2, motorcycle 3, bus 5, truck 7, boat 8) and four sides of 8 box-edge logits. `decode` turns a cell into a box with the softmax-weighted bin times the stride, clamps it to the picture and drops a box under 1 % of a side.
- `Detector::load` takes the kinds to find; a cell is the best scoring of them at or over `min_score`, and `Detector::detect` returns each detection with its kind. It runs on a frame of exactly the input size; any other size, or a failed run, finds nothing. Overlapping boxes (IoU over 0.5) keep the better score.

## Tag

`TagDetector` finds one tag36h11 tag (the wanted id) in a luma frame with `AprilRobotics`' reference detector (BSD-2-Clause) through the `apriltag` crate, which builds the C library with `cmake` (in the build image, `testbed/Containerfile`). The detector holds a raw pointer and is not `Send`: make it on the thread that uses it. It takes no bit errors and no decimation, and finds nothing when the wanted id is in the frame twice.

## Checks

```sh
cargo test -p sat-teleop-detect --locked
```

The tests are pure (a hand-built output tensor, the overlap filter); a run with the real model is a manual check, not a test.
