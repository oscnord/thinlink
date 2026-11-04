# AGENTS.md: crates/video

`thinlink-video` (L1) owns GStreamer video and media feedback. Its workspace dependencies are `thinlink-video-core` and `thinlink-x264`. Keep drone logic in `crates/media`. Read the root `AGENTS.md` and `docs/testbed.md`.

| Read | Subject |
|---|---|
| `docs/video.md#data-path` | Pipelines and transport |
| `docs/video.md#freshness-output` | Ground output and labels |
| `docs/hardware.md#boards` | Camera and encoder support |

## Code map

| File | Main pieces |
|---|---|
| `src/sender.rs` | `SenderStats`, `Hooks`, `FrameHook`, `Snapshot`, rung changes |
| `src/sender/tap.rs` | `Tap`, `TapSpec`, latest-frame handoff |
| `src/sender/pacer.rs` | Bounded RTP backlog and pacing |
| `src/camera.rs`, `src/ingest.rs` | Camera sources, `CaptureClock`, external RTP ingest |
| `src/ingress.rs`, `src/feedback.rs` | Relay forwarding and probe replies |
| `src/adapt.rs` | `AdaptStats`, sender adaptation loop |
| `src/receiver.rs` | `Source`, `Observation`, stamp reading |
| `src/output.rs` | `Decoration`, `Note`, `Tracker`, freshness output |
| `src/lib.rs`, `src/metrics.rs` | `MediaError`, `Health`, `Running`, supervision and metrics endpoint |

## Invariants

- Build pipelines with element factories. Never parse configuration as pipeline syntax.
- Keep queues bounded and leaky. Use drop-only rate conversion, no encoder B-frames or lookahead, and unsynchronised live sinks. This includes sinks created inside `rtspclientsink`.
- Keep extension points to `Hooks` and `Decoration`. A new one needs a second caller that cannot use these; do not add a plugin registry.
- `FrameHook` runs in the streaming thread before the stamp. Keep it cheap; count hook failures and allow the frame through. Taps hand off the newest frame without blocking capture. Honour `TapFrame::stride`.
- Keep the snapshot branch leaky and its handoff nonblocking. Storage belongs to the caller.
- Draw the stamp after scaling. Never add a queue between the V4L2 capture probe and the stamp probe: they share `CaptureClock` in one streaming thread.
- Read stamps before receiver scaling. Match observations and decoded frames by presentation time, never arrival order.
- Keep ingest decoding free of jitter buffers and frame threading. Preserve `output-corrupt=false`; stamp only decoded pictures. Select peers through `ingest::accepts` and release the reader socket when its run ends.
- Keep frame IDs and the requested rung in `SenderStats` across restarts. Apply pipeline changes outside the lock held by the stamp probe.
- Pacer recovery requires an IDR. Refresh-wave starts are delta units. Resume from a drain and `LinkBack` request a keyframe.
- Output copies decoder buffers before drawing. Keep source and output restarts independent. Labels and decoration notes must belong to the same frame; preserve the `LabelDeadline` check at transmission.
- Keep age progression monotonic after the decode-time wall-clock reading. Count new source observations, not output repeats.
- Release pipelines through `Running`; report failures through `MediaError` and `Health`.

## Checks

Run in the build container. GStreamer packages are defined in `testbed/Containerfile`.

```sh
just check
just lab-up && just lab-check skeleton lan && just lab-check qgc-status lan
```
