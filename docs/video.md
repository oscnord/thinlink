# Video

The vehicle sends one H.264 RTP/UDP stream inside WireGuard. The relay publishes it to MediaMTX, and the ground adapter marks its freshness before sending it to QGroundControl. More viewers do not add another vehicle upload.

This page covers the video path, its adaptation and how its age is measured. [Drone features](drone.md) build on this path. [Testbed checks](testbed.md) define the scenarios, metrics and grading. [Hardware](hardware.md) covers cameras and deployment. Hardware and real-link performance remain unverified.

## Ownership

| Package | Owns |
|---|---|
| `thinlink-video-core` (L0) | Pure stamp and state stamp, freshness, pacing admission, byte-rate token bucket, uplink budget, adaptation ladder and controller, bearer-token check. No I/O or clock reads. |
| `thinlink-x264` (L0) | The `thinlinkx264enc` GStreamer element and its libx264 wrapper. |
| `thinlink-video` (L1) | GStreamer sender, relay ingress, feedback and adaptation loop, headless receiver, freshness output, snapshot encoding and metrics endpoint. Its workspace dependencies are `video-core` and `x264`. |

These crates know nothing about drones. Applications wire the video to `media`, `control`, `detail` and `policy`. Dependencies point downward, packages on one layer do not depend on each other, and `cargo xtask layers` checks the boundaries. See [development setup](testbed.md#development-setup).

GStreamer through `gstreamer-rs` supplies capture, encoding, RTP and decoding. Rust owns coordination and policy. Workers use synchronous threads and bounded `sync_channel` queues; there is no async runtime.

The sender offers a raw-frame hook before the stamp, and leaky frame taps off its tee. The hook returns regions in pixel coordinates. After drawing the stamp, the sender attaches them to the frame's buffer as `GstVideoRegionOfInterestMeta`. The output offers a per-frame decoration after the state stamp and before the labels. The caller owns the drone-specific work. A slow tap drops its own frames; it never holds the live path. The raw-frame hook runs in the streaming thread and must remain cheap.

Video produces bounded snapshots. Detail owns their store and transfer, policy owns retention, and applications connect them. [Retained detail](drone.md#retained-detail) has its own queue and yields to live video and control.

## Data path

```text
vehicle
  source -> convert -> tee -> scale/rate -> I420 rung -> frame hook -> stamp
    -> leaky queue -> H.264 encoder -> RTP -> pacer -> WireGuard
                                                        |
relay                                                   v
  UDP :5004 -> jitter buffer -> depay -> parse -> MediaMTX /live
                                                   |
ground                                             v
  RTSP -> decode -> freshness/state stamp -> decoration -> labels
    -> H.264 encoder -> local RTP/UDP -> QGroundControl
```

The testbed sends video to `10.77.0.1:5004/udp`. The relay publishes RTSP path `live` locally to MediaMTX. The ground pulls it over RTSP/TCP with `latency=0`, decodes it and keeps only the newest frame. QGroundControl can also play MediaMTX directly, but that path has no freshness indication.

The sender uses a live test pattern, a looped y4m file, RTP/H.264 ingest, V4L2 or libcamera. For a clip, set `kind = "file"` and `path` under `[source]` in `testbed/config/vehicle.toml`, and mount the y4m file into `companion`. It plays in real time and loops. Ingest decodes before the stamp and refuses datagrams from sources outside its configured `from` list. It drops corrupt decoded pictures rather than stamping them fresh. Real camera setup and its limitations are in [Hardware](hardware.md).

The relay uses `testbed/config/relay.toml`, including `[feedback]`, and publishes to `rtsp://127.0.0.1:8554/live`. MediaMTX's `testbed/mediamtx.yml` permits publishing to `live` from localhost only and reading from anyone who can reach it. RTSP over UDP or TCP and WebRTC are enabled; RTMP, HLS, SRT and MoQ are off. Development ports bind to host loopback.

The ground's `[video]` in `testbed/config/ground.toml` pulls `rtsp://192.168.232.20:8554/live`. Its output is 640x360 RTP/H.264 at up to 2000 kbit/s, sent to QGroundControl and `checks`. The adapter announces it as the vehicle's camera. [Drone setup](drone.md#qgroundcontrol) covers viewing it.

The encoder queue is leaky and holds at most two raw frames. RTP packets are at most 1200 bytes, below WireGuard's 1420-byte MTU. The relay's jitter buffer has `latency=50` and `drop-on-latency=true`. Its publisher's sinks use `sync=false`: in-order packets pass through immediately, and only a missing or reordered packet makes the buffer wait. The relay forwards without transcoding or re-timing.

Pipeline errors are typed. Supervision restarts a failed pipeline after one second and records its health and last error. A stopped pipeline releases its sockets. Frame IDs and the requested rung survive a pipeline restart within the vehicle process.

## Rungs and encoder

The ladder is [testbed/ladder.toml](../testbed/ladder.toml), best first. `start = 1` selects the second rung; the testbed overrides it with `--start-rung 0`.

| Rung | Group | Size | Frames/s | Bitrate |
|---|---|---|---|---|
| 0 | normal | 1280x720 | 15 | 1200 kbit/s |
| 1 | normal | 640x360 | 15 | 500 kbit/s |
| 2 | constrained | 640x360 | 8 | 250 kbit/s |
| 3 | constrained | 480x270 | 5 | 120 kbit/s |
| 4 | awareness only | 320x180 | 2 | 50 kbit/s |
| 5 | awareness only | 320x180 | 1 | 25 kbit/s |

The rung bitrate is the video allowance for any one second, including refresh bursts. x264 gets the rung bitrate times `1000 / (1000 + frame_period_ms)`, with a one-frame VBV buffer. That is about 94 % at 15 fps, two thirds at 2 fps and half at 1 fps. Headers come on top: each RTP packet costs another 122 bytes of RTP, UDP, IPv4, WireGuard and Ethernet headers. The source may send less than the allowance.

The sender uses `thinlinkx264enc`, a GStreamer `VideoEncoder` registered from Rust at startup, with no plugin file. It drives libx264 with preset `ultrafast` and tune `zerolatency`: no lookahead or B-frames, with sliced threads. Rate control uses average bitrate under the one-frame VBV. Parameter sets repeat. Intra refresh sweeps the picture over `refresh-ms`, set from `keyframe_interval_ms = 2000`. There are no periodic IDRs.

`roi-qp-offset` accepts -51 to 0, with 0 off, and is read when caps are set. `thinlinkx264enc` maps each buffer's ROI metadata to x264 quantiser offsets on every 16x16 macroblock a region touches. Overlaps do not add up. A negative offset enables variance adaptive quantization at strength 0.01; otherwise it is off. x264 honours offsets only with adaptive quantization enabled and clears it at strength 0. The offsets leave the input pixels unchanged. The hardware encoder does not use this property.

A size or frame-rate change needs only new caps. The element reopens libx264 for those caps and starts with an IDR. It computes `i_keyint_max` from their frame rate and the refresh period. `bitrate` and `vbv-buf-capacity` changes apply in place at the next frame. A forced key unit from upstream or downstream makes the next picture a whole IDR. Only IDRs are sync points; all other frames, including refresh-wave starts, are delta units. Time-based forced keyframes apply only to V4L2. Metrics keep `requested` and `applied` separate, including the first stamped frame in the new shape.

## Adaptation

The controller keeps the uplink queue empty and raises picture quality only after delay returns to its base. It observes the relay, never the testbed's configured link rate.

```text
vehicle -> probe {seq, sent_mono_us} -> relay 10.77.0.1:5006/udp
vehicle <- report {probe_seq, echo_sent_mono_us, window_ms,
                   rx_bytes, rx_packets, lost_packets, jitter_ms}
  own-clock round trip -> video-core::adapt -> rung and pace -> sender
                       -> video-core::budget -> telemetry cap
```

The vehicle probes every 250 ms inside WireGuard. The relay answers at once with ingress observations collected between answers. Round trip uses the vehicle's monotonic clock, so feedback needs no shared clock. Malformed answers, answers over 512 bytes, answers from another address and answers to unknown or older probes change nothing. The relay's `lost_packets` is jitter-buffer loss minus late packets, so reordering is not counted as loss.

Queue delay is the smaller of the last two round trips minus the smallest over the last 10 s. One late answer can be jitter; two in a row indicate a queue. The trend is the least-squares slope over the last second of probes sent after the last change.

Above 150 ms and rising, the controller cuts to the best rung that fits 75 % of measured capacity after the budget reserve. Capacity includes received video and the other traffic in the proportion that reached the relay. The spare quarter drains the queue. A standing queue causes a one-rung cut. If the delay still does not fall, the base delay changed: the cut is taken back and the base resets. A falling queue is held while it drains.

A single answer more than 300 ms above the base is a deep queue and can cause an immediate cut. At `drain_above_ms = 300`, the video pauses at the stamp probe until the queue drains. Control, telemetry and probes continue. Answers to probes sent ahead of the pause refine the capacity estimate, and video resumes at the rung it supports with a whole IDR. `LinkBack`, the first answer after a fall for silence, also requests an IDR for the next picture. Delay that does not drain while paused is treated as a base shift.

Loss below 2 % permits a climb, above 10 % cuts one rung, and between them holds. The reading needs at least 500 packets within 10 s. At the slow rungs there may be too few packets, so delay guards them alone.

Ten seconds of healthy answers permit one rung up. Healthy means queue delay at most 80 ms and loss low or unknown. Changes are at least 2 s apart, except for a deep queue. After feedback has first arrived, silence for 3 s falls to the floor. Silence without a preceding deep queue is an outage: 1 s of healthy answers restores the earlier rung, limited by measured capacity. Before the first answer, the start rung stands.

### Climb and packet pacing

A climb into a rung costing more than 110 % of the last measured uplink starts with a bitrate ceiling at 110 % of that uplink. After each 2 s of healthy answers in which nearly all the ceiling was used, it rises by 10 %, up to the rung's bitrate. A source sending less cannot prove more capacity, so the ceiling stays. These `probe` changes are not rung changes.

A cut within a dwell of a climb or ceiling raise doubles that rung's hold, starting at 30 s and capped at 5 minutes. A later measurement of at least 10 % more capacity resets the hold. Capacity remains known until another cut measures it or a base shift clears it. The first climb without a measurement is blind.

The sender's pacer holds RTP packets in its own backlog. `Controller::pace_bytes_per_s` uses `video-core::pace::rate`: never below the wire cost allowed to the encoder, never above 120 % of it, and within those bounds 110 % of the believed uplink. The belief includes capacity proved by later ceiling raises, so a recovered link is not held to an old measurement. Without a measurement, the pace is 120 % of the allowance.

A token bucket allows a 20 ms burst, at least two packets. A frame whose backlog ahead would take more than 150 ms to send is dropped whole. Following delta frames are dropped until a frame marked as a sync point fits. The first drop requests a forced key unit, so `thinlinkx264enc` encodes the next picture as a whole IDR. A frame whose own send time exceeds 150 ms is admitted if the backlog ahead fits. Control, probes and telemetry use other sockets and never wait behind this queue.

### Shared uplink budget

`video-core::budget` reserves capacity in order:

1. Control, probes and challenges at their largest wire sizes, about 1.1 kB/s, or measured usage when higher.
2. Telemetry at `offered_percent = 250` of the offered frame bytes over 5 s, between `essential_bytes_per_s = 200` and `bytes_per_s = 8000`, plus measured datagram headers.
3. Retrieval's configured allowance while a job runs, zero while idle.
4. Video gets the rest.

The telemetry share covers its once-a-second burst. Preferred state messages spend the bucket first; bulk messages use only its upper half. After capacity is measured, the telemetry cap is its share less headers. Until then the configured 8000 B/s cap applies. [Drone features](drone.md) describes telemetry and retrieval admission.

### Adaptation thresholds

Values are in `[adapt]` of [testbed/limits.toml](../testbed/limits.toml). Durations are milliseconds unless named otherwise.

| Key | Value | Reason |
|---|---|---|
| `probe_interval_ms` | 250 | Feedback waits for a round trip and any queue. Four probes a second expose a growing queue sooner and cost about 5 kbit/s on the wire. |
| `min_rtt_window_ms` | 10000 | Keeps an empty-queue base while a queue drains, while allowing route changes to replace it. |
| `queue_delay_high_ms` | 150 | Above measured healthy jitter, up to 55 ms in the testbed. The smaller of two answers filters isolated delays. |
| `drain_above_ms` | 300 | Twice the high threshold. With GEO propagation, this queue would make frames about 0.6 s old before further delay. Pausing lets the whole uplink drain it. |
| `queue_delay_base_ms` | 80 | Clears healthy jitter so a healthy streak can accumulate. |
| `trend_window_ms`, `trend_ms_per_s` | 1000, 100 | Four probe intervals distinguish queue growth from jitter. The 75 % capacity fit drains at 250 ms/s, more than twice the trend threshold. |
| `capacity_fit_percent` | 75 | Leaves room to drain and tolerates error in the capacity measurement. |
| `loss_window_ms`, `loss_min_packets` | 10000, 500 | Avoid reacting to a small random burst. Older packets do not describe the current link. |
| `loss_low_percent`, `loss_high_percent` | 2, 10 | GCC loss bands: climb below the lower bound, cut above the upper one, otherwise hold. |
| `stale_after_ms` | 3000 | Twelve missing probe intervals indicate an outage or an already deep queue. |
| `outage_recovery_ms` | 1000 | Four healthy probe intervals show the link back. An outage alone says nothing about its capacity. |
| `dwell_ms` | 10000 | Require sustained health before another rung, which can double the rate. |
| `min_change_interval_ms` | 2000 | Give a change time to appear in feedback. |
| `capacity_hold_ms`, `capacity_hold_max_ms` | 30000, 300000 | Retry a failed climb less often, doubling the hold after another failure. |

The loader validates durations from 1 to 600000 ms, base below high, drain above high, the trend window and stale threshold spanning two probes, and the change interval within the dwell. The trend threshold must be at most half the drain rate implied by `capacity_fit_percent`.

## Frame age and clocks

The vehicle draws a frame ID and capture time into each frame before encoding. The headless receiver decodes MediaMTX's stream and reads the stamp. Frame age is receive time minus capture time, using `stamp::age_ms` for the 32-bit wrap.

For test, file, ingest and libcamera sources, `capture_ms` is wall time at the stamp. Age includes the encoder queue, encoding, packetisation, WireGuard, uplink impairment and queues, relay buffering and parsing, MediaMTX, receiver buffering, decoding and colour conversion. It excludes the source before the stamp and the display after decoding. RTP timestamps alone are relative media time, not capture wall time.

V4L2 uses the capture buffer's timestamp and driver sequence number before MJPEG decoding. A repeated sequence keeps the original capture time. No queue may be added between that probe and the stamp. A driver that repeats pixels with a new timestamp and sequence is not detected. This path remains unverified on hardware; see [camera measurement](hardware.md).

All testbed containers share one kernel clock. Across machines, age needs a clock mapping with a stated uncertainty. The ground adds that bound to age before classifying it. Without a fresh bound it never labels a frame Live or LowRate.

The clock policy derives each machine's bound from `chronyc -c tracking`: the larger absolute system or last offset, plus root dispersion and half root delay, rounded up. It grows by 500 ppm with age and expires after 15 s. Both bounds are added, with 10 ms of margin once. The vehicle reports its bound every 2 s. The testbed uses `clock = "shared"`. Real-mode clock settings are in [setup](real-mode.md#setup); deployment details and fixed-bound limitations are in [clock sync](hardware.md#clock-sync).

The receiver reads wall time once per decoded frame. Thereafter the newest frame's age grows by monotonic elapsed time, so a wall-clock step cannot make a held picture younger. Repeated display frames do not count as new observations. Lab age and cadence use readable stamps with plausible ages from 0 to 60 s and IDs not already seen in the window.

### Stamp layout

`video-core::stamp` owns both stamps:

- Payload: 9 bytes, `frame_id: u32` and `capture_ms: u32`, both big endian, followed by CRC-8/I-432-1 over those eight bytes. The polynomial is `0x07`, init `0x00`, xorout `0x55`. A uniform stamp area never validates.
- The 72 bits are drawn MSB first, row-major, in 24 columns and three rows of square blocks at the I420 luma plane's top-left. Block side is `width / 24`, at least 8 pixels. Width below 192 or height below three blocks is rejected. One is luma 235, zero luma 16.
- Reading takes the mean of each block's centred half-size square. Above 128 means one. A CRC mismatch is unreadable.
- The ground adds a fourth row of 24 blocks: four state bits (1 Live, 2 Stale, 3 Frozen, 4 Disconnected, 5 AgeUnknown, 6 LowRate), 16 age bits in 10 ms units, and CRC-4/INTERLAKEN over those 20 bits. Age is all ones when unknown or above 655 s.

The original stamp survives the ground's decode and re-encode. Frame IDs increase for the vehicle process lifetime, across pipeline restarts. Read stamps before scaling: scaling can move their blocks off the sampling grid.

## Freshness output

The ground reads every decoded stamp. A frame is new when its capture time is later than any before and its age is plausible. Unreadable or implausible stamps count as unreadable. The output applies `video-core::freshness::classify` in this order, using `[freshness]` in [testbed/limits.toml](../testbed/limits.toml):

| State | Condition | Setting |
|---|---|---|
| Disconnected | The source pipeline is down, or no frame was decoded within the threshold. | `disconnected_after_ms = 2000` |
| AgeUnknown | Too few frames arriving in the last second had readable stamps. | `min_readable_percent = 50` |
| Frozen | Frames still arrive, but none has a new stamp within the threshold. At a low rate, add one source frame interval. | `frozen_after_ms = 500` |
| AgeUnknown | No age or clock bound is available. | |
| Stale | Newest frame age plus clock bound exceeds the applicable threshold. | `stale_after_ms = 600`, `low_rate_stale_after_ms = 1800` |
| LowRate | The source frame interval exceeds the threshold. | `low_rate_interval_ms = 150` |
| Live | Otherwise. | |

The source interval comes from capture times and frame IDs, using the lower median over the second before the newest frame. It does not come from network arrival spacing. A blackout does not count as Frozen because that state needs arriving frames.

### Freshness thresholds

At 15 fps a new frame normally follows within about 100 ms. GEO frames in the testbed arrive about 320-350 ms old, leaving the newest picture under about 450 ms before replacement. The 600 ms stale threshold leaves margin but marks an old picture promptly.

The 500 ms frozen threshold is well beyond the normal frame interval. A repeated V4L2 buffer can trigger it; on other sources it catches a repeated stamp or a stall after stamping. Decoder concealment does not make an older picture current: the output does not send a picture older than the newest it has, or one with an unreadable stamp while the state is Live or LowRate.

The 2 s disconnect threshold exceeds measured waits with a running source. Stale or Frozen can show before it. The 50 % readability threshold prevents an age claim when most arriving stamps cannot be read.

A source interval above 150 ms is labelled LowRate. At 2 fps on GEO, a current picture can age from about 330 to 830 ms between frames. The separate 1800 ms stale threshold avoids alternating Live and Stale at every slow-rung frame. At 1 fps it allows the 1000 ms frame interval plus arrival age and margin, while remaining below the disconnect threshold.

### Output timing

A new decoded frame is copied, labelled and sent to the encoder immediately. The last frame repeats only after two output periods without an output, or sooner when `freshness::next_change_ms` says its state may change. Before the first frame, the output is a black slate. Source and output pipelines restart independently, so the indication continues when the source fails.

The output uses x264 `tune=zerolatency speed-preset=ultrafast`, no B-frames and two slice threads. Its encoder input queue holds one frame and is leaky. Decode uses slice threads rather than frame threads. The overlay is white for Live, amber for LowRate and red otherwise; an unreadable state stamp says `NO STATE`.

A frame labelled Live or LowRate carries the deadline when that state may change. If encoding or sending holds it past that deadline, the sender drops it, increments `late` and requests a keyframe. A stall causes an output gap. The ground's labels describe its output; they do not measure QGroundControl's display delay.

## Measurement method

Keep headless decoder age, ground-adapter output age and optical glass-to-glass age separate. Camera exposure and readout require hardware measurement. QGroundControl adds its own decode, sink and display delay. The lab measures independently of adaptation, and missing measurements or thresholds never count as passing. [Testbed](testbed.md) defines check metrics, result files and budget verdicts.

Picture quality is separate from age: a young frame can still be damaged by loss. The [quality check](testbed.md#quality-check) compares decoded frames with a moving reference, both at the received size and scaled to the original size. Missing frames, unreadable stamps and age are measured separately. A quality score does not prove a current picture.

### Stage timing

Each process measures its own stages with pad probes keyed by frame timestamp. `crates/video/src/stages.rs` reports p50, p95 and maximum over the last 1024 frames. This is a rolling window, about 68 s at 15 fps, not the check's own window. A short run after another profile can include old frames.

```sh
just lab-check skeleton geo --warmup-s 20 --duration-s 60
just stages <run>
```

The stage boundaries are:

- Source to stamp, before ordinary stamped age begins.
- Encoder queue, encode, and stamp to the frame's last sent packet, including pacing.
- Relay jitter buffer, then depayload and parse.
- Headless receiver decode.
- Ground-adapter decode, then freshness label and re-encode.

Separate stage percentiles do not add up. Headless age covers the vehicle, link, relay, MediaMTX and one decode. The ground-adapter output adds its own label and re-encode; QGroundControl still adds a decode and display. MediaMTX and process hops account for time not assigned to those stages.

The noise source in `testbed/config/vehicle-snow.toml` fills each rung's bitrate and exposes pacing cost:

```sh
VEHICLE_CONFIG=vehicle-snow.toml just lab-up
```

At low rungs it can blur the stamp past reading, so normal ball-pattern video measures lossy profiles. These are different source workloads.

### Expected latency

These container measurements use the noise source, x264 and a headless receiver on the development Mac. They describe that workload. Pi, real-link and QGroundControl display latency need separate measurements. Values are milliseconds, p50 / p95.

| Stage | `lan` | `geo` |
|---|---|---|
| Source to stamp | 0.2 / 0.4 | 0.3 / 0.5 |
| Encoder queue | 0.1 / 0.3 | 0.1 / 0.3 |
| Encode | 2.2 / 3.7 | 1.9 / 3.5 |
| Stamp to last packet sent, including queue, encode and pacing | 14.4 / 22.9 | 2.3 / 7.9 |
| Link, one way | 0 | 300 ± 10 |
| Relay jitter buffer | 0.0 / 0.1 | 11.1 / 30.1 |
| Relay depayload and parse | 0.0 / 0.1 | 0.0 / 0.0 |
| Headless decode | 0.5 / 0.9 | 0.4 / 0.8 |
| Frame age at headless receiver | 16 / 25 | 317 / 330 |
| Ground adapter decode | 0.5 / 0.9 | 0.4 / 0.8 |
| Ground adapter label and re-encode | 1.2 / 1.9 | 1.0 / 2.0 |

Pacing is the largest LAN stage because noise fills the rung's bitrate. On GEO, the narrower uplink constrains the encoder and most added time is relay waiting for reordered packets. MediaMTX and process hops account for about 1-3 ms. A camera's frames can be smaller than noise. Scene complexity, rung and loss change these expectations.

The operator's loop also includes the command's outward delay. Propagation cannot be removed by encoder tuning. [Drone](drone.md) covers control behavior and [simulation limits](drone.md#simulated-flight).

## Recommended settings

Use `encoder = "hardware"` on a Pi 4 or Zero 2 W, as in `deploy/vehicle/vehicle.toml`. The sender selects `v4l2h264enc` when available and falls back to `thinlinkx264enc`. The hardware path remains unverified; [Hardware](hardware.md) describes the bench checks. CPU quotas on the development Mac do not emulate a board's cores, memory bandwidth, caches or hardware encoder.

Keep the sender's [encoder settings](#rungs-and-encoder) for bounded latency. The ground output uses GStreamer's `x264enc`. H.264 is the implemented codec.

The relay needs no jitter-buffer tuning. `JITTER_LATENCY_MS` in `crates/video/src/ingress.rs` is 50 ms and only bounds a missing packet's wait. Keep its sinks unsynchronised.

Turn on QGroundControl's Low Latency Mode. Without it QGroundControl can buffer packets for up to 80 ms and synchronise its video sink. The [QGroundControl setup](drone.md#qgroundcontrol) covers connecting the freshness output.
