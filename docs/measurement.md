# Measurement

How `lab check` measures the live video path, the MAVLink path, control, the ground's freshness output, the vehicle's adaptation and its retained snapshots and their retrieval, what each metric means, where results go and how they are graded. The testbed itself is described in `docs/testbed.md`.

## What is measured

The vehicle draws a stamp (frame id and capture time) into every frame before encoding. The headless receiver in `checks` plays the relay's RTSP stream, decodes every frame and reads the stamp back. Frame age is receive time minus capture time.

```text
vehicle (companion)                         relay (relay-net namespace)                  checks
videotestsrc -> rung (I420) -> STAMP  ...    udpsrc 10.77.0.1:5004 -> jitterbuffer 50 ms   rtspsrc (UDP, latency 80 ms)
  -> leaky queue -> x264enc -> rtph264pay    -> depay -> parse -> rtspclientsink           -> depay -> parse -> avdec_h264
  -> udpsink 10.77.0.1:5004 (WireGuard)      -> MediaMTX :8554/live                        -> I420 -> appsink -> READ STAMP
        \--> router (tc impairment, NAT) --/                    \---- ground_net, unimpaired ----/
```

So frame age covers: the encoder queue and x264, RTP packetisation, WireGuard, the router's impairment and queues (uplink only), the relay's jitter buffer, depayload and parse, the RTSP hop through MediaMTX, the receiver's jitter buffer, H.264 decoding and colour conversion. Neither jitter buffer holds in-order packets, because no sink downstream of them syncs: their latencies (50 ms at the relay, 80 ms at the receiver, QGC's default) only bound the wait for a missing packet. It does not cover camera exposure and readout (the stamp is drawn after the source) or display: it is headless decoder evidence, not glass-to-glass, and is labelled as such. QGC rendering adds its own sink and display latency on top.

## Clock assumption

`capture_ms` is the vehicle's wall clock (UNIX epoch milliseconds modulo 2^32) when the stamp is drawn; `received_ms` is the receiver's wall clock when the decoded sample is pulled. All testbed containers run on one kernel and share its clock, so the difference is a true age with no offset. Across machines (a real Pi and a ground station) the clocks differ by an unknown offset and drift: a clock mapping with a stated uncertainty is required before an age is meaningful, and none exists yet. Ages wrap with the 32-bit clock (`sat_teleop_policy::stamp::age_ms`).

The ground's freshness output depends on this too: it shows video as live only when the newest frame's age plus the clock bound is below the stale threshold. The bound is the sum of both machines' bounds (`sat_teleop_policy::clock`): each is chrony's error bound from `chronyc -c tracking` (the larger of the system time offset and the last offset, plus the root dispersion and half the root delay), rounded up and grown by 500 ppm of its age and counted only while under 15 s old; the vehicle reports its own to the ground every 2 s over the control endpoint, and 10 ms are added once. The testbed's containers share one clock, so each side's bound is 0 (`clock = "shared"`). Without a fresh bound on both sides the output shows every frame as `AgeUnknown`, never Live. The bound can be injected for tests (`POST /clock/inject/<ms|missing|off>` on the vehicle with debug hooks on): `lab check real-mode` shows the ground following it.

## Stamp layout

Owned by `sat-teleop-policy::stamp` (details in `crates/policy/AGENTS.md`):

- Payload, 9 bytes: `frame_id: u32` big endian, `capture_ms: u32` big endian, CRC-8/I-432-1 (polynomial 0x07, init 0x00, xorout 0x55) over the first 8, so a uniform stamp area (all bits 0 or all 1) never validates. 72 bits, MSB first.
- Drawn into the luma plane of I420 frames as 24 columns x 3 rows of square blocks at the top-left, row-major. Block side `width / 24`, at least 8 px; frames narrower than 192 px or shorter than 3 blocks are rejected. Bit 1 is luma 235, bit 0 is luma 16.
- Read back from the centred half-size square of each block (mean above 128 is 1). A CRC mismatch makes the stamp unreadable.
- The ground's freshness output adds a state stamp as a fourth row of 24 blocks, the same size, right below: 4 bits the state (1 Live, 2 Stale, 3 Frozen, 4 Disconnected, 5 AgeUnknown, 6 LowRate), 16 bits the newest frame's age in 10 ms units (all ones when unknown or above 655 s), 4 bits CRC-4/INTERLAKEN over those 20. The stamp itself passes through decoding and re-encoding unchanged.

Frame ids increase by one per stamped frame for the life of the vehicle process, across pipeline restarts.

## Skeleton check

`lab check skeleton --profile <name>` runs in `checks`:

1. Apply the profile through the router agent (`profile_applied`).
2. Wait until the vehicle's ladder settles, as the adaptation check does (`ladder_settled`), so the window measures the path at a steady rung. Then start the headless receiver on `rtsp://192.168.232.20:8554/live` and wait the warm-up (default 5 s) so it has connected and decoded a keyframe.
3. Read router counters and the vehicle and relay metrics, measure for the duration (default 30 s), read them again.
4. Cut router forwarding. Frames already queued at the router still arrive for a while, so the check waits a drain time (the largest frame age seen in the window plus 0.5 s, at most 10 s), then requires 2 s with no frame at all (`forwarding_cut`).
5. Restore forwarding and wait up to 15 s for a fresh frame: a readable stamp, captured after the cut began, received after the restore (`video_recovery`).

If there was no video before the cut, both cut assertions are `incomplete`: silence from a dead stream proves nothing.

The receiver restarts after a pipeline error (one second apart), so an RTSP drop during the cut does not end the run; `receiver_runs` counts its starts up to the end of the measurement window.

### Metrics

Window metrics use observations received inside the measurement window. A new frame is an observation with a readable stamp, a plausible age (0 to 60 s; anything else is a misread stamp or a clock fault) and a frame id not seen before in the window. Only new frames count towards age and cadence; a repeated display of the same frame is not fresh video.

| Metric | Definition |
|---|---|
| `frame_age_p50_ms`, `frame_age_p95_ms`, `frame_age_max_ms` | Nearest-rank percentiles of the age of new frames. |
| `new_frames_per_s` | New frames divided by the window length. |
| `max_frame_gap_ms` | Longest wait for a new frame, counted from the window start to the first frame and from the last frame to the window end, so silence shows as one long gap. |
| `unreadable_stamps`, `unreadable_ratio` | Observations whose stamp does not read back, and their share of all observations. |
| `implausible_stamps` | Readable stamps with an age outside 0-60 s: a corrupt read that passed the 8-bit CRC by chance (about 1 in 256), or a clock fault. |
| `repeated_frames` | Observations of a frame id already seen. |
| `skipped_frame_ids` | Ids missing between consecutive new frames: frames lost anywhere between the stamp and the decoder (encoder queue, network, jitter buffers, decoder). |
| `uplink_bytes_per_s` | Router `tx_bytes` increase on the uplink interface (towards `ground_net`) per second: on-wire bytes including WireGuard, UDP, IP and Ethernet headers and keepalives. |
| `uplink_drops` | Packets dropped by the uplink qdiscs during the window. |
| `vehicle_frames_per_s` | Frames the vehicle's encoder produced per second (vehicle metrics). |
| `relay_rtp_bytes_per_s`, `relay_lost_packets` | RTP bytes per second received by the relay, and packets its jitter buffer declared lost (relay metrics). |
| `cut_drain_ms` | The drain time allowed after the cut. |
| `cut_last_frame_ms` | When the last frame arrived after the cut, relative to it: how long queued video kept coming. |
| `video_recovery_ms` | From the restore to the first fresh frame. |
| `recovery_frame_age_ms` | Age of that first fresh frame. |
| `receiver_runs`, `observations_dropped` | Receiver pipeline starts up to the end of the window (before the cut), and observations not stored because the in-memory cap (250 000) was reached. |

Rates are only computed when both readings of a source succeeded; a failed reading leaves the metric out (and an enforced budget on it is `incomplete`).

## MAVLink check

`lab check mavlink --profile <name>` runs in `checks` as a ground station (system 255, component 190, like QGC). The ground adapter sends it telemetry on 192.168.232.40:14552 from a socket of its own; it answers that socket.

1. Apply the profile and wait up to 60 s for a vehicle heartbeat (`telemetry_received`). The adapter's address is learned from the first datagram; from then on a GCS heartbeat goes out every second.
2. Read the vehicle and ground metrics, then for the duration (default 30 s) send `COMMAND_LONG` `REQUEST_MESSAGE(AUTOPILOT_VERSION)` to system 1 every 2 s, one at a time, each waiting up to 5 s for its `COMMAND_ACK` and the `AUTOPILOT_VERSION` message. Read the metrics again.
3. Send three forced `ARM_DISARM` commands 1 s apart and watch 4 s more (`arm_refused`): the adapter holds no lease, so it answers them `DENIED` itself.
4. Ping 10.77.0.2 over the overlay, then try tcp 5760, 5762 and 5763 there (`sitl_ports_closed`).
5. Read for a second (what queued meanwhile, and fresh telemetry), cut router forwarding, wait the drain time (the largest acknowledgement round trip plus 0.5 s, at most 10 s), then require 3 s with no vehicle message (`forwarding_cut`). The vehicle must have been heard between the start of that second and the end of the drain, or the silence is `incomplete`. Restore forwarding and wait up to 25 s for one (`telemetry_recovery`).

Every received frame is timestamped when the check reads it, so phases that block reading (the HTTP readings, the port probes) are followed by a read of the queued datagrams before anything is timed against them.

| Metric | Definition |
|---|---|
| `heartbeats_per_s`, `heartbeat_gap_max_ms` | Vehicle heartbeats in the window per second, and the longest wait for one, counted from the window start and up to its end. |
| `request_ack_rtt_p50_ms`, `request_ack_rtt_max_ms` | From sending `REQUEST_MESSAGE` to the first `COMMAND_ACK` for it from the vehicle: the round trip through adapter, mavlink-router, both router directions, the vehicle gate and SITL. |
| `requested_message_p50_ms`, `requested_message_max_ms` | From the request to the first `AUTOPILOT_VERSION`. ArduPilot sends it right before the acknowledgement. |
| `requests_unanswered` | Requests without both answers within 5 s. |
| `received_bytes_per_s` | Frame bytes from the vehicle that reached `checks` per second. |
| `telemetry_bytes_per_s` | Frame bytes the vehicle sent to the relay per second (vehicle metrics): the telemetry on the uplink, without UDP, IP and WireGuard headers. |
| `sitl_bytes_per_s` | Bytes the vehicle read from SITL per second, including anything that formed no frame of the dialect (not forwarded). |
| `telemetry_rate_limited`, `telemetry_expired` | Frames the vehicle dropped over budget, and replies that waited too long, during the window. |
| `arm_refused` | Increase of the adapter's `denied` count over the arm attempts. |
| `cut_drain_ms`, `cut_last_message_ms` | The drain time allowed after the cut, and when the last vehicle message arrived after it. |
| `telemetry_recovery_ms` | From the restore to the first vehicle message. SITL's default streams send once a second, so this includes up to a second of waiting for the next one. |
| `messages_dropped`, `undecodable` | Messages not stored because the in-memory cap (500 000) was reached, and CRC-valid frames the dialect could not decode. |

## Control freshness check

`lab check control-freshness --profile <name>` runs in `checks`, through `just lab-check` (`docs/testbed.md`). Its observer reads SITL's telemetry straight from mavlink-router on a thread of its own, so every message is timed when it arrives. Speed is `GPS_RAW_INT`'s ground speed (independent of the EKF), throttle is `VFR_HUD`'s, both asked for at 10 Hz for the run.

1. Apply the profile; wait up to 60 s for a vehicle heartbeat through the adapter and from mavlink-router. Force-disarm SITL through the adapter, set the 10 Hz rates, free the lease.
2. Second controller: an envelope without a lease, then take the lease and send, between valid envelopes (a liveness one before each case, so the lease never lapses), envelopes with an old epoch, a duplicate sequence number (full throttle with the number of a neutral setpoint that landed), a reordered one (a forced arm with a discrete number set aside before a valid `REQUEST_MESSAGE` that was applied), a forced arm reusing that request's transaction id, a challenge older than its lifetime, one padded past 1200 bytes, one unsigned, one signed with another key, one signed for another vehicle, one signed over a neutral setpoint whose frame was swapped for full throttle, a release of the held lease with no signature and one with another key's; release, let the adapter take the lease once the vehicle shows the release landed, ask again (`one_lease`), send one more as the former holder. Every invalid envelope carries a forced arm or full throttle (`invalid_refused`). The vehicle's applied counts, read while `checks` still held the lease, must have risen by exactly the valid envelopes.
3. Through the adapter: `DO_SET_MODE` manual, then arm and disarm three times, each waiting up to 5 s for SITL's acknowledgement (`discrete_acked`). Resend the last disarm with `confirmation` 1, as after a lost acknowledgement: the vehicle answers it from its transaction cache without applying it again and the adapter acknowledges it `ACCEPTED` (`resend_not_reapplied`). Then manual, arm, stream throttle for 2 s and send `DO_SET_MODE` Hold amid the stream; SITL must accept it (`stop_command_applied`); force-disarm.
4. Without a blackout in the profile: manual, arm, `MANUAL_CONTROL` throttle 20 % at 25 Hz for at least 5 s, sampling the vehicle's `last_continuous` every 200 ms; print `lab: kill ground`, keep streaming, and wait for the vehicle's applied setpoints to stop for 1 s, then watch SITL for up to 12 s (zero throttle, Hold, standstill); then the restarted adapter's lease. With a blackout (`outage-10s`): start the timeline with the adapter holding the lease, send a forced arm 1 s into the blackout and stream throttle until 6 s after the restore, then arm before and after taking the lease again.
5. Force-disarm, default rates, free the lease.

| Metric | Definition |
|---|---|
| `invalid_applied` | Discrete and continuous envelopes the vehicle applied during step 2 (its `applied` counts). Must be 0. |
| `stop_command_ack_ms` | The acknowledgement time of the Hold sent amid the setpoint stream. |
| `discrete_ack_p50_ms`, `discrete_ack_max_ms` | From sending a command to the adapter to its `COMMAND_ACK` from SITL: adapter, envelope over the overlay (downlink), the vehicle's checks, SITL, and the acknowledgement back through the telemetry (uplink, mavlink-router, adapter). |
| `setpoint_delay_p50_ms`, `setpoint_delay_max_ms` | For sampled continuous envelopes, the vehicle's write to SITL minus the envelope's `sent_unix_ms` (the adapter's clock; the containers share one): the adapter, the overlay downlink and the vehicle's checks. `checks` to the adapter is local and not included. |
| `setpoint_apply_max_ms` | The vehicle's write to SITL minus its arrival: the vehicle's own processing. |
| `top_speed_m_s` | Fastest GPS ground speed while driving. |
| `refresh_last_ms` | The vehicle's last refresh after its last setpoint: how long it kept the setpoint alive. Asserted (`refresh_bounded`) to be at most the setpoint lifetime plus one 20 ms tick. |
| `throttle_released_ms` | First `VFR_HUD` with zero throttle after the last setpoint: SITL no longer drives (setpoint lifetime, `RC_OVERRIDE_TIME`, throttle slew, telemetry delay). Graded against `throttle_release_deadline_ms` in `testbed/limits.toml`. |
| `hold_ms` | First heartbeat in Hold after the last setpoint: SITL's GCS failsafe. |
| `speed_zero_ms` | First GPS ground speed at or below 0.2 m/s after the last setpoint: SITL's rover coasting without brakes. Recorded, not graded. |
| `outage_applied` | Discrete and continuous envelopes the vehicle applied from 1 s before the blackout to 6 s after the restore. Must be 0. |
| `reacquired_arm_ack_ms` | The acknowledgement time of the first arm after taking the lease again. |

`timeseries.csv` gets `discrete_ack` per command (`seq` its index), `setpoint_delay` per sampled setpoint and, from the kill on, `ground_speed` (m/s) and `throttle` (%) per message.

Signing cost. Every envelope is signed (Ed25519) and checked before anything else. On an Apple M-series host a release build signs one in 12.8 us and verifies one in 27.7 us (a throwaway loop, 5000 envelopes). In the testbed's vehicle container the vehicle's own counters (`verified`, `verify_us_total`, `verify_us_max` under `mavlink.control`) read 22 checks, 2405 us in all, at most 197 us, in run 1793605335370. Control freshness with signing on, `lan` (runs 1793604791964 and 1793605335370, against the budget's derivation runs 1793508728511 and others before signing): discrete ACK p50 20.2 and 17.5 ms (20.2 ms before, budget 40), setpoint delay p50 1 ms (1 before, budget 15), stop command ACK 16.6 and 17.4 ms (budget 35), nothing applied from invalid envelopes; every budget assertion passed. Real mode adds a parameter read every 10 s per required parameter, one `PARAM_REQUEST_READ` each, and a clock query every 2 s, about 60 bytes each way.

## Freshness output

The ground adapter decodes MediaMTX's stream and sends every frame on to QGC with its freshness shown (`docs/qgc.md`). Per decoded frame it reads the stamp: a frame is new when its capture time is later than any before and its age at decoding is 0 to 60 s; a stamp that does not read back, or an implausible one, counts as unreadable. Each frame it sends gets a state from `sat_teleop_policy::freshness::classify`, judged in this order:

| State | When | `[freshness]` |
|---|---|---|
| `Disconnected` | The pipeline pulling from MediaMTX is down, or no frame at all was decoded for this long. | `disconnected_after_ms = 2000` |
| `AgeUnknown` | Fewer than this share of the frames that arrived in the last second had a readable stamp: a window of time, so the share means the same at every frame rate. | `min_readable_percent = 50` |
| `Frozen` | Frames keep arriving (one within the threshold, at a low rate the threshold plus the frame interval) but none carried a new stamp for longer than this. A V4L2 camera that repeats its frame lands here (`docs/pi.md`, "A stuck camera"); on any other source this catches only a repeated stamp: the hook, or a path stuck between the stamp and the ground. | `frozen_after_ms = 500` |
| `AgeUnknown` | No clock bound configured. | |
| `Stale` | The newest new frame's age now, plus the clock bound, exceeds this; at a low rate (below), the low-rate threshold instead. | `stale_after_ms = 600`, `low_rate_stale_after_ms = 1800` |
| `LowRate` | The source's frame interval, the lower median of the capture intervals between new frames over the second before the newest (by their stamps), is longer than this: the picture is current, at a rate too low for the stale threshold, and says so (`LOW RATE` in amber). | `low_rate_interval_ms = 150` |
| `Live` | Otherwise. | |

Where the thresholds come from:

- `stale_after_ms = 600`: under `geo` frames arrive 321-322 ms old at the median, 333-337 ms at p95 and at most 351 ms (skeleton runs), and without loss the next new frame follows within 99 ms, so live video is at most about 450 ms old when the next frame replaces it; 600 ms leaves a third on top. Once the source stops, Stale shows when the last frame turns 0.6 s old, so an operator never watches older video without being told, and under `geo` that is about 0.2 s after the next frame was due.
- `frozen_after_ms = 500`: new frames come every 67 ms at 15 fps, at most 105 ms apart without loss (skeleton runs). Loss breaks the video until the next keyframe, and the decoder then conceals it with an earlier picture; the output does not send a picture older than the newest it has (it would carry the newest frame's state), so the newest picture stays on screen and turns Frozen or Stale, which it is.
- `disconnected_after_ms = 2000`: well above any wait for a decoded frame seen with the source running (the longest wait for a new frame under `geo`, with loss, was 1272 ms), so No video means the source is gone; Stale has said so since 0.6 s.
- `min_readable_percent = 50`: `geo` runs lost up to 8.4 % of their stamps over a run; below half a second's worth, the age is a guess.
- `low_rate_interval_ms = 150`: a picture stays below the 600 ms stale threshold until its next frame only if that frame follows within 600 ms minus its arrival age, about 250 ms under `geo` (frames arrive up to about 350 ms old), less on a slow link where a frame takes longer to cross. At two frames a second under `geo` the picture is 330-830 ms old between frames and would flicker between Live and Stale with every frame. Frames more than 150 ms apart (below about 7 fps) are therefore a low rate: the source's own interval, read from the capture times and frame ids of the stamps, not the network's timing.
- `low_rate_stale_after_ms = 1800`: at one frame a second a picture gets up to 1000 ms plus its arrival age old before the next frame, about 1350 ms under `geo`; 450 ms more covers jitter and a slow link's serialization. It lies below `disconnected_after_ms`, so Stale still shows before No video: a frame missing at one frame a second shows Stale, a stalled source No video after 2 s.

The stale and frozen thresholds lie below the disconnect one, so Stale or Frozen shows before No video. A blackout (nothing arrives) never shows Frozen: frozen needs frames arriving.

Latency: the output is event-driven. A newly decoded frame is copied, gets its state stamp and overlay, and goes to the encoder at once; only when nothing went out for two output periods (2/15 s), or when the state may change sooner (`sat_teleop_policy::freshness::next_change_ms`: the newest frame turning `stale_after_ms` old, `frozen_after_ms` since the newest new frame, `disconnected_after_ms` since the last frame), is the last one sent again with its state updated. So the output never resamples the source to a fixed clock, a healthy source whose frames arrive a few milliseconds late never meets a repeat in the encoder, and each change of state shows on time. A frame whose state was current when it went to the encoder is dropped, and a keyframe requested, if the encoder and sender hold it past the moment that state may change (the ground's `late` counter): a stall there costs a gap, never an old picture labelled Live. The pull from MediaMTX is RTSP over TCP inside the testbed with no jitter buffer wait (`rtspsrc latency=0`): TCP loses and reorders nothing, so in-order packets leave rtpjitterbuffer as they arrive. MediaMTX forwards each RTP packet to its readers as it arrives, through a per-reader write queue (`writeQueueSize`, 512 packets by default), without holding frames back. The decoder uses slice threads only (frame threads hold frames back); the encoder is x264 `tune=zerolatency speed-preset=ultrafast`, no B-frames, two slice threads; the only queue is the encoder input's, one frame, leaky. `output_added_ms` below measures what all of that adds; QGC's direct RTSP path skips it, without any indication.

## QGC status check

`lab check qgc-status --profile <name>` runs in `checks`. It plays the adapter's output on 192.168.232.40:5602 the way QGC plays a MAVLink camera's stream in low latency mode (`udpsrc` with RTP H.264 caps, no jitter buffer, `avdec_h264`) and reads each frame's stamp and state stamp.

1. Apply the profile, resume the vehicle's video (an aborted run may have left it paused or frozen), wait until the vehicle's ladder settles (`ladder_settled`, as in the adaptation check) and start the receiver.
2. Camera: as a ground station on 192.168.232.40:14552, wait up to 60 s for the camera's heartbeat (system 1, component 100), then send `REQUEST_MESSAGE` for `CAMERA_INFORMATION`, `VIDEO_STREAM_INFORMATION` (stream 0, all) and `VIDEO_STREAM_STATUS` (stream 1) to component 100, as QGC 5.1.5 does. All three must be acknowledged `ACCEPTED` by component 100 with the messages (a video stream, RTP/UDP, nothing thermal), and the vehicle must have received no message from the ground meanwhile, up to 1 s after the answers, since a leaked request would reach it one downlink later (its `mavlink.gcs.messages_in`) (`camera_answered`).
3. Wait out the warm-up (default 5 s) and up to 30 s more for a frame with a state stamp (`output_received`).
4. Healthy window (the duration, default 30 s), with the adapter's metrics read before and after (`healthy_live`: at least one frame read Live; the share is graded by the budget).
5. Pause: `POST /video/pause` on the vehicle's test-only hook, which drops frames before the vehicle's encoder; watch until a frame shows Disconnected (at most the threshold plus 5 s) and 1 s more, then `POST /video/resume` and wait up to 15 s for Live (`pause_indicated`: Stale shows, then Disconnected, and no frame reads Live after the first one that read Stale, Frozen or Disconnected; `pause_recovered`: a current picture). Here and below LowRate counts with Live for the indications and the recovery: it says the picture is current, at a low frame rate; the time to Live itself is recorded apart.
6. Freeze: `POST /video/freeze`, which makes the vehicle send its next frame again and again, stamp and all; the same with Frozen (`freeze_indicated`, `freeze_recovered`). Stale may show before Frozen when the frame turns 0.6 s old first (under `geo`).
7. With a profile whose timeline blacks out both directions (`outage-10s`): start the timeline and watch the blackout (at 10 s) and the restore (at 20 s) the same way, timed from the planned transitions (`blackout_indicated`, `blackout_recovered`). This one takes MAVLink down too.
8. Over the whole run, no frame may read Live while its stamp, as received here, is older than `stale_after_ms` plus 100 ms for the output path after the adapter's decision, nor LowRate while it is older than `low_rate_stale_after_ms` plus 100 ms (`never_falsely_live`).

Why the vehicle hook pauses the video: a router blackout of the uplink would take MAVLink (and the lease) down with the video, and dropping RTP at the relay would break the encoder's reference chain, so the stream would only recover at the next keyframe, and the relay has no control endpoint. Dropping frames at the vehicle's stamp probe, before the encoder, changes nothing else: MAVLink, WireGuard, the relay, MediaMTX and every RTSP session stay up, no frame id is skipped, and the stream resumes with the next frame. The hooks are described in `docs/testbed.md`, "Live video path".

| Metric | Definition |
|---|---|
| `output_frames_per_s`, `output_new_frames_per_s` | Frames received from the output in the healthy window per second, and those with a new stamp. Repeats (nothing new for two output periods, or a change of state due) count in the first. |
| `live_ratio`, `state_unreadable` | Share of the window's frames whose state stamp read Live, and frames whose state stamp did not read back. |
| `output_age_p50_ms`, `output_age_p95_ms`, `output_age_max_ms`, `output_age_mean_ms` | Age of the new frames when they arrived here, on `ground_net`: the output's frame age. QGC on the Mac adds gvproxy's hop and its own rendering. |
| `decode_age_mean_ms` | Mean age of the new frames when the adapter decoded them (its `decode_age_ms_total` and `new_frames` over the window). |
| `output_added_ms` | `output_age_mean_ms` minus `decode_age_mean_ms`: the copy and state stamp, the overlay, the encoder, RTP, the local hop and this receiver's decoding; the adapter's own decoding is in both means and drops out. It is a proxy for re-encoding against passthrough: on the direct path a station decodes once, and the adapter's decode stands in for that one. Frames the output's one-frame leaky queue drops never reach `checks`, and they are the ones that waited longest, so it reads slightly low. |
| `pause_stale_ms`, `pause_disconnected_ms`, `freeze_frozen_ms`, `blackout_stale_ms`, `blackout_disconnected_ms` | From the stall (the hook request, or the planned blackout) to the first frame showing that state. |
| `pause_recovery_ms`, `freeze_recovery_ms`, `blackout_recovery_ms` | From the release (the resume request, or the planned restore) to the first Live or LowRate frame. |
| `pause_live_ms`, `freeze_live_ms`, `blackout_live_ms` | From the release to the first Live frame: after a blackout the vehicle comes back at the floor's low rate and returns to its rung before 2 s later. |
| `pause_gap_max_ms`, `freeze_gap_max_ms`, `blackout_gap_max_ms` | Longest wait for an output frame during the stall: the output keeps coming. |
| `live_age_max_ms` | Oldest frame, by its own stamp as received here, that read Live in the whole run. |
| `observations_dropped` | Frames not stored because the in-memory cap (250 000) was reached. |

## Adaptation check

`lab check adaptation --profile <name>` runs in `checks`:

1. Apply the profile, resume the vehicle's video and wait until the vehicle's ladder settles: no rung change for 15 s (longer than the 10 s dwell) at the top rung (the skeleton and QGC status checks, which only leave the top rung by a fall, wait 6 s: `stale_after_ms` plus `min_change_interval_ms` plus a poll) when the profile has a timeline (it starts from a baseline that carries it), anywhere without one, at most 120 s (`ladder_settled`). A profile that changes the base delay, or the forwarding cut of the check before, sets the ladder moving for a while, and a capacity measured before holds its climb.
2. Start the headless receiver on the relay's RTSP stream, as the skeleton check does, and wait the warm-up.
3. Second viewer: the vehicle's offered uplink over 6 s with one receiver, over 6 s with a second one playing as well, and over 6 s with one again (`second_viewer`, `viewer_change_percent`: the second window against the mean of the other two, which cancels the encoder's own drift in the half minute after a rung change; the budget allows 10 %). The offered uplink is the router's bytes received from `vehicle_net`: everything the vehicle sends, on the wire, before the router's queue and rate limit.
4. The base frame age: the median of the last 5 s.
5. Start the profile's timeline, if it has one, and observe until 20 s after its last step (80 s for `bandwidth-steps`), otherwise for `--duration-s`. Meanwhile read the vehicle's metrics every 0.5 s and the router's counters every second. Each step's time is when the router applied it (`GET /timeline`).
6. For each downward step of the uplink rate (512 to 128, 128 to 64 kbit/s): the offered uplink, in 2 s windows sliding by 1 s that begin after the step, must end the phase below the new cap (`step_<n>_offered_below_cap`), and the frame age must come back to the phase's own level (`step_<n>_age_recovered`). A 2 s window holds one keyframe interval and two of SITL's once-a-second telemetry bursts, so every window carries the same bursts wherever it starts; 1 s windows alternated by a keyframe's worth at a rung near the cap. Each router reading is timed when it is read, since the request around it takes a varying while.

| Metric | Definition |
|---|---|
| `ladder_settle_ms`, `settled_rung` | How long step 1 waited, and the rung the ladder settled at (the skeleton and QGC status checks record them too). |
| `viewer_one_bytes_per_s`, `viewer_two_bytes_per_s`, `viewer_again_bytes_per_s` | The offered uplink with one receiver, with two, and with one again. |
| `viewer_change_percent` | How far the offered uplink with two receivers lies from the mean with one before and after, in per cent of that mean. |
| `base_age_ms` | The median frame age of the 5 s before the timeline. |
| `frame_age_p50_ms`, `frame_age_p95_ms`, `frame_age_max_ms`, `new_frames_per_s` | Over the observation, new frames counted as for the skeleton check. |
| `settle_max_ms` | The longest, over the downward steps, from the step to the end of the last 2 s window in which the vehicle offered more than the new cap; 0 when none did. |
| `over_cap_windows` | 2 s windows above the cap that began more than the settling deadline (6 s) after a downward step and before the next step. |
| `age_over_base_max_ms` | The largest, over the downward steps, of the median frame age in the last 5 s before the next step, minus the base: what is left of a backlog. Serialization at the lower rate counts in it too. |
| `age_recovery_max_ms` | The longest, over the downward steps, from the step to the last new frame that came late: more than 150 ms older than its phase's final median, or after a wait for a new frame more than 150 ms longer than the phase's final median wait (a drain pauses the video, so its queue shows as a wait, not as old frames): how long the queue took to drain. |
| `transitions`, `transitions_per_min` | The vehicle's rung changes during the observation, drains included. |
| `drains`, `drain_max_ms` | Drains the vehicle began during the observation (not among the changes it listed at the first reading), and the longest from a drain to the change after it (the resume, or a fall to the floor), on the vehicle's monotonic clock; 0 when none did. |
| `time_normal_s`, `time_constrained_s`, `time_awareness_only_s` | Time spent at each rung group, from the vehicle's readings. |
| `apply_max_ms` | The longest from a requested rung to the first frame stamped in it (the vehicle's `after_request_ms`, on its monotonic clock), as seen in the readings; a drain's pause counts, since no frame is stamped while the video is paused. |

The settling deadline of 6 s: the vehicle sees a rising queue about a round trip plus two probe intervals after a step (about 1.1-1.7 s under `geo`); a first cut that lands a rung short is followed by a second one at least 2 s later; and a 2 s window that ends after the cut still holds up to 2 s from before it.

The 150 ms margin of `step_<n>_age_recovered` is the controller's own queue threshold: older than that, a frame waited in a queue. The phase's final level, not the base, is the reference, because a lower rate also serializes each frame more slowly; `age_over_base_max_ms` shows how much that is.

## Recording check

`lab check recording --profile lan` runs in `checks`, which mounts the vehicle's snapshot volume and reads the store's limits from the vehicle's config (`docs/testbed.md`, "Recording check"):

1. Apply the profile, wait for the ladder to settle and start the receiver, as the skeleton check does.
2. Wait for the store to write snapshots and to reach its quota (the vehicle's `retained.store.evicted` above 0), then watch it for 20 s: the bytes in the directory and the store's own count against `max_bytes` (`quota_held`, `store_over_quota_bytes`), the oldest snapshot only ever getting newer, and the median interval between retained snapshots (`snapshot_cadence`, `snapshot_interval_median_ms`). The frame age of this stretch is the healthy level.
3. Plant a half-written `.tmp` and two names that are no snapshots in the directory, restart `companion` (the check prints `lab: restart companion`, `just lab-check` restarts it) and wait for the new process to answer: the `.tmp` deleted, the other names left alone and counted `ignored`, `recovery.kept` equal to the snapshots on disk within two, and a snapshot written again (`restart_recovery`; `restart_resume_ms` from the request to the first new snapshot).
4. Make every snapshot write take 3 s (a vehicle hook), hold it for 20 s and lift it: the sender must have dropped snapshots (`snapshots.dropped` rose, `slow_dropped`) while the live frames kept coming (`slow_encoded_per_s`, the receiver's `slow_new_frames_per_s` and `slow_frame_age_p95_ms`, against `healthy_frame_age_p95_ms` as `slow_age_p95_delta_ms`), and writing must resume (`slow_storage_drops`, `slow_storage_recovers`, `live_unaffected`). The vehicle restart before it numbers frames from 0 again, so the frames before and after the restart are counted as new separately.
5. The slow writes left snapshots 3 s apart in the directory: the store must report the same gaps (`gap_reported`).

## Retrieval check

`lab check retrieval --profile <geo|bandwidth-steps|outage-10s>` runs in `checks`, which plays the ground against the relay's job API and the stream's viewer (`docs/testbed.md`, "Retrieval check"):

1. Apply the profile, wait for the ladder to settle and for 12 retained snapshots.
2. A crop (the middle half) and a snapshot of the frame captured 8 s ago are requested and fetched. The retained file the vehicle's store holds for that time is read from the volume first; the object must be that frame (capture time and id), its size, the rectangle `crop::pixel_rect` gives, a JPEG of that size, and for the snapshot the file's bytes (`crop_matches_retained`, `snapshot_matches_retained`; `crop_completion_s`, `crop_bytes`, `snapshot_completion_s`, `snapshot_bytes`, each from the request to the object).
3. The hostile requests (`hostile_requests_refused`; `hostile_requests`, `hostile_accepted`).
4. A snapshot is cancelled once two of its chunks have been sent: `cancel_stop_ms` from the cancel to the vehicle noticing (its `jobs_cancelled` rising), and the relay's chunk count unchanged for 4 s after (`cancel_stops_transfer`).
5. Two passes under the profile, each starting with the profile applied again (which stops a timeline and restores the base rate) and the ladder settled at the top rung for 15 s: one without retrieval, one with crop jobs for the frame 8 s ago, one after the other (`age_p50_without_ms`, `age_p95_without_ms`, `age_p50_with_ms`, `age_p95_with_ms` over the pass, and `age_p50_delta_ms`, `age_p95_delta_ms` their differences). A pass is `--duration-s` under `geo`, and under a profile with a timeline from its start until 20 s after its last step. What retrieval cost: the vehicle's `retrieval.wire_bytes` over the pass as `retrieval_wire_bytes_per_s` and `retrieval_over_allowance_bytes` (above the allowance times the pass plus one second, the bucket's first), `retrieval_jobs_done`, `retrieval_chunks`, `retrieval_holds` (times it began to hold back), `retrieval_held_s` (the time it spent held back), `job_completion_p50_s`, and the router's view of it, `offered_uplink_delta_bytes_per_s`: what the vehicle offered the uplink with retrieval minus without. That delta carries the video's own variation between two passes, so it is recorded, not graded.

Under a timeline the vehicle's retrieval state is read every 100 ms, and `retrieval_yields` requires that it was held back at some point and that no chunk went out between two readings that both show it held (`retrieval_sent_while_held`).

The allowance (`allowance_bytes_per_s` in `[retrieval]` of `testbed/limits.toml`) is wire bytes per second the vehicle may spend on retrieval. Its derivation is under "Retained detail" in the measured numbers below.

## Result files

Every run writes `results/<unix-ms>-<scenario>-<profile>/`:

- `summary.json`: `run_id`, `scenario`, `profile`, `verdict`, `git_revision`, `assertions` (name, verdict, detail), `metrics`, `sample_counts`, `thresholds` (the budget used, or `null`), `effective_profile` (what the router agent applied), and for the skeleton check `settings` (receiver URL, protocols, latency, decoder, warm-up, duration, cut timings, metrics URLs) and `readings` (router counters, both apps' metrics and the receiver's health at the window start and end, and the receiver's health when the run finished). The MAVLink check records its settings and the vehicle and ground metrics at the window start and end and around the arm attempts; the control freshness check its settings (addresses and the `[control]` limits) and the vehicle metrics before and after the second controller's phase; the QGC status check its settings (receiver, the `[freshness]` limits, timings), the adapter's metrics at the window start and end and at the finish, and its camera counts before and after the requests.
- `timeseries.csv`: `unix_s,series,seq,value`. Boot: one row per ping reply, `series` the probe, `seq` the ICMP sequence, `value` the RTT in ms. Skeleton: one row per decoded frame over the whole run (warm-up, window, cut and recovery), `series` `frame_age` (new frame, `seq` frame id, `value` age in ms), `repeated`, `implausible` (raw age) or `unreadable` (`value` NaN). MAVLink: `heartbeat_gap` (each vehicle heartbeat, the gap since the previous one in ms) and, per request (`seq` its index, `unix_s` when it was sent), `ack_rtt` and `requested_message` in ms. Control freshness: as listed in its section. Adaptation: `frame_age` per new frame (`seq` its id), `offered` and `cap` per 1 s router window in bytes per second (`seq` the window), and `rung` and `queue_delay` (the vehicle's, in ms) per reading of the vehicle (`seq` the reading). QGC status: one row per frame received from the output over the whole run, `series` the state its state stamp read (`live`, `stale`, `frozen`, `disconnected`, `age_unknown`, or `no_state`), `seq` its frame id (0 without a stamp), `value` its age in ms (NaN without a plausible stamp).

Image ids are not recorded yet.

## Verdicts and budgets

Each assertion is `pass`, `fail` or `incomplete`. A run fails if any assertion fails; otherwise it is incomplete if any assertion is incomplete or there are none; only then does it pass. Exit codes: 0 pass, 1 fail, 2 incomplete. A run that cannot read its profile or budgets, or cannot write its results, is incomplete. Missing evidence is never a pass.

Budgets live in `perf/budgets.toml`, one table per scenario and profile:

```toml
[skeleton.lan]
status = "enforced"            # or "calibration"
rationale = "how the limits were derived, citing run ids"
max = { frame_age_p95_ms = 200.0 }
min = { new_frames_per_s = 14.0 }
```

- `calibration`: no limits allowed; the metrics are recorded and the run is graded `incomplete` (`budget` assertion). This is how every new budget starts.
- `enforced`: at least one limit and a rationale are required. Each limit becomes an assertion `budget.<metric>`; a metric that was not measured is `incomplete`.
- No budget for the profile: `incomplete`.

Limits are never derived from the impairment the run applied; they come from measured runs with an explicit margin, and the rationale names those runs. Today `skeleton.lan`, `skeleton.geo`, `mavlink.lan`, `mavlink.geo`, `control-freshness` under `lan`, `geo` and `outage-10s` and `adaptation.bandwidth-steps`, `recording.lan` and `retrieval.geo` are enforced (limits 1.25 x worst measured + 10 ms for ages and gaps, 0.95 x worst for the frame rate, the keyframe interval plus the worst recovery frame age, times 1.25, for recovery); every other profile is still in calibration. The MAVLink budgets use the same margins for gaps and round trips, 1.25 x worst for the telemetry rate, 0.95 x worst for the heartbeat rate, zero for counts that were zero, and the 1 s stream period plus half the round trip, times 1.25, for recovery; they grade medians, not maxima, because single answers now and then take about 1 s. The control budgets use the same margin for acknowledgement and setpoint delays and for the vehicle's refresh and SITL's throttle release, zero for applications that were zero, and grade medians; SITL's throttle release is graded against `throttle_release_deadline_ms` in `testbed/limits.toml`, the vehicle's last refresh against the setpoint lifetime plus one tick, and the rover's coast to a stop is recorded only. `geo` grades only frame age and recovery: its frame rate, longest gap and unreadable stamps follow the number of reordered packets in a run (see below) and stay recorded but ungraded until the latency task handles reordered packets. `qgc-status` under `lan`, `geo` and `outage-10s` is enforced too: the times to each state, the longest wait for an output frame during a stall and the latency the output adds get 1.25 x worst + 10 ms, and so do the recoveries under `lan`; under `geo` and `outage-10s` a recovery waits for the vehicle's next keyframe whenever a lost packet broke the picture just before the release, so it gets 1.25 x the larger of the worst measured and (the 2 s keyframe interval + 0.6 s for 8 of the last 15 frames to read back again + the worst freeze recovery); the Live share of the healthy window gets 0.95 x worst under `lan` and a floor of 0.5 under `geo` and `outage-10s`, where it follows the lost packets of a run.

## Measured (container-only)

All numbers below come from the compose testbed on the development Mac (Podman machine, 5 CPUs, synthetic profiles, ball test pattern unless noted) with the headless receiver. None is a real link, a Pi or QGC on screen.

### Results summary

The numbers a reader usually wants, each with the runs it comes from. Everything is container-only: no Pi, no real link, no optical measurement. The runs are the `gate-full` series of 2026-11-02 (images built from `dad4064`, with documentation changes uncommitted), `sim-check lan` of the same day, and the earlier runs the later sections list.

| What | `lan` | `geo` | Runs |
|---|---|---|---|
| Frame age at the receiver, p50 / p95 (`skeleton`) | 7 / 11 ms | 320 / 331 ms | 1793592510178, 1793592556861 |
| Frame age at the ground's output, p50 / p95 (`qgc-status`) | 10 / 14 ms | 322 / 336 ms | 1793592717797, 1793592764231 |
| New frames per second at the top rung | 14.99 | 15.03 | the `skeleton` runs above |
| Latency the output adds | 2.9 ms | 3.2 ms | the `qgc-status` runs above |
| Discrete command acknowledged, p50 | 18.3 ms | about 626 ms | 1793592668376 (`control-freshness lan`); `geo` (626.1 ms worst of 3) from runs 1793508778117, 1793508994123, 1793509209562 (`control-freshness geo`, not in `gate-full`) |
| Time from a pause to `STALE`, to `NO VIDEO` | 0.60 s, 2.01 s | 0.55 s, 2.28 s | the `qgc-status` runs above |
| Time from a freeze to `FROZEN` | 0.54 s | 0.88 s | the `qgc-status` runs above |
| Under `outage-10s`, a 10 s blackout: `STALE` after 0.58 s, `NO VIDEO` after 2.31 s, `LIVE` again 4.9 s after the link returns | | | 1793593245534 |
| Under `bandwidth-steps` (uplink 512, 128, 64 kbit/s): oldest frame 1.07 s, backlog gone after 7.4 s, 3.0 rung changes a minute | | | 1793592891254 |
| Retrieval of a crop (7.3 kB) and of a snapshot (15.6 kB) | | 4.0 s, 6.0 s | 1793593098148 |
| Live frame age p50 / p95 with and without retrieval | | 319 / 332 ms for both | 1793593098148 |
| `recording`: snapshots dropped when every write takes 3 s, and the live frame age p95 meanwhile | 11 of 20; 11 ms | | 1793593012375 |
| Simulated copter (`sim-flight`): Brake, Land, on the ground after, share of the video Live | 0.96 s, 2.42 s, 30.3 s, 100 % | | 1793593432330 |
| Same flight's video age p50 / p95, and the label's at the ground | 5 / 9 ms, 5 / 8 ms | | 1793593432330, 1793593562671 (`sim-check lan`) |

One run of the `gate-full` series failed and passed on its second try: `qgc-status outage-10s` run 1793592817054 measured 2503 ms from the freeze to `FROZEN` against a budget of 1110 ms (every other assertion passed), and run 1793593245534 straight after measured 893 ms with all 25 assertions passing. The cause of the slow one is not established. A second full series on the final tree passed all 12 checks in 727 s: runs 1793593817704 (`boot lan`), 1793593831747, 1793593878238, 1793593940254, 1793593989822, 1793594038959, 1793594085302, 1793594137900, 1793594210714, 1793594331899 and 1793594403059 (`retrieval geo`), and `ingest-isolation`.

Control and SITL identification: on the rover (`control-freshness lan`, 1793592668376) and on the copter (`sim-check lan`, 1793593432330) the vehicle's metrics read `mavlink.control.target` `sitl` and `sunk` 0 with `sitl_marker` true after the autopilot answered the probe for `SIM_WIND_SPD`; the copter's vehicle metrics after the check showed 985 continuous, 90 liveness and 11 discrete envelopes applied, none sunk. The sink itself is pinned by a test against a stand-in autopilot (`crates/control`, `vehicle::sitl_identity`).

Not measured: anything on a Raspberry Pi or a real link, the relay's and the ground adapter's CPU and memory per resolution, glass to glass, or the vehicle's CPU per rung (`docs/pi.md`, "What to measure first").

Skeleton runs at the default rung (640x360, 15 fps, 500 kbit/s):

| | `lan` | `geo` |
|---|---|---|
| Runs | 7, listed in `perf/budgets.toml` | 14, listed in `perf/budgets.toml` |
| Frame age p50 / p95 / max | 7-8 / 10-11 / 15-39 ms | 321-322 / 333-337 / 342-351 ms |
| New frames per second | 14.99-15.03 | 13.59-14.99 |
| Longest frame gap | 79-105 ms | 99 ms without a dropped packet, up to 1272 ms with |
| Unreadable stamps | 0 | 0-38 of 450 (ratio up to 0.084) |
| Uplink on the wire | 30.6-31.4 kB/s | 30.5-31.6 kB/s |
| Recovery after a cut | 93-1837 ms | 869-2321 ms |

The `lan` maximum of 39 ms was one stall: three frames at 31-39 ms within 0.3 s while the median stayed at 7 ms, most likely scheduling on the shared VM.

- The ball pattern is cheap to encode: x264 produces about 20 kB/s (160 kbit/s) of RTP against the 500 kbit/s target, so `geo`'s 512 kbit/s uplink is not saturated. A camera image will use the full bitrate.
- The uplink carries about 1.5 times the RTP bytes. `tune=zerolatency` encodes with sliced threads, one slice per thread, and each slice is its own RTP packet: about 6 packets per frame, each with about 100 bytes of RTP, UDP, IP and WireGuard headers on the wire.
- Recovery waits for the next keyframe (every 2 s): P-frames that reference lost data decode as concealed copies of an older frame (counted `repeated`) or as grey or black frames.
- The `geo` profile's 10 ms per-packet jitter reorders packets (so `ordering = "preserve"` does not prevent reordering under jitter). Some packets arrive more than 50 ms behind their neighbours; the relay receives every packet but counts these late and lost (0-20 per 30 s run), and with no loss repair yet each one breaks video until the next keyframe. That is what spreads the frame rate, gaps and unreadable stamps between `geo` runs.

MAVLink runs (`lab check mavlink`, four per profile with the `ardupilotmega` dialect, listed in `perf/budgets.toml`), SITL at its default streams:

| | `lan` | `geo` |
|---|---|---|
| Heartbeats per second, longest gap | 0.995-0.998, 1008-1012 ms | 0.966-0.999 (29 or 30 in the window), 1030-1048 ms |
| `REQUEST_MESSAGE` to `COMMAND_ACK`, p50 / max | 14.2-19.3 / 18-23 ms | 609-619 / 627-657 ms |
| `AUTOPILOT_VERSION` after the request, p50 | 14.2-19.3 ms | 608-618 ms |
| Telemetry sent by the vehicle, received by `checks` | 887-889 B/s, 913-916 B/s | 888-890 B/s, 880-909 B/s |
| Read from SITL | 887-889 B/s | 888-890 B/s |
| Telemetry recovery after a cut | 206-963 ms | 520-825 ms |

- SITL sends every message of its default streams once a second, in one burst, so heartbeat gaps sit at about 1 s and telemetry recovers with the next burst, up to 1 s after the restore plus the one-way delay.
- Everything SITL sends now crosses the vehicle. With the `common` dialect only (six runs per profile on 2ad9c9a), about a fifth of it (887-890 against 702-708 B/s), ArduPilot's own messages, never left the vehicle.
- Every run answered every request and refused all three forced `ARM_DISARM`s with SITL disarmed; no telemetry was dropped at about 890 B/s against the 8000 B/s budget. The budget is exercised by parameter downloads: ArduPilot sends all 1256 parameters (about 46 KB) in about 0.6 s, and the vehicle's reply queue delivers them in 5.5 s (`lan`) and 6.6 s (`geo`) with heartbeats unaffected (`docs/qgc.md`).
- Two single answers took about 1 s in earlier runs (971 ms under `lan` in run 1793494808693, 1011 ms under `geo` in run 1793496569607) while the rest of their runs stayed near the median; most likely SITL stalling under qemu on the shared VM. Maxima are therefore recorded, not graded.

Control freshness runs (`lab check control-freshness`, three per profile on e1d5edb, alternating, listed in `perf/budgets.toml`):

| | `lan` | `geo` | `outage-10s` |
|---|---|---|---|
| Invalid envelopes applied (8 kinds per run) | 0 | 0 | 0 |
| Discrete ACK through the adapter, p50 / max | 15.4-20.2 / 22.6-24.8 ms | 613.5-626.1 / 620.7-645.6 ms | 616.9-621.3 / 624.4-644.0 ms |
| Hold amid a 25 Hz setpoint stream, acknowledged after | 15.1-18.2 ms | 600.7-615.5 ms | 601.6-618.1 ms |
| Continuous setpoint delay, p50 / max | 1 / 2-4 ms | 296-300 / 315-320 ms | |
| Vehicle's last refresh after the last setpoint | 424-427 ms | 422-427 ms | |
| SITL throttle zero after the last setpoint | 1053-1133 ms | 1373-1409 ms | |
| SITL in Hold (GCS failsafe) | 3442-3504 ms | 3792-3818 ms | |
| SITL at 0.2 m/s or less, from 2.5-2.7 m/s (not graded) | 6261-6443 ms | 6672-6817 ms | |
| Applied from what was sent into the blackout | | | 0 |
| First arm after taking the lease again | | | 584-629 ms |

- The setpoint delay is the downlink (300 ms under `geo`) and almost nothing else: the vehicle writes an admitted setpoint within 1 ms of its arrival.
- After the adapter dies, the vehicle's last refresh comes 422-427 ms after the last setpoint (refreshes every 200 ms on a 20 ms tick; the next would fall after the 500 ms setpoint lifetime), SITL's override ends 0.5 s later and `checks` sees zero throttle one uplink later. The rest of the stop is SITL's rover coasting: it has no brakes and loses speed with a 2 s time constant.
- The expired-challenge envelope was refused as `challenge_expired` under `lan` and as `unknown_challenge` under `geo` and `outage-10s`: by the time it arrives over 300 ms more, the vehicle has issued its fourth newer challenge.

QGC status runs (`lab check qgc-status`, three per profile on e72ac9f, alternating, listed in `perf/budgets.toml`), the output at 640x360, 15 fps, 2000 kbit/s:

| | `lan` | `geo` | `outage-10s` |
|---|---|---|---|
| Frame age at the adapter's decoding, mean | 3.0-3.7 ms | 314.4-317.2 ms | 316.1-317.0 ms |
| Output frame age at `checks`, mean / p95 | 5.4-6.8 / 7.0-9.0 ms | 318.1-320.3 / 324.0-332.0 ms | 318.7-320.3 / 329.0-332.0 ms |
| Added by the output | 2.4-3.0 ms | 3.1-3.6 ms | 2.6-3.3 ms |
| Output frames per second, all / new | 15.0 / 15.0 | 15.0-15.5 / 8.8-15.0 | 15.0-15.1 / 14.2-15.0 |
| Live share of the healthy window | 1.00 | 0.55-1.00 | 0.93-1.00 |
| Pause: Stale / No video after | 560-578 / 1959-1985 ms | 220-602 / 2253-2318 ms | 559-581 / 2273-2291 ms |
| Freeze: Frozen after | 529-550 ms | 869-881 ms | 870-880 ms |
| Blackout: Stale / No video after |  |  | 595-600 / 2304-2316 ms |
| Live again after pause / freeze | 33-61 / 27-69 ms | 379-2781 / 323-339 ms | 358-394 / 328-384 ms |
| Live again after the blackout |  |  | 1711-3592 ms |
| Longest wait for an output frame, pause / freeze | 135-136 / 68-71 ms | 135 / 83-91 ms | 135-137 / 85-91 ms |
| Oldest frame shown Live, as received | 506-512 ms | 589-594 ms | 586-601 ms |

- The output adds 2.4-3.0 ms (`lan`) to 3.1-3.6 ms (`geo`) to the frame age: the copy and state stamp, the overlay, x264 and RTP on the ground, the local hop and the receiver's decode. MediaMTX and the TCP pull add little: the adapter decodes frames 3.0-3.7 ms old under `lan`, counting the vehicle's encoder, WireGuard, the relay, MediaMTX, the TCP hop and the decoder.
- While the source is healthy the output carries its 15 frames per second and nothing else: a repeat goes out only when nothing went out for two periods (133 ms) or the state is about to change. With the earlier rule (a repeat after one period, on c12554d) a quarter to a third of the output frames were repeats, sent a few milliseconds before the next real frame, and the added latency was 2.9-3.7 ms.
- During a stall a frame goes out every two periods, and each change of state shows when it is due (thresholds above). Under `geo` the last frame was captured up to one uplink before the request, so Stale can show before Frozen.
- `geo` loses a packet now and then (the relay's jitter buffer drops late ones), and each loss breaks the picture until the next keyframe: the decoder shows concealed frames whose stamps do not read back (Frozen, then AgeUnknown). A run with many losses (1793519290484, 8.8 new frames per second) kept Live for only 0.56 of its healthy window, and when a loss falls just before a pause the recovery waits for the keyframe the pause postponed (2781 ms in run 1793518944364). After a blackout the same happens every time, and Live returns once 8 of the last 15 frames read back again.
- The ground container used 3.0-5.5 % of one core over 20 s windows of these runs (its cgroup's `cpu.stat`, against 4.9-8.2 % with the one-period repeats) and 31-33 MB of memory (cgroup).

Adaptation runs (`lab check adaptation`, listed in `perf/budgets.toml`; `lte-bad` and `geo-degraded` observed for 120 s), the ball pattern. Four `bandwidth-steps` runs, three on cf21fa5 alternating with one run of each other profile and one on 91cfdcf (the vehicle's code is the same; the check counts drains only during the observation, as computed from the vehicle's listed changes for the others). Earlier figures (before the vehicle drained a deep queue with the video paused and sized telemetry to what SITL offers) come from runs on 2bc8bf2 and c5d5ef9.

| | `bandwidth-steps` | `lte-bad` | `geo-degraded` |
|---|---|---|---|
| Runs | 1793548366795, 1793548686388, 1793548986264; 1793549731777 | 1793548504460 | 1793548823097 |
| Video paused while the queue drained after the 512 to 128 kbit/s step | 3.5-4.2 s, once per run | | |
| Offered below the new cap after the step | 2.19-2.23 s | | |
| Oldest new frame of the run; back at its phase's level and pace after | 357-674 ms; 5.4-7.6 s, the pause included (before: frames up to 2.0-3.5 s old, the backlog gone after 6.5-7.8 s) | | |
| Frame age at the phase's end over the base | -6 to +2 ms | | |
| Windows over the cap after the settling deadline | 0 | | |
| Rung changes per minute | 3.0-4.5 | 0 | 4.0 (before 2.0-2.5) |
| Time normal / constrained / awareness only | 16-17 / 20-22 / 42-44 s of 80 | 120 / 0 / 0 s of 120 | 0 / 56 / 64 s of 120 (before 0 / 35-59 / 61-85) |
| Frame age p50 / p95 | 318-319 / 330-331 ms | 86 / 148 ms | 335 / 530 ms |
| New frames per second | 4.6-4.7 | 4.5 | 1.5 (before 0.5-1.0) |
| Second viewer: with two receivers against the mean with one before and after | 4.2-6.1 % | 4.3 % | 1.2 % |

- With the pause at 500 ms of queue (c63a86e to 32205a7) the cut at this step came with less and drained the queue with the video running: run 1793551927349 on caae26f cut without a pause, frames arrived up to 1.7 s old and were back at their phase's level 5.2 s after the step. The pause is back at 300 ms, now at any answer that late (c63d5a1): run 1793552548374 paused once at the step, for 3.2 s, its oldest new frame was 356 ms and no 2 s window lay over the new cap; `lte-bad` (1793552685501, pareto jitter) paused not once in 120 s.
- Under `geo`'s 512 kbit/s the vehicle sat at the top rung, offering 45-49 kB/s of the 64 kB/s. At the step to 128 kbit/s that rung offered about three times the new cap; the cut paused the video, and the queue drained in 3.5-4.2 s. No frame older than 674 ms arrived; before, the queued frames arrived up to 3.5 s old over 6.5-7.8 s, and the relay's jitter buffer broke the picture meanwhile by dropping late packets. The age recovery now counts the pause as a wait, so it reads about as long as before.
- The uplink measured at that cut can reach back to before the queue (25.5 kB/s against the 16 kB/s link in run 1793538848051 on bcec3c6, which then sat at 2 fps over the 64 kbit/s cap); the answers that come back through the queue while the video is paused measure it again, and the video resumes at the rung that fits: 12.3-14.0 kB/s in these runs, and once a step from 2 fps to 1 fps at the resume.
- Telemetry now takes 2.5 times what SITL offers (a cap of 1970-2070 B/s, plus 3.3 kB/s of headers) instead of the configured 8000. At 128 kbit/s that still leaves video only about 3-4 kB/s of the 75 % a cut fits into (12.3-14.0 kB/s measured, less control and telemetry), so the ladder lands on the floor; at twice what SITL offers one telemetry message in 45-49 was dropped (6963bc5), at 2.5 times none was, apart from 57 at the first check after each `lab-up`, whose profile switch reads as a deep queue for about 4 s until the base shift clears the capacity measured at it.
- The 64 kbit/s step found the vehicle at the floor in every run, and the climb back went a rung every 10 s once the capacity's hold ran out.
- `lte-bad` held 640x360 or 1280x720 at 15 fps: its 5 % random loss reads in the hold band over 10 s and 500 packets. Of 15 frames a second only about 4.5 arrived readable: each lost packet breaks the picture until the next keyframe, with no repair yet.
- The slow rungs send about half to two thirds of their nominal bitrate (`docs/architecture.md`, "Adaptation"). At the floor that is about 12.5 of the 25 kbit/s.
- `geo-degraded` cycles from the floor up to 640x360 at 8 fps, a rung every 10-30 s, which overloads the 128 kbit/s and is cut back to the floor, about every 80 s; no cut there was deep enough to drain. It stays in calibration with `lte-bad`.
- After the 10 s blackout of `outage-10s` the output reads Live again 4.9 s after the restore (run 1793549585140 on d8decf7; current, at the floor's low rate, after 0.8 s): 2 s of healthy answers bring back the rung before the outage. Before, the vehicle climbed back a rung every 10 s, about 50 s to the top.
- The second viewer moves the offered uplink by 1.2-6.1 % (more under `bandwidth-steps` and `lte-bad`, less under `geo-degraded`); the vehicle cannot see the receivers (MediaMTX serves them on `ground_net`); the cause of the few per cent was not found, and it stays within the 10 %.
- The climb from 640x360 to 1280x720 under `geo` once broke the picture for a keyframe interval: the relay's jitter buffer gave up on three packets of the first 720p keyframe, which then came late, so the ground showed AgeUnknown for 2 s. This is the reordering of the skeleton `geo` runs above, which the latency task owns.

Relay timing. `rtspclientsink` synced its internal sinks to the ingress pipeline's latency, which turned the relay's jitter buffer latency into a hold on every frame: `lan` p50 was 35, 73 and 225 ms with a relay jitter latency of 0, 50 and 200 ms (runs 1793487491551, 1793487541132, 1793487445892). Worse, after the path delay dropped (`geo` then `lan`, run 1793488338310) the relay kept the old delay for about 8 s, its blocked jitter buffer dropped packets, and only keyframes decoded cleanly: p95 133 ms, max 3744 ms, 11.3 new frames per second. The relay now sets `sync=false` on those sinks; `lan` p50 is 7 ms and a `lan` run straight after `geo` is clean (1793488563825).

Sender timing. `videorate` held every frame until the next one arrived when allowed to duplicate frames, one frame interval (67 ms at 15 fps, 500 ms at 2 fps) before the stamp, so no frame age could show it; GStreamer debug logs showed the hold, and `drop-only=true` removes it.

Stamp survival at low rungs on `lan`, 2 fps and 50 kbit/s unless noted, keyframe every 2 s:

| Run | Source and rung | Encoded RTP | Unreadable |
|---|---|---|---|
| 1793489160747 | ball, 192x108 (8 px blocks) | 1.6 kB/s | 0 of 120 |
| 1793489010077 | `foreman_cif.y4m`, 192x108 | 5.0 kB/s | 0 of 121 |
| 1793489085546 | `foreman_cif.y4m`, 320x180 | 4.3 kB/s | 0 of 121 |
| 1793489248776 | `foreman_cif.y4m`, 192x108, 15 fps | 6.4 kB/s | 0 of 451 |

The clip fills the bitrate where the ball pattern does not, so those runs quantize the stamp hardest; it read back every time.

Before the stamp used CRC-8/I-432-1, its plain CRC-8 (no xorout) accepted the all-zero payload, so a grey or black stamp area read back as frame id 0 at capture time 0: 30 such reads in run 1793488338310, all caught by the 0-60 s age bound. The xorout now rejects them. The bound stays, because an 8-bit CRC still passes about 1 in 256 corrupt reads, such as concealed frames after loss.

### Boot check

Across several runs (development Mac, Podman machine, synthetic profiles): 20 pings from the vehicle to 10.77.0.1 averaged 1.3-1.5 ms under `lan`, 605-607 ms under `geo` (single pings 566-636 ms) and 133 ms under `lte-bad` (16 of 20 answered); `lab check boot` medians were 1.2-3.2 ms (`lan`), 605-613 ms (`geo`), 106-114 ms (`lte-bad`) and 627 ms (`geo-degraded`).

### Camera ingest

The vehicle can take its frames from a camera outside the container (`docs/demo.md`). The ingest adds one H.264 decode before the stamp; the stamp, and so the frame age, starts after it. Per-frame decode time, from probes on the decoder's input and output (`ingest.decode_ms_*` in the vehicle's metrics), 1280x720 on the build container's CPU (Apple Silicon under libkrun, 5 CPUs):

| Source | Runs | Decode mean / max |
|---|---|---|
| Mac camera (FaceTime HD), `lan` | skeleton 1793570042262 on 32c1774 | 1.08-1.12 / 13.8 ms (the maximum is the first frames) |
| Test-pattern feeder, `lan` | ingest 1793569811946, skeleton 1793569832462 | 0.84 / 7.5 ms |
| Test-pattern feeder, `geo` | ingest 1793570183015, skeleton 1793570203500 | 0.96 / 9.6 ms |

Frame age through the link with the ingest as the source (stamp to the headless receiver): `lan` with the camera p50 / p95 / max 6 / 8 / 12 ms (the camera's scene needs 104 kB/s at the top rung against the ball's 47), `lan` with the feeder 3 / 5 / 7 ms, `geo` with the feeder 316 / 330 / 353 ms against 319 / 332 / 342 ms with the built-in test pattern (retained detail, below). The ingest therefore adds about 1 ms and no measurable age under `geo`. The camera's snapshot branch took 60 snapshots, dropped none and left none unidentified in the camera run, with the queue held to one buffer.

The decoder drops a picture broken by loss (`output-corrupt=false`) and decode timings are matched by buffer timestamp, so dropped frames do not shift later pairs. Measured after that change: `lan` feeder ingest 1793572229298 (decode 0.83 / 7.1 ms), skeleton 1793572249810 (age 3 / 5 / 7 ms); `geo` feeder ingest 1793572318862 (0.89 / 9.1 ms), skeleton 1793572339296 (316 / 330 / 338 ms). Neither moved.

What sits before the stamp and is not measured by the age: the camera's capture, and on the host ffmpeg's x264 (`ultrafast`, `zerolatency`, no B-frames, 3 Mbit/s: 1.0 s of CPU for 300 frames of a test source, 3.4 ms a frame, at 48 times real time) and one loopback hop through the Podman machine's gvproxy. Optical glass-to-glass was not measured, so the camera's own latency is unknown.

Raw I420 over RTP instead of H.264 was tried at the transport level and rejected: 3 s of 720p30 (127 MB, about 338 Mbit/s as ffmpeg packs it) through the published port delivered about 97 % of the datagrams (102 753 of about 105 800), any lost packet tears a frame, and it would save at most the 1.1 ms decode while costing the Mac and the machine's userspace forwarder 35 000 packets a second against about 400 for H.264.

### Retained detail

The testbed's test pattern is 1280x720 now (the rungs scale it), a 720p JPEG of the ball is 15.6 kB, a crop of its middle half (640x360, quality 80) 6.9-7.3 kB. Not a camera: a real scene is ten to thirty times larger and the same allowance would take that much longer.

Snapshot branch against live video, `skeleton` at the top rung (1280x720, 15 fps) with the branch off and on (off: the vehicle config without its `[snapshot]` table, so the sender builds no tee; on: the testbed's `[snapshot]`, 1 fps, quality 90; nothing else differs), three `lan` runs and one `geo` run each:

| | branch off | branch on |
|---|---|---|
| Runs | `lan` 1793563892979, 1793564011717, 1793564066619; `geo` 1793563957030 | `lan` 1793563825705, 1793564191658, 1793564246542; `geo` 1793564135889 |
| `lan` frame age p50 / p95 / max | 7 / 10 / 14-27 ms | 7-8 / 10-11 / 14-16 ms |
| `geo` frame age p50 / p95 / max | 319 / 332 / 342 ms | 319 / 334 / 346 ms |
| Uplink on the wire (`lan`) | 45.3-45.9 kB/s | 44.7-46.0 kB/s |
| Companion container CPU (`podman stats`, 12 samples over each window, % of one core) | `lan` 20.5, 18.9, 19.3; `geo` 18.9 | `lan` 23.2, 19.4, 20.2; `geo` 17.6 |

The branch costs about 1 percentage point of one core (the means of the `lan` runs are 19.6 % and 20.9 %, the spread inside a configuration about 1.5), too little to separate from the noise of three runs. On a Pi 4 at the camera's full size the ratio will differ; it is unmeasured there.

Retention and recovery, `recording lan` (runs 1793566512809, 1793566695540, 1793566877955 on 02d4112): the store held 514 kB of its 524 kB quota with the oldest snapshot going first, snapshots 1000 ms apart, a restarted vehicle deleted the planted `.tmp`, ignored the two other names, indexed the 33 snapshots on disk and wrote again 303-404 ms after the restart was asked for; with every write made to take 3 s, 11 of the 20 snapshots in 20 s were dropped and counted, live frames kept coming at 14.9996 per second, and the frame age p95 was 10-12 ms against 10-11 ms healthy; the directory then had 6 gaps (largest 3-4 s) and the store reported 6.

Retrieval, `retrieval geo` (runs 1793566354368, 1793566566903, 1793566749663 on 02d4112), crop of the frame 8 s ago:

| | |
|---|---|
| Crop (6.9-7.3 kB) complete after | 4.01 s |
| Snapshot (15.6 kB) complete after | 6.01 s |
| Median job, 30 s pass of crops back to back | 3.5-3.8 s, 8 jobs, 32 chunks |
| Retrieval on the wire | 2.91-3.02 kB/s (allowance 4 kB/s), 0 bytes over the allowance |
| Cancel, from the request to the vehicle noticing | 608-609 ms |
| Hostile requests | 30 refused, the table filled to its bounds (fifth job, third 1 MiB job) and refused, relay counted them |
| Live frame age p50 / p95 with retrieval, against the paired pass without | 319-320 / 331-333 ms against 320 / 332 ms: delta -1 to 0 / -1 to +1 ms |

A chunk takes a round trip (about 0.6 s under `geo`), one in flight, so `geo` moves about 3.0 kB/s of the 4 kB/s allowance: the allowance is not what limits the transfer there; on `lan` it is (a snapshot in about 5 s).

`retrieval bandwidth-steps` (1793568748926 on 5116e0f): retrieval held back for 61 s of the 80 s pass (the ladder left its normal rungs, `not_normal`, or the uplink measured at a cut left no room for the reserve beside a normal rung, `no_allowance`), sent no chunk while held, did 4 jobs, and the frame age delta was 0 / +3 ms. `retrieval outage-10s` (1793566155402): held back for 13.9 s (the blackout and the climb back), no chunk while held, 2 jobs, delta 0 / +5 ms. Both profiles are in calibration (one run each).

Allowance, `[retrieval] allowance_bytes_per_s = 4000` (wire bytes). It is chosen from what the link can take, then checked against the paired runs, not fitted to them. Under `geo` the uplink is 64 kB/s and the vehicle at the top rung offers about 45 kB/s (control and telemetry included); the controller's 75 % fit is 48 kB/s. `Budget::split` partitions that share, so retrieval comes out of the video share and the total stays at the fit; the rest of the uplink still drains a queue. While a job sends, the reserve costs about 4 kB/s of video bitrate (about 8 % at the geo fit) when the source fills its share. The paired runs used the ball pattern, which under-fills the share, so they show no cost; the cost with a filling source is unmeasured. The reserve is granted only while the rung fitted to the share less the allowance is still a normal one. The allowance is 6 % of the uplink. Its chunk is half a second of it, 2000 bytes (about 2.7 kB on the wire with the request, TCP and tunnel headers), which occupies the uplink for 42 ms at 64 kB/s, below the 80 ms of queue delay (`queue_delay_base_ms`) that the controller and `permit` take as no queue: a video packet behind a chunk waits less than a healthy queue. Because the chunk grows with the allowance, it would reach 80 ms at about 8 kB/s, twice the allowance. Below the normal rungs the allowance is not used: at 128 kbit/s (16 kB/s) the same chunk would take 170 ms of the uplink. What it costs the live video, measured: nothing the frame age percentiles resolve in three paired `geo` runs (p50 -1 to 0, p95 -1 to +1 ms), and 0 to +5 ms in the other two profiles.

Limits of this evidence: the passes are 30 to 80 s, the video is a ball pattern at about 30 kB/s (a real scene fills the 48 kB/s the cut leaves and leaves less headroom, which the queue guard in `permit` then takes up by holding retrieval), the snapshots are small, nothing was run on a Pi or a real link, and the paired passes differ by their own run-to-run spread.

