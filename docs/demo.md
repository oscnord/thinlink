# Demo: the Mac's camera over an emulated link

`just demo <profile>` shows the Mac's camera in QGroundControl after the delay, jitter, loss and bandwidth of a link profile. The camera is the vehicle's video source: its frames are stamped, adapted and sent through the testbed's router like any other frame, so the age in the picture is the testbed's real emulated link plus what the camera path adds before the stamp. Nothing here controls a real vehicle: the autopilot is still ArduRover SITL.

## Run it

Requirements: the quickstart's (Podman, `just`, QGroundControl) and `ffmpeg` (`brew install ffmpeg`).

```sh
just demo-cameras        # list the camera indexes; 0 is the default
just demo lan            # or geo, lte-good, lte-bad, geo-degraded, outage-10s
just demo geo 1          # a profile and a camera index; CAMERA=1 just demo geo works too
just demo-stop           # stop the camera, then the testbed
```

`just demo` brings the testbed up with the vehicle taking its video from the camera ingest (`testbed/config/vehicle-ingest.toml`), applies the profile, starts the camera and waits for its first frame. It takes about a minute with built images. If anything fails after the camera started, it stops the camera again but leaves the testbed up (`just lab-down` or `just demo-stop` takes it down); otherwise the camera runs until `just demo-stop`. A process already using UDP 5700 on the Mac stops `just lab-up`, because the testbed cannot publish the port. A plain `just lab-up` afterwards goes back to the test pattern.

`just regional=1 demo geo` also turns the person detector and softening (`soften_factor = 2`) on (`testbed/config/vehicle-ingest-regional.toml`): people the webcam sees are outlined in amber and kept sharp, the rest is softened when you box a region or a person is found. Draw a box in QGroundControl after `just take-control` (`docs/qgc.md`, "Regional video: drawing a box").

### Camera permission on macOS

The first run asks whether the terminal app (the one `just` runs in) may use the camera. Say yes. If it was refused, or the prompt never appeared, the recipe reports that the camera delivered no frames, prints what ffmpeg said and where to allow it: System Settings, Privacy & Security, Camera, then your terminal app. It does not try to work around a refusal. Another app holding the camera can fail the same way.

### Watching it in QGroundControl

Start QGroundControl; nothing needs setting up (`docs/qgc.md`). It finds the video as the vehicle's camera and plays it with the label the ground adapter draws, bottom left (`docs/qgc.md`, "Video: the freshness output"); turn on Low Latency Mode in Application Settings, Video.

## What to expect per profile

The age is the newest frame's age at the ground adapter's output, measured from the stamp, so it leaves out the camera's own capture and encode time (see "How the camera gets in"). With the camera on `lan` the age is a few milliseconds and the label `LIVE`; on `geo` it is about 0.33 s, still `LIVE` (`geo` was not run with the camera; the link does not depend on the source). The figures are in `docs/measurement.md`, "Camera ingest".

The other profiles add their own delay on top, as in `docs/testbed.md`, "Profiles". Under `lte-bad`, `geo-degraded` and `outage-10s` the vehicle's adaptation steps the picture down or pauses it, and the labels follow (`LOW RATE`, `STALE`, `NO VIDEO`).

## Showing each label

- **LIVE with a covered lens.** Cover the lens: the picture turns dark but the label stays `LIVE`. A covered lens is a live camera of a dark scene, and the demo's camera arrives through the ingest, whose frames are stamped when they reach the vehicle. Only a check at the camera could see a stuck camera (`docs/qgc.md`, "What the states cannot see").
- **STALE, then NO VIDEO.** `just video-hook pause` stops the vehicle's video at the stamp: `STALE`, then `NO VIDEO`, the last picture kept (timings in `docs/qgc.md`, "Checking it in QGC"). `just video-hook resume` brings `LIVE` back at once. `just video-hook freeze` repeats one frame: `FROZEN`.
- **No camera frames.** Stopping the camera (`just demo-stop` also takes the testbed down, so stop only ffmpeg with `kill $(cat .cache/demo-camera.pid)`) leaves the vehicle without frames, the same as a pause at the source; the ground labels follow the same rules. This has not been watched on screen.
- **A worse link.** `just demo geo-degraded`, or change the profile of a running demo with `just impair geo`.

## Driving the simulated rover while watching the camera

The camera and the control lease are independent; the video shows what the camera sees, the rover is SITL.

1. With the demo running and QGroundControl showing the vehicle, `just take-control` asks the ground adapter for the vehicle's control lease.
2. In QGC set the flight mode to Manual and arm, then drive with a joystick or the virtual thumbsticks. Commands cross the same impaired link, so under `geo` the rover reacts about 0.6 s later.
3. Disarm, then `just release-control`. `docs/qgc.md`, "Taking control", has the details and what happens if the link drops.

## How the camera gets in

`just demo` runs ffmpeg on the Mac: avfoundation capture at 1280x720, 30 fps, without input buffering, encoded with x264 `ultrafast`, `zerolatency`, no B-frames, a keyframe every 15 frames (half a second) with the parameter sets repeated at each, 3 Mbit/s, sent as RTP/H.264 in 1200-byte packets to `127.0.0.1:5700`. That port is published to the vehicle's sidecar on a network of its own that carries nothing else (`docs/testbed.md`, "Topology"). The vehicle accepts the packets only from the host's address on that network, decodes them with a decoder that holds no frames back (slice threading, no jitter buffer in front of it) and stamps the result, so the ingest adds one decode before the stamp, about 1 ms. Camera capture and encode happen before it, and are not in the label's age: on this Mac they add the camera's own latency (unmeasured, as it needs an optical test) plus about 3 ms of CPU per frame for x264. Measured numbers: `docs/measurement.md`, "Camera ingest".

Without a camera, `just ingest-check lan` feeds the same port from a test-pattern container and grades the path.

## Using it for real

The ingest accepts RTP/H.264 (payload type 96, no B-frames) from any sender whose address is listed in `from` of the vehicle's `[source]`, so a different camera or encoder on the host needs only to send that stream to `127.0.0.1:5700`. Port, addresses and the single-port filter are in `testbed/compose.yml` and `testbed/scripts/ingest.nft`. `from` trusts a source address only, and nothing proves the sender is the camera: on Podman, any process on the Mac can send from the gateway address that `from` lists. The stream should have no B-frames, to keep the decode short. After a lost packet the picture is broken until the next keyframe, so the ingest drops pictures until then rather than showing them as fresh, for at most the camera's keyframe interval (half a second with `just demo`). It has no jitter buffer and does not ask the camera for keyframes.
