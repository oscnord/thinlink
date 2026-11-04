# Real mode

`[mavlink] mode = "real"` lets control reach a real flight controller only while every interlock holds. Otherwise commands are refused, QGroundControl shows them denied, and `mavlink.control.interlocks` and the vehicle log identify each failed check and its reason.

## Setup

| Setting | Value |
|---|---|
| Vehicle `[mavlink]` | `mode = "real"`; `autopilot = "serial:<device>:<baud>"`; nonempty `require` below |
| Both configs | Matching `vehicle_id` (under `[mavlink]` on the vehicle, top-level on the ground) |
| Vehicle `[mavlink].controllers` | `[{ address = "10.77.0.3", public_key = "/etc/thinlink/ctl-ground.pub" }]`; use the ground's overlay address |
| Ground `control_key` | `/secrets/ctl-ground.key`, the matching Ed25519 private key; permissions `600` |
| Vehicle `[mavlink].clock` | `"chrony"` or `{ chrony_at = "host" }`; `shared` and `fixed` are refused |
| Ground `clock` | `{ chrony_at = "host.containers.internal" }` on Linux; a measured `{ fixed = <ms> }` on Mac; [clock setup](hardware.md#clock-sync) |

`just deploy-keys <relay-endpoint>` creates the keys; [deployment](hardware.md#images-and-binaries) installs them. Keys load once; restart after changing them. Omit `debug_hooks` and `sitl_in_real_mode`: release builds reject them.

## Interlocks

| Check | Requires | Failure evidence |
|---|---|---|
| `identity` | Copter or rover heartbeat; no `SIM_WIND_SPD` reply after three probes | Reason names the missing heartbeat, SITL, or unfinished probes |
| `controller_keys` | Every configured public key loaded at startup | `mavlink.control.key_errors`; reason `no key for ...` |
| `failsafe_params` | Nonempty `require`; matching type and comparison from the autopilot's own system/component, read within 30 s | Reason names unread, stale or mismatched parameters |
| `clock_bound` | A fresh measured clock bound observed since startup | `mavlink.control.clock.started = false`, `bound_ms = null` |

Parameters are reread after 10 s. A failed check withdraws commands and the ground heartbeat; the autopilot's current failsafe settings decide its response. Restoring the parameter restores eligibility for control.

The clock check gates startup only. Later loss makes the video `AGE UNKNOWN` without withdrawing control or heartbeat.

## Required autopilot parameters

`require` defines the base set, with no built-in names or values; `allow_missions = true` adds the mission rows below, and `[landing] target = true` adds the `PLND_*` rows.

Each entry has `name`, `type`, `value` and optional `compare` (default `equal`). Types: `uint8`, `int8`, `uint16`, `int16`, `uint32`, `int32`, `real32`. Set parameters on the autopilot separately; thinlink only reads them.

| Name | Required value |
|---|---|
| `FS_OPTIONS` | Two entries: `compare = "bits_set", value = 3`; `compare = "bits_clear", value = 16` |
| `FENCE_ENABLE` | 1 |
| `FENCE_ACTION` | 1 |
| `FENCE_TYPE` | `compare = "bits_set", value = 3` |
| `FENCE_ALT_MAX` | `at_most` a configured limit above 0 and at most 120 m |
| `BATT_FS_LOW_ACT` | `at_least` a configured minimum of 1 or more |
| `PLND_ENABLED` | 1 |
| `PLND_TYPE` | 1 |
| `PLND_STRICT` | 1 |
| `PLND_YAW_ALIGN` | 0 |
| `PLND_ALT_MIN` | 0 |
| `PLND_ALT_MAX` | 0 |

Bit masks may include extra bits.

## Missions flown out of reach

Real mode rejects `allow_missions = true` at startup unless `require` contains every mission rule above. Those readings remain interlocks while Auto continues through link loss. See [missions](drone.md#missions) for uploads and limits.

## Landing on a marker

Real mode requires a fixed camera with `[camera] tilt_deg = -90` and the `PLND_*` entries above; gimbals are refused. See [marker landing](drone.md#landing-on-a-marker) for camera settings and target behavior.

## What is authenticated, and what is not

- Control envelopes and lease requests/releases are Ed25519-signed for the vehicle ID, with replay checks.
- Bearer tokens protect management endpoints: `vehicle.token` for vehicle metrics, `ground.token` for ground metrics/lease endpoints, `job.token` for relay metrics/jobs.
- Video, telemetry, allowed raw MAVLink, clock queries/replies, challenges and command results are not signed end to end. WireGuard protects each relay link; the relay remains trusted for received data.

## Checking real mode

1. Read authenticated `GET /metrics`: `mavlink.control` has `mode = real`, `target = autopilot`, empty `interlocks`/`key_errors` and numeric `clock.bound_ms`.
2. Change a required parameter. After rereading, expect `failsafe_params` and a denied arm in QGroundControl. Restore it and confirm the check clears.
3. Stop chrony. Within 15 s, expect a null bound and `AGE UNKNOWN`, with control continuing. Restart the vehicle while chrony is stopped: expect `clock_bound` and refused control. Restore chrony.
4. With the lease held, pull the link and verify the autopilot's configured failsafe response.
5. With `allow_missions = true`, upload/start a mission and verify Auto continues through link loss. Restore the link, clear `FS_OPTIONS` bit 1 and expect `failsafe_params`. Restore the parameter. With the switch off, upload, start and Auto must be denied.
