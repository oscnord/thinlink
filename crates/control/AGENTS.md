# AGENTS.md: crates/control

`thinlink-control` (L2) owns the MAVLink bridges and control envelopes. It depends on `thinlink-policy` and `thinlink-video-core`. It builds natively without GStreamer. Follow the root `AGENTS.md`.

## Main pieces

Paths are relative to this crate.

| File | Ownership |
|---|---|
| `src/frame.rs` | `Frame`, MAVLink parsing through `mavlink::MavlinkReader` |
| `src/gate.rs` | `decide`, `route`, `refused`, `CommandKey` |
| `src/identity.rs`, `src/params.rs` | Destination decision and parameter checks |
| `src/autopilot.rs`, `src/vehicle.rs` | `Wire`, autopilot connection and telemetry bridge |
| `src/vehicle/control.rs` | `ControlStats`, vehicle endpoint and lease application |
| `src/ground.rs`, `src/ground/control.rs` | Ground bridge and `Controller` |
| `src/ground/camera.rs` | `Camera`, discovery and tracking |
| `src/envelope.rs`, `src/auth.rs` | `ToVehicle`, `ToGround`, `Refusal`, `Signer`, `Verifier` |
| `src/clock.rs`, `src/telemetry.rs` | `Clock`, `RateCap` |
| `src/landing.rs`, `src/position.rs`, `src/sighting.rs` | `Landing`, `NewestPosition`, `Announcer` |
| `src/stats.rs` | Bridge counters and link state |

## Rules

- Keep `identity::failures` as the single destination decision, wrapped by `identity::control_target`. Do not add a bypass or build release artifacts with the `testbed` feature.
- Route station commands through `gate::route` and the vehicle gate. Keep `allow_missions` checks in `gate::refused`. Add command rules and tests together.
- Use `mavlink` framing and `ed25519-dalek` signing. Verify envelopes before changing lease state. Match `Refusal` variants, never reason strings.
- Judge freshness on the vehicle's monotonic clock. `sent_unix_ms` is measurement only. The ground's challenge check does not replace vehicle admission.
- Do not queue commands or replay them after reconnect. Continuous refresh uses only the current `LeaseState::setpoint`. Discrete retries retain their transaction and must not execute twice.
- Serialize writes under the connection lock. A failed or timed-out write closes the connection; never retry a partial frame.
- Keep telemetry replies bounded by bytes and age. Clear them on reconnect. Rates count MAVLink frame bytes, excluding transport headers.
- Region envelopes never carry MAVLink frames or reach the autopilot. Use `RegionCell::confirmed` for the applied box. Clear region state with the lease.
- Keep camera traffic local to the ground adapter. Camera replies require recent autopilot traffic and must not mask a lost link.
- `Landing::next_frame` owns `LANDING_TARGET` construction. Do not repeat a consumed sighting or predict one.
- Keep the freshness output's `GroundStats::clock_bound` read free of the controller mutex. Report failed connections as down; count whole-message drops by reason.

## References and checks

Protocol: `docs/drone.md#control-protocol`. Ground station: `docs/drone.md#qgroundcontrol`. Display: `docs/drone.md#predictive-display`. Marker path: `docs/drone.md#landing-on-a-marker`. Lab setup: `docs/testbed.md`.

```sh
cargo test -p thinlink-control --locked
just lab-up && just lab-check mavlink lan && just lab-check control-freshness lan
```

Use table tests for gate decisions and reply-queue priority. Lease timelines belong in `thinlink-policy`. `src/vehicle/sitl_identity.rs` exercises destination selection with a loopback stand-in. Run `lab check qgc-status` for camera wiring and `just sim-check lan` for copter integration when affected.
