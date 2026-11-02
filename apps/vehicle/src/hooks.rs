//! The test-only hooks of the metrics endpoint. Compiled only with the `testbed` feature: a release
//! build has no way to switch an interlock off.

use std::sync::atomic::Ordering;
use std::time::Duration;

use serde_json::{Value, json};

use sat_teleop_control::clock::{Clock, Injection};
use sat_teleop_control::vehicle::VehicleStats;
use sat_teleop_media::sender::{SenderStats, SourceMode};

use super::{Retained, SLOW_WRITE};

/// The test-only hooks: `POST /video/pause`, `/video/freeze` and `/video/resume` set the sender's
/// source mode, for testing the ground's freshness output; `POST /snapshots/slow` and
/// `/snapshots/normal` set how long each snapshot write waits, `POST /clock/inject/<ms|missing|off>` replaces
/// the clock's bound (testing that the ground follows it and that real mode needs one),
/// `POST /real/allow-sitl/<on|off>` and `/real/require/<name>/<value>|reset` change what real mode
/// accepts and requires (testing that each interlock refuses control), for testing that slow storage drops
/// snapshots and never live frames. Refused unless the config enables debug hooks.
pub struct Hooks<'a> {
    pub enabled: bool,
    pub video: &'a SenderStats,
    pub retained: Option<&'a Retained>,
    pub clock: Option<&'a Clock>,
    pub bridge: &'a VehicleStats,
}

impl Hooks<'_> {
    pub fn answer(&self, method: &str, path: &str) -> Option<(u16, Value)> {
        let hook = match (method, path) {
            ("POST", p) if p.starts_with("/real/") => {
                real_hook(p.trim_start_matches("/real/")).map_or(Hook::Invalid, Hook::Real)
            }
            ("POST", p) if p.starts_with("/clock/inject/") => {
                let injection = injection(p.trim_start_matches("/clock/inject/"));
                injection.map_or(Hook::Invalid, Hook::Clock)
            }
            ("POST", "/video/pause") => Hook::Mode(SourceMode::Paused),
            ("POST", "/video/freeze") => Hook::Mode(SourceMode::Frozen),
            ("POST", "/video/resume") => Hook::Mode(SourceMode::Live),
            ("POST", "/snapshots/slow") => Hook::WriteDelay(SLOW_WRITE),
            ("POST", "/snapshots/normal") => Hook::WriteDelay(Duration::ZERO),
            _ => return None,
        };
        if !self.enabled {
            let error = "debug hooks are off: they exist for testing only (debug_hooks = true)";
            return Some((403, json!({ "error": error })));
        }
        Some(self.apply(hook))
    }

    fn apply(&self, hook: Hook) -> (u16, Value) {
        match hook {
            Hook::Mode(mode) => {
                self.video.set_mode(mode);
                (200, json!({ "mode": mode }))
            }
            Hook::Invalid => (400, json!({ "error": "not a value this hook takes" })),
            Hook::Real(hook) => {
                let control = &self.bridge.control;
                match hook {
                    RealHook::AllowSitl(allow) => control.set_allow_sitl(allow),
                    RealHook::Require(name, value) => control.require(&name, value),
                    RealHook::ResetRequired => control.reset_required(),
                }
                (200, json!({ "interlocks": control.failures() }))
            }
            Hook::Clock(injection) => {
                let Some(clock) = self.clock else {
                    return (409, json!({ "error": "the vehicle has no clock" }));
                };
                clock.inject(injection);
                (200, json!(clock.snapshot()))
            }
            Hook::WriteDelay(delay) => {
                let Some(retained) = self.retained else {
                    return (409, json!({ "error": "the vehicle keeps no snapshots" }));
                };
                let ms = u64::try_from(delay.as_millis()).unwrap_or(u64::MAX);
                retained.write_delay_ms.store(ms, Ordering::Relaxed);
                (200, json!({ "write_delay_ms": ms }))
            }
        }
    }
}

enum Hook {
    Mode(SourceMode),
    WriteDelay(Duration),
    Clock(Injection),
    Real(RealHook),
    Invalid,
}

/// What a `/real/...` hook does to real mode's interlocks.
enum RealHook {
    AllowSitl(bool),
    Require(String, f32),
    ResetRequired,
}

/// `allow-sitl/<on|off>`, `require/<name>/<value>` and `require/reset`.
fn real_hook(path: &str) -> Option<RealHook> {
    let parts: Vec<&str> = path.split('/').collect();
    match parts[..] {
        ["allow-sitl", "on"] => Some(RealHook::AllowSitl(true)),
        ["allow-sitl", "off"] => Some(RealHook::AllowSitl(false)),
        ["require", "reset"] => Some(RealHook::ResetRequired),
        ["require", name, value] => {
            let value = value.parse().ok().filter(|v: &f32| v.is_finite())?;
            Some(RealHook::Require(name.to_owned(), value))
        }
        _ => None,
    }
}

/// `POST /clock/inject/<ms>` gives the clock that bound, `missing` takes its bound away and `off`
/// gives the source its say again.
fn injection(text: &str) -> Option<Injection> {
    match text {
        "off" => Some(Injection::Off),
        "missing" => Some(Injection::Missing),
        ms => ms.parse().ok().map(Injection::Bound),
    }
}
