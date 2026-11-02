//! How far a machine's clock can be from true time, and how two machines' bounds add up. The ground
//! labels video `Live` only when the newest frame's age plus this bound is under the stale
//! threshold (`freshness`), so a bound too small lets old video look live and a missing one means
//! `AgeUnknown`. Each machine's bound comes from chrony (`chronyc -c tracking`); the vehicle reports
//! its own to the ground, which adds it to its own.

/// A reading older than this is no bound at all: the clock may have drifted or lost its sources
/// since, and the machine that took it may be gone.
pub const MAX_AGE_MS: u64 = 15_000;
/// What a clock may wander per second between readings, in parts per million: the most the kernel
/// is willing to slew it (`adjtimex`, 500 ppm).
const DRIFT_PPM: u64 = 500;
/// Added once to the sum of both machines' bounds: the stamp reads the wall clock a moment before
/// the frame is pulled, and a reading is rounded up to a millisecond.
pub const MARGIN_MS: u64 = 10;

/// A bound in milliseconds and how long ago it was taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub struct Reading {
    pub bound_ms: u64,
    pub age_ms: u64,
}

/// The bound a reading gives now: it grows by what the clock may have wandered since, and it is
/// nothing once older than `MAX_AGE_MS`.
pub fn aged(reading: Reading) -> Option<u64> {
    if reading.age_ms > MAX_AGE_MS {
        return None;
    }
    let wandered = (reading.age_ms * DRIFT_PPM).div_ceil(1_000_000);
    Some(reading.bound_ms.saturating_add(wandered))
}

/// The bound on the difference between two machines' clocks, `None` when either has no fresh one.
pub fn combined(own: Option<Reading>, other: Option<Reading>) -> Option<u64> {
    let (own, other) = (own.and_then(aged)?, other.and_then(aged)?);
    Some(own.saturating_add(other).saturating_add(MARGIN_MS))
}

/// Whether a bound measured by hand still holds: the wall clock and the monotonic clock advanced by
/// the same amount, within the bound, since the last time they were compared. A wall clock that
/// was stepped (set by a time service, or resumed after a suspend) moves them apart by the step,
/// and the operator's measurement says nothing about the clock after it.
pub fn fixed_holds(bound_ms: u64, wall_elapsed_ms: u64, monotonic_elapsed_ms: u64) -> bool {
    wall_elapsed_ms.abs_diff(monotonic_elapsed_ms) <= bound_ms
}

/// An upper bound, in milliseconds, on how far the system clock is from true time, from one line of
/// `chronyc -c tracking`: the larger of the system time offset and the last offset, plus the root
/// dispersion and half the root delay (chrony's own definition of the error bound of its time,
/// `man chronyc`, "tracking"), rounded up. `None` when chrony says it is not synchronised, the line
/// is not a tracking report or a number is not finite.
pub fn from_tracking(line: &str) -> Option<u64> {
    let fields: Vec<&str> = line.trim().split(',').collect();
    // refid, name, stratum, ref time, system time, last offset, rms offset, frequency, residual
    // frequency, skew, root delay, root dispersion, update interval, leap status
    let [
        refid,
        _,
        stratum,
        _,
        system,
        last,
        _,
        _,
        _,
        _,
        delay,
        dispersion,
        _,
        leap,
    ] = fields[..]
    else {
        return None;
    };
    let synchronised = !refid.chars().all(|c| c == '0') && stratum != "0" && leap == "Normal";
    if !synchronised {
        return None;
    }
    let number = |text: &str| text.parse::<f64>().ok().filter(|n| n.is_finite());
    let offset = number(system)?.abs().max(number(last)?.abs());
    let seconds = offset + number(dispersion)?.abs() + number(delay)?.abs() / 2.0;
    whole_ms(seconds)
}

/// `seconds` as whole milliseconds rounded up; `None` for a bound of a day or more, which is a
/// broken line, not a clock.
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "the value is at least 0 and under a day of milliseconds"
)]
fn whole_ms(seconds: f64) -> Option<u64> {
    let ms = (seconds * 1000.0).ceil();
    (0.0..86_400_000.0).contains(&ms).then_some(ms as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bound_is_chronys_error_bound_rounded_up_and_none_when_unsynchronised() {
        // A line from chrony 4.x: offsets in seconds, 1.2 ms system offset, 0.3 ms last offset,
        // 25 ms root delay, 3.1 ms root dispersion: 1.2 + 3.1 + 12.5 = 16.8 ms.
        let line = |refid, stratum, leap| {
            format!(
                "{refid},ntp.example,{stratum},1730000000.0,0.0012,-0.0003,0.0001,1.5,0.01,0.2,0.025,0.0031,64.0,{leap}"
            )
        };
        let cases = [
            (line("C0A80001", 2, "Normal"), Some(17)),
            (line("C0A80001", 2, "Insert second"), None),
            (line("C0A80001", 2, "Not synchronised"), None),
            (line("00000000", 0, "Not synchronised"), None),
            (line("C0A80001", 0, "Normal"), None),
            ("not,a,tracking,line".to_owned(), None),
            (String::new(), None),
            (line("C0A80001", 2, "Normal").replace("0.0031", "nan"), None),
        ];
        for (text, expected) in cases {
            assert_eq!(from_tracking(&text), expected, "{text}");
        }
    }

    #[test]
    fn a_fixed_bound_holds_until_the_wall_clock_steps_by_more_than_it() {
        // (bound, wall elapsed, monotonic elapsed, holds) over one 2 s poll interval
        let cases = [
            (50, 2_000, 2_000, true),
            (50, 2_001, 2_000, true),
            (50, 2_050, 2_000, true),
            (50, 2_051, 2_000, false),
            (50, 1_949, 2_000, false),
            // The wall clock set an hour ahead, and the VM clock resumed after a night's sleep.
            (50, 3_602_000, 2_000, false),
            (50, 0, 2_000, false),
        ];
        for (bound, wall, monotonic, holds) in cases {
            assert_eq!(
                fixed_holds(bound, wall, monotonic),
                holds,
                "{wall} {monotonic}"
            );
        }
    }

    #[test]
    fn two_bounds_add_up_only_while_both_are_fresh() {
        let at = |bound_ms, age_ms| Some(Reading { bound_ms, age_ms });
        let cases = [
            (at(5, 0), at(7, 0), Some(5 + 7 + MARGIN_MS)),
            // 10 s old: 500 ppm of 10 s is 5 ms more.
            (at(5, 10_000), at(7, 0), Some(5 + 5 + 7 + MARGIN_MS)),
            (at(5, MAX_AGE_MS), at(7, 0), Some(5 + 8 + 7 + MARGIN_MS)),
            (at(5, MAX_AGE_MS + 1), at(7, 0), None),
            (at(5, 0), at(7, MAX_AGE_MS + 1), None),
            (None, at(7, 0), None),
            (at(5, 0), None, None),
        ];
        for (own, other, expected) in cases {
            assert_eq!(combined(own, other), expected, "{own:?} {other:?}");
        }
    }
}
