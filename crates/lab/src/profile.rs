use std::fmt::Display;

use serde::{Deserialize, Serialize};

pub const MAX_DELAY_MS: u32 = 5000;
pub const MIN_RATE_KBIT: u32 = 8;
pub const MAX_RATE_KBIT: u32 = 1_000_000;
const MIN_BURST_BYTES: u32 = 1514;
const MAX_BURST_BYTES: u32 = 10_000_000;
const MAX_QUEUE_PACKETS: u32 = 100_000;
const MAX_TIMELINE_ENTRIES: usize = 1000;
const MAX_TIMELINE_MS: u64 = 86_400_000;

#[derive(Debug, thiserror::Error)]
pub enum ProfileError {
    #[error("profile does not parse: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("{field}: {reason}")]
    Invalid { field: String, reason: String },
}

fn invalid(field: &str, reason: impl Display) -> ProfileError {
    ProfileError::Invalid {
        field: field.to_owned(),
        reason: reason.to_string(),
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    #[serde(rename = "profile")]
    pub meta: Meta,
    pub uplink: Link,
    pub downlink: Link,
    #[serde(default)]
    pub timeline: Vec<TimelineEntry>,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Meta {
    pub name: String,
    pub description: String,
    pub provenance: Provenance,
    pub source: String,
    pub seed: Option<u64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum Provenance {
    Synthetic,
    Measured,
}

/// One direction of the link. Uplink is router egress toward `ground_net`,
/// downlink is router egress toward `vehicle_net`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Link {
    pub delay_ms: u32,
    #[serde(default)]
    pub jitter_ms: u32,
    #[serde(default)]
    pub jitter_distribution: Distribution,
    pub rate_kbit: u32,
    #[serde(default = "default_burst_bytes")]
    pub burst_bytes: u32,
    #[serde(default = "default_queue_packets")]
    pub queue_packets: u32,
    #[serde(default)]
    pub ordering: Ordering,
    pub loss: Option<Loss>,
}

fn default_burst_bytes() -> u32 {
    16_384
}

fn default_queue_packets() -> u32 {
    1000
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Distribution {
    #[default]
    Normal,
    Pareto,
    Uniform,
}

impl Distribution {
    pub fn tc_name(self) -> &'static str {
        match self {
            Self::Normal => "normal",
            Self::Pareto => "pareto",
            Self::Uniform => "uniform",
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Ordering {
    #[default]
    Preserve,
    AllowReorder,
}

/// Percentages are 0..=100. Gilbert-Elliott: `p` good to bad, `r` bad to good,
/// `one_minus_h` loss in the bad state, `one_minus_k` loss in the good state.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "model", rename_all = "kebab-case", deny_unknown_fields)]
pub enum Loss {
    Random {
        percent: f64,
    },
    GilbertElliott {
        p: f64,
        r: f64,
        one_minus_h: f64,
        one_minus_k: f64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Uplink,
    Downlink,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Direction {
    Uplink,
    Downlink,
    Both,
}

impl Direction {
    pub fn sides(self) -> &'static [Side] {
        match self {
            Self::Uplink => &[Side::Uplink],
            Self::Downlink => &[Side::Downlink],
            Self::Both => &[Side::Uplink, Side::Downlink],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    SetRate { rate_kbit: u32 },
    Blackout,
    Restore,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(try_from = "RawTimelineEntry")]
pub struct TimelineEntry {
    pub at_ms: u64,
    pub direction: Direction,
    pub action: Action,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawTimelineEntry {
    at_ms: u64,
    direction: Direction,
    action: RawAction,
    rate_kbit: Option<u32>,
}

#[derive(Deserialize)]
#[serde(rename_all = "kebab-case")]
enum RawAction {
    SetRate,
    Blackout,
    Restore,
}

impl TryFrom<RawTimelineEntry> for TimelineEntry {
    type Error = ProfileError;

    fn try_from(raw: RawTimelineEntry) -> Result<Self, ProfileError> {
        let action = match (raw.action, raw.rate_kbit) {
            (RawAction::SetRate, Some(rate_kbit)) => {
                in_range(
                    "timeline.rate_kbit",
                    rate_kbit,
                    MIN_RATE_KBIT,
                    MAX_RATE_KBIT,
                )?;
                Action::SetRate { rate_kbit }
            }
            (RawAction::SetRate, None) => {
                return Err(invalid("timeline.rate_kbit", "required by set-rate"));
            }
            (RawAction::Blackout, None) => Action::Blackout,
            (RawAction::Restore, None) => Action::Restore,
            (RawAction::Blackout | RawAction::Restore, Some(_)) => {
                return Err(invalid("timeline.rate_kbit", "only valid with set-rate"));
            }
        };
        in_range("timeline.at_ms", raw.at_ms, 0, MAX_TIMELINE_MS)?;
        Ok(Self {
            at_ms: raw.at_ms,
            direction: raw.direction,
            action,
        })
    }
}

impl Profile {
    pub fn parse(text: &str) -> Result<Self, ProfileError> {
        let profile: Self = toml::from_str(text)?;
        profile.validate()?;
        Ok(profile)
    }

    pub fn link(&self, side: Side) -> &Link {
        match side {
            Side::Uplink => &self.uplink,
            Side::Downlink => &self.downlink,
        }
    }

    fn validate(&self) -> Result<(), ProfileError> {
        self.meta.validate()?;
        self.uplink.validate("uplink")?;
        self.downlink.validate("downlink")?;
        self.validate_timeline()
    }

    fn validate_timeline(&self) -> Result<(), ProfileError> {
        if self.timeline.len() > MAX_TIMELINE_ENTRIES {
            return Err(invalid(
                "timeline",
                format!("more than {MAX_TIMELINE_ENTRIES} entries"),
            ));
        }
        let ordered = self
            .timeline
            .iter()
            .zip(self.timeline.iter().skip(1))
            .all(|(a, b)| a.at_ms <= b.at_ms);
        if !ordered {
            return Err(invalid(
                "timeline.at_ms",
                "entries must not go back in time",
            ));
        }
        Ok(())
    }
}

/// Profile names are also file names: 1-64 characters of a-z, 0-9 and '-'.
pub fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 64
        && name
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
}

impl Meta {
    fn validate(&self) -> Result<(), ProfileError> {
        if !valid_name(&self.name) {
            return Err(invalid(
                "profile.name",
                "1-64 characters of a-z, 0-9 and '-'",
            ));
        }
        if self.provenance == Provenance::Measured && self.source.trim().is_empty() {
            return Err(invalid(
                "profile.source",
                "required when provenance is measured",
            ));
        }
        Ok(())
    }
}

impl Link {
    fn validate(&self, dir: &str) -> Result<(), ProfileError> {
        let f = |field: &str| format!("{dir}.{field}");
        in_range(&f("delay_ms"), self.delay_ms, 0, MAX_DELAY_MS)?;
        in_range(&f("jitter_ms"), self.jitter_ms, 0, MAX_DELAY_MS)?;
        if self.jitter_ms > self.delay_ms {
            return Err(invalid(&f("jitter_ms"), "must not exceed delay_ms"));
        }
        in_range(
            &f("rate_kbit"),
            self.rate_kbit,
            MIN_RATE_KBIT,
            MAX_RATE_KBIT,
        )?;
        in_range(
            &f("burst_bytes"),
            self.burst_bytes,
            MIN_BURST_BYTES,
            MAX_BURST_BYTES,
        )?;
        in_range(
            &f("queue_packets"),
            self.queue_packets,
            1,
            MAX_QUEUE_PACKETS,
        )?;
        match &self.loss {
            None => Ok(()),
            Some(loss) => loss.validate(&format!("{dir}.loss")),
        }
    }
}

impl Loss {
    fn validate(&self, field: &str) -> Result<(), ProfileError> {
        match *self {
            Self::Random { percent } => percent_in_range(&format!("{field}.percent"), percent),
            Self::GilbertElliott {
                p,
                r,
                one_minus_h,
                one_minus_k,
            } => {
                percent_in_range(&format!("{field}.p"), p)?;
                percent_in_range(&format!("{field}.r"), r)?;
                percent_in_range(&format!("{field}.one_minus_h"), one_minus_h)?;
                percent_in_range(&format!("{field}.one_minus_k"), one_minus_k)
            }
        }
    }
}

fn in_range<T: PartialOrd + Display + Copy>(
    field: &str,
    value: T,
    min: T,
    max: T,
) -> Result<(), ProfileError> {
    if value < min || value > max {
        return Err(invalid(field, format!("{value} is outside {min}..={max}")));
    }
    Ok(())
}

fn percent_in_range(field: &str, value: f64) -> Result<(), ProfileError> {
    if !value.is_finite() || !(0.0..=100.0).contains(&value) {
        return Err(invalid(
            field,
            format!("{value} is not a percentage in 0..=100"),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID: &str = r#"
[profile]
name = "t"
description = "test"
provenance = "synthetic"
source = "unit test"

[uplink]
delay_ms = 300
jitter_ms = 10
rate_kbit = 512

[downlink]
delay_ms = 300
rate_kbit = 2048
"#;

    #[test]
    fn minimal_profile_gets_defaults() {
        let p = Profile::parse(VALID).unwrap();
        assert_eq!(p.uplink.jitter_distribution, Distribution::Normal);
        assert_eq!(p.uplink.ordering, Ordering::Preserve);
        assert_eq!(p.uplink.loss, None);
        assert_eq!(p.downlink.jitter_ms, 0);
        assert_eq!(p.timeline, []);
    }

    #[test]
    fn loss_and_timeline_parse() {
        let text = format!(
            "{VALID}
[uplink.loss]
model = \"gilbert-elliott\"
p = 3
r = 20
one_minus_h = 80.5
one_minus_k = 0

[downlink.loss]
model = \"random\"
percent = 1.5

[[timeline]]
at_ms = 10000
action = \"blackout\"
direction = \"both\"

[[timeline]]
at_ms = 15000
action = \"set-rate\"
direction = \"uplink\"
rate_kbit = 128
"
        );
        let p = Profile::parse(&text).unwrap();
        assert_eq!(
            p.uplink.loss,
            Some(Loss::GilbertElliott {
                p: 3.0,
                r: 20.0,
                one_minus_h: 80.5,
                one_minus_k: 0.0
            })
        );
        assert_eq!(p.downlink.loss, Some(Loss::Random { percent: 1.5 }));
        assert_eq!(p.timeline[0].action, Action::Blackout);
        assert_eq!(p.timeline[1].action, Action::SetRate { rate_kbit: 128 });
        assert_eq!(p.timeline[1].direction, Direction::Uplink);
    }

    #[test]
    fn invalid_profiles_are_rejected() {
        let cases: &[(&str, String, &str)] = &[
            (
                "unknown field",
                VALID.replace("rate_kbit = 512", "rate_kbit = 512\nbogus = 1"),
                "bogus",
            ),
            (
                "delay too large",
                VALID.replace("delay_ms = 300\njitter_ms", "delay_ms = 5001\njitter_ms"),
                "uplink.delay_ms",
            ),
            (
                "jitter above delay",
                VALID.replace("jitter_ms = 10", "jitter_ms = 301"),
                "uplink.jitter_ms",
            ),
            (
                "rate too low",
                VALID.replace("rate_kbit = 512", "rate_kbit = 7"),
                "uplink.rate_kbit",
            ),
            (
                "rate too high",
                VALID.replace("rate_kbit = 2048", "rate_kbit = 1000001"),
                "downlink.rate_kbit",
            ),
            (
                "burst below mtu",
                VALID.replace("rate_kbit = 512", "rate_kbit = 512\nburst_bytes = 100"),
                "uplink.burst_bytes",
            ),
            (
                "empty queue",
                VALID.replace("rate_kbit = 512", "rate_kbit = 512\nqueue_packets = 0"),
                "uplink.queue_packets",
            ),
            (
                "bad name",
                VALID.replace("name = \"t\"", "name = \"Bad Name\""),
                "profile.name",
            ),
            (
                "measured without source",
                VALID
                    .replace("synthetic", "measured")
                    .replace("source = \"unit test\"", "source = \" \""),
                "profile.source",
            ),
            (
                "unknown distribution",
                VALID.replace(
                    "rate_kbit = 512",
                    "rate_kbit = 512\njitter_distribution = \"cauchy\"",
                ),
                "cauchy",
            ),
            (
                "loss over 100",
                format!("{VALID}\n[uplink.loss]\nmodel = \"random\"\npercent = 100.5\n"),
                "uplink.loss.percent",
            ),
            (
                "loss nan",
                format!("{VALID}\n[uplink.loss]\nmodel = \"random\"\npercent = nan\n"),
                "uplink.loss.percent",
            ),
            (
                "loss negative",
                format!(
                    "{VALID}\n[downlink.loss]\nmodel = \"gilbert-elliott\"\np = 1\nr = -1\none_minus_h = 1\none_minus_k = 1\n"
                ),
                "downlink.loss.r",
            ),
            (
                "ge missing field",
                format!("{VALID}\n[uplink.loss]\nmodel = \"gilbert-elliott\"\np = 1\n"),
                "missing field",
            ),
            (
                "unknown loss model",
                format!("{VALID}\n[uplink.loss]\nmodel = \"markov\"\n"),
                "markov",
            ),
            (
                "set-rate without rate",
                format!(
                    "{VALID}\n[[timeline]]\nat_ms = 0\naction = \"set-rate\"\ndirection = \"both\"\n"
                ),
                "timeline.rate_kbit",
            ),
            (
                "set-rate out of range",
                format!(
                    "{VALID}\n[[timeline]]\nat_ms = 0\naction = \"set-rate\"\ndirection = \"both\"\nrate_kbit = 1\n"
                ),
                "timeline.rate_kbit",
            ),
            (
                "blackout with rate",
                format!(
                    "{VALID}\n[[timeline]]\nat_ms = 0\naction = \"blackout\"\ndirection = \"both\"\nrate_kbit = 100\n"
                ),
                "timeline.rate_kbit",
            ),
            (
                "timeline out of order",
                format!(
                    "{VALID}\n[[timeline]]\nat_ms = 5\naction = \"restore\"\ndirection = \"both\"\n[[timeline]]\nat_ms = 4\naction = \"restore\"\ndirection = \"both\"\n"
                ),
                "timeline.at_ms",
            ),
        ];
        for (label, text, expected) in cases {
            let err = Profile::parse(text).expect_err(label).to_string();
            assert!(err.contains(expected), "{label}: {err}");
        }
    }
}
