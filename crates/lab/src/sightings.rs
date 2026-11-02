//! `lab check sightings`: people the detector sees while the link is cut are told to the ground
//! when it returns. On the simulated flight with the person detector on (`just regional=1
//! sim-up`), `checks` is a ground station; the profile's timeline cuts the link for 30 s while the
//! two people stand in view. After the restore the check expects the sightings as `STATUSTEXT`
//! with positions and the times they were first seen, none before it, and the same sightings
//! listed through the job API with a snapshot each, one of which it fetches.

use std::thread;
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use sat_teleop_detail::client::Client;
use sat_teleop_detail::wire::{JobRequest, Kind as JobKind, Listing, MAX_LISTED};
use sat_teleop_media::unix_ms;

use crate::check::{self, Results, Verdict};
use crate::client::AgentClient;
use crate::control::blackout;
use crate::mavlink::gcs::{Gcs, Kind, Seen};
use crate::profile::Profile;

const HEARTBEAT_DEADLINE: Duration = Duration::from_secs(60);
/// The detector has run on the people before the link is cut.
const DETECTOR_DEADLINE: Duration = Duration::from_secs(60);
/// After the restore, the first sighting must arrive within this, and the rest follow the first
/// within this of quiet.
const FIRST_WITHIN: Duration = Duration::from_secs(30);
const QUIET: Duration = Duration::from_secs(5);
/// A sighting's time of day may lie this far outside the cut.
const TIME_SLACK_S: u64 = 3;
/// All sightings are of the copter standing still: positions agree within this, in degrees times 1e7.
const SAME_PLACE_E7: i64 = 10_000;
const CLIENT_TIMEOUT: Duration = Duration::from_secs(10);
const JOB_MAX_BYTES: u32 = 524_288;
const JOB_LIFE_MS: u32 = 120_000;
const JOB_DEADLINE: Duration = Duration::from_secs(110);

#[derive(Debug)]
pub struct Settings {
    pub gcs_listen: std::net::SocketAddr,
    pub vehicle_metrics: String,
    pub ground_api: String,
}

impl Settings {
    fn to_json(&self) -> Value {
        json!({
            "gcs_listen": self.gcs_listen,
            "vehicle_metrics": self.vehicle_metrics,
            "ground_api": self.ground_api,
        })
    }
}

/// A `STATUSTEXT` of the form `person 59.32931 18.06862 12m 14:02:31`.
#[derive(Debug, PartialEq)]
struct Told {
    lat_e7: i64,
    lon_e7: i64,
    seconds_of_day: u64,
}

fn parse(text: &str) -> Option<Told> {
    let mut words = text.strip_prefix("person ")?.split(' ');
    let (lat, lon, altitude, time) = (words.next()?, words.next()?, words.next()?, words.next()?);
    altitude.strip_suffix('m')?.parse::<i64>().ok()?;
    let mut clock = time.split(':').map(|part| part.parse::<u64>().ok());
    let (h, m, s) = (clock.next()??, clock.next()??, clock.next()??);
    words.next().is_none().then_some(Told {
        lat_e7: degrees_e7(lat)?,
        lon_e7: degrees_e7(lon)?,
        seconds_of_day: h * 3600 + m * 60 + s,
    })
}

fn degrees_e7(text: &str) -> Option<i64> {
    let degrees: f64 = text.parse().ok()?;
    #[expect(
        clippy::cast_possible_truncation,
        reason = "a rounded latitude or longitude"
    )]
    Some((degrees * 1e7).round() as i64)
}

/// Whether a time of day lies in the cut, give or take `TIME_SLACK_S`, across midnight too.
fn in_cut(seconds_of_day: u64, cut_unix_s: (u64, u64)) -> bool {
    let day = 86_400;
    let (from, to) = cut_unix_s;
    let since = (seconds_of_day + day - (from.saturating_sub(TIME_SLACK_S)) % day) % day;
    since <= to - from + 2 * TIME_SLACK_S
}

pub fn run(
    agent: &AgentClient,
    profile: &Profile,
    profile_text: &str,
    settings: &Settings,
) -> Results {
    let mut results = Results {
        settings: settings.to_json(),
        ..Results::default()
    };
    let Some((blackout_ms, restore_ms)) = blackout(profile) else {
        let detail = "the profile has no blackout and restore in its timeline";
        results.record("profile_cuts_the_link", Verdict::Incomplete, detail);
        return results;
    };
    if !check::apply_profile(&mut results, agent, profile_text) {
        return results;
    }
    let vehicle = AgentClient::new(&settings.vehicle_metrics);
    let Some(mut gcs) = ground_station(&mut results, settings, &vehicle) else {
        return results;
    };
    let started = Instant::now();
    let started_unix_s = unix_ms() / 1000;
    if let Err(e) = agent.post("/timeline/start", "") {
        let detail = format!("cannot start the timeline: {e}");
        results.record("link_cut", Verdict::Incomplete, detail);
        return results;
    }
    let restored = started + Duration::from_millis(restore_ms);
    gcs.pump(restored);
    let during = told(&gcs.seen, |s| s.at < restored);
    let detail = format!("{} sightings arrived during the cut", during.len());
    let verdict = if during.is_empty() {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    results.record("none_during_the_cut", verdict, detail);

    let after = collect(&mut gcs, restored);
    let cut = (
        started_unix_s + blackout_ms / 1000,
        started_unix_s + restore_ms / 1000,
    );
    grade_told(&mut results, &after, cut);
    grade_listing(&mut results, settings, after.len());
    results
}

/// The ground station, once the vehicle is heard and its detector runs.
fn ground_station(
    results: &mut Results,
    settings: &Settings,
    vehicle: &AgentClient,
) -> Option<Gcs> {
    let mut gcs = match Gcs::bind(settings.gcs_listen) {
        Ok(gcs) => gcs,
        Err(e) => {
            let detail = format!("cannot bind {}: {e}", settings.gcs_listen);
            results.record("vehicle_heard", Verdict::Incomplete, detail);
            return None;
        }
    };
    let heard = gcs.pump_until(Instant::now() + HEARTBEAT_DEADLINE, |seen| {
        seen.iter().any(Seen::vehicle_heartbeat)
    });
    if !heard {
        results.record("vehicle_heard", Verdict::Incomplete, "no heartbeat");
        return None;
    }
    let until = Instant::now() + DETECTOR_DEADLINE;
    while Instant::now() < until {
        let reading = check::get_json(vehicle, "/metrics");
        let found = check::counter(&reading, "/detector/people").unwrap_or(0);
        if found > 0 {
            return Some(gcs);
        }
        gcs.pump(Instant::now() + Duration::from_secs(1));
    }
    let detail = "the detector found nobody: is it on (`just regional=1 sim-up`)?";
    results.record("detector_sees_people", Verdict::Incomplete, detail);
    None
}

/// The sightings that arrive from `restored` until the first (at most `FIRST_WITHIN`) and then
/// `QUIET` pass without another.
fn collect(gcs: &mut Gcs, restored: Instant) -> Vec<Told> {
    gcs.pump_until(restored + FIRST_WITHIN, |seen| {
        !told(seen, |_| true).is_empty()
    });
    let mut count = told(&gcs.seen, |s| s.at >= restored).len();
    loop {
        gcs.pump(Instant::now() + QUIET);
        let now = told(&gcs.seen, |s| s.at >= restored).len();
        if now == count {
            break;
        }
        count = now;
    }
    told(&gcs.seen, |s| s.at >= restored)
}

fn told(seen: &[Seen], within: impl Fn(&Seen) -> bool) -> Vec<Told> {
    seen.iter()
        .filter(|s| s.from_vehicle && within(s))
        .filter_map(|s| match s.kind {
            Kind::StatusText { text } => parse(text.to_str().ok()?),
            _ => None,
        })
        .collect()
}

fn grade_told(results: &mut Results, after: &[Told], cut: (u64, u64)) {
    let detail = format!("{} sightings arrived after the restore", after.len());
    let verdict = if after.is_empty() {
        Verdict::Fail
    } else {
        Verdict::Pass
    };
    results.record("sightings_arrive", verdict, detail);
    let Some(first) = after.first() else {
        return;
    };
    let same_place = after.iter().all(|t| {
        (t.lat_e7 - first.lat_e7).abs() <= SAME_PLACE_E7
            && (t.lon_e7 - first.lon_e7).abs() <= SAME_PLACE_E7
    });
    let placed = first.lat_e7 != 0 && first.lon_e7 != 0 && same_place;
    let detail = format!(
        "first at {}, {} (degrees times 1e7)",
        first.lat_e7, first.lon_e7
    );
    results.record("positions_agree", verdict_of(placed), detail);
    let seen_in_cut = after.iter().all(|t| in_cut(t.seconds_of_day, cut));
    let detail = "every time of day lies inside the cut, give or take 3 s";
    results.record("seen_in_the_cut", verdict_of(seen_in_cut), detail);
}

/// The listing through the ground's job API holds what was told, each with a snapshot, and one
/// of the snapshots can be fetched.
fn grade_listing(results: &mut Results, settings: &Settings, told: usize) {
    let client = Client::new(&settings.ground_api, CLIENT_TIMEOUT)
        .with_token(crate::client::token_for(&settings.ground_api));
    let request = JobRequest {
        kind: JobKind::Sightings,
        at_capture_ms: 0,
        crop: None,
        max_bytes: 4096,
        expires_in_ms: JOB_LIFE_MS,
    };
    let listing = client
        .request(&request)
        .and_then(|id| {
            let listing = client.list(&id, JOB_DEADLINE);
            let _ = client.cancel(&id);
            listing
        })
        .map_err(|e| e.to_string());
    let listing = match listing {
        Ok(listing) => listing,
        Err(e) => {
            results.record("listing_received", Verdict::Incomplete, e);
            return;
        }
    };
    let complete = told < MAX_LISTED && listing.sightings.len() == told;
    let detail = format!("{} listed, {told} told", listing.sightings.len());
    results.record("listing_matches_told", verdict_of(complete), detail);
    let all_sent = listing.sightings.iter().all(|l| l.sighting.sent);
    results.record(
        "listed_as_told",
        verdict_of(all_sent),
        "every listed sighting is marked told",
    );
    let with_snapshot = listing.sightings.iter().all(|l| l.snapshot.is_some());
    let detail = "every listed sighting names a snapshot within reach";
    results.record("snapshot_named", verdict_of(with_snapshot), detail);
    fetch_first(results, &client, &listing);
}

fn fetch_first(results: &mut Results, client: &Client, listing: &Listing) {
    let Some(snapshot) = listing.sightings.iter().find_map(|l| l.snapshot) else {
        return;
    };
    let request = JobRequest {
        kind: JobKind::Snapshot,
        at_capture_ms: snapshot.capture_ms,
        crop: None,
        max_bytes: JOB_MAX_BYTES,
        expires_in_ms: JOB_LIFE_MS,
    };
    let fetched = client
        .request(&request)
        .and_then(|id| {
            let fetched = client.fetch(&id, JOB_DEADLINE);
            let _ = client.cancel(&id);
            fetched
        })
        .map_err(|e| e.to_string());
    match fetched {
        Ok(f) => {
            let right = f.meta.capture_ms == snapshot.capture_ms;
            let detail = format!(
                "fetched the snapshot of {} ms, {} bytes",
                f.meta.capture_ms,
                f.jpeg.len()
            );
            results.record("snapshot_fetched", verdict_of(right), detail);
        }
        Err(e) => {
            results.record("snapshot_fetched", Verdict::Fail, e);
        }
    }
    thread::sleep(Duration::from_millis(100));
}

fn verdict_of(holds: bool) -> Verdict {
    if holds { Verdict::Pass } else { Verdict::Fail }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_status_text_reads_back_as_a_position_and_a_time_and_nothing_else_does() {
        let told = parse("person -59.32931 18.06862 -2m 14:02:31").unwrap();
        assert_eq!(
            told,
            Told {
                lat_e7: -593_293_100,
                lon_e7: 180_686_200,
                seconds_of_day: 50_551
            }
        );
        for text in [
            "",
            "person 1 2 3m",
            "Arm: not ready",
            "person 1.0 2.0 3 14:02:31",
            "person 1.0 2.0 3m 14:02:31 x",
        ] {
            assert_eq!(parse(text), None, "{text:?}");
        }
    }

    #[test]
    fn a_time_of_day_lies_in_the_cut_across_midnight_too() {
        let cut = (86_400 - 10, 86_400 + 20);
        let cases = [
            (86_400 - 12, true),
            (5, true),
            (30, false),
            (86_400 - 20, false),
            (0, true),
        ];
        for (second, expected) in cases {
            assert_eq!(in_cut(second % 86_400, cut), expected, "{second}");
        }
    }
}
