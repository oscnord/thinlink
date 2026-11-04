//! The vehicle's snapshot store: one directory of JPEGs named `<capture_ms>-<frame_id>.jpg`. A file
//! is written as `.tmp` and renamed, so a crash leaves either a whole snapshot or a `.tmp` the next
//! start deletes; the index is the directory itself, scanned at start. A snapshot taken with a
//! fresh position, or while the ground was out of reach, has a small
//! `<capture_ms>-<frame_id>.pos` beside it; one without is just the JPEG, as every snapshot was
//! before positions. Retention is decided by `thinlink_policy::retention`, which keeps the
//! out-of-reach snapshots until they are fetched.

use std::collections::HashMap;
use std::fs::{self, File};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

use serde::{Deserialize, Serialize};

use thinlink_policy::position::Position;
use thinlink_policy::retention::{self, Limits, Object};

use crate::DetailError;

/// The largest snapshot the store takes.
const MAX_SNAPSHOT_BYTES: usize = 16 * 1024 * 1024;
const SUFFIX: &str = ".jpg";
const TMP_SUFFIX: &str = ".jpg.tmp";
const POS_SUFFIX: &str = ".pos";
const POS_TMP_SUFFIX: &str = ".pos.tmp";
/// A position file is a few dozen bytes; anything larger is not one.
const MAX_POS_BYTES: u64 = 1024;

/// `[store]` of the vehicle config.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StoreSettings {
    /// The directory of snapshots: a volume of its own, so a full store cannot fill anything else.
    pub dir: PathBuf,
    pub max_bytes: u64,
    pub max_age_s: u64,
    /// The volume keeps this much free.
    pub min_free_bytes: u64,
    /// Snapshots further apart than this have a gap between them.
    pub gap_after_ms: u64,
    /// Snapshots taken while the ground was not heard are kept against the limits up to this
    /// percentage of `max_bytes`, the newest first.
    #[serde(default = "default_outage_share_percent")]
    pub outage_share_percent: u8,
    /// They are kept this long, unless fetched sooner.
    #[serde(default = "default_outage_keep_s")]
    pub outage_keep_s: u64,
}

fn default_outage_share_percent() -> u8 {
    50
}

fn default_outage_keep_s() -> u64 {
    3600
}

impl StoreSettings {
    pub fn validate(self) -> Result<StoreConfig, DetailError> {
        let rules = [
            (self.dir.is_absolute(), "dir must be an absolute path"),
            (self.max_bytes > 0, "max_bytes must be above 0"),
            (
                (1..=31_536_000).contains(&self.max_age_s),
                "max_age_s must be 1-31536000",
            ),
            (
                (1..=3_600_000).contains(&self.gap_after_ms),
                "gap_after_ms must be 1-3600000",
            ),
            (
                self.outage_share_percent <= 100,
                "outage_share_percent must be 0-100",
            ),
            (
                (1..=31_536_000).contains(&self.outage_keep_s),
                "outage_keep_s must be 1-31536000",
            ),
        ];
        if let Some((_, rule)) = rules.iter().find(|(holds, _)| !holds) {
            return Err(DetailError::Config(format!("[store]: {rule}")));
        }
        Ok(StoreConfig {
            dir: self.dir,
            limits: Limits {
                max_bytes: self.max_bytes,
                max_age_ms: self.max_age_s * 1000,
                min_free_bytes: self.min_free_bytes,
                outage_share_percent: self.outage_share_percent,
                outage_keep_ms: self.outage_keep_s * 1000,
            },
            gap_after_ms: self.gap_after_ms,
        })
    }
}

#[derive(Debug, Clone)]
pub struct StoreConfig {
    pub dir: PathBuf,
    pub limits: Limits,
    /// Snapshots further apart than this have a gap between them.
    pub gap_after_ms: u64,
}

/// One retained snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry {
    pub capture_ms: u64,
    pub frame_id: u32,
    pub bytes: u64,
    /// Where the vehicle was when it was taken; `None` when the autopilot's position was stale.
    pub position: Option<Position>,
    /// Taken while the ground was not heard and not fetched since.
    pub outage: bool,
}

/// What a snapshot was taken with: where the vehicle was and whether the ground was out of
/// reach. It is the contents of the snapshot's `.pos` file.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Tag {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position: Option<Position>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub outage: bool,
}

impl Tag {
    fn is_empty(self) -> bool {
        self == Self::default()
    }
}

#[derive(Debug, Clone, Copy)]
struct Kept {
    entry: Entry,
    pinned_until: Option<Instant>,
}

/// What the directory scan at start found.
#[derive(Debug, Clone, Copy, Default, Serialize)]
pub struct Recovery {
    pub kept: usize,
    /// `.tmp` files and position files without a snapshot that a crash left, deleted.
    pub tmp_removed: usize,
    /// Names that are not snapshots, left alone and not indexed.
    pub ignored: usize,
}

/// Where snapshots are missing: two consecutive ones further apart than the store's gap limit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Gap {
    pub from_ms: u64,
    pub to_ms: u64,
}

#[derive(Debug, Clone, Serialize)]
pub struct StoreSnapshot {
    pub count: usize,
    pub bytes: u64,
    pub oldest_ms: Option<u64>,
    pub newest_ms: Option<u64>,
    pub written: u64,
    pub evicted: u64,
    pub gaps: usize,
    pub largest_gap_ms: u64,
    pub recovery: Recovery,
}

#[derive(Debug)]
pub struct Store {
    config: StoreConfig,
    /// Oldest first.
    kept: Vec<Kept>,
    written: u64,
    evicted: u64,
    recovery: Recovery,
}

/// `<capture_ms>-<frame_id>.jpg` as numbers; `None` for any other name.
fn parse_name(name: &str) -> Option<(u64, u32)> {
    parse_stem(name, SUFFIX)
}

/// `<capture_ms>-<frame_id><suffix>` as numbers; `None` for any other name.
fn parse_stem(name: &str, suffix: &str) -> Option<(u64, u32)> {
    let stem = name.strip_suffix(suffix)?;
    let (capture, frame) = stem.split_once('-')?;
    let digits = |s: &str| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit());
    if !digits(capture) || !digits(frame) {
        return None;
    }
    let (capture, frame) = (capture.parse().ok()?, frame.parse().ok()?);
    (format!("{capture}-{frame}{suffix}") == name).then_some((capture, frame))
}

fn file_name(capture_ms: u64, frame_id: u32) -> String {
    format!("{capture_ms}-{frame_id}{SUFFIX}")
}

/// Where `entry` lies in `dir`: built from its numbers only.
pub fn entry_path(dir: &Path, entry: Entry) -> PathBuf {
    dir.join(file_name(entry.capture_ms, entry.frame_id))
}

fn position_path(dir: &Path, capture_ms: u64, frame_id: u32) -> PathBuf {
    dir.join(format!("{capture_ms}-{frame_id}{POS_SUFFIX}"))
}

/// The tag in the file at `path`, if it is a plain small file holding one.
fn read_tag(path: &Path) -> Option<Tag> {
    let small = fs::metadata(path).is_ok_and(|m| m.is_file() && m.len() <= MAX_POS_BYTES);
    let text = fs::read_to_string(path).ok().filter(|_| small)?;
    serde_json::from_str(&text).ok()
}

/// What a snapshot directory holds, read without changing it.
#[derive(Debug, Default)]
pub struct Listing {
    /// The snapshots, oldest first.
    pub entries: Vec<Entry>,
    /// Files a crash may have left half written.
    pub tmp_files: Vec<PathBuf>,
    /// Position files whose snapshot is not there.
    pub orphans: Vec<PathBuf>,
    /// Names that are none of these, and anything that is not a plain file.
    pub ignored: usize,
}

/// Reads `dir`: valid snapshot names are entries, `.jpg.tmp` files are leftovers, the rest is
/// ignored. Only plain files count.
pub fn list(dir: &Path) -> std::io::Result<Listing> {
    let mut listing = Listing::default();
    let mut positions: HashMap<(u64, u32), (Tag, PathBuf)> = HashMap::new();
    for item in fs::read_dir(dir)?.flatten() {
        let name = item.file_name();
        let name = name.to_string_lossy();
        let is_file = item.file_type().is_ok_and(|t| t.is_file());
        if is_file && (name.ends_with(TMP_SUFFIX) || name.ends_with(POS_TMP_SUFFIX)) {
            listing.tmp_files.push(item.path());
            continue;
        }
        if let Some(key) = parse_stem(&name, POS_SUFFIX).filter(|_| is_file) {
            match read_tag(&item.path()) {
                Some(tag) => {
                    positions.insert(key, (tag, item.path()));
                }
                None => listing.ignored += 1,
            }
            continue;
        }
        let parsed = parse_name(&name).filter(|_| is_file);
        let (Some((capture_ms, frame_id)), Ok(meta)) = (parsed, item.metadata()) else {
            listing.ignored += 1;
            continue;
        };
        listing.entries.push(Entry {
            capture_ms,
            frame_id,
            bytes: meta.len(),
            position: None,
            outage: false,
        });
    }
    for entry in &mut listing.entries {
        let found = positions.remove(&(entry.capture_ms, entry.frame_id));
        let tag = found.map(|(tag, _)| tag).unwrap_or_default();
        (entry.position, entry.outage) = (tag.position, tag.outage);
    }
    listing.orphans = positions.into_values().map(|(_, path)| path).collect();
    listing.entries.sort_by_key(|e| (e.capture_ms, e.frame_id));
    Ok(listing)
}

impl Store {
    /// Opens `config.dir`, creating it: deletes the `.tmp` files a crash left, indexes the
    /// snapshots, ignores every other name, and applies the retention limits.
    pub fn open(config: StoreConfig, now_unix_ms: u64, now: Instant) -> Result<Self, DetailError> {
        let dir = &config.dir;
        fs::create_dir_all(dir)
            .map_err(|e| DetailError::io(format!("cannot create {}", dir.display()), e))?;
        let listing =
            list(dir).map_err(|e| DetailError::io(format!("cannot read {}", dir.display()), e))?;
        let mut recovery = Recovery {
            ignored: listing.ignored,
            ..Recovery::default()
        };
        for tmp in listing.tmp_files.iter().chain(&listing.orphans) {
            recovery.tmp_removed += usize::from(fs::remove_file(tmp).is_ok());
        }
        let mut kept: Vec<Kept> = listing
            .entries
            .into_iter()
            .map(|entry| Kept {
                entry,
                pinned_until: None,
            })
            .collect();
        kept.sort_by_key(|k| (k.entry.capture_ms, k.entry.frame_id));
        recovery.kept = kept.len();
        let mut store = Self {
            config,
            kept,
            written: 0,
            evicted: 0,
            recovery,
        };
        store.enforce(now_unix_ms, now)?;
        Ok(store)
    }

    /// Writes a snapshot with its tag, then deletes what the limits say. A failed write leaves
    /// no `.tmp`.
    pub fn put(
        &mut self,
        capture_ms: u64,
        frame_id: u32,
        jpeg: &[u8],
        tag: Tag,
        now_unix_ms: u64,
        now: Instant,
    ) -> Result<(), DetailError> {
        if jpeg.is_empty() || jpeg.len() > MAX_SNAPSHOT_BYTES {
            return Err(DetailError::Config(format!(
                "a snapshot of {} bytes is not stored",
                jpeg.len()
            )));
        }
        let name = file_name(capture_ms, frame_id);
        let path = self.config.dir.join(&name);
        let tmp = self.config.dir.join(format!("{name}.tmp"));
        self.write_tag(capture_ms, frame_id, tag)?;
        write_whole(&tmp, jpeg).map_err(|e| {
            let _ = fs::remove_file(&tmp);
            DetailError::io(format!("cannot write {}", path.display()), e)
        })?;
        fs::rename(&tmp, &path).map_err(|e| {
            let _ = fs::remove_file(&tmp);
            DetailError::io(format!("cannot rename {}", tmp.display()), e)
        })?;
        let entry = Entry {
            capture_ms,
            frame_id,
            bytes: jpeg.len() as u64,
            position: tag.position,
            outage: tag.outage,
        };
        self.kept
            .retain(|k| (k.entry.capture_ms, k.entry.frame_id) != (capture_ms, frame_id));
        let at = self
            .kept
            .partition_point(|k| (k.entry.capture_ms, k.entry.frame_id) <= (capture_ms, frame_id));
        self.kept.insert(
            at,
            Kept {
                entry,
                pinned_until: None,
            },
        );
        self.written += 1;
        self.enforce(now_unix_ms, now)
    }

    /// Writes the tag file before the snapshot, so a snapshot that has a tag never shows up
    /// without it; an empty tag removes the file an earlier snapshot of the same name left.
    fn write_tag(&self, capture_ms: u64, frame_id: u32, tag: Tag) -> Result<(), DetailError> {
        let path = position_path(&self.config.dir, capture_ms, frame_id);
        if tag.is_empty() {
            let _ = fs::remove_file(&path);
            return Ok(());
        }
        let tmp = path.with_extension("pos.tmp");
        let body = serde_json::to_vec(&tag)
            .map_err(|e| DetailError::Config(format!("cannot encode a position: {e}")))?;
        let written = write_whole(&tmp, &body).and_then(|()| fs::rename(&tmp, &path));
        written.map_err(|e| {
            let _ = fs::remove_file(&tmp);
            DetailError::io(format!("cannot write {}", path.display()), e)
        })
    }

    /// The outage snapshots captured at or after `from_ms`, oldest first, at most `limit`.
    pub fn outage_from(&self, from_ms: u64, limit: usize) -> Vec<Entry> {
        self.kept
            .iter()
            .map(|k| k.entry)
            .filter(|e| e.outage && e.capture_ms >= from_ms)
            .take(limit)
            .collect()
    }

    /// Ends an outage snapshot's keeping: it was fetched, so retention treats it as any other.
    pub fn release(&mut self, entry: Entry) {
        let key = (entry.capture_ms, entry.frame_id);
        let found = self
            .kept
            .iter_mut()
            .find(|k| (k.entry.capture_ms, k.entry.frame_id) == key);
        let Some(kept) = found else {
            return;
        };
        kept.entry.outage = false;
        let tag = Tag {
            position: kept.entry.position,
            outage: false,
        };
        // A tag that cannot be rewritten only means the snapshot is kept again after a restart.
        let _ = self.write_tag(entry.capture_ms, entry.frame_id, tag);
    }

    /// The newest snapshot captured at or before `at_capture_ms`, if the store holds one.
    pub fn nearest_at_or_before(&self, at_capture_ms: u64) -> Option<Entry> {
        let end = self
            .kept
            .partition_point(|k| k.entry.capture_ms <= at_capture_ms);
        end.checked_sub(1)
            .and_then(|last| self.kept.get(last))
            .map(|k| k.entry)
    }

    /// The snapshot captured nearest to `at_ms` and within `within_ms` of it, if the store holds one.
    pub fn nearest(&self, at_ms: u64, within_ms: u64) -> Option<Entry> {
        let after = self.kept.partition_point(|k| k.entry.capture_ms < at_ms);
        let before = after.checked_sub(1).and_then(|last| self.kept.get(last));
        [before, self.kept.get(after)]
            .into_iter()
            .flatten()
            .map(|k| k.entry)
            .filter(|e| e.capture_ms.abs_diff(at_ms) <= within_ms)
            .min_by_key(|e| e.capture_ms.abs_diff(at_ms))
    }

    /// Keeps `entry` from retention until `until`.
    pub fn pin(&mut self, entry: Entry, until: Instant) {
        let key = (entry.capture_ms, entry.frame_id);
        let found = self
            .kept
            .iter_mut()
            .find(|k| (k.entry.capture_ms, k.entry.frame_id) == key);
        if let Some(kept) = found {
            kept.pinned_until = Some(until);
        }
    }

    /// The file's path from the numbers of a stored entry, never from request data.
    pub fn path(&self, entry: Entry) -> PathBuf {
        entry_path(&self.config.dir, entry)
    }

    pub fn gaps(&self) -> Vec<Gap> {
        self.kept
            .windows(2)
            .filter_map(|pair| {
                let (a, b) = (
                    pair.first()?.entry.capture_ms,
                    pair.get(1)?.entry.capture_ms,
                );
                (b.saturating_sub(a) > self.config.gap_after_ms).then_some(Gap {
                    from_ms: a,
                    to_ms: b,
                })
            })
            .collect()
    }

    pub fn snapshot(&self) -> StoreSnapshot {
        let gaps = self.gaps();
        StoreSnapshot {
            count: self.kept.len(),
            bytes: self.kept.iter().map(|k| k.entry.bytes).sum(),
            oldest_ms: self.kept.first().map(|k| k.entry.capture_ms),
            newest_ms: self.kept.last().map(|k| k.entry.capture_ms),
            written: self.written,
            evicted: self.evicted,
            gaps: gaps.len(),
            largest_gap_ms: gaps
                .iter()
                .map(|g| g.to_ms.saturating_sub(g.from_ms))
                .max()
                .unwrap_or(0),
            recovery: self.recovery,
        }
    }

    fn enforce(&mut self, now_unix_ms: u64, now: Instant) -> Result<(), DetailError> {
        let free = free_bytes(&self.config.dir)?;
        let objects: Vec<Object> = self
            .kept
            .iter()
            .map(|k| Object {
                capture_ms: k.entry.capture_ms,
                bytes: k.entry.bytes,
                pinned_until: k.pinned_until,
                outage: k.entry.outage,
            })
            .collect();
        let doomed = retention::evict(&objects, self.config.limits, free, now_unix_ms, now);
        let mut keep = vec![true; self.kept.len()];
        for index in doomed {
            let Some(kept) = self.kept.get(index) else {
                continue;
            };
            let removed = fs::remove_file(self.path(kept.entry));
            let _ = fs::remove_file(position_path(
                &self.config.dir,
                kept.entry.capture_ms,
                kept.entry.frame_id,
            ));
            // A file that is already gone is not kept in the index either.
            let gone = removed.is_ok() || !self.path(kept.entry).exists();
            if let Some(flag) = keep.get_mut(index) {
                *flag = !gone;
            }
            self.evicted += u64::from(gone);
        }
        let mut flags = keep.into_iter();
        self.kept.retain(|_| flags.next().unwrap_or(true));
        Ok(())
    }
}

fn write_whole(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let mut file = File::create(path)?;
    file.write_all(bytes)?;
    file.sync_data()
}

/// Bytes available to an unprivileged writer on the volume holding `dir`.
fn free_bytes(dir: &Path) -> Result<u64, DetailError> {
    let stat = rustix::fs::statvfs(dir)
        .map_err(|e| DetailError::io(format!("cannot stat {}", dir.display()), e.into()))?;
    Ok(stat.f_bavail.saturating_mul(stat.f_frsize))
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicU64, Ordering};

    use super::*;

    const JPEG: &[u8] = b"not decoded here";

    fn config() -> StoreConfig {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let dir = std::env::temp_dir().join(format!(
            "thinlink-store-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        StoreConfig {
            dir,
            limits: Limits {
                max_bytes: 1 << 20,
                max_age_ms: 3_600_000,
                min_free_bytes: 0,
                outage_share_percent: 50,
                outage_keep_ms: 3_600_000,
            },
            gap_after_ms: 1500,
        }
    }

    fn fix(lat_e7: i32) -> Position {
        Position {
            lat_e7,
            lon_e7: 180_000_000,
            rel_alt_mm: 12_000,
            heading_cdeg: Some(9000),
        }
    }

    fn found(store: &Store, at: u64) -> Entry {
        store.nearest_at_or_before(at).unwrap()
    }

    #[test]
    fn the_nearest_snapshot_may_lie_either_side_within_reach() {
        let (now_ms, now) = (1_000_000, Instant::now());
        let config = config();
        let mut store = Store::open(config.clone(), now_ms, now).unwrap();
        for (capture_ms, frame_id) in [(999_000, 1), (999_800, 2)] {
            let put = store.put(capture_ms, frame_id, JPEG, Tag::default(), now_ms, now);
            put.unwrap();
        }
        let cases = [
            ("after is nearer", 999_600, 500, Some(2)),
            ("before is nearer", 999_300, 500, Some(1)),
            ("exactly", 999_800, 0, Some(2)),
            ("out of reach", 999_400, 300, None),
            ("past the newest", 1_000_100, 500, Some(2)),
        ];
        for (name, at, within, expected) in cases {
            let found = store.nearest(at, within).map(|e| e.frame_id);
            assert_eq!(found, expected, "{name}");
        }
        fs::remove_dir_all(&config.dir).unwrap();
    }

    #[test]
    fn tags_survive_a_restart_and_snapshots_without_one_stay_readable() {
        let config = config();
        let (now_ms, now) = (1_000_000, Instant::now());
        let mut store = Store::open(config.clone(), now_ms, now).unwrap();
        let at = |position, outage| Tag { position, outage };
        let puts = [
            (999_000, 1, at(Some(fix(590_000_000)), false)),
            (999_500, 2, at(None, true)),
            (999_800, 3, at(Some(fix(590_000_001)), true)),
            (999_900, 4, Tag::default()),
        ];
        for (capture_ms, frame_id, tag) in puts {
            store
                .put(capture_ms, frame_id, JPEG, tag, now_ms, now)
                .unwrap();
        }
        // A snapshot from before positions existed: the JPEG alone.
        fs::write(config.dir.join("998000-0.jpg"), JPEG).unwrap();
        // A tag file whose snapshot a crash took.
        fs::write(config.dir.join("1-1.pos"), br#"{"outage":true}"#).unwrap();
        store.release(found(&store, 999_800));
        drop(store);

        let mut store = Store::open(config.clone(), now_ms, now).unwrap();
        let tags: Vec<_> = [998_000, 999_000, 999_500, 999_800, 999_900]
            .map(|t| found(&store, t))
            .map(|e| (e.capture_ms, e.position, e.outage))
            .to_vec();
        assert_eq!(
            tags,
            [
                (998_000, None, false),
                (999_000, Some(fix(590_000_000)), false),
                (999_500, None, true),
                (999_800, Some(fix(590_000_001)), false),
                (999_900, None, false),
            ]
        );
        assert_eq!(store.snapshot().recovery.tmp_removed, 1);
        assert!(!config.dir.join("1-1.pos").exists());
        let listed: Vec<_> = store.outage_from(0, 8).iter().map(|e| e.frame_id).collect();
        assert_eq!(listed, [2]);
        store.release(found(&store, 999_500));
        assert!(!config.dir.join("999500-2.pos").exists());
        fs::remove_dir_all(&config.dir).unwrap();
    }
}
