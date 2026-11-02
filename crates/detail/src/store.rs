//! The vehicle's snapshot store: one directory of JPEGs named `<capture_ms>-<frame_id>.jpg`. A file
//! is written as `.tmp` and renamed, so a crash leaves either a whole snapshot or a `.tmp` the next
//! start deletes; the index is the directory itself, scanned at start. Retention is decided by
//! `sat_teleop_policy::retention`.

use std::fs::{self, File};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

use serde::{Deserialize, Serialize};

use sat_teleop_policy::retention::{self, Limits, Object};

use crate::DetailError;

/// The largest snapshot the store takes.
const MAX_SNAPSHOT_BYTES: usize = 16 * 1024 * 1024;
const SUFFIX: &str = ".jpg";
const TMP_SUFFIX: &str = ".jpg.tmp";

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
    /// `.tmp` files a crash left, deleted.
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
    let stem = name.strip_suffix(SUFFIX)?;
    let (capture, frame) = stem.split_once('-')?;
    let digits = |s: &str| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit());
    if !digits(capture) || !digits(frame) {
        return None;
    }
    let (capture, frame) = (capture.parse().ok()?, frame.parse().ok()?);
    (file_name(capture, frame) == name).then_some((capture, frame))
}

fn file_name(capture_ms: u64, frame_id: u32) -> String {
    format!("{capture_ms}-{frame_id}{SUFFIX}")
}

/// Where `entry` lies in `dir`: built from its numbers only.
pub fn entry_path(dir: &Path, entry: Entry) -> PathBuf {
    dir.join(file_name(entry.capture_ms, entry.frame_id))
}

/// What a snapshot directory holds, read without changing it.
#[derive(Debug, Default)]
pub struct Listing {
    /// The snapshots, oldest first.
    pub entries: Vec<Entry>,
    /// Files a crash may have left half written.
    pub tmp_files: Vec<PathBuf>,
    /// Names that are neither, and anything that is not a plain file.
    pub ignored: usize,
}

/// Reads `dir`: valid snapshot names are entries, `.jpg.tmp` files are leftovers, the rest is
/// ignored. Only plain files count.
pub fn list(dir: &Path) -> std::io::Result<Listing> {
    let mut listing = Listing::default();
    for item in fs::read_dir(dir)?.flatten() {
        let name = item.file_name();
        let name = name.to_string_lossy();
        let is_file = item.file_type().is_ok_and(|t| t.is_file());
        if is_file && name.ends_with(TMP_SUFFIX) {
            listing.tmp_files.push(item.path());
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
        });
    }
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
        for tmp in &listing.tmp_files {
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

    /// Writes a snapshot, then deletes what the limits say. A failed write leaves no `.tmp`.
    pub fn put(
        &mut self,
        capture_ms: u64,
        frame_id: u32,
        jpeg: &[u8],
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

    /// The newest snapshot captured at or before `at_capture_ms`, if the store holds one.
    pub fn nearest_at_or_before(&self, at_capture_ms: u64) -> Option<Entry> {
        let end = self
            .kept
            .partition_point(|k| k.entry.capture_ms <= at_capture_ms);
        end.checked_sub(1)
            .and_then(|last| self.kept.get(last))
            .map(|k| k.entry)
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
            })
            .collect();
        let doomed = retention::evict(&objects, self.config.limits, free, now_unix_ms, now);
        let mut keep = vec![true; self.kept.len()];
        for index in doomed {
            let Some(kept) = self.kept.get(index) else {
                continue;
            };
            let removed = fs::remove_file(self.path(kept.entry));
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
