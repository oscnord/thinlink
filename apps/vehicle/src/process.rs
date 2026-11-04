//! The vehicle process's own CPU time and memory, read from `/proc` on Linux.

use std::fs;

use serde::Serialize;

/// Clock ticks per second of the `utime` and `stime` fields: 100 on every Linux the vehicle runs on.
const TICKS_PER_SECOND: u64 = 100;

#[derive(Debug, PartialEq, Eq, Serialize)]
pub struct ProcessSnapshot {
    /// CPU time of all threads, user and system, since the process started.
    pub cpu_ms: u64,
    pub rss_kb: u64,
}

/// `None` where `/proc` has no such files or they do not parse.
pub fn snapshot() -> Option<ProcessSnapshot> {
    let stat = fs::read_to_string("/proc/self/stat").ok()?;
    let status = fs::read_to_string("/proc/self/status").ok()?;
    parse(&stat, &status)
}

fn parse(stat: &str, status: &str) -> Option<ProcessSnapshot> {
    // The command name is in parentheses and may hold spaces: the fields count from after it, where
    // the state is field 3, `utime` field 14 and `stime` field 15.
    let fields: Vec<&str> = stat.rsplit_once(')')?.1.split_whitespace().collect();
    let ticks = |index: usize| fields.get(index)?.parse::<u64>().ok();
    let cpu_ms = (ticks(11)? + ticks(12)?) * 1000 / TICKS_PER_SECOND;
    let rss_kb = status
        .lines()
        .find_map(|line| line.strip_prefix("VmRSS:"))?
        .split_whitespace()
        .next()?
        .parse()
        .ok()?;
    Some(ProcessSnapshot { cpu_ms, rss_kb })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_cpu_time_past_a_command_name_with_spaces() {
        let stat =
            "42 (thin link) S 1 42 42 0 -1 4194560 100 0 0 0 250 50 0 0 20 0 8 0 1000 5000 1200";
        let status = "Name:\tsat\nVmRSS:\t   23456 kB\nThreads:\t8\n";
        assert_eq!(
            parse(stat, status),
            Some(ProcessSnapshot {
                cpu_ms: 3000,
                rss_kb: 23456
            })
        );
    }
}
