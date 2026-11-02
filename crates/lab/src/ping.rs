//! iputils `ping` as a pure argv builder and output parser.

use std::time::Duration;

use crate::command::Command;

const INTERVAL_S: &str = "0.2";

/// One echo reply.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reply {
    /// Receive time as printed by `ping -D` (UNIX seconds).
    pub unix_s: f64,
    pub seq: u32,
    pub rtt_ms: f64,
}

/// What one `ping` run saw.
#[derive(Debug, Clone, PartialEq)]
pub struct Probe {
    /// Echo requests sent, from the statistics line; 0 when ping printed none (it could not start).
    pub transmitted: u32,
    pub replies: Vec<Reply>,
}

/// Sends exactly `probes` echo requests 0.2 s apart and waits up to `timeout` for replies.
/// There is deliberately no `-w` deadline: with one, iputils stops at the first ICMP error and
/// cuts the stream short. Fewer replies (or none) is an outcome to grade, so the exit status is
/// tolerated. A probe dropped by a local firewall still counts as transmitted.
pub fn command(target: &str, probes: u32, timeout: Duration) -> Command {
    Command::from_args(vec![
        "ping".to_owned(),
        "-D".to_owned(),
        "-n".to_owned(),
        "-i".to_owned(),
        INTERVAL_S.to_owned(),
        "-c".to_owned(),
        probes.to_string(),
        "-W".to_owned(),
        timeout.as_secs().max(1).to_string(),
        target.to_owned(),
    ])
    .tolerant()
}

/// Reply lines look like `[1793483622.704641] 64 bytes from 10.77.0.2: icmp_seq=1 ttl=63 time=617 ms`,
/// the statistics line like `6 packets transmitted, 2 received, ...`. Everything else (errors,
/// duplicates, malformed numbers) is skipped.
pub fn parse(output: &str) -> Probe {
    Probe {
        transmitted: output.lines().find_map(transmitted).unwrap_or(0),
        replies: output.lines().filter_map(parse_reply).collect(),
    }
}

fn transmitted(line: &str) -> Option<u32> {
    line.split_once(" packets transmitted")?
        .0
        .trim()
        .parse()
        .ok()
}

fn parse_reply(line: &str) -> Option<Reply> {
    let (stamp, rest) = line.strip_prefix('[')?.split_once("] ")?;
    if !rest.contains(" bytes from ") || rest.contains("DUP!") {
        return None;
    }
    let unix_s: f64 = stamp.parse().ok()?;
    let seq: u32 = field(rest, "icmp_seq=")?.parse().ok()?;
    let rtt_ms: f64 = field(rest, "time=")?.parse().ok()?;
    let sane = unix_s.is_finite() && rtt_ms.is_finite() && rtt_ms >= 0.0;
    sane.then_some(Reply {
        unix_s,
        seq,
        rtt_ms,
    })
}

fn field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    line.split_once(key)?.1.split_whitespace().next()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_probes_and_keeps_only_well_formed_replies() {
        let output = "\
PING 10.77.0.2 (10.77.0.2) 56(84) bytes of data.
[1793483622.704641] 64 bytes from 10.77.0.2: icmp_seq=1 ttl=63 time=617 ms
[1793483622.904641] 64 bytes from 10.77.0.2: icmp_seq=2 ttl=63 time=0.481 ms
[1793483622.904700] 64 bytes from 10.77.0.2: icmp_seq=2 ttl=63 time=0.490 ms (DUP!)
[1793483623.104641] From 10.77.0.1 icmp_seq=3 Destination Host Unreachable
[1793483623.304641] 64 bytes from 10.77.0.2: icmp_seq=4 ttl=63 time=nan ms
[1793483623.404641] 64 bytes from 10.77.0.2: icmp_seq=x ttl=63 time=1 ms
64 bytes from 10.77.0.2: icmp_seq=5 ttl=63 time=1 ms
[1793483623.504641] 64 bytes from 10.77.0.2: icmp_seq=6 ttl=63 time=-1 ms

--- 10.77.0.2 ping statistics ---
6 packets transmitted, 2 received, 66% packet loss, time 1005ms
";
        let probe = parse(output);
        assert_eq!(probe.transmitted, 6);
        assert_eq!(
            probe.replies,
            [
                Reply {
                    unix_s: 1_793_483_622.704_641,
                    seq: 1,
                    rtt_ms: 617.0
                },
                Reply {
                    unix_s: 1_793_483_622.904_641,
                    seq: 2,
                    rtt_ms: 0.481
                },
            ]
        );
    }
}
