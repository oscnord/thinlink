use std::fs;

use serde::{Deserialize, Serialize};

#[derive(Debug, thiserror::Error)]
pub enum CountersError {
    #[error("tc statistics do not parse: {0}")]
    Json(#[from] serde_json::Error),
    #[error("cannot read {path}: {source}")]
    Read {
        path: String,
        source: std::io::Error,
    },
    #[error("{path} does not hold a counter")]
    NotANumber { path: String },
}

#[derive(Debug, PartialEq, Eq, Deserialize, Serialize)]
pub struct QdiscStats {
    pub kind: String,
    pub handle: Option<String>,
    pub parent: Option<String>,
    #[serde(default)]
    pub bytes: u64,
    #[serde(default)]
    pub packets: u64,
    #[serde(default)]
    pub drops: u64,
    #[serde(default)]
    pub overlimits: u64,
    #[serde(default)]
    pub backlog: u64,
}

/// Parses the output of `tc -s -j qdisc show dev <dev>`.
pub fn parse_qdisc_stats(tc_json: &str) -> Result<Vec<QdiscStats>, CountersError> {
    Ok(serde_json::from_str(tc_json)?)
}

#[derive(Debug, Serialize)]
pub struct NetStats {
    pub rx_bytes: u64,
    pub rx_packets: u64,
    pub rx_dropped: u64,
    pub tx_bytes: u64,
    pub tx_packets: u64,
    pub tx_dropped: u64,
}

/// `dev` must be a validated interface name.
pub fn read_net_stats(dev: &str) -> Result<NetStats, CountersError> {
    let read = |name: &str| read_counter(&format!("/sys/class/net/{dev}/statistics/{name}"));
    Ok(NetStats {
        rx_bytes: read("rx_bytes")?,
        rx_packets: read("rx_packets")?,
        rx_dropped: read("rx_dropped")?,
        tx_bytes: read("tx_bytes")?,
        tx_packets: read("tx_packets")?,
        tx_dropped: read("tx_dropped")?,
    })
}

fn read_counter(path: &str) -> Result<u64, CountersError> {
    let text = fs::read_to_string(path).map_err(|source| CountersError::Read {
        path: path.to_owned(),
        source,
    })?;
    text.trim().parse().map_err(|_| CountersError::NotANumber {
        path: path.to_owned(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_tc_stats_and_ignores_extra_fields() {
        let json = r#"[
            {"kind":"htb","handle":"1:","root":true,"options":{"r2q":10},"bytes":1000,"packets":10,"drops":1,"overlimits":2,"requeues":0,"backlog":0,"qlen":0},
            {"kind":"netem","handle":"10:","parent":"1:10","options":{"limit":1000},"bytes":900,"packets":9,"drops":3,"overlimits":0,"requeues":0,"backlog":1500,"qlen":1}
        ]"#;
        let stats = parse_qdisc_stats(json).unwrap();
        assert_eq!(stats.len(), 2);
        assert_eq!(
            (stats[1].kind.as_str(), stats[1].drops, stats[1].backlog),
            ("netem", 3, 1500)
        );
        assert_eq!(stats[0].parent, None);
        assert!(parse_qdisc_stats("not json").is_err());
    }
}
