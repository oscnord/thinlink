use serde::Deserialize;

use crate::command::{Command, ExecError};
use crate::profile::Side;

/// Router address on `ground_net`; the interface carrying it is uplink egress.
pub const UPLINK_ADDR: &str = "192.168.232.254";
/// Router address on `vehicle_net`; the interface carrying it is downlink egress.
pub const DOWNLINK_ADDR: &str = "192.168.231.254";

const AUTO: &str = "auto";
const IFNAME_MAX: usize = 15;

#[derive(Debug, thiserror::Error)]
pub enum InterfaceError {
    #[error("`ip -j addr` output does not parse: {0}")]
    Parse(#[from] serde_json::Error),
    #[error("no interface has address {0}")]
    NotFound(String),
    #[error("invalid interface name `{0}`")]
    InvalidName(String),
    #[error("uplink and downlink must be different interfaces, both are `{0}`")]
    Same(String),
    #[error(transparent)]
    Exec(#[from] ExecError),
}

/// The only two interfaces the agent ever shapes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Interfaces {
    uplink: String,
    downlink: String,
}

impl Interfaces {
    /// Each selector is an interface name or `auto`.
    pub fn resolve(uplink: &str, downlink: &str) -> Result<Self, InterfaceError> {
        let uplink = resolve_one(uplink, UPLINK_ADDR)?;
        let downlink = resolve_one(downlink, DOWNLINK_ADDR)?;
        if uplink == downlink {
            return Err(InterfaceError::Same(uplink));
        }
        Ok(Self { uplink, downlink })
    }

    pub fn dev(&self, side: Side) -> &str {
        match side {
            Side::Uplink => &self.uplink,
            Side::Downlink => &self.downlink,
        }
    }
}

fn resolve_one(selector: &str, addr: &str) -> Result<String, InterfaceError> {
    if selector != AUTO {
        validate_name(selector)?;
        return Ok(selector.to_owned());
    }
    let json = Command::new(&["ip", "-j", "addr"]).run()?;
    find_by_address(&json, addr)
}

#[derive(Deserialize)]
struct IpLink {
    ifname: String,
    #[serde(default)]
    addr_info: Vec<IpAddr>,
}

#[derive(Deserialize)]
struct IpAddr {
    local: Option<String>,
}

fn find_by_address(ip_json: &str, addr: &str) -> Result<String, InterfaceError> {
    let links: Vec<IpLink> = serde_json::from_str(ip_json)?;
    links
        .into_iter()
        .find(|l| l.addr_info.iter().any(|a| a.local.as_deref() == Some(addr)))
        .map(|l| l.ifname)
        .ok_or_else(|| InterfaceError::NotFound(addr.to_owned()))
}

fn validate_name(name: &str) -> Result<(), InterfaceError> {
    let starts_ok = name
        .bytes()
        .next()
        .is_some_and(|b| b.is_ascii_alphanumeric());
    let chars_ok = name
        .bytes()
        .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'.' | b'-'));
    if !starts_ok || !chars_ok || name.len() > IFNAME_MAX || name == "lo" {
        return Err(InterfaceError::InvalidName(name.to_owned()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const IP_JSON: &str = r#"[
        {"ifindex":1,"ifname":"lo","addr_info":[{"family":"inet","local":"127.0.0.1"}]},
        {"ifindex":2,"ifname":"eth0","addr_info":[{"family":"inet","local":"192.168.231.254"}]},
        {"ifindex":3,"ifname":"eth1","addr_info":[{"family":"inet6","local":"fe80::1"},{"family":"inet","local":"192.168.232.254"}]},
        {"ifindex":4,"ifname":"eth2"}
    ]"#;

    #[test]
    fn finds_interface_by_address() {
        assert_eq!(find_by_address(IP_JSON, UPLINK_ADDR).unwrap(), "eth1");
        assert_eq!(find_by_address(IP_JSON, DOWNLINK_ADDR).unwrap(), "eth0");
        assert!(matches!(
            find_by_address(IP_JSON, "192.168.233.254"),
            Err(InterfaceError::NotFound(_))
        ));
        assert!(matches!(
            find_by_address("{", UPLINK_ADDR),
            Err(InterfaceError::Parse(_))
        ));
    }

    #[test]
    fn explicit_names_are_validated() {
        for bad in [
            "",
            "lo",
            "-eth0",
            "eth0 root",
            "a/b",
            "averyveryverylongname",
        ] {
            assert!(validate_name(bad).is_err(), "{bad}");
        }
        for good in ["eth0", "veth-up", "dummy_1", "br0.100"] {
            assert!(validate_name(good).is_ok(), "{good}");
        }
        assert!(matches!(
            Interfaces::resolve("eth0", "eth0"),
            Err(InterfaceError::Same(_))
        ));
    }
}
