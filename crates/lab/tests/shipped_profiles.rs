use std::fs;
use std::path::Path;

use sat_teleop_lab::profile::{Profile, Provenance};

const SHIPPED: [&str; 8] = [
    "lan",
    "lte-good",
    "lte-bad",
    "geo",
    "geo-degraded",
    "outage-10s",
    "outage-30s",
    "bandwidth-steps",
];

#[test]
fn every_shipped_profile_validates_and_is_labelled_synthetic() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testbed/profiles");
    for name in SHIPPED {
        let text = fs::read_to_string(dir.join(format!("{name}.toml"))).unwrap();
        let profile = Profile::parse(&text).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(profile.meta.name, name);
        assert_eq!(profile.meta.provenance, Provenance::Synthetic, "{name}");
    }
}
