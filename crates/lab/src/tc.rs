//! Pure translation of impairment settings to `tc` argv. Chain per interface:
//! HTB root `1:` -> class `1:10` -> netem `10:` -> (ordering = preserve) pfifo `11:`.

use crate::command::Command;
use crate::profile::{Action, Link, Loss, Ordering};

const CLASS_ID: &str = "1:10";
const NETEM_HANDLE: &str = "10:";
const PFIFO_HANDLE: &str = "11:";
const HTB_QUANTUM: &str = "1514";

/// Builds the qdisc chain from scratch. Not for use mid-run; use [`timeline_step`] for live changes.
pub fn setup(dev: &str, link: &Link, seed: Option<u64>) -> Vec<Command> {
    let mut cmds = vec![
        teardown(dev),
        Command::new(&[
            "tc", "qdisc", "add", "dev", dev, "root", "handle", "1:", "htb", "default", "10",
        ]),
        htb_class("add", dev, link, link.rate_kbit),
        netem("add", dev, link, link.loss.as_ref(), seed),
    ];
    if link.ordering == Ordering::Preserve {
        cmds.push(pfifo(dev, link.queue_packets));
    }
    cmds
}

pub fn teardown(dev: &str) -> Command {
    Command::new(&["tc", "qdisc", "del", "dev", dev, "root"]).tolerant()
}

/// A live change: rate changes the HTB class, blackout and restore change the netem qdisc.
pub fn timeline_step(dev: &str, link: &Link, seed: Option<u64>, action: Action) -> Command {
    match action {
        Action::SetRate { rate_kbit } => htb_class("change", dev, link, rate_kbit),
        Action::Blackout => netem(
            "change",
            dev,
            link,
            Some(&Loss::Random { percent: 100.0 }),
            seed,
        ),
        Action::Restore => netem("change", dev, link, link.loss.as_ref(), seed),
    }
}

pub fn show_qdisc(dev: &str) -> Command {
    Command::new(&["tc", "-j", "qdisc", "show", "dev", dev])
}

pub fn show_class(dev: &str) -> Command {
    Command::new(&["tc", "-j", "class", "show", "dev", dev])
}

pub fn show_qdisc_stats(dev: &str) -> Command {
    Command::new(&["tc", "-s", "-j", "qdisc", "show", "dev", dev])
}

fn htb_class(verb: &str, dev: &str, link: &Link, rate_kbit: u32) -> Command {
    let rate = format!("{rate_kbit}kbit");
    let burst = link.burst_bytes.to_string();
    Command::new(&[
        "tc",
        "class",
        verb,
        "dev",
        dev,
        "parent",
        "1:",
        "classid",
        CLASS_ID,
        "htb",
        "rate",
        &rate,
        "ceil",
        &rate,
        "burst",
        &burst,
        "cburst",
        &burst,
        "quantum",
        HTB_QUANTUM,
    ])
}

fn netem(verb: &str, dev: &str, link: &Link, loss: Option<&Loss>, seed: Option<u64>) -> Command {
    let mut argv: Vec<String> = [
        "tc",
        "qdisc",
        verb,
        "dev",
        dev,
        "parent",
        CLASS_ID,
        "handle",
        NETEM_HANDLE,
        "netem",
    ]
    .map(str::to_owned)
    .into();
    match (link.delay_ms, link.jitter_ms) {
        (0, _) => {}
        (delay, 0) => argv.extend(["delay".into(), format!("{delay}ms")]),
        (delay, jitter) => argv.extend([
            "delay".into(),
            format!("{delay}ms"),
            format!("{jitter}ms"),
            "distribution".into(),
            link.jitter_distribution.tc_name().into(),
        ]),
    }
    argv.extend(["limit".into(), link.queue_packets.to_string()]);
    argv.extend(loss.map_or_else(Vec::new, loss_args));
    argv.extend(seed.map_or_else(Vec::new, |s| vec!["seed".into(), s.to_string()]));
    Command::from_args(argv)
}

fn loss_args(loss: &Loss) -> Vec<String> {
    match *loss {
        Loss::Random { percent } => vec!["loss".into(), "random".into(), format!("{percent}%")],
        Loss::GilbertElliott {
            p,
            r,
            one_minus_h,
            one_minus_k,
        } => vec![
            "loss".into(),
            "gemodel".into(),
            format!("{p}%"),
            format!("{r}%"),
            format!("{one_minus_h}%"),
            format!("{one_minus_k}%"),
        ],
    }
}

fn pfifo(dev: &str, limit: u32) -> Command {
    Command::new(&[
        "tc",
        "qdisc",
        "add",
        "dev",
        dev,
        "parent",
        NETEM_HANDLE,
        "handle",
        PFIFO_HANDLE,
        "pfifo",
        "limit",
        &limit.to_string(),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::Profile;

    const BASE: &str = r#"
[profile]
name = "t"
description = "test"
provenance = "synthetic"
source = "unit test"
seed = 7

[uplink]
delay_ms = 300
jitter_ms = 10
jitter_distribution = "pareto"
rate_kbit = 512
burst_bytes = 6000
queue_packets = 200

[downlink]
delay_ms = 0
rate_kbit = 2048
ordering = "allow-reorder"
"#;

    fn profile(extra: &str) -> Profile {
        Profile::parse(&format!("{BASE}{extra}")).unwrap()
    }

    fn lines(cmds: &[Command]) -> Vec<String> {
        cmds.iter().map(Command::display).collect()
    }

    #[test]
    fn delay_jitter_preserve_ordering() {
        let p = profile("");
        let got = lines(&setup("eth1", &p.uplink, p.meta.seed));
        assert_eq!(
            got,
            [
                "tc qdisc del dev eth1 root",
                "tc qdisc add dev eth1 root handle 1: htb default 10",
                "tc class add dev eth1 parent 1: classid 1:10 htb rate 512kbit ceil 512kbit burst 6000 cburst 6000 quantum 1514",
                "tc qdisc add dev eth1 parent 1:10 handle 10: netem delay 300ms 10ms distribution pareto limit 200 seed 7",
                "tc qdisc add dev eth1 parent 10: handle 11: pfifo limit 200",
            ]
        );
        assert!(setup("eth1", &p.uplink, None)[0].tolerate_failure);
        assert!(!setup("eth1", &p.uplink, None)[1].tolerate_failure);
    }

    #[test]
    fn no_delay_allow_reorder_has_no_pfifo_and_no_delay_args() {
        let p = profile("");
        let got = lines(&setup("eth0", &p.downlink, None));
        assert_eq!(got.len(), 4);
        assert_eq!(
            got[3],
            "tc qdisc add dev eth0 parent 1:10 handle 10: netem limit 1000"
        );
    }

    #[test]
    fn delay_without_jitter_omits_distribution() {
        let p = profile("");
        let mut link = p.uplink.clone();
        link.jitter_ms = 0;
        let got = lines(&setup("eth1", &link, None));
        assert!(
            got[3].ends_with("netem delay 300ms limit 200"),
            "{}",
            got[3]
        );
    }

    #[test]
    fn gilbert_elliott_loss() {
        let p = profile(
            "\n[uplink.loss]\nmodel = \"gilbert-elliott\"\np = 3\nr = 20\none_minus_h = 80.5\none_minus_k = 0\n",
        );
        let got = lines(&setup("eth1", &p.uplink, None));
        assert_eq!(
            got[3],
            "tc qdisc add dev eth1 parent 1:10 handle 10: netem delay 300ms 10ms distribution pareto limit 200 loss gemodel 3% 20% 80.5% 0%"
        );
    }

    #[test]
    fn random_loss() {
        let p = profile("\n[downlink.loss]\nmodel = \"random\"\npercent = 1.5\n");
        let got = lines(&setup("eth0", &p.downlink, None));
        assert_eq!(
            got[3],
            "tc qdisc add dev eth0 parent 1:10 handle 10: netem limit 1000 loss random 1.5%"
        );
    }

    #[test]
    fn live_changes_never_delete_qdiscs() {
        let p = profile("\n[uplink.loss]\nmodel = \"random\"\npercent = 2\n");
        let step = |action| timeline_step("eth1", &p.uplink, p.meta.seed, action).display();
        assert_eq!(
            step(Action::SetRate { rate_kbit: 128 }),
            "tc class change dev eth1 parent 1: classid 1:10 htb rate 128kbit ceil 128kbit burst 6000 cburst 6000 quantum 1514"
        );
        assert_eq!(
            step(Action::Blackout),
            "tc qdisc change dev eth1 parent 1:10 handle 10: netem delay 300ms 10ms distribution pareto limit 200 loss random 100% seed 7"
        );
        assert_eq!(
            step(Action::Restore),
            "tc qdisc change dev eth1 parent 1:10 handle 10: netem delay 300ms 10ms distribution pareto limit 200 loss random 2% seed 7"
        );
    }

    #[test]
    fn teardown_tolerates_missing_qdisc() {
        let cmd = teardown("eth1");
        assert_eq!(cmd.display(), "tc qdisc del dev eth1 root");
        assert!(cmd.tolerate_failure);
    }
}
