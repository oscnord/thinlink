use crate::command::Command;

/// Disabled forwarding drops everything the router would forward, for cut tests.
/// Enabled forwarding removes the table again. Both are idempotent.
pub fn forwarding(enabled: bool) -> Vec<Command> {
    let mut cmds =
        vec![Command::new(&["nft", "delete", "table", "inet", "thinlink_cut"]).tolerant()];
    if !enabled {
        cmds.push(Command::new(&[
            "nft",
            "add",
            "table",
            "inet",
            "thinlink_cut",
        ]));
        cmds.push(Command::new(&[
            "nft",
            "add",
            "chain",
            "inet",
            "thinlink_cut",
            "forward",
            "{ type filter hook forward priority -100; policy accept; }",
        ]));
        cmds.push(Command::new(&[
            "nft",
            "add",
            "rule",
            "inet",
            "thinlink_cut",
            "forward",
            "counter",
            "drop",
        ]));
    }
    cmds
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cut_adds_a_drop_rule_and_enable_only_removes_it() {
        let cut: Vec<String> = forwarding(false).iter().map(Command::display).collect();
        assert_eq!(
            cut,
            [
                "nft delete table inet thinlink_cut",
                "nft add table inet thinlink_cut",
                "nft add chain inet thinlink_cut forward { type filter hook forward priority -100; policy accept; }",
                "nft add rule inet thinlink_cut forward counter drop",
            ]
        );
        let enable = forwarding(true);
        assert_eq!(enable.len(), 1);
        assert_eq!(enable[0].display(), "nft delete table inet thinlink_cut");
        assert!(enable[0].tolerate_failure);
    }
}
