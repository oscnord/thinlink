#!/bin/sh
# Network sidecar: an optional extra route, the ctl_net filter, optional extra nft files, WireGuard
# wg0 from /secrets/wg0.conf, then idle until stopped.
set -eu

if [ -n "${ROUTE:-}" ]; then
    # ROUTE is an `ip route` spec such as "default via 192.168.231.254"; it must split into words.
    # shellcheck disable=SC2086
    ip route replace $ROUTE
fi
ctl_if=$(ip -o -4 addr show to 192.168.233.0/24 | awk '{print $2}')
nft -D "ctl_if=${ctl_if:?no interface on ctl_net 192.168.233.0/24}" -f /scripts/ctl.nft
# EXTRA_NFT lists files (no spaces in names); ingest.nft takes the interface on 192.168.234.0/24.
ingest_if=$(ip -o -4 addr show to 192.168.234.0/24 | awk '{print $2}')
# The ingest network's default route would bypass the router; the output filter drops it anyway.
[ -z "$ingest_if" ] || ip route del default dev "$ingest_if" 2>/dev/null || true
for file in ${EXTRA_NFT:-}; do
    nft -D "ingest_if=${ingest_if:-none}" -f "$file"
done
wg-quick up /secrets/wg0.conf

trap 'exit 0' TERM INT
sleep infinity &
wait
