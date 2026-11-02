#!/bin/sh
# Router: check forwarding, install NAT and filtering, then hand over to the router agent.
set -eu

if [ "$(sysctl -n net.ipv4.ip_forward)" != 1 ]; then
    echo "router: net.ipv4.ip_forward is off; set it in compose.yml" >&2
    exit 1
fi
nft -f /scripts/router.nft
exec lab router --listen 192.168.233.254:9000
