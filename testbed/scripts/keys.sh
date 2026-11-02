#!/bin/sh
# Writes one WireGuard key pair per node into the current directory, keeping existing keys,
# and (re)writes each node's wg-quick config from them. Run by `just lab-keys`.
set -eu
umask 077

relay_endpoint=192.168.232.20:51820
nodes="relay vehicle ground checks"
# node:host part of its 10.77.0.0/24 overlay address
clients="vehicle:2 ground:3 checks:4"

for node in $nodes; do
    [ -s "$node.key" ] || wg genkey > "$node.key"
    wg pubkey < "$node.key" > "$node.pub"
done

{
    printf '[Interface]\nAddress = 10.77.0.1/24\nListenPort = 51820\nPrivateKey = %s\n' "$(cat relay.key)"
    for client in $clients; do
        node=${client%:*}
        printf '\n[Peer]\nPublicKey = %s\nAllowedIPs = 10.77.0.%s/32\n' "$(cat "$node.pub")" "${client#*:}"
    done
} > relay.conf

for client in $clients; do
    node=${client%:*}
    printf '[Interface]\nAddress = 10.77.0.%s/24\nPrivateKey = %s\n\n[Peer]\nPublicKey = %s\nEndpoint = %s\nAllowedIPs = 10.77.0.0/24\nPersistentKeepalive = 15\n' \
        "${client#*:}" "$(cat "$node.key")" "$(cat relay.pub)" "$relay_endpoint" > "$node.conf"
done
# One bearer token per service, readable by its owner only: the vehicle's endpoints, the ground
# adapter's (the lease) and the relay's (its metrics and the job API, which the vehicle and the ground
# also present). A node holds only the tokens it serves or calls with; `checks` holds all three.
for token in vehicle ground job; do
    [ -s "$token.token" ] || (umask 077 && head -c 32 /dev/urandom | base64 > "$token.token")
    chmod 600 "$token.token"
done
# Control envelope signing keys of the two controllers: the ground adapter and the lab runner.
lab keygen ground checks
echo "WireGuard keys and configs ready: $nodes"
