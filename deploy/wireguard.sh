#!/bin/sh
# Writes a WireGuard key pair per node (and the ground adapter's control signing key) and a wg-quick config per node into the current directory,
# keeping existing keys. Run by `just deploy-keys <relay endpoint>`; the endpoint is the relay's
# public host:port, for example 203.0.113.7:51820. The vehicle and the ground dial out to it, so
# neither needs a public address.
set -eu
umask 077

endpoint=${1:?usage: wireguard.sh <relay public host:port>}
nodes="relay vehicle ground"
# node:host part of its 10.77.0.0/24 overlay address
clients="vehicle:2 ground:3"

for node in $nodes; do
    [ -s "$node.key" ] || wg genkey > "$node.key"
    wg pubkey < "$node.key" > "$node.pub"
done

{
    printf '[Interface]\nAddress = 10.77.0.1/24\nListenPort = 51820\nPrivateKey = %s\n' "$(cat relay.key)"
    for client in $clients; do
        node=${client%:*}
        printf '\n[Peer]\n# %s\nPublicKey = %s\nAllowedIPs = 10.77.0.%s/32\n' "$node" "$(cat "$node.pub")" "${client#*:}"
    done
} > relay.conf

# PersistentKeepalive keeps the NAT mapping of a cellular or satellite modem open. The tunnel's MTU
# is 1420, which the sender's 1200-byte RTP packets stay under.
for client in $clients; do
    node=${client%:*}
    printf '[Interface]\nAddress = 10.77.0.%s/24\nPrivateKey = %s\n\n[Peer]\nPublicKey = %s\nEndpoint = %s\nAllowedIPs = 10.77.0.0/24\nPersistentKeepalive = 15\n' \
        "${client#*:}" "$(cat "$node.key")" "$(cat relay.pub)" "$endpoint" > "$node.conf"
done
echo "WireGuard keys and configs ready: $nodes"
# One bearer token per service, readable by its owner only: the vehicle's endpoints, the ground
# adapter's (the lease, which never leaves the ground) and the relay's (its metrics and the job API,
# which the vehicle and the ground also present).
for token in vehicle ground job; do
    [ -s "$token.token" ] || (umask 077 && head -c 32 /dev/urandom | base64 > "$token.token")
    chmod 600 "$token.token"
done
# The ground adapter signs its control envelopes; the vehicle holds the public key.
lab keygen ground
