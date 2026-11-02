#!/bin/sh
# Provisions a public Ubuntu LTS server (24.04, one small vCPU is enough) as the sat-teleop relay:
# Docker, WireGuard, the firewall, and the relay, MediaMTX and mavlink-router. Run as root from the
# unpacked release directory (`just relay-release`):
#
#   sudo ./provision.sh
#
# The directory holds runtime.tar (the amd64 runtime image from `just image-for amd64`), its
# SHA256SUMS, wireguard/relay.conf and job.token (from `just deploy-keys`) and the files of deploy/relay. The image
# is checked against SHA256SUMS before it is loaded; nothing is downloaded but apt packages and the
# MediaMTX image Docker pulls. Running it again keeps /etc/wireguard/wg0.conf as it is.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
[ "$(id -u)" = 0 ] || { echo "provision: run as root" >&2; exit 1; }
[ "$(uname -m)" = x86_64 ] || { echo "provision: the relay image is amd64; this machine is $(uname -m)" >&2; exit 1; }

(cd "$here" && sha256sum -c SHA256SUMS)

# Every container uses host networking, so Docker's own iptables rules are not wanted: they set the
# FORWARD policy to drop, which would block the control envelopes forwarded between the peers
# (nftables.conf). The file is written before Docker is installed so it never starts without it.
daemon=/etc/docker/daemon.json
want='{"iptables": false, "ip6tables": false}'
had_docker=$(command -v dockerd || true)
wrote=
if [ -e "$daemon" ]; then
    [ "$(cat "$daemon")" = "$want" ] || { echo "provision: $daemon exists and differs from $want; merge the two keys by hand and run again" >&2; exit 1; }
else
    install -d -m 0755 /etc/docker
    echo "$want" > "$daemon"
    wrote=1
fi

apt-get update
apt-get install -y --no-install-recommends docker.io docker-compose-v2 wireguard-tools nftables chrony ca-certificates

[ -z "$wrote" ] || [ -z "$had_docker" ] || systemctl restart docker

# The relay forwards the control envelopes between the vehicle and the ground (nftables.conf).
echo 'net.ipv4.ip_forward = 1' > /etc/sysctl.d/99-sat-teleop.conf
sysctl -q -p /etc/sysctl.d/99-sat-teleop.conf

install -d -m 0700 /etc/wireguard
[ -e /etc/wireguard/wg0.conf ] || install -m 0600 "$here/wireguard/relay.conf" /etc/wireguard/wg0.conf
systemctl enable --now wg-quick@wg0

# Check the firewall file loads before it replaces the running rules: a mistake must not lock out SSH.
nft -c -f "$here/nftables.conf"
install -m 0644 "$here/nftables.conf" /etc/nftables.conf
systemctl enable --now nftables
systemctl restart nftables

docker load -i "$here/runtime.tar"
install -d -m 0755 /opt/sat-teleop
for file in compose.yml relay.toml mediamtx.yml mavlink-router.conf; do
    install -m 0644 "$here/$file" "/opt/sat-teleop/$file"
done
install -m 0600 "$here/job.token" /opt/sat-teleop/job.token
(cd /opt/sat-teleop && docker compose up -d)

echo "provision: done. Check: docker compose -f /opt/sat-teleop/compose.yml ps; wg show"
echo "provision: then docs/deploy.md"
