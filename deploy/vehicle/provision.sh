#!/bin/sh
# Provisions a Raspberry Pi (Raspberry Pi OS Lite 64-bit, Debian 13 "trixie" or the same release as
# the build container) as the sat-teleop vehicle: packages, the service user, chrony, WireGuard, the
# binary, the configs and the systemd unit. Run as root from the unpacked release directory:
#
#   sudo ./provision.sh
#
# The directory must hold what `just vehicle-release` made: sat-teleop-vehicle, SHA256SUMS, limits.toml,
# ladder.toml, wireguard/vehicle.conf, ctl-ground.pub, vehicle.token and job.token (from `just deploy-keys`), and the files of deploy/vehicle. The
# binary is checked against SHA256SUMS before it is installed; nothing is downloaded but apt packages.
# Running it again keeps /etc/sat-teleop/vehicle.toml and the WireGuard config as they are.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
[ "$(id -u)" = 0 ] || { echo "provision: run as root" >&2; exit 1; }
[ "$(uname -m)" = aarch64 ] || { echo "provision: this build is for a 64-bit Raspberry Pi OS (aarch64)" >&2; exit 1; }

(cd "$here" && sha256sum -c SHA256SUMS)

apt-get update
# The GStreamer packages the vehicle's pipelines use, v4l-utils for inspecting the camera, chrony for
# the clock and wireguard-tools for the tunnel. gstreamer1.0-libcamera is needed only for a CSI camera
# module (libcamerasrc) and is installed when the archive has it.
apt-get install -y --no-install-recommends \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
    v4l-utils chrony wireguard-tools ca-certificates
apt-get install -y --no-install-recommends gstreamer1.0-libcamera || echo "provision: no gstreamer1.0-libcamera; a CSI camera module will not work"

id sat-teleop >/dev/null 2>&1 || useradd --system --home-dir /var/lib/sat-teleop --shell /usr/sbin/nologin sat-teleop
usermod -aG video,dialout sat-teleop

install -D -m 0755 "$here/sat-teleop-vehicle" /usr/local/bin/sat-teleop-vehicle
install -d -m 0755 /etc/sat-teleop
install -m 0644 "$here/limits.toml" /etc/sat-teleop/limits.toml
install -m 0644 "$here/ladder.toml" /etc/sat-teleop/ladder.toml
[ -e /etc/sat-teleop/vehicle.toml ] || install -m 0644 "$here/vehicle.toml" /etc/sat-teleop/vehicle.toml

install -m 0644 "$here/ctl-ground.pub" /etc/sat-teleop/ctl-ground.pub
for token in vehicle job; do
    install -m 0600 -o sat-teleop -g sat-teleop "$here/$token.token" "/etc/sat-teleop/$token.token"
done

install -d -m 0755 /etc/chrony/conf.d
install -m 0644 "$here/chrony-sat-teleop.conf" /etc/chrony/conf.d/sat-teleop.conf
systemctl enable --now chrony
systemctl restart chrony

install -d -m 0700 /etc/wireguard
[ -e /etc/wireguard/wg0.conf ] || install -m 0600 "$here/wireguard/vehicle.conf" /etc/wireguard/wg0.conf
systemctl enable --now wg-quick@wg0

install -m 0644 "$here/sat-teleop-vehicle.service" /etc/systemd/system/sat-teleop-vehicle.service
systemctl daemon-reload
systemctl enable --now sat-teleop-vehicle

echo "provision: done. Check: systemctl status sat-teleop-vehicle; curl -s -H \"Authorization: Bearer \$(sudo cat /etc/sat-teleop/vehicle.token)\" http://127.0.0.1:9100/metrics"
echo "provision: then docs/pi.md, the bench checklist in docs/deploy.md"
