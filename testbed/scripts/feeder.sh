#!/bin/sh
# Test-pattern camera for the ingest path. A 720p30 H.264 stream goes to the companion's ingest
# port from this container's address, which the vehicle accepts; a trickle goes from an alias
# address it does not, to show the refusal.
set -eu

ip addr add 192.168.234.6/24 dev "$(ip -o -4 addr show to 192.168.234.0/24 | awk '{print $2}')"
trap 'kill 0' TERM INT

gst-launch-1.0 -q videotestsrc is-live=true pattern=ball ! video/x-raw,width=1280,height=720,framerate=30/1 \
    ! x264enc tune=zerolatency speed-preset=ultrafast bframes=0 key-int-max=30 bitrate=2000 \
    ! rtph264pay config-interval=1 pt=96 mtu=1200 \
    ! udpsink host=192.168.234.10 port=5700 sync=false &
gst-launch-1.0 -q videotestsrc is-live=true pattern=snow ! video/x-raw,width=160,height=120,framerate=5/1 \
    ! x264enc tune=zerolatency speed-preset=ultrafast bframes=0 bitrate=50 \
    ! rtph264pay pt=96 mtu=1200 \
    ! udpsink host=192.168.234.10 port=5700 bind-address=192.168.234.6 sync=false &
wait
