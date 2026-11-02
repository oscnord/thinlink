# CONTAINER_ENGINE=docker runs the lab with Docker and `docker compose`; the Podman machine
# workarounds below apply only to Podman.
engine := env_var_or_default("CONTAINER_ENGINE", "podman")
# keep-id maps the host user into the container on Podman; Docker needs the uid spelled out.
user_flags := if engine == "podman" { "--userns=keep-id" } else { "--user $(id -u):$(id -g)" }
gstreamer_packages := "sat-teleop-media sat-teleop-lab sat-teleop-lab-cli sat-teleop-vehicle sat-teleop-relay sat-teleop-ground"
# The testbed's runtime image has the vehicle's test hooks; the release images (`image-for`) do not.
testbed_features := "--build-arg CARGO_FEATURES='--features sat-teleop-vehicle/testbed'"
build_image := "localhost/sat-teleop-build:dev"
native_arch := if arch() == "x86_64" { "amd64" } else { "arm64" }
runtime_image := "localhost/sat-teleop-runtime:dev"
# Docker does not know the name Podman gives the host; testbed/compose.docker.yml maps it.
engine_compose := if engine == "podman" { "" } else { " -f testbed/compose.docker.yml" }
# SIM=1 adds the simulated flight's services (`just sim-up`). PI=pi4 or PI=pi-zero-2w caps the
# vehicle's companion to that board (testbed/compose.pi4.yml and compose.pi-zero-2w.yml, docs/latency.md), for example
# `PI=pi4 just lab-up`; set it for `lab-check` too.
sim_file := if env_var_or_default("SIM", "") != "" { " -f testbed/compose.sim.yml" } else { "" }
pi_file := if env_var_or_default("PI", "") != "" { " -f testbed/compose." + env_var("PI") + ".yml" } else { "" }
compose := engine + " compose -p sat-teleop -f testbed/compose.yml" + engine_compose + sim_file + pi_file
# Taking the testbed down names every service of either mode: a simulator container shares the
# vehicle sidecar's namespace, so the sidecar cannot be removed before it.
compose_down := engine + " compose -p sat-teleop -f testbed/compose.yml" + engine_compose + " -f testbed/compose.sim.yml"
# Builds and `check` run in a cgroup capped below the shared Podman machine's memory, so running out of
# memory kills the build, not another project's containers. BuildKit has no build memory limit. The
# defaults fit a 2 GiB machine; a larger one can raise them (BUILD_MEMORY=4g BUILD_JOBS=4).
# `just regional=1 sim-up` or `just regional=1 demo` (or REGIONAL=1) runs the vehicle with the person
# detector on (vehicle-sim-regional.toml, vehicle-ingest-regional.toml).
regional := env_var_or_default("REGIONAL", "")
regional_suffix := if regional == "1" { "-regional" } else { "" }
build_mem := env_var_or_default("BUILD_MEMORY", "1g")
build_jobs := env_var_or_default("BUILD_JOBS", "1")
# Each checkout (the main one, each worktree) builds into its own cargo target directory, named after
# its folder: all of them are mounted at /src, and cargo, which judges freshness by file times, would
# otherwise take another checkout's newer build of a file for this one's.
checkout := file_name(justfile_directory())
build_memory := (if engine == "podman" { "--memory " + build_mem } else { "" }) + " --build-arg CARGO_BUILD_JOBS=" + build_jobs + " --build-arg CHECKOUT=" + checkout

default:
    @just --list

# Authoritative gate: full workspace in the build container.
check: (image "build")
    {{engine}} run --rm {{user_flags}} --memory {{build_mem}} -e CARGO_BUILD_JOBS={{build_jobs}} \
        -v "{{justfile_directory()}}:/src" -w /src \
        -v sat-teleop-cargo-registry:/cache/cargo -v sat-teleop-target:/cache/target \
        -e HOME=/tmp -e CARGO_HOME=/cache/cargo -e CARGO_TARGET_DIR=/cache/target/{{checkout}} \
        {{build_image}} sh -ec ' \
            cargo fmt --all -- --check; \
            cargo clippy --workspace --all-targets --locked -- -D warnings; \
            cargo test --workspace --locked; \
            cargo clippy -p sat-teleop-control -p sat-teleop-vehicle --all-targets --locked --features sat-teleop-vehicle/testbed -- -D warnings; \
            cargo test -p sat-teleop-control -p sat-teleop-vehicle --locked --features sat-teleop-vehicle/testbed; \
            cargo xtask layers'

# Same gates on the host, for packages that do not link GStreamer.
check-native:
    #!/usr/bin/env bash
    set -euo pipefail
    channel=$(sed -n 's/^channel *= *"\(.*\)"/\1/p' rust-toolchain.toml)
    if command -v rustup >/dev/null; then PATH="$(dirname "$(rustup which --toolchain "$channel" cargo)"):$PATH"; fi
    exclude=""
    names=$(cargo metadata --no-deps --format-version 1 | jq -r '.packages[].name')
    for p in $names; do
        case " {{gstreamer_packages}} " in *" $p "*) exclude="$exclude --exclude $p";; esac
    done
    cargo fmt --all -- --check
    cargo clippy --workspace $exclude --all-targets --locked -- -D warnings
    cargo test --workspace $exclude --locked
    cargo clippy -p sat-teleop-control --all-targets --locked --features testbed -- -D warnings
    cargo test -p sat-teleop-control --locked --features testbed
    cargo xtask layers

# The runtime build also tags the build stage: its untagged layers (the apt install) are otherwise
# only reachable from an untagged release image, which `podman image prune` removes.
image target: vm-memory
    {{ if target == "runtime" { engine + " build " + build_memory + " --target build -t " + build_image + " -f testbed/Containerfile ." } else { "true" } }}
    {{engine}} build {{build_memory}} {{ if target == "runtime" { testbed_features } else { "" } }} --target {{target}} -t localhost/sat-teleop-{{target}}:dev -f testbed/Containerfile .

# The runtime image for one platform, `arm64` (the Pi, Apple Silicon) or `amd64` (a typical VPS),
# tagged with the architecture. amd64 is cross-built from the host's own architecture (rustc cannot
# run emulated); only its runtime stage runs emulated. docs/deploy.md, "Images and binaries".
image-for arch: vm-memory
    {{engine}} build {{build_memory}} --platform linux/{{arch}} --target {{ if arch == "amd64" { "runtime-amd64" } else { "runtime" } }} -t localhost/sat-teleop-runtime:{{arch}} -f testbed/Containerfile .

# Podman reads the build context through the machine's file share, which needs free 512 KiB blocks
# of VM memory; once page cache has fragmented it, `COPY` fails with "cannot allocate memory".
# Dropping clean page cache and compacting gives them back without losing anything.
[private]
vm-memory:
    #!/usr/bin/env bash
    set -euo pipefail
    [ "{{engine}}" = podman ] && podman machine inspect >/dev/null 2>&1 || exit 0
    blocks=$(podman machine ssh cat /proc/buddyinfo | awk '{ for (i = 12; i <= 15; i++) s += $i } END { print s + 0 }')
    [ "$blocks" -ge 16 ] && exit 0
    podman machine ssh "sudo sh -c 'sync; echo 1 > /proc/sys/vm/drop_caches; echo 1 > /proc/sys/vm/compact_memory'"
    echo "vm-memory: $blocks free blocks of 512 KiB or more; dropped the machine's page cache for the build"

# ArduRover SITL image; x86_64 only, emulated on other hosts.
sitl-image: vm-memory
    {{engine}} build {{build_memory}} --platform linux/amd64 -t localhost/sat-teleop-sitl:dev -f testbed/Containerfile.sitl .

# The simulated flight's images: ArduCopter SITL (x86_64, emulated on other hosts) and Gazebo with its
# camera (native). docs/simulation.md.
sim-image: vm-memory
    {{engine}} build {{build_memory}} --platform linux/amd64 -t localhost/sat-teleop-sitl-copter:dev -f testbed/Containerfile.sitl-copter .
    {{engine}} build {{build_memory}} -t localhost/sat-teleop-sim:dev -f testbed/Containerfile.sim .

fmt:
    cargo fmt --all

# The person detector's model (NanoDet-Plus-m 320, Apache-2.0) in models/person.onnx, pinned by SHA-256.
# An existing file is only checked; a mismatch fails and removes nothing.
model_url := "https://huggingface.co/nouman/nanodet-plus-onnx/resolve/1c71d967632f4b5cdfbc08326328df6def7a6ecc/plus-m-320/nanodet-plus-m_320.onnx"
model_sha256 := "4f12723cce3d48e47ca92cb925ba74d97a965c069208edca660bbb9f7ce2c610"
model:
    #!/usr/bin/env bash
    set -euo pipefail
    mkdir -p models
    if [ ! -f models/person.onnx ]; then
        curl -fsSL -o models/person.onnx.part "{{model_url}}"
        mv models/person.onnx.part models/person.onnx
    fi
    echo "{{model_sha256}}  models/person.onnx" | shasum -a 256 -c -

# WireGuard key pairs and configs in testbed/secrets/ (gitignored). Existing keys are kept.
lab-keys: (image "runtime")
    mkdir -p testbed/secrets
    {{engine}} run --rm {{user_flags}} \
        -v "{{justfile_directory()}}/testbed/secrets:/secrets" -w /secrets \
        -v "{{justfile_directory()}}/testbed/scripts:/scripts:ro" \
        {{runtime_image}} /scripts/keys.sh

# WireGuard keys and configs for a deployment in deploy/secrets/ (gitignored), for the relay's public
# endpoint, for example `just deploy-keys 203.0.113.7:51820`. Existing keys are kept. docs/deploy.md.
deploy-keys endpoint: (image "runtime")
    mkdir -p deploy/secrets
    {{engine}} run --rm {{user_flags}} \
        -v "{{justfile_directory()}}/deploy/secrets:/secrets" -w /secrets \
        -v "{{justfile_directory()}}/deploy/wireguard.sh:/wireguard.sh:ro" \
        {{runtime_image}} /wireguard.sh {{endpoint}}

# dist/vehicle: what to copy to a Raspberry Pi, with its provisioning script. The binary is the one
# in the arm64 runtime image (built here, natively on Apple Silicon), checked by SHA256SUMS on the Pi.
vehicle-release: lab-down (image-for "arm64")
    #!/usr/bin/env bash
    set -euo pipefail
    [ -f deploy/secrets/vehicle.conf ] || { echo "vehicle-release: run just deploy-keys <relay endpoint> first" >&2; exit 1; }
    out=dist/vehicle
    rm -rf "$out"
    mkdir -p "$out/wireguard"
    id=$({{engine}} create localhost/sat-teleop-runtime:arm64)
    {{engine}} cp "$id:/usr/local/bin/sat-teleop-vehicle" "$out/sat-teleop-vehicle"
    {{engine}} rm "$id" > /dev/null
    file "$out/sat-teleop-vehicle" | grep -q aarch64 || { echo "vehicle-release: the packed binary is not aarch64 ($(file -b "$out/sat-teleop-vehicle")); build on an arm64 host (docs/deploy.md)" >&2; exit 1; }
    cp deploy/vehicle/* testbed/limits.toml testbed/ladder.toml "$out/"
    cp deploy/secrets/vehicle.conf "$out/wireguard/vehicle.conf"
    cp deploy/secrets/ctl-ground.pub deploy/secrets/vehicle.token deploy/secrets/job.token "$out/"
    (cd "$out" && shasum -a 256 sat-teleop-vehicle > SHA256SUMS)
    echo "vehicle-release: $out is ready; copy it to the Pi and run sudo ./provision.sh (docs/deploy.md)"

# dist/relay: what to copy to the relay server: the amd64 runtime image as a tar (built under
# emulation: slow), its checksum, the configs and the provisioning script.
relay-release: lab-down (image-for "amd64")
    #!/usr/bin/env bash
    set -euo pipefail
    [ -f deploy/secrets/relay.conf ] || { echo "relay-release: run just deploy-keys <relay endpoint> first" >&2; exit 1; }
    out=dist/relay
    rm -rf "$out"
    mkdir -p "$out/wireguard"
    cp deploy/secrets/job.token "$out/job.token"
    {{ if engine == "podman" { "podman save --format docker-archive" } else { "docker save" } }} -o "$out/runtime.tar" localhost/sat-teleop-runtime:amd64
    cp deploy/relay/* "$out/"
    cp deploy/secrets/relay.conf "$out/wireguard/relay.conf"
    (cd "$out" && shasum -a 256 runtime.tar > SHA256SUMS)
    echo "relay-release: $out is ready; copy it to the server and run sudo ./provision.sh (docs/deploy.md)"

# dist/ground: the ground adapter's compose file and configs for this machine, run with the release
# runtime image built here for its architecture (no test hooks in it); `.env` names the image.
ground-release: lab-down (image-for native_arch)
    #!/usr/bin/env bash
    set -euo pipefail
    [ -f deploy/secrets/ground.conf ] || { echo "ground-release: run just deploy-keys <relay endpoint> first" >&2; exit 1; }
    out=dist/ground
    rm -rf "$out"
    mkdir -p "$out/wireguard"
    cp deploy/ground/* testbed/limits.toml "$out/"
    cp deploy/secrets/ground.conf "$out/wireguard/ground.conf"
    cp deploy/secrets/ctl-ground.key deploy/secrets/ground.token deploy/secrets/job.token "$out/"
    echo "SAT_TELEOP_IMAGE=localhost/sat-teleop-runtime:{{native_arch}}" > "$out/.env"
    echo "ground-release: $out is ready; cd into it and run {{engine}} compose up -d (docs/deploy.md)"

# Build images, make keys and (re)start compose project sat-teleop. Services share sidecar network
# namespaces, which Podman cannot recreate under them, so a running project is taken down first, and
# before any build: the Podman machine's 2 GiB cannot build beside a running testbed.
# On a Podman machine, netavark#1497 can leave forwarding off on ground_net's bridge, which stops
# the published ports carrying data; it is turned back on for the ground and ingest bridges only.
lab-up: lab-down lab-keys sitl-image
    #!/usr/bin/env bash
    set -euo pipefail
    mkdir -p results
    {{compose}} up -d
    [ "{{engine}}" = podman ] && podman machine inspect >/dev/null 2>&1 || exit 0
    for net in ground_net ingest_net; do
        bridge=$(podman network inspect sat-teleop_$net --format '{{{{.NetworkInterface}}')
        [[ $bridge =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "lab-up: unexpected bridge name '$bridge'" >&2; exit 1; }
        forwarding=$(podman machine ssh "cat /proc/sys/net/ipv4/conf/$bridge/forwarding" | tr -d '\r')
        [ "$forwarding" = 0 ] || continue
        podman machine ssh "sudo sysctl -q -w net.ipv4.conf.$bridge.forwarding=1"
        echo "lab-up: forwarding was off on $bridge (sat-teleop_$net), turned it on; see https://github.com/containers/netavark/issues/1497"
    done

# Run `lab check` in the checks container; results land in results/<run-id>/. Further arguments go
# to `lab check`, for example `--duration-s 120`. A check that needs the ground adapter killed
# (control-freshness) prints `lab: kill ground` on stdout; only the host can kill a container, so
# this does it and starts the adapter again at once, without a lease. The recording check prints
# `lab: restart companion` to have the vehicle's companion restarted, to test the store's recovery.
# The real-mode check prints `lab: vehicle-config <file>` to have the companion started on another
# config of testbed/config (real mode, real mode without a key, and the normal one again at the end).
lab-check scenario profile *args:
    #!/usr/bin/env bash
    set -uo pipefail
    {{compose}} exec -T checks lab check {{scenario}} --profile {{profile}} --git-rev "$(git describe --always --dirty)" {{args}} |
    while IFS= read -r line; do
        if [ "$line" = "lab: kill ground" ]; then
            {{compose}} kill ground >&2 && {{compose}} start ground >&2
        elif [ "$line" = "lab: restart companion" ]; then
            {{compose}} restart companion >&2
        elif [[ "$line" =~ ^"lab: vehicle-config "(vehicle[a-z-]*\.toml)$ ]]; then
            VEHICLE_CONFIG="${BASH_REMATCH[1]}" {{compose}} up -d --no-deps companion >&2 < /dev/null
        else
            printf '%s\n' "$line"
        fi
    done
    exit "${PIPESTATUS[0]}"

# Where a frame's time goes, from a skeleton run's summary (`just stages 1793633897126-skeleton-geo`):
# p50, p95 and max in ms per stage, from the processes' last 1024 frames at the end of the window.
stages run:
    @jq -r '.readings.end as $r | .metrics as $m | [ \
        ["source to stamp", $r.vehicle.stages.before_stamp], ["encoder queue", $r.vehicle.stages.queue], \
        ["encode", $r.vehicle.stages.encode], ["stamp to last packet sent", $r.vehicle.stages.sent], \
        ["relay jitter buffer", $r.relay.stages.jitter], ["relay depay and parse", $r.relay.stages.parse], \
        ["ground decode", $r.ground.video.decode], ["ground state stamp and re-encode", $r.ground.video.reencode], \
        ["headless decode", $r.headless_decode], \
        ["frame age at the headless receiver", {n: $m.new_frames_per_s, p50_ms: $m.frame_age_p50_ms, p95_ms: $m.frame_age_p95_ms, max_ms: $m.frame_age_max_ms}] \
      ][] | "\(.[0])\t\(.[1].p50_ms)\t\(.[1].p95_ms)\t\(.[1].max_ms)"' results/{{run}}/summary.json | column -t -s "$(printf '\t')"

# Fast gate for one fix: rebuild what changed, bring the testbed up only if the images or the build
# inputs changed (a build needs it down), then run one check. docs/development.md, "Gates".
gate-fast scenario profile *args:
    #!/usr/bin/env bash
    set -euo pipefail
    stamp=.cache/gate-fast.inputs
    inputs=$(git ls-files -co --exclude-standard -z -- Cargo.toml Cargo.lock rust-toolchain.toml .cargo crates apps xtask fixtures testbed/Containerfile |
        xargs -0 shasum | shasum | cut -d' ' -f1)
    running=$({{compose}} ps --services --status running | wc -l)
    services=$({{compose}} config --services | wc -l)
    image_of() { {{engine}} inspect --format '{{{{.Image}}' "$1" 2>/dev/null || true; }
    current=$({{engine}} image inspect --format '{{{{.Id}}' {{runtime_image}} 2>/dev/null || true)
    if [ "$running" -ne "$services" ] || [ "$(cat "$stamp" 2>/dev/null)" != "$inputs" ] || [ "$(image_of sat-teleop-companion-1)" != "$current" ]; then
        just lab-up
        mkdir -p .cache
        echo "$inputs" > "$stamp"
    fi
    just lab-check {{scenario}} {{profile}} {{args}}

# The per-task series: every check a task is reviewed on, with the testbed rebuilt from scratch
# first. Runs them all and fails if any did not pass. Budgets and the minimum-hardware runs come from this, on
# release builds. Calibration-only profiles stay out of it.
gate-full: lab-up
    #!/usr/bin/env bash
    set -uo pipefail
    started=$SECONDS
    failed=0
    for pair in "boot lan" "skeleton lan" "skeleton geo" "mavlink lan" "control-freshness lan" \
        "qgc-status lan" "qgc-status geo" "qgc-status outage-10s" "adaptation bandwidth-steps" \
        "recording lan" "retrieval geo" "real-mode lan"; do
        # shellcheck disable=SC2086
        just lab-check $pair || failed=$((failed + 1))
    done
    just ingest-isolation || failed=$((failed + 1))
    echo "gate-full: $failed of 13 checks did not pass, $((SECONDS - started)) s"
    exit "$((failed > 0))"

# The camera ingest path graded without a camera: the testbed comes up with the vehicle taking its
# video from the ingest (vehicle-ingest.toml), the feeder container sends a test pattern in place of
# the webcam, `ingest` grades the ingest and `skeleton` the video through the link, and
# `ingest-isolation` proves the ingest network reaches nothing else. docs/demo.md.
ingest-check profile="lan" *args:
    #!/usr/bin/env bash
    set -uo pipefail
    export VEHICLE_CONFIG=vehicle-ingest.toml
    just lab-up || exit 2
    {{compose}} --profile feeder up -d feeder || exit 2
    failed=0
    just lab-check ingest {{profile}} --warmup-s 10 --duration-s 10 {{args}} || failed=$((failed + 1))
    just lab-check skeleton {{profile}} {{args}} || failed=$((failed + 1))
    just ingest-isolation || failed=$((failed + 1))
    {{compose}} --profile feeder stop feeder
    exit "$((failed > 0))"

# Proves the ingest network carries nothing but the ingest port. The sidecar publishes exactly one
# port. Then each drop rule of the ingest filter must count packets while a probe tries the thing it
# stops: a TCP connection from the ingest side (input), the companion reaching the ingest side
# (output), and packets routed through the sidecar from and to the ingest side (forward). A timeout
# proves nothing, since replies leave through the router anyway. Needs a testbed up; a one-off
# feeder container makes the probes from the ingest side if the feeder is not running.
ingest-isolation:
    #!/usr/bin/env bash
    set -uo pipefail
    failed=0
    published=$({{engine}} port sat-teleop-vehicle-net-1)
    if [ "$published" = "5700/udp -> 127.0.0.1:5700" ]; then
        echo "ingest-isolation: the vehicle sidecar publishes only $published"
    else
        echo "ingest-isolation: FAIL, the vehicle sidecar publishes: $published" >&2
        failed=1
    fi
    ingest_host() {
        if {{compose}} --profile feeder ps --services --status running | grep -qx feeder; then
            {{compose}} exec -T feeder "$@"
        else
            {{compose}} --profile feeder run --rm --no-deps -T feeder "$@"
        fi
    }
    dropped() {
        {{compose}} exec -T vehicle-net nft list chain inet sat_teleop_ingest "$1" |
            awk -v rule="$2" '$0 ~ rule && /counter packets .* drop$/ { for (i = 1; i < NF; i++) if ($i == "packets") print $(i + 1) }'
    }
    # prove <chain> <rule pattern> <what the probe tries> <probe command...>
    prove() {
        local chain=$1 rule=$2 what=$3 before after
        shift 3
        before=$(dropped "$chain" "$rule")
        "$@" > /dev/null 2>&1
        after=$(dropped "$chain" "$rule")
        if [ -n "$before" ] && [ -n "$after" ] && [ "$after" -gt "$before" ]; then
            echo "ingest-isolation: $chain drop counted $((after - before)) packets: $what"
        else
            echo "ingest-isolation: FAIL, $chain drop did not count: $what (counter ${before:-missing} -> ${after:-missing})" >&2
            failed=1
        fi
    }
    sidecar() { {{compose}} exec -T vehicle-net "$@"; }
    prove input iifname "a TCP connection from the ingest side to the sidecar" \
        ingest_host curl -s -m 2 -o /dev/null http://192.168.234.10:9100/
    prove output oifname "the companion reaching the host on the ingest side" \
        sidecar curl -s -m 2 -o /dev/null http://192.168.234.1:9100/
    prove forward iifname "the ingest side reaching the router through the sidecar" \
        ingest_host sh -c 'ip route replace 192.168.231.254 via 192.168.234.10 && curl -s -m 2 -o /dev/null http://192.168.231.254:9000/'
    prove forward oifname "the router reaching the ingest side through the sidecar" \
        {{compose}} exec -T router sh -c 'ip route replace 192.168.234.5 via 192.168.231.10 && curl -s -m 2 -o /dev/null http://192.168.234.5:9000/'
    exit "$failed"

camera_pid := ".cache/demo-camera.pid"
camera_log := ".cache/demo-camera.log"
camera_progress := ".cache/demo-camera.progress"

# `camera` is an avfoundation index (`just demo-cameras`), also settable as CAMERA=. The first run asks
# macOS for camera access for the terminal app. `just demo-stop` ends it. docs/demo.md.
# The demo: the Mac's camera as the vehicle's video, through the emulated link of <profile>.
demo profile="lan" camera=env_var_or_default("CAMERA", "0"):
    #!/usr/bin/env bash
    set -euo pipefail
    command -v ffmpeg >/dev/null || { echo "demo: ffmpeg is not installed (brew install ffmpeg)" >&2; exit 1; }
    [[ {{camera}} =~ ^[0-9]+$ ]] || { echo "demo: the camera is an index, see just demo-cameras" >&2; exit 1; }
    [ -f testbed/profiles/{{profile}}.toml ] || { echo "demo: no profile testbed/profiles/{{profile}}.toml" >&2; exit 1; }
    just demo-camera-stop
    VEHICLE_CONFIG=vehicle-ingest{{regional_suffix}}.toml just lab-up
    just impair {{profile}}
    mkdir -p .cache
    : > {{camera_log}}
    started=0
    trap '[ "$started" = 1 ] || just demo-camera-stop' EXIT
    # Low latency: no input buffering, x264 zerolatency without B-frames, a keyframe every half second
    # with its parameter sets in the stream (the rtp muxer keeps them out otherwise), 1200-byte packets.
    nohup ffmpeg -nostdin -hide_banner -loglevel warning -progress {{camera_progress}} \
        -fflags nobuffer -flags low_delay \
        -f avfoundation -framerate 30 -video_size 1280x720 -i "{{camera}}" -an \
        -c:v libx264 -preset ultrafast -tune zerolatency -bf 0 -g 15 -pix_fmt yuv420p \
        -b:v 3M -maxrate 3M -bufsize 600k -bsf:v dump_extra=freq=keyframe \
        -f rtp -payload_type 96 -pkt_size 1200 rtp://127.0.0.1:5700 \
        > {{camera_log}} 2>&1 < /dev/null &
    echo $! > {{camera_pid}}
    for _ in $(seq 1 30); do
        sleep 0.5
        kill -0 "$(cat {{camera_pid}})" 2>/dev/null || break
        frames=$(sed -n 's/^frame=//p' {{camera_progress}} 2>/dev/null | tail -1 || true)
        [ "${frames:-0}" -gt 0 ] && { started=1; break; }
    done
    if [ "$started" != 1 ]; then
        echo "demo: camera {{camera}} delivered no frames. ffmpeg said:" >&2
        tail -5 {{camera_log}} >&2
        echo "demo: if macOS has not asked, or you said no, allow your terminal app under System Settings, Privacy & Security, Camera, then run again; just demo-cameras lists the indexes" >&2
        exit 1
    fi
    echo "demo: camera {{camera}} is streaming through the {{profile}} link."
    echo "demo: start QGroundControl (nothing to set up): the video appears with its LIVE age label."
    echo "demo: STALE: just video-hook pause, then just video-hook resume. Drive: just take-control, arm in QGC, release-control after."
    echo "demo: end it with just demo-stop (the camera stays on until then)."

# Stop the demo: the camera, then the testbed.
demo-stop: demo-camera-stop lab-down

# Ends the demo's ffmpeg: TERM, then KILL after about 3 s. Only a process whose command line is the
# demo's camera capture is signalled, so a recycled pid of anything else is left alone; the pid file
# goes once that process is gone.
[private]
demo-camera-stop:
    #!/usr/bin/env bash
    set -uo pipefail
    pid=$(cat {{camera_pid}} 2>/dev/null || true)
    is_camera() {
        [[ $pid =~ ^[0-9]+$ ]] || return 1
        [[ $(ps -p "$pid" -o command= 2>/dev/null) == *ffmpeg*"-f avfoundation"*"rtp://127.0.0.1:5700"* ]]
    }
    if is_camera; then
        kill "$pid"
        for _ in $(seq 1 15); do
            is_camera || break
            sleep 0.2
        done
        is_camera && kill -9 "$pid"
        sleep 0.2
        is_camera && { echo "demo: camera process $pid did not end, pid file kept" >&2; exit 1; }
        echo "demo: camera stopped"
    fi
    rm -f {{camera_pid}} {{camera_progress}}

# The Mac's cameras and their indexes for `just demo` (ffmpeg lists them, then exits with an error).
demo-cameras:
    -ffmpeg -hide_banner -f avfoundation -list_devices true -i "" 2>&1 | sed -n '/video devices/,/audio devices/p'

# Apply a link profile (testbed/profiles/) to the running testbed.
impair profile:
    {{compose}} exec -T checks lab impair --agent http://192.168.233.254:9000 --profile testbed/profiles/{{profile}}.toml > /dev/null

# Ask the ground adapter to take the vehicle's control lease (docs/qgc.md, "Taking control").
take-control:
    {{compose}} exec -T ground sat-teleop-ground --token-file /secrets/ground.token take-control

# Give the control lease back.
release-control:
    {{compose}} exec -T ground sat-teleop-ground --token-file /secrets/ground.token release-control

# Show the ground adapter's lease.
control-status:
    {{compose}} exec -T ground sat-teleop-ground --token-file /secrets/ground.token status

# Ask the relay for the retained camera frame from <ago_s> seconds ago, whole or a crop given as
# fractions x,y,w,h (for example 0.25,0.25,0.5,0.5). Prints the request's id for `detail-fetch`.
# docs/architecture.md, "Retained detail".
detail-request ago_s="10" crop="":
    {{compose}} exec -T ground sat-teleop-ground --job-token-file /secrets/job.token detail request --ago-s {{ago_s}} {{ if crop != "" { "--crop " + crop } else { "" } }}

# Like `detail-request`, for the frame captured at <at_ms> (UNIX milliseconds, as `detail-outage`
# prints it).
detail-request-at at_ms crop="":
    {{compose}} exec -T ground sat-teleop-ground --job-token-file /secrets/job.token detail request --at-ms {{at_ms}} {{ if crop != "" { "--crop " + crop } else { "" } }}

# List the snapshots the vehicle took while the ground was out of reach, with where it was, from the last
# <since_s> seconds. They stay on the vehicle until fetched (`detail-request-at`, `detail-fetch`).
detail-outage since_s="3600":
    {{compose}} exec -T ground sat-teleop-ground --job-token-file /secrets/job.token detail outage --since-s {{since_s}}

# Wait for the answer to a `detail-request` and save the JPEG on this machine; prints which frame it
# is and how old, labelled as historical.
detail-fetch id file="detail.jpg":
    {{compose}} exec -T ground sat-teleop-ground --job-token-file /secrets/job.token detail fetch {{id}} -o /tmp/detail.jpg
    {{engine}} cp sat-teleop-ground-1:/tmp/detail.jpg {{file}}

# The vehicle's test-only video hooks (docs/qgc.md, "Checking it in QGC"): pause, freeze or resume.
video-hook action:
    {{compose}} exec -T checks sh -c 'curl -fsS -X POST -H "Authorization: Bearer $(cat /secrets/vehicle.token)" http://192.168.233.10:9100/video/{{action}}'

# The simulated flight: the testbed with an ArduCopter flown in Gazebo, its rendered camera as the
# vehicle's video. Builds first, so the testbed goes down first. `just lab-up` brings the default
# rover testbed back. docs/simulation.md.
sim-up config=("vehicle-sim" + regional_suffix + ".toml"):
    just lab-down
    just sim-image
    SIM=1 VEHICLE_CONFIG={{config}} just lab-up

# Grade the simulated flight through the emulated link of <profile>: takes off, flies a leg, loses
# the ground adapter and must land safe (`sim-flight`), and records how the camera's video reads at
# the ground (`qgc-status`).
sim-check profile="lan" *args:
    #!/usr/bin/env bash
    set -uo pipefail
    export SIM=1 VEHICLE_CONFIG=vehicle-sim.toml
    just sim-up || exit 2
    failed=0
    just lab-check sim-flight {{profile}} {{args}} || failed=$((failed + 1))
    just lab-check qgc-status {{profile}} {{args}} || failed=$((failed + 1))
    exit "$((failed > 0))"

# A mission flown out of reach (`sim-mission`) through the emulated link of <profile>, on the vehicle
# with allow_missions on (the default) or off: with it on the copter flies the mission to its end
# after the link is cut at the first waypoint, with it off the upload, the start and Auto are refused.
sim-mission profile="geo" switch="on" *args:
    #!/usr/bin/env bash
    set -uo pipefail
    export SIM=1
    case "{{switch}}" in
        on) config=vehicle-sim-missions.toml ;;
        off) config=vehicle-sim.toml ;;
        *) echo "switch is on or off" >&2; exit 2 ;;
    esac
    export VEHICLE_CONFIG="$config"
    just sim-up "$config" || exit 2
    just lab-check sim-mission {{profile}} {{args}}

# Remove the sat-teleop containers and networks; nothing outside the project. On a Podman machine,
# netavark#1497 leaves a forwarding = 0 sysctl drop-in behind for each internal network, which
# would later disable forwarding on any network reusing the bridge name; sat-teleop's are removed.
lab-down:
    #!/usr/bin/env bash
    set -euo pipefail
    bridges=""
    if [ "{{engine}}" = podman ] && podman machine inspect >/dev/null 2>&1; then
        for net in sat-teleop_vehicle_net sat-teleop_ctl_net; do
            bridges="$bridges $(podman network inspect "$net" --format '{{{{.NetworkInterface}}' 2>/dev/null || true)"
        done
    fi
    {{compose_down}} --profile feeder down --remove-orphans
    for bridge in $bridges; do
        [[ $bridge =~ ^[A-Za-z0-9_.-]+$ ]] || continue
        podman machine ssh "ip link show $bridge >/dev/null 2>&1 || sudo rm -f /run/sysctl.d/10-netavark-$bridge.conf"
    done
