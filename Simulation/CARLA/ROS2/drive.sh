#!/usr/bin/env bash
# One-command CARLA <-> VisionPilot drive orchestrator:  ./drive.sh {up|down|status}
#
#   up      full teardown, then bring everything up fresh (mandatory between experiments —
#           stale actors / lingering DDS topics from a previous run corrupt the next one):
#             1. CARLA server  — on the HOST, from $CARLA_ROOT, WINDOWED (never offscreen)
#             2. spawn helper  — on the HOST (ros_carla_config.py: ego + main_cam; pure
#                                PythonAPI, NO ROS — host needs no rclpy)
#             3. bridge build  — one-shot colcon build of visionpilot_carla_bridge
#             4. VisionPilot + bridge — ONE dev container, graphic mode (run_visionpilot.sh
#                                launches the control relay + ego_telemetry beside the binary:
#                                cross-container DDS delivery proved unreliable, so every ROS 2
#                                node we own shares a single container)
#   down    stop all components; verify port 2000 free and no vp_* containers
#   status  show what is running
#
# CARLA is NEVER run in Docker — only VisionPilot and the bridge are containerized.
#
# Env (all optional):
#   CARLA_ROOT    CARLA install dir              (default: $HOME/CARLA_0.9.16)
#   RIG_JSON      ego/sensor rig                 (default: config/VisionPilot_carla916.json)
#   SPAWN_PYTHON  host python for the spawn helper; a CARLA wheel matching its version
#                 is staged automatically from $CARLA_ROOT (default: python3)
#   DISPLAY       X display for CARLA + VisionPilot windows (default: :1)
#   SPAWN_INDEX   spawn-point override (default: "spawn_index" in RIG_JSON)
#   IMAGE         dev-container image            (default: autoware-visionpilot-dev:cuda)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" # .../Simulation/CARLA/ROS2
ROOT="$(cd "$HERE/../../.." && pwd)"                 # repo root
CARLA_ROOT="${CARLA_ROOT:-$HOME/CARLA_0.9.16}"
RIG_JSON="${RIG_JSON:-$HERE/config/VisionPilot_carla916.json}"
SPAWN_PYTHON="${SPAWN_PYTHON:-python3}"
DISPLAY="${DISPLAY:-:1}"
IMAGE="${IMAGE:-autoware-visionpilot-dev:cuda}"
PKG_DIR="$HERE/.carla_pkg" # staged CARLA wheel (gitignored)
CARLA_LOG=/tmp/carla_server.log
SPAWN_LOG=/tmp/carla_spawn.log

die() {
    echo "drive.sh: ERROR: $*" >&2
    exit 1
}

port_2000_pids() { ss -ltnp 2>/dev/null | grep ':2000 ' | grep -oP 'pid=\K[0-9]+' | sort -u; }

down() {
    echo "[down] stopping VisionPilot(+bridge) container"
    docker rm -f vp_drive vp_bridge vp_bridge_build >/dev/null 2>&1 || true
    echo "[down] stopping spawn helper"
    pkill -9 -f ros_carla_config.py 2>/dev/null || true
    echo "[down] stopping CARLA server (by port-2000 PID)"
    for pid in $(port_2000_pids); do kill -9 "$pid" 2>/dev/null || true; done
    pkill -9 -f 'CarlaUE4-Linux-Shipping|CarlaUnreal-Linux-Shipping' 2>/dev/null || true
    sleep 2
    [ -z "$(port_2000_pids)" ] || die "port 2000 still busy after teardown"
    [ -z "$(docker ps -q --filter name=vp_ 2>/dev/null)" ] || die "vp_* containers still running"
    echo "[down] clean (port 2000 free, no vp_* containers)"
}

status() {
    echo "── containers ──"
    docker ps --filter name=vp_ --format '{{.Names}}  {{.Status}}' || true
    echo "── CARLA (port 2000) ──"
    port_2000_pids | sed 's/^/pid /' || true
    pgrep -af ros_carla_config.py | sed 's/^/spawn helper: /' || echo "spawn helper: not running"
    command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=memory.used --format=csv,noheader
}

# Stage the CARLA wheel matching a python tag into .carla_pkg/<tag>/ (PYTHONPATH import).
stage_carla_wheel() {
    local pytag="$1" wheel wheels
    wheels=("$CARLA_ROOT"/PythonAPI/carla/dist/carla-*"$pytag"*.whl)
    wheel="${wheels[0]}"
    [ -f "$wheel" ] || die "no CARLA wheel for $pytag under $CARLA_ROOT/PythonAPI/carla/dist —
  available: $(cd "$CARLA_ROOT/PythonAPI/carla/dist" 2>/dev/null && echo ./*.whl)"
    if [ ! -d "$PKG_DIR/$pytag/carla" ] || [ "$wheel" -nt "$PKG_DIR/$pytag/carla" ]; then
        echo "[up] staging CARLA wheel $(basename "$wheel") -> .carla_pkg/$pytag/"
        rm -rf "${PKG_DIR:?}/$pytag"
        mkdir -p "$PKG_DIR/$pytag"
        unzip -qo "$wheel" -d "$PKG_DIR/$pytag"
    fi
}

# Python tag of the dev container (Ubuntu 24.04 -> 3.12); ego_telemetry runs there.
CONTAINER_PYTAG=cp312

up() {
    # ── Fail fast on every prerequisite ──────────────────────────────────────
    [ -d "$CARLA_ROOT" ] || die "CARLA_ROOT '$CARLA_ROOT' does not exist (set CARLA_ROOT)"
    local launcher
    if [ -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
        launcher="$CARLA_ROOT/CarlaUE4.sh" # 0.9.x
    elif [ -x "$CARLA_ROOT/CarlaUnreal.sh" ]; then
        launcher="$CARLA_ROOT/CarlaUnreal.sh" # 0.10
    else die "no CarlaUE4.sh / CarlaUnreal.sh in $CARLA_ROOT"; fi
    [ -f "$RIG_JSON" ] || die "rig json '$RIG_JSON' not found"
    docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image '$IMAGE' missing — Docker/build.sh"
    local host_pytag
    host_pytag="$("$SPAWN_PYTHON" -c 'import sys; print("cp%d%d" % sys.version_info[:2])')"
    stage_carla_wheel "$host_pytag"      # spawn helper (host)
    stage_carla_wheel "$CONTAINER_PYTAG" # ego_telemetry (bridge container)

    down

    echo "[up] 1/4 CARLA server (windowed, --ros2) from $CARLA_ROOT — log: $CARLA_LOG"
    DISPLAY="$DISPLAY" "$launcher" -nosound --ros2 >"$CARLA_LOG" 2>&1 &
    local waited=0
    until [ -n "$(port_2000_pids)" ]; do
        sleep 2
        waited=$((waited + 2))
        [ "$waited" -lt 90 ] || die "CARLA did not open port 2000 in 90 s — see $CARLA_LOG"
    done
    sleep 5 # let the world settle before API connections

    echo "[up] 2/4 spawn helper (host, pure PythonAPI) — rig $(basename "$RIG_JSON") — log: $SPAWN_LOG"
    (cd "$HERE" &&
        PYTHONPATH="$PKG_DIR/$host_pytag${PYTHONPATH:+:$PYTHONPATH}" \
            ${SPAWN_INDEX:+SPAWN_INDEX=$SPAWN_INDEX} \
            "$SPAWN_PYTHON" ros_carla_config.py -f "$RIG_JSON" >"$SPAWN_LOG" 2>&1) &
    sleep 6
    pgrep -f ros_carla_config.py >/dev/null || die "spawn helper died — see $SPAWN_LOG"

    echo "[up] 3/4 building the bridge (one-shot container)"
    docker run --rm --name vp_bridge_build \
        -v "$ROOT":/workspace -w /workspace/Simulation/CARLA/ROS2 \
        "$IMAGE" bash -lc 'source /opt/ros/jazzy/setup.bash && ./build_bridge.sh' >/dev/null

    echo "[up] 4/4 VisionPilot + bridge (one container vp_drive, graphic) — ./drive.sh down to stop"
    DISPLAY="$DISPLAY" "$HERE/run_visionpilot.sh"
}

case "${1:-}" in
up) up ;;
down) down ;;
status) status ;;
*)
    echo "usage: $0 {up|down|status}   (env: CARLA_ROOT RIG_JSON SPAWN_PYTHON DISPLAY SPAWN_INDEX IMAGE)"
    exit 2
    ;;
esac
