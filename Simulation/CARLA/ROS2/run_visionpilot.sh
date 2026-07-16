#!/usr/bin/env bash
# Run VisionPilot against the CARLA ego WITHOUT modifying VisionPilot/.
#
# Uses the OFFICIAL VisionPilot image (visionpilot:gpu-ros2, built by
# VisionPilot/docker/build.sh --gpu --ros2): the binary is baked at /usr/bin/VisionPilot
# with its config + model weights under /usr/share/visionpilot/. The binary resolves
# config/<file> relative to its workdir, falling back to /usr/share/visionpilot/config/
# (modules/common/src/utils.cpp), so the CARLA variants are bind-mounted (read-only)
# over those exact paths — the tracked VisionPilot/ tree stays pristine. The C matrix
# is (re)generated from H_carla.yaml first.
#
# The CARLA bridge (control relay + ego_telemetry) runs INSIDE this same container,
# launched before the binary: cross-container Fast-DDS delivery (bridge -> VP
# /vehicle/speed) proved unreliable — it died after discovery, leaving VP speed-blind.
# One container = one DDS host. The bridge install/ is colcon-built beforehand
# (build_bridge.sh in a ros:jazzy container; drive.sh does it) and mounted from the repo.
#
# Prereqs (see README.md): the visionpilot:gpu-ros2 image, the bridge colcon-built,
# the CARLA cp312 wheel staged (drive.sh does all of that), CARLA + ego up.
# Env overrides: IMAGE, VP_BIN, DISPLAY, VP_CONF, VP_ROS2_CONF.
# NOTE: DISPLAY defaults to :1 here (live sim display).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" # .../Simulation/CARLA/ROS2
ROOT="$(cd "$HERE/../../.." && pwd)"                 # repo root
CFG="$HERE/config"
IMAGE="${IMAGE:-visionpilot:gpu-ros2}"
VP_BIN="${VP_BIN:-/usr/bin/VisionPilot}"
DISPLAY="${DISPLAY:-:1}"
# Conf overrides let a headless/CPU smoke swap the drive conf without editing this script.
VP_CONF="${VP_CONF:-$CFG/visionpilot.carla.conf}"
VP_ROS2_CONF="${VP_ROS2_CONF:-$CFG/visionpilot_ros2.carla.conf}"

die() {
    echo "run_visionpilot: ERROR: $*" >&2
    exit 1
}

# ── Fail fast on every hidden prerequisite ────────────────────────────────────
docker image inspect "$IMAGE" >/dev/null 2>&1 ||
    die "image '$IMAGE' not found — build it: cd VisionPilot/docker && ./build.sh --gpu --ros2"
[ -f "$VP_CONF" ] || die "missing conf $VP_CONF"
[ -f "$VP_ROS2_CONF" ] || die "missing conf $VP_ROS2_CONF"
# H_carla.yaml is mounted over config/H.yaml — without it AutoSteer silently fits 0 lane points.
[ -f "$CFG/H_carla.yaml" ] || die "missing $CFG/H_carla.yaml"
[ -d "$HERE/.carla_pkg/cp312/carla" ] ||
    die "CARLA cp312 wheel not staged at .carla_pkg/cp312 (ego_telemetry needs it) — run drive.sh up"
[ -f "$HERE/install/setup.bash" ] ||
    die "bridge not built (no install/setup.bash) — run build_bridge.sh (drive.sh up does it)"
mkdir -p "$HOME/.cache/vp_cuda"

# 1) derive the CARLA preprocess C matrix from H_carla.yaml (reuses VisionPilot's committed logic)
python3 "$HERE/gen_carla_C_matrix.py"
[ -f "$CFG/homography_C_matrix.yaml" ] || die "gen_carla_C_matrix.py produced no C matrix"

# 2) allow the container to reach the X display (graphic mode — never offscreen for a live drive)
DISPLAY="$DISPLAY" xhost +local: >/dev/null 2>&1 || true

# 3) run bridge + VisionPilot in ONE container (single DDS host); bind-mount the CARLA config
#    over the official-image paths the binary reads (VP/ untouched). The repo is mounted only
#    for the bridge install/ overlay and the staged CARLA wheel.
exec docker run --rm --name vp_drive --gpus all --net=host --ipc=host \
    -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e CUDA_CACHE_PATH=/cache -e CUDA_CACHE_MAXSIZE=2147483648 \
    -e PYTHONPATH=/opt/carla_python \
    -v "$HERE/.carla_pkg/cp312":/opt/carla_python:ro \
    -v "$HOME/.cache/vp_cuda:/cache" -v "$ROOT":/workspace -w /usr/share/visionpilot \
    -v "$VP_CONF:/usr/share/visionpilot/config/vision_pilot.conf:ro" \
    -v "$VP_ROS2_CONF:/usr/share/visionpilot/config/vision_pilot_ros2.conf:ro" \
    -v "$CFG/homography_C_matrix.yaml:/usr/share/visionpilot/config/homography_C_matrix.yaml:ro" \
    -v "$CFG/H_carla.yaml:/usr/share/visionpilot/config/H.yaml:ro" \
    --entrypoint bash "$IMAGE" -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/Simulation/CARLA/ROS2/install/setup.bash
        ros2 launch visionpilot_carla_bridge carla_bridge.launch.py &
        exec $VP_BIN"
