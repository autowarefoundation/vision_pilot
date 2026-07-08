#!/usr/bin/env bash
# Run VisionPilot against the CARLA ego WITHOUT modifying VisionPilot/.
#
# The binary loads config/vision_pilot{,_ros2}.conf and config/homography_C_matrix.yaml
# relative to its CWD (/workspace/VisionPilot) and ignores --config. Instead of copying the
# CARLA variants into the tracked tree, we bind-mount them (read-only) over those exact paths,
# so the host VisionPilot/ stays pristine. The C matrix is (re)generated from H_carla.yaml first.
#
# The CARLA bridge (control relay + ego_telemetry) runs INSIDE this same container, launched
# before the binary: cross-container Fast-DDS delivery (bridge -> VP /vehicle/speed) proved
# unreliable — it died after discovery, leaving VP speed-blind. One container = one DDS host.
#
# Prereqs (see README.md): VisionPilot built with -DENABLE_ROS2_INTERFACE=ON into
# VisionPilot/build_docker_ros2/ (Docker/run.sh --test builds it), the bridge colcon-built
# (build_bridge.sh), the CARLA cp312 wheel staged (drive.sh does all of that), CARLA + ego up.
# Override IMAGE / VP_BIN / DISPLAY via env.
# NOTE: DISPLAY defaults to :1 here (live sim display); Docker/run.sh defaults to :0.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" # .../Simulation/CARLA/ROS2
ROOT="$(cd "$HERE/../../.." && pwd)"                 # repo root
CFG="$HERE/config"
IMAGE="${IMAGE:-autoware-visionpilot-dev:cuda}"
VP_BIN="${VP_BIN:-./build_docker_ros2/VisionPilot}"
DISPLAY="${DISPLAY:-:1}"

die() {
    echo "run_visionpilot: ERROR: $*" >&2
    exit 1
}

# ── Fail fast on every hidden prerequisite ────────────────────────────────────
docker image inspect "$IMAGE" >/dev/null 2>&1 ||
    die "image '$IMAGE' not found — build it with Docker/build.sh"
[ -x "$ROOT/VisionPilot/${VP_BIN#./}" ] ||
    die "VisionPilot binary missing at VisionPilot/${VP_BIN#./} — build it first:
  Docker/run.sh --test    # or: cmake -B build_docker_ros2 -DENABLE_ROS2_INTERFACE=ON && cmake --build build_docker_ros2"
for f in visionpilot.carla.conf visionpilot_ros2.carla.conf H_carla.yaml; do
    # H_carla.yaml is mounted over config/H.yaml — without it AutoSteer silently fits 0 lane points.
    [ -f "$CFG/$f" ] || die "missing $CFG/$f"
done
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
#    over the paths the binary reads (VP/ untouched)
exec docker run --rm --name vp_drive --gpus all --net=host --ipc=host \
    -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e CUDA_CACHE_PATH=/cache -e CUDA_CACHE_MAXSIZE=2147483648 \
    -e PYTHONPATH=/opt/carla_python \
    -v "$HERE/.carla_pkg/cp312":/opt/carla_python:ro \
    -v "$HOME/.cache/vp_cuda:/cache" -v "$ROOT":/workspace -w /workspace/VisionPilot \
    -v "$CFG/visionpilot.carla.conf:/workspace/VisionPilot/config/vision_pilot.conf:ro" \
    -v "$CFG/visionpilot_ros2.carla.conf:/workspace/VisionPilot/config/vision_pilot_ros2.conf:ro" \
    -v "$CFG/homography_C_matrix.yaml:/workspace/VisionPilot/config/homography_C_matrix.yaml:ro" \
    -v "$CFG/H_carla.yaml:/workspace/VisionPilot/config/H.yaml:ro" \
    "$IMAGE" bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/Simulation/CARLA/ROS2/install/setup.bash
        ros2 launch visionpilot_carla_bridge carla_bridge.launch.py &
        exec $VP_BIN"
