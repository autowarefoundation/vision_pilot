#!/usr/bin/env bash
# Run the CARLA GPU runtime image: subscribes the CARLA camera, drives, publishes
# ackermann_msgs/AckermannDriveStamped. Build first with ./build-carla-gpu.sh.
#
#   ./run-carla-gpu.sh /abs/path/to/weights_dir   # weights mounted read-only
#
# The weights dir must contain autodrive_fp32.onnx, autosteer_fp32.onnx,
# autospeed_fp32.onnx (matching the config). CARLA must already be running with --ros2
# and an ego + camera spawned (see Simulation/CARLA/ROS2/README.md). Needs the NVIDIA
# container runtime (--gpus all).
#
# Shows the VisionPilot window on the host display by default. Set HEADLESS=1 for an
# offscreen run (CI / no display). The sm_120 JIT cache is persisted to $CUDA_CACHE_DIR
# on the host so repeat runs start fast.
set -euo pipefail

IMAGE="${IMAGE:-visionpilot-carla-gpu}"
DISPLAY_ENV="${DISPLAY:-:0}"
CUDA_CACHE_DIR="${CUDA_CACHE_DIR:-$HOME/.cache/vp_cuda}"
WEIGHTS_DIR="${1:-}"

if ! docker image inspect "${IMAGE}" &>/dev/null; then
    echo "ERROR: Image '${IMAGE}' not found. Run './build-carla-gpu.sh' first." >&2
    exit 1
fi
if [[ -z "${WEIGHTS_DIR}" || ! -d "${WEIGHTS_DIR}" ]]; then
    echo "ERROR: pass an existing weights directory. Usage: $0 /abs/path/to/weights" >&2
    exit 1
fi
WEIGHTS_DIR="$(cd "${WEIGHTS_DIR}" && pwd)"
mkdir -p "${CUDA_CACHE_DIR}"

# Display: graphic by default (window on the host); offscreen with HEADLESS=1.
display_args=(-e QT_QPA_PLATFORM=offscreen)
if [[ "${HEADLESS:-0}" != "1" ]]; then
    xhost +local: >/dev/null 2>&1 || true
    display_args=(
        -e "DISPLAY=${DISPLAY_ENV}" -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1
        -v /tmp/.X11-unix:/tmp/.X11-unix
    )
fi

docker run --rm -it \
    --gpus all \
    --net=host \
    --ipc=host \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    "${display_args[@]}" \
    -v "${CUDA_CACHE_DIR}:/cache" -e CUDA_CACHE_PATH=/cache -e CUDA_CACHE_MAXSIZE=2147483648 \
    -e VISIONPILOT_CONFIG=/autoware/VisionPilot/config/vision_pilot_carla.conf.example \
    -v "${WEIGHTS_DIR}:/autoware/VisionPilot/modules/models/weights:ro" \
    "${IMAGE}" \
    bash -lc 'source /opt/ros/jazzy/setup.bash && exec ./build/VisionPilot'
