#!/usr/bin/env bash
# Run the self-contained ROS2 VisionPilot image with the right ROS2/DDS init so it can
# talk to host `ros2` tools and other containers. No host ROS2 install needed.
#
#   ./run.sh smoke    # in-process publish->subscribe round-trip test (no weights needed)
#   ./run.sh echo     # ros2 topic echo /control/ackermann_cmd
#   ./run.sh bash     # interactive shell inside the container
#   ./run.sh run      # launch the app (needs a config + model weights, see below)
#
# Env overrides: IMAGE, ROS_DOMAIN_ID, CONTROL_TOPIC, CONFIG (host conf to mount),
# WEIGHTS_DIR (host model-weights dir to mount at /autoware/VisionPilot/models).
set -euo pipefail

IMAGE="${IMAGE:-visionpilot-ros2}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
CONTROL_TOPIC="${CONTROL_TOPIC:-/control/ackermann_cmd}"
MODE="${1:-smoke}"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker not found on PATH." >&2
    exit 1
fi

# Common docker args. --net=host so DDS discovery reaches host ros2 tools / other
# containers; FASTDDS_BUILTIN_TRANSPORTS=UDPv4 because host<->container Fast-DDS shared
# memory does not reliably connect across the boundary (see CLAUDE.md).
DOCKER_ARGS=(
    --rm
    --net=host
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    -e "FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
    -e "QT_QPA_PLATFORM=offscreen"
)

# Optional host mounts (uncomment-by-env): a user config and the model weights.
if [[ -n "${CONFIG:-}" ]]; then
    DOCKER_ARGS+=(-v "${CONFIG}:/autoware/VisionPilot/config/vision_pilot.conf:ro")
fi
if [[ -n "${WEIGHTS_DIR:-}" ]]; then
    DOCKER_ARGS+=(-v "${WEIGHTS_DIR}:/autoware/VisionPilot/models:ro")
fi

# Everything runs with ROS2 sourced and from the app's working directory.
ros_prelude='. /opt/ros/jazzy/setup.sh && cd /autoware/VisionPilot'

case "${MODE}" in
    smoke)
        # Real runtime proof of the publisher: publishes a known ControlCommand and reads
        # it back through a subscriber in one process. No weights / camera required.
        docker run "${DOCKER_ARGS[@]}" "${IMAGE}" \
            bash -lc "${ros_prelude} && ./build/test_control_cmd_publisher"
        ;;
    echo)
        docker run -it "${DOCKER_ARGS[@]}" "${IMAGE}" \
            bash -lc "${ros_prelude} && ros2 topic echo ${CONTROL_TOPIC}"
        ;;
    run)
        # Full app: requires a config and model weights (mount via CONFIG / WEIGHTS_DIR).
        docker run -it "${DOCKER_ARGS[@]}" "${IMAGE}" \
            bash -lc "${ros_prelude} && ./build/VisionPilot --config config/vision_pilot.conf"
        ;;
    bash)
        docker run -it "${DOCKER_ARGS[@]}" "${IMAGE}" \
            bash -lc "${ros_prelude} && exec bash"
        ;;
    *)
        echo "Usage: $0 {smoke|echo|run|bash}" >&2
        exit 2
        ;;
esac
