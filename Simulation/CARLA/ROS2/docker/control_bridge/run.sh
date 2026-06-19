#!/usr/bin/env bash
# Run the carla-control-bridge Docker container.
#
# Build the image first with ./build.sh, then:
#   ./run.sh
#
# Topics / gains are overridable via env (ACK_TOPIC, ODOM_TOPIC, CTRL_TOPIC,
# MAX_STEER_RAD, SPEED_KP, THROTTLE_MAX, BRAKE_DEADBAND) — see control_bridge.py.
set -euo pipefail

IMAGE_NAME="carla-control-bridge"

if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
    echo "ERROR: Image '${IMAGE_NAME}' not found. Run './build.sh' first."
    exit 1
fi

echo "Starting ${IMAGE_NAME}..."
docker run --rm \
    --net=host \
    --ipc=host \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    "${IMAGE_NAME}"
