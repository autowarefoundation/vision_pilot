#!/usr/bin/env bash
# Run the carla-odom-bridge Docker container.
#
# Build the image first with ./build.sh, then:
#   ./run.sh

set -euo pipefail

IMAGE_NAME="carla-odom-bridge"

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
