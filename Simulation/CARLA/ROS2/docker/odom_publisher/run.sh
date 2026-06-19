#!/usr/bin/env bash
# Run the carla-odom-publisher container. Build first with CARLA_ROOT=... ./build.sh.
# CARLA must be running with --ros2 and the ego ('hero') spawned. --net=host so it reaches
# both the CARLA RPC (localhost:2000) and the DDS bus.
set -euo pipefail

IMAGE_NAME="carla-odom-publisher"

if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
    echo "ERROR: Image '${IMAGE_NAME}' not found. Run 'CARLA_ROOT=... ./build.sh' first." >&2
    exit 1
fi

echo "Starting ${IMAGE_NAME}..."
docker run --rm \
    --net=host \
    --ipc=host \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    "${IMAGE_NAME}"
