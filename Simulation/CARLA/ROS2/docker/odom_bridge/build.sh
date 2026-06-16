#!/usr/bin/env bash
# Build the carla-odom-bridge Docker image.
#
# The CARLA Python 3.10 wheel is NOT in the repo. Download it from the CARLA
# releases page and set CARLA_WHL before running this script:
#
#   export CARLA_WHL=/path/to/carla-0.10.0-cp310-cp310-linux_x86_64.whl
#   ./build.sh

set -euo pipefail

IMAGE_NAME="carla-odom-bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "${CARLA_WHL:-}" ]]; then
    echo "ERROR: CARLA_WHL is not set."
    echo "  export CARLA_WHL=/path/to/carla-0.10.0-cp310-cp310-linux_x86_64.whl"
    exit 1
fi

if [[ ! -f "${CARLA_WHL}" ]]; then
    echo "ERROR: CARLA_WHL file not found: ${CARLA_WHL}"
    exit 1
fi

WHL_NAME="$(basename "${CARLA_WHL}")"
TMP_COPY="${SCRIPT_DIR}/${WHL_NAME}"

if [[ "${CARLA_WHL}" != "${TMP_COPY}" ]]; then
    cp "${CARLA_WHL}" "${TMP_COPY}"
    trap 'rm -f "${TMP_COPY}"' EXIT
fi

echo "Building ${IMAGE_NAME} with wheel: ${WHL_NAME}"
docker build \
    --build-arg CARLA_WHL_NAME="${WHL_NAME}" \
    -t "${IMAGE_NAME}" \
    "${SCRIPT_DIR}"

echo "Build complete: ${IMAGE_NAME}"
