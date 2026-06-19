#!/usr/bin/env bash
# Build the carla-odom-publisher Docker image.
#
# CARLA 0.10 does not publish ego odometry, so this node reads it via the CARLA PythonAPI
# and republishes nav_msgs/Odometry on /carla/hero/odometry. The image needs the
# version-matched CARLA wheel (cp312, since ros:jazzy is Python 3.12), which lives in your
# CARLA install — not the repo. Point CARLA_ROOT at it:
#
#   CARLA_ROOT=/home/you/Carla-0.10.0-Linux-Shipping ./build.sh
set -euo pipefail

IMAGE_NAME="carla-odom-publisher"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CARLA_ROOT="${CARLA_ROOT:-}"

if [[ -z "${CARLA_ROOT}" ]]; then
    echo "ERROR: set CARLA_ROOT to your CARLA install (for the cp312 PythonAPI wheel)." >&2
    exit 1
fi
# `|| true` so the failing glob does not trip `set -e` before the clear error below.
WHEEL=$(ls "${CARLA_ROOT}"/PythonAPI/carla/dist/carla-*-cp312-*.whl 2>/dev/null | head -1 || true)
if [[ -z "${WHEEL}" ]]; then
    echo "ERROR: no carla cp312 wheel under ${CARLA_ROOT}/PythonAPI/carla/dist/." >&2
    exit 1
fi

STAGE="$(mktemp -d)"
trap 'rm -rf "${STAGE}"' EXIT
cp "${WHEEL}" "${STAGE}/"
cp "${SCRIPT_DIR}/Dockerfile" "${STAGE}/"
cp "${SCRIPT_DIR}/../../src/odom_publisher/odom_publisher/pub_odom_node.py" "${STAGE}/"

echo "Building ${IMAGE_NAME} (wheel: $(basename "${WHEEL}"))..."
docker build -t "${IMAGE_NAME}" "${STAGE}"
echo "Build complete: ${IMAGE_NAME}"
