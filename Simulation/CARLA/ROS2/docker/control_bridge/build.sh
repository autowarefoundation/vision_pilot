#!/usr/bin/env bash
# Build the carla-control-bridge Docker image.
#
# This bridge converts VisionPilot's agnostic ackermann output
# (/carla/hero/ackermann_control_cmd) into the carla_msgs/CarlaEgoVehicleControl
# that a CARLA UE5 0.10 Shipping ego accepts on /carla/hero/vehicle_control_cmd.
# No external artifacts are needed — everything is in this directory.
#
#   ./build.sh                 # tag carla-control-bridge
set -euo pipefail

IMAGE_NAME="carla-control-bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Building ${IMAGE_NAME}..."
docker build -t "${IMAGE_NAME}" "${SCRIPT_DIR}"

echo "Build complete: ${IMAGE_NAME}"
