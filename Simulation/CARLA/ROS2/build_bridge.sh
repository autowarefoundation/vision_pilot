#!/usr/bin/env bash
# Build the VisionPilot<->CARLA bridge (carla_msgs + visionpilot_carla_bridge).
#
# Agnostic: runs identically on a host with ROS 2 Jazzy or inside the VisionPilot
# dev container. Builds only the two bridge packages (--packages-select) so the
# legacy shared-memory packages under src/ are skipped.
#
#   # host:
#   ./build_bridge.sh
#   # dev container:
#   ../../../Docker/run.sh -- bash -lc 'cd /workspace/Simulation/CARLA/ROS2 && ./build_bridge.sh'
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

# ROS 2's setup.bash references unbound vars; relax -u only while sourcing it.
# shellcheck disable=SC1090
if [ -f "${ROS_SETUP}" ]; then
    set +u
    source "${ROS_SETUP}"
    set -u
fi

cd "${WS_DIR}"
colcon build --packages-select carla_msgs visionpilot_carla_bridge "$@"

echo "[build_bridge] done. Source the overlay with:"
echo "    source ${WS_DIR}/install/setup.bash"
