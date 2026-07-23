#!/usr/bin/env bash
# Build the VisionPilot<->CARLA bridge (carla_msgs + visionpilot_carla_bridge).
#
# Agnostic: runs identically on a host with ROS 2 Jazzy or in any colcon-capable
# container (the official VisionPilot image ships no colcon — drive.sh uses ros:jazzy).
# Builds only the two bridge packages (--packages-select).
#
#   # host:
#   ./build_bridge.sh
#   # container (what drive.sh does):
#   docker run --rm -v "$(git rev-parse --show-toplevel)":/workspace \
#       -w /workspace/Simulation/CARLA/ROS2 ros:jazzy bash -lc './build_bridge.sh'
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

# A build tree created under a different mount prefix (host vs /ws vs /workspace)
# poisons CMakeCache and colcon fails; detect and start clean.
for cache in build/*/CMakeCache.txt; do
    [ -f "$cache" ] || continue
    if ! grep -q "=${WS_DIR}/build/" "$cache"; then
        echo "[build_bridge] stale build tree (different mount prefix) — cleaning build/ install/"
        rm -rf build install
        break
    fi
done

colcon build --packages-select carla_msgs visionpilot_carla_bridge "$@"

echo "[build_bridge] done. Source the overlay with:"
echo "    source ${WS_DIR}/install/setup.bash"
