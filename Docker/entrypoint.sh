#!/usr/bin/env bash
# Source ROS2 then exec the requested command (default: interactive bash).
set -e

# shellcheck disable=SC1091
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash

exec "$@"
