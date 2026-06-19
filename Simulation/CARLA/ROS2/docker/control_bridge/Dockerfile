# syntax=docker/dockerfile:1
# CARLA ackermann -> vehicle-control bridge. Builds a minimal carla_msgs package (so
# the type matches CARLA's native --ros2 DDS) and runs control_bridge.py. See
# build.sh / run.sh.
FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# Pinning apt versions on a ROS base image is impractical.
# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ackermann-msgs \
        ros-jazzy-nav-msgs \
        ros-jazzy-std-msgs \
        python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY carla_msgs /ws/src/carla_msgs
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select carla_msgs

COPY bridge_logic.py /ws/bridge_logic.py
COPY control_bridge.py /ws/control_bridge.py

ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENTRYPOINT ["bash", "-lc", \
  "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec python3 /ws/control_bridge.py"]
