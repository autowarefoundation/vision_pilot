# syntax=docker/dockerfile:1
# CARLA ego odometry publisher. CARLA UE5 0.10 --ros2 does NOT publish ego odometry, so
# VisionPilot's vehicle_state_subscriber (live ego speed) and the control bridge's speed
# P-controller have no source. This node reads the ego state via the CARLA PythonAPI and
# publishes nav_msgs/Odometry on /carla/hero/odometry. See build.sh / run.sh.
#
# ros:jazzy-ros-base is Ubuntu 24.04 / Python 3.12, so build.sh stages the matching
# carla-0.10.0-cp312 wheel (from your CARLA install) + pub_odom_node.py into the context.
FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# tf_transformations (+ its transforms3d backend) ships as a ROS apt package, not on PyPI.
# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-nav-msgs \
        ros-jazzy-geometry-msgs \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf-transformations \
        python3-numpy \
        python3-pip \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
# Staged by build.sh: the version-matched CARLA PythonAPI wheel + the node.
COPY carla-*.whl /tmp/
RUN pip3 install --no-cache-dir --break-system-packages /tmp/carla-*.whl
COPY pub_odom_node.py /ws/pub_odom_node.py

ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENTRYPOINT ["bash", "-lc", \
  "source /opt/ros/jazzy/setup.bash && exec python3 /ws/pub_odom_node.py"]
