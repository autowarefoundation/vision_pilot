# syntax=docker/dockerfile:1
# CARLA -> ROS2 ego-odometry bridge. A separate image because only RoboStack conda
# satisfies all of: carla 0.10 (cp310-only wheel) + rclpy + glibc >= 2.38. Set
# CARLA_WHL to your carla cp310 wheel, then build.sh / run.sh.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
ENV MAMBA_ROOT_PREFIX=/opt/conda

# Pinning apt versions on a base image is impractical.
# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl bzip2 ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# pipefail so a failed curl aborts the build instead of feeding an empty pipe to tar
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# micromamba
RUN curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest \
      | tar -xvj -C /usr/local bin/micromamba

# RoboStack ROS2 (Humble, Python 3.10) — gives rclpy that coexists with the
# carla cp310 wheel in one interpreter.
RUN micromamba create -y -p /opt/conda/envs/odom \
        -c robostack-staging -c conda-forge \
        python=3.10 \
        ros-humble-rclpy ros-humble-nav-msgs ros-humble-geometry-msgs \
        ros-humble-builtin-interfaces \
        numpy \
 && micromamba clean -ay

# carla 0.10 Python client — wheel is NOT in the repo (too large / binary artifact).
# The build script copies it into the context from $CARLA_WHL and passes the
# basename here so the image stays reproducible without hard-coding the filename.
ARG CARLA_WHL_NAME
RUN test -n "${CARLA_WHL_NAME}" || { echo "ERROR: CARLA_WHL_NAME build-arg is required"; exit 1; }
# Keep the original wheel filename — pip requires the canonical
# name-version-...-platform.whl form and rejects a renamed file.
COPY ${CARLA_WHL_NAME} /tmp/wheel/
RUN micromamba run -p /opt/conda/envs/odom pip install --no-deps "/tmp/wheel/${CARLA_WHL_NAME}" \
 && rm -rf /tmp/wheel

COPY odom_bridge.py /opt/odom_bridge.py

ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENTRYPOINT ["micromamba", "run", "-p", "/opt/conda/envs/odom", "python", "/opt/odom_bridge.py"]
