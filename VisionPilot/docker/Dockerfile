# VisionPilot — GPU build (CUDA + TensorRT execution provider)

# Build:
#   docker build -t visionpilot:gpu -f docker/Dockerfile.gpu .     (run from inside VisionPilot/)
# Run (requires nvidia-container-toolkit on the host):
#   docker run --rm -it --gpus all -v <TEST_DATA_DIR>:<CONFIG_TEST_DATA_DIR>:ro -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw visionpilot:gpu

# syntax=docker/dockerfile:1
ARG CUDA_TAG=13.0.0-devel-ubuntu24.04
ARG ONNXRUNTIME_VERSION=1.26.0
ARG ENABLE_ROS2=OFF


# build
FROM nvcr.io/nvidia/cuda:${CUDA_TAG} AS builder
ARG ONNXRUNTIME_VERSION
ARG ENABLE_ROS2
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
    && add-apt-repository -y universe \
    && apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget ca-certificates gnupg \
        python3 python3-pip \
        libopencv-dev \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-bad1.0-dev \
        libsoup2.4-dev \
        libjson-glib-dev \
        coinor-libipopt-dev \
        libcppad-dev \
        liblapack-dev \
        libblas-dev \
    && rm -rf /var/lib/apt/lists/*

# CppAD's cppad/ipopt/solve.hpp hardcodes #include <coin-or/IpIpoptApplication.hpp>,
# but Ubuntu 24.04's coinor-libipopt-dev ships an older Ipopt whose headers land
# under a differently-named directory (e.g. /usr/include/coin/ instead of
# /usr/include/coin-or/ — that rename only happened in newer Ipopt packaging).
# Resolve wherever the header actually is rather than hardcoding the name, and
# symlink coin-or to it so CppAD's #include resolves.
RUN hdr_dir="$(dirname "$(find /usr/include -name IpIpoptApplication.hpp | head -1)")" \
    && [ -n "$hdr_dir" ] \
    && ln -sf "$hdr_dir" /usr/include/coin-or

RUN pip3 install --no-cache-dir --break-system-packages opencv-python numpy

# ROS2 Jazzy — only installed when ENABLE_ROS2=ON.
RUN if [ "$ENABLE_ROS2" = "ON" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            curl \
        && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg \
        && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
            > /etc/apt/sources.list.d/ros2.list \
        && apt-get update && apt-get install -y --no-install-recommends \
            ros-jazzy-ros-base \
            ros-jazzy-cv-bridge \
            ros-jazzy-image-transport \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# TensorRT + cuDNN from NVIDIA's CUDA network repo (already configured in the
# nvidia/cuda base image). Package names/versions must match your CUDA_TAG —
# check `apt-cache search tensorrt` / `libnvinfer` inside the image if this fails.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libcudnn9-cuda-13 \
        libnvinfer10 \
        libnvinfer-plugin10 \
        libnvonnxparsers10 \
        libnvinfer-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN mkdir -p /usr/share/visionpilot /opt/ort_extract \
    && wget -q -O ort.tgz "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-gpu_cuda13-${ONNXRUNTIME_VERSION}.tgz" \
    && tar -xzf ort.tgz -C /opt/ort_extract \
    && mv /opt/ort_extract/*/ /usr/share/visionpilot/onnxruntime \
    && rm -rf ort.tgz /opt/ort_extract

WORKDIR /opt/visionpilot
COPY . .

RUN mkdir build && cd build \
    && if [ "$ENABLE_ROS2" = "ON" ]; then . /opt/ros/jazzy/setup.bash; fi \
    && cmake -DONNXRUNTIME_ROOT=/usr/share/visionpilot/onnxruntime -DGPU=ON \
             -DENABLE_ROS2_INTERFACE=${ENABLE_ROS2} \
             -DCMAKE_CXX_STANDARD_LIBRARIES="-lcppad_lib" .. \
    && make -j"$(nproc)" VisionPilot

# runtime
FROM nvcr.io/nvidia/cuda:${CUDA_TAG%-devel*}-runtime-ubuntu24.04 AS runtime
ARG ENABLE_ROS2
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
    && add-apt-repository -y universe \
    && apt-get update && apt-get install -y --no-install-recommends \
        libopencv-dev \
        libgstreamer1.0-0 \
        libgstreamer-plugins-base1.0-0 \
        libgstreamer-plugins-bad1.0-0 \
        libsoup2.4-1 \
        libjson-glib-1.0-0 \
        libcudnn9-cuda-13 \
        libnvinfer10 \
        libnvinfer-plugin10 \
        libnvonnxparsers10 \
        coinor-libipopt-dev \
        liblapack3 \
        libblas3 \
        libcppad-lib1456.0t64 \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Jazzy runtime libs — only needed when built with ROS2 support (ENABLE_ROS2=ON).
# Mirrors the builder-stage install so all transitive apt dependencies (fastrtps,
# tinyxml2, python3, etc.) are present here too, not just in the builder image.
RUN if [ "$ENABLE_ROS2" = "ON" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            curl \
        && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg \
        && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
            > /etc/apt/sources.list.d/ros2.list \
        && apt-get update && apt-get install -y --no-install-recommends \
            ros-jazzy-ros-base \
            ros-jazzy-cv-bridge \
            ros-jazzy-image-transport \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# ROS2 apt packages don't register /opt/ros/jazzy/lib with ldconfig — normally
# `setup.bash` sets LD_LIBRARY_PATH for you, but ENTRYPOINT runs the binary
# directly with no shell in between to source it. Set it explicitly instead.
ENV LD_LIBRARY_PATH="/opt/ros/jazzy/lib:${LD_LIBRARY_PATH}"

RUN mkdir -p /usr/share/visionpilot

COPY --from=builder /usr/share/visionpilot/onnxruntime /usr/share/visionpilot/onnxruntime

COPY --from=builder /opt/visionpilot/build/VisionPilot /usr/bin/VisionPilot
COPY --from=builder /opt/visionpilot/build/config /usr/share/visionpilot/config
COPY --from=builder /opt/visionpilot/assets/icons /usr/share/visionpilot/assets/icons
COPY --from=builder /opt/visionpilot/modules/models/weights /usr/share/visionpilot/modules/models/weights

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics,display

WORKDIR /usr/share/visionpilot
ENTRYPOINT ["/usr/bin/VisionPilot"]