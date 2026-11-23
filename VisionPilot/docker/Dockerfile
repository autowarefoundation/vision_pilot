# VisionPilot — GPU build (CUDA + TensorRT execution provider)

# Build:
#   docker build -t visionpilot:gpu -f docker/Dockerfile.gpu .     (run from inside VisionPilot/)
# Run (requires nvidia-container-toolkit on the host):
#   docker run --rm -it --gpus all -v <TEST_DATA_DIR>:<CONFIG_TEST_DATA_DIR>:ro -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw visionpilot:gpu

# syntax=docker/dockerfile:1
ARG CUDA_TAG=13.0.0-devel-ubuntu24.04
ARG ONNXRUNTIME_VERSION=1.32.0
ARG ONNXRUNTIME_SHA256=3d01949bc846d41961b37988d1e63d782a6effcb93b650a6e0577fef982d38b5
ARG ENABLE_ROS2=OFF
ARG ENABLE_OCCUPANCY=OFF


# build
FROM nvcr.io/nvidia/cuda:${CUDA_TAG} AS builder
ARG ONNXRUNTIME_VERSION
ARG ONNXRUNTIME_SHA256
ARG ENABLE_ROS2
ARG ENABLE_OCCUPANCY
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
    && add-apt-repository -y universe \
    && apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget unzip ca-certificates gnupg \
        python3 python3-pip \
        libopencv-dev \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-bad1.0-dev \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-nice \
        libnice-dev \
        libsrtp2-dev \
        libboost-system-dev \
        nlohmann-json3-dev \
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

# ROS 2 Jazzy — only installed when ENABLE_ROS2=ON.
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

# ONNX Runtime GPU build from the NuGet package Microsoft.ML.OnnxRuntime.Gpu.Linux.
# The GitHub release downloads of ONNX Runtime return 404. The GPU package
# 1.26.0 on NuGet links CUDA 12, so this image uses 1.32.0, which links CUDA 13.
# This step makes the include/ and lib/ layout of the release tarball.
# If you change ONNXRUNTIME_VERSION, also change ONNXRUNTIME_SHA256.
WORKDIR /opt
RUN set -eux; \
    ort_root=/usr/share/visionpilot/onnxruntime; \
    ort_native=/opt/ort_extract/runtimes/linux-x64/native; \
    mkdir -p /opt/ort_extract "${ort_root}/include" "${ort_root}/lib"; \
    wget -q -O ort.nupkg "https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime.gpu.linux/${ONNXRUNTIME_VERSION}/microsoft.ml.onnxruntime.gpu.linux.${ONNXRUNTIME_VERSION}.nupkg"; \
    echo "${ONNXRUNTIME_SHA256}  ort.nupkg" | sha256sum -c -; \
    unzip -q ort.nupkg 'buildTransitive/native/include/*' 'runtimes/linux-x64/native/*' -d /opt/ort_extract; \
    cp /opt/ort_extract/buildTransitive/native/include/* "${ort_root}/include/"; \
    cp "${ort_native}"/libonnxruntime_providers_*.so "${ort_root}/lib/"; \
    cp "${ort_native}/libonnxruntime.so" "${ort_root}/lib/libonnxruntime.so.${ONNXRUNTIME_VERSION}"; \
    ln -s "libonnxruntime.so.${ONNXRUNTIME_VERSION}" "${ort_root}/lib/libonnxruntime.so.1"; \
    ln -s libonnxruntime.so.1 "${ort_root}/lib/libonnxruntime.so"; \
    rm -rf ort.nupkg /opt/ort_extract

WORKDIR /opt/visionpilot
COPY . .

RUN mkdir build && cd build \
    && if [ "$ENABLE_ROS2" = "ON" ]; then . /opt/ros/jazzy/setup.bash; fi \
    && cmake -DONNXRUNTIME_ROOT=/usr/share/visionpilot/onnxruntime -DGPU=ON \
             -DENABLE_ROS2_INTERFACE=${ENABLE_ROS2} \
             -DENABLE_OCCUPANCY=${ENABLE_OCCUPANCY} \
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
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-nice \
        libnice10 \
        libsrtp2-1 \
        libboost-system1.83.0 \
        libcudnn9-cuda-13 \
        libnvinfer10 \
        libnvinfer-plugin10 \
        libnvonnxparsers10 \
        coinor-libipopt-dev \
        liblapack3 \
        libblas3 \
        libcppad-lib1456.0t64 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy runtime libs — only needed when built with ROS 2 support (ENABLE_ROS2=ON).
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

# ROS 2 apt packages don't register /opt/ros/jazzy/lib with ldconfig — normally
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
