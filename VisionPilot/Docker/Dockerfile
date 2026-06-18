# Canonical build image for the VisionPilot single-binary app (app/ + modules/).
#
# Builds the full perception -> planning -> control tree the team's way: Ubuntu 24.04
# with apt-provided C++ dependencies + the ONNX Runtime SDK. No conda. This mirrors the
# distro the OpenADKit images use and is the reproducible, exportable build for the
# single binary (the conda env under .claude/skills is only a local no-sudo convenience).
#
# Build from the repository root (build context = repo root):
#   docker build -f VisionPilot/Docker/Dockerfile -t visionpilot-app \
#       --build-arg ARCH=x64 --build-arg ONNXRUNTIME_VERSION=1.22.0 .
#
# Run the binary (offscreen, no display) inside the image:
#   docker run --rm visionpilot-app \
#       bash -lc 'cd /autoware/VisionPilot && QT_QPA_PLATFORM=offscreen ./build/VisionPilot --help || true'

# x64 or aarch64 (matches the onnxruntime release artifact names)
ARG ARCH=x64
ARG ONNXRUNTIME_VERSION=1.22.0

FROM ubuntu:24.04 AS build
ARG ARCH
ARG ONNXRUNTIME_VERSION
ARG DEBIAN_FRONTEND=noninteractive

# Toolchain + the single-binary app's C++ dependencies (all from apt):
#   - libcppad-dev (+ libcppad-lib): MPC automatic differentiation in modules/planning.
#       Modern CppAD ships CppAD::local::temp_file() in the runtime lib libcppad_lib.
#   - coinor-libipopt-dev: the MPC NLP solver. Its pkg-config --libs pulls
#       -llapack -lblas -ldmumps_seq -lgfortran, so install those dev packages too.
#   - libeigen3-dev / libopencv-dev / libyaml-cpp-dev: linear algebra / vision / config.
#   - python3 + numpy + opencv: the generate_config homography codegen (uses cv2).
# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        git \
        wget \
        ca-certificates \
        libopencv-dev \
        libyaml-cpp-dev \
        libeigen3-dev \
        coinor-libipopt-dev \
        libcppad-dev \
        liblapack-dev \
        libblas-dev \
        libmumps-seq-dev \
        gfortran \
        python3 \
        python3-numpy \
        python3-opencv \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu's IPOPT installs its headers under the legacy /usr/include/coin prefix, but
# CppAD's ipopt binding includes <coin-or/IpIpoptApplication.hpp> (the modern prefix).
# Bridge the two so the apt headers are reachable as coin-or/ (distro packaging lag,
# not a source bug; resolved from the default /usr/include search path).
RUN ln -sfn /usr/include/coin /usr/include/coin-or

# ONNX Runtime SDK at a fixed path. The CPU provider is enough to build and run the
# engine; the TensorRT/CUDA providers are a runtime-only concern (see CLAUDE.md).
RUN wget -q "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-${ARCH}-${ONNXRUNTIME_VERSION}.tgz" \
    && tar -xzf "onnxruntime-linux-${ARCH}-${ONNXRUNTIME_VERSION}.tgz" \
    && mv "onnxruntime-linux-${ARCH}-${ONNXRUNTIME_VERSION}" /opt/onnxruntime \
    && rm "onnxruntime-linux-${ARCH}-${ONNXRUNTIME_VERSION}.tgz"
ENV ONNXRUNTIME_ROOT=/opt/onnxruntime

WORKDIR /autoware
COPY VisionPilot /autoware/VisionPilot

# Configure + build the single binary. The homography codegen reads the in-tree ground
# homography (config/H.yaml) and writes the preprocess matrix to build/config/; the app
# reads it relative to the working directory, so we build into the in-tree `build/` dir the
# app expects. Model weights ship in modules/models/weights/. We build the VisionPilot
# target (the app), which is what this image exists to provide.
WORKDIR /autoware/VisionPilot
RUN cmake -B build \
        -DONNXRUNTIME_ROOT=/opt/onnxruntime \
    && cmake --build build --target VisionPilot -j"$(nproc)"

CMD ["bash"]
