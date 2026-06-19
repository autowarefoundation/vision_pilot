#!/usr/bin/env bash
# Build the self-contained CARLA GPU runtime image for VisionPilot 1.0.
# All deps (CUDA/cuDNN runtime, ROS2 Jazzy, ONNX Runtime GPU, ipopt/cppad/opencv) are
# baked in; the host needs only Docker + the NVIDIA container runtime. Run from anywhere.
#
#   ./build-carla-gpu.sh                     # x64 + ONNX Runtime GPU 1.23.2
set -euo pipefail

IMAGE="${IMAGE:-visionpilot-carla-gpu}"
ARCH="${ARCH:-x64}"
ONNXRUNTIME_VERSION="${ONNXRUNTIME_VERSION:-1.23.2}"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker not found on PATH." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "Building ${IMAGE} (ARCH=${ARCH}, ONNX Runtime GPU ${ONNXRUNTIME_VERSION})..."
DOCKER_BUILDKIT=1 docker build \
    -f "${REPO_ROOT}/VisionPilot/Docker/Dockerfile.carla-gpu" \
    -t "${IMAGE}" \
    --build-arg "ARCH=${ARCH}" \
    --build-arg "ONNXRUNTIME_VERSION=${ONNXRUNTIME_VERSION}" \
    "${REPO_ROOT}"

echo "Done. Image: ${IMAGE}. Next: ./run-carla-gpu.sh /path/to/weights"
