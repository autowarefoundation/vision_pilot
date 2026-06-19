#!/usr/bin/env bash
# Build the self-contained ROS2 VisionPilot image. All dependencies (ROS2 Jazzy,
# ackermann_msgs, ONNX Runtime, ipopt/cppad/opencv) are installed inside the image;
# the host needs only Docker. Run from anywhere.
#
#   ./build.sh                 # default x64 + ONNX Runtime 1.22.0
#   ARCH=aarch64 ./build.sh    # arm64
set -euo pipefail

IMAGE="${IMAGE:-visionpilot-ros2}"
ARCH="${ARCH:-x64}"
ONNXRUNTIME_VERSION="${ONNXRUNTIME_VERSION:-1.22.0}"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker not found on PATH. Install Docker to build this image." >&2
    exit 1
fi

# Repo root = two levels up from this script (VisionPilot/Docker/). The build context is
# the repo root so the Dockerfile can COPY VisionPilot/.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "Building ${IMAGE} (ARCH=${ARCH}, ONNX Runtime ${ONNXRUNTIME_VERSION})..."
# BuildKit so the scoped Dockerfile.ros2.dockerignore is honored (it is ignored by the
# legacy builder), keeping the build context small.
DOCKER_BUILDKIT=1 docker build \
    -f "${REPO_ROOT}/VisionPilot/Docker/Dockerfile.ros2" \
    -t "${IMAGE}" \
    --build-arg "ARCH=${ARCH}" \
    --build-arg "ONNXRUNTIME_VERSION=${ONNXRUNTIME_VERSION}" \
    "${REPO_ROOT}"

echo "Done. Image: ${IMAGE}. Next: ./run.sh smoke   (or  ./run.sh bash)"
