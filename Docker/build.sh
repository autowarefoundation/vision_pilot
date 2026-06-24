#!/usr/bin/env bash
# Build the VisionPilot dev/build Docker image.
#
#   ./build.sh             GPU image  (nvidia/cuda base, ONNX Runtime GPU)  -> tag :cuda
#   ./build.sh --no-cuda   CPU image  (ubuntu base, ONNX Runtime CPU)       -> tag :cpu
#                          (CPU image also targets arm64 / Apple Silicon)
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_BASE="autoware-visionpilot-dev"
ORT_VERSION="1.23.2"
USE_CUDA=true
NO_CACHE=""

usage() {
    cat <<EOF
Usage: $0 [options]

  --no-cuda     Build the CPU-only image (ubuntu base; x86_64 or arm64/Apple Silicon).
                Default builds the GPU image (nvidia/cuda base; run with --gpus all).
  --no-cache    Rebuild from scratch (docker build --no-cache).
  -h, --help    Show this help.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --no-cuda)  USE_CUDA=false ;;
        --no-cache) NO_CACHE="--no-cache" ;;
        -h|--help)  usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
    shift
done

ARCH="$(uname -m)"

if ${USE_CUDA}; then
    if [ "${ARCH}" != "x86_64" ]; then
        echo "[build] GPU image supports x86_64 only (host is ${ARCH}); use --no-cuda." >&2
        exit 1
    fi
    BASE_IMAGE="nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04"
    ORT_TARBALL="onnxruntime-linux-x64-gpu-${ORT_VERSION}"
    TAG="${IMAGE_BASE}:cuda"
else
    BASE_IMAGE="ubuntu:24.04"
    case "${ARCH}" in
        x86_64)        ORT_TARBALL="onnxruntime-linux-x64-${ORT_VERSION}" ;;
        aarch64|arm64) ORT_TARBALL="onnxruntime-linux-aarch64-${ORT_VERSION}" ;;
        *) echo "[build] Unsupported architecture: ${ARCH}" >&2; exit 1 ;;
    esac
    TAG="${IMAGE_BASE}:cpu"
fi

echo "[build] base=${BASE_IMAGE}  ort=${ORT_TARBALL}  tag=${TAG}"

# shellcheck disable=SC2086
DOCKER_BUILDKIT=1 docker build ${NO_CACHE} \
    -f "${DIR}/Dockerfile" \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    --build-arg ORT_VERSION="${ORT_VERSION}" \
    --build-arg ORT_TARBALL="${ORT_TARBALL}" \
    -t "${TAG}" \
    "${DIR}"

echo "[build] done: ${TAG}"
