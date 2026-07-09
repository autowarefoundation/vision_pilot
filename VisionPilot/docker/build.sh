#!/usr/bin/env bash
#
# Build the VisionPilot Docker image, choosing GPU or CPU variant and
# optionally enabling ROS2 support.
#
# Usage:
#   ./build.sh [--gpu|--cpu] [--ros2] [--no-cache] [--tag <name>]
#
# Examples:
#   ./build.sh                    # CPU build, ROS2 off (defaults)
#   ./build.sh --gpu              # GPU build, ROS2 off
#   ./build.sh --gpu --ros2       # GPU build, ROS2 on
#   ./build.sh --cpu --no-cache   # CPU build, force a clean rebuild
#   ./build.sh --gpu --tag myimg:latest   # custom tag instead of the default

set -euo pipefail

VARIANT="gpu"
ENABLE_ROS2="OFF"
NO_CACHE=""
TAG=""

usage() {
    awk '/^#!/{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"
    exit 1
}

while [ $# -gt 0 ]; do
    case "$1" in
        --gpu)
            VARIANT="gpu"
            shift
            ;;
        --cpu)
            VARIANT="cpu"
            shift
            ;;
        --ros2)
            ENABLE_ROS2="ON"
            shift
            ;;
        --no-cache)
            NO_CACHE="--no-cache"
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Error: unknown argument '$1'" >&2
            usage
            ;;
    esac
done

# Sanity checks before doing anything expensive
if ! command -v docker >/dev/null 2>&1; then
    echo "Error: docker is not installed or not on PATH." >&2
    exit 1
fi

#if [ ! -f "CMakeLists.txt" ]; then
#    echo "Error: CMakeLists.txt not found in the current directory." >&2
#    echo "Run this script from inside VisionPilot/ (the build context root)." >&2
#    exit 1
#fi

if [ "$VARIANT" = "gpu" ]; then
    DOCKERFILE="Dockerfile"
else
    DOCKERFILE="Dockerfile.cpu"
fi

if [ ! -f "$DOCKERFILE" ]; then
    echo "Error: $DOCKERFILE not found." >&2
    exit 1
fi

# Default tag reflects the chosen options, e.g. visionpilot:gpu-ros2

TAG="visionpilot:${VARIANT}"
if [ "$ENABLE_ROS2" = "ON" ]; then
    TAG="${TAG}-ros2"
fi

echo "=================================================="
echo " VisionPilot Docker build"
echo "=================================================="
echo " Variant:      $VARIANT"
echo " ROS2 support: $ENABLE_ROS2"
echo " Dockerfile:   $DOCKERFILE"
echo " Image tag:    $TAG"
echo " No cache:     $([ -n "$NO_CACHE" ] && echo yes || echo no)"
echo "=================================================="

docker build $NO_CACHE \
    -t "$TAG" \
    -f "$DOCKERFILE" \
    --build-arg ENABLE_ROS2="$ENABLE_ROS2" \
    ..

echo
echo "Build complete: $TAG"
if [ "$VARIANT" = "gpu" ]; then
    echo "Run with:  docker run --rm -it --gpus all $TAG"
else
    echo "Run with:  docker run --rm -it $TAG"
fi