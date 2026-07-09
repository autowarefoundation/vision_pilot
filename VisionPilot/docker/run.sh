#!/usr/bin/env bash
#
# Run the VisionPilot Docker container — GPU or CPU variant (default: GPU).
#
#
# Usage:
#   ./run.sh [--gpu|--cpu] [--ros2] [--v4l2 <host_device>[:<container_device>]] [--data <host_dir>[:<container_dir>]] [--no-display] [--no-xhost]
#
# Examples:
#   ./run.sh                                  # GPU build, no test data/display
#   ./run.sh --cpu                            # CPU build
#   ./run.sh --v4l2 /dev/video0:/dev/video0   # CPU build
#   ./run.sh --data /data                     # same path in container
#   ./run.sh --data /host/data:/data          # mounted at a different path
#   ./run.sh --gpu --ros2                     # matches the -ros2 tag suffix from build.sh
#
# Anything after a literal `--` is passed through as extra arguments to
# VisionPilot itself:
#   ./run.sh --cpu -- --some-app-flag value

set -euo pipefail

VARIANT="gpu"
V4L2=""
ENABLE_ROS2="OFF"
TAG=""
DATA_DIR=""
NO_DISPLAY=""
NO_XHOST=""

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
        --v4l2)
            [ $# -ge 2 ] || { echo "Error: --v4l2 requires a value" >&2; exit 1; }
            V4L2="$2"
            shift 2
            ;;
        --ros2)
            ENABLE_ROS2="ON"
            shift
            ;;
        --no-ros2)
            ENABLE_ROS2="OFF"
            shift
            ;;
        --data)
            [ $# -ge 2 ] || { echo "Error: --data requires a value" >&2; exit 1; }
            DATA_DIR="$2"
            shift 2
            ;;
        --no-display)
            NO_DISPLAY="1"
            shift
            ;;
        --no-xhost)
            NO_XHOST="1"
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

if ! command -v docker >/dev/null 2>&1; then
    echo "Error: docker is not installed or not on PATH." >&2
    exit 1
fi

# Default tag matches build.sh's naming convention
TAG="visionpilot:${VARIANT}"
if [ "$ENABLE_ROS2" = "ON" ]; then
    TAG="${TAG}-ros2"
fi


if ! docker image inspect "$TAG" >/dev/null 2>&1; then
    echo "Error: image '$TAG' not found locally." >&2
    echo "Build it first, e.g.: ./build.sh --${VARIANT}$( [ "$ENABLE_ROS2" = "ON" ] && echo " --ros2" )" >&2
    exit 1
fi

DOCKER_ARGS=(--rm -it)

if [ "$VARIANT" = "gpu" ]; then
    DOCKER_ARGS+=(--gpus all)
fi

if [ "$ENABLE_ROS2" = "ON" ]; then
    DOCKER_ARGS+=(--network host)
fi

if [ -n "$DATA_DIR" ]; then
    if [[ "$DATA_DIR" == *:* ]]; then
        DATA_HOST_PATH="${DATA_DIR%%:*}"
        DATA_CONTAINER_PATH="${DATA_DIR#*:}"
    else
        DATA_HOST_PATH="$DATA_DIR"
        DATA_CONTAINER_PATH="$DATA_DIR"
    fi
    if [ ! -e "$DATA_HOST_PATH" ]; then
        echo "Error: --data host path '$DATA_HOST_PATH' does not exist." >&2
        exit 1
    fi
    # Resolve to an absolute path for the same reason as --dev-config below —
    # Docker's -v silently does the wrong thing with a relative host path.
    DATA_HOST_PATH="$(cd "$DATA_HOST_PATH" && pwd)"
    DOCKER_ARGS+=(-v "${DATA_HOST_PATH}:${DATA_CONTAINER_PATH}:ro")
fi

# Allow to modify config outside the container
DOCKER_ARGS+=(-v "$(cd ../config && pwd)/vision_pilot.conf:/usr/share/visionpilot/config/vision_pilot.conf:ro")
DOCKER_ARGS+=(-v "$(cd ../config && pwd)/vision_pilot_test.conf:/usr/share/visionpilot/config/vision_pilot_test.conf:ro")
if [ "$ENABLE_ROS2" = "ON" ]; then
    DOCKER_ARGS+=(-v "$(cd ../config && pwd)/vision_pilot_ros2.conf:/usr/share/visionpilot/config/vision_pilot_ros2.conf:ro")
fi

if [ -n "$V4L2" ]; then
  DOCKER_ARGS+=(--device "${V4L2}")
fi

if [ -z "$NO_DISPLAY" ]; then
    DOCKER_ARGS+=(-e "DISPLAY=${DISPLAY:-}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    DOCKER_ARGS+=(--device /dev/dri:/dev/dri)
    DOCKER_ARGS+=(-e "XDG_RUNTIME_DIR=/tmp/runtime-root")
    if [ -z "$NO_XHOST" ] && command -v xhost >/dev/null 2>&1; then
        echo "Granting local Docker containers X11 access (xhost +local:docker) —"
        echo "skip this with --no-xhost if you've already set it up, or --no-display"
        echo "if you don't need the visualization window at all."
        xhost +local:docker >/dev/null 2>&1 || true
    fi
fi

echo "=================================================="
echo " VisionPilot Docker run"
echo "=================================================="
echo " Variant:      $VARIANT"
echo " Image tag:    $TAG"
echo " Display:      $([ -n "$NO_DISPLAY" ] && echo "disabled" || echo "enabled")"
if [ -n "$DATA_DIR" ]; then
    if [ "$DATA_HOST_PATH" = "$DATA_CONTAINER_PATH" ]; then
        echo " Data mount:   $DATA_HOST_PATH (same path in container)"
    else
        echo " Data mount:   $DATA_HOST_PATH -> $DATA_CONTAINER_PATH"
    fi
fi
echo "=================================================="

docker run "${DOCKER_ARGS[@]}" "$TAG"

echo "${DOCKER_ARGS[@]}"

