#!/usr/bin/env bash
# Run the VisionPilot dev/build container.
#
# Mounts the repo at /workspace (edit on host, build inside) and shares the host
# X server so the OpenCV/visualizer windows appear on your display.
#
#   ./run.sh                Interactive shell (GPU image, --gpus all)
#   ./run.sh --no-cuda      Interactive shell (CPU image)
#   ./run.sh --test         Build VisionPilot + run the tests, then exit
#   ./run.sh -- <cmd...>    Run an arbitrary command in the container
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${DIR}/.." && pwd)"

IMAGE_BASE="autoware-visionpilot-dev"
USE_CUDA=true
RUN_TEST=false

usage() {
    cat <<EOF
Usage: $0 [options] [-- command...]

  --no-cuda     Use the CPU image (:cpu) and drop --gpus all.
                Default uses the GPU image (:cuda) with --gpus all.
  --test        Run build_and_test.sh (build + run tests) instead of a shell.
  -- command... Run the given command in the container instead of a shell.
  -h, --help    Show this help.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --no-cuda) USE_CUDA=false; shift ;;
        --test)    RUN_TEST=true;  shift ;;
        -h|--help) usage; exit 0 ;;
        --)        shift; break ;;
        *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

if ${USE_CUDA}; then
    IMAGE="${IMAGE_BASE}:cuda"
    GPU_ARGS=(--gpus all)
else
    IMAGE="${IMAGE_BASE}:cpu"
    GPU_ARGS=()
fi

# Share the host X server so cv::imshow / the visualizer render on the host display.
xhost +local:root >/dev/null 2>&1 || true
X11_ARGS=(
    --env "DISPLAY=${DISPLAY:-:0}"
    --env QT_X11_NO_MITSHM=1
    -v /tmp/.X11-unix:/tmp/.X11-unix
)

# Choose the command: --test, an explicit "-- cmd", or an interactive shell.
if ${RUN_TEST}; then
    CMD=(/usr/local/bin/build_and_test.sh)
elif [ $# -gt 0 ]; then
    CMD=("$@")
else
    CMD=(bash)
fi

echo "[run] image=${IMAGE}  workspace=${REPO_ROOT}"

# Interactive TTY only when attached to one (so --test / -- cmd work in CI too).
TTY_ARGS=(-i)
[ -t 0 ] && [ -t 1 ] && TTY_ARGS+=(-t)

docker run --rm "${TTY_ARGS[@]}" \
    --name visionpilot-dev \
    --net=host \
    "${GPU_ARGS[@]}" \
    "${X11_ARGS[@]}" \
    -v "${REPO_ROOT}:/workspace" \
    -w /workspace \
    "${IMAGE}" "${CMD[@]}"
