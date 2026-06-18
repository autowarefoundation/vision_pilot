# Docker

**Optional** containerized build of the VisionPilot single-binary app (`app/` + `modules/`) — a
fast-setup helper for anyone who doesn't want to install the C++ toolchain on their host. All
dependencies (ipopt/cppad/eigen/opencv/yaml-cpp + the ONNX Runtime SDK) live **inside** the image,
so the host needs only Docker. The host build (see `../README.md`) is fully supported on its own;
this image is not required for it.

| Image | Dockerfile | Builds with | For |
|-------|-----------|-------------|-----|
| `visionpilot-app` | `Dockerfile` | Ubuntu 24.04 + apt | optional no-ROS2 build (video / v4l2 input) |

The image bakes the ONNX Runtime SDK at `/opt/onnxruntime` (the CPU provider is enough to build and
run the engine), bridges the legacy `/usr/include/coin` → `coin-or/` IPOPT path, and builds into the
in-tree `build/` with the sample ground homography `config/H.yaml`.

## No-ROS2 image — `Dockerfile`

Build/run directly (build context = repo root). **BuildKit is required**: the build is scoped by
`Dockerfile.dockerignore`, which only BuildKit honors. With the legacy builder the repo-root
`.dockerignore` (an allowlist) excludes `VisionPilot/app` and `VisionPilot/modules`, so
`COPY VisionPilot` lands an empty tree and the build fails. BuildKit is the default on modern Docker;
set `DOCKER_BUILDKIT=1` explicitly to be safe on hosts where it was turned off.

```bash
DOCKER_BUILDKIT=1 docker build -f VisionPilot/Docker/Dockerfile -t visionpilot-app .
docker run --rm visionpilot-app \
    bash -lc 'cd /autoware/VisionPilot && QT_QPA_PLATFORM=offscreen ./build/VisionPilot --help || true'
```
