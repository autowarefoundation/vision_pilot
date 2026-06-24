# VisionPilot Docker (dev / build container)

A self-contained dev container to **compile and run the VisionPilot tests** (and the
visualizer) without installing the C++ toolchain, ROS2, or ONNX Runtime on the host.

Unlike a baked release image, the **repo is bind-mounted** at `/workspace` — you edit on
the host and build inside — and the **host X server is shared** so OpenCV / visualizer
windows appear on your display.

## Requirements

- Docker.
- GPU image only: NVIDIA driver + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (`--gpus all`).

## Build the image

```bash
./build.sh            # GPU image  -> autoware-visionpilot-dev:cuda   (nvidia/cuda base, ORT GPU)
./build.sh --no-cuda  # CPU image  -> autoware-visionpilot-dev:cpu    (ubuntu base, ORT CPU)
```

The CPU image also targets **arm64 / Apple Silicon** (picks the aarch64 ONNX Runtime).
The GPU image is x86_64-only. Add `--no-cache` to rebuild from scratch.

## Compile + run the tests

```bash
./run.sh --test            # GPU image
./run.sh --no-cuda --test  # CPU image
```

Builds into `VisionPilot/build_docker` (kept separate from any host `build/`), then runs
`test_planning`, `image_preprocessing_tests`, and `test_vision_pilot`. ROS2 is **off** by
default (the video/v4l2 build); enable the ROS2 interface with
`ENABLE_ROS2_INTERFACE=ON`:

```bash
./run.sh --no-cuda -- env ENABLE_ROS2_INTERFACE=ON /usr/local/bin/build_and_test.sh
```

## Interactive shell

```bash
./run.sh             # GPU image, drops you in /workspace with ROS2 sourced
./run.sh --no-cuda   # CPU image
```

Inside, build manually if you prefer:

```bash
cd VisionPilot
cmake -B build_docker -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT"   # add -DENABLE_ROS2_INTERFACE=ON for ROS2
cmake --build build_docker -j"$(nproc)"
./build_docker/VisionPilot -c config/vision_pilot.conf   # window shows on your host display
```

Run an arbitrary command directly:

```bash
./run.sh -- ./VisionPilot/build/test_planning
```

## What's in the image

Ubuntu 24.04 base (`nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04` for GPU), ROS2 **Jazzy**
(`ros-base` + `cv_bridge` + `image_transport`), ONNX Runtime **1.23.2** SDK at
`/opt/onnxruntime` (`$ONNXRUNTIME_ROOT`), OpenCV, Ipopt + CppAD, yaml-cpp, Eigen, Boost,
and `python3` + `cv2`/`numpy` for the `generate_config` homography step.

## Notes

- `run.sh` runs `xhost +local:root` so the container (root) can reach your X server. If
  windows don't appear, run `xhost +local:` on the host once, and confirm `DISPLAY` is set.
- Model `.onnx` weights are **not** included — fetch them per the model READMEs and mount
  the directory (e.g. `./run.sh -- ...` with an extra `-v`) for a full pipeline run.
- The container runs as root; files it creates under `build/` are root-owned on the host.
