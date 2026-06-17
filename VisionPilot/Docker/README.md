# Docker

Containerized builds of the VisionPilot single-binary app (`app/` + `modules/`). All C++
dependencies (ipopt/cppad/eigen/opencv/yaml-cpp + the ONNX Runtime SDK) live **inside** the image —
the host needs only Docker, no conda, no ROS2 install.

Two images:

| Image | Dockerfile | Builds with | For |
|-------|-----------|-------------|-----|
| `visionpilot-app` | `Dockerfile` | Ubuntu 24.04 + apt | the canonical no-ROS2 build (video / v4l2 input) |
| `visionpilot-ros2` | `Dockerfile.ros2` | `ros:jazzy-ros-base` + apt | `ENABLE_ROS2_INTERFACE=ON` — Ackermann control out, odometry/camera in |

Both bake the ONNX Runtime SDK at `/opt/onnxruntime` (CPU provider is enough to build and run the
engine), bridge the legacy `/usr/include/coin` → `coin-or/` IPOPT path, and build into the in-tree
`build/` with the sample ground homography `config/H.yaml`.

## ROS2 image — `build.sh` / `run.sh` (use these, don't call docker by hand)

```bash
cd VisionPilot/Docker
./build.sh                 # builds visionpilot-ros2 (x64, ONNX Runtime 1.22.0)
./run.sh smoke             # in-process ROS2 round-trip tests (no weights needed)
```

`build.sh` — build the self-contained ROS2 image. Build context is the repo root (so the Dockerfile
can `COPY VisionPilot`). Env overrides: `IMAGE`, `ARCH` (`x64`|`aarch64`), `ONNXRUNTIME_VERSION`.

`run.sh <mode>` — run it with the ROS2/DDS init right (`--net=host` for DDS discovery to host `ros2`
tools / other containers; `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` because host↔container Fast-DDS shared
memory is unreliable; `QT_QPA_PLATFORM=offscreen`). ROS2 is sourced for you.

| Mode | What it does | Needs weights? |
|------|--------------|----------------|
| `smoke` (default) | runs `test_control_cmd_publisher` + `test_vehicle_state_subscriber` (publish→subscribe round-trips) | no |
| `echo` | `ros2 topic echo /control/ackermann_cmd` | no |
| `run` | launches `./build/VisionPilot --config config/vision_pilot.conf` | **yes** |
| `bash` | interactive shell, ROS2 sourced | — |

Env overrides for `run.sh`: `IMAGE`, `ROS_DOMAIN_ID`, `CONTROL_TOPIC`, `CONFIG` (host conf mounted
read-only), `WEIGHTS_DIR` (host model-weights dir mounted at `/autoware/VisionPilot/models`). A full
app run needs the AutoDrive/AutoSteer/AutoSpeed `.onnx` weights (not in git — see the model READMEs);
the `smoke`/`echo` modes prove the ROS2 plumbing without them.

```bash
# Full app against a host config + weights:
CONFIG=/path/vision_pilot.conf WEIGHTS_DIR=/path/models ./run.sh run
```

## No-ROS2 image — `Dockerfile`

No wrapper script; build/run directly (build context = repo root):

```bash
docker build -f VisionPilot/Docker/Dockerfile -t visionpilot-app .
docker run --rm visionpilot-app \
    bash -lc 'cd /autoware/VisionPilot && QT_QPA_PLATFORM=offscreen ./build/VisionPilot --help || true'
```
