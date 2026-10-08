# vision_pilot (Python bindings)

Python bindings for one iteration of the VisionPilot loop:

```text
camera frame ─► ImagePreprocessor ─► AutoDrive / AutoSteer / AutoSpeed ─► lateral + longitudinal fusion ─► Planner ─► Command
```

The C++ app and the bindings run the same code: the loop body lives in
`modules/pilot` (`visionpilot::pilot::Pilot`), which `app/vision_pilot.cpp` calls
once per frame and `vision_pilot.Pilot` wraps. A closed-loop simulation therefore
drives the code that ships, not a reimplementation of it.

## How it is built

- [nanobind](https://github.com/wjakob/nanobind) for the bindings: smaller and
  faster to build than pybind11, and it produces `abi3` (stable ABI) extensions,
  so one wheel serves CPython 3.12 and every later version. Older interpreters
  (3.10, 3.11) get a per-version wheel.
- [scikit-build-core](https://github.com/scikit-build/scikit-build-core) as the
  PEP 517 backend, driving the project's own CMake: `pip install` / `uv sync`
  builds the extension; there is no `setup.py`.
- Typed stubs (`_core.pyi`) are generated from the bindings when the wheel is
  built, so editors and mypy see every signature and docstring.
- Images cross the boundary without copies (NumPy buffer ⇄ `cv::Mat`), and the
  GIL is released while a frame is processed.
- The ONNX Runtime shared library is placed next to the extension (`$ORIGIN`
  RPATH): no `LD_LIBRARY_PATH`, no system-wide install. When `ONNXRUNTIME_ROOT`
  is unset, the pinned CPU release (1.26.0, SHA-256 checked) is downloaded.

## Install

System libraries (Ubuntu 24.04), the same ones the app needs minus the I/O and
display stack:

```bash
sudo apt-get install build-essential cmake libopencv-dev coinor-libipopt-dev libcppad-dev liblapack-dev libblas-dev
# CppAD includes <coin-or/...>; Ubuntu's Ipopt installs to /usr/include/coin
sudo ln -sf "$(dirname "$(find /usr/include -name IpIpoptApplication.hpp | head -1)")" /usr/include/coin-or
```

Then, from `VisionPilot/`:

```bash
uv sync                 # builds the extension into .venv (editable)
uv run pytest           # Python test suite
```

or install into another project's environment:

```bash
uv pip install ./VisionPilot
# GPU: point at a CUDA / TensorRT build of ONNX Runtime instead of the CPU download
uv pip install ./VisionPilot -C cmake.define.ONNXRUNTIME_ROOT=/path/to/onnxruntime-gpu
```

The model weights are not part of the wheel; pass their directory as
`InferenceConfig(model_dir=...)` (`modules/models/weights` in this repository).

## Usage

```python
import numpy as np
import vision_pilot as vp

# Ground homography of the simulated camera: pixel -> (x forward, y left) [m].
H = vp.ground_homography(
    width=1280, height=720, horizontal_fov_deg=52.0,
    camera_height_m=1.5, pitch_down_deg=2.0, x_m=1.0,
)

config = vp.PilotConfig(
    engine=vp.EngineConfig(provider="cpu"),          # "cuda" | "tensorrt"
    inference=vp.InferenceConfig(model_dir="modules/models/weights", seed=0),
    speed_limit_mps=16.7,
    front_axle_to_cog_m=1.4,
    mpc_max_cpu_time_s=1.0,                           # converge every solve: reproducible runs
)
pilot = vp.Pilot(config, H)

for frame in episode:                                 # (H, W, 3) uint8
    result = pilot.step(frame.rgb, frame.speed_mps, channel_order=vp.ChannelOrder.RGB, dt_s=0.1)
    if not result.ready:                              # first frame primes the 2-frame buffer
        continue
    apply(result.command.steering_tyre_rad, result.command.acceleration_mps2)
    log(result.perception.lateral.cte_m, result.perception.cipo.distance_m, result.plan.warnings)

pilot.reset()                                         # before the next episode
```

### Things that matter in closed loop

|                              |                                                                                                                                                                                                                                                                                                                                |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `channel_order`              | Required. CARLA and PIL images are RGB, OpenCV images BGR. The input array is never modified.                                                                                                                                                                                                                                  |
| `dt_s`                       | Simulation time since the previous frame. The fusion filters otherwise assume their nominal 10 Hz period, which is wrong for a simulator running faster or slower than real time.                                                                                                                                              |
| `seed`, `mpc_max_cpu_time_s` | `seed` seeds both particle filters (`None` keeps the app's nondeterministic seeding). The lateral MPC stops at a 15 ms CPU budget on the vehicle, so by default its plan depends on how fast the machine is; set `mpc_max_cpu_time_s=1.0` (or more) to let every solve converge. With both, a run is reproducible bit for bit. |
| `reset()`                    | Clears the frame buffer, the filters and the planner's curvature history between episodes.                                                                                                                                                                                                                                     |
| `ground_homography`          | Must describe the simulated camera. `config/H.yaml` is calibrated for the OpenLane camera and does not fit a CARLA sensor. `ground_homography()` documents the mapping from a CARLA `Transform` (left-handed, pitch up).                                                                                                       |
| Output images                | `result.warped` / `result.resized` are read-only NumPy views of the network inputs, valid for as long as you hold them.                                                                                                                                                                                                        |
| Threads                      | `step()` releases the GIL; calls on one `Pilot` are serialised internally.                                                                                                                                                                                                                                                     |

`Planner` is bound on its own as well, for testing the IDM + MPC planner without
the networks.

## Layout

```text
VisionPilot/
├── pyproject.toml              # scikit-build-core configuration
├── cmake/OnnxRuntime.cmake     # ONNXRUNTIME_ROOT or the pinned download
├── modules/pilot/              # the loop step shared by the app and the bindings
└── python/
    ├── CMakeLists.txt          # nanobind extension, stubs, bundled ONNX Runtime
    ├── bindings.cpp            # vision_pilot._core
    ├── src/vision_pilot/       # Python package (camera geometry, re-exports)
    └── tests/
```
