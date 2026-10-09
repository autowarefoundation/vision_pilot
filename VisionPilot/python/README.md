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
- Every native dependency comes from uv, not apt (see `cmake/PythonDeps.cmake`):

  | Dependency                            | Source                                                                            |
  | ------------------------------------- | --------------------------------------------------------------------------------- |
  | ONNX Runtime 1.26.0                   | Official release (SHA-256 pinned), shipped inside the wheel next to the extension |
  | Ipopt 3.14 (+ MUMPS, METIS, OpenBLAS) | The [`casadi`](https://pypi.org/project/casadi/) wheel                            |
  | CppAD                                 | The [`cmeel-cppad`](https://pypi.org/project/cmeel-cppad/) wheel                  |
  | OpenCV 4.12 (core, imgproc)           | Built from source and linked statically: no wheel ships OpenCV's headers          |
  | Eigen 5.0                             | Fetched (header-only), as for the app                                             |
  | CMake, Ninja                          | PyPI, when the system has none or an older one                                    |

  casadi and cmeel-cppad are both build requirements and runtime dependencies
  at the same pinned versions, and the extension finds their libraries through
  RPATH entries relative to `site-packages`, so nothing needs `LD_LIBRARY_PATH`.

## Install

Only [uv](https://docs.astral.sh/uv/), a C/C++ compiler and git are needed
(`build-essential` and `git` on Ubuntu). From `VisionPilot/`:

```bash
uv sync                 # creates .venv and builds the extension into it
uv run pytest           # Python test suite
```

The first build compiles OpenCV and takes a few minutes; it is kept in
`build/<wheel tag>/`, so later builds only recompile what changed (C++ edits
trigger a rebuild on the next `uv sync` / `uv run`).

To install it into another project's environment:

```bash
uv pip install ./VisionPilot
# or, from that project's pyproject.toml:
#   [tool.uv.sources]
#   vision-pilot = { path = "../vision_pilot/VisionPilot" }
```

Build options go through `-C cmake.define.<NAME>=<value>`:

| Option                                      | Effect                                                                           |
| ------------------------------------------- | -------------------------------------------------------------------------------- |
| `ONNXRUNTIME_ROOT=/path/to/onnxruntime-gpu` | Link a CUDA / TensorRT ONNX Runtime build instead of downloading the CPU release |
| `VISIONPILOT_VENDOR_OPENCV=OFF`             | Use an installed OpenCV instead of building one                                  |

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

## Closed loop: scenarios in CARLA

`vision_pilot.driver.VisionPilotDriver` is a policy for alpasim's `EgodriverService`,
built on [`carla-driver-interface`](https://pypi.org/project/carla-driver-interface/)
2.x, the policy side of the contract
[autoware_carla_scenario](https://github.com/autowarefoundation/autoware_carla_scenario)
speaks. The `closed-loop-test` extra brings it:

```bash
uv sync --extra closed-loop-test
uv run vision-pilot-driver --model-dir modules/models/weights --port 50051 --seed 0
```

The CARLA side is autoware_carla_scenario, run from its own checkout and environment.
Its `driver=vision_pilot` preset hands the ego to the policy with the vehicle and camera
VisionPilot is tuned for: the Lincoln MKZ and 1920×1280, 50° front camera at 10 Hz of
VisionPilot's own CARLA rig (`Simulation/CARLA/ROS2/config/carla916.json`):

```bash
uv run scenario driver=vision_pilot                 # in autoware_carla_scenario
```

A runtime whose camera is far from ~52° (alpasim's default 120° camera, say) still
works, but the driver logs a warning.

Without a simulator, `carla_driver_interface.testing.FakeLoop` drives the same
server over real gRPC on a straight road, rendering the declared camera; the tests use
it (`python/tests/test_driver.py`), and so can a quick check from the command line:

```bash
uv run carla-driver-interface demo --driver localhost:50051 \
    --camera-width 1920 --camera-height 1280 --camera-fov 50 --steps 20
```

What happens on each `drive` call:

1. **Camera → ground homography** (once per session). The runtime declares its
   cameras in `start_session`; the driver takes the configured one
   (`--camera`, or the only one), and builds VisionPilot's `H` from its pinhole
   intrinsics and pose in the rig, with the origin on the road below the
   camera, as for the calibrated `config/H.yaml`. Distorted or fisheye cameras
   are rejected.
2. **Frame → command.** A new frame goes through `Pilot.step`, with the
   simulation time between frames as `dt_s`. The first frame of a session only
   primes the two-frame buffer; until then the command is "hold" (no steering,
   no acceleration).
3. **Command → trajectory.** The command (front tyre angle δ, acceleration a)
   is held over the horizon on a kinematic bicycle model
   (`vision_pilot.kinematics.rollout_command`): an arc of curvature
   `tan(δ) / front_axle_to_cog_m` (the planner's own model) with a
   constant-acceleration speed profile that stops at zero rather than
   reversing. This is what VisionPilot would do to the car until its next
   cycle, and a trajectory follower (pure pursuit, in autoware_carla_scenario)
   reproduces that curvature on whatever vehicle it drives.

Each plan carries VisionPilot's state in `debug_scalars` (CTE, heading error,
curvature, lead vehicle distance/speed, FCW/AEB/LDW flags, inference time),
which `CarlaDriveDebugInfo` forwards to the runtime.

`--mpc-max-cpu-time` defaults to 1 s in the driver so that results do not
depend on the machine; pass 0.015 to reproduce the vehicle's real-time cut-off.

## Layout

```text
VisionPilot/
├── pyproject.toml              # scikit-build-core configuration
├── cmake/OnnxRuntime.cmake     # ONNXRUNTIME_ROOT or the pinned download
├── modules/pilot/              # the loop step shared by the app and the bindings
└── python/
    ├── CMakeLists.txt          # nanobind extension, stubs, bundled ONNX Runtime
    ├── bindings.cpp            # vision_pilot._core
    ├── src/vision_pilot/       # Python package: re-exports, camera geometry,
    │                           #   kinematics (command → trajectory), driver (alpasim policy)
    └── tests/
```
