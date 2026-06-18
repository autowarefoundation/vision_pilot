# Vision Pilot 1.0 - L2 ADAS System

Single front-camera L2 ADAS pipeline: preprocess → inference (AutoDrive / AutoSteer / AutoSpeed) →
fusion → optional planning + control → display / actuation.

## Dependencies

A C++17 toolchain + CMake ≥ 3.22 and: OpenCV, IPOPT, CppAD, Eigen, yaml-cpp, Boost, plus the ONNX
Runtime SDK. Model weights ship in `modules/models/weights/` and the ground homography in
`config/H.yaml` — there are no per-build paths/flags to set.

conda-forge (no sudo):

```bash
conda create -n vp -c conda-forge python=3.10 opencv ipopt cppad eigen yaml-cpp boost \
    pkg-config cmake make cxx-compiler
conda activate vp
export CPATH="$CONDA_PREFIX/include"   # CppAD / Boost headers are otherwise invisible to CMake
```

Debian / Ubuntu (apt):

```bash
sudo apt-get install -y build-essential cmake pkg-config libopencv-dev libyaml-cpp-dev \
    libeigen3-dev coinor-libipopt-dev libcppad-dev liblapack-dev libblas-dev libmumps-seq-dev \
    gfortran python3 python3-numpy python3-opencv
```

ONNX Runtime SDK — the CPU provider is enough to build and run the engine (TensorRT/CUDA providers
are a runtime-only concern):

```bash
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz     # set ONNXRUNTIME_ROOT to the extracted dir
```

The `generate_config` CMake target runs `scripts/find_homography_C_matrix.py`, which needs a
`python3` with `numpy` + `cv2`.

## Build (host)

```bash
cd VisionPilot
cmake -B build -DONNXRUNTIME_ROOT=/path/to/onnxruntime
cmake --build build --target VisionPilot -j$(nproc)
```

- With a conda env, also pass `-DCMAKE_PREFIX_PATH=$CONDA_PREFIX`.
- For ROS2 camera input add `-DENABLE_ROS2_INTERFACE=ON`.
- Build in the in-tree `build/` directory: the homography codegen reads `config/H.yaml` and the app
  reads the generated matrix relative to it.

> Prefer not to install the toolchain on your host? See [`Docker/README.md`](Docker/README.md) for
> an optional self-contained image. It is a convenience only — the host build above is fully
> supported on its own.

## Homography

Two fixed matrices, both in-tree — nothing per-build to configure:

- **Ground H** — `config/H.yaml`; the codegen builds the preprocess matrix **C** from it.
- **Model V** — hardcoded in `scripts/find_homography_C_matrix.py` (VisionPilot 1024×512 → world;
  **do not modify**).

## Run

```bash
cp config/vision_pilot.conf.example config/vision_pilot.conf   # then edit as needed
./build/VisionPilot -c config/vision_pilot.conf
```

## Control (optional)

The flag-gated perception → planner → control drive loop (`control.enabled`, **off by default**)
turns the planner's intent into a `ControlCommand` `{steering_angle_rad, speed_mps,
acceleration_mps2}`. See [`modules/control`](modules/control/README.md) — it is
middleware-/vehicle-agnostic and owns no transport.
