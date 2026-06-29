# Vision Pilot 1.0 - L2 ADAS System

## Homography

| What | Where | Role |
|------|-------|------|
| **Ground H** | `-DVISIONPILOT_GROUND_HOMOGRAPHY=` at cmake | Your camera, full-frame → world; builds preprocess **C** |
| **Model V** | Hardcoded in `scripts/find_homography_C_matrix.py` | VisionPilot 1024×512 → world (**do not modify**) |
| **Fusion** | `tracker.homography_path` in conf | YAML with the **same** model-view **V** (1024×512 bbox → world) |

Only the **ground** homography is per dataset / camera. **V** is fixed for all VisionPilot builds.

```bash
cd VisionPilot
cmake -B build -DONNXRUNTIME_ROOT=/path/to/onnxruntime -DVISIONPILOT_GROUND_HOMOGRAPHY=/path/to/H.yaml
cmake --build build --target VisionPilot -j$(nproc)
```

Regenerate **C** only:

```bash
python3 scripts/find_homography_C_matrix.py --ground-h /path/to/H.yaml
```

## Prerequisites: Eigen

```bash
mkdir -p modules/safety_guardian/planning/src

wget https://gitlab.com/libeigen/eigen/-/archive/5.0.0/eigen-5.0.0.tar.gz
tar -xzf eigen-5.0.0.tar.gz -C modules/safety_guardian/planning/src/
rm eigen-5.0.0.tar.gz
```

Notes:
- Ensure extracted folder is named exactly `eigen-5.0.0` at `./modules/safety_guardian/planning/src/eigen-5.0.0`.
- **Do not commit this folder to Git**.


## Build (video / v4l2 — no ROS2 required)

```bash
cmake -B build -DONNXRUNTIME_ROOT=... -DVISIONPILOT_GROUND_HOMOGRAPHY=... 
cmake --build build --target VisionPilot -j$(nproc)
```

## Build (with ROS2 camera input)

```bash
cmake -B build -DENABLE_ROS2_INTERFACE=ON -DONNXRUNTIME_ROOT=... -DVISIONPILOT_GROUND_HOMOGRAPHY=...
cmake --build build --target VisionPilot -j$(nproc)
```

## Run

Edit `config/vision_pilot.conf`
```bash
./VisionPilot -c /path/to/config/vision_pilot.conf
```
