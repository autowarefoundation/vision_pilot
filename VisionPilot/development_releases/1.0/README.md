# Vision Pilot 1.0 - L2 ADAS System

## Homography

| What | Where | Role |
|------|-------|------|
| **Ground H** | `-DVISIONPILOT_GROUND_HOMOGRAPHY=` at cmake | Your camera, full-frame → world; builds preprocess **C** |
| **Model V** | Hardcoded in `scripts/find_homography_C_matrix.py` | VisionPilot 1024×512 → world (**do not modify**) |
| **Fusion** | `tracker.homography_path` in conf | YAML with the **same** model-view **V** (1024×512 bbox → world) |

Only the **ground** homography is per dataset / camera. **V** is fixed for all VisionPilot builds.

```bash
cd VisionPilot/development_releases/1.0
cmake -B build -DVISIONPILOT_GROUND_HOMOGRAPHY=/path/to/your_full_frame_homography.yaml
cmake --build build --target VisionPilot -j$(nproc)
```

Regenerate **C** only:

```bash
python3 scripts/find_homography_C_matrix.py --ground-h /path/to/your_full_frame_homography.yaml
```

OpenLane example ground file: `../../middleware_recipes/Standalone/AutoSpeed/homography.yaml`

## Build (video / v4l2 — no ROS2 required)

```bash
cmake -B build -DVISIONPILOT_GROUND_HOMOGRAPHY=... 
cmake --build build --target VisionPilot -j$(nproc)
```

## Build (with ROS2 camera input)

```bash
cmake -B build -DENABLE_ROS2_INTERFACE=ON -DVISIONPILOT_GROUND_HOMOGRAPHY=...
cmake --build build --target VisionPilot -j$(nproc)
```

## Run

Edit `config/vision_pilot.conf` — set `tracker.homography_path` to your model-view YAML (must match VP **V** in the script). Then:

```bash
./run_vision_pilot.sh
```

Or `./build/VisionPilot` from this directory.
