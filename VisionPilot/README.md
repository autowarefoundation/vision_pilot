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

## Docker (no host deps)

Containerized builds that carry every dependency internally — host needs only Docker. See
[`Docker/README.md`](Docker/README.md): `visionpilot-app` (no-ROS2) and the self-contained
`visionpilot-ros2` image driven by `Docker/build.sh` + `Docker/run.sh`.

## Control & ROS2 interface

The optional perception → planner → control drive loop (off by default, `control.enabled`) turns the
Planner's intent into a `ControlCommand` `{steering_angle_rad, speed_mps, acceleration_mps2}`. It is
middleware-/vehicle-agnostic:

- [`modules/control`](modules/control/README.md) — controllers, `ControlCommand`, the `IDbwSink`/
  `DbwAdapter` DBW seam, and the deterministic test suite (incl. the closed-loop SIL harness).
- [`modules/middleware_interfaces/ros2_interface/control_cmd_publisher`](modules/middleware_interfaces/ros2_interface/control_cmd_publisher/README.md)
  — publishes the command as `ackermann_msgs/AckermannDriveStamped` (`-DENABLE_ROS2_INTERFACE=ON`).
- [`.../vehicle_state_subscriber`](modules/middleware_interfaces/ros2_interface/vehicle_state_subscriber/README.md)
  — subscribes odometry for live ego speed.

## Tests

Plain executables, no weights/sim required — build the target and run the binary (each prints
`PASS`/`FAIL`, non-zero exit on failure): `test_control`, `test_dbw_adapter`, `test_closed_loop`,
`test_planning`, and (with ROS2) `test_control_cmd_publisher`, `test_vehicle_state_subscriber`.
