# visionpilot_carla_bridge

Bridges VisionPilot's middleware-agnostic **Float64** actuation contract to CARLA's native
`--ros2` interface so the camera-only VisionPilot loop can drive the CARLA ego. Works
identically against **CARLA 0.9.16 (UE4) and 0.10 (UE5)** — same native ROS 2 topic names.

## Contract

| Topic                             | Type                                | Dir | Meaning                                                                                                                  |
| --------------------------------- | ----------------------------------- | --- | ------------------------------------------------------------------------------------------------------------------------ |
| `/vehicle/steering_cmd`           | `std_msgs/Float64`                  | in  | VisionPilot tyre angle (rad)                                                                                             |
| `/vehicle/throttle_cmd`           | `std_msgs/Float64`                  | in  | VisionPilot longitudinal accel (m/s²)                                                                                    |
| `/vehicle/speed`                  | `std_msgs/Float64`                  | in  | ego speed (m/s), published by the package's own `ego_telemetry` node — the **same** topic VisionPilot's planner consumes |
| `/carla_bridge/max_steer_angle`   | `std_msgs/Float64` (latched)        | in  | spawned ego's real front-wheel limit (rad); overrides the `max_steer` param                                              |
| `/carla/hero/vehicle_control_cmd` | `carla_msgs/CarlaEgoVehicleControl` | out | normalized steer/throttle/brake at `control_rate_hz`                                                                     |

CARLA has **no `ros2_ackermann_control`**, so the ego only accepts
`carla_msgs/CarlaEgoVehicleControl`, and publishes the camera natively at
`/carla/hero/main_cam/image`.

## Node

**`carla_control_relay`** — caches the latest inputs and publishes a control command at a
fixed rate. The per-tick decision is pure + unit-tested in
`control_mapping.decide_control`:

- **steering** — direct proportional: `steer = clamp(steer_gain · steer_rad / max_steer, ±1)`,
  normalized by the ego's real wheel limit (latched topic; `max_steer` param is the fallback).
- **longitudinal** — stateless **feed-forward** accel→pedal map (`throttle_ref_accel`,
  `brake_ref_accel`, `accel_deadband`). **No PID, no target speed** — VisionPilot closes the
  speed loop; the bridge only translates its accel command.
- **AEB** — accel below `aeb_accel_threshold` forces full brake (steering stays active).
- **watchdog** — brake-and-hold until fresh VP commands _and_ fresh speed arrive
  (`vp_cmd_timeout`, `speed_timeout`), so the car never drives during VisionPilot start-up
  and stops if either input stalls — a dead speed feed can never cause runaway throttle.

Params (all launch-exposed): `steer_gain`, `max_steer`, `control_rate_hz`,
`throttle_ref_accel`, `brake_ref_accel`, `accel_deadband`, `aeb_accel_threshold`,
`vp_cmd_timeout`, `speed_timeout`, `speed_topic`, `control_topic`.

## Build

Self-contained colcon package — builds identically on a host with ROS 2 Jazzy or in the
VisionPilot dev container. From the workspace root (`Simulation/CARLA/ROS2`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select carla_msgs visionpilot_carla_bridge   # or: ./build_bridge.sh
source install/setup.bash
```

## Test

```bash
cd src/visionpilot_carla_bridge && PYTHONPATH=$PWD python3 -m pytest test/ -v
```

## Run

```bash
ros2 launch visionpilot_carla_bridge carla_bridge.launch.py steer_gain:=1.0
# or directly:
ros2 run visionpilot_carla_bridge carla_control_relay
```

See `Simulation/CARLA/ROS2/README.md` for the full CARLA bring-up (both versions; `drive.sh`).
