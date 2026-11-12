# VisionPilot ⇄ CARLA (0.9.16) — ROS 2

In order to connect VisionPilot to CARLA for closed loop simulation and to not introduce carla_bridge dependency, two
nodes are required, one to publish vehicle speed and one to publish a command message so CARLA can drive the vehicle.

`carla_vehicle_speed_publisher` package publishes the vehicle speed to `/vehicle/speed` topic, and
`carla_control_publisher` publishes `autoware_control_msgs/msg/Control` messages to `/control/command/control_cmd`.

## Running against a world owned by `autoware_carla_interface` (autoware_carla_tum)

When the CARLA map / ego / sensors are spawned by `autoware_carla_interface` from the
`autoware_carla_tum` workspace, **do not run `config_carla.py`** (it would load its own
map, spawn a second `hero` and fight the interface for the sim clock). Use
`carla_bridge_autoware_interface.launch.py` instead.

Data flow:

```
autoware_carla_interface ──/sensing/camera/CAM_FRONT/image_raw──▶ VisionPilot
                         ──/vehicle/status/velocity_status──▶ velocity_status_bridge ──/vehicle/speed──▶ VisionPilot
                                                                                   └─/localization/kinematic_state─┐
VisionPilot ──/vehicle/steering_cmd,/vehicle/throttle_cmd──▶ carla_control_publisher                             │
      ──/control/command/control_cmd──▶ autoware_raw_vehicle_cmd_converter (started by the interface launch) ◀────┘
      ──/control/command/actuation_cmd──▶ autoware_carla_interface ──▶ CARLA ego
```

`autoware_raw_vehicle_cmd_converter` does not emit anything until it has received an
odometry message, so `velocity_status_bridge` publishes a twist-only `nav_msgs/Odometry`
on `/localization/kinematic_state` (disable with `publish_odometry:=false` if something
else already provides it).

### 1. Start CARLA and the interface (autoware_carla_tum workspace)

```bash
source ~/autoware_carla_tum/install/setup.bash
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml \
  map_path:=<autoware map dir whose basename is the CARLA town, e.g. .../Town04> \
  sensor_mapping_file:=$(pwd)/config/sensor_mapping_vision_pilot.yaml \
  use_traffic_manager:=True
```

`config/sensor_mapping_vision_pilot.yaml` sets `CAM_FRONT` to the VisionPilot rig
(1920x1280, fov 50, 10 Hz) and drops the LiDAR. The camera pose comes from
`carla_sensor_kit_description`, not from `carla916.json`; if `H.yaml` was calibrated for
the `main_cam` pose, re-check it (or edit the sensor kit calibration). Optional: follow the
ego with `ros2 run autoware_carla_interface spectator_follow`.

### 2. Start this bridge (this workspace, with the autoware workspace sourced)

```bash
source ~/autoware_carla_tum/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch carla_bridge_bringup carla_bridge_autoware_interface.launch.py
```

### 3. Start VisionPilot

`VisionPilot/config/vision_pilot_ros2.conf` already points `source.input_camera_topic` at
`/sensing/camera/CAM_FRONT/image_raw`.
