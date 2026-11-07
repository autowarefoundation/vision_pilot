# VisionPilot ⇄ CARLA (0.9.16) — ROS 2

In order to connect VisionPilot to CARLA for closed loop simulation and to not introduce carla_bridge dependency, two
nodes are required, one to publish vehicle speed and one to publish a command message so CARLA can drive the vehicle.

`carla_vehicle_speed_publisher` package publishes the vehicle speed to `/vehicle/speed` topic, and
`carla_control_publisher` publish control messages to `/carla/hero/vehicle_control_cmd`.

## PoV Highway map (optional)

Closed loop on the PoV Highway map with 30 Traffic Manager cars (stock CARLA 0.9.16).

1. Request the map package `20260701_PoV_Highway_0.9.16.tar.gz` from its owner (see
   `autowarefoundation/autoware.trucks#4`, "PoV_Highway map only"). It is not redistributable.
2. Install it on a copy of CARLA 0.9.16: put the package in `<CARLA>/Import` and run
   `./ImportAssets.sh` from the CARLA root.
3. Start CARLA with `./CarlaUE4.sh --ros2` and wait until it has fully loaded.
4. Launch the bridge with the PoV rig (the `carla` Python wheel of 0.9.16 must be importable):

```bash
ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway \
  rig_file:=$PWD/config/carla916_pov_highway.json
```

5. Once the camera topic publishes, start VisionPilot last, with `source.mode = ros2` and
   `source.input_camera_topic = /carla/hero/main_cam/image`.

The rig spawns the ego on road 2 / lane -2 / s=550 and the traffic from s=700, one car every
150 m per lane. The NPCs wait until the ego moves and then drive at the Traffic Manager default.
To make the traffic slower than the ego, add `ego_speed_mph` (default: VisionPilot's 33.3 m/s) and
`npc_speed_delta_mph` to the rig; the NPCs then drive `ego_speed_mph - npc_speed_delta_mph`. Set
VisionPilot's `speed_limit` to the m/s value the bridge logs at start-up.
