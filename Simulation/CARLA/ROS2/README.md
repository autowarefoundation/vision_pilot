# VisionPilot ⇄ CARLA (0.9.16 / 1.0) — ROS 2

In order to connect VisionPilot to CARLA for closed loop simulation and to not introduce carla_bridge dependency, two
nodes are required, one to publish vehicle speed and one to publish a command message so CARLA can drive the vehicle.

`carla_vehicle_speed_publisher` package publishes the vehicle speed to `/vehicle/speed` topic, and
`carla_control_publisher` publish control messages to `/carla/hero/vehicle_control_cmd`.

## CARLA 0.9.16 (UE4)

Spawn the ego with `config_carla.py -f config/carla916.json`; use the 0.9.16 client wheel.

## CARLA 1.0 (UE5)

CARLA 1.0 needs the AdditionalMaps content pack for `Town04_Opt`. See
[Get the Town04_Opt map](../../../github-io/simulation.md#get-the-town04_opt-map).

Server (Low quality, off-screen to save GPU memory; `--ros2` is required for the camera topic):

```sh
docker run -d --name carla-server --runtime=nvidia --net=host --user=$(id -u):$(id -g) \
  --env=DISPLAY=$DISPLAY --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --env=FASTDDS_BUILTIN_TRANSPORTS=UDPv4 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:1.0-maps bash CarlaUnreal.sh -nosound -quality-level=Low -RenderOffScreen --ros2
```

Ego (CARLA 1.0 client matching the server; a mismatch can segfault):

```bash
python src/carla_bridge_bringup/scripts/config_carla_1_0.py -f config/carla1_0.json
```

- Map `Town04_Opt` (`Town04` is not found); ego `vehicle.lincoln.mkz`.
- Sensors publish natively (`ros_name`); `ros2_ackermann_control=true` makes the ego subscribe
  `/carla/hero/ackermann_control_cmd` (`AckermannDriveStamped`).
- Bridge nodes are unchanged but need the CARLA 1.0 cp312 wheel on `PYTHONPATH` and
  `ros-jazzy-ackermann-msgs`.
- Start order: server, spawn script, bridge, VisionPilot. Stop clients with SIGINT, never
  `kill -9`; restart the server after a hard kill.

Details: [github-io/simulation.md](../../../github-io/simulation.md#carla-10-ue5).
