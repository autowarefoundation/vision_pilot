# VisionPilot ⇄ CARLA (0.9.16 / 0.10) — ROS 2

In order to connect VisionPilot to CARLA for closed loop simulation and to not introduce carla_bridge dependency, two
nodes are required, one to publish vehicle speed and one to publish a command message so CARLA can drive the vehicle.

`carla_vehicle_speed_publisher` package publishes the vehicle speed to `/vehicle/speed` topic, and
`carla_control_publisher` publishes `ackermann_msgs/AckermannDriveStamped` to
`/carla/hero/ackermann_control_cmd`.

## CARLA 0.10

CARLA 0.10 publishes its sensors over ROS 2 natively, so the server must be built with `-DENABLE_ROS2=ON` and started
with `-ros2 -rmw=fastdds`. The rig is described by `config/carla10.json`, the 0.10 sibling of `config/carla916.json`:

- `map` selects the town. 0.10 ships the `*_Opt` variants, so it is `Town04_Opt` rather than `Town04`.
- `attributes.ros2_ackermann_control` must be `True`, or the ego never moves and neither side reports an error. The
  comment that applies these attributes in `config_carla.py` gives the mechanism.
- `spawn_index` is `5`. Town04_Opt spawn indices changed meaning in 0.10: the old default sits inside static geometry,
  and several low-curvature candidates are open paved aprons with no lane markings on either side. Indices 0-7 are
  middle lanes of the multi-lane highway.

### Steering sign

VisionPilot and CARLA both mean "steer right" by a positive tyre angle, so `carla_control_publisher` passes the value
through unchanged. Its `steering_sign` parameter exists only for a differently-signed source. A value of `-1.0` turns
lane keeping into positive feedback. The comment on that parameter in `carla_control_publisher_node.py` cites the
CARLA sources that settle the convention.

### VisionPilot configuration

The `config/visionpilot*.carla.conf` files are bind-mounted read-only over VisionPilot's own configs at run time so the
in-tree templates stay pristine. `config/H_carla.yaml` must be mounted over `VisionPilot/config/H.yaml`: VisionPilot
loads that filename at runtime, and mounting only `homography_C_matrix.yaml` leaves it projecting AutoSteer waypoints
with the default OpenLane homography. `gen_carla_C_matrix.py` regenerates `config/homography_C_matrix.yaml` from
`config/H_carla.yaml` whenever the camera rig changes.
