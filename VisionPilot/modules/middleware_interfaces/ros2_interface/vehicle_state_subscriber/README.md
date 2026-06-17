# Vehicle state subscriber module

Subscribes to vehicle **odometry** over ROS2 and exposes the live **ego speed** to the drive loop
(issue #288, Phase 5) — the vehicle-state **input** that closes the longitudinal loop. Built only
when `-DENABLE_ROS2_INTERFACE=ON`.

## What it does

`vehicle_state_subscriber::VehicleStateSubscriber` (`ros2_to_can.{hpp,cpp}` — name kept for the #288
"convert to internal CAN/state" intent):

- Subscribes `nav_msgs/Odometry` (QoS reliable, KeepLast(1)).
- `ego_speed_mps()` = `hypot(twist.twist.linear.x, .y)` (planar speed); `has_state()` is false until
  the first message, so callers fall back to a configured constant.
- Runs a flag-controlled `spin_some` background loop and uses the same **init-ownership guard** as
  the control publisher, so it never shuts down a context owned by a co-resident node.

```cpp
VehicleStateSubscriber sub("/vehicle/odometry");
double v = sub.has_state() ? sub.ego_speed_mps() : cfg.control.ego_speed_mps;
```

## Config (in `vision_pilot.conf`)

| Key | Default | Meaning |
|-----|---------|---------|
| `control.vehicle_state_topic` | `/vehicle/odometry` | odometry topic (CARLA overlay: `/hero/odom`) |
| `control.ego_speed_mps` | `10.0` | fallback ego speed until odometry arrives |

Used only when `control.enabled = true`.

## Build & test

Built with `-DENABLE_ROS2_INTERFACE=ON`. Smoke test (publish a known `Odometry`, read back the ego
speed): `tests/.../vehicle_state_subscriber/test_vehicle_state_subscriber.cpp` — run via
`Docker/run.sh smoke`, or build the `test_vehicle_state_subscriber` target and run the binary.
