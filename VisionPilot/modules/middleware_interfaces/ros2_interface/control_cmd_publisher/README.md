# Control command publisher module

Publishes the agnostic `ControlCommand` (from `modules/control`) as
`ackermann_msgs/AckermannDriveStamped` on a ROS2 topic (issue #288) — the actuation **output** of
the drive loop. Built only when `-DENABLE_ROS2_INTERFACE=ON`.

## What it does

`control_cmd_publisher::ControlCmdPublisher` (`cmd_to_ros2.{hpp,cpp}`):

- Subscribes nothing; it is a pure publisher (no spin thread).
- Maps `ControlCommand` 1:1 → `AckermannDriveStamped.drive`: `steering_angle` (rad, left-positive),
  `speed` (m/s), `acceleration` (m/s²); stamps `header` with `node->now()` and a configurable
  `frame_id`.
- QoS: **reliable, KeepLast(1)** (control is state, not a stream).
- **Init-ownership guard:** initializes ROS2 only if no one else has (`owns_init_ = !rclcpp::ok()`)
  and shuts it down in the dtor only if it owns it — so it coexists with the camera subscriber
  (which owns init/shutdown) without tearing down a shared context.

```cpp
ControlCmdPublisher pub("/control/ackermann_cmd", "base_link");
pub.publish(cmd);   // cmd : ControlCommand
```

## Config (in `vision_pilot.conf`)

| Key | Default | Meaning |
|-----|---------|---------|
| `control.topic` | `/control/ackermann_cmd` | output topic |
| `control.frame_id` | `base_link` | stamped frame |

The app constructs and publishes only when `control.enabled = true`.

## Build & test

Built into the tree with `-DENABLE_ROS2_INTERFACE=ON`. Smoke test (publish→subscribe round-trip in
one process, no weights): `tests/.../control_cmd_publisher/test_control_cmd_publisher.cpp` — run via
`Docker/run.sh smoke`, or build the `test_control_cmd_publisher` target and run the binary.
