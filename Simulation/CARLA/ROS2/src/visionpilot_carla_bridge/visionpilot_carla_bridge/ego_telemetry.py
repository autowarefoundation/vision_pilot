#!/usr/bin/env python3
"""Publish CARLA ego telemetry as ROS 2 topics — runs IN the bridge container.

CARLA emits no usable ROS odometry, so speed must be sourced from the PythonAPI.
Publishing it from the host proved unreliable (host rclpy <-> container Jazzy DDS
discovery flakes silently -> the bridge watchdog holds the car forever), so this node
keeps ALL ROS publishing inside the container's DDS domain: it connects to the CARLA
server over plain TCP (host:2000) as a client and publishes:

  speed_topic      std_msgs/Float64  ego speed [m/s]  (consumed by VisionPilot + the
                                     bridge watchdog — one speed source of truth)
  max_steer_topic  std_msgs/Float64  ego front-wheel steer limit [rad], latched once

Needs the `carla` package on PYTHONPATH (drive.sh stages the matching wheel and
mounts it). The ego is found by role_name and re-searched if it disappears (respawn);
while absent nothing is published, so the bridge watchdog safely brakes.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

try:
    import carla
except ImportError as exc:  # clear failure instead of a bare stack trace
    raise SystemExit(
        "ego_telemetry: cannot import the 'carla' PythonAPI package — stage the wheel "
        "matching this python onto PYTHONPATH (drive.sh does this automatically)"
    ) from exc


class EgoTelemetry(Node):
    def __init__(self):
        super().__init__("carla_ego_telemetry")
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("role_name", "hero")
        self.declare_parameter("speed_topic", "/vehicle/speed")
        self.declare_parameter("max_steer_topic", "/carla_bridge/max_steer_angle")
        self.declare_parameter("rate_hz", 20.0)

        g = self.get_parameter
        self.role_name = g("role_name").value
        self.speed_pub = self.create_publisher(Float64, g("speed_topic").value, 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.max_steer_pub = self.create_publisher(Float64, g("max_steer_topic").value, latched)

        self.client = carla.Client(g("carla_host").value, int(g("carla_port").value))
        self.client.set_timeout(5.0)
        self.ego = None
        self.timer = self.create_timer(1.0 / float(g("rate_hz").value), self._tick)
        self.get_logger().info(
            "telemetry: CARLA %s:%d role_name=%s -> %s @ %.0f Hz"
            % (
                g("carla_host").value,
                int(g("carla_port").value),
                self.role_name,
                g("speed_topic").value,
                float(g("rate_hz").value),
            )
        )

    def _find_ego(self):
        try:
            for actor in self.client.get_world().get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") == self.role_name:
                    return actor
        except RuntimeError as exc:  # server not up yet / restarting
            self.get_logger().warn("CARLA not reachable: %s" % exc, throttle_duration_sec=5.0)
        return None

    def _publish_max_steer(self):
        wheels = self.ego.get_physics_control().wheels
        if not wheels or wheels[0].max_steer_angle <= 0.0:
            self.get_logger().warn("ego reports no usable max_steer_angle; keeping fallback")
            return
        max_steer_rad = math.radians(wheels[0].max_steer_angle)
        self.max_steer_pub.publish(Float64(data=max_steer_rad))
        self.get_logger().info("published ego max_steer_angle %.4f rad (latched)" % max_steer_rad)

    def _tick(self):
        if self.ego is None or not self.ego.is_alive:
            self.ego = self._find_ego()
            if self.ego is None:
                # No ego -> no speed published -> the bridge watchdog brakes. Safe.
                self.get_logger().warn(
                    "waiting for ego '%s'..." % self.role_name, throttle_duration_sec=5.0
                )
                return
            self.get_logger().info("found ego %s (id %d)" % (self.ego.type_id, self.ego.id))
            self._publish_max_steer()
        try:
            v = self.ego.get_velocity()
        except RuntimeError:  # ego destroyed between checks
            self.ego = None
            return
        self.speed_pub.publish(Float64(data=math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)))


def main(args=None):
    rclpy.init(args=args)
    node = EgoTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
