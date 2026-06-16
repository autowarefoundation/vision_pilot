#!/usr/bin/env python3
"""CARLA → ROS2 ego odometry bridge.

CARLA UE5 0.10 has no native vehicle odometry over ROS2 (sensors expose
enable_for_ros(); the Vehicle actor does not — see the CARLA-ROS2 findings doc).
VisionPilot's MPC therefore reads ego_v = 0, which removes its cross-track /
heading correction authority and pins cruise speed at the crawl. This node closes
that gap: it reads the ego's velocity from the CARLA Python API and publishes
nav_msgs/Odometry (body-frame twist) on the topic VisionPilot subscribes to.

Runs in a RoboStack conda env (Python 3.10) so carla (cp310-only) and rclpy
coexist in one interpreter — the only combination that works given carla 0.10
ships no cp312 wheel and ROS2 Humble (py3.10) can't satisfy carla's glibc 2.38.

Env overrides:
  CARLA_HOST (localhost), CARLA_PORT (2000),
  CARLA_ODOM_TOPIC (/carla/hero/odometry), CARLA_EGO_ROLE (hero)
"""
import math
import os
import time

import carla
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CarlaOdomBridge(Node):
    def __init__(self):
        super().__init__("carla_odom_bridge")
        host = os.environ.get("CARLA_HOST", "localhost")
        port = int(os.environ.get("CARLA_PORT", "2000"))
        self.topic = os.environ.get("CARLA_ODOM_TOPIC", "/carla/hero/odometry")
        self.role = os.environ.get("CARLA_EGO_ROLE", "hero")

        self.client = carla.Client(host, port)
        self.client.set_timeout(60.0)
        self.world = self.client.get_world()

        self.ego = None
        while self.ego is None:
            self.ego = self._find_ego()
            if self.ego is None:
                self.get_logger().warn("ego '%s' not found, waiting..." % self.role)
                time.sleep(1.0)

        self.pub = self.create_publisher(Odometry, self.topic, 10)
        dt = self.world.get_settings().fixed_delta_seconds or 0.05
        self.timer = self.create_timer(dt, self.tick)
        self.get_logger().info("publishing ego odometry on %s (dt=%.3f)" % (self.topic, dt))

    def _find_ego(self):
        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == self.role:
                return actor
        return None

    def tick(self):
        if self.ego is None:
            return
        try:
            tf = self.ego.get_transform()
            v = self.ego.get_velocity()  # world frame (CARLA left-handed)
            w = self.ego.get_angular_velocity()
            snap_t = self.world.get_snapshot().timestamp.elapsed_seconds
        except RuntimeError as e:
            # Transient CARLA RPC hiccup (e.g. ego re-resolved mid-tick). Skip this
            # tick rather than letting the exception unwind the node.
            self.get_logger().warn("CARLA RPC failed this tick, skipping: %s" % e)
            return
        yaw = math.radians(tf.rotation.yaw)
        # World → body frame (flip y for CARLA LH → ROS RH).
        c, s = math.cos(yaw), math.sin(yaw)
        v_body = np.array([[c, -s], [s, c]]) @ np.array([v.x, -v.y])

        stamp = Time(sec=int(snap_t), nanosec=int((snap_t - int(snap_t)) * 1e9))
        q = quaternion_from_yaw(math.radians(-tf.rotation.yaw))

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = self.role
        msg.pose.pose = Pose(
            position=Point(x=tf.location.x, y=-tf.location.y, z=tf.location.z),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )
        msg.twist.twist.linear = Vector3(x=float(v_body[0]), y=float(v_body[1]), z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=math.radians(w.z))
        self.pub.publish(msg)


def main():
    # Do NOT let rclpy install signal handlers: carla (libcarla) shares this
    # interpreter and perturbs signal delivery, which intermittently tripped
    # rclpy's handler and tore the context down right after the first publish
    # (ExternalShutdownException). Without the handler the bridge keeps running;
    # docker stop (SIGTERM/SIGKILL) still stops the container cleanly.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CarlaOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
