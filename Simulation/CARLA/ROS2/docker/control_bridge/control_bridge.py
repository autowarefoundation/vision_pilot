#!/usr/bin/env python3
"""Ackermann -> CarlaEgoVehicleControl bridge.

CARLA UE5 0.10 Shipping builds have no ros2_ackermann_control, so the ego only
accepts carla_msgs/CarlaEgoVehicleControl on /carla/hero/vehicle_control_cmd. This
node normalizes VisionPilot's agnostic ackermann output into it:
  steering_angle [rad] -> steer [-1, 1]  (divide by max steer angle)
  target speed  [m/s] -> throttle/brake  (P-controller on /carla/hero/odometry speed)
Topics and gains are env-configurable.
"""
import math
import os

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry
from rclpy.node import Node


class AckermannToVehicleControl(Node):
    def __init__(self):
        super().__init__("ackermann_to_vehicle_control")
        self.ack_topic = os.environ.get("ACK_TOPIC", "/carla/hero/ackermann_control_cmd")
        self.odom_topic = os.environ.get("ODOM_TOPIC", "/carla/hero/odometry")
        self.ctrl_topic = os.environ.get("CTRL_TOPIC", "/carla/hero/vehicle_control_cmd")
        # Vehicle max front-wheel steer angle [rad] used to normalize to [-1, 1].
        # lincoln.mkz ~ 70 deg ~ 1.22 rad.
        self.max_steer = float(os.environ.get("MAX_STEER_RAD", "1.22"))
        self.kp = float(os.environ.get("SPEED_KP", "0.5"))  # throttle/brake P-gain
        self.throttle_max = float(os.environ.get("THROTTLE_MAX", "0.7"))
        self.brake_deadband = float(os.environ.get("BRAKE_DEADBAND", "0.5"))  # m/s

        self.cur_speed = 0.0
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(AckermannDriveStamped, self.ack_topic, self._on_ack, 10)
        self.pub = self.create_publisher(CarlaEgoVehicleControl, self.ctrl_topic, 10)
        self.get_logger().info(
            "bridging %s + %s -> %s (max_steer=%.3f rad, kp=%.2f)"
            % (self.ack_topic, self.odom_topic, self.ctrl_topic, self.max_steer, self.kp)
        )

    def _on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.cur_speed = math.hypot(v.x, v.y)

    def _on_ack(self, msg: AckermannDriveStamped):
        target = msg.drive.speed
        steer_rad = msg.drive.steering_angle

        out = CarlaEgoVehicleControl()
        out.header = msg.header
        out.steer = float(max(-1.0, min(1.0, steer_rad / self.max_steer)))

        err = target - self.cur_speed
        if err > 0.0:
            out.throttle = float(max(0.0, min(self.throttle_max, self.kp * err)))
            out.brake = 0.0
        elif err < -self.brake_deadband:
            out.throttle = 0.0
            out.brake = float(max(0.0, min(1.0, -self.kp * err)))
        else:
            out.throttle = 0.0
            out.brake = 0.0

        out.hand_brake = False
        out.reverse = False
        out.gear = 1
        out.manual_gear_shift = False
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AckermannToVehicleControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
