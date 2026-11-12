import rclpy
from rclpy.node import Node

import numpy as np
import math
import carla
from autoware_control_msgs.msg import Control
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float64

MAX_ACCELERATION = 1.5   # m/s²
MAX_DECELERATION = 3.0   # m/s²
A_DES_MIN = -8.0         # reject planner spikes below this (phantom CIPO gave -50)
A_DES_MAX = MAX_ACCELERATION


class CarlaControlPublisher(Node):
    def __init__(self):
        super().__init__('carla_control_publisher')
        self.steering_sub_ = self.create_subscription(Float64, '/vehicle/steering_cmd', self.steering_callback, 1)
        self.throttle_sub_ = self.create_subscription(Float64, '/vehicle/throttle_cmd', self.throttle_callback, 1)
        self.speed_sub_ = self.create_subscription(Float64, '/vehicle/speed', self.speed_callback, 1)
        self.control_cmd_pub_ = self.create_publisher(Control, "/control/command/control_cmd", 1)

        self.speed = 0.0
        self.v_ref = 30.0
        self.steering_angle_cmd = 0.0
        self.acceleration = 0.0

        # Publish only once BOTH a fresh steering and a fresh throttle have
        # arrived. Each planning cycle in VisionPilot emits one steering + one
        # throttle together, so this fires once per cycle, at the planner's own
        # rate, with no zero-order-hold delay.
        self.have_steering = False
        self.have_throttle = False

        # Failsafe watchdog: if upstream stalls, don't keep applying the last
        # (possibly full-lock) command forever. See note below.
        self.last_cmd_time = self.get_clock().now()
        # self.watchdog = self.create_timer(0.2, self.watchdog_callback)


    def publish_control(self):
        v = max(0.0, self.speed + self.acceleration * 2.0)

        now = self.get_clock().now().to_msg()

        cmd = Control()
        cmd.stamp = now
        cmd.control_time = now

        cmd.lateral.stamp = now
        cmd.lateral.control_time = now
        # VisionPilot uses CARLA's convention (positive = right); autoware_control_msgs
        # uses Autoware's (positive = left). autoware_carla_interface negates again
        # when applying the actuation cmd, so this flip is required.
        cmd.lateral.steering_tire_angle = -float(self.steering_angle_cmd)  # radians
        cmd.lateral.steering_tire_rotation_rate = 0.0
        cmd.lateral.is_defined_steering_tire_rotation_rate = False

        cmd.longitudinal.stamp = now
        cmd.longitudinal.control_time = now
        cmd.longitudinal.velocity = float(v)
        cmd.longitudinal.acceleration = float(self.acceleration)
        cmd.longitudinal.jerk = 10.0
        cmd.longitudinal.is_defined_acceleration = True
        cmd.longitudinal.is_defined_jerk = True

        self.control_cmd_pub_.publish(cmd)

        # self.last_cmd_time = now

    def try_publish(self):
        if self.have_steering and self.have_throttle:
            self.publish_control()
            self.have_steering = False
            self.have_throttle = False

    def steering_callback(self, msg):
        # self.get_logger().info(f'Steering command received: {msg.data}')
        self.steering_angle_cmd = msg.data
        self.have_steering = True
        self.try_publish()

    def throttle_callback(self, msg):
        # self.get_logger().info(f'Throttle command received: {msg.data}')
        self.acceleration = msg.data
        self.have_throttle = True
        self.try_publish()

    def speed_callback(self, msg):
        self.speed = msg.data

    def watchdog_callback(self):
        # If no fresh pair has been published recently, command a safe stop
        # (zero throttle, no brake) rather than latching the last steering.
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > 0.2:
            msg = CarlaEgoVehicleControl()
            msg.throttle = 0.0
            msg.steer = 0.0
            msg.brake = 0.0
            msg.gear = 1
            self.control_pub_.publish(msg)
            self.get_logger().warn(f'Watchdog: no fresh command for {dt:.2f}s — safing')


def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
