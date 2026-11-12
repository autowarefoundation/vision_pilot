"""Bridge autoware_carla_interface vehicle status into the topics VisionPilot + its bridge need.

Used when the CARLA world/ego is owned by `autoware_carla_interface` (autoware_carla_tum
workspace) instead of config_carla.py. No CARLA PythonAPI client is needed: everything
is derived from ROS 2 topics the interface already publishes.

  /vehicle/status/velocity_status (VelocityReport)
      -> /vehicle/speed              (Float64, m/s)          consumed by VisionPilot
      -> /localization/kinematic_state (Odometry, twist only) consumed by
         autoware_raw_vehicle_cmd_converter, which refuses to convert
         /control/command/control_cmd -> /control/command/actuation_cmd until it has
         received an odometry message (it reads twist.twist.linear.x only).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from autoware_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class VelocityStatusBridgeNode(Node):
    def __init__(self):
        super().__init__('velocity_status_bridge_node')

        self.declare_parameter('velocity_status_topic', '/vehicle/status/velocity_status')
        self.declare_parameter('speed_topic', '/vehicle/speed')
        self.declare_parameter('odometry_topic', '/localization/kinematic_state')
        self.declare_parameter('publish_odometry', True)
        self.declare_parameter('odom_frame_id', 'map')
        self.declare_parameter('odom_child_frame_id', 'base_link')

        vel_topic = self.get_parameter('velocity_status_topic').value
        speed_topic = self.get_parameter('speed_topic').value
        odom_topic = self.get_parameter('odometry_topic').value
        self.publish_odometry = self.get_parameter('publish_odometry').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.odom_child_frame_id = self.get_parameter('odom_child_frame_id').value

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE)

        self.speed_pub_ = self.create_publisher(Float64, speed_topic, 1)
        self.odom_pub_ = (self.create_publisher(Odometry, odom_topic, 1)
                          if self.publish_odometry else None)
        self.vel_sub_ = self.create_subscription(
            VelocityReport, vel_topic, self.velocity_callback, qos)

        self.get_logger().info(
            f'{vel_topic} -> {speed_topic}'
            + (f' + {odom_topic}' if self.publish_odometry else ''))

    def velocity_callback(self, msg: VelocityReport):
        speed = Float64()
        speed.data = float(math.hypot(msg.longitudinal_velocity, msg.lateral_velocity))
        self.speed_pub_.publish(speed)

        if self.odom_pub_ is None:
            return
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.odom_child_frame_id
        # Pose is intentionally left at identity: the cmd converter only uses the twist.
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(msg.longitudinal_velocity)
        odom.twist.twist.linear.y = float(msg.lateral_velocity)
        odom.twist.twist.angular.z = float(msg.heading_rate)
        self.odom_pub_.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityStatusBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
