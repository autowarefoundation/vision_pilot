import rclpy
from rclpy.node import Node

import carla
import math
import numpy as np
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from builtin_interfaces.msg import Time

class CarlaOdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(60.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        while True:
            self.ego = self._find_ego_vehicle()
            if self.ego:
                break
            self.get_logger().warn('Ego vehicle not found, waiting ...')
            time.sleep(1.0)
        self.odom_pub_ = self.create_publisher(Odometry, '/hero/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(self.world.get_settings().fixed_delta_seconds, self.timer_callback)

    def _find_ego_vehicle(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'hero':
                return actor
        return None

    def carla_transform_to_ros_pose(self, transform):
        """Convert CARLA transform to ROS Pose (LH → RH)"""
        location = transform.location
        rotation = transform.rotation

        # Position (flip y)
        x = location.x
        y = -location.y
        z = location.z

        # Orientation (flip pitch and yaw signs)
        roll = math.radians(rotation.roll)
        pitch = math.radians(-rotation.pitch)
        yaw = math.radians(-rotation.yaw)

        q = quaternion_from_euler(roll, pitch, yaw)

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def flip_vector_y(self, vector):
        return Vector3(x=vector.x, y=-vector.y, z=vector.z)

    def publish_tf(self, odom: Odometry):
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        if not self.ego:
            return
    
        transform = self.ego.get_transform()
        velocity = self.ego.get_velocity()
        yaw = math.radians(transform.rotation.yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s],
                    [s, c]])
        v_xy = R @ np.array([velocity.x, -velocity.y])
   
        angular_velocity = self.ego.get_angular_velocity()
        pose = self.carla_transform_to_ros_pose(transform)
        
        snapshot = self.world.get_snapshot()
        elapsed = snapshot.timestamp.elapsed_seconds

        # Create ROS time
        ros_time = Time()
        ros_time.sec = int(elapsed)
        ros_time.nanosec = int((elapsed - ros_time.sec) * 1e9)
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = ros_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'hero'
        odom_msg.pose.pose = pose
        odom_msg.twist.twist.linear = Vector3(x=v_xy[0], y=v_xy[1], z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=math.radians(angular_velocity.z))

        self.odom_pub_.publish(odom_msg)

        # Publish TF
        self.publish_tf(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
