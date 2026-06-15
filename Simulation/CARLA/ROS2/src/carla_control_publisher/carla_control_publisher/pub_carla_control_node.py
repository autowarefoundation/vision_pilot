import math
import carla
import rclpy
from rclpy.node import Node

import numpy as np
# from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class CarlaControlPublisher(Node):
    def __init__(self):
        super().__init__('carla_control_publisher')
        self.vehicle_control_cmd_topic = "/carla/hero/vehicle_control_cmd"
        self.steering_angle_sub_ = self.create_subscription(Float32, '/vehicle/steering_angle', self.steering_callback, 1)
        self.speed_sub_ = self.create_subscription(Float32, '/vehicle/speed', self.speed_callback, 1)
        self.brake_sub_    = self.create_subscription(Float32, '/vehicle/brake_cmd', self.brake_callback, 1)

        # Connect to CARLA to identify which actor is 'hero'
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        hero_actor = None
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'hero':
                hero_actor = actor
                actor_id = hero_actor.id
                self.vehicle_control_cmd_topic = f"/carla/actor{actor_id}/ackermann_control_cmd"
                break

        # self.control_pub_ = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 1)
        self.control_pub_ = self.create_publisher(AckermannDriveStamped, self.vehicle_control_cmd_topic, 1)
        # self.timer = self.create_timer(0.02, self.timer_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.steering_angle = 0.0
        self.speed = 0.0
        self.brake_cmd = 0.0

    # def timer_callback(self):
    #     msg = CarlaEgoVehicleControl()
    #     msg.throttle = self.throttle_cmd if self.throttle_cmd > 0 else 0.0
    #     msg.steer = np.clip((180.0 / np.pi) * (self.steering_angle_cmd / 70.0), -1.0, 1.0) # vehicle specific
    #     msg.brake = self.brake_cmd
    #     msg.hand_brake = False
    #     msg.reverse = False
    #     msg.manual_gear_shift = False
    #     msg.gear = 0
    #     self.control_pub_.publish(msg)

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # steering angle in radians (NOT normalized)
        # msg.drive.steering_angle = math.radians(self.steering_angle)  # rad
        carla_tyre_angle = self.steering_angle * (-1 / 10.3)
        msg.drive.steering_angle = math.radians(carla_tyre_angle)  # rad
        self.get_logger().info(f'Steering deg: {self.steering_angle}')
        self.get_logger().info(f'Steering rad: { msg.drive.steering_angle}')

        # desired speed (m/s)
        msg.drive.speed = self.speed / 3.6

        # optional
        msg.drive.acceleration = 0.0
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.jerk = 0.0
        self.control_pub_.publish(msg)
    
    def steering_callback(self, msg):
        # self.get_logger().info(f'Steering command received: {msg.data}')
        self.steering_angle = msg.data
        
    def speed_callback(self, msg):
        # self.get_logger().info(f'Speed command received: {msg.data}')
        self.speed = msg.data
        
    def brake_callback(self, msg):
        # self.get_logger().info(f'Brake command received: {msg.data}')
        self.brake_cmd = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
