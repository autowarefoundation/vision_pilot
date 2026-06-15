import rclpy
from rclpy.node import Node

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time 

import carla
import math
import numpy as np

LOOKAHEAD_DISTANCE = 100.0  # meters
STEP_DISTANCE = 2.0        # distance between waypoints
DEFAULT_SPEED = 10.0       # m/s constant assumed speed

def yaw_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0)
    }
def rpy_to_matrix(roll, pitch, yaw):
    """Return 3x3 rotation matrix from roll, pitch, yaw (in radians)"""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr]
    ])
    return R

class CarlaTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('carla_trajectory_publisher')

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.ego = self._find_ego_vehicle()
        if self.ego is None:
            self.get_logger().error('Ego vehicle not found, exiting.')
            rclpy.shutdown()
            return
    
        self.publisher_ = self.create_publisher(Trajectory, '/planning/trajectory', 10)
        self.path_pub = self.create_publisher(Path, '/planning/path', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _find_ego_vehicle(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'hero':
                return actor
        self.get_logger().error('Ego vehicle not found')
        return None

    def timer_callback(self):
        if not self.ego:
            return

        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        ego_rot = ego_tf.rotation
        ego_yaw = math.radians(ego_rot.yaw)
        ego_pitch = -math.radians(ego_rot.pitch) # CARLA uses left-handed coordinate system
        ego_roll = math.radians(ego_rot.roll)

        R_world_to_ego = rpy_to_matrix(ego_roll, ego_pitch, ego_yaw).T  # inverse = transpose

        snapshot = self.world.get_snapshot()
        elapsed = snapshot.timestamp.elapsed_seconds

        # Create ROS time
        ros_time = Time()
        ros_time.sec = int(elapsed)
        ros_time.nanosec = int((elapsed - ros_time.sec) * 1e9)

        traj_msg = Trajectory()
        traj_msg.header.frame_id = "hero"
        traj_msg.header.stamp = ros_time

        path_msg = Path()
        path_msg.header = traj_msg.header

        curr_wp = self.map.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        total_dist = 0.0
        time_from_start = 0.0

        while total_dist < LOOKAHEAD_DISTANCE and curr_wp is not None:
            wp_loc = curr_wp.transform.location
            wp_pos = np.array([wp_loc.x - ego_loc.x,
                            wp_loc.y - ego_loc.y,
                            wp_loc.z - ego_loc.z])
            local_pos = R_world_to_ego @ wp_pos  # rotate to ego frame

            pt = TrajectoryPoint()
            pt.pose.position.x = local_pos[0]
            pt.pose.position.y = -local_pos[1] # CARLA uses left-handed coordinate system
            pt.pose.position.z = local_pos[2]

            # Orientation: relative yaw only (you can also convert full R_w to local frame if needed)
            wp_yaw = math.radians(curr_wp.transform.rotation.yaw)
            relative_yaw = wp_yaw - ego_yaw
            q = yaw_to_quaternion(math.degrees(relative_yaw))
            pt.pose.orientation.x = q["x"]
            pt.pose.orientation.y = q["y"]
            pt.pose.orientation.z = q["z"]
            pt.pose.orientation.w = q["w"]

            pt.longitudinal_velocity_mps = DEFAULT_SPEED
            pt.time_from_start.sec = int(time_from_start)
            pt.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
            traj_msg.points.append(pt)

            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose = pt.pose
            path_msg.poses.append(ps)

            # Advance
            next_wps = curr_wp.next(STEP_DISTANCE)
            if not next_wps:
                break
            next_wp = next_wps[0]
            dist = curr_wp.transform.location.distance(next_wp.transform.location)
            total_dist += dist
            time_from_start += dist / DEFAULT_SPEED
            curr_wp = next_wp

        self.publisher_.publish(traj_msg)
        self.path_pub.publish(path_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = CarlaTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
