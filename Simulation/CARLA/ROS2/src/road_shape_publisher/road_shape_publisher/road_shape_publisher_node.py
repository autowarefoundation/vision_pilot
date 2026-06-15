import rclpy
from rclpy.node import Node

import carla
import math
import numpy as np
from builtin_interfaces.msg import Time 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time

LOCAL_PATH_LEN = 20.0     # meters
STEP_DISTANCE = 0.5       # distance between waypoints
LANE_WIDTH = 4.0          # meters, typical lane width
FRONT2BASE = 1.425        # meters, distance from front of vehicle to hero base link
NEAREST_VISIBLE = 5     # meters, nearest visible waypoint in front of ego vehicle

# TESTS:
# working: 
# - all lanes
# - left and right lane only
# - ego lane and left lane only
# - ego lane and right lane only  
# - left lane only
# - right lane only 
# not working:
# - ego lane only (invalid yaw and curvature, due to not using egoPath as source)

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

class RoadShapePublisher(Node):
    def __init__(self):
        super().__init__('road_shape_publisher')
        
        self.egoPath_viz_pub_ = self.create_publisher(Path, '/egoPath', 2)
        self.egoLaneL_viz_pub_ = self.create_publisher(Path, '/egoLaneL', 2)
        self.egoLaneR_viz_pub_ = self.create_publisher(Path, '/egoLaneR', 2)
        self.global_path_pub_ = self.create_publisher(Path, '/globalPath', 2)
        
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
        self.waypoints = self.get_global_waypoints()
        self.timer = self.create_timer(0.05, self.timer_callback)
            
    def get_global_waypoints(self):
        all_waypts = self.map.generate_waypoints(STEP_DISTANCE)
        print(len(all_waypts), "total waypoints generated in the map")
        waypoints = []

        if "Town06" in self.map.name:
            allowed_road_lane_ids = [(22  , 0.00 , 6),
                                     (1129, 9.34 , 5),
                                     (1129, 0.00 , 7),
                                     (21  , 0.00 , 7),
                                     (404 , 28.40 , 7),
                                     (404 , 0.00 , 5),
                                     (20  , 0.00 , 6),
                                     (67  , 0.00 , -6),
                                     (12  , 0.00 , -6),
                                     (764 , 0.00 , -4),
                                     (764 , 7.57 , -5),
                                     (764 , 19.75 , -6),
                                     (13  , 0.00 , -6),
                                     (484 , 0.00 , -1),
                                     (58  , 0.00 , -3),
                                     (1   , 0.00 , 6),
                                     (245 , 0.00 , 1),
                                     (22  , 0.00 , 6)
                                    ]
        
        else:
            allowed_road_lane_ids = [(17, 0.0, 1),
                                    (10 , 0.0, 1),
                                    (0  , 0.0, 1),
                                    (3  , 0.0, 1),
                                    (565, 0.0, 1),
                                    (2  , 0.0, 1),
                                    (676, 0.0, 1),
                                    (1  , 0.0, 1),
                                    (8  , 0.0, 1),
                                    (4  , 0.0,-1),
                                    (515, 0.0,-1),
                                    (5  , 0.0,-1),
                                    (736, 0.0,-1),
                                    (6  , 0.0,-1),
                                    (89 , 0.0,-1),
                                    (7  , 0.0,-1)]
            
        for road_id, section_id, lane_id in allowed_road_lane_ids:
            seg = []
            for wp in all_waypts:
                if wp.road_id == road_id and wp.section_id == section_id and wp.lane_id == lane_id:
                    seg.append(wp)
            if lane_id > 0:
                seg.reverse()
            waypoints.extend(seg)
            
        for wp in waypoints:
            print(f"Road {wp.road_id}, lane {wp.lane_id}, s {wp.s}, loc {wp.transform.location}")
              
        
        # for road_id, lane_id in allowed_road_lane_ids: 
        #     wp = self.map.get_waypoint_xodr(road_id, lane_id, 0.0)
        #     if wp is None:
        #         self.get_logger.error(f"Could not find waypoint for road {road_id}, lane {lane_id}")
        #         continue
        #     segment = []
            
        #     seg = wp.previous_until_lane_start(STEP_DISTANCE)
            
        #     try: 
        #         next = wp.next_until_lane_end(STEP_DISTANCE)
        #     except RuntimeError as e:
        #         self.get_logger().error(f"Error getting next waypoints for road {road_id}, lane {lane_id}: {e}")
        #         tmp = wp.next(STEP_DISTANCE)
        #         next = []
        #         for t in tmp:
        #             if t.lane_id == lane_id and t.road_id == road_id:
        #                 next.append(t)
        #     if next:
        #         seg.extend(next)
        #     if lane_id > 0:
        #         seg.reverse()
        #     for s in seg:
        #         if s.lane_id == lane_id and s.road_id == road_id:
        #             print(f"Road {s.road_id}, lane {s.lane_id}, s {s.s}, loc {s.transform.location}")
        #             segment.append(s)
        #     waypoints.extend(segment)
        
        print(f"Collected {len(waypoints)} waypoints across {len(allowed_road_lane_ids)} lanes")
        
        global_path_msg = Path()
        elapsed = self.world.get_snapshot().timestamp.elapsed_seconds
        ros_time = Time()
        ros_time.sec = int(elapsed)
        ros_time.nanosec = int((elapsed - ros_time.sec) * 1e9)
        global_path_msg.header.stamp = ros_time
        global_path_msg.header.frame_id = "odom"
        
        for i, wp in enumerate(waypoints):
            ps = PoseStamped()
            ps.header = global_path_msg.header
            ps.pose.position.x = wp.transform.location.x
            ps.pose.position.y = -wp.transform.location.y
            ps.pose.position.z = wp.transform.location.z
            wp_yaw = math.radians(wp.transform.rotation.yaw)
            q = yaw_to_quaternion(math.degrees(wp_yaw))
            ps.pose.orientation.x = q["x"]
            ps.pose.orientation.y = q["y"]
            ps.pose.orientation.z = q["z"]
            ps.pose.orientation.w = q["w"]
            global_path_msg.poses += [ps]
        self.global_path_pub_.publish(global_path_msg)

        return waypoints

    def _find_ego_vehicle(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'hero':
                return actor
        self.get_logger().error('Ego vehicle not found')
        return None
        
    def timer_callback(self):
        if not self.ego:
            return
        waypoints = self.waypoints

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
    
        path_msg = Path()
        path_msg.header.stamp = ros_time
        path_msg.header.frame_id = "hero"
        path_msg.poses = []
            
        left_lane = Path()
        right_lane = Path()
        left_lane.header = path_msg.header
        right_lane.header = path_msg.header
        
        # Find nearest waypoint index
        nearest_idx = None
        nearest_dist = float('inf')
        for i, wp in enumerate(waypoints):
            d = (wp.transform.location.x - ego_loc.x)**2 + (wp.transform.location.y - ego_loc.y)**2
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        # Extract local horizon waypoints
        horizon = int(LOCAL_PATH_LEN / STEP_DISTANCE)
        offset = int(NEAREST_VISIBLE / STEP_DISTANCE) # offset to account for visibility in camera FOV 
        local_wps = [waypoints[(nearest_idx + i + offset) % len(waypoints)] for i in range(horizon)]
        print(len(local_wps), "local waypoints extracted")
        
        for wp in local_wps:
            wp_loc = wp.transform.location
            wp_pos = np.array([wp_loc.x - ego_loc.x,
                            wp_loc.y - ego_loc.y,
                            wp_loc.z - ego_loc.z])
            local_pos = R_world_to_ego @ wp_pos  # rotate to ego frame
            
            ps = PoseStamped()
            ps.header.stamp = ros_time
            ps.header.frame_id = "hero_front"  # relative to front of ego vehicle
            ps.pose.position.x = local_pos[0] - FRONT2BASE
            ps.pose.position.y = -local_pos[1]
            ps.pose.position.z = local_pos[2]

            wp_yaw = math.radians(wp.transform.rotation.yaw)
            relative_yaw = wp_yaw - ego_yaw
            q = yaw_to_quaternion(math.degrees(relative_yaw))
            ps.pose.orientation.x = q["x"]
            ps.pose.orientation.y = q["y"]
            ps.pose.orientation.z = q["z"]
            ps.pose.orientation.w = q["w"]
            
            path_msg.poses.append(ps)
            
            # Create left lane
            left_ps = PoseStamped()
            left_ps.header = ps.header
            left_ps.pose.position.x = ps.pose.position.x - LANE_WIDTH / 2 * math.sin(-relative_yaw)
            left_ps.pose.position.y = ps.pose.position.y + LANE_WIDTH / 2 * math.cos(-relative_yaw)
            left_ps.pose.position.z = ps.pose.position.z
            left_ps.pose.orientation = ps.pose.orientation
            left_lane.poses.append(left_ps)

            # Create right lane
            right_ps = PoseStamped()
            right_ps.header = ps.header
            right_ps.pose.position.x = ps.pose.position.x + LANE_WIDTH / 2 * math.sin(-relative_yaw)
            right_ps.pose.position.y = ps.pose.position.y - LANE_WIDTH / 2 * math.cos(-relative_yaw)
            right_ps.pose.position.z = ps.pose.position.z
            right_ps.pose.orientation = ps.pose.orientation
            right_lane.poses.append(right_ps)
            
        if path_msg.poses:
            # Comment out to test partially missing lane detections
            self.egoPath_viz_pub_.publish(path_msg)
            self.egoLaneL_viz_pub_.publish(left_lane)
            self.egoLaneR_viz_pub_.publish(right_lane)  
            self.get_logger().info(f'Publishing egoPath and egoLanes with {len(path_msg.poses)} waypoints on lane {waypoints[0].lane_id}')
        
def main(args=None):
    rclpy.init(args=args)
    node = RoadShapePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
