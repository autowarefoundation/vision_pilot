"""VisionPilot <-> CARLA bridge for a world owned by `autoware_carla_interface`.

Use this instead of carla_bridge.launch.py when the CARLA map, ego vehicle and sensors are
spawned by `autoware_carla_interface` (autoware_carla_tum workspace). config_carla.py is
NOT run: the interface already loads the world, spawns the ego (role_name=ego_vehicle),
drives the sim clock and publishes the camera on /sensing/camera/CAM_FRONT/image_raw.

Only two lightweight nodes are started here:
  velocity_status_bridge  /vehicle/status/velocity_status -> /vehicle/speed,
                                                             /localization/kinematic_state
  carla_control_publisher /vehicle/{steering,throttle}_cmd -> /control/command/control_cmd

The interface's autoware_raw_vehicle_cmd_converter then turns control_cmd into
/control/command/actuation_cmd, which autoware_carla_interface applies to the ego.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    velocity_status_topic_arg = DeclareLaunchArgument(
        'velocity_status_topic', default_value='/vehicle/status/velocity_status')
    odometry_topic_arg = DeclareLaunchArgument(
        'odometry_topic', default_value='/localization/kinematic_state',
        description='Odometry topic fed to autoware_raw_vehicle_cmd_converter')
    publish_odometry_arg = DeclareLaunchArgument(
        'publish_odometry', default_value='true',
        description='Set false if something else (e.g. use_e2e_planning) already publishes odometry')

    velocity_status_bridge_node = Node(
        package='carla_vehicle_speed_publisher',
        executable='velocity_status_bridge_node',
        name='velocity_status_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'velocity_status_topic': LaunchConfiguration('velocity_status_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'publish_odometry': LaunchConfiguration('publish_odometry'),
        }],
    )

    carla_control_publisher_node = Node(
        package='carla_control_publisher',
        executable='carla_control_publisher_node',
        name='carla_control_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        velocity_status_topic_arg,
        odometry_topic_arg,
        publish_odometry_arg,
        velocity_status_bridge_node,
        carla_control_publisher_node,
    ])
