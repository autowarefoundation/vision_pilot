import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    install_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(install_dir))))

    camera_publisher_node = Node(
        package='camera_publisher',
        executable='camera_publisher_node',
        name='camera_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    camera_spectator_node = Node(
        package='camera_spectator',
        executable='camera_spectator_node',
        name='camera_spectator_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    carla_control_publisher_node = Node(
        package='carla_control_publisher',
        executable='pub_carla_control',
        name='pub_carla_control',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    road_shape_publisher_node = Node(
        package='road_shape_publisher',
        executable='road_shape_publisher_node',
        name='road_shape_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    odom_publisher_node = Node(
        package='odom_publisher',
        executable='pub_odom_node',
        name='pub_odom_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    waypoint_publisher_node = Node(
        package='waypoints_publisher',
        executable='pub_waypoints_node',
        name='pub_waypoints_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tf2_ros_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_to_base_broadcaster',
        arguments=[
            "1.425", "0", "0",  # translation: x y z
            "0", "0", "0",  # rotation in rpy (roll pitch yaw in radians)
            "hero",  # parent frame
            "hero_front"  # child frame
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.abspath(os.path.join(ws_dir, 'config', 'PathFinder.rviz'))
        ]
    )

    visionpilot_publisher_node = Node(
        package='visionpilot_publisher',
        executable='visionpilot_publisher_node',
        name='visionpilot_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        camera_publisher_node,
        camera_spectator_node,
        carla_control_publisher_node,
        # road_shape_publisher_node,
        # odom_publisher_node,
        # waypoint_publisher_node,
        tf2_ros_node,
        # rviz_node,
        visionpilot_publisher_node,
    ])
