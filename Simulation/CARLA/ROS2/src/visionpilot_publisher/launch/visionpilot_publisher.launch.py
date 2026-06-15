from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    visionpilot_publisher_node = Node(
        package='visionpilot_publisher',
        executable='visionpilot_publisher_node',
        name='visionpilot_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        visionpilot_publisher_node,
    ])
