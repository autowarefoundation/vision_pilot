from launch import LaunchDescription
from launch_ros.actions import Node

import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    install_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) 
    ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(install_dir))))
    script_path = os.path.abspath(os.path.join(ws_dir, 'ros_carla_config.py'))
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script_path,'-f', os.path.join(ws_dir,'config/VisionPilot_carla9.json')],
            output='screen'
        ),
        
        Node(
            package='steering_controller',  
            executable='steering_controller_node',
            name='steering_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='longitudinal_controller',  
            executable='longitudinal_controller_node',
            name='longitudinal_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='carla_control_publisher',
            executable='pub_carla_control',
            name='pub_carla_control',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='road_shape_publisher',
            executable='road_shape_publisher_node',
            name='road_shape_publisher_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='PATHFINDER',
            executable='pathfinder_node',
            name='pathfinder_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='odom_publisher',
            executable='pub_odom_node',
            name='pub_odom_node',
            output='screen',           
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_to_base_broadcaster',
            arguments=[
                "1.425", "0", "0",   # translation: x y z
                "0", "0", "0",   # rotation in rpy (roll pitch yaw in radians)
                "hero",           # parent frame
                "hero_front"           # child frame
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.abspath(os.path.join(ws_dir, 'config', 'PathFinder.rviz'))
            ]
        ),
    ])