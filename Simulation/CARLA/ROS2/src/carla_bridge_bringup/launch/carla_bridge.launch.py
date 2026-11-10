import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    install_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(install_dir))))

    host_arg = DeclareLaunchArgument('host', default_value='localhost')
    port_arg = DeclareLaunchArgument('port', default_value='2000')

    map_arg = DeclareLaunchArgument('map', default_value='Town04')

    default_rig_file = os.path.join(ws_root, "config", "carla916.json")
    rig_file_arg = DeclareLaunchArgument(
        "rig_file",
        default_value=default_rig_file,
        description="Path to the CARLA sensor rig JSON",
    )

    carla_config = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(
                get_package_share_directory("carla_bridge_bringup"),
                "scripts",
                "config_carla.py",
            ),
            "--host", LaunchConfiguration("host"),
            "--port", LaunchConfiguration("port"),
             "-f", LaunchConfiguration("rig_file"),
            "--map", LaunchConfiguration("map"),
            # "-a",
        ],
        output="screen",
        emulate_tty=True,
    )

    carla_vehicle_speed_publisher_node = Node(
        package='carla_vehicle_speed_publisher',
        executable='carla_vehicle_speed_publisher_node',
        name='carla_vehicle_speed_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'host': LaunchConfiguration('host'),
                     'port': LaunchConfiguration('port'),
        }]
    )

    carla_control_publisher_node = Node(
        package='carla_control_publisher',
        executable='carla_control_publisher_node',
        name='carla_control_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    def on_output(event):
        text = event.text.decode(errors="replace")
        if "Running..." in text:
            return [carla_vehicle_speed_publisher_node, carla_control_publisher_node]
        return []

    start_nodes_when_ready = RegisterEventHandler(
        OnProcessIO(
            target_action=carla_config,
            on_stdout=on_output,
            on_stderr=on_output,
        )
    )

    return LaunchDescription([
        host_arg, port_arg, map_arg, rig_file_arg,
        carla_config,
        start_nodes_when_ready,
    ])
