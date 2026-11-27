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

    # "0.9.16" or "1.0" picks config/carla_<version>.json and the map; empty = ask the server.
    carla_version_arg = DeclareLaunchArgument(
        "carla_version",
        default_value="",
        description="CARLA version: 0.9.16 or 1.0 (empty = detect from the server)",
    )

    # Empty: CARLA_MAP env, else the rig JSON "map", else Town04 (0.9.x) / Town04_Opt (1.0).
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="CARLA map to load, e.g. Town04 / Town04_Opt (empty = from the rig JSON)",
    )

    # Empty: config_carla.py picks config/carla_0.9.16.json or config/carla_1.0.json
    # from the CARLA server version.
    rig_file_arg = DeclareLaunchArgument(
        "rig_file",
        default_value="",
        description="Path to the CARLA sensor rig JSON (empty = chosen by server version)",
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
            "--carla-version", LaunchConfiguration("carla_version"),
            "-m", LaunchConfiguration("map"),
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
        host_arg, port_arg, carla_version_arg, map_arg, rig_file_arg,
        carla_config,
        start_nodes_when_ready,
    ])
