"""Launch the VisionPilot<->CARLA bridge: control relay + ego telemetry.

  carla_control_relay  /vehicle/{steering,throttle}_cmd (+ /vehicle/speed)
                        -> carla_msgs/CarlaEgoVehicleControl
  ego_telemetry        CARLA PythonAPI (TCP host:2000)
                        -> /vehicle/speed + latched /carla_bridge/max_steer_angle

Both run in the SAME container/DDS domain as VisionPilot — ROS never crosses the
host/container boundary (host<->container DDS delivery is unreliable). ego_telemetry
needs the `carla` wheel on PYTHONPATH (drive.sh mounts it). Works identically against
CARLA 0.9.16 and 0.10 (same native --ros2 topic names).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name, default, type, description) — mirrors the parameters carla_control_relay declares.
_ARGS = [
    ("steer_gain", "1.0", float, "Proportional gain on the VisionPilot steer prediction."),
    (
        "max_steer",
        "1.2217",
        float,
        "Fallback front-wheel steer limit [rad]; the latched /carla_bridge/max_steer_angle "
        "published by ros_carla_config.py overrides it with the spawned ego's real limit.",
    ),
    (
        "speed_topic",
        "/vehicle/speed",
        str,
        "Ego-speed topic [m/s] — shared with VisionPilot; feeds the bridge watchdog.",
    ),
    (
        "control_topic",
        "/carla/hero/vehicle_control_cmd",
        str,
        "CarlaEgoVehicleControl output topic (CARLA native --ros2 ego input).",
    ),
    (
        "control_rate_hz",
        "100.0",
        float,
        "Relay re-emit rate [Hz]. 100 keeps loop delay low; 20 fed the limit cycle.",
    ),
    (
        "throttle_ref_accel",
        "2.0",
        float,
        "Accel [m/s^2] that maps to throttle=1.0 (feed-forward slope).",
    ),
    ("brake_ref_accel", "3.0", float, "Decel [m/s^2] that maps to brake=1.0 (feed-forward slope)."),
    (
        "accel_deadband",
        "0.1",
        float,
        "|accel| below this [m/s^2] -> coast (no throttle, no brake).",
    ),
    (
        "aeb_accel_threshold",
        "-2.0",
        float,
        "Accel command below this [m/s^2] forces full brake (planner AEB override).",
    ),
    (
        "vp_cmd_timeout",
        "1.0",
        float,
        "Watchdog: hold stopped if no VisionPilot command within this many seconds.",
    ),
    (
        "speed_timeout",
        "1.0",
        float,
        "Watchdog: hold stopped if no ego speed within this many seconds.",
    ),
]

# ego_telemetry parameters (CARLA PythonAPI side).
_TELEMETRY_ARGS = [
    ("carla_host", "localhost", str, "CARLA server host (TCP, PythonAPI)."),
    ("carla_port", "2000", int, "CARLA server port."),
    ("role_name", "hero", str, "role_name of the ego to publish telemetry for."),
    ("rate_hz", "20.0", float, "Ego speed publish rate [Hz]."),
]


def generate_launch_description():
    telemetry_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=type_)
        for name, _, type_, _ in _TELEMETRY_ARGS
    }
    telemetry_params["speed_topic"] = ParameterValue(
        LaunchConfiguration("speed_topic"), value_type=str
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(name, default_value=default, description=desc)
            for name, default, _, desc in _ARGS + _TELEMETRY_ARGS
        ]
        + [
            Node(
                package="visionpilot_carla_bridge",
                executable="carla_control_relay",
                name="carla_control_relay",
                output="screen",
                parameters=[
                    {
                        name: ParameterValue(LaunchConfiguration(name), value_type=type_)
                        for name, _, type_, _ in _ARGS
                    }
                ],
            ),
            Node(
                package="visionpilot_carla_bridge",
                executable="ego_telemetry",
                name="carla_ego_telemetry",
                output="screen",
                parameters=[telemetry_params],
            ),
        ]
    )
