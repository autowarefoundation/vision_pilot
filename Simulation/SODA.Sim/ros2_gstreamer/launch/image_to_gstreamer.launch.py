from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# GStreamer pipeline templates
_APPSRC = (
    "appsrc name=ros_appsrc is-live=true format=time do-timestamp=true"
)

_PIPELINES = {
    "udp": (
        "{appsrc} ! videoconvert ! "
        "x264enc tune=zerolatency bitrate=4000 speed-preset=veryfast ! "
        "rtph264pay config-interval=1 pt=96 ! "
        "udpsink host={{host}} port={{port}} sync=false"
    ).format(appsrc=_APPSRC),

    "v4l2": (
        "{appsrc} ! videoconvert ! "
        "video/x-raw,format=UYVY ! "
        "v4l2sink device={{device}} sync=false"
    ).format(appsrc=_APPSRC),
}


def _make_node(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    input_topic = LaunchConfiguration("input_topic").perform(context)
    target_fps = LaunchConfiguration("target_fps").perform(context)

    if mode == "udp":
        host = LaunchConfiguration("host").perform(context)
        port = LaunchConfiguration("port").perform(context)
        pipeline = _PIPELINES["udp"].format(host=host, port=port)
    elif mode == "v4l2":
        device = LaunchConfiguration("device").perform(context)
        pipeline = _PIPELINES["v4l2"].format(device=device)
    else:
        raise RuntimeError(f"Unknown mode '{mode}'. Choose 'udp' or 'v4l2'.")

    return [
        Node(
            package="sodasim_gstreamer",
            executable="image_to_gstreamer_node_exe",
            name="image_to_gstreamer",
            parameters=[{
                "input_topic": input_topic,
                "pipeline": pipeline,
                "appsrc_name": "ros_appsrc",
                "target_fps": float(target_fps),
            }],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="udp",
                              description="Output mode: 'udp' or 'v4l2'"),
        DeclareLaunchArgument("input_topic", default_value="/vehicle/camera",
                              description="ROS2 image topic to read from"),
        DeclareLaunchArgument("target_fps", default_value="10.0",
                              description="Target output framerate"),
        # UDP-specific args
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="[udp] Destination host"),
        DeclareLaunchArgument("port", default_value="5600",
                              description="[udp] Destination port"),
        # v4l2-specific args
        DeclareLaunchArgument("device", default_value="/dev/video10",
                              description="[v4l2] V4L2 loopback device path"),

        OpaqueFunction(function=_make_node),
    ])
