"""Launch the web UI: rosbridge_server + web_server."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    web_port = LaunchConfiguration("web_port")
    rb_port = LaunchConfiguration("rosbridge_port")

    # rosbridge is launched via nlra_bringup's run_rosbridge.sh wrapper: its
    # own `#!/usr/bin/env python3` shebang strips DYLD_LIBRARY_PATH on macOS,
    # breaking its action clients (see nlra_bringup/scripts/run_rosbridge.sh).
    rosbridge_server = Node(
        package="nlra_bringup",
        executable="run_rosbridge.sh",
        name="rosbridge_websocket",
        output="screen",
        arguments=["--ros-args", "-p", ["port:=", rb_port]],
    )

    web_server = Node(
        package="nlra_web_ui",
        executable="web_server",
        output="screen",
        parameters=[{"port": web_port, "rosbridge_port": rb_port}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        rosbridge_server,
        TimerAction(period=2.0, actions=[web_server]),
    ])
