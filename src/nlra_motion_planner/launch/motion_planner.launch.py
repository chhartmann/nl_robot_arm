"""Launch the motion planner node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nlra_motion_planner",
            executable="motion_planner",
            name="nlra_motion_planner",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
