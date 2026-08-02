#!/usr/bin/env python3
"""Launch the skill servers (expects the Gazebo sim to be running)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nlra_skills",
            executable="skill_servers",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
