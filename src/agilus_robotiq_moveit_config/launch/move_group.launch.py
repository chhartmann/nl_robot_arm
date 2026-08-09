"""Launch MoveGroup node with planning scene monitor."""
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    agilus_desc_share = get_package_share_directory("agilus_robotiq_description")
    agilus_moveit_share = get_package_share_directory("agilus_robotiq_moveit_config")

    # Load robot description from xacro
    xacro_file = os.path.join(agilus_desc_share, "urdf", "agilus_robotiq.urdf.xacro")
    srdf_file = os.path.join(agilus_moveit_share, "srdf", "agilus_robotiq.srdf")
    kinematics_file = os.path.join(agilus_moveit_share, "config", "kinematics.yaml")
    joint_limits_file = os.path.join(agilus_moveit_share, "config", "joint_limits.yaml")

    # Read robot description
    import subprocess
    robot_description_cmd = ["xacro", xacro_file]
    robot_description_output = subprocess.run(robot_description_cmd, capture_output=True, text=True)
    robot_description = {"robot_description": robot_description_output.stdout}

    # Read SRDF
    with open(srdf_file, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Read kinematics
    import yaml
    with open(kinematics_file, "r") as f:
        robot_description_kinematics = {
            "robot_description_kinematics": yaml.safe_load(f)
        }

    # Read joint limits
    with open(joint_limits_file, "r") as f:
        robot_description_planning = {"robot_description_planning": yaml.safe_load(f)}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "use_sim_time": True,
            },
            {
                "ompl.planning_plugin": "ompl_interface/OMPLPlanner",
                "ompl.planning_plugins": ["ompl_interface/OMPLPlanner"],
                "ompl.request_adapters": [
                    "default_planning_request_adapters/ValidateWorkspaceBounds",
                    "default_planning_request_adapters/CheckStartStateBounds",
                    "default_planning_request_adapters/CheckStartStateCollision",
                    "default_planning_request_adapters/ResolveConstraintFrames",
                ],
            },
        ],
    )

    # Robot state publisher (already started by sim, but ensure it's there)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
    ])
