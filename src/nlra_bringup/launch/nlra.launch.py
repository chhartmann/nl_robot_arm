"""Full NL-robot-arm stack in one launch file.

  ros2 launch nlra_bringup nlra.launch.py            # full stack, headless sim
  ros2 launch nlra_bringup nlra.launch.py gui:=true  # with Gazebo GUI
  ros2 launch nlra_bringup nlra.launch.py nl:=false  # without the NL interface

Order matters: sim first, then (delayed) bridges + world model + skills +
orchestrator + NL interface. Delays are conservative for slow hosts;
each node is independently restartable at runtime.

The NL interface opens its own chat GUI window when a DISPLAY is present;
on headless hosts it falls back to the inline terminal REPL.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def nl_gui_action(condition):
    """Open the NL chat GUI in its own window."""
    return Node(package="nlra_nl_interface", executable="nl_gui",
                output="screen", condition=condition)


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    nl = LaunchConfiguration("nl")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("agilus_robotiq_description"),
                "launch", "spawn_agilus_robotiq.launch.py"])),
        launch_arguments={"gui": gui, "world": world}.items())

    pose_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="object_pose_bridge",
        arguments=[
            "/model/red_cube/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/blue_box/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/table/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen")

    world_model = Node(package="nlra_world_model", executable="world_model",
                       output="screen")
    skills = Node(package="nlra_skills", executable="skill_servers",
                  output="screen")
    orchestrator = Node(package="nlra_orchestrator", executable="orchestrator",
                        output="screen")
    nl_interface = Node(package="nlra_nl_interface", executable="nl_interface",
                        output="screen",
                        condition=IfCondition(nl))
    nl_gui = nl_gui_action(IfCondition(nl))

    # A previous launch can leave delayed Python nodes orphaned after its
    # parent is interrupted. Duplicate action servers make ActionClient match
    # responses to the wrong process, producing misleading timeouts. Remove
    # only this workspace's stack nodes before starting a fresh instance.
    reset_stack = ExecuteProcess(
        cmd=["bash", "-c", " && ".join([
            "pkill -TERM -f '[n]lra_orchestrator/orchestrator' || true",
            "pkill -TERM -f '[n]lra_skills/skill_servers' || true",
            "pkill -TERM -f '[n]lra_world_model/world_model' || true",
            "pkill -TERM -f '[n]lra_nl_interface/nl_interface' || true",
            "pkill -TERM -f '[n]lra_nl_interface/nl_gui' || true",
            "pkill -TERM -f '[p]arameter_bridge.*object_pose_bridge' || true",
            "sleep 1",
        ])],
        output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("world", default_value="tabletop_scene.sdf"),
        DeclareLaunchArgument("nl", default_value="true"),
        reset_stack,
        sim,
        TimerAction(period=25.0, actions=[pose_bridge]),
        TimerAction(period=30.0, actions=[world_model, skills]),
        TimerAction(period=35.0, actions=[orchestrator]),
        TimerAction(period=40.0, actions=[nl_interface]),
        TimerAction(period=45.0, actions=[nl_gui]),
    ])
