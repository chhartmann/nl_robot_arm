"""Full NL-robot-arm stack in one launch file.

  ros2 launch nlra_bringup nlra.launch.py            # full stack, headless sim
  ros2 launch nlra_bringup nlra.launch.py gui:=true  # with Gazebo GUI
  ros2 launch nlra_bringup nlra.launch.py nl:=false  # without the NL interface

Order matters: sim first, then (delayed) bridges + world model + move_group +
motion_planner + skills + orchestrator + NL interface + web UI. Delays are
conservative for slow hosts; each node is independently restartable at runtime.

The NL web UI is served at http://localhost:8080 and connects to rosbridge
on ws://localhost:9090. Open the browser to control the robot.
"""
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    nl = LaunchConfiguration("nl")

    # Use get_package_share_directory to avoid PathJoinSubstitution issues
    agilus_desc_share = get_package_share_directory("agilus_robotiq_description")
    agilus_moveit_share = get_package_share_directory("agilus_robotiq_moveit_config")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(agilus_desc_share, "launch", "spawn_agilus_robotiq.launch.py")),
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

    # MoveIt move_group (starts after sim is up)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(agilus_moveit_share, "launch", "move_group.launch.py")))

    world_model = Node(package="nlra_world_model", executable="world_model",
                       output="screen",
                       parameters=[{"use_sim_time": True}])
    # NOTE: no standalone motion_planner node here — the skill servers embed
    # their own MotionPlanner instance (same MoveItPy). A second standalone
    # node would only duplicate the "nlra_motion_planner_moveit" node name
    # and spin an unused MoveItPy.
    skills = Node(package="nlra_skills", executable="skill_servers",
                  output="screen",
                  parameters=[{"use_sim_time": True}])
    orchestrator = Node(package="nlra_orchestrator", executable="orchestrator",
                        output="screen",
                        parameters=[{"use_sim_time": True}])
    nl_interface = Node(package="nlra_nl_interface", executable="nl_interface",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                        condition=IfCondition(nl))

    # Web UI: rosbridge websocket server + static file server. The rosbridge
    # server is launched through run_rosbridge.sh because its own
    # `#!/usr/bin/env python3` shebang strips DYLD_LIBRARY_PATH on macOS,
    # which would break its action clients (see scripts/run_rosbridge.sh).
    rosbridge_server = Node(
        package="nlra_bringup",
        executable="run_rosbridge.sh",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090}],
        condition=IfCondition(nl),
    )
    web_server = Node(
        package="nlra_web_ui",
        executable="web_server",
        output="screen",
        parameters=[{"port": 8080, "rosbridge_port": 9090}],
        condition=IfCondition(nl),
    )

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
            "pkill -TERM -f '[n]lra_web_ui/web_server' || true",
            "pkill -TERM -f '[r]osbridge_websocket' || true",
            "pkill -TERM -f '[m]ove_group' || true",
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
        TimerAction(period=30.0, actions=[world_model, move_group]),
        TimerAction(period=40.0, actions=[skills]),
        TimerAction(period=45.0, actions=[orchestrator]),
        TimerAction(period=50.0, actions=[nl_interface, rosbridge_server]),
        TimerAction(period=55.0, actions=[web_server]),
    ])
