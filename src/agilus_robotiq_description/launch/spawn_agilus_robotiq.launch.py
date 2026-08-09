#!/usr/bin/env python3
"""
Spawn KUKA Agilus KR 16 R1100-3 + Robotiq 2F-85 in Gazebo Harmonic with ros2_control.

Headless-friendly: pass gui:=false (default true) to run the gz server only
(useful on machines without a display / for CI smoke tests).

macOS (and Windows) cannot run the gz server and GUI in one process, so they
are launched separately: the server always runs with `-s`, and the GUI (when
gui:=true) connects to it with `-g` after the robot is spawned (which is
guaranteed to be after the server+world are up).

Launches:
  * robot_state_publisher (robot_description from xacro)
  * gz_sim server (`-s`)
  * gz_sim GUI (`-g`, when gui:=true) — macOS/Windows compatible
  * ros_gz spawn of the robot
  * controller spawners: joint_state_broadcaster, arm_controller, gripper_controller
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "agilus_robotiq_description"
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")

    controller_config = PathJoinSubstitution(
        [FindPackageShare(pkg), "config", "agilus_robotiq_controllers.yaml"]
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(pkg), "urdf", "agilus_robotiq.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [FindExecutable(name="xacro"), " ", xacro_file,
                 " controller_config:=", controller_config]
            ),
            value_type=str,
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare(pkg), "worlds", LaunchConfiguration("world")]
    )
    gui_config = PathJoinSubstitution(
        [FindPackageShare(pkg), "config", "gui_front.config"]
    )

    # Always reset: kill any leftover gz sim server from a previous run so the
    # simulation always starts fresh (world state, robot pose, attach state).
    # Also kill leftover /clock bridges: every launch starts its own
    # parameter_bridge for /clock, and a stale one from a previous run keeps
    # publishing the clock. Two+ interleaved /clock streams are not strictly
    # monotonic — subscribers see the merged timestamps jump backward, which
    # makes every use_sim_time tf2 buffer clear and flood the log with
    # "Detected jump back in time. Clearing TF buffer."
    # Runs as its own process (chained to gz start via OnProcessExit below):
    # a pkill in the same cmdline as gz would match its own wrapper. The
    # "gz [s]im" pattern avoids matching this reset wrapper's own cmdline.
    reset_sim = ExecuteProcess(
        cmd=["bash", "-c", "pkill -f 'gz [s]im' || true; "
                           "pkill -f '[p]arameter_bridge.*/clock' || true"],
        output="screen",
    )

    # On macOS (and Windows) gz sim cannot run server + GUI in one process
    # (gazebosim/gz-sim#44), so they are split here: the server always runs
    # with -s, and the GUI (gui:=true) connects to the running server with -g.
    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "3", world_file],
        output="screen",
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g", "-v", "3", "--render-engine", render_engine,
             "--gui-config", gui_config, world_file],
        output="screen", condition=IfCondition(gui),
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "agilus_robotiq", "-z", "0.0"],
    )

    # Bridge the Gazebo /clock to ROS so use_sim_time works (controllers need it).
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    grip = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Run Gazebo GUI (false = headless server only)"),
        DeclareLaunchArgument("render_engine", default_value="ogre2",
                              description="gz sim render engine (ogre2 works on macOS/Apple "
                                          "GPU; use ogre on drivers/UTM that only support Ogre1)"),
        DeclareLaunchArgument("world", default_value="empty_bullet.sdf",
                              description="World file in this package's worlds/ dir"),
        rsp,
        reset_sim,
        spawn,
        # The /clock bridge starts only after reset_sim finishes, so the reset
        # above never kills the bridge this launch just started.
        RegisterEventHandler(OnProcessExit(target_action=reset_sim,
                                           on_exit=[gz_server, clock_bridge])),
        # The GUI (macOS/Windows) attaches to the running server, so start it
        # only once the robot is spawned — that guarantees server + world are up.
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[jsb, gz_gui])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, grip])),
    ])
