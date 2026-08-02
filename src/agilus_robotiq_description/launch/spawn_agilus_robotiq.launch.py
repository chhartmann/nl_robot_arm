#!/usr/bin/env python3
"""
Spawn KUKA Agilus KR 16 R1100-3 + Robotiq 2F-85 in Gazebo Harmonic with ros2_control.

Headless-friendly: pass gui:=false (default true) to run the gz server only
(useful on machines without a display / for CI smoke tests).

Launches:
  * robot_state_publisher (robot_description from xacro)
  * gz_sim (server; +gui when gui:=true)
  * ros_gz spawn of the robot
  * controller spawners: joint_state_broadcaster, arm_controller, gripper_controller
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
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
    # Runs as its own process (chained to gz start via OnProcessExit below):
    # a pkill in the same cmdline as gz would match its own wrapper. The
    # "gz [s]im" pattern avoids matching this reset wrapper's own cmdline.
    reset_sim = ExecuteProcess(
        cmd=["bash", "-c", "pkill -f 'gz [s]im' || true"],
        output="screen",
    )

    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "3", "--render-engine", render_engine,
             "--gui-config", gui_config, world_file],
        output="screen", condition=IfCondition(gui),
    )
    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "3", world_file],
        output="screen", condition=UnlessCondition(gui),
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

    # gz-sim8's DetachableJoint welds the child model to the parent link on the
    # first physics tick after the robot spawns: its attachRequested flag
    # defaults to true and <suppress_initial_attach> is NOT supported by the
    # plugin (silently ignored). Without this, red_cube is glued to the
    # fingertip from t=0 and gets dragged off the table by the first arm move.
    # A few seconds after spawn: detach it and restore its tabletop pose so
    # the pick flow owns attach/detach.
    cube_release = ExecuteProcess(
        cmd=["bash", "-c",
             "sleep 3; "
             "gz topic -t /gripper/detach -m gz.msgs.Empty -p ''; "
             "gz service -s /world/scene/set_pose --reqtype gz.msgs.Pose "
             "--reptype gz.msgs.Boolean "
             "--req 'name: \"red_cube\", position: {x: 0.55, y: 0.15, "
             "z: 0.425}, orientation: {w: 1.0}}'"],
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
        DeclareLaunchArgument("render_engine", default_value="ogre",
                              description="gz sim render engine (ogre2 default upstream; "
                                          "use ogre on drivers/UTM that only support Ogre1)"),
        DeclareLaunchArgument("world", default_value="empty_bullet.sdf",
                              description="World file in this package's worlds/ dir"),
        rsp,
        reset_sim,
        spawn,
        clock_bridge,
        RegisterEventHandler(OnProcessExit(target_action=reset_sim,
                                           on_exit=[gz_gui, gz_headless])),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, grip])),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[cube_release])),
    ])
