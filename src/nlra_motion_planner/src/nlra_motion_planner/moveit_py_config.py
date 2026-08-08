"""Build the parameter dict for a MoveItPy instance from the MoveIt config package."""
import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory


def build_moveit_config_dict():
    """Return the full parameter dict MoveItPy needs (URDF, SRDF, kinematics, pipelines).

    Structure follows the official moveit_py motion_planning_python_api
    tutorial config: planning_scene_monitor_options + planning_pipelines
    (pipeline_names) + a plan_request_params block (loaded by
    PlanRequestParameters(node, "plan_request_params")) + one block per
    pipeline name with the OMPL config.
    """
    agilus_desc_share = get_package_share_directory("agilus_robotiq_description")
    agilus_moveit_share = get_package_share_directory("agilus_robotiq_moveit_config")

    xacro_file = os.path.join(agilus_desc_share, "urdf", "agilus_robotiq.urdf.xacro")
    srdf_file = os.path.join(agilus_moveit_share, "srdf", "agilus_robotiq.srdf")
    kinematics_file = os.path.join(agilus_moveit_share, "config", "kinematics.yaml")
    ompl_file = os.path.join(agilus_moveit_share, "config", "ompl_planning.yaml")
    joint_limits_file = os.path.join(agilus_moveit_share, "config", "joint_limits.yaml")

    robot_description = subprocess.run(
        ["xacro", xacro_file], capture_output=True, text=True, check=True
    ).stdout

    with open(srdf_file, "r") as f:
        robot_description_semantic = f.read()

    with open(kinematics_file, "r") as f:
        robot_description_kinematics = yaml.safe_load(f)

    with open(ompl_file, "r") as f:
        ompl_config = yaml.safe_load(f)["ompl"]

    with open(joint_limits_file, "r") as f:
        robot_description_planning = yaml.safe_load(f)

    # MoveItCpp expects the pipeline plugin list under `planning_plugins` (plural)
    # and per-group `planner_configs` as a list of planner ids, with the planner
    # parameter map at the top level of the pipeline namespace.
    manipulator = ompl_config.get("manipulator", {})
    planner_map = manipulator.get("planner_configs", {})
    if not isinstance(planner_map, dict):
        planner_map = {}
    planner_ids = manipulator.get("planner_ids")
    if not isinstance(planner_ids, list):
        planner_ids = list(planner_map.keys())
    default_planner = manipulator.get("default_planner_config", "RRTConnectkConfigDefault")

    ompl_config["planning_plugins"] = [ompl_config["planning_plugin"]]
    ompl_config["planner_configs"] = planner_map
    manipulator["planner_configs"] = list(planner_ids)
    manipulator["default_planner_config"] = default_planner
    ompl_config["manipulator"] = manipulator

    return {
        # Gazebo sim runs on /clock; MoveItPy's internal node must follow it
        # or the current_state_monitor declares every joint-state update stale
        # ("Unable to configure planning scene monitor" -> exit 1).
        "use_sim_time": True,
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": robot_description_kinematics,
        # velocity/acceleration/jerk limits; required by AddTimeOptimalParameterization
        "robot_description_planning": robot_description_planning,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {"pipeline_names": ["ompl"]},
        # moveit_py's PlanRequestParameters(moveit_cpp, "plan_request_params")
        # reads keys under the DOUBLE namespace plan_request_params.plan_request_params.*
        "plan_request_params": {
            "plan_request_params": {
                "planning_attempts": 3,
                "planning_pipeline": "ompl",
                "max_velocity_scaling_factor": 0.2,
                "max_acceleration_scaling_factor": 0.2,
                "planning_time": 10.0,
                # Controller integration overshoots a joint that ends exactly
                # AT its limit (observed: joint_4 -3.49081 vs limit -3.4907,
                # 0.00011 rad). CheckStartStateBounds reads this tolerance from
                # plan_request_params, not from the OMPL pipeline config.
                "start_state_max_bounds_error": 0.1,
                # jazzy CheckStartStateBounds calls satisfiesBounds() with NO
                # tolerance; fix_start_state=true clamps a slightly-out-of-
                # bounds start state back onto the limit instead of failing.
                "fix_start_state": True,
            }
        },
        "ompl": ompl_config,
    }
