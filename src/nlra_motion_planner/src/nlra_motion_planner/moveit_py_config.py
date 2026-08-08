"""Build the parameter dict for a MoveItPy instance from the MoveIt config package."""
import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory


def build_moveit_config_dict():
    """Return the full parameter dict MoveItPy needs (URDF, SRDF, kinematics, pipelines)."""
    agilus_desc_share = get_package_share_directory("agilus_robotiq_description")
    agilus_moveit_share = get_package_share_directory("agilus_robotiq_moveit_config")

    xacro_file = os.path.join(agilus_desc_share, "urdf", "agilus_robotiq.urdf.xacro")
    srdf_file = os.path.join(agilus_moveit_share, "srdf", "agilus_robotiq.srdf")
    kinematics_file = os.path.join(agilus_moveit_share, "config", "kinematics.yaml")
    ompl_file = os.path.join(agilus_moveit_share, "config", "ompl_planning.yaml")

    robot_description = subprocess.run(
        ["xacro", xacro_file], capture_output=True, text=True, check=True
    ).stdout

    with open(srdf_file, "r") as f:
        robot_description_semantic = f.read()

    with open(kinematics_file, "r") as f:
        robot_description_kinematics = yaml.safe_load(f)

    with open(ompl_file, "r") as f:
        ompl_config = yaml.safe_load(f)["ompl"]

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
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": robot_description_kinematics,
        "planning_pipelines": {"pipeline_names": ["ompl"]},
        "ompl": ompl_config,
    }
