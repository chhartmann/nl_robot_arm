"""MoveIt 2 motion planner wrapper for NL Robot Arm.

Provides high-level planning and execution interface using moveit_py.
"""
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit import MoveItPy
from moveit.planning import PlanningComponent

from nlra_motion_planner.moveit_py_config import build_moveit_config_dict


def _quaternion_multiply(q1: List[float], q2: List[float]) -> List[float]:
    """Multiply two quaternions q1 * q2. Input format: [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


class MotionPlanner(Node):
    """High-level motion planner using MoveIt 2 Python API."""

    def __init__(self):
        super().__init__("nlra_motion_planner")

        # MoveItPy instance (its own node gets the pipeline parameters via config_dict)
        self._moveit = MoveItPy(
            node_name="nlra_motion_planner_moveit",
            config_dict=build_moveit_config_dict(),
        )
        self._moveit_arm = self._moveit.get_planning_component("manipulator")

        # TF buffer for frame transformations
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Current joint state
        self._joint_state = None
        self._js_lock = threading.Lock()
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        # Arm controller action client for execution
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

        self.get_logger().info("Motion planner initialized")

    def _on_joint_state(self, msg: JointState):
        with self._js_lock:
            self._joint_state = msg

    def _get_current_joint_positions(self) -> List[float]:
        """Get current joint positions for the 6 arm joints."""
        with self._js_lock:
            if self._joint_state is None:
                return [0.0, -1.5708, 1.5, 0.0, 0.0, 0.0]
            try:
                indices = [
                    self._joint_state.name.index("joint_1"),
                    self._joint_state.name.index("joint_2"),
                    self._joint_state.name.index("joint_3"),
                    self._joint_state.name.index("joint_4"),
                    self._joint_state.name.index("joint_5"),
                    self._joint_state.name.index("joint_6"),
                ]
                return [self._joint_state.position[i] for i in indices]
            except ValueError:
                return [0.0, -1.5708, 1.5, 0.0, 0.0, 0.0]

    # ============================================================
    # Public planning interface
    # ============================================================

    def plan_to_pose(
        self,
        target_pose: PoseStamped,
        velocity_scaling: float = 0.2,
        acceleration_scaling: float = 0.2,
        planning_time: float = 10.0,
    ) -> Optional[RobotTrajectory]:
        """Plan a collision-free path to a target pose (absolute Cartesian).

        Args:
            target_pose: Target pose in base_link frame
            velocity_scaling: Velocity scaling factor (0..1)
            acceleration_scaling: Acceleration scaling factor (0..1)
            planning_time: Max planning time in seconds

        Returns:
            RobotTrajectory or None if planning failed
        """
        self.get_logger().info(f"Planning to pose: {target_pose.pose.position}")

        # Set goal
        self._moveit_arm.set_goal_state(pose_stamped_msg=target_pose)

        # Configure planning
        plan_result = self._moveit_arm.plan(
            single_plan_parameters=[
                "RRTConnectkConfigDefault",
            ],
            max_velocity_scaling_factor=velocity_scaling,
            max_acceleration_scaling_factor=acceleration_scaling,
            planning_time=planning_time,
        )

        if plan_result and plan_result.trajectory:
            self.get_logger().info("Planning succeeded")
            return plan_result.trajectory
        else:
            self.get_logger().error("Planning failed")
            return None

    def plan_cartesian_path(
        self,
        waypoints: List[Pose],
        eef_step: float = 0.01,
        jump_threshold: float = 0.0,
        velocity_scaling: float = 0.2,
        acceleration_scaling: float = 0.2,
    ) -> Tuple[Optional[RobotTrajectory], float]:
        """Plan a Cartesian path through waypoints.

        Args:
            waypoints: List of waypoints in base_link frame
            eef_step: Max step for end-effector translation
            jump_threshold: Jump threshold for IK solutions
            velocity_scaling: Velocity scaling factor
            acceleration_scaling: Acceleration scaling factor

        Returns:
            (RobotTrajectory, fraction) - fraction is how much of path was planned
        """
        self.get_logger().info(f"Planning Cartesian path with {len(waypoints)} waypoints")

        start_state = self._moveit_arm.get_start_state()
        current_positions = self._get_current_joint_positions()
        start_state.joint_positions = dict(
            zip(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"], current_positions)
        )

        # Compute Cartesian path
        fraction, trajectory = self._moveit_arm.compute_cartesian_path(
            waypoints=waypoints,
            eef_step=eef_step,
            jump_threshold=jump_threshold,
            start_state=start_state,
        )

        if trajectory and fraction > 0.9:
            self.get_logger().info(f"Cartesian planning succeeded (fraction={fraction:.2f})")
            return trajectory, fraction
        else:
            self.get_logger().warn(f"Cartesian planning partial: fraction={fraction:.2f}")
            return trajectory, fraction

    def plan_to_joint_target(
        self,
        joint_positions: List[float],
        velocity_scaling: float = 0.2,
        acceleration_scaling: float = 0.2,
        planning_time: float = 10.0,
    ) -> Optional[RobotTrajectory]:
        """Plan to a joint-space target (absolute positions).

        Args:
            joint_positions: 6 joint positions in radians
            velocity_scaling: Velocity scaling factor
            acceleration_scaling: Acceleration scaling factor
            planning_time: Max planning time

        Returns:
            RobotTrajectory or None
        """
        self.get_logger().info(f"Planning to joint target: {joint_positions}")

        goal_state = self._moveit_arm.get_start_state()
        goal_state.joint_positions = dict(
            zip(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"], joint_positions)
        )
        self._moveit_arm.set_goal_state(joint_state=goal_state)

        plan_result = self._moveit_arm.plan(
            single_plan_parameters=["RRTConnectkConfigDefault"],
            max_velocity_scaling_factor=velocity_scaling,
            max_acceleration_scaling_factor=acceleration_scaling,
            planning_time=planning_time,
        )

        if plan_result and plan_result.trajectory:
            self.get_logger().info("Joint planning succeeded")
            return plan_result.trajectory
        else:
            self.get_logger().error("Joint planning failed")
            return None

    def plan_relative_cartesian(
        self,
        translation: Vector3,
        rotation_delta: Optional[Quaternion] = None,
        reference_frame: str = "tool0",
        velocity_scaling: float = 0.2,
        acceleration_scaling: float = 0.2,
    ) -> Tuple[Optional[RobotTrajectory], float]:
        """Plan a relative Cartesian move from current pose.

        Args:
            translation: Translation delta (x, y, z)
            rotation_delta: Optional orientation delta (quaternion)
            reference_frame: Frame for the delta ("tool0" or "base_link")
            velocity_scaling: Velocity scaling factor
            acceleration_scaling: Acceleration scaling factor

        Returns:
            (RobotTrajectory, fraction)
        """
        # Get current end-effector pose
        current_pose = self._get_current_ee_pose(reference_frame)
        if current_pose is None:
            self.get_logger().error("Failed to get current EE pose")
            return None, 0.0

        # Build target pose
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + translation.x
        target_pose.position.y = current_pose.position.y + translation.y
        target_pose.position.z = current_pose.position.z + translation.z

        if rotation_delta is not None:
            # Apply rotation delta (quaternion multiplication)
            q_current = [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            ]
            q_delta = [
                rotation_delta.x,
                rotation_delta.y,
                rotation_delta.z,
                rotation_delta.w,
            ]
            q_target = _quaternion_multiply(q_current, q_delta)
            target_pose.orientation.x = q_target[0]
            target_pose.orientation.y = q_target[1]
            target_pose.orientation.z = q_target[2]
            target_pose.orientation.w = q_target[3]
        else:
            target_pose.orientation = current_pose.orientation

        # Plan Cartesian path to target
        waypoints = [target_pose]
        return self.plan_cartesian_path(
            waypoints=waypoints,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
        )

    def _get_current_ee_pose(self, frame: str = "tool0") -> Optional[Pose]:
        """Get current end-effector pose in base_link frame."""
        try:
            if frame == "tool0":
                # Get transform from base_link to tool0
                transform = self._tf_buffer.lookup_transform(
                    "base_link", "tool0", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
                )
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                return pose
            else:
                # For other frames, use FK
                joint_positions = self._get_current_joint_positions()
                # MoveIt can compute FK - use planning scene
                from moveit_msgs.srv import GetPositionFK
                # Simplified: return identity for now
                return None
        except Exception as e:
            self.get_logger().warn(f"Failed to get EE pose: {e}")
            return None

    # ============================================================
    # Execution interface
    # ============================================================

    def execute_trajectory(
        self,
        trajectory: RobotTrajectory,
        timeout: float = 30.0,
    ) -> bool:
        """Execute a trajectory via the arm controller.

        Args:
            trajectory: RobotTrajectory to execute
            timeout: Max execution time

        Returns:
            True if execution succeeded
        """
        # Convert to FollowJointTrajectory goal
        from control_msgs.action import FollowJointTrajectory
        from builtin_interfaces.msg import Duration

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory

        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm controller not available")
            return False

        send_future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error("Goal send timed out")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                goal_handle.cancel_goal_async()
                self.get_logger().error("Execution timeout")
                return False
            time.sleep(0.1)

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory executed successfully")
            return True
        else:
            self.get_logger().error(f"Execution failed: {result.error_string}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
