"""MoveIt 2 motion planner wrapper for NL Robot Arm.

Provides high-level planning and execution interface using moveit_py.

API NOTE (verified against moveit2 jazzy moveit_py pybindings):
- PlanningComponent.set_goal_state(pose_stamped_msg=...) REQUIRES pose_link
  ("Must specify both message and corresponding link" otherwise).
- PlanningComponent.plan() does NOT take max_velocity_scaling_factor /
  max_acceleration_scaling_factor / planning_time kwargs. Scaling and planner
  id go on a PlanRequestParameters object (constructed per-plan), which is
  passed as single_plan_parameters.
- PlanningComponent has NO compute_cartesian_path binding in Jazzy. Relative
  Cartesian moves are implemented as absolute pose goals computed from the
  current EE pose via TF.
"""
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from moveit import MoveItPy
from moveit.planning import PlanRequestParameters
from moveit.utils import create_params_file_from_dict

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


def _rotate_vector(v: Vector3, q: Quaternion) -> List[float]:
    """Rotate vector v by quaternion q (Hamilton product: q * v * q^-1)."""
    qv = [q.x, q.y, q.z, q.w]
    vin = [v.x, v.y, v.z, 0.0]
    # q * v
    t = _quaternion_multiply(qv, vin)
    # q * v * q^-1  (q^-1 = [-x, -y, -z, w])
    t = _quaternion_multiply(t, [-qv[0], -qv[1], -qv[2], qv[3]])
    return t[:3]


class MotionPlanner(Node):
    """High-level motion planner using MoveIt 2 Python API."""

    def __init__(self):
        super().__init__("nlra_motion_planner")

        # MoveItPy instance. use_sim_time cannot be passed via config_dict
        # (moveit2#2220/#2940: node-scoped params file makes rclcpp abort on
        # 'qos_overrides./clock.subscription.durability could not be set').
        # Workaround: write the dict as a params file under the GLOBAL /**
        # namespace and hand it over as launch_params_filepaths.
        self._moveit = MoveItPy(
            node_name="nlra_motion_planner_moveit",
            launch_params_filepaths=[
                create_params_file_from_dict(build_moveit_config_dict(), "/**")
            ],
        )
        self._moveit_arm = self._moveit.get_planning_component("manipulator")
        self._moveit_arm.set_start_state_to_current_state()

        # TF buffer for frame transformations. Bind it to this node so its
        # clock follows /clock (use_sim_time) instead of a plain system-time
        # Clock. The transforms on /tf are stamped with sim time; a buffer on
        # wall-clock time sees those stamps as a "jump back in time" whenever
        # wall time and sim time diverge, clearing the buffer and flooding the
        # log with "Detected jump back in time. Clearing TF buffer."
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Current joint state (for FK of the current EE pose fallback)
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
    # Planning parameter helpers
    # ============================================================

    def _make_plan_params(self, velocity_scaling: float,
                          acceleration_scaling: float,
                          planning_time: float,
                          planner_id: str = "RRTConnectkConfigDefault"):
        """Build a PlanRequestParameters for one plan() call."""
        params = PlanRequestParameters(self._moveit, "plan_request_params")
        params.planner_id = planner_id
        params.planning_pipeline = "ompl"
        params.planning_attempts = 3
        params.planning_time = planning_time
        params.max_velocity_scaling_factor = velocity_scaling
        params.max_acceleration_scaling_factor = acceleration_scaling
        return params

    def _plan(self, velocity_scaling, acceleration_scaling, planning_time):
        """plan() with a per-call PlanRequestParameters (Jazzy API)."""
        params = self._make_plan_params(velocity_scaling, acceleration_scaling,
                                        planning_time)
        return self._moveit_arm.plan(single_plan_parameters=params)

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

        The pose is transformed into the MoveIt planning frame (base_link)
        first. Pose goals need the tip link (tool0) explicitly.

        Returns:
            RobotTrajectory or None if planning failed
        """
        self.get_logger().info(f"Planning to pose: {target_pose.pose.position}")

        # Transform the goal into the MoveIt planning frame (base_link) when it
        # arrives in another frame (world-model grasp poses are in "world").
        # The fixed virtual joint world->base_link is identity in this sim, so a
        # failed transform is not fatal — the coordinates coincide anyway.
        if target_pose.header.frame_id not in ("", "base_link"):
            try:
                target_pose = self._tf_buffer.transform(
                    target_pose, "base_link",
                    timeout=rclpy.duration.Duration(seconds=1.0))
            except Exception as e:
                self.get_logger().warn(
                    f"pose frame transform {target_pose.header.frame_id}->"
                    f"base_link failed ({e}); using pose as-is")

        # Set goal with the required pose_link argument.
        self._moveit_arm.set_goal_state(pose_stamped_msg=target_pose,
                                        pose_link="tool0")

        plan_result = self._plan(velocity_scaling, acceleration_scaling,
                                 planning_time)

        if plan_result and plan_result.trajectory:
            self.get_logger().info("Planning succeeded")
            return plan_result.trajectory
        else:
            self.get_logger().error("Planning failed")
            return None

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

        from moveit.core import RobotState
        goal_state = RobotState(self._moveit.get_robot_model())
        goal_state.joint_positions = dict(
            zip(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
                 "joint_6"], joint_positions)
        )
        self._moveit_arm.set_goal_state(robot_state=goal_state)

        plan_result = self._plan(velocity_scaling, acceleration_scaling,
                                 planning_time)

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

        Jazzy moveit_py has no compute_cartesian_path binding, so the relative
        delta is applied to the current EE pose (TF) and planned as an absolute
        pose goal (fraction is 1.0 on success).

        Args:
            translation: Translation delta (x, y, z)
            rotation_delta: Optional orientation delta (quaternion)
            reference_frame: Frame for the delta ("tool0" or "base_link")
            velocity_scaling: Velocity scaling factor
            acceleration_scaling: Acceleration scaling factor

        Returns:
            (RobotTrajectory, fraction)
        """
        # Get current end-effector pose in base_link
        current_pose = self._get_current_ee_pose("tool0")
        if current_pose is None:
            self.get_logger().error("Failed to get current EE pose")
            return None, 0.0

        # Build target pose
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z
        target_pose.orientation = current_pose.orientation

        if reference_frame == "tool0":
            # Rotate the delta by the tool orientation before adding.
            d = _rotate_vector(translation, current_pose.orientation)
            target_pose.position.x += d[0]
            target_pose.position.y += d[1]
            target_pose.position.z += d[2]
        else:
            target_pose.position.x += translation.x
            target_pose.position.y += translation.y
            target_pose.position.z += translation.z

        if rotation_delta is not None:
            q_current = [current_pose.orientation.x, current_pose.orientation.y,
                         current_pose.orientation.z, current_pose.orientation.w]
            q_delta = [rotation_delta.x, rotation_delta.y, rotation_delta.z,
                       rotation_delta.w]
            q_target = _quaternion_multiply(q_current, q_delta)
            target_pose.orientation.x = q_target[0]
            target_pose.orientation.y = q_target[1]
            target_pose.orientation.z = q_target[2]
            target_pose.orientation.w = q_target[3]

        target_stamped = PoseStamped()
        target_stamped.header.frame_id = "base_link"
        target_stamped.header.stamp = self.get_clock().now().to_msg()
        target_stamped.pose = target_pose

        traj = self.plan_to_pose(target_stamped,
                                 velocity_scaling=velocity_scaling,
                                 acceleration_scaling=acceleration_scaling)
        if traj is None:
            return None, 0.0
        return traj, 1.0

    def _get_current_ee_pose(self, frame: str = "tool0") -> Optional[Pose]:
        """Get current end-effector pose in base_link frame."""
        try:
            if frame == "tool0":
                # Get transform from base_link to tool0
                transform = self._tf_buffer.lookup_transform(
                    "base_link", "tool0", rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                return pose
            else:
                self.get_logger().error(
                    f"Unsupported reference frame '{frame}' (only tool0)")
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
        timeout: float = 120.0,
    ) -> bool:
        """Execute a trajectory via the arm controller.

        Args:
            trajectory: RobotTrajectory to execute
            timeout: Max execution time (s); generous because OMPL paths at
                0.2 velocity scaling can take a while.

        Returns:
            True if execution succeeded
        """
        from control_msgs.action import FollowJointTrajectory

        goal = FollowJointTrajectory.Goal()
        # pybind moveit.core.robot_trajectory.RobotTrajectory -> msg
        goal.trajectory = trajectory.get_robot_trajectory_msg().joint_trajectory

        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm controller not available")
            return False

        send_future = self._arm_client.send_goal_async(goal)
        t0 = time.monotonic()
        while not send_future.done():
            if time.monotonic() - t0 > 10.0:
                self.get_logger().error("Goal send timed out")
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller")
            return False

        # Wait for result (poll; the node's executor does the spinning)
        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        while not result_future.done():
            if time.monotonic() - start_time > timeout:
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
