"""NL Robot Arm — skill layer.

One node hosting all skill action servers. Each skill:
  1. validates preconditions (controllers alive, fresh joint states, limits),
  2. delegates execution to the ros2_control controller actions,
  3. streams phase/progress feedback and returns a typed result.

Skills are deliberately thin, deterministic wrappers — intelligence lives in
the orchestrator, physics lives in Gazebo/ros2_control.
"""
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from nlra_interfaces.action import Grasp, Home, MoveJoints, MoveTo, MoveRelative, MoveAxis, Release
from nlra_motion_planner.motion_planner import MotionPlanner

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
# KUKA Agilus KR 16 R1100-3 limits (rad) from kuka_agilus_support URDF
ARM_LIMITS = [
    (-2.9671, 2.9671),
    (-3.4034, 0.9599),
    (-2.0071, 2.8798),
    (-3.4907, 3.4907),
    (-2.0944, 2.0944),
    (-6.1087, 6.1087),
]
HOME_POSE = [0.0, -1.5708, 1.5, 0.0, 0.0, 0.0]
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_RANGE = (0.0, 0.8)

OK = 0
ERR_PRECONDITION = 1
ERR_PLANNING = 2
ERR_EXECUTION = 3
ERR_CANCELLED = 4

JOINT_STATE_MAX_AGE = 2.0  # s


class SkillServers(Node):
    def __init__(self):
        super().__init__("nlra_skill_servers")
        self._cb = ReentrantCallbackGroup()

        self._js_lock = threading.Lock()
        self._joint_pos = {}
        self._js_stamp = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_js, 10,
                                 callback_group=self._cb)

        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory", callback_group=self._cb)
        self._grip_client = ActionClient(
            self, GripperCommand,
            "/gripper_controller/gripper_cmd", callback_group=self._cb)

        # Motion planner for Cartesian/joint planning
        self._motion_planner = MotionPlanner()

        ActionServer(self, MoveJoints, "skills/move_joints",
                     execute_callback=self._exec_move_joints,
                     goal_callback=self._accept, cancel_callback=self._cancel,
                     callback_group=self._cb)
        ActionServer(self, MoveTo, "skills/move_to",
                     execute_callback=self._exec_move_to,
                     goal_callback=self._accept, cancel_callback=self._cancel,
                     callback_group=self._cb)
        ActionServer(self, MoveRelative, "skills/move_relative",
                     execute_callback=self._exec_move_relative,
                     goal_callback=self._accept, cancel_callback=self._cancel,
                     callback_group=self._cb)
        ActionServer(self, MoveAxis, "skills/move_axis",
                     execute_callback=self._exec_move_axis,
                     goal_callback=self._accept, cancel_callback=self._cancel,
                     callback_group=self._cb)
        ActionServer(self, Grasp, "skills/grasp",
                     execute_callback=self._exec_grasp,
                     goal_callback=self._accept, cancel_callback=self._cancel,
                     callback_group=self._cb)
        ActionServer(self, Release, "skills/release",
                     execute_callback=self._exec_release,
                     goal_callback=self._accept, cancel_callback=self._cancel,
                     callback_group=self._cb)
        ActionServer(self, Home, "skills/home",
                     execute_callback=self._exec_home,
                     goal_callback=self._accept, cancel_callback=self._cancel,
                     callback_group=self._cb)

        self.get_logger().info("skill servers up: move_joints, move_to, move_relative, move_axis, grasp, release, home")

    # ------------------------------------------------------------- helpers
    def _on_js(self, msg: JointState):
        with self._js_lock:
            for n, p in zip(msg.name, msg.position):
                self._joint_pos[n] = p
            self._js_stamp = time.monotonic()

    def _accept(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _joint_states_fresh(self):
        with self._js_lock:
            age = time.monotonic() - self._js_stamp
            have = all(j in self._joint_pos for j in ARM_JOINTS)
        return have and age < JOINT_STATE_MAX_AGE

    def _preconditions(self, need_arm=False, need_gripper=False):
        """Return (ok, message)."""
        if not self._joint_states_fresh():
            return False, "no fresh /joint_states (is the sim running?)"
        if need_arm and not self._arm_client.server_is_ready():
            if not self._arm_client.wait_for_server(timeout_sec=2.0):
                return False, "arm_controller action server unavailable"
        if need_gripper and not self._grip_client.server_is_ready():
            if not self._grip_client.wait_for_server(timeout_sec=2.0):
                return False, "gripper_controller action server unavailable"
        return True, ""

    def _send_arm_trajectory(self, positions, duration_s, goal_handle, fb_msg, fb_phase):
        """Send one-point trajectory, block until done. Returns (code, message)."""
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(duration_s),
                                      nanosec=int((duration_s % 1) * 1e9))
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send = self._arm_client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.05)
        ctrl_handle = send.result()
        if ctrl_handle is None or not ctrl_handle.accepted:
            return ERR_EXECUTION, "arm_controller rejected trajectory"

        result_future = ctrl_handle.get_result_async()
        t0 = time.monotonic()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                ctrl_handle.cancel_goal_async()
                return ERR_CANCELLED, "cancelled"
            fb_msg.phase = fb_phase
            fb_msg.progress = min(0.95, (time.monotonic() - t0) / max(duration_s, 0.1))
            goal_handle.publish_feedback(fb_msg)
            time.sleep(0.1)

        res = result_future.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return ERR_EXECUTION, f"trajectory failed: {res.error_string}"
        return OK, "ok"

    def _send_gripper(self, position, max_effort, goal_handle, fb_msg, fb_phase,
                      timeout_s=15.0, keep_pressing=False):
        """Send gripper command, block until done. Returns (code, message, stalled, reached).

        Position-mode GripperActionController: a goal blocked by an object
        never completes, so a stall is detected here from the knuckle no
        longer making meaningful progress.  A caller can keep a deliberately
        low-overclose command active after that stall, until a later release
        command preempts it; this maintains the normal force needed for a
        physical friction grip.  Calls that do not request that behavior
        cancel the pending command at the stalled aperture.
        """
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        send = self._grip_client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.05)
        ctrl_handle = send.result()
        if ctrl_handle is None or not ctrl_handle.accepted:
            return ERR_EXECUTION, "gripper_controller rejected goal", False, False

        result_future = ctrl_handle.get_result_async()
        t0 = time.monotonic()
        window_pos = None
        window_t = None
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                ctrl_handle.cancel_goal_async()
                return ERR_CANCELLED, "cancelled", False, False
            # Stall detection: GripperActionController never finishes the goal
            # if an object blocks the fingers short of the target position.
            # A grasp is stalled when the knuckle makes NO MEANINGFUL PROGRESS
            # over a ~0.5 s window while the goal is still running — i.e. the
            # fingers are blocked. We check window progress rather than the
            # instantaneous velocity: the gz position servo keeps a small
            # residual press force on a blocked knuckle, so it may creep at a
            # slow, nearly constant rate instead of stopping dead. With
            # keep_pressing=True we leave the low-overclose command
            # active.  This retains a small normal force, hence physical
            # friction, while the arm transfers the object.  Cancelling at
            # this point merely freezes the aperture and can remove that
            # force, letting a cube fall despite a successful stall.
            cur = self._joint_pos.get("robotiq_85_left_knuckle_joint")
            now = time.monotonic()
            if cur is not None:
                if window_pos is None:
                    window_pos, window_t = cur, now
                elif now - window_t >= 0.5:
                    if abs(cur - window_pos) < 0.003 and now - t0 > 1.0:
                        # The goal is still running here: the fingers stopped
                        # short of the target position, so an object is
                        # blocking them -> never "reached".
                        reached = False
                        if keep_pressing:
                            return OK, "ok", True, reached
                        ctrl_handle.cancel_goal_async()
                        return OK, "ok", True, reached
                    window_pos, window_t = cur, now
            if now - t0 > timeout_s:
                ctrl_handle.cancel_goal_async()
                return ERR_EXECUTION, "gripper command timed out", False, False
            fb_msg.phase = fb_phase
            fb_msg.progress = min(0.95, (now - t0) / timeout_s)
            goal_handle.publish_feedback(fb_msg)
            time.sleep(0.1)

        res = result_future.result().result
        return OK, "ok", bool(res.stalled), bool(res.reached_goal)

    # ------------------------------------------------------------- skills
    def _exec_move_joints(self, goal_handle):
        fb = MoveJoints.Feedback()
        result = MoveJoints.Result()
        req = goal_handle.request

        fb.phase = "validating"
        goal_handle.publish_feedback(fb)
        ok, msg = self._preconditions(need_arm=True)
        if not ok:
            result.success, result.error_code, result.message = False, ERR_PRECONDITION, msg
            goal_handle.abort()
            return result
        for i, (p, (lo, hi)) in enumerate(zip(req.positions, ARM_LIMITS)):
            if not (lo <= p <= hi):
                result.success = False
                result.error_code = ERR_PRECONDITION
                result.message = (f"joint_{i+1}={p:.3f} outside limits "
                                  f"[{lo:.3f}, {hi:.3f}]")
                goal_handle.abort()
                return result

        duration = max(float(req.duration), 0.5)
        code, msg = self._send_arm_trajectory(req.positions, duration,
                                              goal_handle, fb, "executing")
        result.success = code == OK
        result.error_code = code
        result.message = msg
        if code == OK:
            goal_handle.succeed()
        elif code == ERR_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _exec_move_to(self, goal_handle):
        fb = MoveTo.Feedback()
        result = MoveTo.Result()
        req = goal_handle.request

        fb.phase = "validating"
        goal_handle.publish_feedback(fb)
        ok, msg = self._preconditions(need_arm=True)
        if not ok:
            result.success, result.error_code, result.message = False, ERR_PRECONDITION, msg
            goal_handle.abort()
            return result

        fb.phase = "planning"
        goal_handle.publish_feedback(fb)

        # Plan using motion planner
        trajectory = self._motion_planner.plan_to_pose(
            req.target,
            velocity_scaling=req.velocity_scaling,
            acceleration_scaling=req.acceleration_scaling,
        )

        if trajectory is None:
            result.success = False
            result.error_code = ERR_PLANNING
            result.message = "MoveIt planning failed"
            goal_handle.abort()
            return result

        fb.phase = "executing"
        goal_handle.publish_feedback(fb)

        # Execute trajectory
        success = self._motion_planner.execute_trajectory(trajectory)

        result.success = success
        result.error_code = OK if success else ERR_EXECUTION
        result.message = "MoveTo completed" if success else "MoveTo execution failed"
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _exec_move_relative(self, goal_handle):
        fb = MoveRelative.Feedback()
        result = MoveRelative.Result()
        req = goal_handle.request

        fb.phase = "validating"
        goal_handle.publish_feedback(fb)
        ok, msg = self._preconditions(need_arm=True)
        if not ok:
            result.success, result.error_code, result.message = False, ERR_PRECONDITION, msg
            goal_handle.abort()
            return result

        fb.phase = "planning"
        goal_handle.publish_feedback(fb)

        # Plan relative Cartesian move
        rotation_delta = None
        if req.rotation_delta.x != 0.0 or req.rotation_delta.y != 0.0 or \
           req.rotation_delta.z != 0.0 or req.rotation_delta.w != 0.0:
            rotation_delta = req.rotation_delta

        trajectory, fraction = self._motion_planner.plan_relative_cartesian(
            req.translation,
            rotation_delta=rotation_delta,
            reference_frame=req.reference_frame,
            velocity_scaling=req.velocity_scaling,
            acceleration_scaling=req.acceleration_scaling,
        )

        if trajectory is None or fraction < 0.9:
            result.success = False
            result.error_code = ERR_PLANNING
            result.message = f"Cartesian planning failed (fraction={fraction:.2f})"
            goal_handle.abort()
            return result

        fb.phase = "executing"
        goal_handle.publish_feedback(fb)

        # Execute trajectory
        success = self._motion_planner.execute_trajectory(trajectory)

        result.success = success
        result.error_code = OK if success else ERR_EXECUTION
        result.message = "MoveRelative completed" if success else "MoveRelative execution failed"
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _exec_move_axis(self, goal_handle):
        fb = MoveAxis.Feedback()
        result = MoveAxis.Result()
        req = goal_handle.request

        fb.phase = "validating"
        goal_handle.publish_feedback(fb)
        ok, msg = self._preconditions(need_arm=True)
        if not ok:
            result.success, result.error_code, result.message = False, ERR_PRECONDITION, msg
            goal_handle.abort()
            return result

        # Validate joint names
        if len(req.joint_names) != len(req.positions):
            result.success = False
            result.error_code = ERR_PRECONDITION
            result.message = "joint_names and positions length mismatch"
            goal_handle.abort()
            return result

        for name in req.joint_names:
            if name not in ARM_JOINTS:
                result.success = False
                result.error_code = ERR_PRECONDITION
                result.message = f"Unknown joint: {name}"
                goal_handle.abort()
                return result

        fb.phase = "planning"
        goal_handle.publish_feedback(fb)

        # Get current positions for relative moves
        if req.relative:
            current_positions = {}
            with self._js_lock:
                for name in ARM_JOINTS:
                    if name in self._joint_pos:
                        current_positions[name] = self._joint_pos[name]
            target_positions = []
            for name, delta in zip(req.joint_names, req.positions):
                target_positions.append(current_positions.get(name, 0.0) + delta)
        else:
            target_positions = list(req.positions)

        # Validate joint limits
        for i, (name, pos) in enumerate(zip(req.joint_names, target_positions)):
            joint_idx = ARM_JOINTS.index(name)
            lo, hi = ARM_LIMITS[joint_idx]
            if not (lo <= pos <= hi):
                result.success = False
                result.error_code = ERR_PRECONDITION
                result.message = f"{name}={pos:.3f} outside limits [{lo:.3f}, {hi:.3f}]"
                goal_handle.abort()
                return result

        # Plan to joint target
        trajectory = self._motion_planner.plan_to_joint_target(
            target_positions,
            velocity_scaling=req.velocity_scaling,
            acceleration_scaling=req.acceleration_scaling,
        )

        if trajectory is None:
            result.success = False
            result.error_code = ERR_PLANNING
            result.message = "Joint-space planning failed"
            goal_handle.abort()
            return result

        fb.phase = "executing"
        goal_handle.publish_feedback(fb)

        # Execute trajectory
        success = self._motion_planner.execute_trajectory(trajectory)

        result.success = success
        result.error_code = OK if success else ERR_EXECUTION
        result.message = "MoveAxis completed" if success else "MoveAxis execution failed"
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _exec_grasp(self, goal_handle):
        fb = Grasp.Feedback()
        result = Grasp.Result()
        req = goal_handle.request

        fb.phase = "validating"
        goal_handle.publish_feedback(fb)
        ok, msg = self._preconditions(need_gripper=True)
        if not ok:
            result.success, result.error_code, result.message = False, ERR_PRECONDITION, msg
            goal_handle.abort()
            return result
        pos = min(max(float(req.position), GRIPPER_RANGE[0]), GRIPPER_RANGE[1])

        code, msg, stalled, reached = self._send_gripper(
            pos, req.max_effort, goal_handle, fb, "closing",
            keep_pressing=True)
        # semantics: reaching target = closed on nothing; stalling early =
        # object held.  On a stall, retain the low-force close command until
        # a release goal preempts it; this is a physical squeeze, not a
        # virtual attachment.
        result.object_detected = stalled and not reached
        result.success = code == OK and (reached or stalled)
        result.error_code = code if code != OK else (OK if result.success else ERR_EXECUTION)
        result.message = msg if code != OK else (
            "object gripped (fingers holding at object width)" if result.object_detected
            else "closed to target (no object)")
        if result.success:
            goal_handle.succeed()
        elif code == ERR_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _exec_release(self, goal_handle):
        fb = Release.Feedback()
        result = Release.Result()

        ok, msg = self._preconditions(need_gripper=True)
        if not ok:
            result.success, result.error_code, result.message = False, ERR_PRECONDITION, msg
            goal_handle.abort()
            return result

        code, msg, _stalled, reached = self._send_gripper(
            GRIPPER_RANGE[0], 40.0, goal_handle, fb, "opening")
        result.success = code == OK and reached
        result.error_code = code if code != OK else (OK if reached else ERR_EXECUTION)
        result.message = "gripper open" if result.success else msg
        if result.success:
            goal_handle.succeed()
        elif code == ERR_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _exec_home(self, goal_handle):
        fb = Home.Feedback()
        result = Home.Result()
        req = goal_handle.request

        ok, msg = self._preconditions(need_arm=True,
                                      need_gripper=req.open_gripper)
        if not ok:
            result.success, result.error_code, result.message = False, ERR_PRECONDITION, msg
            goal_handle.abort()
            return result

        code, msg = self._send_arm_trajectory(HOME_POSE, 4.0, goal_handle, fb,
                                              "arm_homing")
        if code != OK:
            result.success, result.error_code, result.message = False, code, msg
            if code == ERR_CANCELLED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        if req.open_gripper:
            code, msg, _st, reached = self._send_gripper(
                GRIPPER_RANGE[0], 40.0, goal_handle, fb, "gripper_opening")
            if code != OK or not reached:
                result.success = False
                result.error_code = code if code != OK else ERR_EXECUTION
                result.message = f"arm homed but gripper open failed: {msg}"
                goal_handle.abort()
                return result

        result.success, result.error_code, result.message = True, OK, "at home"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SkillServers()
    executor = MultiThreadedExecutor(num_threads=8)
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
