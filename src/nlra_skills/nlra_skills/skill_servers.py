"""NL Robot Arm — skill layer.

One node hosting all skill action servers. Each skill:
  1. validates preconditions (controllers alive, fresh joint states, limits),
  2. delegates execution to the ros2_control controller actions,
  3. streams phase/progress feedback and returns a typed result.

Skills are deliberately thin, deterministic wrappers — intelligence lives in
the orchestrator, physics lives in Gazebo/ros2_control. MoveTo (Cartesian)
will delegate to MoveIt in a later phase and currently returns
planning_failed with a clear message.
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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from nlra_interfaces.action import Grasp, Home, MoveJoints, MoveTo, Release

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

        ActionServer(self, MoveJoints, "skills/move_joints",
                     execute_callback=self._exec_move_joints,
                     goal_callback=self._accept, cancel_callback=self._cancel,
                     callback_group=self._cb)
        ActionServer(self, MoveTo, "skills/move_to",
                     execute_callback=self._exec_move_to,
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

        self.get_logger().info("skill servers up: move_joints, move_to, grasp, release, home")

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
                      timeout_s=15.0):
        """Send gripper command, block until done. Returns (code, message, stalled, reached)."""
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
        last_pos = None
        still_since = None
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                ctrl_handle.cancel_goal_async()
                return ERR_CANCELLED, "cancelled", False, False
            # Stall detection: GripperActionController never finishes the goal
            # if an object blocks the fingers short of the target position.
            # A knuckle that hasn't moved for >1.5s while the goal is pending
            # means the gripper is squeezing something -> stalled grasp.
            cur = self._joint_pos.get("robotiq_85_left_knuckle_joint")
            if cur is not None:
                if last_pos is not None and abs(cur - last_pos) < 0.002:
                    if still_since is None:
                        still_since = time.monotonic()
                    elif time.monotonic() - still_since > 1.5 and \
                            time.monotonic() - t0 > 2.0:
                        ctrl_handle.cancel_goal_async()
                        reached = abs(cur - position) < 0.02
                        return OK, "ok", not reached, reached
                else:
                    still_since = None
                last_pos = cur
            if time.monotonic() - t0 > timeout_s:
                ctrl_handle.cancel_goal_async()
                return ERR_EXECUTION, "gripper command timed out", False, False
            fb_msg.phase = fb_phase
            fb_msg.progress = min(0.95, (time.monotonic() - t0) / timeout_s)
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
        # Cartesian motion requires MoveIt (Phase: motion planning integration).
        result = MoveTo.Result()
        result.success = False
        result.error_code = ERR_PLANNING
        result.message = ("MoveTo (Cartesian) not yet wired to MoveIt; "
                          "use skills/move_joints for now")
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
            pos, req.max_effort, goal_handle, fb, "closing")
        # semantics: reaching target = closed on nothing; stalling early = object held
        result.object_detected = stalled and not reached
        result.success = code == OK and (reached or stalled)
        result.error_code = code if code != OK else (OK if result.success else ERR_EXECUTION)
        result.message = msg if code != OK else (
            "object detected (gripper stalled)" if result.object_detected
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
