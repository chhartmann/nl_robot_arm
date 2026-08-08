"""NL Robot Arm — orchestrator.

Executes symbolic tasks by grounding object references through the world
model and sequencing skill actions. Exposes:

  /orchestrator/execute_task  (nlra_interfaces/action/ExecuteTask)

Plans are simple declarative step lists; each step calls a skill action
and checks success. Failure aborts the plan with failed_step set.

Grounding: object pose (world model, ground truth) -> arm joint targets
via MoveIt-backed Cartesian planning (move_to, move_relative skills).
"""
import json
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Quaternion
from nlra_interfaces.action import ExecuteTask, Grasp, Home, MoveJoints, MoveTo, MoveRelative, MoveAxis, Release
from nlra_interfaces.srv import GetGraspPose, GetObjectPose
from sensor_msgs.msg import JointState

OK = 0
ERR_UNKNOWN_TASK = 1
ERR_GROUNDING = 2
ERR_SKILL = 3
ERR_CANCELLED = 4

# Joint name aliases accepted in move_joints args (LLM-friendly)
JOINT_INDEX = {
    "joint_1": 0, "a1": 0, "j1": 0,
    "joint_2": 1, "a2": 1, "j2": 1,
    "joint_3": 2, "a3": 2, "j3": 2,
    "joint_4": 3, "a4": 3, "j4": 3,
    "joint_5": 4, "a5": 4, "j5": 4,
    "joint_6": 5, "a6": 5, "j6": 5,
}

# The gripper closes toward a small angle PAST first pad contact and stalls on
# whatever object blocks it — the stall angle adapts to the object's size. The
# press preload must exceed the GripperActionController's goal_tolerance plus
# the contact solver's minDepth compliance; otherwise the goal completes early
# ("reached", no object) or the fingers keep creeping into the object. The
# grasp skill detects the stall and leaves this low-force command active until
# release, maintaining the normal force required for physical friction.
GRIPPER_FULLY_CLOSED = 0.8     # rad, knuckle fully closed
# Firm-grip effort: the knuckle's URDF effort limit is 50 N, the maximum
# squeeze the joint can apply regardless of the requested value.
GRIPPER_GRASP_EFFORT = 50.0
# Knuckle angle -> finger-pad gap calibration, measured in sim:
#   visual_pad_gap(cm) = 8.838 - 11.03 * angle(rad)
# The 8.838 cm intercept includes the real asymmetric fingertip meshes.  The
# rendered inner faces project 19.26 mm farther in on each side than the old
# simplified collision boxes, so using the former 12.69 cm calibration drove
# the *visual* claws through a 5 cm cube.
# The fingers contact an object of width w when pad_gap == w. This is gripper
# geometry only — keep in sync if the gripper URDF changes.
# Keep a modest position error behind the object: it must exceed both the
# controller tolerance and Gazebo's contact compliance, otherwise the first
# close only touches the cube and cannot develop enough normal force to hold
# it during the lift.  This maps to about 2.8 mm of additional pad closure,
# while remaining far below a full-close command that could eject the cube.
GRASP_PRESS_DELTA = 0.025      # rad past contact -> sustained holding press
APPROACH_CLEARANCE = 0.18      # m above grasp height for approach pose
# Grasp-point height above an object's bottom — same constant as GRASP_RAISE
# in the world model (keep in sync): visual fingertip length (57 mm) +
# closing sweep (13.5 mm) + support clearance (10 mm). Gripper/table geometry
# only, independent of object size.
GRASP_RAISE = 0.0805
# An object held between the fingertips hangs ~17 mm below the grasp point
# (grasp point = midpoint of the fingertip link origins; empirical, measured
# on the cube). Used to aim the drop height in the place skill.
GRASP_HANG = 0.017


class Orchestrator(Node):
    def __init__(self):
        super().__init__("nlra_orchestrator")
        self._cb = ReentrantCallbackGroup()

        self._move_joints = ActionClient(self, MoveJoints, "skills/move_joints",
                                         callback_group=self._cb)
        self._move_to = ActionClient(self, MoveTo, "skills/move_to",
                                     callback_group=self._cb)
        self._move_relative = ActionClient(self, MoveRelative, "skills/move_relative",
                                           callback_group=self._cb)
        self._grasp = ActionClient(self, Grasp, "skills/grasp",
                                   callback_group=self._cb)
        self._release = ActionClient(self, Release, "skills/release",
                                     callback_group=self._cb)
        self._home = ActionClient(self, Home, "skills/home",
                                  callback_group=self._cb)
        self._get_pose = self.create_client(GetObjectPose,
                                            "world_model/get_object_pose",
                                            callback_group=self._cb)
        self._get_grasp_pose = self.create_client(GetGraspPose,
                                                  "world_model/get_grasp_pose",
                                                  callback_group=self._cb)

        # latest arm joint positions (for move_joints: unspecified joints stay)
        self._joint_state = None
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self._cb)

        ActionServer(self, ExecuteTask, "orchestrator/execute_task",
                     execute_callback=self._exec_task,
                     goal_callback=lambda _r: GoalResponse.ACCEPT,
                     cancel_callback=lambda _h: CancelResponse.ACCEPT,
                     callback_group=self._cb)
        self.get_logger().info(
            "orchestrator up: tasks home, pick, place, pick_and_place, move_joints")

    def _on_joint_state(self, msg):
        try:
            idx = [msg.name.index(j) for j in ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]]
        except ValueError:
            return
        self._joint_state = [msg.position[i] for i in idx]

    # ---------------------------------------------------------------- utils
    def _object_pose(self, obj_id, timeout=5.0):
        if not self._get_pose.wait_for_service(timeout_sec=timeout):
            return None, "world model service unavailable"
        fut = self._get_pose.call_async(GetObjectPose.Request(id=obj_id))
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return None, "world model query timed out"
            time.sleep(0.05)
        res = fut.result()
        if not res.found:
            return None, f"object '{obj_id}' not known to world model"
        p = res.object.pose.pose.position
        return (p.x, p.y, p.z, res.object.size, res.object.kind), ""

    def _grasp_pose(self, obj_id, timeout=5.0):
        """Grasp pose from the world model: grasp point + orientation with
        the gripper parallel to the object. Returns (PoseStamped, "") or
        (None, err)."""
        if not self._get_grasp_pose.wait_for_service(timeout_sec=timeout):
            return None, "grasp pose service unavailable"
        fut = self._get_grasp_pose.call_async(
            GetGraspPose.Request(object_id=obj_id))
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return None, "grasp pose query timed out"
            time.sleep(0.05)
        res = fut.result()
        if not res.found:
            return None, res.message
        self.get_logger().info(f"grasp pose for '{obj_id}': {res.message}")
        return res.grasp_pose, ""

    def _grasp_step(self, position, max_effort):
        """Close the gripper; succeed only if an object was actually caught."""
        ok, msg, res = self._call_raw(
            self._grasp, Grasp.Goal(position=position, max_effort=max_effort))
        if not ok:
            return False, msg
        if res is None or not res.object_detected:
            return False, "gripper closed without contact (no object between fingers)"
        return True, "object firmly gripped; holding pressure until release"

    def _call(self, client, goal, timeout=60.0):
        ok, msg, _ = self._call_raw(client, goal, timeout)
        return ok, msg

    def _call_raw(self, client, goal, timeout=60.0):
        """Send a skill goal, wait, return (ok, message, result_or_None)."""
        if not client.wait_for_server(timeout_sec=5.0):
            return False, "skill server unavailable", None
        fut = client.send_goal_async(goal)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 10:
                return False, "goal not accepted in time", None
            time.sleep(0.05)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False, "goal rejected", None
        rfut = handle.get_result_async()
        t0 = time.monotonic()
        while not rfut.done():
            if time.monotonic() - t0 > timeout:
                handle.cancel_goal_async()
                return False, "skill timed out", None
            time.sleep(0.1)
        result = rfut.result().result
        return bool(result.success), result.message, result

    def _move_relative_step(self, name, translation, reference_frame="tool0", duration=4.0):
        goal = MoveRelative.Goal()
        goal.translation = translation
        goal.reference_frame = reference_frame
        goal.velocity_scaling = 0.2
        goal.acceleration_scaling = 0.2
        return (name, lambda: self._call(self._move_relative, goal))

    def _move_to_step(self, name, target_pose, duration=4.0):
        goal = MoveTo.Goal()
        goal.target = target_pose
        goal.velocity_scaling = 0.2
        goal.acceleration_scaling = 0.2
        return (name, lambda: self._call(self._move_to, goal))

    def _grasp_target_angle(self, width):
        """Knuckle angle that presses an object of the given width.

        The fingers contact the object when the pad gap equals its width.
        Returning a target a small angle PAST the contact angle keeps a small
        residual position error, so the gz velocity-based position servo
        presses the object gently; the grasp skill detects that the fingers
        are blocked (stall) and retains that low-force command through the
        transfer.
        """
        width_cm = float(width) * 100.0
        a_contact = (8.838 - width_cm) / 11.03
        return min(GRIPPER_FULLY_CLOSED, max(0.0, a_contact + GRASP_PRESS_DELTA))

    # ---------------------------------------------------------------- plans
    def _plan_for(self, task, args):
        """Return list of (step_name, callable) or None."""
        if task == "home":
            return [("home", lambda: self._call(
                self._home, Home.Goal(open_gripper=True)))]

        if task == "pick":
            obj = args.get("object_id", "red_cube")
            return self._pick_steps(obj)

        if task == "place":
            tgt = args.get("target_id", "blue_box")
            return self._place_steps(tgt)

        if task == "pick_and_place":
            obj = args.get("object_id", "red_cube")
            tgt = args.get("target_id", "blue_box")
            steps = self._pick_steps(obj)
            if steps is None:
                return None
            more = self._place_steps(tgt)
            if more is None:
                return None
            return steps + more + [("home", lambda: self._call(
                self._home, Home.Goal(open_gripper=False)))]

        if task == "move_joints":
            return self._move_joints_steps(args)
        return None

    def _move_joints_steps(self, args):
        """Ground {'joints': {a1: 90, ...}} (degrees) to a 6-dof absolute pose."""
        joints = args.get("joints", {})
        if not isinstance(joints, dict) or not joints:
            self._ground_err = ("move_joints requires a 'joints' map, "
                                "e.g. {\"a1\": 90}")
            return None
        if self._joint_state is None:
            self.get_logger().warn("no /joint_states yet — assuming zeros for "
                                   "unspecified joints")
        pos = list(self._joint_state or [0.0] * 6)
        for name, deg in joints.items():
            idx = JOINT_INDEX.get(str(name).strip().lower())
            if idx is None:
                self._ground_err = (f"unknown joint '{name}' — use a1..a6 or "
                                    "joint_1..joint_6")
                return None
            try:
                pos[idx] = math.radians(float(deg))
            except (TypeError, ValueError):
                self._ground_err = f"joint '{name}': angle {deg!r} is not a number"
                return None
        goal = MoveJoints.Goal()
        goal.positions = pos
        goal.duration = 5.0
        return [("move_joints", lambda: self._call(self._move_joints, goal))]

    def _pick_steps(self, obj_id):
        gpose, err = self._grasp_pose(obj_id)
        if gpose is None:
            self._ground_err = err
            return None

        x, y, z_grasp = gpose.pose.position.x, gpose.pose.position.y, gpose.pose.position.z

        # Use MoveRelative for approach, descend, lift (in tool0 frame)
        # Approach: move up by APPROACH_CLEARANCE from grasp pose
        approach_trans = Vector3()
        approach_trans.x = 0.0
        approach_trans.y = 0.0
        approach_trans.z = APPROACH_CLEARANCE

        # Descend: move down by APPROACH_CLEARANCE to grasp pose
        descend_trans = Vector3()
        descend_trans.x = 0.0
        descend_trans.y = 0.0
        descend_trans.z = -APPROACH_CLEARANCE

        # Lift: move up by APPROACH_CLEARANCE from grasp pose
        lift_trans = Vector3()
        lift_trans.x = 0.0
        lift_trans.y = 0.0
        lift_trans.z = APPROACH_CLEARANCE

        # Object width along the closing axis -> gentle press target angle.
        pose, perr = self._object_pose(obj_id)
        width = pose[3][0] if pose is not None else 0.05
        grasp_pos = self._grasp_target_angle(width)
        self.get_logger().info(
            f"grasp target angle for '{obj_id}' (w={width*100:.1f} cm): "
            f"{grasp_pos:.3f} rad")

        return [
            ("open_gripper", lambda: self._call(self._release, Release.Goal())),
            self._move_to_step(f"approach:{obj_id}", gpose),
            self._move_relative_step(f"descend:{obj_id}", descend_trans, "tool0", 2.5),
            (f"grasp:{obj_id}", lambda: self._grasp_step(
                grasp_pos, GRIPPER_GRASP_EFFORT)),
            self._move_relative_step(f"lift:{obj_id}", lift_trans, "tool0", 6.0),
        ]

    def _place_steps(self, tgt_id):
        pose, err = self._object_pose(tgt_id)
        if pose is None:
            self._ground_err = err
            return None
        x, y, z, size, kind = pose
        if kind == "tray":
            # Support surface = the tray's interior floor. Its 2 cm-thick
            # floor box is centered at the model origin, so the top is 1 cm
            # above it.
            support = z + 0.01
        else:
            # Support surface = the target's top.
            support = z + size[2] / 2
        # drop_z is the GRASP-POINT height (midpoint of the fingertip link
        # origins, same convention as the pick). An attached object's center
        # hangs GRASP_HANG below it, so its bottom sits at
        # drop_z - GRASP_HANG - size[2]/2. Aim the bottom at the support, but
        # never lower the fingers below GRASP_RAISE so the closed fingertips
        # keep their clearance (same geometry constants as the world model —
        # keep in sync).
        drop_z = support + max(GRASP_RAISE, GRASP_HANG + size[2] / 2.0)

        # Create absolute poses for transfer, lower, retreat
        from geometry_msgs.msg import PoseStamped, Pose
        above_pose = PoseStamped()
        above_pose.header.frame_id = "base_link"
        above_pose.pose.position.x = x
        above_pose.pose.position.y = y
        above_pose.pose.position.z = drop_z + 0.10
        above_pose.pose.orientation.w = 1.0  # tool pointing down

        drop_pose = PoseStamped()
        drop_pose.header.frame_id = "base_link"
        drop_pose.pose.position.x = x
        drop_pose.pose.position.y = y
        drop_pose.pose.position.z = drop_z
        drop_pose.pose.orientation.w = 1.0

        retreat_pose = PoseStamped()
        retreat_pose.header.frame_id = "base_link"
        retreat_pose.pose.position.x = x
        retreat_pose.pose.position.y = y
        retreat_pose.pose.position.z = drop_z + 0.10
        retreat_pose.pose.orientation.w = 1.0

        return [
            self._move_to_step(f"transfer:{tgt_id}", above_pose),
            self._move_to_step(f"lower:{tgt_id}", drop_pose, 2.5),
            (f"release:{tgt_id}", lambda: self._call(self._release, Release.Goal())),
            self._move_to_step(f"retreat:{tgt_id}", retreat_pose, 2.5),
        ]

    # ------------------------------------------------------------- executor
    def _verify_at_target(self, obj_id, tgt_id, xy_tol=0.12):
        """Postcondition: obj within xy_tol of target (both from world model)."""
        obj, e1 = self._object_pose(obj_id)
        tgt, e2 = self._object_pose(tgt_id)
        if obj is None or tgt is None:
            return False, e1 or e2
        dx = obj[0] - tgt[0]
        dy = obj[1] - tgt[1]
        d = math.hypot(dx, dy)
        if d <= xy_tol:
            return True, f"{obj_id} at target (xy dist {d*100:.1f} cm)"
        return False, f"{obj_id} NOT at target (xy dist {d*100:.1f} cm > {xy_tol*100:.0f} cm)"

    def _exec_task(self, goal_handle):
        result = ExecuteTask.Result()
        req = goal_handle.request
        try:
            args = json.loads(req.args_json) if req.args_json else {}
        except json.JSONDecodeError as e:
            result.success = False
            result.error_code = ERR_UNKNOWN_TASK
            result.message = f"bad args_json: {e}"
            goal_handle.abort()
            return result

        max_attempts = 2 if req.task == "pick_and_place" else 1
        last_msg = ""
        for attempt in range(1, max_attempts + 1):
            ok, code, msg, failed = self._run_plan(goal_handle, req.task, args,
                                                   attempt, max_attempts)
            if code == ERR_CANCELLED:
                result.success = False
                result.error_code = code
                result.message = msg
                result.failed_step = failed
                goal_handle.canceled()
                return result
            last_msg, last_code, last_failed = msg, code, failed
            if not ok:
                continue          # execution failed -> retry with fresh poses
            # postcondition for pick_and_place: object actually at target
            if req.task == "pick_and_place":
                obj = args.get("object_id", "red_cube")
                tgt = args.get("target_id", "blue_box")
                placed, pmsg = self._verify_at_target(obj, tgt)
                self.get_logger().info(f"postcondition: {pmsg}")
                if not placed:
                    last_msg = f"postcondition failed: {pmsg}"
                    last_code = ERR_SKILL
                    last_failed = "verify"
                    continue      # regrounds from live world model on retry
                msg = f"{msg}; verified: {pmsg}"
            result.success = True
            result.error_code = OK
            result.message = msg + (f" (attempt {attempt})" if attempt > 1 else "")
            goal_handle.succeed()
            return result

        result.success = False
        result.error_code = last_code
        result.message = f"{last_msg} (after {max_attempts} attempts)"
        result.failed_step = last_failed
        goal_handle.abort()
        return result

    def _run_plan(self, goal_handle, task, args, attempt, max_attempts):
        """Ground + execute one attempt. Returns (ok, code, message, failed_step)."""
        self._ground_err = ""
        try:
            plan = self._plan_for(task, args)
        except Exception as e:
            self.get_logger().error(f"planning crashed: {e!r}")
            return False, ERR_GROUNDING, f"planning error: {e}", ""
        if plan is None:
            if self._ground_err:
                return False, ERR_GROUNDING, self._ground_err, ""
            return False, ERR_UNKNOWN_TASK, f"unknown task '{task}'", ""

        fb = ExecuteTask.Feedback()
        fb.step_count = len(plan)
        self.get_logger().info(
            f"task '{task}' attempt {attempt}/{max_attempts}: {len(plan)} steps")
        for i, (name, fn) in enumerate(plan):
            if goal_handle.is_cancel_requested:
                return False, ERR_CANCELLED, "cancelled", name
            fb.step = name
            fb.step_index = i
            fb.progress = i / len(plan)
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f"  step {i+1}/{len(plan)}: {name}")
            ok, msg = fn()
            if not ok:
                self.get_logger().warn(f"  step {i+1}/{len(plan)}: {name} FAILED: {msg}")
                return False, ERR_SKILL, f"step '{name}' failed: {msg}", name
        return True, OK, f"task '{task}' completed ({len(plan)} steps)", ""


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()