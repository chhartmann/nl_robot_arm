"""NL Robot Arm — orchestrator.

Executes symbolic tasks by grounding object references through the world
model and sequencing skill actions. Exposes:

  /orchestrator/execute_task  (nlra_interfaces/action/ExecuteTask)

Plans are simple declarative step lists; each step calls a skill action
and checks success. Failure aborts the plan with failed_step set.

Grounding: object pose (world model, ground truth) -> arm joint targets
via a small numeric IK for the arm's first three joints + wrist, with
the tool pointing straight down. Good enough for tabletop pick/place in
sim; MoveIt-backed Cartesian planning replaces this in a later phase.
"""
import json
import math
import os
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nlra_interfaces.action import ExecuteTask, Grasp, Home, MoveJoints, Release
from nlra_interfaces.srv import GetObjectPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

OK = 0
ERR_UNKNOWN_TASK = 1
ERR_GROUNDING = 2
ERR_SKILL = 3
ERR_CANCELLED = 4

# --- numeric IK on the real URDF (yourdfpy FK + scipy least squares) ---
URDF_PATH = os.environ.get("NLRA_URDF_PATH",
                           "/opt/data/nl_robot_arm/.snapshot_urdf.xml")
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
TOOL_LINK = "tool0"
# joint name aliases accepted in move_joints args (LLM-friendly)
JOINT_INDEX = {
    "joint_1": 0, "a1": 0, "j1": 0,
    "joint_2": 1, "a2": 1, "j2": 1,
    "joint_3": 2, "a3": 2, "j3": 2,
    "joint_4": 3, "a4": 3, "j4": 3,
    "joint_5": 4, "a5": 4, "j5": 4,
    "joint_6": 5, "a6": 5, "j6": 5,
}
# grasp point offset below tool0 (gripper fingers) along tool z
GRASP_OFFSET = 0.15

GRIPPER_CLOSED_ON_CUBE = 0.60  # knuckle rad, squeeze slightly past 5cm-cube contact
APPROACH_CLEARANCE = 0.18      # m above grasp height for approach pose
# Max 3D distance between the cube center and the current FK grasp point for
# the detachable-joint attach to be allowed. gz-sim's DetachableJoint has NO
# proximity check of its own: it rigidly glues the child model on any
# /gripper/attach message, wherever it is. This guard prevents the cube from
# being dragged along when the gripper closed on air or a pose was stale.
ATTACH_PROXIMITY = 0.09        # m

_urdf = None


def _resolve_urdf_path():
    """URDF for FK/IK: env override -> legacy snapshot -> xacro (cached)."""
    if os.path.exists(URDF_PATH):
        return URDF_PATH
    from ament_index_python.packages import get_package_share_directory
    pkg = get_package_share_directory("agilus_robotiq_description")
    xacro_file = os.path.join(pkg, "urdf", "agilus_robotiq.urdf.xacro")
    controller_cfg = os.path.join(pkg, "config",
                                  "agilus_robotiq_controllers.yaml")
    cache = os.path.join(tempfile.gettempdir(),
                         "nlra_agilus_robotiq_urdf.xml")
    if not os.path.exists(cache):
        out = subprocess.run(
            ["xacro", xacro_file, f"controller_config:={controller_cfg}"],
            capture_output=True, text=True, check=True)
        with open(cache, "w") as f:
            f.write(out.stdout)
    return cache


def _get_urdf():
    global _urdf
    if _urdf is None:
        import yourdfpy
        _urdf = yourdfpy.URDF.load(
            _resolve_urdf_path(), load_meshes=False,
            build_collision_scene_graph=False, load_collision_meshes=False)
    return _urdf


def _fk(q):
    """Return (grasp_point, approach_dir) for joint vector q.

    grasp point = midpoint between the two fingertip links;
    approach dir = unit vector from gripper base to that midpoint
    (the direction the fingers point; must face down for a top grasp).
    """
    import numpy as np
    u = _get_urdf()
    u.update_cfg(dict(zip(ARM_JOINTS, q)))
    Tl = u.get_transform("robotiq_85_left_finger_tip_link", "base_link")
    Tr = u.get_transform("robotiq_85_right_finger_tip_link", "base_link")
    Tb = u.get_transform("robotiq_85_base_link", "base_link")
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    v = mid - Tb[:3, 3]
    n = np.linalg.norm(v)
    approach = v / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])
    return mid, approach


def ik_top_down(x, y, z_grasp):
    """Numeric IK: put the grasp point at (x,y,z) with tool z pointing down.

    Returns [j1..j6] or None if no converged solution.
    """
    import numpy as np
    from scipy.optimize import least_squares

    target = np.array([x, y, z_grasp])

    def resid(q):
        pos, approach = _fk(q)
        # fingers must point straight down (-z) at the target point
        return np.concatenate([
            (pos - target) * 10.0,
            (approach - np.array([0.0, 0.0, -1.0])) * 3.0,
        ])

    yaw = math.atan2(y, x)
    seeds = [
        [yaw, -0.6, 1.8, 0.0, 1.0, 0.0],
        [yaw, -0.9, 1.5, 0.0, 1.2, 0.0],
        [yaw, -0.3, 2.0, 0.0, 0.8, 0.0],
    ]
    # KUKA Agilus KR 16 R1100-3 joint limits (rad) from kuka_agilus_support URDF
    lo = [-2.9671, -3.4034, -2.0071, -3.4907, -2.0944, -6.1087]
    hi = [2.9671, 0.9599, 2.8798, 3.4907, 2.0944, 6.1087]
    best = None
    for s in seeds:
        try:
            r = least_squares(resid, s, bounds=(lo, hi), xtol=1e-10, max_nfev=200)
        except Exception:
            continue
        if r.cost < 1e-5 and (best is None or r.cost < best.cost):
            best = r
    if best is None:
        return None
    return [float(v) for v in best.x]


class Orchestrator(Node):
    def __init__(self):
        super().__init__("nlra_orchestrator")
        self._cb = ReentrantCallbackGroup()

        self._move = ActionClient(self, MoveJoints, "skills/move_joints",
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

        # DetachableJoint attach/detach (bridged to gz) — reliable grasp in
        # bullet-featherstone, which ignores contact friction for grasping.
        self._attach_pub = self.create_publisher(Empty, "/gripper/attach", 1)
        self._detach_pub = self.create_publisher(Empty, "/gripper/detach", 1)

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
            idx = [msg.name.index(j) for j in ARM_JOINTS]
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
        return (p.x, p.y, p.z, res.object.size), ""

    def _attach(self, obj_id):
        """Attach obj to the gripper ONLY if it is really between the fingers.

        gz-sim's DetachableJoint glues the child model on any attach message
        regardless of distance, so we verify against the live world-model pose
        before publishing.
        """
        if self._joint_state is None:
            return False, "no joint state — cannot verify grip proximity"
        pose, err = self._object_pose(obj_id)
        if pose is None:
            return False, err
        x, y, z, _ = pose
        grasp_point, _ = _fk(self._joint_state)
        gx, gy, gz = grasp_point
        d = math.dist((x, y, z), (gx, gy, gz))
        if d > ATTACH_PROXIMITY:
            return False, (f"refusing attach: {obj_id} is {d*100:.1f} cm from "
                           f"the grasp point ({gx:.2f},{gy:.2f},{gz:.2f}) — "
                           "not in the gripper")
        self._attach_pub.publish(Empty())
        time.sleep(0.5)   # let gz create the fixed joint before lifting
        return True, "attached"

    def _detach(self):
        self._detach_pub.publish(Empty())
        time.sleep(0.5)   # let gz remove the joint before releasing fingers
        return True, "detached"

    def _grasp_step(self, position, max_effort):
        """Close the gripper; succeed only if an object was actually caught."""
        ok, msg, res = self._call_raw(
            self._grasp, Grasp.Goal(position=position, max_effort=max_effort))
        if not ok:
            return False, msg
        if res is None or not res.object_detected:
            return False, "gripper closed without contact (no object between fingers)"
        return True, "object detected in gripper"

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

    # ---------------------------------------------------------------- plans
    def _plan_for(self, task, args):
        """Return list of (step_name, callable) or None."""
        if task == "home":
            return [("detach", lambda: self._detach()),
                    ("home", lambda: self._call(
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
        return [("detach", lambda: self._detach()),
                self._move_step("move_joints", pos, 5.0)]

    def _move_step(self, name, positions, duration=4.0):
        goal = MoveJoints.Goal()
        goal.positions = positions
        goal.duration = float(duration)
        return (name, lambda: self._call(self._move, goal))

    def _pick_steps(self, obj_id):
        pose, err = self._object_pose(obj_id)
        if pose is None:
            self._ground_err = err
            return None
        x, y, z, size = pose
        grasp_z = z + 0.005            # slightly above object center
        jl_app = ik_top_down(x, y, grasp_z + APPROACH_CLEARANCE)
        jl_grasp = ik_top_down(x, y, grasp_z)
        if jl_app is None or jl_grasp is None:
            self._ground_err = f"'{obj_id}' at ({x:.2f},{y:.2f},{z:.2f}) unreachable"
            return None
        return [
            ("detach", lambda: self._detach()),
            ("open_gripper", lambda: self._call(self._release, Release.Goal())),
            self._move_step(f"approach:{obj_id}", jl_app),
            self._move_step(f"descend:{obj_id}", jl_grasp, 2.5),
            (f"grasp:{obj_id}", lambda: self._grasp_step(
                GRIPPER_CLOSED_ON_CUBE, 120.0)),
            (f"attach:{obj_id}", lambda: self._attach(obj_id)),
            self._move_step(f"lift:{obj_id}", jl_app, 6.0),
        ]

    def _place_steps(self, tgt_id):
        pose, err = self._object_pose(tgt_id)
        if pose is None:
            self._ground_err = err
            return None
        x, y, z, size = pose
        drop_z = z + (size[2] if len(size) >= 3 else 0.08) + 0.06
        jl_above = ik_top_down(x, y, drop_z + 0.10)
        jl_drop = ik_top_down(x, y, drop_z)
        if jl_above is None or jl_drop is None:
            self._ground_err = f"'{tgt_id}' at ({x:.2f},{y:.2f},{z:.2f}) unreachable"
            return None
        return [
            ("detach", lambda: self._detach()),
            self._move_step(f"transfer:{tgt_id}", jl_above),
            self._move_step(f"lower:{tgt_id}", jl_drop, 2.5),
            (f"detach:{tgt_id}", lambda: self._detach()),
            (f"release:{tgt_id}", lambda: self._call(self._release, Release.Goal())),
            self._move_step(f"retreat:{tgt_id}", jl_above, 2.5),
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
                self._detach()   # never leave an object glued to the arm
                return False, ERR_CANCELLED, "cancelled", name
            fb.step = name
            fb.step_index = i
            fb.progress = i / len(plan)
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f"  step {i+1}/{len(plan)}: {name}")
            ok, msg = fn()
            if not ok:
                self._detach()   # best effort: clear any stale grip glue
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
