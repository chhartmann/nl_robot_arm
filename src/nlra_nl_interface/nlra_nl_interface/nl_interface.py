"""NL Robot Arm — natural-language interface (Phase 5).

Exposes /nl_command (nlra_interfaces/srv/NLCommand). Each request:
  1. queries the world model for the live object list,
  2. asks the LLM (OpenAI-compatible endpoint) to ground the utterance
     into a single execute_task call, an execute_plan (ordered sequence of
     tasks), or a plain-text answer,
  3. dispatches the grounded task/plan to /orchestrator/execute_task,
  4. on failure, feeds the error + fresh world state back to the LLM and
     re-dispatches (up to 3 rounds) before reporting failure,
  5. returns task result + a natural-language response.

Config via env (a `.env` file in the workspace root is loaded first, see
`.env.example`):
  NLRA_LLM_BASE   (default https://api.hcnsec.cn/v1)
  NLRA_LLM_MODEL  (default Qwen3.6-35B-A3B)
  NLRA_LLM_KEY    (required)
"""
import json
import os
import time
import urllib.error
import urllib.request

from dotenv import load_dotenv
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nlra_interfaces.action import ExecuteTask
from nlra_interfaces.srv import GetGraspPose, GetObjects, NLCommand

load_dotenv()
LLM_BASE = os.environ.get("NLRA_LLM_BASE", "https://api.hcnsec.cn/v1")
LLM_MODEL = os.environ.get("NLRA_LLM_MODEL", "Qwen3.6-35B-A3B")
LLM_KEY = os.environ.get("NLRA_LLM_KEY", "")

TASK_ENUM = ["home", "pick", "place", "pick_and_place", "drop", "lift",
             "move_joints", "move_axis", "move_relative", "move_to",
             "grasp", "release"]

TASK_PARAMETERS = {
    "type": "object",
    "properties": {
        "task": {"type": "string", "enum": TASK_ENUM},
        "object_id": {"type": "string",
                      "description": ("id of the object to manipulate "
                                      "(pick, pick_and_place)")},
        "target_id": {"type": "string",
                      "description": ("id of the destination object "
                                      "(place, pick_and_place)")},
        "placement": {
            "type": "object",
            "description": ("world drop point for place/drop: {mode, x, y, z}. "
                            "x, y, z are the WORLD coordinates (meters) of the "
                            "surface the object's bottom rests on. mode is "
                            "'auto', 'stack' (on top of an existing object) or "
                            "'side_by_side' (next to it)."),
            "properties": {
                "mode": {"type": "string",
                         "enum": ["auto", "stack", "side_by_side"]},
                "x": {"type": "number"}, "y": {"type": "number"},
                "z": {"type": "number"},
            },
            "required": ["x", "y", "z"],
        },
        "orientation": {
            "type": "object",
            "description": ("optional quaternion {rx, ry, rz, rw} for the "
                            "object's rest orientation when placing/dropping"),
        },
        "distance": {"type": "number",
                     "description": ("lift distance in meters (default 0.18)")},
        "joints": {"type": "object",
                   "description": ("only for move_joints / move_axis: "
                                   "map of joint id -> angle in degrees, "
                                   "e.g. {\"a1\": 90, \"a3\": -45}")},
        "relative": {"type": "boolean",
                     "description": ("only for move_axis: if true, joint "
                                     "values are deltas from the current "
                                     "pose, not absolute")},
        "translation": {"type": "object",
                        "description": ("only for move_relative: map of "
                                        "axis -> meters, e.g. {\"z\": 0.1} "
                                        "(positive z is up in base_link)")},
        "rotation_delta": {"type": "object",
                           "description": ("only for move_relative: optional "
                                           "orientation delta as a quaternion "
                                           "map {x, y, z, w}; omit for pure "
                                           "translation")},
        "reference_frame": {"type": "string",
                            "description": ("only for move_relative: frame "
                                            "the delta is expressed in; "
                                            "\"tool0\" (default) is gripper-"
                                            "relative, \"base_link\" is world "
                                            "axes")},
        "pose": {"type": "object",
                 "description": ("only for move_to: absolute target pose map "
                                 "with x, y, z (meters) and optional rx, ry, "
                                 "rz, rw (quaternion) in base_link")},
    },
    "required": ["task"],
}

TOOLS = [{
    "type": "function",
    "function": {
        "name": "execute_task",
        "description": (
            "Execute a single manipulation task on the robot arm. "
            "pick grasps and lifts an object. place puts the held object "
            "into/onto a target (or at an explicit world position). "
            "drop releases the held object at an explicit world position "
            "(no target object). lift raises the gripper straight up. "
            "pick_and_place does pick then place in one call. "
            "home returns the arm to its rest pose. "
            "move_joints / move_axis / move_relative / move_to move the arm. "
            "grasp closes the gripper, release opens it. "
            "For a multi-step job (e.g. 'clean up the desk'), call "
            "execute_plan instead."),
        "parameters": TASK_PARAMETERS,
    },
}, {
    "type": "function",
    "function": {
        "name": "execute_plan",
        "description": (
            "Execute an ordered sequence of manipulation tasks. Compose steps "
            "from the available tasks (pick, place, drop, lift, pick_and_place, "
            "move_*, grasp, release, home). Use this for multi-object commands "
            "like 'clean up the desk': one step per object, each picking it and "
            "placing it into the tray. Consider the live object positions to "
            "choose free drop spots (stack on top or put side by side, avoid "
            "dropping onto an occupied position)."),
        "parameters": {
            "type": "object",
            "properties": {
                "steps": {
                    "type": "array",
                    "items": TASK_PARAMETERS,
                    "description": "ordered list of tasks to execute",
                },
            },
            "required": ["steps"],
        },
    },
}, {
    "type": "function",
    "function": {
        "name": "get_grasp_pose",
        "description": (
            "Answer where/how the robot would grasp an object without "
            "moving: returns the grasp position and orientation for the "
            "given object_id. By default the gripper is oriented parallel "
            "to the object (fingers aligned with the object's axes, "
            "approach from above). Use for questions like 'where would "
            "you grab the cube?'."),
        "parameters": {
            "type": "object",
            "properties": {
                "object_id": {"type": "string",
                              "description": "id of the object to grasp"},
            },
            "required": ["object_id"],
        },
    },
}, ]

SYSTEM_TMPL = (
    "You are the command interface of a KUKA Agilus KR 16 R1100-3 robot arm "
    "with a Robotiq "
    "gripper working on a tabletop. Ground the user's command into a function "
    "call using ONLY the object ids listed below. "
    "Users speak informally: resolve descriptions, synonyms and translations "
    "to the closest matching object id (e.g. 'the tray'/'the container'/'die "
    "blaue Schale' -> blue_box, 'the block'/'der Wuerfel' -> red_cube). Only "
    "if no listed object plausibly matches, or the request is not a "
    "manipulation command, do NOT call a function; answer briefly in plain "
    "text instead (same language as the user).\n\n"
    "Task composition:\n"
    "- pick {{object_id}}: grasp and lift an object.\n"
    "- place {{target_id?, placement?}}: put the held object into/onto a target "
    "or at an explicit world position. placement = {{mode, x, y, z}} where x,y,z "
    "is the WORLD position of the surface the object's bottom rests on (m). "
    "mode is 'stack' (on top of an existing object), 'side_by_side' (next to "
    "it), or 'auto'.\n"
    "- drop {{placement}}: release the held object at an explicit world position "
    "(no target object).\n"
    "- lift {{distance?}}: raise the gripper straight up by distance meters "
    "(default 0.18).\n"
    "- pick_and_place {{object_id, target_id?, placement?}}: pick then place.\n"
    "- home: return the arm to its rest pose.\n"
    "- move_joints / move_axis / move_relative / move_to / grasp / release: "
    "raw arm/gripper motion.\n"
    "For a multi-object command (e.g. 'clean up the desk', 'put everything "
    "into the tray'), call execute_plan with an ordered list of steps. Look at "
    "the live positions below to decide where each object goes: if a spot in "
    "the tray is already occupied, place the next object next to it "
    "(side_by_side) or on top of it (stack), and pick a free world position so "
    "objects do not collide. Give explicit world x,y,z (z = height the object's "
    "bottom rests on).\n\n"
    "For joint motion use move_joints with 'joints': {{\"a1\": 90}}. Joint ids "
    "are a1..a6 (a.k.a. joint_1..joint_6), angles in degrees, absolute "
    "targets (not relative), within limits: a1: -170..170; a2: -195..55; "
    "a3: -115..165; a4: -200..200; a5: -120..120; a6: -350..350. "
    "Use move_axis instead when moving only a few joints, or to apply a "
    "relative delta ('relative': true, degrees).\n\n"
    "For relative gripper motion use move_relative with a 'translation' map in "
    "meters and, for world-aligned moves (up/down/left/right/forward), set "
    "'reference_frame': 'base_link' (e.g. 'move 10 cm up' -> {{\"translation\": "
    "{{\"z\": 0.1}}, \"reference_frame\": \"base_link\"}}). Omit reference_frame "
    "for gripper-relative motion (default 'tool0'). For an absolute Cartesian "
    "target use move_to with a 'pose' map {{x, y, z}} in base_link. "
    "grasp closes the gripper, release opens it.\n\n"
    "World state (id | kind | graspable | size | position | orientation):\n"
    "{objects}\n"
    "The blue_box tray is 16x16 cm with a ~14x14 cm interior (walls 1 cm "
    "thick, 6 cm tall); its interior floor top is at z = blue_box.z + 0.01 m. "
    "Objects placed inside must stay within the interior footprint.")


def llm_chat(messages, tools=None, timeout=90, retries=2):
    """Minimal OpenAI-compatible chat call via urllib (no SDK needed)."""
    body = {"model": LLM_MODEL, "messages": messages, "max_tokens": 2048}
    if tools:
        body["tools"] = tools
        body["tool_choice"] = "auto"
    req = urllib.request.Request(
        f"{LLM_BASE}/chat/completions",
        data=json.dumps(body).encode(),
        headers={"Authorization": f"Bearer {LLM_KEY}",
                 "Content-Type": "application/json"})
    last = None
    for attempt in range(retries + 1):
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                data = json.loads(r.read().decode())
            if "error" in data:
                last = data["error"].get("message", str(data["error"]))
                # rpm exhaustion -> brief backoff and retry
                time.sleep(20 * (attempt + 1))
                continue
            return data["choices"][0]["message"], None
        except (urllib.error.URLError, TimeoutError, KeyError, json.JSONDecodeError) as e:
            last = str(e)
            time.sleep(5)
    return None, last


class NLInterface(Node):
    def __init__(self):
        super().__init__("nlra_nl_interface")
        self._cb = ReentrantCallbackGroup()
        self._exec = ActionClient(self, ExecuteTask, "orchestrator/execute_task",
                                  callback_group=self._cb)
        self._get_objects = self.create_client(GetObjects, "world_model/get_objects",
                                               callback_group=self._cb)
        self._get_grasp_pose = self.create_client(
            GetGraspPose, "world_model/get_grasp_pose", callback_group=self._cb)
        self.create_service(NLCommand, "nl_command", self._on_command,
                            callback_group=self._cb)
        if not LLM_KEY:
            self.get_logger().warn("NLRA_LLM_KEY not set — LLM calls will fail")
        self.get_logger().info(f"NL interface up (model {LLM_MODEL} @ {LLM_BASE})")

    def _world_objects(self, timeout=5.0):
        if not self._get_objects.wait_for_service(timeout_sec=timeout):
            return None
        fut = self._get_objects.call_async(GetObjects.Request(kind_filter=""))
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return None
            time.sleep(0.05)
        return fut.result().objects

    def _describe_objects(self, objs):
        lines = []
        for o in objs:
            p = o.pose.pose.position
            q = o.pose.pose.orientation
            lines.append(
                f"- {o.id} | {o.kind} | graspable={o.graspable} | "
                f"size={[round(s, 3) for s in o.size]} | "
                f"pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) | "
                f"quat=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")
        return "\n".join(lines) or "(none)"

    def _dispatch(self, task, args_json, timeout=300.0):
        if not self._exec.wait_for_server(timeout_sec=5.0):
            return False, "orchestrator unavailable"
        goal = ExecuteTask.Goal(task=task, args_json=args_json)
        fut = self._exec.send_goal_async(goal)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 10:
                return False, "orchestrator did not accept goal"
            time.sleep(0.05)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False, "goal rejected"
        rfut = handle.get_result_async()
        t0 = time.monotonic()
        while not rfut.done():
            if time.monotonic() - t0 > timeout:
                handle.cancel_goal_async()
                return False, "task timed out"
            time.sleep(0.2)
        res = rfut.result().result
        return bool(res.success), res.message

    def _answer_grasp_pose(self, resp, args):
        """Answer a 'where/how would you grasp X?' query from the world model."""
        obj_id = str(args.get("object_id", "")).strip()
        if not obj_id:
            resp.error = "get_grasp_pose requires an object_id"
            return resp
        if not self._get_grasp_pose.wait_for_service(timeout_sec=5.0):
            resp.error = "grasp pose service unavailable"
            return resp
        fut = self._get_grasp_pose.call_async(
            GetGraspPose.Request(object_id=obj_id))
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 5.0:
                resp.error = "grasp pose query timed out"
                return resp
            time.sleep(0.05)
        res = fut.result()
        if not res.found:
            resp.error = res.message
            return resp
        p = res.grasp_pose.pose.position
        resp.success = True
        resp.response = (f"{res.message}; grasp point at "
                         f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m in the world frame")
        return resp

    def _on_command(self, req, resp):
        text = req.text.strip()
        self.get_logger().info(f"NL command: {text!r}")
        resp.success = False
        resp.task = ""
        resp.args_json = ""
        resp.llm_trace_json = ""

        MAX_ROUNDS = 3
        last_error = ""
        trace = []
        for round_ in range(1, MAX_ROUNDS + 1):
            objs = self._world_objects()
            if objs is None:
                resp.error = "world model unavailable"
                resp.llm_trace_json = json.dumps(trace)
                return resp
            system = SYSTEM_TMPL.format(objects=self._describe_objects(objs))

            user = text
            if round_ > 1:
                user = (f"{text}\n\n[The previous attempt failed: {last_error}. "
                        "The world state above is current. Produce an updated "
                        "plan (e.g. different drop positions or step order) "
                        "and retry.]")

            msg, err = llm_chat(
                [{"role": "system", "content": system},
                 {"role": "user", "content": user}], tools=TOOLS)
            trace.append({"round": round_, "system": system, "user": user,
                          "assistant": msg})
            if msg is None:
                resp.error = f"LLM error: {err}"
                resp.llm_trace_json = json.dumps(trace)
                return resp

            calls = msg.get("tool_calls") or []
            if not calls:
                # No function call: informational / refusal answer
                resp.success = True
                resp.response = (msg.get("content") or "").strip()
                resp.llm_trace_json = json.dumps(trace)
                return resp

            try:
                fn = calls[0]["function"]
                args = json.loads(fn.get("arguments") or "{}")
            except (KeyError, json.JSONDecodeError) as e:
                resp.error = f"bad tool call from LLM: {e}"
                resp.llm_trace_json = json.dumps(trace)
                return resp

            if fn.get("name") == "get_grasp_pose":
                resp.llm_trace_json = json.dumps(trace)
                return self._answer_grasp_pose(resp, args)

            if fn.get("name") == "execute_plan":
                steps = args.get("steps") or []
                task = "plan"
                args_json = json.dumps({"steps": steps})
            else:
                task = args.pop("task", "")
                args_json = json.dumps(args)
            self.get_logger().info(
                f"grounded (round {round_}): task={task} args={args_json}")

            ok, msg_txt = self._dispatch(task, args_json)
            if ok:
                resp.success = True
                resp.task = task
                resp.args_json = args_json
                resp.response = msg_txt
                resp.error = ""
                resp.llm_trace_json = json.dumps(trace)
                return resp

            last_error = msg_txt
            self.get_logger().warn(f"round {round_} failed: {msg_txt}")

        resp.success = False
        resp.error = f"{last_error} (after {MAX_ROUNDS} attempts)"
        resp.llm_trace_json = json.dumps(trace)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = NLInterface()
    executor = MultiThreadedExecutor(num_threads=6)
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
