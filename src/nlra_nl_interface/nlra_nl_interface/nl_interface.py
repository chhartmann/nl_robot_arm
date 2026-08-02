"""NL Robot Arm — natural-language interface (Phase 5).

Exposes /nl_command (nlra_interfaces/srv/NLCommand). Each request:
  1. queries the world model for the live object list,
  2. asks the LLM (OpenAI-compatible endpoint) to ground the utterance
     into ONE execute_task function call (or a plain-text answer),
  3. dispatches the grounded task to /orchestrator/execute_task,
  4. returns task result + a natural-language response.

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

TOOLS = [{
    "type": "function",
    "function": {
        "name": "execute_task",
        "description": (
            "Execute a manipulation task on the robot arm. "
            "pick_and_place moves an object into/onto a target. "
            "pick only grasps and lifts. place puts a held object on a target. "
            "home returns the arm to its rest pose. "
            "move_joints moves individual joints to given angles."),
        "parameters": {
            "type": "object",
            "properties": {
                "task": {"type": "string",
                         "enum": ["home", "pick", "place", "pick_and_place",
                                  "move_joints"]},
                "object_id": {"type": "string",
                              "description": "id of the object to manipulate"},
                "target_id": {"type": "string",
                              "description": "id of the destination object"},
                "joints": {"type": "object",
                           "description": ("only for move_joints: map of joint "
                                           "id -> target angle in degrees, e.g. "
                                           "{\"a1\": 90, \"a3\": -45}")},
            },
            "required": ["task"],
        },
    },
    },
    {
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
    },
]

SYSTEM_TMPL = (
    "You are the command interface of a KUKA Agilus KR 16 R1100-3 robot arm "
    "with a Robotiq "
    "gripper working on a tabletop. Ground the user's command into exactly "
    "one execute_task function call using ONLY the object ids listed below. "
    "Users speak informally: resolve descriptions, synonyms and translations "
    "to the closest matching object id (e.g. 'the tray'/'the container'/'die "
    "blaue Schale' -> blue_box, 'the block'/'der Wuerfel' -> red_cube). Only "
    "if no listed object plausibly matches, or the request is not a "
    "manipulation command, do NOT call a function; answer briefly in plain "
    "text instead (same language as the user).\n\n"
    "For joint motion use move_joints with 'joints': {{\"a1\": 90}}. Joint ids "
    "are a1..a6 (a.k.a. joint_1..joint_6), angles in degrees, absolute "
    "targets (not relative), within limits: a1: -170..170; a2: -195..55; "
    "a3: -115..165; a4: -200..200; a5: -120..120; a6: -350..350.\n\n"
    "Objects currently visible (id | kind | graspable | position):\n{objects}")


def llm_chat(messages, tools=None, timeout=90, retries=2):
    """Minimal OpenAI-compatible chat call via urllib (no SDK needed)."""
    body = {"model": LLM_MODEL, "messages": messages, "max_tokens": 400}
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

        objs = self._world_objects()
        if objs is None:
            resp.error = "world model unavailable"
            return resp
        lines = []
        for o in objs:
            p = o.pose.pose.position
            lines.append(f"- {o.id} | {o.kind} | graspable={o.graspable} | "
                         f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        system = SYSTEM_TMPL.format(objects="\n".join(lines) or "(none)")

        msg, err = llm_chat(
            [{"role": "system", "content": system},
             {"role": "user", "content": text}], tools=TOOLS)
        if msg is None:
            resp.error = f"LLM error: {err}"
            return resp

        calls = msg.get("tool_calls") or []
        if not calls:
            # No function call: informational / refusal answer
            resp.success = True
            resp.response = (msg.get("content") or "").strip()
            return resp

        try:
            fn = calls[0]["function"]
            args = json.loads(fn.get("arguments") or "{}")
        except (KeyError, json.JSONDecodeError) as e:
            resp.error = f"bad tool call from LLM: {e}"
            return resp

        if fn.get("name") == "get_grasp_pose":
            return self._answer_grasp_pose(resp, args)

        task = args.pop("task", "")
        args_json = json.dumps(args)
        self.get_logger().info(f"grounded: task={task} args={args_json}")

        ok, msg_txt = self._dispatch(task, args_json)
        resp.success = ok
        resp.task = task
        resp.args_json = args_json
        resp.response = msg_txt if ok else ""
        resp.error = "" if ok else msg_txt
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
