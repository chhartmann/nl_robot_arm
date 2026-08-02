# NL-Driven Robot Arm — Architecture & Implementation Plan

**Project:** Natural-language-commanded manipulation with an orchestrator + skills
**Status:** Concept aligned; ready to start implementation
**Scope:** Simulation only

---

## 1. Locked Baseline

| Layer | Choice | Source of truth |
|-------|--------|-----------------|
| Robot | KUKA Agilus KR 16 R1100-3 | `kroshu/kuka_robot_descriptions` → `kuka_agilus_support` |
| Gripper | Robotiq 2F-85 | Robotiq ROS2 description + controller |
| ROS2 distro | Jazzy (Ubuntu 24.04) | KUKA's own recommendation |
| Simulation | Gazebo Harmonic | ROS2-native, ros2_control integration |
| Motion planning | MoveIt 2 (`moveit_py`) | — |
| Hardware abstraction | ros2_control | sim/real parity (real = `kuka_iiqka_eac_driver`) |
| NL layer | Cloud LLM function-calling | grounded in World Model symbols |
| Perception | Sim camera → object poses | color/tag detection |
| Scope | Simulation only | — |

> Sim/real parity is preserved by design: the whole stack above the driver is
> identical. If real Agilus hardware is ever added, swap Gazebo for
> `kuka_rsi_driver` (KRC5) — a config change, not a rewrite.

---

## 2. Architecture — Three Planes

```
NL command
   │
   ▼
┌───────────────────────────────────────────────────────────────┐
│  1. COGNITIVE PLANE  (natural language → plan)                 │
│   NL input ─► LLM (function-calling) ─► Task Plan ─► Orchestrator│
│                          │                                     │
│                   World Model / Knowledge                      │
│                   (objects, poses, robot/gripper state)        │
└─────────────────────────────────┬─────────────────────────────┘
                                   │ typed skill goals (ROS2 actions)
┌─────────────────────────────────▼─────────────────────────────┐
│  2. SKILL PLANE  (ROS2 action servers)                         │
│   move_to  pick  place  grasp  release  scan_scene  home       │
│   • preconditions • execution • postconditions • feedback      │
└─────────────────────────────────┬─────────────────────────────┘
                                   │ MoveIt / controller calls
┌─────────────────────────────────▼─────────────────────────────┐
│  3. EXECUTION PLANE                                             │
│   MoveIt 2 ─► ros2_control ─► Gazebo (Agilus KR 16 R1100-3 + Robotiq 2F-85) │
│   Perception (sim camera → poses) ─► World Model               │
│   TF2 (frames)                                                 │
└───────────────────────────────────────────────────────────────┘
```

### Command flow (worked example)
> **User:** "Pick up the red cube and put it in the blue box."

1. **LLM** parses → intent `relocate`, params `{object: red_cube, destination: blue_box}`
2. **World Model** resolves symbols → poses for `red_cube`, `blue_box`
3. **Orchestrator** plans: `scan_scene → move_to(pre-grasp) → grasp(red_cube) → move_to(pre-place) → release → home`
4. Each **skill** (action server) checks preconditions → calls MoveIt → streams feedback → returns success/failure+reason
5. **Orchestrator monitors**: on failure (e.g. unreachable IK) → replan or report in NL
6. **Response:** "Done — red cube is now in the blue box."

---

## 3. Component Responsibilities

| Component | Responsibility | Tech |
|-----------|---------------|------|
| NLU/Planner | NL → structured intent → ordered skill plan | Cloud LLM function-calling |
| Orchestrator | Sequence skills, monitor, recover, replan, report | Python node; optional `py_trees` |
| World Model | Ground truth: objects + poses + robot/gripper state | ROS2 services + blackboard |
| Skills | Self-contained, typed, reusable capabilities | ROS2 action servers |
| MoveIt 2 | Planning, IK, collision-aware trajectories | `moveit_py` |
| Perception | Sim camera → detected object poses | Gazebo camera + CV/tag |
| Sim | Physics, robot, gripper, scene | Gazebo Harmonic + ros2_control |

### Design principles (committed)
1. **Skills = ROS2 action servers** with explicit pre/postconditions → async, cancellable, monitorable.
2. **Orchestrator plans, never actuates** → planning decoupled from real-time control.
3. **World Model is the grounding layer** → single source of truth; turns language into poses.
4. **Two-stage NL:** LLM proposes structured plan; deterministic orchestrator disposes. Debuggable + safer.
5. **Everything typed** → skill goals are ROS2 `.action` contracts.

---

## 4. Repository Layout (as-built)

```
nl_robot_arm/                      # colcon workspace root (pixi-managed)
├── docs/
│   ├── ARCHITECTURE.md            # this file
│   ├── arc42-architecture.html    # arc42 architecture doc (HTML)
│   └── agilus_robotiq_demo.mp4     # NL pick-and-place demo (software render)
├── src/
│   ├── agilus_robotiq_description/  # combined URDF (Agilus KR 16 R1100-3 + Robotiq 2F-85), world, ros2_control, spawn launch
│   ├── nlra_interfaces/           # .action / .srv / .msg (MoveJoints, Grasp, Release, Home, MoveTo, ExecuteTask, NLCommand, ...)
│   ├── nlra_skills/               # skill action servers (move_joints, grasp, release, home)
│   ├── nlra_world_model/          # world model services (get_objects, get_object_pose) — ground-truth poses
│   ├── nlra_orchestrator/         # task planner: grounding + numeric IK + skill sequencing + retry + postcondition
│   ├── nlra_nl_interface/         # LLM function-calling NL front-end (/nl_command)
│   └── nlra_bringup/              # single ros2 launch for the whole stack
└── .demo/                         # trace recorder + software-render demo scripts
```
Note (as-built vs. original plan): MoveIt config and a separate vision `nlra_perception`
package were deferred — motion uses a numeric IK in the orchestrator, and object poses
come from Gazebo ground truth. `nlra_bringup` was added for one-command startup.

---

## 5. Implementation Tasks (phased)

Each phase produces something runnable. **STATUS: Phases 0–6 all COMPLETE and
verified in sim (headless, aarch64). As-built notes inline below.**

### Phase 0 — Environment & scaffolding — ✅ DONE
- [x] ROS2 Jazzy + Gazebo Harmonic + MoveIt 2 via **pixi/RoboStack** (not bare Ubuntu; runs on aarch64 + x86_64)
- [x] Clone/build `kuka_robot_descriptions` (Agilus). **kuka_drivers NOT used — sim-only.**
- [x] Colcon workspace + repo structure
- **Done when:** `colcon build` succeeds; Agilus URDF valid. ✅

### Phase 1 — Robot in sim (motion foundation) — ✅ DONE (10/10 checks)
- [x] `agilus_robotiq_description`: Robotiq 2F-85 on Agilus flange, one hand-written ros2_control block + one gz plugin
- [x] Gazebo spawn with ros2_control (arm_controller, gripper_controller, joint_state_broadcaster all active)
- [x] Gripper mimic solved via **bullet-featherstone** physics + 5 mimic joints
- **Done when:** joint trajectory executes in Gazebo; gripper opens/closes. ✅

### Phase 2 — Skill layer — ✅ DONE (12/12 checks)
- [x] `nlra_interfaces`: MoveJoints, Grasp, Release, Home actions (+ MoveTo, ExecuteTask, NLCommand later)
- [x] `nlra_skills`: each an action server; **client-side gripper stall-detection** (controller never completes a blocked goal)
- **Done when:** each skill callable via CLI; pick+place works with hardcoded poses. ✅

### Phase 3 — World Model + perception — ✅ DONE (4/4 checks, demo MP4)
- [x] Tabletop scene + colored objects (red_cube dynamic, blue_box tray, table)
- [x] World poses via **per-model PosePublisher** (ground truth); static models need `pose_static` topic
- [x] `nlra_world_model`: `get_objects` / `get_object_pose` services
- Perception is ground-truth from sim (camera-sensor render segfaults headless w/o GPU); vision-based detection deferred
- **Done when:** World Model reports live object poses matching the scene. ✅

### Phase 4 — Orchestrator — ✅ DONE (8/8 checks)
- [x] `nlra_orchestrator`: `ExecuteTask` action; grounds object → **numeric IK** (yourdfpy FK + scipy least-squares, 0.0 mm err, tool-down); sequences skills with per-step feedback + typed error codes
- [x] Tasks: home, pick, place, pick_and_place
- **Done when:** structured task runs full monitored pick-and-place. ✅

### Phase 5 — NL interface — ✅ DONE (7/7 checks)
- [x] `nlra_nl_interface`: LLM function-calling (`/nl_command` service) onto `ExecuteTask`; system prompt grounded from live world-model object list
- [x] LLM: OpenAI-compatible endpoint, model Qwen3.6-35B-A3B (function-calling verified); urllib client with rpm-backoff retry
- **Done when:** "put the red cube in the blue tray" runs end-to-end (incl. German + synonym grounding + refusal path). ✅

### Phase 6 — Robustness & polish — ✅ DONE
- [x] `nlra_bringup`: **single `ros2 launch` file** for the whole stack (sim → NL), staged timers
- [x] Orchestrator **retry with re-grounding** + **postcondition verification** (object actually at target via world model)
- [x] **Grasp physics: DetachableJoint** attach/detach (bullet-featherstone ignores contact friction) — verified cube transported to tray, postcondition 5.1 cm
- [x] Honest NL failure reporting (no false success)
- **Done when:** NL commands run reliably; failures reported clearly. ✅

---

## 6. Skill Interface Contract (draft)

Each skill is a ROS2 action. Example `Pick.action`:

```
# Goal
string object_id            # symbol resolved via World Model
float64 approach_distance    # pre-grasp standoff (m)
---
# Result
bool success
string message               # human-readable reason on failure
---
# Feedback
string stage                 # e.g. "approaching", "grasping", "retreating"
float32 progress             # 0.0 - 1.0
```

Preconditions/postconditions (checked in the server):
- **Pick** pre: object known + reachable, gripper open. post: object attached, gripper closed.
- **Place** pre: object attached, destination reachable. post: object detached at destination.

---

## 7. Open Items / Future
Phases 0–6 are complete in sim. Remaining/optional work:
- **MoveIt Cartesian planning** to replace the analytic/numeric top-down IK (collision-aware, arbitrary orientations).
- **Vision-based perception** (sim camera → detection) to replace ground-truth poses — blocked headless by GPU/EGL (camera render segfaults); needs a GPU host or software-render pipeline.
- More objects / multi-step & conditional NL commands; conversational multi-turn context.
- Local-LLM option for offline NL (currently cloud, model Qwen3.6-35B-A3B).
- Isaac Sim upgrade path for photoreal perception / learned grasping.
- Real Agilus bring-up via `kuka_rsi_driver` (KRC5/RSI) — the ros2_control HW abstraction keeps the whole stack above the driver unchanged.
- Formal test suite (current verification is ad-hoc scripted checks, all passing).
- Multi-object / multi-step task planning, spatial reasoning ("left of", "on top of").
