# NL-Driven Robot Arm — Architecture & Implementation Plan

**Project:** Natural-language-commanded manipulation with an orchestrator + skills
**Status:** Implemented and verified in sim — Phases 0–7 complete (headless, aarch64 + x86_64)
**Scope:** Simulation only

---

## 1. Locked Baseline

| Layer | Choice | Source of truth |
|-------|--------|-----------------|
| Robot | KUKA Agilus KR 16 R1100-3 | vendored URDF + meshes in `agilus_robotiq_description` (from `kroshu/kuka_robot_descriptions`, Apache-2.0) |
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
│   move_joints  move_to  move_relative  move_axis               │
│   grasp  release  home                                         │
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
3. **Orchestrator** plans: `move_to(approach) → move_to(grasp) → grasp(red_cube) → move_to(lift) → move_to(transfer) → move_to(lower) → release → home`
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
│   └── how-to-use.md              # environment setup + launch how-to
├── src/
│   ├── agilus_robotiq_description/        # combined URDF (Agilus KR 16 R1100-3 + Robotiq 2F-85), Gazebo worlds, ros2_control, spawn launch
│   ├── agilus_robotiq_moveit_config/      # MoveIt 2 config (move_group launch, SRDF, OMPL, kinematics, controllers)
│   ├── nlra_interfaces/                  # .action / .srv / .msg (MoveJoints, MoveTo, MoveRelative, MoveAxis, Grasp, Release, Home, ExecuteTask, NLCommand, ...)
│   ├── nlra_motion_planner/              # MoveItPy wrapper: plan_to_pose, plan_relative_cartesian, plan_to_joint_target, execute_trajectory
│   ├── nlra_gz_grasp_fix/                # Gazebo Harmonic grasp-fix plugin (detachable joint on fingertip contact)
│   ├── nlra_skills/                      # skill action servers (move_joints, move_to, move_relative, move_axis, grasp, release, home)
│   ├── nlra_world_model/                 # world model services (get_objects, get_object_pose, get_grasp_pose) — ground-truth poses
│   ├── nlra_orchestrator/                # task planner: grounding + MoveIt-backed skill sequencing + retry + postcondition
│   ├── nlra_nl_interface/                # LLM function-calling NL front-end (/nl_command service)
│   ├── nlra_web_ui/                       # web UI: manual control, NL chat, diagnostics (rosbridge + vanilla JS)
│   └── nlra_bringup/                     # single ros2 launch for the whole stack
```
Note (as-built vs. original plan): a separate vision `nlra_perception` package
remains deferred — object poses come from Gazebo ground truth (per-model
PosePublisher → ros_gz_bridge). Motion is MoveIt-backed: `nlra_motion_planner`
wraps MoveItPy and plans Cartesian pose / relative / joint targets that the
skill servers execute through `arm_controller`. `nlra_bringup` provides
one-command startup.

---

## 5. Implementation Tasks (phased)

Each phase produces something runnable. **STATUS: Phases 0–7 all COMPLETE and
verified in sim (headless, aarch64 + x86_64). As-built notes inline below.**

### Phase 0 — Environment & scaffolding — ✅ DONE
- [x] ROS2 Jazzy + Gazebo Harmonic + MoveIt 2 via **pixi/RoboStack** (not bare Ubuntu; runs on aarch64 + x86_64)
- [x] Vendor Agilus KR 16 R1100-3 URDF macro + meshes into `agilus_robotiq_description` (no `kuka_robot_descriptions` dependency). **kuka_drivers NOT used — sim-only.**
- [x] Colcon workspace + repo structure
- **Done when:** `colcon build` succeeds; Agilus URDF valid. ✅

### Phase 1 — Robot in sim (motion foundation) — ✅ DONE (10/10 checks)
- [x] `agilus_robotiq_description`: Robotiq 2F-85 on Agilus flange, one hand-written ros2_control block + one gz plugin
- [x] Gazebo spawn with ros2_control (arm_controller, gripper_controller, joint_state_broadcaster all active)
- [x] Gripper mimic solved via **bullet-featherstone** physics + 5 mimic joints
- **Done when:** joint trajectory executes in Gazebo; gripper opens/closes. ✅

### Phase 2 — Skill layer — ✅ DONE (12/12 checks)
- [x] `nlra_interfaces`: MoveJoints, Grasp, Release, Home actions (+ MoveTo, MoveRelative, MoveAxis, ExecuteTask, NLCommand later)
- [x] `nlra_skills`: each an action server; **client-side gripper stall-detection** (controller never completes a blocked goal)
- **Done when:** each skill callable via CLI; pick+place works with hardcoded poses. ✅

### Phase 3 — World Model + perception — ✅ DONE (4/4 checks)
- [x] Tabletop scene + colored objects (red_cube dynamic, blue_box tray, table)
- [x] World poses via **per-model PosePublisher** (ground truth); static models need `pose_static` topic
- [x] `nlra_world_model`: `get_objects` / `get_object_pose` / `get_grasp_pose` services
- [x] `get_grasp_pose` answers "where/how to grasp an object" and by default returns the gripper parallel to the object (approach from above, finger closing axis aligned with the object's yaw); pick uses it, so pick grasps are orientation-aligned instead of yaw-agnostic
- Perception is ground-truth from sim (camera-sensor render segfaults headless w/o GPU); vision-based detection deferred
- **Done when:** World Model reports live object poses matching the scene. ✅

### Phase 4 — Orchestrator — ✅ DONE (8/8 checks)
- [x] `nlra_orchestrator`: `ExecuteTask` action; grounds object symbols through the world model, computes absolute approach/grasp/lift poses in `base_link` from the world-model grasp pose, and sequences **MoveIt-backed skills** (`move_to`, `move_relative`, `grasp`, `release`, `home`) with per-step feedback + typed error codes
- [x] Tasks: home, pick, place, pick_and_place, move_joints
- **Done when:** structured task runs full monitored pick-and-place. ✅

### Phase 5 — NL interface — ✅ DONE (7/7 checks)
- [x] `nlra_nl_interface`: LLM function-calling (`/nl_command` service) onto `ExecuteTask`; system prompt grounded from live world-model object list
- [x] LLM: OpenAI-compatible endpoint, model Qwen3.6-35B-A3B (function-calling verified); urllib client with rpm-backoff retry
- **Done when:** "put the red cube in the blue tray" runs end-to-end (incl. German + synonym grounding + refusal path). ✅

### Phase 6 — Robustness & polish — ✅ DONE
- [x] `nlra_bringup`: **single `ros2 launch` file** for the whole stack (sim → NL), staged timers
- [x] Orchestrator **retry with re-grounding** + **postcondition verification** (object actually at target via world model)
- [x] **Grasp: physical (friction) grip + grasp-fix plugin.** The gripper is a position-servo (gz_ros2_control velocity-based) drive; the grasp skill detects a blocked close and retains a small over-close command until release, maintaining the normal force for friction. The `nlra_gz_grasp_fix` gz plugin (Gazebo Harmonic port of gazebo_grasp_fix) additionally creates a fixed detachable joint from the palm to a dynamic object when both fingertip contact sensors touch it, and removes it once both contacts are absent long enough for the gripper to open — so lifts do not rely on friction alone. Fingertip collision volumes match the asymmetric rendered meshes, and the grasp target is calibrated from those visible pad faces so the claws do not appear inside an object. Verified: cube physically transported to tray, postcondition passes.
- [x] Honest NL failure reporting (no false success)
- **Done when:** NL commands run reliably; failures reported clearly. ✅

### Phase 7 — MoveIt 2 Cartesian planning — ✅ DONE (10/10 checks)
- [x] `agilus_robotiq_moveit_config`: MoveIt 2 config for the Agilus arm (move_group launch, SRDF, OMPL, kinematics, joint limits, controller mapping)
- [x] `nlra_motion_planner`: MoveItPy wrapper — `plan_to_pose` (absolute Cartesian goal, `pose_link="tool0"`), `plan_relative_cartesian` (Jazzy moveit_py has no `compute_cartesian_path` binding, so relative deltas are applied to the current EE pose from TF and planned as absolute goals), `plan_to_joint_target`, and `execute_trajectory` via `/arm_controller/follow_joint_trajectory`
- [x] Skills `move_to` / `move_relative` / `move_axis` delegate planning to an embedded `MotionPlanner` node (no standalone planner node in the stack; `skill_servers` spins both nodes)
- [x] Full NL pick-and-place verified 10/10 on fresh boot; fcl rebuilt against Eigen 5 to fix an arm64 ABI crash in MoveIt (`scripts/rebuild_fcl.sh` + `patches/`)
- **Done when:** pick-and-place runs end-to-end through MoveIt Cartesian planning. ✅

---

## 6. Skill Interface Contract (draft)

Each skill is a ROS2 action served by `nlra_skills` (`skills/move_joints`,
`skills/move_to`, `skills/move_relative`, `skills/move_axis`, `skills/grasp`,
`skills/release`, `skills/home`). Example `MoveTo.action`:

```
# Goal
geometry_msgs/PoseStamped target   # EE (tool0) pose in a named frame (world/base_link)
float32 velocity_scaling 0.2       # 0..1, fraction of joint velocity limits
float32 acceleration_scaling 0.2
---
# Result
bool success
int32 error_code            # 0 ok, 1 precondition_failed, 2 planning_failed, 3 execution_failed, 4 cancelled
string message
---
# Feedback
string phase                # e.g. "validating", "planning", "executing"
float32 progress            # 0.0 - 1.0 (best-effort)
```

Preconditions/postconditions (checked in the server):
- **Pick** (orchestrated `move_to` + `grasp`) pre: object known + reachable, gripper open. post: object gripped (grasp-fix attaches), gripper closed.
- **Place** (orchestrated `move_to` + `release`) pre: object attached, destination reachable. post: object released at destination.

The high-level `ExecuteTask` action (`orchestrator/execute_task`) takes a task
name (`home` | `pick` | `place` | `pick_and_place` | `move_joints`) plus a JSON
args blob and streams per-step feedback (`step`, `step_index`, `progress`).

---

## 7. Open Items / Future
Phases 0–7 are complete in sim. Remaining/optional work:
- **Vision-based perception** (sim camera → detection) to replace ground-truth poses — blocked headless by GPU/EGL (camera render segfaults); needs a GPU host or software-render pipeline.
- More objects / multi-step & conditional NL commands; conversational multi-turn context.
- Local-LLM option for offline NL (currently cloud, model Qwen3.6-35B-A3B).
- Isaac Sim upgrade path for photoreal perception / learned grasping.
- Real Agilus bring-up via `kuka_rsi_driver` (KRC5/RSI) — the ros2_control HW abstraction keeps the whole stack above the driver unchanged.
- Formal test suite (current verification is ad-hoc scripted checks, all passing).
- Multi-object / multi-step task planning, spatial reasoning ("left of", "on top of").
