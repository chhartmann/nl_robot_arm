# Agent Guide: `nl_robot_arm`

This repository is a **pixi-managed**, simulation-only ROS 2 workspace for commanding a KUKA
Agilus KR 16 R1100-3 arm with a Robotiq 2F-85 gripper using natural language.
The stack runs in Gazebo Harmonic on ROS 2 Jazzy and uses **Pixi/RoboStack** for
reproducible cross-platform (aarch64 + x86_64) dependency management.

## Pixi Quick Start

```bash
# Install dependencies and build
pixi install
pixi run build

# Enter activated shell (sources install/setup.sh, sets Gazebo paths)
pixi shell

# Run full stack
pixi run sim_headless
```

All commands below assume you are inside `pixi shell` (or have run `pixi run`).

## Start Here

Read these files before changing behavior:

- `docs/ARCHITECTURE.md`: system design, component responsibilities, interfaces,
  completed phases, and future work.
- `docs/how-to-use.md`: environment setup, launch commands, and runtime checks.
- `pixi.toml`: ROS/Python dependencies, environment variables, and build tasks.
- `src/nlra_bringup/launch/nlra.launch.py`: full-stack startup order and launch
  arguments.

## Important Scope and Constraints

- The project targets ROS 2 Jazzy, Gazebo Harmonic, and Python 3.12.
- The supported execution target is simulation. Do not add real hardware drivers
  or assume a connected KUKA controller.
- The robot description and meshes are vendored in
  `src/agilus_robotiq_description`; a separate KUKA description checkout is not
  required.
- The orchestrator grounds object symbols through the world model and sequences
  MoveIt-backed skills; motion is planned by `nlra_motion_planner` (MoveItPy)
  and executed via the arm controller.
- Object poses come from Gazebo ground truth through `ros_gz_bridge`; camera
  perception is intentionally deferred.
- Skills are ROS 2 action servers. The orchestrator plans and sequences skills;
  it should not directly actuate controllers.
- The world model is the grounding source of truth for object IDs and poses.
- Do not commit `.env`, API keys, generated ROS artifacts, or changes to files
  unrelated to the task.

## Repository Map

```text
docs/
  ARCHITECTURE.md                 Design and implementation status
  how-to-use.md                   Operational commands
src/
  agilus_robotiq_description/     URDF, meshes, Gazebo worlds, controllers
  agilus_robotiq_moveit_config/   MoveIt 2 config (move_group, SRDF, OMPL)
  nlra_interfaces/                ROS messages, services, and actions
  nlra_motion_planner/            MoveItPy wrapper (plan_to_pose, relative/joint, execute)
  nlra_gz_grasp_fix/              Gazebo grasp-fix plugin (detachable joint on contact)
  nlra_skills/                    Move/grasp/release/home action servers
  nlra_world_model/               Ground-truth object and grasp-pose services
  nlra_orchestrator/              Task grounding, MoveIt-backed sequencing/retry
   nlra_nl_interface/              LLM-backed /nl_command service
   nlra_web_ui/                     Web UI: manual control, NL chat, diagnostics
   nlra_bringup/                   Single launch file for the complete stack
```

Generated or local-only directories include `build/`, `install/`, `log/`, and
`.pixi/`. They should not be edited or used as the source of truth.

## Environment Setup

From the repository root, all development happens inside the Pixi environment:

```bash
# One-time setup: install ROS/Python deps, build workspace
pixi install
pixi run build

# Enter activated shell — this sources install/setup.sh and configures
# Gazebo plugin/resource paths. Use this for all interactive work.
pixi shell

# Or run one-off commands without the shell:
pixi run ros2 launch nlra_bringup nlra.launch.py
```

`pixi install` resolves ROS and Python dependencies from `pixi.toml` (RoboStack channels).
`pixi run build` executes:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The Pixi activation sources `install/setup.sh` and configures Gazebo plugin and
resource paths. The first `pixi install` may warn that `install/setup.sh` does
not exist yet; build the workspace before relying on the overlay.

If dependency resolution needs to be refreshed, use the repository task:

```bash
pixi run deps
```

This runs `rosdep install --from-paths src --ignore-src -r -y` while skipping
the Pixi-provided desktop metapackage.

## Running the Stack

All launch commands assume you are inside `pixi shell` (which sources the
overlay). The normal full-stack command is headless and uses the tabletop scene:

```bash
ros2 launch nlra_bringup nlra.launch.py
```

Useful variants:

```bash
# Enable Gazebo GUI
ros2 launch nlra_bringup nlra.launch.py gui:=true

# Run without the LLM/NL interface
ros2 launch nlra_bringup nlra.launch.py nl:=false

# Select a packaged world
ros2 launch nlra_bringup nlra.launch.py world:=empty_bullet.sdf
```

You can also run directly via Pixi without an interactive shell:

```bash
pixi run ros2 launch nlra_bringup nlra.launch.py
```

The full launch starts components in stages: Gazebo and the robot first, the
pose bridge at about 25 seconds, world model and MoveIt `move_group` at about
30 seconds, skill servers at about 40 seconds, orchestrator at about 45
seconds, and the NL interface + web UI at about 50–55 seconds. Allow the
stack to finish starting before sending commands.

The robot-only launch is also available:

```bash
ros2 launch agilus_robotiq_description spawn_agilus_robotiq.launch.py \
  gui:=false world:=tabletop_scene.sdf
```

The standalone skill launch is useful when Gazebo is already running:

```bash
ros2 launch nlra_skills skills.launch.py
```

The NL interface exposes an `/nl_command` service. The web UI connects via
rosbridge and provides manual control, NL chat, and diagnostics. It requires
an LLM key for actual language commands.

## NL Configuration

The NL node loads a workspace-root `.env` using `python-dotenv`. The file is
ignored by Git. Configure these variables in the shell or `.env`:

```text
NLRA_LLM_KEY=replace-with-secret
NLRA_LLM_BASE=https://api.hcnsec.cn/v1
NLRA_LLM_MODEL=Qwen3.6-35B-A3B
```

`NLRA_LLM_KEY` is required. The base URL and model have defaults in
`src/nlra_nl_interface/nlra_nl_interface/nl_interface.py`. Never place a real
key in source, documentation, or commits.

## Runtime Inspection and Smoke Checks

With the stack running, use:

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 service list
ros2 control list_controllers
ros2 topic echo /joint_states
```

The basic environment check is:

```bash
pixi run ros_check
```

For a build-only verification, rerun `pixi run build`. There is currently no
formal `pytest`, `ament_cmake_pytest`, or launch-testing suite in the repository;
runtime verification is performed with ROS CLI checks and the simulation.

To stop the stack, press `Ctrl-C` in the launch terminal. If a Gazebo process
survives and prevents a clean restart, inspect it and then use:

```bash
pkill -f 'gz sim'
```

## Interfaces and Data Flow

ROS contracts live in `src/nlra_interfaces`:

- Actions: `MoveTo`, `MoveRelative`, `MoveAxis`, `MoveJoints`, `Grasp`,
  `Release`, `Home`, `ExecuteTask`.
- Services: `GetObjects`, `GetObjectPose`, `GetGraspPose`, `NLCommand`.
- Message: `WorldObject`.

Typical command flow:

1. `/nl_command` queries the world model for live object IDs and poses.
2. The OpenAI-compatible LLM maps the user text to one typed task/tool call.
3. The NL interface dispatches `/orchestrator/execute_task`.
4. The orchestrator grounds symbols, builds approach/grasp/lift poses, and
   calls the MoveIt-backed skill action servers.
5. Skills plan through `nlra_motion_planner` (MoveItPy) and drive the arm and
   gripper controllers, returning typed success/failure.
6. The orchestrator verifies postconditions through the world model and reports
   failure rather than claiming success.

When changing a message, service, or action, update its definition first, rebuild
the workspace, and then update all clients and servers. Generated interface
artifacts belong in the build/install outputs, not in source.

## Development Workflow

1. Inspect the relevant package, launch file, interface, and architecture notes.
2. Make the smallest source change that preserves the three-plane separation:
   cognitive/NL, skill actions, and Gazebo execution.
3. Rebuild with `pixi run build`.
4. Run the affected node or the full launch headlessly.
5. Check nodes, controllers, topics/actions, and the relevant success or failure
   result with ROS CLI tools.
6. Review `git diff` and ensure generated files, secrets, and unrelated user
   changes are not included.

For controller, URDF, grasp, or motion changes, validate at least the robot
spawn, controller activation, joint movement, gripper behavior, and pick/place
postcondition in Gazebo. Keep geometry-dependent constants synchronized between
the URDF, `nlra_skills`, `nlra_world_model`, and `nlra_orchestrator`.

## Known Limitations

- MoveIt planning is pose-based; `moveit_py` on Jazzy has no
  `compute_cartesian_path` binding, so "relative" Cartesian moves are planned
  as absolute goals from the current EE pose (fine for the current tabletop
  sim, not true path-constrained motion).
- Vision-based perception is not enabled; Gazebo ground-truth pose bridges are
  used instead.
- The NL layer depends on a cloud, OpenAI-compatible endpoint unless replaced.
- Formal automated tests and broad multi-object/multi-step planning are future
  work.
- `docs/ARCHITECTURE.md` records the detailed status and remaining roadmap.
