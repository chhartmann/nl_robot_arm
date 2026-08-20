# How to use / start the NL robot arm (ROS 2)

Sim-only stack: KUKA Agilus KR 16 R1100-3 + Robotiq 2F-85 in Gazebo Harmonic, controlled
via `ros2_control`. Everything runs inside the pixi environment.

## 1. Environment setup (one-time)

```bash
pixi install                 # create env + install conda/PyPI deps
pixi run build               # colcon build the workspace (src/)
```

All sources are self-contained (Agilus URDF + meshes are vendored in
`agilus_robotiq_description` — no `kuka_robot_descriptions` clone needed).

`pixi install` warns about `install/setup.sh` missing — expected until the
workspace is built. The shell activation (Gazebo plugin paths, colcon overlay)
only works after `pixi run build`.

## 2. Enter the environment

```bash
pixi shell
```

This sources the colcon overlay and sets `GZ_SIM_SYSTEM_PLUGIN_PATH` /
`GZ_SIM_RESOURCE_PATH` so Gazebo finds the gz_ros2_control plugins.
(`pixi run` also activates these for single commands.)

## 3. Quick sanity check

```bash
ros2 pkg list | grep -E 'moveit|ros_gz|controller' | head
```

## 4. Launch the full stack

```bash
# headless sim (no Gazebo GUI) — default
ros2 launch nlra_bringup nlra.launch.py

# with Gazebo GUI
ros2 launch nlra_bringup nlra.launch.py gui:=true

# different world (empty_bullet.sdf, empty_bullet_cam.sdf, tabletop_scene.sdf)
ros2 launch nlra_bringup nlra.launch.py world:=empty_bullet.sdf

# without the NL interface (if no API key / offline)
ros2 launch nlra_bringup nlra.launch.py nl:=false
```

`nlra.launch.py` brings up components as soon as their dependencies are ready
(no fixed delays):

| t (s) | component                                   |
|-------|---------------------------------------------|
| 0     | Gazebo sim + robot spawn + controllers      |
| ~2    | pose bridge, world model, orchestrator, NL interface, rosbridge, web UI |
| ~10   | MoveIt `move_group` (after controllers are active) |
| ~12   | skill servers (embed the MoveItPy motion planner) |

The pure-Python nodes discover their services/actions lazily, so they start
early; `move_group` and the skill servers wait for a readiness gate that
observes the arm controller. Startup adapts to the host (about 15 s on this
machine).

Each node is independently restartable at runtime.

## 5. Launching pieces separately

If the sim is already running and you only need the skill servers (which embed
the MoveItPy motion planner — `move_group` must be up too):

```bash
ros2 launch nlra_skills skills.launch.py
```

Or run individual nodes:

```bash
ros2 run nlra_world_model world_model
ros2 run nlra_skills skill_servers
ros2 run nlra_orchestrator orchestrator
ros2 run nlra_nl_interface nl_interface
```

For the web UI (manual control, NL chat, diagnostics), the full stack launch
starts rosbridge + web server automatically. Open http://localhost:8080 in a browser.

For the `move_to` / `move_relative` / `move_axis` skills to plan, a MoveIt
`move_group` must be running (the full stack launches it automatically; on a
manual setup start it via `ros2 launch agilus_robotiq_moveit_config move_group.launch.py`).

## 6. Inspecting the running system

```bash
ros2 node list                                   # all live nodes (incl. move_group)
ros2 topic list                                  # all topics (bridged too)
ros2 action list -t                              # skill + orchestrator actions
ros2 control list_controllers                    # controller states
ros2 topic echo /joint_states                    # arm/gripper joint states
```

## 7. Shutting down

Ctrl-C in the launch terminal stops the stack. Check leftover Gazebo processes
if needed:

```bash
pkill -f 'gz sim'          # only if a server was left behind
```

## 8. Notes

- The world model relies on the ros_gz pose bridge from the launch file; if
  you start nodes manually, bring that bridge up too.
- `move_to` / `move_relative` / `move_axis` need MoveIt (`move_group` + the
  embedded motion planner) to be up; `move_joints` / `grasp` / `release` /
  `home` talk straight to the controllers.
- The NL interface calls an LLM — configure your provider/API key (see
  `nlra_nl_interface` / `.env` handling via `python-dotenv`) before using it.
- Real-time hardware drivers (`kuka_drivers`) are intentionally not installed;
  this repo is sim-only.