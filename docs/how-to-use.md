# How to use / start the NL robot arm (ROS 2)

Sim-only stack: KUKA Agilus KR 16 R1100-3 + Robotiq 2F-85 in Gazebo Harmonic, controlled
via `ros2_control`. Everything runs inside the pixi environment.

## 1. Environment setup (one-time)

```bash
pixi install                 # create env + install conda/PyPI deps
pixi run fetch_sources       # clone kuka_robot_descriptions (if not done)
pixi run build               # colcon build the workspace (src/)
```

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

`nlra.launch.py` brings up (in order, with delays):

| t (s) | component                                   |
|-------|---------------------------------------------|
| 0     | Gazebo sim + robot spawn + controllers      |
| 25    | ros_gz bridges (object poses, gripper attach/detach, /clock) |
| 30    | world model + skill servers                 |
| 35    | orchestrator                                |
| 40    | NL interface                                |

Each node is independently restartable at runtime.

## 5. Launching pieces separately

If the sim is already running and you only need the skill servers:

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

## 6. Inspecting the running system

```bash
ros2 node list                                   # all live nodes
ros2 topic list                                  # all topics (bridged too)
ros2 action list                                 # skill actions
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

- The world model relies on the ros_gz pose bridges from the launch file; if
  you start nodes manually, bring those bridges up too.
- The NL interface calls an LLM — configure your provider/API key (see
  `nlra_nl_interface` / `.env` handling via `python-dotenv`) before using it.
- Real-time hardware drivers (`kuka_drivers`) are intentionally not installed;
  this repo is sim-only.
