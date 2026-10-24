# Embodied-RobotSim

[中文](README.md) · [Installation and build guide (Chinese)](INSTALL.md)

Mobile manipulation simulation with **ROS 2 Jazzy, Isaac Sim 6.1 / Gazebo Harmonic**, integrating FAST-LIO, Nav2, YOLOE, semantic OctoMap, GraspNet / MoveIt and ACT for natural-language fetch tasks.

| Manipulation | Mapping and navigation | Embodied tasks |
|---|---|---|
| ![Isaac Sim manipulation](assets/IssacSim.gif) | ![Gazebo mapping](assets/GazeboSim.gif) | ![Embodied tasks](assets/Embodied_LLM.gif) |

## Quick start

Complete the [dependency, model and build setup](INSTALL.md), then run from the repository root:

```bash
# Embodied tasks: Gazebo, local Qwen, traditional grasping by default
./start_embodied.sh

# ACT grasping: rgb / rgbd / rgbxyz
./start_embodied.sh --pipeline act --act-model rgbd

# Explore and save a map in Gazebo
./start_mapping_and_explore.sh --sim gazebo

# Navigate on an existing map; set Nav2 Goal in RViz
./start_mapping_and_explore.sh --sim gazebo --mode navigation --bundle maps/gazebo/semantic_simple_room

# Standalone pick-and-place: Isaac Sim, manipulation_test world by default
./start_pick_and_place.sh

# Stop project services
./stop_robot_sim.sh
```

Open **http://localhost:8888** and enter a command such as `到厨房拿可乐`. To use the cloud model:

```bash
export DASHSCOPE_API_KEY='your-api-key'
./start_embodied.sh --llm cloud
```

The embodied launcher defaults to Gazebo; mapping and standalone manipulation default to Isaac Sim. Select the backend with `--sim isaac|gazebo`. Common options include `--world`, `--bundle`, `--headless`, `--build` and `--no-build`; see each launcher's `--help`. Mapping and manipulation build by default; embodied tasks reuse an existing build. Startup stops the previous project session.

ACT requires an external ACT environment and checkpoints configured in [config.json](src/embodied/act_policy/config.json). Override the selected checkpoint with `EMBODIED_ACT_CHECKPOINT`. See the [localization guide](src/localization/README.md) for experimental VIO configuration and limitations.

## Task execution and maps

Explicit fetch commands follow predefined stages:

**Map readiness → room region or VLM-estimated search waypoint → Nav2 navigation → local camera mapping and target search → visual servoing into a candidate grasp workspace → grasp, lift and return the arm Home while holding the object.**

Other commands use Qwen's native tool-calling loop. The VLM infers regions from mapped objects and camera observations; Nav2, GraspNet / MoveIt or ACT execute the movements.

- **Persistent maps:** both launchers default to `maps/<backend>/semantic_<world>`; use the same `--sim`, `--world` and `--bundle`. Geometry is stored as `bundle.json`, `map.pcd`, `map.yaml` and `map.pgm`. Sibling `.semantic.json` and `.semantic.json.gz` files store object XYZ, room regions, VLM search hints, evidence and a geometry fingerprint. When room boundaries remain unconfirmed, the VLM can choose a map-checked search waypoint, followed by local visual search. See the [map handoff and storage format](src/embodied/semantic_navigation/README.md).
- **Local map:** camera depth input, paused by default; reset and enabled when target search starts, publishing `/local_map/octomap_full` for MoveIt collision checking, then paused when the task ends.
- **Global map:** continuously integrates FAST-LIO geometry in LIO mode; camera mask projection and raycasting label visible occupied LiDAR voxels. Visualize `/global_map/occupied_cells_vis_array`, also available through `/occupied_cells_vis_array`.
- **Visual servoing:** locks a target instance using fresh RGB-D detections and adjusts base heading and distance at low speed. Detection loss stops translation and triggers reacquisition. After the target enters a candidate grasp workspace and the base stops, GraspNet / MoveIt or ACT takes over. Commands still pass through velocity smoothing, collision checking and the localization safety gate.
- **Execution checks:** localization, observation freshness, parking error and visual confirmation gate execution. Maps must match the backend and world.

Pass the same `--bundle` to hand maps from exploration to embodied tasks. Exploration refuses to overwrite an existing bundle; select a new path when remapping. Embodied tasks relocalize directly when a complete geometry bundle exists. Generated maps, semantic summaries and local validation data are excluded from Git.

On 2026-10-24, one Gazebo kitchen-cola trial completed VLM waypoint handoff, search, visual servoing, grasping, lifting, return Home and a two-second hold check in about 71 seconds. Servo distance converged from 1.05 m to 0.72 m. The preceding trial stopped after losing the detection; VLM boundaries can still be incorrect. This single success does not establish performance across starting poses or occlusions. See the [semantic navigation validation record](src/embodied/semantic_navigation/README.md).

## Modules and experiments

| Module | Contents |
|---|---|
| [Semantic navigation](src/embodied/semantic_navigation/README.md) | Persistent maps, VLM regions, tracking and validation status |
| [Agent](src/embodied/llm_agent/README.md) | Language orchestration and historical experiments |
| [ACT](src/embodied/act_policy/README.md) | RGB / RGBD / RGBXYZ results and replays |
| [Semantic mapping](src/mapping/semantic_voxel_mapping/README.md) | Local and global OctoMap configuration |
| [Hand-eye calibration](src/manipulation/handeye_demo/README.md) | AprilGrid calibration, depth grasping and error analysis |

Source groups: `robot`, `localization`, `perception`, `planning`, `control`, `mapping`, `manipulation`, `simulation`, and `embodied`. Launcher defaults live in root scripts; runtime helpers are in `scripts/runtime/`. Models, maps, caches and runtime logs are excluded from Git.
