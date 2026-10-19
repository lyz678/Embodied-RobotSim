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
./start_mapping_and_explore.sh --sim gazebo --mode navigation --bundle maps/gazebo/simple_room

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

**Map readiness → semantic region lookup → Nav2 navigation → camera VLM room confirmation → YOLOE target tracking → approach → grasp, lift and return the arm Home while holding the object.**

Other commands use Qwen's native tool-calling loop. The VLM infers regions from mapped objects and camera observations; Nav2, GraspNet / MoveIt or ACT execute the movements.

- **Persistent maps:** explore and save when no complete bundle exists; otherwise relocalize on the saved map. The embodied default is `maps/<backend>/semantic_<world>`, containing `bundle.json`, `map.pcd`, `map.yaml` and `map.pgm`, with a sibling `.semantic.json` file for structured semantics.
- **Local map:** depth input by default, publishing `/local_map/octomap_full` for MoveIt collision checking.
- **Global map:** FAST-LIO clouds in LIO mode, publishing `/global_map/octomap_full`; visualize `/global_map/occupied_cells_vis_array`, also available through `/occupied_cells_vis_array`.
- **Execution checks:** localization, observation freshness, parking error and visual confirmation gate execution. Maps must match the backend and world.

Map saving and restart localization have been verified. The new semantic fetch pipeline without predefined room coordinates **has not passed end-to-end validation**. VLM region boundaries remain unreliable; historical fixed-goal results do not establish this pipeline's success rate.

## Modules and experiments

| Module | Contents |
|---|---|
| [Semantic navigation](src/embodied/semantic_navigation/README.md) | Persistent maps, VLM regions, tracking and validation status |
| [Agent](src/embodied/llm_agent/README.md) | Language orchestration and historical experiments |
| [ACT](src/embodied/act_policy/README.md) | RGB / RGBD / RGBXYZ results and replays |
| [Semantic mapping](src/mapping/semantic_voxel_mapping/README.md) | Local and global OctoMap configuration |
| [Hand-eye calibration](src/manipulation/handeye_demo/README.md) | AprilGrid calibration, depth grasping and error analysis |

Source groups: `robot`, `localization`, `perception`, `planning`, `control`, `mapping`, `manipulation`, `simulation`, and `embodied`. Launcher defaults live in root scripts; runtime helpers are in `scripts/runtime/`. Models, maps, caches and runtime logs are excluded from Git.
