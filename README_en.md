# Embodied-RobotSim

[中文完整配置指南](README.md)

ROS 2 Jazzy workspace with Isaac Sim 6.1 and Gazebo Harmonic backends for a differential-drive mobile robot with a Franka FR3 arm, simulated MID-360 lidar and stereo RGB-D cameras. FAST-LIO provides lidar-inertial odometry, Nav2 handles exploration/navigation, and YOLOE + GraspNet + MoveIt handle manipulation. CUDA semantic voxel mapping integrates geometry with hit/miss updates and semantic colors with voting. Qwen3 and the Web UI provide natural-language task control.

## Demos

### Isaac Sim

![Isaac Sim simulation](assets/IssacSim.gif)

### Gazebo Harmonic

![Gazebo Harmonic simulation](assets/GazeboSim.gif)

Both backends share the entry scripts. Isaac Sim is the default; add `--sim gazebo` to switch:

```bash
./start_mapping.sh
./start_mapping.sh --sim gazebo
```

### LLM Agent

![LLM Agent](assets/Embodied_LLM.gif)

## Source layout

| Group | Responsibility |
|---|---|
| `src/description/` | x_bot / Franka URDF and meshes |
| `src/localization/` | FAST-LIO adapter, ICP and localization TF |
| `src/perception/` | YOLOE, GraspNet, TensorRT plugins and optional local LSM |
| `src/planning/` | Exploration, Nav2, MoveIt configuration and unified RViz |
| `src/control/` | Velocity arbitration, safety, path controller, base and joint parameters |
| `src/mapping/` | CUDA semantic voxels, 2D grids and paired-map persistence |
| `src/manipulation/` | Scan, Go Home, gripper, pick/place and physical acceptance |
| `src/simulation/` | Isaac, Gazebo Harmonic and shared scene asset parsing |
| `src/sensors/` | Livox bridge and shared MID-360 configuration/sampling |
| `src/embodied/` | Agent and Web UI |
| `src/common/` / `src/interfaces/` | Geometry, system composition and message definitions |
| `src/vendor/` | Ignored, pinned FAST-LIO dependency checkout |

Runtime helpers live in `scripts/runtime/`; installation, assets and diagnostics are grouped separately. Root launchers are `start_mapping.sh`, `start_pick.sh`, `start_embodied.sh` and `stop_robot_sim.sh`.

## Setup

Use Ubuntu 24.04, ROS 2 Jazzy and Isaac Sim 6.1. The tested local CUDA/TensorRT versions are 13.3/10.14.1.48. Install the NVIDIA driver and CUDA/TensorRT development libraries compatible with your GPU. Follow NVIDIA's [workstation installation](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_workstation.html) and [ROS setup](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html). Runtime code enables the required URDF importer, ROS bridge/control and experimental physics-sensor extensions.

Add your local paths to `~/.zshrc` or `~/.bashrc`:

```bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
export ISAAC_ASSETS_PATH="$HOME/isaacsim_assets/6.1"
export CUDA_HOME=/usr/local/cuda
export PATH="$CUDA_HOME/bin:/usr/src/tensorrt/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

Check `nvidia-smi`, `nvcc --version`, `trtexec --version` and that `$ISAAC_SIM_PATH/python.sh` is executable. Use system Python for ROS builds and Isaac's `python.sh` for asset tools. Deactivate Conda before building ROS packages. In interactive Zsh, source `setup.zsh`; project Bash scripts source `setup.bash` internally.

From the repository root:

```bash
sudo apt update
sudo apt install python3-vcstool python3-rosdep python3-colcon-common-extensions \
  python3-numpy python3-yaml python3-dev libeigen3-dev libpcl-dev \
  ros-jazzy-moveit ros-jazzy-moveit-ros-perception \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# Run sudo rosdep init once if rosdep has not been initialized.
bash scripts/setup/setup_dependencies.sh
bash -c 'source /opt/ros/jazzy/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y'
```

The [dependency manifest](scripts/setup/vendor.repos) pins FAST_LIO_ROS2 commit `2fffc570a25d0df172720bac034fbdb6a13d2162`. The setup script imports it into `src/vendor/FAST_LIO_ROS2` and initializes the ikd-Tree submodule. Keep this dependency for simulation builds. No Livox hardware SDK is required: `src/interfaces/livox_ros_driver2` supplies the `livox_ros_driver2` message package. Do not add another package with that name.

Download models from the [model folder](https://drive.google.com/drive/folders/1gPPyvKqiYd7cg2vUyqucV1CjLTf6J0y2?usp=drive_link):

| Model | Destination |
|---|---|
| `yoloe-v8l-text-prompt-multi_nc10_fp16.onnx` | `src/perception/yoloe_infer/models/` |
| `graspnet.onnx` | `src/perception/graspnet_infer/` |

Keep the required `src/perception/yoloe_infer/models/tokenizer_data.json.gz`. Generate engines locally when changing GPU or TensorRT versions:

```bash
bash src/perception/graspnet_infer/tensorrt_plugins/build.sh
trtexec --onnx=src/perception/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.onnx \
  --saveEngine=src/perception/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.engine
trtexec --onnx=src/perception/graspnet_infer/graspnet.onnx \
  --saveEngine=src/perception/graspnet_infer/graspnet.trt \
  --staticPlugins=src/perception/graspnet_infer/tensorrt_plugins/build/libfps_plugin.so
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

Default rooms are converted from the original `main` branch scenes, including textures. Keep a local `main` branch; a single-branch clone may need `git fetch origin main` and `git branch main origin/main` if absent. Missing default assets are converted on first startup, or prepare them explicitly:

```bash
"$ISAAC_SIM_PATH/python.sh" scripts/assets/migrate_gazebo_scenes.py \
  --worlds simple_room manipulation_test
# Optional NVIDIA Office and Simple Room assets:
"$ISAAC_SIM_PATH/python.sh" scripts/assets/download_isaac_environments.py
```

Assets are cached under `$ISAAC_ASSETS_PATH`. `WORLD=simple_room` uses main's room, `WORLD=isaac_simple_room` uses NVIDIA's room, and `WORLD=office` uses the downloaded Office with additional indoor lighting. Change the map bundle and rebuild maps when switching worlds. Gazebo in the migration tool's name refers to the source asset format; Isaac runtime uses its converted USD; Gazebo uses the pinned SDF and source models.

## Gazebo compatibility

Gazebo uses the current Isaac-first task, mapping and motion defaults, FAST-LIO, ICP, Nav2 and the CUDA semantic mapper. Install optional dependencies and rebuild:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-sim-vendor ros-jazzy-gz-common-vendor ros-jazzy-gz-plugin-vendor libembree-dev
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
./start_mapping.sh --sim gazebo
./start_pick.sh --sim gazebo
```

Set `SIM_BACKEND=gazebo` in an entry script for argument-free Gazebo startup. Supported worlds are `simple_room` and `manipulation_test`; USD-only worlds are rejected. Gazebo maps live under `maps/gazebo/<world>`; existing Isaac map paths remain compatible. Gazebo semantic maps are saved to `semantic_map.svm` in the selected bundle. On completion, Gazebo exploration calls `/localization/save_map` to save the complete paired PCD/2D bundle. Existing bundles are protected from overwrite; choose a new `--bundle` when remapping. Use `DEPTH_SOURCE=sim` for the selected simulator's depth, or `lsm` for inferred depth. The legacy `isaac` depth value remains accepted only by Isaac.

Shared base parameters live in `src/control/x_bot_control/config/base_motion.yaml`; Gazebo controller interface overrides live in `gazebo_controllers.yaml`, layered over the existing Isaac controller defaults. Gazebo joint-2 effort-drive gains, finger drives and friction live in `gazebo_physics.yaml`. Controllers load sequentially: manager discovery allows 600 s, each service call 300 s, and activation 120 s, including cold shader compilation and collision decomposition. The optional C++ backend uses Embree and Gazebo's native mesh loader for collision rays: 1,000 actual rays per 5 ms step, 100 ms packets, identical approximate nonrepeating directions and timestamps to Isaac. Dynamic scene meshes use convex decomposition shared by physics and Embree. Bottle damping uses physical forces and torques. Its ground truth never supplies navigation TF. Physical verification uses the neutral telemetry topic on both backends. PCA top grasps pair the depth-derived center and direction; retraction preserves vertical lift clearance with a 1 m minimum. Verification telemetry does not supply motion targets.

Scene extraction is independent of Isaac Python and pinned to `92b0409ccf83549e74d03966bdde7f0f700ff927`, rather than the moving main branch. Keep full Git history for that revision. Missing Gazebo/Embree dependencies disable the optional plugin build, without preventing Isaac builds. See the Chinese guide for detailed setup.

## Run

Edit defaults at the top of the root scripts; no arguments are required. Isaac remains the default; append `--sim gazebo` to select Gazebo.

| Entry point | Default behavior |
|---|---|
| `./start_mapping.sh` | Automatic exploration/mapping in `simple_room` |
| `./start_mapping.sh --mode navigation` | ICP localization and Nav2 in `simple_room` |
| `./start_pick.sh` | Pick/place book, cup, coke, bottle and shoe in `manipulation_test` |
| `./start_pick.sh --mode navigation` | Navigation and manipulation in `simple_room` |
| `./start_embodied.sh` | Qwen3 task server and Web UI in `simple_room` |
| `./stop_robot_sim.sh` | Stop ROS/Isaac/Gazebo services and close project service terminals |

Startup cleans previous sessions and logs. One RViz displays FAST-LIO clouds, odometry trails, semantic voxels, maps and navigation paths. Use **Nav2 Goal** once navigation is ready. Set `AUTO_EXPLORE=false` for manual mapping/navigation, or pause exploration on `/explore/resume` with `std_msgs/msg/Bool {data: false}`.

Save an exploration map from another ROS-enabled terminal:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 service call /localization/save_map std_srvs/srv/Trigger '{}'
```

Isaac default output: `maps/gazebo_simple_room`; Gazebo default: `maps/gazebo/simple_room`. Navigation modes require `bundle.json`, `map.pcd`, `map.yaml` and `map.pgm`; a 2D map alone is insufficient for ICP localization. Saving does not overwrite an existing bundle. Task coordinates must be checked when changing maps. Set `DASHSCOPE_API_KEY` before starting the LLM mode; Web UI runs at `http://localhost:8888`.

## Configuration

FAST-LIO uses **`src/localization/x_bot_localization/config/fastlio_mid360.yaml`**, not the vendor repository's default YAML. Topics are `/livox/lidar` and `/livox/imu`, lidar type is 1, scan lines 4, timestamp unit 3 (ns), and scan rate 20 Hz in simulation time (shared `mid360.yaml` setting). Time synchronization is disabled with zero offset because both sensors share the simulation clock. Simulated lidar/IMU origins are colocated, so extrinsics are zero translation and identity rotation, with online extrinsic estimation disabled. Map/surface filters use 0.15 m and point filtering uses every third point.

The PhysX-raycast lidar approximates MID-360 sampling rather than its optical response. Physics/IMU run at 200 Hz and clouds at 10 Hz in simulation time. Navigation TF is `map → odom → base_footprint → mid360_imu_link`; FAST-LIO's native TF is kept private. Ground truth is debug-only. Known-map ICP requires an approximate initial pose; use RViz **2D Pose Estimate** to correct it. Invalid localization stops the base. Restart the complete localization pipeline after resetting the simulation clock.

| File / setting | Purpose |
|---|---|
| Root script defaults | World, map bundle, initial pose, build and headless options |
| `src/planning/x_bot_navigation/config/nav2.yaml` | Navigation velocity, footprint, inflation and path tracking |
| `src/mapping/semantic_voxel_mapping/config/map.yaml` | Exploration semantic mapping |
| `src/mapping/semantic_voxel_mapping/config/manipulation.yaml` | 2 cm manipulation mapping, 5 Hz integration / 2 Hz publication limits |
| `src/perception/yoloe_infer/configs/manipulation.yaml` | Manipulation classes/colors and semantic depth sampling |
| `src/control/x_bot_control/config/isaac_controllers.yaml` | Controllers and gripper stall thresholds |
| `src/manipulation/x_bot_manipulation/config/pick_and_place_demo.yaml` | Grasp task and physical verification |

Exploration defaults to FAST-LIO geometry projected into synchronized RGB masks. Manipulation defaults to Isaac depth; set `DEPTH_SOURCE=lsm` with `SEMANTIC_CLOUD_SOURCE=depth` for stereo inferred depth. FAST-LIO mode does not need LSM. All semantic maps use the new CUDA voxel mapper; grasping gates cloud publication while YOLOE inference and images remain active, and occupancy updates resume after release. Semantic depth clouds use stride 2 while GraspNet keeps full density. See [semantic mapping](src/mapping/semantic_voxel_mapping/README.md) for details. `src/perception/LSM_depth_infer` is an ignored local optional module; existing local files remain, but new clones must provide the module, configuration and inference engine and build `stereo_matching` before selecting `lsm`.

Physical grasp checks verify lift, retained carry and settled placement. `/simulation/debug/object_states` is read-only validation telemetry (Isaac retains its legacy alias); perception selects targets and contact/friction move objects. An action success alone does not prove a physical grasp.

## Repository organization and checks

`scripts/runtime/robot_services.sh`, `isaac_runtime.sh` and `close_robot_terminals.py` implement common runtime/cleanup. `scripts/setup/setup_dependencies.sh` uses `scripts/setup/vendor.repos`. Asset utilities live in `scripts/assets`; optional read-only profiling lives in `scripts/diagnostics`. Offline regressions stay in `tests`. Historical validation reports and one-off migration validation scripts have been removed. Build products, map bundles and vendor dependencies are ignored.

```bash
python3 -m unittest discover -s tests -v
# In a ROS-enabled terminal:
python3 scripts/diagnostics/measure_isaac_performance.py --seconds 30 --output /tmp/isaac_performance.json
ros2 topic echo /localization/ready
ros2 topic echo /localization/status
ros2 control list_controllers
```

Set `HEADLESS=true` to disable the observation viewport while retaining camera/ROS perception. Configured velocity is measured per simulation second; wall-clock motion depends on real-time factor. Profiling adds overhead. Without desktop terminals, service logs are written to `log/isaac` or `log/gazebo`.

During grasping, YOLOE keeps inference and annotated images active. The `/yoloe_multi_text_prompt/enable_pointcloud` (`std_srvs/srv/SetBool`) service gates both colored and semantic clouds; the demo restores publication on completion or failure. Occupancy updates from these clouds pause during grasping and resume afterward.

Thanks to FAST-LIO, FAST_LIO_ROS2, YOLOE, GraspNet, m-explore-ros2, Franka ROS 2 and bcr_bot. Original asset licenses are retained alongside cached scene sources.

## Functional layout and entrypoint migration

The source tree is grouped into description, localization, perception, planning, control, mapping, manipulation, simulation, sensors, embodied, common, interfaces and vendor directories. `x_bot` retains only URDF and meshes so mesh package URIs stay valid. `x_bot_bringup` composes estimator, sensors, safety and map services; individual modules own their algorithms and configuration. Unified RViz is installed by `x_bot_navigation`.

| Previous entry | Current entry |
|---|---|
| `start_explore_and_mapping.sh` | `./start_mapping.sh` |
| `start_navigation.sh` | `./start_mapping.sh --mode navigation` |
| `start_pick_and_place_demo.sh` | `./start_pick.sh` |
| `start_navigation_and_pick_demo.sh` | `./start_pick.sh --mode navigation` |
| `start_llm_agent.sh` | `./start_embodied.sh` |

The root contains these three launchers and `stop_robot_sim.sh`. All launchers default to Isaac and support `--sim gazebo`; runtime helpers live in `scripts/runtime/`. Existing topics, services, actions, TF, 20 Hz scan framing and numerical control defaults are preserved. Both backends save paired maps via `/localization/save_map`. Maps stay in `maps/`, with pre-migration 2D outputs preserved in `maps/legacy/` and `maps/pre_reorganization/`. Rebuild after moving sources; the old install must not be reused. Local inference models, LSM and the pinned FAST-LIO checkout are preserved in their new directories.

Optional Agent / Web dependencies are installed explicitly, outside the simulation startup:

```bash
bash scripts/setup/setup_embodied_dependencies.sh --local
export DASHSCOPE_API_KEY='your-key'
./start_embodied.sh
```

The launcher uses `.venv/agent/bin/python` when present (system ROS packages remain available), or an explicit `AGENT_PYTHON`. Runtime scripts do not install system packages. Web logs are stored in `log/web/`.

`--local` extracts optional Web ROS packages into ignored `.cache/embodied_sysroot/` and installs Agent Python dependencies into `.venv/agent/`, without sudo. The embodied launcher loads that local overlay. Without `--local`, installation uses system apt packages. Without an API key, Agent status remains available and chat reports the missing configuration.

Repeated mapping runs preserve an existing bundle and print a new timestamped output directory. Pass that path with `--bundle` when navigating on the newly saved map.
