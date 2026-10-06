# Embodied-RobotSim

[中文完整配置指南](README.md)

ROS 2 Jazzy and Isaac Sim 6.1 workspace for a differential-drive mobile robot with a Franka FR3 arm, simulated MID-360 lidar and stereo RGB-D cameras. FAST-LIO provides lidar-inertial odometry, Nav2 handles exploration/navigation, and YOLOE + GraspNet + MoveIt handle manipulation. CUDA semantic voxel mapping integrates geometry with hit/miss updates and semantic colors with voting. Qwen3 and the Web UI provide natural-language task control.

## Demos

![Isaac Sim](assets/IssacSim.gif)

![LLM Agent](assets/Embodied_LLM.gif)

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
bash scripts/setup_isaac_dependencies.sh
bash -c 'source /opt/ros/jazzy/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y'
```

The [dependency manifest](scripts/isaac.repos) pins FAST_LIO_ROS2 commit `2fffc570a25d0df172720bac034fbdb6a13d2162`. The setup script imports it into `src/isaac_vendor/FAST_LIO_ROS2` and initializes the ikd-Tree submodule. Keep this dependency for simulation builds. No Livox hardware SDK is required: `src/isaac_livox_interfaces` supplies the `livox_ros_driver2` message package. Do not add another package with that name.

Download models from the [model folder](https://drive.google.com/drive/folders/1gPPyvKqiYd7cg2vUyqucV1CjLTf6J0y2?usp=drive_link):

| Model | Destination |
|---|---|
| `yoloe-v8l-text-prompt-multi_nc10_fp16.onnx` | `src/yoloe_infer/models/` |
| `graspnet.onnx` | `src/graspnet_infer/` |

Keep the required `src/yoloe_infer/models/tokenizer_data.json.gz`. Generate engines locally when changing GPU or TensorRT versions:

```bash
bash src/graspnet_infer/tensorrt_plugins/build.sh
trtexec --onnx=src/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.onnx \
  --saveEngine=src/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.engine
trtexec --onnx=src/graspnet_infer/graspnet.onnx \
  --saveEngine=src/graspnet_infer/graspnet.trt \
  --staticPlugins=src/graspnet_infer/tensorrt_plugins/build/libfps_plugin.so
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

Default rooms are converted from the original `main` branch scenes, including textures. Keep a local `main` branch; a single-branch clone may need `git fetch origin main` and `git branch main origin/main` if absent. Missing default assets are converted on first startup, or prepare them explicitly:

```bash
"$ISAAC_SIM_PATH/python.sh" scripts/assets/migrate_gazebo_scenes.py \
  --worlds simple_room manipulation_test
# Optional NVIDIA Office and Simple Room assets:
"$ISAAC_SIM_PATH/python.sh" scripts/assets/download_isaac_environments.py
```

Assets are cached under `$ISAAC_ASSETS_PATH`. `WORLD=simple_room` uses main's room, `WORLD=isaac_simple_room` uses NVIDIA's room, and `WORLD=office` uses the downloaded Office with additional indoor lighting. Change the map bundle and rebuild maps when switching worlds. Gazebo in the migration tool's name refers to the source asset format; runtime uses Isaac Sim.

## Run

Edit defaults at the top of the root scripts; no arguments are required.

| Entry point | Default behavior |
|---|---|
| `./start_explore_and_mapping.sh` | Automatic exploration/mapping in `simple_room` |
| `./start_navigation.sh` | ICP localization and Nav2 in `simple_room` |
| `./start_pick_and_place_demo.sh` | Pick/place book, cup, coke, bottle and shoe in `manipulation_test` |
| `./start_navigation_and_pick_demo.sh` | Navigation and manipulation in `simple_room` |
| `./start_llm_agent.sh` | Qwen3 task server and Web UI in `simple_room` |
| `./stop_robot_sim.sh` | Stop ROS/Isaac services and close project service terminals |

Startup cleans previous sessions and logs. One RViz displays FAST-LIO clouds, odometry trails, semantic voxels, maps and navigation paths. Use **Nav2 Goal** once navigation is ready. Set `AUTO_EXPLORE=false` for manual mapping/navigation, or pause exploration on `/explore/resume` with `std_msgs/msg/Bool {data: false}`.

Save an exploration map from another ROS-enabled terminal:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 service call /localization/save_map std_srvs/srv/Trigger '{}'
```

Default output: `maps/gazebo_simple_room`. Navigation modes require `bundle.json`, `map.pcd`, `map.yaml` and `map.pgm`; a 2D map alone is insufficient for ICP localization. Saving does not overwrite an existing bundle. Task coordinates must be checked when changing maps. Set `DASHSCOPE_API_KEY` before starting the LLM mode; Web UI runs at `http://localhost:8888`.

## Configuration

FAST-LIO uses **`src/x_bot_localization/config/fastlio_mid360.yaml`**, not the vendor repository's default YAML. Topics are `/livox/lidar` and `/livox/imu`, lidar type is 1, scan lines 4, timestamp unit 3 (ns), and scan rate 10 Hz in simulation time. Time synchronization is disabled with zero offset because both sensors share the simulation clock. Simulated lidar/IMU origins are colocated, so extrinsics are zero translation and identity rotation, with online extrinsic estimation disabled. Map/surface filters use 0.15 m and point filtering uses every third point.

The PhysX-raycast lidar approximates MID-360 sampling rather than its optical response. Physics/IMU run at 200 Hz and clouds at 10 Hz in simulation time. Navigation TF is `map → odom → base_footprint → mid360_imu_link`; FAST-LIO's native TF is kept private. Ground truth is debug-only. Known-map ICP requires an approximate initial pose; use RViz **2D Pose Estimate** to correct it. Invalid localization stops the base. Restart the complete localization pipeline after resetting the simulation clock.

| File / setting | Purpose |
|---|---|
| Root script defaults | World, map bundle, initial pose, build and headless options |
| `src/x_bot_localization/config/nav2_isaac.yaml` | Navigation velocity, footprint, inflation and path tracking |
| `src/semantic_voxel_mapping/config/map.yaml` | Exploration semantic mapping |
| `src/semantic_voxel_mapping/config/manipulation.yaml` | 2 cm manipulation mapping, 5 Hz integration / 2 Hz publication limits |
| `src/yoloe_infer/configs/manipulation.yaml` | Manipulation classes/colors and semantic depth sampling |
| `src/x_bot/config/isaac_controllers.yaml` | Controllers and gripper stall thresholds |
| `src/x_bot/config/pick_and_place_demo.yaml` | Grasp task and physical verification |

Exploration defaults to FAST-LIO geometry projected into synchronized RGB masks. Manipulation defaults to Isaac depth; set `DEPTH_SOURCE=lsm` with `SEMANTIC_CLOUD_SOURCE=depth` for stereo inferred depth. FAST-LIO mode does not need LSM. All semantic maps use the new CUDA voxel mapper; recognition pauses retain geometry updates so observed miss rays can clear moved objects. Semantic depth clouds use stride 2 while GraspNet keeps full density. See [semantic mapping](src/semantic_voxel_mapping/README.md) and [LSM](src/LSM_depth_infer/README.md) for details.

Physical grasp checks verify lift, retained carry and settled placement. `/isaac/debug/object_states` is read-only validation telemetry; perception selects targets and contact/friction move objects. An action success alone does not prove a physical grasp.

## Repository organization and checks

`scripts/robot_services.sh`, `isaac_runtime.sh` and `close_robot_terminals.py` implement common runtime/cleanup. `scripts/setup_isaac_dependencies.sh` uses `scripts/isaac.repos`. Asset utilities live in `scripts/assets`; optional read-only profiling lives in `scripts/diagnostics`. Offline regressions stay in `tests`. Historical validation reports and one-off migration validation scripts have been removed. Build products, map bundles and vendor dependencies are ignored.

```bash
python3 -m unittest discover -s tests -v
# In a ROS-enabled terminal:
python3 scripts/diagnostics/measure_isaac_performance.py --seconds 30 --output /tmp/isaac_performance.json
ros2 topic echo /localization/ready
ros2 topic echo /localization/status
ros2 control list_controllers
```

Set `HEADLESS=true` to disable the observation viewport while retaining camera/ROS perception. Configured velocity is measured per simulation second; wall-clock motion depends on real-time factor. Profiling adds overhead. Without desktop terminals, service logs are written to `log/isaac_sim`.

Thanks to FAST-LIO, FAST_LIO_ROS2, YOLOE, GraspNet, m-explore-ros2, Franka ROS 2 and bcr_bot. Original asset licenses are retained alongside cached scene sources.
