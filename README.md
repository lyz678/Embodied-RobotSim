# Embodied-RobotSim

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen.svg)](https://docs.ros.org/en/jazzy/index.html)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-6.1-76B900.svg)](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/)
[English](README_en.md)

基于 **ROS 2 Jazzy + Isaac Sim 6.1** 的移动抓取仿真项目。机器人搭载 Franka FR3、MID-360 近似雷达和双目 RGB-D 相机，支持 FAST-LIO 建图定位、Nav2 自动探索与导航、YOLOE 语义感知、CUDA 语义体素建图、GraspNet 抓取及 Qwen3 自然语言任务控制。

## 演示

### Isaac Sim 仿真与抓取

![Isaac Sim 仿真演示](assets/IssacSim.gif)

### 大模型任务闭环

![LLM Agent](assets/Embodied_LLM.gif)

## 项目结构

| 目录 / 文件 | 内容 |
|---|---|
| `start_*.sh`、`stop_robot_sim.sh` | 日常启动与清理入口，仿真脚本无参数即可运行 |
| `scripts/` | 公共启动流程、运行环境、依赖安装及版本清单 |
| `scripts/assets/` | 场景下载、main 分支场景与贴图转 USD；首次启动会调用迁移工具 |
| `scripts/diagnostics/` | 可选的只读性能测量工具 |
| `src/x_bot/` | 机器人描述、Isaac 后端、MoveIt、抓取任务和统一 RViz |
| `src/x_bot_localization/` | Livox 消息桥接、FAST-LIO 适配、地图保存、ICP 定位、Nav2 配置 |
| `src/semantic_voxel_mapping/` | CUDA raycast、hit/miss 占据融合及类别颜色多数投票 |
| `src/yoloe_infer/`、`src/graspnet_infer/` | YOLOE / GraspNet TensorRT 推理 |
| `src/LSM_depth_infer/` | 可选双目深度推理，ROS 包名为 `stereo_matching` |
| `src/isaac_vendor/FAST_LIO_ROS2/` | 安装脚本下载的外部依赖，不提交到仓库 |
| `llm_agent/`、`web_ui/` | Qwen3 服务及 Web 控制界面 |
| `tests/` | 底盘、定位、场景和抓取的离线回归测试，不参与仿真启动 |
| `assets/` | README 演示资源 |

`build/`、`install/`、`log/`、`maps/` 和本地资产缓存是生成数据，已忽略。历史 `docs/validation` 报告不属于运行依赖，已清理；依赖清单合并为 `scripts/isaac.repos`。未被调用的 Qwen API 临时测试和机械臂合成点云避障演示也已移除。

## 首次配置

以下命令从项目根目录执行。ROS/C++ 构建使用系统 Python；Isaac 脚本使用 Isaac 自带的 `python.sh`。

### 1. 系统与 Isaac Sim

| 依赖 | 项目使用版本 / 用途 |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy，包含 Nav2、MoveIt 2、ros2_control |
| Isaac Sim | 6.1，安装目录需包含可执行的 `python.sh` |
| NVIDIA 驱动、CUDA Toolkit | 与 GPU 和 Isaac 兼容；本机 CUDA 13.3，必须能找到 `nvcc` |
| TensorRT | 本机 10.14.1.48，包含开发头文件、库及 `trtexec` |
| 系统 Python | Ubuntu 24.04 的 Python 3.12；避免用 Conda Python 构建 ROS 包 |

按 [Isaac Sim 工作站安装说明](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_workstation.html) 安装；ROS 环境参见 [Isaac ROS 2 配置说明](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html)。项目启动代码会启用 URDF Importer、ROS 2 Bridge/Core、ros2_control 和 experimental physics sensors 扩展，无需逐次手动点击启用。必须使用包含这些扩展的 Isaac Sim 6.1 安装。

把以下配置追加到 `~/.zshrc`（Bash 用户使用 `~/.bashrc`）。路径按实际安装修改；TensorRT 的 tar 包安装还需将其 `bin`、`lib` 加入对应搜索路径。

```bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
export ISAAC_ASSETS_PATH="$HOME/isaacsim_assets/6.1"
export CUDA_HOME=/usr/local/cuda
export PATH="$CUDA_HOME/bin:/usr/src/tensorrt/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

重新打开终端后检查：

```bash
test -x "$ISAAC_SIM_PATH/python.sh"
nvidia-smi
nvcc --version
trtexec --version
```

在交互式 Zsh 中使用 `source /opt/ros/jazzy/setup.zsh` 和 `source install/setup.zsh`。项目 `.sh` 由 Bash 执行，内部加载 `setup.bash`。不要在开启 `set -u` 时直接加载 ROS setup；ROS 脚本会引用可选的未设置变量。若启用了 Conda，先退出 Conda 环境再执行下面的系统构建命令。

### 2. FAST_LIO_ROS2 与 ROS 依赖

```bash
sudo apt update
sudo apt install python3-vcstool python3-rosdep python3-colcon-common-extensions \
  python3-numpy python3-yaml python3-dev libeigen3-dev libpcl-dev \
  ros-jazzy-moveit ros-jazzy-moveit-ros-perception \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# 系统尚未初始化 rosdep 时执行一次：sudo rosdep init
bash scripts/setup_isaac_dependencies.sh
bash -c 'source /opt/ros/jazzy/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y'
```

安装脚本根据 [scripts/isaac.repos](scripts/isaac.repos) 下载 `src/isaac_vendor/FAST_LIO_ROS2`，固定提交 `2fffc570a25d0df172720bac034fbdb6a13d2162` 并递归初始化 `include/ikd-Tree` 子模块。已有不同版本时脚本报错退出，不会覆盖本地修改。检查安装：

```bash
git -C src/isaac_vendor/FAST_LIO_ROS2 rev-parse HEAD
git -C src/isaac_vendor/FAST_LIO_ROS2 submodule status
```

无需安装真实 Livox SDK 或驱动。`src/isaac_livox_interfaces` 提供同名 `livox_ros_driver2` 消息包；不要再加入第二份同名包。不要删除 `src/isaac_vendor/FAST_LIO_ROS2`，它是当前定位链路的构建依赖。

### 3. 模型、插件与编译

从 [模型下载目录](https://drive.google.com/drive/folders/1gPPyvKqiYd7cg2vUyqucV1CjLTf6J0y2?usp=drive_link) 下载：

| 模型 | 放置位置 |
|---|---|
| `yoloe-v8l-text-prompt-multi_nc10_fp16.onnx` | `src/yoloe_infer/models/` |
| `graspnet.onnx` | `src/graspnet_infer/` |

`src/yoloe_infer/models/tokenizer_data.json.gz` 是必要资源，应随源码保留。TensorRT engine 在本机生成，换 GPU 或 TensorRT 版本后重新生成。

```bash
bash src/graspnet_infer/tensorrt_plugins/build.sh

trtexec --onnx=src/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.onnx \
  --saveEngine=src/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.engine
trtexec --onnx=src/graspnet_infer/graspnet.onnx \
  --saveEngine=src/graspnet_infer/graspnet.trt \
  --staticPlugins=src/graspnet_infer/tensorrt_plugins/build/libfps_plugin.so

bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

FPS 插件默认按本机 GPU 编译。遇到旧 CMake 缓存导致的 CUDA 架构错误时，可明确重新配置插件：

```bash
cmake -S src/graspnet_infer/tensorrt_plugins \
  -B src/graspnet_infer/tensorrt_plugins/build -DCMAKE_CUDA_ARCHITECTURES=native
cmake --build src/graspnet_infer/tensorrt_plugins/build -j
```

### 4. 场景资产

默认 `simple_room` 和 `manipulation_test` 从 **main 分支原始场景**转换，保留网格、UV、贴图和层级位姿。迁移工具读取 Git 中的 `main`，新克隆若没有本地 main 分支，先创建：

```bash
git fetch origin main
git branch main origin/main  # 仅在本地 main 不存在时执行
"$ISAAC_SIM_PATH/python.sh" scripts/assets/migrate_gazebo_scenes.py \
  --worlds simple_room manipulation_test
```

默认场景首次启动会自动转换缺失资产。转换结果放在 `$ISAAC_ASSETS_PATH/GazeboMain`，源码及许可证保留在缓存内。运行时仅使用 Isaac Sim；迁移工具名字中的 Gazebo 表示源格式。

使用 NVIDIA Office 或官方 Simple Room 时，额外执行：

```bash
"$ISAAC_SIM_PATH/python.sh" scripts/assets/download_isaac_environments.py
```

该工具同时下载官方场景和转换 main 中的抓取物体。`WORLD=office` 为 Office；`WORLD=isaac_simple_room` 为 NVIDIA Simple Room；`WORLD=simple_room` 为 main 的房间。Office 已配置室内补光。切换场景时同时换 `MAP_BUNDLE` 并重新建图，不能沿用其他场景的定位地图。

## 直接运行

所有入口的默认配置写在各脚本顶部，直接执行即可。每次仿真启动先运行 `stop_robot_sim.sh`，关闭上次项目服务窗口、清理 ROS / Isaac 进程与日志。统一只开一个 RViz：`src/x_bot/rviz/octomap.rviz`。

| 命令 | 默认场景 / 行为 |
|---|---|
| `./start_explore_and_mapping.sh` | `simple_room`，FAST-LIO 建图 + Nav2 自动探索 |
| `./start_navigation.sh` | `simple_room`，已知地图 ICP 定位 + Nav2 |
| `./start_pick_and_place_demo.sh` | `manipulation_test`，扫描、抓取并放置 book / cup / coke / bottle / shoe |
| `./start_navigation_and_pick_demo.sh` | `simple_room`，导航 + 感知抓取 |
| `./start_llm_agent.sh` | `simple_room`，导航 + 抓取 + Qwen3 / Web UI |
| `./stop_robot_sim.sh` | 停止服务并关闭对应窗口 |

探索时保存地图：

```bash
# 在另一个终端加载 ROS 和工作空间环境；Zsh 使用 setup.zsh
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 service call /localization/save_map std_srvs/srv/Trigger '{}'
```

探索默认输出 `maps/gazebo_simple_room`。导航类入口要求目录同时包含 `bundle.json`、`map.pcd`、`map.yaml`、`map.pgm`；只有二维地图无法启动当前 ICP 定位。保存不会覆盖已有目录。导航类任务含固定地图目标点，换地图后须检查任务坐标。

RViz 显示 FAST-LIO 点云、里程计轨迹、语义体素、导航地图及规划路径。导航服务器就绪后使用 **Nav2 Goal**。自动探索默认开启；手动设目标前可将脚本的 `AUTO_EXPLORE=false`，或暂停运行中的探索：

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
```

LLM 模式启动前设置 `DASHSCOPE_API_KEY`，接口配置见 `llm_agent/`；Web UI 地址为 `http://localhost:8888`。不要将密钥写入提交文件。

## 常用配置

| 配置位置 | 修改内容 |
|---|---|
| 各 `start_*.sh` 顶部 | `WORLD`、`MAP_BUNDLE`、出生位姿 `INITIAL_X/Y/YAW`、`HEADLESS`、`BUILD` |
| `start_explore_and_mapping.sh` | `AUTO_EXPLORE`、`SEMANTIC_CLOUD_SOURCE`、`DEPTH_SOURCE` |
| `start_pick_and_place_demo.sh` | 默认 Isaac 深度、`OCTOMAP_RESOLUTION=0.02` |
| `src/x_bot_localization/config/nav2_isaac.yaml` | Nav2 速度、路径跟随、机器人半径、障碍膨胀 |
| `src/x_bot_localization/config/fastlio_mid360.yaml` | FAST-LIO 参数；项目 launch 实际加载此文件 |
| `src/semantic_voxel_mapping/config/map.yaml` / `manipulation.yaml` | 探索 / 抓取的点云输入、范围、分辨率、hit/miss、融合与发布频率 |
| `src/yoloe_infer/configs/config.yaml` / `manipulation.yaml` | 文本类别、颜色、检测阈值、深度语义点云采样步长 |
| `src/x_bot/config/isaac_controllers.yaml` | 控制器及夹爪停滞判定 |
| `src/x_bot/config/pick_and_place_demo.yaml` | 抓取任务与物理验收参数 |
| `src/LSM_depth_infer/config/isaac_params.yaml` | LSM ROS 输入与输出；详见 [LSM README](src/LSM_depth_infer/README.md) |

### FAST-LIO 仿真配置

外部仓库的默认 `config/mid360.yaml` **不是项目启动时加载的配置**。修改 `src/x_bot_localization/config/fastlio_mid360.yaml` 后重新构建或通过现有 symlink install 更新配置即可；无需改 vendor 源码。

| 参数 | 当前值 / 含义 |
|---|---|
| `common.lid_topic` / `imu_topic` | `/livox/lidar` / `/livox/imu` |
| `preprocess.lidar_type` / `scan_line` | `1`（Livox CustomMsg）/ `4` |
| `preprocess.timestamp_unit` / `scan_rate` | `3`（ns）/ `10` Hz，按仿真时间 |
| `common.time_sync_en` / `time_offset_lidar_to_imu` | `false` / `0.0`，雷达和 IMU 共用仿真时钟 |
| `mapping.extrinsic_est_en` | `false` |
| `mapping.extrinsic_T` / `extrinsic_R` | 零平移 / 单位矩阵，模拟 IMU 与雷达光心共点 |
| `filter_size_surf` / `filter_size_map` | `0.15` m |
| `point_filter_num` / `max_iteration` | `3` / `4` |

雷达通过 PhysX 碰撞几何近似 MID-360 非重复扫描；物理及 IMU 为 200 Hz、点云为 10 Hz（仿真时间），逐点采样偏移为 5 ms 分辨率。它不是官方 Livox 光学模型。机器人内部 TF 由 robot_state_publisher 发布，FAST-LIO 原生 TF 重映射为私有话题；导航链为 `map → odom → base_footprint → mid360_imu_link`。`map → odom` 在建图时由地图节点提供、定位时由 ICP 提供。真值 `/debug/ground_truth/odom` 仅用于调试。

`INITIAL_X/Y/YAW` 是底盘在地图中的先验位姿；已知地图定位不支持无先验全局搜索，初值不准时在 RViz 用 **2D Pose Estimate** 修正。定位不健康时安全门控停止底盘。仿真时钟重置后需重启完整定位链路。

### 语义建图与抓取

探索默认 `SEMANTIC_CLOUD_SOURCE=fastlio`：同期 FAST-LIO 点云投影到相机图像，FOV 内命中 YOLOE mask 的点赋类别颜色，其他点保留未知几何。二维导航地图仍由 FAST-LIO 点云生成。

抓取默认 `SEMANTIC_CLOUD_SOURCE=depth`、`DEPTH_SOURCE=isaac`，使用 `/x_bot/camera_left/depth/image_raw`；改为 `DEPTH_SOURCE=lsm` 使用 `/x_bot/camera_left/nn_depth`。FAST-LIO 模式无需运行 LSM。

所有语义建图统一使用 `semantic_voxel_mapping`，输入 `/yoloe_multi_text_prompt/pointcloud_semantic`，CUDA raycast 更新 hit/miss，类别 RGB 跨帧多数投票。抓取地图分辨率 2 cm，融合上限 5 Hz、发布 2 Hz。语义深度点云默认 `semantic_depth_stride: 2`，GraspNet 点云仍为全密度。暂停识别时仍发布几何点云，使移走物体的位置能通过有效 miss 射线逐步清空。详见 [语义地图 README](src/semantic_voxel_mapping/README.md)。

夹爪闭合接触可接受停滞，张开必须到位；动作成功不等同于抓住物体。默认物理校验检查抬升、搬运保持和入桶后停留，`/isaac/debug/object_states` 为只读验收数据，不参与检测、目标计算或物体移动。MoveIt 桌子碰撞配置在 `manipulation_fixture.yaml`，搬运物体边界来自分割深度点云。

## 验证与排查

```bash
# 离线回归；不启动仿真、不控制机器人
python3 -m unittest discover -s tests -v

# 已编译工作空间中的 C++ 回归
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --packages-select x_bot_localization semantic_voxel_mapping yoloe_infer'
for package in x_bot_localization semantic_voxel_mapping yoloe_infer; do
  colcon test-result --test-result-base "build/$package" --verbose
done

# 在已加载 ROS 环境的终端检查运行状态
ros2 topic echo /localization/ready
ros2 topic echo /localization/status
ros2 control list_controllers
ros2 run tf2_ros tf2_echo map base_footprint
python3 scripts/diagnostics/measure_isaac_performance.py --seconds 30 --output /tmp/isaac_performance.json
```

性能工具输出实时倍率及按仿真时间统计的传感器频率。实际观看速度受实时倍率影响，配置 1 m/s 不代表墙钟时间内也移动 1 m。入口设置 `HEADLESS=true` 可关闭观察视口，相机和感知仍运行。完整 CPU/GPU 负载会影响帧率，测量工具也有额外开销。

若 GNOME Terminal 报旧 screen object path 错误，当前公共启动脚本会清除继承的 `GNOME_TERMINAL_SCREEN` / `GNOME_TERMINAL_SERVICE`。没有桌面终端时服务写入 `log/isaac_sim`；有桌面时看各服务窗口及 ROS 日志。`Package ... not found` 时检查依赖安装、构建结果和当前终端的工作空间环境。

## 致谢

[FAST-LIO](https://github.com/hku-mars/FAST_LIO)、[FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)、[YOLOE](https://github.com/THU-MIG/yoloe)、[GraspNet](https://github.com/graspnet/graspnet-baseline)、[m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)、[franka_ros2](https://github.com/frankaemika/franka_ros2)、[bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot)。原始场景和模型的许可证随资产保留。
