# Embodied-RobotSim

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen.svg)](https://docs.ros.org/en/jazzy/index.html)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-6.1-76B900.svg)](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/)
[English](README_en.md)

基于 **ROS 2 Jazzy + Isaac Sim 6.1 / Gazebo Harmonic** 的移动抓取仿真项目。机器人搭载 Franka FR3、MID-360 近似雷达和双目 RGB-D 相机，支持 FAST-LIO 建图定位、Nav2 自动探索与导航、YOLOE 语义感知、CUDA 语义体素建图、GraspNet 抓取及 Qwen3 自然语言任务控制。

## 演示

### Isaac Sim 仿真与抓取

![Isaac Sim 仿真演示](assets/IssacSim.gif)

### Gazebo Harmonic 仿真

![Gazebo Harmonic 仿真演示](assets/GazeboSim.gif)

两种仿真器共用启动入口：无参数默认 Isaac Sim，添加 `--sim gazebo` 切换到 Gazebo。

```bash
./start_mapping.sh
./start_mapping.sh --sim gazebo
```

### 大模型任务闭环

![LLM Agent](assets/Embodied_LLM.gif)

## 项目结构

| 目录 | 职责 / ROS 包 |
|---|---|
| `scripts/runtime/` | 公共服务调度、Isaac / Gazebo 后端、Web 和窗口清理 |
| `scripts/setup/`、`scripts/assets/`、`scripts/diagnostics/` | 依赖准备、历史场景提取与 USD 转换、诊断 |
| `src/description/` | `x_bot`、`franka_description`：URDF 与网格；保留 `package://x_bot/` URI |
| `src/localization/` | `x_bot_localization`：FAST-LIO 适配、ICP 与定位 TF |
| `src/perception/` | YOLOE、GraspNet、FPS TensorRT 插件和本地可选 LSM |
| `src/planning/` | `explore_lite`、`x_bot_navigation`、`x_bot_moveit_config`、Franka MoveIt 配置和统一 RViz |
| `src/control/` | `x_bot_control`：速度仲裁、安全门、碰撞点云、连续路径控制插件、底盘及关节参数 |
| `src/mapping/` | `semantic_voxel_mapping`、`x_bot_mapping`：语义体素、二维栅格、完整地图包保存 |
| `src/manipulation/` | `x_bot_manipulation`：扫描、Go Home、夹爪、抓取放置和物理验收 |
| `src/simulation/` | `x_bot_isaac`、`x_bot_gazebo`、`x_bot_scene_assets`：后端与公共场景处理 |
| `src/sensors/` | `x_bot_sensors`：Livox 桥接、MID-360 采样及公共配置 |
| `src/embodied/` | Agent 服务及 Web UI |
| `src/common/` | `x_bot_common` 几何运算、`x_bot_bringup` 系统组合 launch |
| `src/interfaces/`、`src/vendor/` | Franka / Livox 消息；固定版本 FAST-LIO（忽略本地源码） |
| `maps/`、`assets/`、`tests/` | 运行地图、演示动图、离线 / C++ 回归 |

包名与话题、服务、动作及 TF 保持原有接口；拆出的功能使用独立包名，没有旧 ROS 命令转发壳。

`build/`、`install/`、`log/`、`maps/` 和本地资产缓存是生成数据，已忽略。历史 `docs/validation` 报告不属于运行依赖，已清理；依赖清单合并为 `scripts/setup/vendor.repos`。未被调用的 Qwen API 临时测试和机械臂合成点云避障演示也已移除。

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

按 [Isaac Sim 工作站安装说明](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_workstation.html) 安装；ROS 环境参见 [Isaac ROS 2 配置说明](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html)。Isaac 后端启动代码会启用 URDF Importer、ROS 2 Bridge/Core、ros2_control 和 experimental physics sensors 扩展，无需逐次手动点击启用。必须使用包含这些扩展的 Isaac Sim 6.1 安装。

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
bash scripts/setup/setup_dependencies.sh
bash -c 'source /opt/ros/jazzy/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y'
```

安装脚本根据 [scripts/setup/vendor.repos](scripts/setup/vendor.repos) 下载 `src/vendor/FAST_LIO_ROS2`，固定提交 `2fffc570a25d0df172720bac034fbdb6a13d2162` 并递归初始化 `include/ikd-Tree` 子模块。已有不同版本时脚本报错退出，不会覆盖本地修改。检查安装：

```bash
git -C src/vendor/FAST_LIO_ROS2 rev-parse HEAD
git -C src/vendor/FAST_LIO_ROS2 submodule status
```

无需安装真实 Livox SDK 或驱动。`src/interfaces/livox_ros_driver2` 提供同名 `livox_ros_driver2` 消息包；不要再加入第二份同名包。不要删除 `src/vendor/FAST_LIO_ROS2`，它是当前定位链路的构建依赖。

### 3. 模型、插件与编译

从 [模型下载目录](https://drive.google.com/drive/folders/1gPPyvKqiYd7cg2vUyqucV1CjLTf6J0y2?usp=drive_link) 下载：

| 模型 | 放置位置 |
|---|---|
| `yoloe-v8l-text-prompt-multi_nc10_fp16.onnx` | `src/perception/yoloe_infer/models/` |
| `graspnet.onnx` | `src/perception/graspnet_infer/` |

`src/perception/yoloe_infer/models/tokenizer_data.json.gz` 是必要资源，应随源码保留。TensorRT engine 在本机生成，换 GPU 或 TensorRT 版本后重新生成。

```bash
bash src/perception/graspnet_infer/tensorrt_plugins/build.sh

trtexec --onnx=src/perception/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.onnx \
  --saveEngine=src/perception/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.engine
trtexec --onnx=src/perception/graspnet_infer/graspnet.onnx \
  --saveEngine=src/perception/graspnet_infer/graspnet.trt \
  --staticPlugins=src/perception/graspnet_infer/tensorrt_plugins/build/libfps_plugin.so

bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

FPS 插件默认按本机 GPU 编译。遇到旧 CMake 缓存导致的 CUDA 架构错误时，可明确重新配置插件：

```bash
cmake -S src/perception/graspnet_infer/tensorrt_plugins \
  -B src/perception/graspnet_infer/tensorrt_plugins/build -DCMAKE_CUDA_ARCHITECTURES=native
cmake --build src/perception/graspnet_infer/tensorrt_plugins/build -j
```

### 4. 场景资产

默认 `simple_room` 和 `manipulation_test` 从 **main 分支原始场景**转换，保留网格、UV、贴图和层级位姿。迁移工具读取 Git 中的 `main`，新克隆若没有本地 main 分支，先创建：

```bash
git fetch origin main
git branch main origin/main  # 仅在本地 main 不存在时执行
"$ISAAC_SIM_PATH/python.sh" scripts/assets/migrate_gazebo_scenes.py \
  --worlds simple_room manipulation_test
```

Isaac 默认场景首次启动会自动转换缺失资产。转换结果放在 `$ISAAC_ASSETS_PATH/GazeboMain`，源码及许可证保留在缓存内。Isaac 使用转换后的 USD；Gazebo 使用同一固定源提交提取的 SDF 和模型。迁移工具名字中的 Gazebo 表示源格式。

使用 NVIDIA Office 或官方 Simple Room 时，额外执行：

```bash
"$ISAAC_SIM_PATH/python.sh" scripts/assets/download_isaac_environments.py
```

该工具同时下载官方场景和转换 main 中的抓取物体。`WORLD=office` 为 Office；`WORLD=isaac_simple_room` 为 NVIDIA Simple Room；`WORLD=simple_room` 为 main 的房间。Office 已配置室内补光。切换场景时同时换 `MAP_BUNDLE` 并重新建图，不能沿用其他场景的定位地图。

## Gazebo Harmonic 配置与切换

默认以 Isaac Sim 的抓取、建图、底盘和 Nav2 参数为基准，Gazebo 使用相同任务接口。仅切换仿真器不会恢复旧 main 的 Cartographer 或真值导航。

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-sim-vendor ros-jazzy-gz-common-vendor ros-jazzy-gz-plugin-vendor libembree-dev
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
./start_mapping.sh --sim gazebo
./start_pick.sh --sim gazebo
```

也可修改入口顶部的 `SIM_BACKEND=isaac` 为 `gazebo`，保持无参数启动。缺少 Gazebo 或 Embree 时，可选包 `x_bot_gazebo` 不构建插件，Isaac 仍可构建运行；Gazebo 启动会检查插件是否存在。底盘公共配置在 `src/control/x_bot_control/config/base_motion.yaml`，值沿用 Isaac 基线；机械臂和夹爪默认参数共享 `isaac_controllers.yaml`，Gazebo 的接口覆盖在 `gazebo_controllers.yaml`；Gazebo 原生位置伺服控制其他关节，第 2 关节的力驱动、夹爪驱动与接触摩擦配置在 `gazebo_physics.yaml`。

Gazebo 首批支持 `simple_room`、`manipulation_test`，不支持 Office 或其他 USD 专属场景。Gazebo 默认地图目录分别为 `maps/gazebo/simple_room`、`maps/gazebo/manipulation_test`；Isaac 保留原路径；Gazebo 语义地图也保存到对应地图目录的 `semantic_map.svm`。两种后端自动探索结束时调用 `/localization/save_map`，保存完整地图包；已有目录会拒绝覆盖，重新建图时指定新的 `--bundle`。Gazebo 导航入口同样需要先保存完整地图包。

`DEPTH_SOURCE=sim` 表示当前仿真器的深度图；Isaac 后端兼容旧值 `isaac`，Gazebo 使用 `sim`，两者继续支持 `lsm`。三维雷达和 IMU 输出共用 FAST-LIO，二维地图及扫描从去畸变点云派生。Gazebo 原生真值不发布导航 TF。

Gazebo MID-360 插件在每个 5 ms 物理步采集 1,000 条真实碰撞几何射线，默认 50 ms 组帧（20 Hz）；扫描方向、范围、字段及真实采样偏移与当前 Isaac 近似算法一致。Embree 查询使用 Gazebo 原生网格加载器，动态物体和机械臂位姿持续更新；命中机器人自身的射线不进入输出。动态场景网格使用凸分解，物理引擎和 Embree 共用 Gazebo 的网格加载接口；瓶子阻尼通过真实力和力矩实现。两种物理引擎的接触与运行速度可能不同。

源码资产固定来自提交 `92b0409ccf83549e74d03966bdde7f0f700ff927`，不依赖未来 main 的文件布局。浅克隆需获取包含该提交的完整历史；模型及许可证保留在缓存内。Gazebo 提取资产不需要 Isaac Python：

```bash
python3 scripts/assets/prepare_gazebo_scene.py --world simple_room
```

## 直接运行

所有入口的默认配置写在各脚本顶部，直接执行默认使用 Isaac Sim；增加 `--sim gazebo` 可切换。每次仿真启动先运行 `stop_robot_sim.sh`，关闭上次项目服务窗口、清理 ROS / Isaac / Gazebo 进程与日志。统一只开一个 RViz：`src/planning/x_bot_navigation/rviz/octomap.rviz`。

| 命令 | 默认场景 / 行为 |
|---|---|
| `./start_mapping.sh` | `simple_room`，FAST-LIO 建图 + Nav2 自动探索 |
| `./start_mapping.sh --mode navigation` | `simple_room`，已知地图 ICP 定位 + Nav2 |
| `./start_pick.sh` | `manipulation_test`，扫描、抓取并放置 book / cup / coke / bottle / shoe |
| `./start_pick.sh --mode navigation` | `simple_room`，导航 + 感知抓取 |
| `./start_embodied.sh` | `simple_room`，导航 + 抓取 + Qwen3 / Web UI |
| `./stop_robot_sim.sh` | 停止服务并关闭对应窗口 |

探索时保存地图：

```bash
# 在另一个终端加载 ROS 和工作空间环境；Zsh 使用 setup.zsh
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 service call /localization/save_map std_srvs/srv/Trigger '{}'
```

Isaac 探索默认输出 `maps/gazebo_simple_room`，Gazebo 默认输出 `maps/gazebo/simple_room`。导航类入口要求目录同时包含 `bundle.json`、`map.pcd`、`map.yaml`、`map.pgm`；只有二维地图无法启动当前 ICP 定位。保存不会覆盖已有目录；再次启动建图入口时，已有输出目录会保留，本次输出使用带时间戳的新目录，并打印实际路径。使用新地图导航时将该路径传给 `--bundle`。导航类任务含固定地图目标点，换地图后须检查任务坐标。

RViz 显示 FAST-LIO 点云、里程计轨迹、语义体素、导航地图及规划路径。导航服务器就绪后使用 **Nav2 Goal**。探索的卡住检测按累计 5 cm / 0.17 rad 进展及 20 秒仿真时间判断，不再按连续三次小位移提前打断 Nav2 脱困。探索点仅作为观测位置：当前目标对应的未知边界全部在 `/map` 中变为已知（空地或障碍）后，提前取消该导航目标并选择下一个，不要求抵达探索点。未观测到的边界继续探索；`params_costmap.yaml` 中 `finish_on_observation` 默认开启，`observation_map_topic` 指定原始二维地图。自动探索默认开启；手动设目标前可将脚本的 `AUTO_EXPLORE=false`，或暂停运行中的探索：

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
```

LLM 模式启动前设置 `DASHSCOPE_API_KEY`，接口配置见 `src/embodied/llm_agent/`；Web UI 地址为 `http://localhost:8888`。不要将密钥写入提交文件。

## 常用配置

| 配置位置 | 修改内容 |
|---|---|
| 各 `start_*.sh` 顶部 | `WORLD`、`MAP_BUNDLE`、出生位姿 `INITIAL_X/Y/YAW`、`HEADLESS`、`BUILD` |
| `start_mapping.sh` | `AUTO_EXPLORE`、`SEMANTIC_CLOUD_SOURCE`、`DEPTH_SOURCE` |
| `start_pick.sh` | 默认 Isaac 深度、`OCTOMAP_RESOLUTION=0.02` |
| `src/planning/x_bot_navigation/config/nav2.yaml` | Nav2 速度、路径跟随、机器人半径、障碍膨胀 |
| `src/localization/x_bot_localization/config/fastlio_mid360.yaml` | FAST-LIO 参数；项目 launch 实际加载此文件 |
| `src/mapping/semantic_voxel_mapping/config/map.yaml` / `manipulation.yaml` | 探索 / 抓取的点云输入、范围、分辨率、hit/miss、融合与发布频率 |
| `src/perception/yoloe_infer/configs/config.yaml` / `manipulation.yaml` | 文本类别、颜色、检测阈值、深度语义点云采样步长 |
| `src/control/x_bot_control/config/isaac_controllers.yaml` | 控制器及夹爪停滞判定 |
| `src/manipulation/x_bot_manipulation/config/pick_and_place_demo.yaml` | 抓取任务与物理验收参数 |
| `src/perception/LSM_depth_infer/config/isaac_params.yaml` | 可选本地 LSM 模块的 ROS 输入与输出，需自行提供模块及配置 |

两种后端默认使用 Nav2 **Graceful Controller**，以连续曲率控制律同时生成前进和转向速度，前视距离 0.3–1.0 m。`initial_rotation: false`、`enable_heading_alignment: false` 关闭“先原地对准再前进”的切换；路径仍通过碰撞检查和速度平滑器，空间不足时可以停止，目标末端保留最终朝向停车。外层 `ForwardHeadingController` 保留 TF 新鲜度检查与速度上限，`tracking_goal_tolerance_margin: 0.08` 修正 Jazzy Graceful 用最近路径栅格点计算剩余距离时提前进入原地转向的问题，控制服务器的目标容差仍是 0.25 m。

公共上限为 1 m/s、1.2 rad/s；底盘仍为四轮滑移转向，前进和转向同时执行，不启用侧移。规划半径为 0.47 m（另有 0.01 m padding），碰撞监控固定保留原 0.42 m + 0.01 m padding 多边形。障碍物附近与目标末端会减速，碰撞保护及定位失效停车保持启用。参数在 `src/planning/x_bot_navigation/config/nav2.yaml`，修改后重启导航服务。依赖由 `rosdep install --from-paths src --ignore-src -r -y` 安装，Jazzy 包名为 `ros-jazzy-nav2-graceful-controller`。文件保留 MPPI 参数供对比；切回 MPPI 时将 `FollowPath.primary_controller` 改为 `nav2_mppi_controller::MPPIController`，同时将 `tracking_goal_tolerance_margin` 设为 `0.0`。

### FAST-LIO 仿真配置

外部仓库的默认 `config/mid360.yaml` **不是项目启动时加载的配置**。修改 `src/localization/x_bot_localization/config/fastlio_mid360.yaml` 后重新构建或通过现有 symlink install 更新配置即可；无需改 vendor 源码。

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

雷达通过 PhysX 碰撞几何近似 MID-360 非重复扫描；物理及 IMU 为 200 Hz、点云默认 20 Hz（仿真时间），逐点采样偏移为 5 ms 分辨率。Isaac 底盘轮速使用缓存的 OmniGraph 运行时属性写入，避免每个物理步创建 USD 修改和撤销记录。发布频率在 `src/sensors/x_bot_sensors/config/mid360.yaml` 的 `publish_hz` 中配置，两种后端和 FAST-LIO 共用；支持 10/20/25/40 Hz，修改后重启完整仿真。射线采样总量不变，提高组帧频率会减少每帧点数。`ros2 topic hz` 显示真实时间频率，等于配置频率 × 仿真实时率，例如实时率 0.68 时，20 Hz 雷达约输出 13.6 Hz。它不是官方 Livox 光学模型。机器人内部 TF 由 robot_state_publisher 发布，FAST-LIO 原生 TF 重映射为私有话题；导航链为 `map → odom → base_footprint → mid360_imu_link`。`map → odom` 在建图时由地图节点提供、定位时由 ICP 提供。真值 `/debug/ground_truth/odom` 仅用于调试。

`INITIAL_X/Y/YAW` 是底盘在地图中的先验位姿；已知地图定位不支持无先验全局搜索，初值不准时在 RViz 用 **2D Pose Estimate** 修正。定位不健康时安全门控停止底盘。仿真时钟重置后需重启完整定位链路。

### 语义建图与抓取

探索默认 `SEMANTIC_CLOUD_SOURCE=fastlio`：同期 FAST-LIO 点云投影到相机图像，FOV 内命中 YOLOE mask 的点赋类别颜色，其他点保留未知几何。二维导航地图仍由 FAST-LIO 点云生成。

PCA 顶抓使用同帧分割深度的中心和方向；回收机械臂保持已完成的垂直抬升高度，1 m 仍作为最低回收高度。物理成功判定只读取中立状态话题，不提供运动目标。

抓取默认 `SEMANTIC_CLOUD_SOURCE=depth`、`DEPTH_SOURCE=sim`，使用 `/x_bot/camera_left/depth/image_raw`；改为 `DEPTH_SOURCE=lsm` 使用 `/x_bot/camera_left/nn_depth`。FAST-LIO 模式无需运行 LSM。`src/perception/LSM_depth_infer` 已从版本控制移除，现有本地文件保留；新克隆选择 `lsm` 前需自行提供该模块、配置和推理引擎并构建 `stereo_matching`。

所有语义建图统一使用 `semantic_voxel_mapping`，输入 `/yoloe_multi_text_prompt/pointcloud_semantic`，CUDA raycast 更新 hit/miss，类别 RGB 跨帧多数投票。抓取地图分辨率 2 cm，融合上限 5 Hz、发布 2 Hz。语义深度点云默认 `semantic_depth_stride: 2`，GraspNet 点云仍为全密度。抓取过程中 YOLOE 保持推理和结果画面输出，仅通过 `/yoloe_multi_text_prompt/enable_pointcloud`（`std_srvs/srv/SetBool`）暂停彩色及语义点云；抓取结束或失败时自动恢复，建图继续通过有效 miss 射线清空移走物体的位置。详见 [语义地图 README](src/mapping/semantic_voxel_mapping/README.md)。

夹爪闭合接触可接受停滞，张开必须到位；动作成功不等同于抓住物体。默认物理校验检查抬升、搬运保持和入桶后停留，`/simulation/debug/object_states` 为两种后端的只读验收数据（Isaac 保留 `/isaac/debug/object_states` 别名），不参与检测、目标计算或物体移动。MoveIt 桌子碰撞配置在 `manipulation_fixture.yaml`，搬运物体边界来自分割深度点云。

单点与多点导航行为树位于 `src/planning/x_bot_navigation/behavior_trees/`，每次全局规划后调用 `simple_smoother` 再跟踪，减少栅格折线的转向突变；开启平滑结果碰撞检查，平滑失败时回退原规划路径，重规划期间继续跟踪已有路径。

`bt_navigator.default_server_timeout: 200` 的单位是毫秒，仅控制动作请求应答等待；原 20 ms 容易在重规划调度抖动时中断导航。它不修改定位健康检查或速度命令超时停车规则。

脱困顺序为清理代价地图、低速倒退 0.20 m、旋转、等待。`contact_scan_guard` 保留 `/x_bot/scan_collision` 调试输出，并将扫描时刻的障碍点转换到 `odom`，发布碰撞监控专用 `/x_bot/collision_points`：仅在新鲜定位和扫描、低速直退（不超过 0.10 m/s）、后方有充分扫描覆盖且 0.30 m 倒退扫掠范围没有新障碍时，允许退离已有前方接触。侧面或后方接触、转动、数据过期均保留完整碰撞扫描。规划和建图始终使用原始扫描；该节点未启动时碰撞监控因缺少输入而停车。碰撞监控使用最新可用的 `odom → base_link` 变换处理每条平滑速度指令，避免等待下一帧定位而把速度输出降到扫描频率；点云保留原扫描时间戳，超过 0.3 秒仿真时间即停车。原始扫描不变，扫描期间到最新定位的运动补偿通过 `odom` 中的固定障碍点保留。

底盘的打滑转向反馈保留小幅指令调整时的积分补偿，明显减速、反向或实际超速时平滑卸载，避免正常路径修正反复丢失转向力。`base_motion.yaml` 中 `yaw_integral_release_time: 0.08` 是补偿卸载时间，`wheel_yaw_accel: 12.0` 限制轮差速指令的变化率；它们不改变车身 1 m/s、1.2 rad/s 上限。零指令、定位失效与命令超时仍立即停止。修改这些底盘参数后需重启仿真器。

## 验证与排查

```bash
# 离线回归；不启动仿真、不控制机器人
python3 -m unittest discover -s tests -v

# 连续曲线、90° 起步及取消停车；79 必须是未使用的 ROS domain
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ROS_DOMAIN_ID=79 python3 tests/check_nav2_motion.py --initial-yaw 1.5708 --odom-hz 10 --tf-lag 0.05'

# 已编译工作空间中的 C++ 回归
bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --packages-select x_bot_control explore_lite x_bot_localization semantic_voxel_mapping yoloe_infer x_bot_gazebo'
for package in x_bot_control explore_lite x_bot_localization semantic_voxel_mapping yoloe_infer x_bot_gazebo; do
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

首次启动可能需要数分钟编译 RTX 着色器或生成碰撞凸分解。控制器顺序加载以避免多个加载进程争抢锁；等待管理器最长 600 秒、单次服务调用最长 300 秒、激活切换最长 120 秒，适应初始化时的卡顿；可查看对应后端日志判断进度。

若 GNOME Terminal 报旧 screen object path 错误，当前公共启动脚本会清除继承的 `GNOME_TERMINAL_SCREEN` / `GNOME_TERMINAL_SERVICE`。没有桌面终端时服务写入 `log/isaac` 或 `log/gazebo`；有桌面时看各服务窗口及 ROS 日志。`Package ... not found` 时检查依赖安装、构建结果和当前终端的工作空间环境。

## 致谢

[FAST-LIO](https://github.com/hku-mars/FAST_LIO)、[FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)、[YOLOE](https://github.com/THU-MIG/yoloe)、[GraspNet](https://github.com/graspnet/graspnet-baseline)、[m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)、[franka_ros2](https://github.com/frankaemika/franka_ros2)、[bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot)。原始场景和模型的许可证随资产保留。

## 旧入口迁移

根目录仅保留三个启动入口和一个停止入口。旧脚本已移除；默认后端、底盘与抓取参数沿用 Isaac 配置。

| 旧命令 | 新命令 |
|---|---|
| `start_explore_and_mapping.sh` | `./start_mapping.sh` |
| `start_navigation.sh` | `./start_mapping.sh --mode navigation` |
| `start_pick_and_place_demo.sh` | `./start_pick.sh` |
| `start_navigation_and_pick_demo.sh` | `./start_pick.sh --mode navigation` |
| `start_llm_agent.sh` | `./start_embodied.sh` |

所有入口支持 `--sim isaac|gazebo`、`--world`、`--bundle`、深度来源等现有选项。地图文件统一放在根目录 `maps/`；迁移前生成的二维地图保留在 `maps/legacy/` 和 `maps/pre_reorganization/`，原完整地图包路径保持不变。需要重新建图时指定尚不存在的地图目录。

结构迁移后必须重新构建，旧 install 不再有效。可使用隔离目录验证：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base .cache/check/log build --build-base .cache/check/build --install-base .cache/check/install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source .cache/check/install/setup.bash
python3 -m unittest discover -s tests -p 'test_*.py'
ctest --test-dir .cache/check/build/x_bot_control -R '^test_' --output-on-failure
```

Agent / Web 依赖通过独立安装脚本准备，仿真启动时不执行安装：

```bash
bash scripts/setup/setup_embodied_dependencies.sh --local
export DASHSCOPE_API_KEY='your-key'
./start_embodied.sh
```

入口优先使用 `.venv/agent/bin/python`（可访问系统 ROS 包），也可设置 `AGENT_PYTHON`。Web 日志保存在 `log/web/`。

`--local` 将 Web 的 ROS 依赖解包到忽略目录 `.cache/embodied_sysroot/`，Agent 依赖安装到 `.venv/agent/`，不需要 sudo；具身入口会加载该本地环境。省略 `--local` 则使用 apt 安装系统包。没有 API 密钥时，Agent 状态接口仍可运行，但不会执行 LLM 指令。
