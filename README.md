# Embodied-RobotSim: 基于大模型的具身智能移动抓取仿真系统

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen.svg)](https://docs.ros.org/en/jazzy/index.html)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-6.1-76B900.svg)](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/)
[![English](https://img.shields.io/badge/🌍_Language-English-blue.svg)](README_en.md)

Embodied-RobotSim 是一个基于 ROS 2 (Jazzy) 构建的综合仿真工作空间。差速移动平台搭载 **Franka FR3 机械臂**、激光雷达及 RGB-D 深度传感器，Isaac Sim 使用顶部 MID-360 近似仿真和 FAST-LIO 定位链路。项目集成建图、导航、视觉感知、移动抓取以及 **Qwen3 大语言模型驱动的具身智能闭环**，支持自然语言指令和 Web UI 控制。

## 🎬 演示 (Demos)

### 1. 具身大模型闭环 (LLM Agent)
![LLMAgent](assets/Embodied_LLM.gif)

*基于 Qwen3 与视觉模型的多模态指令闭环*

### 2. 移动抓取 (Pick & Place)
![Pick&Place](assets/Pick&Place.gif)

*基于 YOLOE 与 GraspNet 的自主抓取*

### 3. 仿真环境 (Isaac Sim)

*Isaac Sim 6.1 室内场景仿真，集成 Nav2、MoveIt 2 和感知栈*

## 🌟 主要特性

* **Isaac Sim 仿真:** 使用 Xacro 导入机器人，连接 ROS 2 话题和控制器接口。
* **移动抓取操作 (Mobile Manipulation):** 为 Franka FR3 机械臂提供 MoveIt 2 集成，同时支持稳定的四轮差速移动底盘控制。
* **自主探索与建图:** 使用 MID-360 + FAST-LIO 建图、已知地图 ICP 定位，配合 `m-explore-ros2` 进行前沿探索。
* **先进感知系统 (视觉):** 
  * **YOLOE 感知推理:** 支持输入文本提示词 (Text Prompt) 的实时多目标检测 (`yoloe_infer`)。
* **语义占据栅格与任意物体抓取:** 
  * 通过 OctoMap 生成三维语义占据地图。
  * **自主抓取集成:** 结合 **YOLOE** (目标定位) 与 **GraspNet** (位姿估计)，直接由点云生成高成功率的 6-DoF 抓取位姿。系统能够对环境中的未知或任意姿态物体进行抓取，并在点云和 OctoMap 级别支持复杂的碰撞检测与避障逻辑。
* **具身智能与多模态大模型交互:**
  * **Qwen3 LLM 引擎:** 集成 Qwen3 大语言模型，支持将自然语言指令解析为机器人任务序列（导航、抓取等）。
  * **场景预识别:** 结合视觉大模型 (VLM)，在执行任务前自动“看一眼”当前环境，动态调整并规划后续操作。
  * **基于 WebSocket 的全功能 Web UI:** 提供直观美观的控制面板浏览器端界面（包含地图、相机流、手柄遥控、状态展示和 AI 聊天侧边栏），彻底摆脱复杂的终端操作。

## 📦 架构概览

### ROS 2 功能包列表

| 功能包 | 用途 |
|---------|---------|
| `x_bot` | 核心机器人包：包含 URDF、Isaac Sim 后端、场景、启动脚本、导航配置以及 MoveIt 机械臂控制节点 (`robot_actions`)。 |
| `yoloe_infer` | 基于 TensorRT 加速的 YOLOE 文本提示目标检测。 |
| `graspnet_infer` | 基于 TensorRT 的 GraspNet 推理封装，支持直接从杂乱点云场景中计算物体的 6-DoF 抓取位姿。 |
| `m-explore-ros2` | 适配 ROS 2 的 `explore_lite` 包，为建图过程提供完全自主的探索与地图边界拓展能力。 |
| `franka_description` | Franka FR3 机械臂的 URDF 描述文件和可视化网格模型。 |
| `franka_ros2` | FR3 的 MoveIt 2 配置包 (`franka_fr3_moveit_config`)。 |

## 🛠️ 环境要求 (Requirements)

为了确保仿真系统的正常运行，请在以下环境下进行部署：

| 依赖项 | 版本 |
|---------|---------|
| **操作系统** | [Ubuntu 24.04 (Noble)](https://ubuntu.com/download/desktop) |
| **ROS 2** | [Jazzy Jalisco](https://docs.ros.org/en/jazzy/installation.html) ([一键安装](https://fishros.org.cn/forum/topic/20)) |
| **Isaac Sim** | 6.1，启用 ROS 2 Bridge、URDF Importer、experimental physics sensors 与 ros2_control 扩展 |
| **CUDA** | [13.1](https://developer.nvidia.com/cuda-toolkit) |
| **TensorRT** | [10.14.1.48](https://developer.nvidia.com/tensorrt) |
| **Python** | 3.12+ |

> **💻 测试硬件参考 (Tested Hardware)**
>
> 本项目在以下主流中端配置上运行流畅，无需高端工作站即可快速部署：
> *   **CPU**: Intel Core i5-13400F
> *   **GPU**: NVIDIA GeForce RTX 4060
> *   **内存**: 32GB RAM

## 📦 权重文件下载 (Models Download)

由于模型文件较大，请从以下链接下载预训练权重并放置到指定目录：

*   **下载链接**：[Google Drive 文件夹](https://drive.google.com/drive/folders/1gPPyvKqiYd7cg2vUyqucV1CjLTf6J0y2?usp=drive_link)

| 文件名 | 存放路径 (相对于项目根目录) |
| :--- | :--- |
| `yoloe-v8l-text-prompt-multi_nc10_fp16.onnx` | `src/yoloe_infer/models/` |
| `graspnet.onnx` | `src/graspnet_infer/` |

> **注意**：`.trt` 和 `.engine` 文件不再提供，需根据下方说明在本机从 ONNX 文件自行生成。

## 🚀 快速启动指南

> **重要**：ROS 2 Jazzy 是公共依赖。所有仿真启动脚本统一使用 Isaac Sim 6.1；运行完整视觉抓取功能时仍需 CUDA、TensorRT 和对应模型。

### 1. 编译工作空间

```bash
cd /path/to/Embodied-RobotSim
# 1. 编译 TensorRT 插件 (GraspNet 依赖)
bash src/graspnet_infer/tensorrt_plugins/build.sh

# 2. 安装依赖扩展包 (rosdep 及额外系统包)
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-jazzy-moveit-ros-perception

# 3. 构建所有功能包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Isaac Sim 6.1：顶部 MID-360 + FAST-LIO

> **验证范围：** 已在目标机完成 ROS/C++ 构建、Isaac Sim 6.1 静止 IMU/TF、Nav2 运动与连续前沿目标探索联测。已知地图 ICP、完整场景覆盖和抓取/LLM 全流程仍需分别验证。

五个演示入口统一使用 **Isaac Sim**，通过共享 Xacro 导入机器人，集成 FR3、RGB-D、Nav2、MoveIt。

#### 传感器与定位设计

* MID-360 外形为 65 × 65 × 60 mm、质量 0.265 kg，固定在底盘顶板前部，默认相对 `roof_link` 为 `xyz=[0.25, 0, 0.0875]`、`rpy=[0,0,0]`，避开中央 FR3 底座。模拟 IMU 与雷达光心共点，FAST-LIO 外参为单位变换，不使用真实设备的内部标定值。
* 使用 **PhysX 碰撞几何射线**近似非重复扫描：水平 360°、垂直 -7°～52°、0.1～40 m、200,000 条射线/仿真秒、10 Hz 点云。不是官方 Livox 光学模型，也不是 RTX 材质回波模型；强度为常量，遮挡由碰撞几何决定，自身命中丢弃。40 m 为当前仿真截断距离。
* 每个 200 Hz 物理步真实采集 1,000 条射线，每 100 ms 组帧。点的 `offset_time` 是实际物理采样时刻，**5 ms 分辨率**，不是给瞬时点云伪造逐点时间。Python 射线查询可能显著低于实时，目标机必须检查实时系数。
* IMU 200 Hz，发布含重力反作用的比力（静止时约 +9.81 m/s²），不将世界真值姿态送给 FAST-LIO。控制器更新率也设为 200 Hz。
* Isaac 的 `base_footprint` 位于轮胎接地平面（出生高度为 0）；机械臂初始关节姿态和零速度在时间线启动前设置。轮胎使用凸包碰撞体，机器人连续静止稳定 0.5 秒后才发布雷达和 IMU，避免启动瞬态干扰 FAST-LIO 的重力初始化。导航坐标链为 `map → odom → base_footprint → mid360_imu_link`，其中 `odom` 原点是初始底盘位姿，不是雷达的安装高度。
* FAST-LIO 提供局部激光惯性里程计；**已知三维地图定位另外通过多分辨率 ICP 完成**，需要配置初始位姿或 RViz「2D Pose Estimate」，不支持无先验全局搜索。建图没有回环优化，长程漂移仍可能存在。

| 输出 / TF | 来源 |
|---|---|
| `/x_bot/mid360/points` | Isaac 标准 PointCloud2：xyz、intensity、offset_time(ns)、line、tag |
| `/livox/lidar` / `/livox/imu` | 外部转换节点的 Livox CustomMsg / Isaac 物理 IMU |
| `/odom`、`/x_bot/odom`、`odom → base_footprint` | FAST-LIO 适配器，包含 IMU 到底盘的外参变换 |
| `map → odom` | 建图时为配置原点；定位时仅 ICP 配准节点发布 |
| `/map` / `/x_bot/scan` | FAST-LIO 三维点云的 5 cm 射线清空占据栅格 / 同一去畸变点云派生二维扫描 |
| `/localization/ready` / `/localization/status` | 可运行状态 / 配准质量与失败原因 |
| `/debug/ground_truth/odom` | 仅调试真值，无导航 TF，不参与定位 |

FAST-LIO 的原生 `camera_init/body` TF 被重映射到私有话题，避免重复 TF 发布者。机器人内部 TF 由 robot_state_publisher 发布。底盘命令保持 `/x_bot/cmd_vel`，通过健康门控变为 `/x_bot/cmd_vel_safe`；定位无效、数据过期或时钟回退时停止底盘，仿真端还有命令超时保护。速度限制为前进 2 m/s、后退 0.2 m/s、转向 1 rad/s。门控不接管已执行中的机械臂轨迹。

RViz 的 `FAST-LIO Point Cloud` 显示 `/fastlio/cloud_odom`，按高度着色并保留 10 秒点云；`Navigation 2` 面板负责将 `Nav2 Goal` 工具选取的位姿发送给导航动作服务器。手动建图导航可将 `start_explore_and_mapping.sh` 顶部的 `AUTO_EXPLORE` 改为 `false` 后直接运行，避免自动探索抢占手动目标。已有探索会话可向 `/explore/resume` 发布 `std_msgs/msg/Bool` 的 `data: false` 暂停探索，再设置 Goal；发布 `data: true` 恢复自动探索。

#### 目标机安装

**Isaac 底盘稳定性：** 四个车轮由一个关节控制节点统一写入，使用模拟 IMU 角速度 PI 闭环补偿四轮滑移。目标命令按仿真时间做联动斜坡，线加速度上限 1 m/s²、角加速度上限 1 rad/s²。零命令、失效和超时立即归零并清空积分，支持原地转向。调试时可对照 `/x_bot/cmd_vel`、`/x_bot/cmd_vel_safe`、`/odom` 和轮关节速度。

需要 Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.1 的 ROS 2 Bridge、URDF Importer、experimental physics sensors 和 ros2_control 扩展。无需本地安装 Livox 硬件 SDK。

```bash
source /opt/ros/jazzy/setup.bash
# 先安装 python3-vcstool、rosdep，以及项目原有依赖
bash scripts/setup_isaac_dependencies.sh
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ISAAC_SIM_PATH=/path/to/isaac-sim
# 或 export ISAAC_SIM_PYTHON=/path/to/isaac-sim/python.sh
```

[依赖清单](dependencies/isaac.repos)固定 FAST_LIO_ROS2 提交并递归初始化子模块；[消息子集](src/isaac_livox_interfaces/README.md)固定 Livox Driver2 消息定义，仅提供接口、没有硬件驱动。不要同时在工作空间放入另一份同名 `livox_ros_driver2` 包。Isaac 内嵌 Python 只加载标准 ROS 消息，自定义消息由系统 ROS Python 转换。

#### 先建图，再定位

```bash
# main 的 simple_room 建图，默认输出 maps/gazebo_simple_room；保存时不会覆盖已有地图
./start_explore_and_mapping.sh

# 另一终端，待走过所需区域后保存配套地图
source install/setup.bash
ros2 service call /localization/save_map std_srvs/srv/Trigger '{}'

# 导航与建图使用相同的 main simple_room 场景及地图目录
# 停止本次运行后，使用刚保存的三维地图定位
./start_navigation.sh
```

地图目录包含 `map.pcd`、`map.pgm`、`map.yaml`、`bundle.json`。栅格保留未观测区域为 unknown，利用传感器原点到障碍物的射线清空自由区，不把所有空白区域标成可通行。PCD 与二维地图共享配置的 `map` 原点。仅有二维 YAML/PGM 地图或 `isaac_simple_room.yaml` **不能单独用于这条定位链路**；仓库不提供伪造的「已验证」PCD。

脚本顶部的 `INITIAL_X/Y/YAW` 表示 **base_footprint 在 map 中的先验位姿**（yaw 为弧度），不是 IMU 位姿或真值订阅。探索默认 yaw=1.5708、导航默认 yaw=0，和各自示例的初始朝向一致。改变仿真出生点或换地图时修改脚本顶部的初始位姿；不确定时在已启动的 RViz 中重新设定。

| 入口 | 模式 / 场景 |
|---|---|
| `start_explore_and_mapping.sh` | 自动探索建图 / main simple_room |
| `start_pick_and_place_demo.sh` | 建图定位 + 抓取 / main 的 manipulation_test |
| `start_navigation.sh` | 已知地图定位 / simple_room（main 的房间布局） |
| `start_navigation_and_pick_demo.sh` | 已知地图定位 + 抓取 / simple_room（main 的房间布局） |
| `start_llm_agent.sh` | 已知地图定位 + LLM / simple_room（main 的房间布局） |

直接执行脚本即可，无需命令行参数。默认配置写在各入口顶部：`WORLD`、`MAP_BUNDLE`、`INITIAL_X/Y/YAW`、`HEADLESS`、`BUILD`，探索另有 `AUTO_EXPLORE`。探索地图为 `maps/gazebo_simple_room`，单独抓取为 `maps/manipulation_test`，导航类为 `maps/gazebo_simple_room`。与 main 一致，导航类要求已有地图，缺地图会报错退出；探索、导航和抓取入口默认构建，LLM 按需构建。已有地图时使用 ICP 定位。导航与任务入口等待 ready，180 秒内未就绪则不启动；ICP 连续失败 5 次后需要重新给初始位姿。定位丢失会停止底盘。暂停时停止运动；重置仿真时钟后必须重启 FAST-LIO、适配器和导航链路，不能沿用旧估计器状态。启动脚本先执行 `stop_robot_sim.sh`，清理上一轮进程并关闭带项目标记的服务终端。该脚本会清理所有 ROS 会话并删除日志。

Isaac 导航的直行目标速度为 **2 m/s（按仿真时间）**，Nav2 平滑器、定位安全节点和底盘驱动采用相同前进上限；线加速度为 1 m/s²、减速度为 2 m/s²。弯道、障碍附近和接近目标时仍会减速。实际观看速度还取决于仿真实时倍率，2 m/s 的配置不会自动让仿真达到实时运行。


本机 Office 实测：原 Python 逐射线版本实时倍率约 0.20，原生 C++ 批量雷达约 0.51（约 2.5 倍）。物理、轮控制和 IMU 仍为 200 Hz，雷达仍为每步 1000 条射线、10 Hz 点云，采样时间偏移为 0–95 ms。渲染目标频率为 30 Hz；GPU 物理与当前 ros2_control 的 CPU 张量接口不兼容，因此保留 CPU 物理。倍率会随视野、探索路径和其他进程负载变化，未达到实时运行。更快的无界面运行可将入口顶部 `HEADLESS=true`，相机与 ROS 感知仍运行，停用额外的观察视口。

探索入口默认使用 CUDA 多帧语义体素建图，输入 YOLOE 的 `/yoloe_multi_text_prompt/pointcloud_semantic`，类别 RGB 经跨帧多数投票确认，几何通过 hit/miss 更新。配置位于 `src/semantic_voxel_mapping/config/map.yaml`，入口顶部 `OCTOMAP_BACKEND=legacy` 可切回原后端。详见 [语义地图配置、保存及验证](src/semantic_voxel_mapping/README.md)。

当前探索入口 `DEPTH_SOURCE=isaac` 使用仿真器深度；改为 `lsm` 可启用 `src/LSM_depth_infer`（ROS 包 `stereo_matching`），YOLOE 使用 `/x_bot/camera_left/nn_depth`。语义地图随 YOLOE 使用所选深度来源；二维 `/map` 和 FAST-LIO 定位仍使用 MID360。脚本顶部暴露 `LSM_CONFIG_FILE`、`LSM_PARAMS_FILE` 和下游输入话题。详见 [LSM 配置接口](src/LSM_depth_infer/README.md)。

RViz 定位窗口默认显示橙色 `/plan` 全局导航路径和青色 `/received_global_plan` 控制器路径；收到导航目标并规划成功后出现。手动速度指令可发送到 `/cmd_vel` 或 `/x_bot/cmd_vel`，优先于自动导航。停止发送 0.5 秒后底盘停下，最后一次手动指令 2 秒后恢复自动导航；定位未就绪时两种输入均停止。自动导航经过接近障碍减速，再经定位安全节点输出到 `/x_bot/cmd_vel_safe`，避免自动零速度覆盖手动指令。

测量当前运行性能（系统 ROS Python）：
```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/measure_isaac_performance.py --seconds 30 --output /tmp/isaac_performance.json
```
输出仿真实时倍率、按仿真时间计的 IMU/雷达/相机消息频率、指令速度和真实位移速度。`ISAAC_PROFILE_SECONDS=20 ./start_explore_and_mapping.sh` 可生成 `/tmp/isaac_performance_profile.txt`；性能采样本身会增加开销，比较速度时应关闭采样。内部运行脚本支持 `--lidar-backend python` 对比参考算法。优化依据：[NVIDIA 性能手册](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html)。

底盘和顶板的 Isaac 视觉/碰撞轮廓统一为 96 段圆形，底盘半径 0.35 m，收拢机械臂后的碰撞包络约 0.369 m；Nav2 全局/局部安全半径均为 0.42 m，另加 0.01 m 足迹 padding，保留约 6 cm 的径向余量。RViz 的 `Navigation Safety Footprint` 以绿色显示真实规划足迹。`/x_bot/scan` 是已做高度筛选的二维虚拟激光，因此局部地图仅使用最新虚拟扫描的 ObstacleLayer 和 InflationLayer，分辨率为 4 cm；历史 `/map` 保留在全局规划地图，避免历史占据格重新覆盖局部清除结果。扫描未观测方向不当作空闲射线清除。

导航抓取/LLM 的既有任务包含硬编码地图坐标，换地图或原点后必须检查/修改目标点；定位 ready 不代表目标点在新地图中有效。

根目录仅保留 main 的启动入口，共用流程位于 `scripts/robot_services.sh`，Isaac 运行环境位于 `scripts/isaac_runtime.sh`。Office 使用环境补光和八盏面光源，照亮封闭室内。首次下载场景及 main 的抓取物体：
```bash
/home/lyz/isaacsim/python.sh scripts/download_isaac_environments.py
```

main 分支的原始 Gazebo 场景转换到本机 `~/isaacsim_assets/6.1/GazeboMain`，保留源模型和许可证。首次启动自动转换，也可提前执行：
```bash
/home/lyz/isaacsim/python.sh scripts/migrate_gazebo_scenes.py
```
支持原始 `simple_room`（兼容旧名称 `legacy_room` 或 `gazebo_simple_room`）、`manipulation_test`、`small_house`、`ware_house`、`obstacle_avoidance_test`、`empty`。`WORLD=simple_room` 表示 main 的房间；NVIDIA 官方 Simple Room 的名称为 `isaac_simple_room`。迁移保留原始网格、UV/贴图、DAE 单位和节点矩阵、SDF 层级位姿和缩放；视觉和碰撞模型分别导入。PhysX 动态网格使用凸分解，灯光按 USD 强度调整，接触和光照效果不能保证与 Gazebo 完全一致。

抓取入口默认使用 main 的原始桌面测试场景。原始房间的导航入口使用 `maps/gazebo_simple_room`；需要重新建图。探索默认已设置 `WORLD=simple_room`、`MAP_BUNDLE="$ROOT_DIR/maps/gazebo_simple_room"`，直接启动并保存地图后再导航。Office 仍可通过修改脚本顶部的场景和地图目录使用。

资源与位姿验证、参考渲染（输出到 `docs/validation/gazebo_scene_migration`）：
```bash
/home/lyz/isaacsim/python.sh scripts/validate_gazebo_scene_assets.py
```

动态网格的 PhysX 凸分解开启 shrink-wrap 并降低误差，避免桌面碰撞体膨胀、物体悬空。以下测试运行真实物理 4 秒，再比较物体底部和可见桌面的间距：
```bash
/home/lyz/isaacsim/python.sh scripts/validate_gazebo_scene_contacts.py
/home/lyz/isaacsim/python.sh scripts/validate_gazebo_scene_contacts.py --world simple_room
```
资产默认存于 `~/isaacsim_assets/6.1`，可用 `ISAAC_ASSETS_PATH` 修改。

低层调试：
```bash
bash scripts/isaac_runtime.sh
ros2 launch x_bot isaac_controllers.launch.py
ros2 launch x_bot_localization localization.launch.py mode:=mapping bundle:=/absolute/path/new_map
```
这三个命令分别在独立终端执行。`--publish-odom-tf` 已移除；不再用仿真真值补足导航 TF。更改安装位置时，仿真 `--mid360-xyz/--mid360-rpy` 与控制器 launch 的 `mid360_xyz/mid360_rpy` 必须一致；FAST-LIO 的共点 IMU 外参仍为单位变换。

#### 验证

本机可运行：
```bash
python3 -m unittest discover -s tests -v
```
覆盖点云布局/端序、真实采样偏移、回退清缓存、外参/四元数、栅格清空与 unknown、健康状态、Xacro 双后端展开、配置与语法。测试需要 NumPy、PyYAML；Xacro 展开测试在未安装 xacro 时跳过。PCL 合成配准测试位于 `src/x_bot_localization/test`，需目标机编译后运行：
```bash
colcon test --packages-select x_bot_localization
colcon test-result --verbose
ros2 topic hz /livox/imu
ros2 topic hz /livox/lidar
ros2 topic echo /localization/ready
ros2 topic echo /localization/status
ros2 run tf2_ros tf2_echo map base_footprint
ros2 control list_controllers
```

目标机验收还需：静止重力方向、IMU/点云共同时间基准、实际 200/10 Hz（按仿真时间）、转弯去畸变、雷达/机械臂遮挡、唯一 TF、地图保存与重新加载、错误初值拒绝、定位丢失/停流停车，以及 Nav2/抓取回归。相机与机械臂话题沿用原接口。

实现参考：[FAST-LIO ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)、[Livox 消息定义](https://github.com/Livox-SDK/livox_ros_driver2/tree/21445540f0d100dc86a7e6df312dd70bbdb4afdf/msg)、[NVIDIA PhysX scene queries](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.1/extensions/runtime/source/omni.physx/docs/dev_guide/scene_queries.html)。

### 3. 生成 TensorRT Engine 文件

TensorRT engine 文件与 GPU 型号和 TensorRT 版本绑定，不可跨设备使用，需在本机重新生成：

```bash
# YOLOE 目标检测模型
trtexec --onnx=src/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.onnx \
  --saveEngine=src/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.engine

# GraspNet 抓取模型（依赖 FPS 插件，需先完成工作空间编译部分的插件构建）
trtexec --onnx=src/graspnet_infer/graspnet.onnx \
  --saveEngine=src/graspnet_infer/graspnet.trt \
  --staticPlugins=src/graspnet_infer/tensorrt_plugins/build/libfps_plugin.so
```

### 4. 运行演示案例

项目根目录提供了 5 个“一键启动”脚本，全部无参数直接使用 Isaac Sim。探索默认开启自动探索；导航、移动抓取和 LLM 默认读取 `maps/gazebo_simple_room`，缺少配套地图时提示先建图并退出。

#### 模式 1: 自主探索建图 (Explore & Mapping)
无参数启动 Isaac Sim、FAST-LIO、Nav2 和 `explore_lite`。自动探索等待定位就绪和 Nav2 导航服务器激活，随后自行选择前沿目标并持续建图。Isaac 的 Nav2 使用 Regulated Pure Pursuit，先对齐路径方向再前进；进度检测同时考虑平移和转向。底盘用模拟 IMU 角速度闭环补偿四轮滑移，保留速度和加速度限制。每次启动先执行 `stop_robot_sim.sh` 并关闭上次服务窗口。
```bash
./start_explore_and_mapping.sh
```

#### 模式 2: 静态导航 (Static Navigation)
使用 Nav2、FAST-LIO 与已保存地图的 ICP 定位进行导航与巡逻：
```bash
./start_navigation.sh
```

#### 模式 3: 自主移动抓取全流程 (Mobile Pick and Place)
在 main 的原始 `manipulation_test` 场景中唤醒机器人，同时启动完整的视觉感知流水线 (YOLOE, GraspNet)，并触发一个 MoveIt! 语义物体的循环搬运操作演示 (例如：循环寻找、抓取、移动与放置 coke、book、cup)：
```bash
./start_pick_and_place_demo.sh
```

#### 模式 4: 导航 + 抓取 (Navigation and Pick)

在 `simple_room`（沿用 main 的房间布局和任务坐标）中启动 Nav2、MoveIt 2 与完整视觉抓取栈，自动执行“导航到厨房 → 检测并抓取 → 返回”的移动操作流程：

```bash
./start_navigation_and_pick_demo.sh
```

#### 模式 5: 大语言模型具身智能闭环 (LLM Embodied Agent + Web UI)
> ⚠️ **运行前准备**：本模式依赖阿里云百炼平台提供的 Qwen 大语言模型服务。
> 1. 请先前往 [阿里云百炼平台](https://www.aliyun.com/product/bailian) 注册/登录，并在“API-KEY管理”页面创建获取您的 **API Key**。
> 2. 在运行启动脚本之前，需要在**当前终端**中导出该 API Key 环境变量：
>    ```bash
>    export DASHSCOPE_API_KEY="您的_DASHSCOPE_API_KEY"
>    ```

此模式将启动全套底层控制与感知节点，并挂载 Qwen3 LLM Agent 服务器，最终自动打开 Web UI 浏览器面板。你可以直接在浏览器右侧用自然语言输入指令（如：“帮我去厨房拿瓶可乐”、“到书房拿本书”），系统会自动识别场景并规划执行：
```bash
./start_llm_agent.sh
```

> **提示:** 如需快速彻底清理 / 杀死所有的仿真进程与 ROS 2 守护节点，可以直接运行根目录的助手脚本：  
> `./stop_robot_sim.sh`

## ⚙️ 关键话题 (Topics) 与服务 (Services)

* **/arm_command/pose** (Topic, `geometry_msgs/msg/PoseStamped`): 给 Franka 发送目标末端位姿。
* **/robot_actions/go_home** (Service, `std_srvs/srv/Trigger`): 驱使机械臂回到默认收纳/预备状态。
* **/robot_actions/scan** (Service, `std_srvs/srv/Trigger`): 控制机械臂转到视角最佳扫描观测位姿。
* **/yoloe_multi_text_prompt/set_cloud_filter** (Topic, `std_msgs/msg/Int32`): 传入语义检测的 ID 掩码，将不需要的背景点云过滤掉。

## 🤝 自定义与贡献建议

* **仿真世界构建:** 场景结构文件位于 `src/x_bot/worlds/`；Isaac 程序化 USD 场景位于 `src/x_bot/isaac_sim/scene_builder.py`。
* **导航调优:** Isaac 的 Nav2 和 FAST-LIO 配置位于 `src/x_bot_localization/config/`。
## 👏 致谢 (Acknowledgements)

本项目的开发离不开以下开源社区和仓库的贡献，在此深表感谢：

* **[YOLOE](https://github.com/THU-MIG/yoloe):** 强大的 2D 目标检测框架。
* **[GraspNet-Baseline](https://github.com/graspnet/graspnet-baseline):** 6-DoF 抓取位姿估计的基石。
* **[m-explore-ros2](https://github.com/robo-friends/m-explore-ros2):** ROS 2 的自主探索组件。
* **[franka_ros2](https://github.com/frankaemika/franka_ros2) & [franka_description](https://github.com/frankaemika/franka_description):** Franka Emika 提供的官方 ROS 2 支持。
* **[FAST-LIO](https://github.com/hku-mars/FAST_LIO):** 激光惯性里程计与建图。
* **[bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot):** 差速移动底盘仿真参考。
