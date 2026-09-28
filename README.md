# Embodied-RobotSim: 基于大模型的具身智能移动抓取仿真系统

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen.svg)](https://docs.ros.org/en/jazzy/index.html)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-6.1-76B900.svg)](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/)
[![English](https://img.shields.io/badge/🌍_Language-English-blue.svg)](README_en.md)

Embodied-RobotSim 是一个基于 ROS 2 (Jazzy) 构建的综合仿真工作空间。它提供了一套完整的差速驱动移动平台方案，搭载 **Franka FR3 骨干机械臂**、2D 激光雷达 (LiDAR) 以及立体 RGB-D 深度传感器。本系统经过深度优化，旨在**低硬件门槛**下实现高性能仿真，无缝集成建图、导航、先进视觉感知、移动抓取操作以及 **Qwen3 大语言模型驱动的具身智能闭环**，支持通过自然语言指令和 Web UI 控制台完成复杂的机器人环境交互任务。

## 🎬 演示 (Demos)

### 1. 具身大模型闭环 (LLM Agent)
![LLMAgent](assets/Embodied_LLM.gif)

*基于 Qwen3 与视觉模型的多模态指令闭环*

### 2. 移动抓取 (Pick & Place)
![Pick&Place](assets/Pick&Place.gif)

*基于 YOLOE 与 GraspNet 的自主抓取*

### 3. 仿真环境 (Gazebo / Isaac Sim)
![GazeboSim](assets/GazeboSim.gif)

*室内场景仿真：Gazebo Harmonic 与 Isaac Sim 6.1 共用 Nav2、MoveIt 2 和感知栈*

## 🌟 主要特性

* **双仿真后端:** 同一套 Xacro、ROS 2 话题和控制器接口可运行于 Gazebo Harmonic 或 Isaac Sim 6.1。
* **移动抓取操作 (Mobile Manipulation):** 为 Franka FR3 机械臂提供 MoveIt 2 集成，同时支持稳定的四轮差速移动底盘控制。
* **自主探索与建图:** 采用 Cartographer (原生支持 2D LiDAR 和 IMU 融合) 实现高精度 SLAM，集成 `m-explore-ros2` 包进行基于前沿的 (Frontier-based) 未知环境自主探索。
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
| `x_bot` | 核心机器人包：包含 URDF、Gazebo/Isaac Sim 后端、场景、启动脚本、导航配置以及 MoveIt 机械臂控制节点 (`robot_actions`)。 |
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
| **Gazebo** | [Harmonic (Gz Sim 8)](https://gazebosim.org/docs/harmonic/install) |
| **Isaac Sim（可选后端）** | 6.1，启用 ROS 2 Bridge、URDF Importer、RTX Sensor 与 ros2_control 扩展 |
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

> **重要**：ROS 2 Jazzy 是公共依赖。仿真器可在 Gazebo Harmonic 与 Isaac Sim 6.1 中二选一；运行完整视觉抓取功能时仍需 CUDA、TensorRT 和对应模型。

### 1. 编译工作空间

```bash
cd /path/to/Embodied-RobotSim
# 1. 编译 TensorRT 插件 (GraspNet 依赖)
bash src/graspnet_infer/tensorrt_plugins/build.sh

# 2. 安装依赖扩展包 (rosdep 及额外系统包)
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-moveit-ros-perception

# 3. 构建所有功能包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Isaac Sim 6.1 后端

Isaac 后端与 Gazebo 并存，不改变原有无参数启动方式。运行时会展开并导入同一套 `x_bot.xacro`，无需预先维护一份独立机器人 USD。

> **当前状态：** Isaac Sim 代码以 **6.1 + ROS 2 Jazzy** API 为目标完成；开发机未安装 Isaac Sim/ROS 2，因此尚未进行目标机运行验证。首次部署请按下方清单逐项检查。

#### 后端能力对照

| 能力 | Gazebo Harmonic | Isaac Sim 6.1 |
|---|---|---|
| 机器人描述 | Xacro → SDF | Xacro → URDF → 运行时 USD |
| 移动底盘 | Gz 四轮差速插件 | 两组 OmniGraph 差速控制器同步驱动四轮 |
| FR3 控制 | `gz_ros2_control` | Isaac 内置 `ROS2ControlManager` |
| 2D LiDAR | GPU LiDAR | RTX LiDAR |
| RGB-D | Gz RGB-D Sensor | Render Product + ROS 2 Camera Helper |
| 场景 | 原有 SDF 世界 | 程序化轻量 USD 场景 |
| ROS 接口 | `ros_gz_bridge` | Isaac ROS 2 Bridge |

Isaac 后端当前支持：

* `simple_room`：探索建图、静态导航、导航抓取和 LLM Agent；
* `manipulation_test`：桌面循环抓取；
* `coke`、`cup`、`book` 等操作物体由 USD 几何体直接生成，不依赖大型 Gazebo 模型目录。

#### 安装与环境变量

目标机需安装 Ubuntu 24.04、ROS 2 Jazzy 和 Isaac Sim 6.1，并确保 Isaac Sim 包含 ROS 2 Bridge、URDF Importer、RTX Sensor 和 ros2_control 扩展。

```bash
cd /path/to/Embodied-RobotSim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 标准独立安装：目录内应包含 python.sh
export ISAAC_SIM_PATH=/path/to/isaac-sim

# 或直接指定 Isaac Sim Python 启动器
# export ISAAC_SIM_PYTHON=/path/to/isaac-sim/python.sh
```

默认使用 `rmw_fastrtps_cpp`。如目标环境使用其他 RMW，可在启动前覆盖 `RMW_IMPLEMENTATION`。

#### 启动方式

完整演示只需在原脚本后增加 `--sim isaac`：

| 演示 | 命令 | Isaac 场景 | `odom → base_footprint` 来源 |
|---|---|---|---|
| 探索建图 | `./start_explore_and_mapping.sh --sim isaac` | `simple_room` | Cartographer |
| 静态导航 | `./start_navigation.sh --sim isaac` | `simple_room` | Isaac Sim |
| 桌面抓取 | `./start_pick_and_place_demo.sh --sim isaac` | `manipulation_test` | Cartographer |
| 导航抓取 | `./start_navigation_and_pick_demo.sh --sim isaac` | `simple_room` | Isaac Sim |
| LLM Agent | `./start_llm_agent.sh --sim isaac` | `simple_room` | Isaac Sim |

也可以用环境变量选择后端：

```bash
SIM_BACKEND=isaac ./start_navigation.sh
```

只启动仿真与控制器，便于接口调试：

```bash
# 终端 1
./start_isaac_sim.sh --world simple_room --publish-odom-tf

# 终端 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch x_bot isaac_controllers.launch.py
```

`start_isaac_sim.sh` 支持以下参数：

| 参数 | 说明 |
|---|---|
| `--world simple_room\|manipulation_test` | 选择程序化场景 |
| `--x`、`--y`、`--yaw` | 设置机器人初始位姿，偏航角单位为弧度 |
| `--publish-odom-tf` | 发布 `odom → base_footprint`；使用 Cartographer 时不要开启 |
| `--headless` | 无窗口运行 Isaac Sim |

#### ROS 2 接口

| 类型 | 名称 |
|---|---|
| 底盘命令 | `/x_bot/cmd_vel` |
| 仿真时钟 | `/clock` |
| 里程计 / IMU / 雷达 | `/x_bot/odom`、`/x_bot/imu`、`/x_bot/scan` |
| 左相机 | `/x_bot/camera_left/image_raw`、`/x_bot/camera_left/depth/image_raw`、`/x_bot/camera_left/camera_info`、`/x_bot/camera_left/points` |
| 右相机 | `/x_bot/camera_right/image_raw`、`/x_bot/camera_right/camera_info` |
| 关节状态 | `/joint_states`、`/x_bot/joint_states` |
| 机械臂控制 | `/fr3_arm_controller/follow_joint_trajectory` |
| 夹爪控制 | `/fr3_gripper_controller/gripper_cmd` |

对应控制器为 `joint_state_broadcaster`、`fr3_arm_controller` 和 `fr3_gripper_controller`，配置位于 `src/x_bot/config/isaac_controllers.yaml`。

#### 关键文件

```text
start_isaac_sim.sh                     # Isaac Sim 低层入口
start_isaac_demo.sh                    # 五种完整演示的统一编排
src/x_bot/isaac_sim/run_sim.py         # SimulationApp 生命周期
src/x_bot/isaac_sim/robot_importer.py  # Xacro/URDF 导入和关节驱动
src/x_bot/isaac_sim/scene_builder.py   # 程序化 USD 场景
src/x_bot/isaac_sim/ros_bridge.py      # OmniGraph、相机和 RTX LiDAR
```

#### 首次目标机验证

```bash
ros2 topic echo /clock --once
ros2 topic hz /x_bot/scan
ros2 topic hz /x_bot/camera_left/image_raw
ros2 topic echo /x_bot/odom --once
ros2 control list_controllers
```

期望三个 controller 均为 `active`。若 Isaac Sim 已启动但 ROS 话题不可见，优先检查是否在启动 Isaac Sim 前 source 了 Jazzy 环境、`ROS_DISTRO=jazzy`、RMW 实现是否一致，以及 `isaacsim.ros2.bridge` 扩展是否成功加载。

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

项目根目录提供了 5 个“一键启动”脚本。以下命令默认使用 Gazebo；增加 `--sim isaac` 即可切换到 Isaac Sim。

#### 模式 1: 自主探索建图 (Explore & Mapping)
使用 `explore_lite` 在完全未知的 Gazebo 房间中进行自主探索，同步运行 Cartographer 生成高精度地图与 YOLOE / OctoMap 语义网格。
```bash
./start_explore_and_mapping.sh
```

#### 模式 2: 静态导航 (Static Navigation)
由于场景已被扫描完毕，可以使用 Nav2 和预存地图依靠 Cartographer amcl 进行无缝导航与巡逻：
```bash
./start_navigation.sh
```

#### 模式 3: 自主移动抓取全流程 (Mobile Pick and Place)
在 `manipulation_test` 场景中唤醒机器人，同时启动完整的视觉感知流水线 (YOLOE, GraspNet)，并触发一个 MoveIt! 语义物体的循环搬运操作演示 (例如：循环寻找、抓取、移动与放置 coke、book、cup)：
```bash
./start_pick_and_place_demo.sh
```

#### 模式 4: 导航 + 抓取 (Navigation and Pick)

在 `simple_room` 中启动 Nav2、MoveIt 2 与完整视觉抓取栈，自动执行“导航到厨房 → 检测并抓取 → 返回”的移动操作流程：

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

* **仿真世界构建:** Gazebo SDF 位于 `src/x_bot/worlds/`；Isaac 程序化 USD 场景位于 `src/x_bot/isaac_sim/scene_builder.py`。
* **导航调优:** Nav2 和 Cartographer 的核心配置文件存放在 `src/x_bot/config/`，可根据使用环境调整。
## 👏 致谢 (Acknowledgements)

本项目的开发离不开以下开源社区和仓库的贡献，在此深表感谢：

* **[YOLOE](https://github.com/THU-MIG/yoloe):** 强大的 2D 目标检测框架。
* **[GraspNet-Baseline](https://github.com/graspnet/graspnet-baseline):** 6-DoF 抓取位姿估计的基石。
* **[m-explore-ros2](https://github.com/robo-friends/m-explore-ros2):** ROS 2 的自主探索组件。
* **[franka_ros2](https://github.com/frankaemika/franka_ros2) & [franka_description](https://github.com/frankaemika/franka_description):** Franka Emika 提供的官方 ROS 2 支持。
* **[Cartographer](https://github.com/cartographer-project/cartographer_ros):** 高效的 2D/3D SLAM 解决方案。
* **[bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot):** 差速移动底盘仿真参考。
