# Embodied-RobotSim: 基于大模型的具身智能移动抓取仿真系统

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen.svg)](https://docs.ros.org/en/jazzy/index.html)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-6.1-76B900.svg)](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/)
[![English](https://img.shields.io/badge/🌍_Language-English-blue.svg)](README_en.md)

Embodied-RobotSim 是一个基于 ROS 2 (Jazzy) 构建的综合仿真工作空间。差速移动平台搭载 **Franka FR3 机械臂**、激光雷达及 RGB-D 深度传感器：Gazebo 使用 2D LiDAR，Isaac Sim 使用顶部 MID-360 近似仿真和 FAST-LIO 定位链路。项目集成建图、导航、视觉感知、移动抓取以及 **Qwen3 大语言模型驱动的具身智能闭环**，支持自然语言指令和 Web UI 控制。

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
* **自主探索与建图:** Gazebo 使用 Cartographer；Isaac 使用 MID-360 + FAST-LIO 建图、已知地图 ICP 定位，配合 `m-explore-ros2` 进行前沿探索。
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
| **Isaac Sim（可选后端）** | 6.1，启用 ROS 2 Bridge、URDF Importer、experimental physics sensors 与 ros2_control 扩展 |
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

### 2. Isaac Sim 6.1：顶部 MID-360 + FAST-LIO

> **验证范围：** 当前完成代码与离线测试；开发机没有 ROS 2 Jazzy、Isaac Sim 或 PCL，尚未完成 ROS/C++ 编译和目标机联调。不要把下面的启动入口理解为已实测通过的运行结果。

只替换 **Isaac 后端**：Gazebo 保留原来的二维雷达、Cartographer/AMCL 和无参数启动方式。Isaac 仍通过共享 Xacro 导入机器人，保留 FR3、RGB-D、Nav2、MoveIt 和五种演示入口。

#### 传感器与定位设计

* MID-360 外形为 65 × 65 × 60 mm、质量 0.265 kg，固定在底盘顶板前部，默认相对 `roof_link` 为 `xyz=[0.25, 0, 0.0875]`、`rpy=[0,0,0]`，避开中央 FR3 底座。模拟 IMU 与雷达光心共点，FAST-LIO 外参为单位变换，不使用真实设备的内部标定值。
* 使用 **PhysX 碰撞几何射线**近似非重复扫描：水平 360°、垂直 -7°～52°、0.1～40 m、200,000 条射线/仿真秒、10 Hz 点云。不是官方 Livox 光学模型，也不是 RTX 材质回波模型；强度为常量，遮挡由碰撞几何决定，自身命中丢弃。40 m 为当前仿真截断距离。
* 每个 200 Hz 物理步真实采集 1,000 条射线，每 100 ms 组帧。点的 `offset_time` 是实际物理采样时刻，**5 ms 分辨率**，不是给瞬时点云伪造逐点时间。Python 射线查询可能显著低于实时，目标机必须检查实时系数。
* IMU 200 Hz，发布含重力反作用的比力（静止时约 +9.81 m/s²），不将世界真值姿态送给 FAST-LIO。控制器更新率也设为 200 Hz。
* FAST-LIO 提供局部激光惯性里程计；**已知三维地图定位另外通过多分辨率 ICP 完成**，需要配置初始位姿或 RViz「2D Pose Estimate」，不支持无先验全局搜索。建图没有回环优化，长程漂移仍可能存在。

| 输出 / TF | 来源 |
|---|---|
| `/x_bot/mid360/points` | Isaac 标准 PointCloud2：xyz、intensity、offset_time(ns)、line、tag |
| `/livox/lidar` / `/livox/imu` | 外部转换节点的 Livox CustomMsg / Isaac 物理 IMU |
| `/odom`、`/x_bot/odom`、`odom → base_footprint` | FAST-LIO 适配器，包含 IMU 到底盘的外参变换 |
| `map → odom` | 建图时为配置原点；定位时仅 ICP 配准节点发布 |
| `/map` / `/x_bot/scan` | 5 cm 射线清空占据栅格 / 去畸变三维点云派生二维扫描 |
| `/localization/ready` / `/localization/status` | 可运行状态 / 配准质量与失败原因 |
| `/debug/ground_truth/odom` | 仅调试真值，无导航 TF，不参与定位 |

FAST-LIO 的原生 `camera_init/body` TF 被重映射到私有话题，避免重复 TF 发布者。机器人内部 TF 由 robot_state_publisher 发布。底盘命令保持 `/x_bot/cmd_vel`，通过健康门控变为 `/x_bot/cmd_vel_safe`；定位无效、数据过期或时钟回退时停止底盘，仿真端还有命令超时保护。速度限制为前进 0.5 m/s、后退 0.2 m/s、转向 1 rad/s。门控不接管已执行中的机械臂轨迹。

#### 目标机安装

**Isaac 底盘稳定性：** 四个车轮使用固定的左右映射，由一个关节控制节点统一写入；命令按仿真时间做联动斜坡，线加速度上限 0.5 m/s²、角加速度上限 1 rad/s²，饱和时保持目标曲率。零命令、失效和超时停车绕过斜坡立即归零，正常原地调头仍可使用。这是针对突变和多写入路径的预防性优化，尚未在目标机复现/确认转圈根因。若仍转圈，请同时记录 `/x_bot/cmd_vel`、`/x_bot/cmd_vel_safe`、`/odom` 和轮关节速度，区分导航主动转向与底盘执行偏差；Gazebo 控制配置不受此改动影响。

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
# 建图；目录必须尚不存在，保存时不会覆盖已有地图
./start_explore_and_mapping.sh --sim isaac --bundle maps/room_a

# 另一终端，待走过所需区域后保存配套地图
source install/setup.bash
ros2 service call /localization/save_map std_srvs/srv/Trigger '{}'

# 停止本次运行后，使用刚保存的三维地图定位
./start_navigation.sh --sim isaac --bundle maps/room_a \
  --initial-x 0.0 --initial-y 0.0 --initial-yaw 0.0
```

地图目录包含 `map.pcd`、`map.pgm`、`map.yaml`、`bundle.json`。栅格保留未观测区域为 unknown，利用传感器原点到障碍物的射线清空自由区，不把所有空白区域标成可通行。PCD 与二维地图共享配置的 `map` 原点。旧的 Gazebo 地图或 `isaac_simple_room.yaml` **不能单独用于这条定位链路**；仓库不提供伪造的「已验证」PCD。

`--initial-x/y/yaw` 表示 **base_footprint 在 map 中的先验位姿**（yaw 为弧度），不是 IMU 位姿或真值订阅。探索默认 yaw=1.5708、导航默认 yaw=0，和各自示例的初始朝向一致。改变仿真出生点或换地图时必须相应提供先验；不确定时在已启动的 RViz 中重新设定。

| 入口 | 模式 / 场景 |
|---|---|
| `start_explore_and_mapping.sh --sim isaac --bundle DIR` | 建图 / simple_room |
| `start_pick_and_place_demo.sh --sim isaac --bundle DIR` | 建图定位 / manipulation_test |
| `start_navigation.sh --sim isaac --bundle DIR` | 已知地图定位 / simple_room |
| `start_navigation_and_pick_demo.sh --sim isaac --bundle DIR` | 已知地图定位 + 抓取 / simple_room |
| `start_llm_agent.sh --sim isaac --bundle DIR` | 已知地图定位 + LLM / simple_room |

支持 `--headless`、`--build` 和 `SIM_BACKEND=isaac`。导航与任务入口等待 ready，180 秒内未就绪则不启动；ICP 连续失败 5 次后需要重新给初始位姿。定位丢失会停止底盘。暂停时停止运动；重置仿真时钟后必须重启 FAST-LIO、适配器和导航链路，不能沿用旧估计器状态。启动脚本不再自动杀掉已有仿真，请先结束上一轮会话。原 `stop_robot_sim.sh` 是全局清理脚本，可能影响其他 ROS 会话并删除日志，谨慎使用。

导航抓取/LLM 的既有任务包含硬编码地图坐标，换地图或原点后必须检查/修改目标点；定位 ready 不代表目标点在新地图中有效。

低层调试：
```bash
./start_isaac_sim.sh --world simple_room --headless
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
