# RobotSim: 自主探索与移动抓取仿真系统

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen.svg)](https://docs.ros.org/en/jazzy/index.html)
[![English](https://img.shields.io/badge/🌍_Language-English-blue.svg)](README_en.md)

RobotSim 是一个基于 ROS 2 (Jazzy) 构建的综合仿真工作空间。它提供了一套完整的差速驱动移动平台方案，搭载 **Franka FR3 骨干机械臂**、2D 激光雷达 (LiDAR) 以及立体 RGB-D 深度传感器。本系统在 Gazebo 仿真环境中运行，无缝集成建图、导航、先进 2D/3D 视觉感知与移动操作 (Mobile Manipulation) 等最前沿的算法方案。

## 🎬 演示 (Demos)

| 仿真环境 (Gazebo Sim) | 移动抓取 (Pick & Place) |
| :---: | :---: |
| ![GazeboSim](assets/GazeboSim.gif) | ![Pick&Place](assets/Pick&Place.gif) |
| *室内场景仿真* | *基于 GraspNet 的自主抓取* |

## 🌟 主要特性

* **移动抓取操作 (Mobile Manipulation):** 为 Franka FR3 机械臂提供 MoveIt 2 集成，同时支持稳定的差速移动底盘控制。
* **自主探索与建图:** 采用 Cartographer (原生支持 2D LiDAR 和 IMU 融合) 实现高精度 SLAM，集成 `m-explore-ros2` 包进行基于前沿的 (Frontier-based) 未知环境自主探索。
* **先进感知系统 (视觉):** 
  * **YOLOE 感知推理:** 支持输入文本提示词 (Text Prompt) 的实时多目标检测 (`yoloe_infer`)。
* **语义占据栅格与任意物体抓取:** 
  * 通过 OctoMap 生成三维语义占据地图。
  * **GraspNet 集成:** 直接由点云生成高成功率的 6-DoF 抓取位姿。系统能够对环境中的未知或任意姿态物体进行抓取，并在点云和 OctoMap 级别支持复杂的碰撞检测与避障逻辑。

## 📦 架构概览

### ROS 2 功能包列表

| 功能包 | 用途 |
|---------|---------|
| `x_bot` | 核心机器人包：包含 URDF、Gazebo 世界文件、启动脚本 (Launch)、导航配置以及 MoveIt 机械臂控制节点 (`robot_actions`)。 |
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
| **CUDA** | [13.1](https://developer.nvidia.com/cuda-toolkit) |
| **TensorRT** | [10.14.1.48](https://developer.nvidia.com/tensorrt) |
| **Python** | 3.12+ |

## 📦 权重文件下载 (Models Download)

由于模型文件较大，请从以下链接下载预训练权重并放置到指定目录：

*   **下载链接**：[Google Drive 文件夹](https://drive.google.com/drive/folders/1gPPyvKqiYd7cg2vUyqucV1CjLTf6J0y2?usp=drive_link)

| 文件名 | 存放路径 (相对于项目根目录) |
| :--- | :--- |
| `yoloe-v8l-text-prompt-multi_nc10_fp16.engine` | `src/yoloe_infer/models/` |
| `yoloe-v8l-text-prompt-multi_nc10_fp16.onnx` | `src/yoloe_infer/models/` |
| `graspnet.trt` | `src/graspnet_infer/` |
| `graspnet.onnx` | `src/graspnet_infer/` |

## 🚀 快速启动指南

> **重要**：在开始编译之前，请确保你已经按照上面的表格完成了 **ROS 2**、**Gazebo**、**CUDA** 和 **TensorRT** 的安装。

### 1. 编译工作空间

```bash
cd ~/robotSim
# 1. 编译 TensorRT 插件 (GraspNet 依赖)
bash src/graspnet_infer/tensorrt_plugins/build.sh

# 2. 安装依赖扩展包 (rosdep)
rosdep install --from-paths src --ignore-src -r -y

# 3. 构建所有功能包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. 运行演示案例

在项目根目录下提供了 3 个非常方便的 "一键启动" 脚本，涵盖系统的几大核心场景。

#### 模式 1: 自主探索建图 (Explore & Mapping)
使用 `explore_lite` 在完全未知的 Gazebo 房间中进行自主探索，同步运行 Cartographer 生成高精度地图与 YOLOE / OctoMap 语义网格。
```bash
./start_explore_mapping.sh
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

> **提示:** 如需快速彻底清理 / 杀死所有的仿真进程与 ROS 2 守护节点，可以直接运行根目录的助手脚本：  
> `./stop_robot_sim.sh`

## ⚙️ 关键话题 (Topics) 与服务 (Services)

* **/arm_command/pose** (Topic, `geometry_msgs/msg/PoseStamped`): 给 Franka 发送目标末端位姿。
* **/robot_actions/go_home** (Service, `std_srvs/srv/Trigger`): 驱使机械臂回到默认收纳/预备状态。
* **/robot_actions/scan** (Service, `std_srvs/srv/Trigger`): 控制机械臂转到视角最佳扫描观测位姿。
* **/yoloe_multi_text_prompt/set_cloud_filter** (Topic, `std_msgs/msg/Int32`): 传入语义检测的 ID 掩码，将不需要的背景点云过滤掉。

## 🤝 自定义与贡献建议

* **仿真世界构建:** 你可以直接在 `src/x_bot/worlds/` 下修改或创建新的 Gazebo 场景模型与物料配置。
* **导航调优:** Nav2 和 Cartographer 的核心配置文件存放在 `src/x_bot/config/`，可根据使用环境调整。
## 👏 致谢 (Acknowledgements)

本项目的开发离不开以下开源社区和仓库的贡献，在此深表感谢：

* **[YOLOE](https://github.com/THU-MIG/yoloe):** 强大的 2D 目标检测框架。
* **[GraspNet-Baseline](https://github.com/graspnet/graspnet-baseline):** 6-DoF 抓取位姿估计的基石。
* **[m-explore-ros2](https://github.com/robo-friends/m-explore-ros2):** ROS 2 的自主探索组件。
* **[franka_ros2](https://github.com/frankaemika/franka_ros2) & [franka_description](https://github.com/frankaemika/franka_description):** Franka Emika 提供的官方 ROS 2 支持。
* **[Cartographer](https://github.com/cartographer-project/cartographer_ros):** 高效的 2D/3D SLAM 解决方案。
* **[bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot):** 差速移动底盘仿真参考。
