# Embodied-RobotSim

[English](README_en.md) · [安装与构建](INSTALL.md)

基于 **ROS 2 Jazzy、Isaac Sim 6.1 / Gazebo Harmonic** 的移动抓取仿真项目，集成 FAST-LIO、Nav2、YOLOE、语义 OctoMap、GraspNet／MoveIt 和 ACT，支持自然语言取物任务。

| 抓取 | 建图导航 | 具身任务 |
|---|---|---|
| ![Isaac Sim 抓取](assets/IssacSim.gif) | ![Gazebo 建图导航](assets/GazeboSim.gif) | ![具身任务](assets/Embodied_LLM.gif) |

## 快速启动

先完成[依赖安装、模型准备和构建](INSTALL.md)，在项目根目录执行：

```bash
# 具身任务：默认 Gazebo、本地 Qwen、传统抓取链路
./start_embodied.sh

# ACT 抓取：rgb / rgbd / rgbxyz
./start_embodied.sh --pipeline act --act-model rgbd

# Gazebo 自动探索建图；完成后保存地图
./start_mapping_and_explore.sh --sim gazebo

# 使用已有地图导航，在 RViz 设置 Nav2 Goal
./start_mapping_and_explore.sh --sim gazebo --mode navigation --bundle maps/gazebo/semantic_simple_room

# 独立抓取放置：默认 Isaac Sim、manipulation_test 场景
./start_pick_and_place.sh

# 停止项目服务
./stop_robot_sim.sh
```

具身界面：**http://localhost:8888**，例如输入“到厨房拿可乐”。使用云端模型：

```bash
export DASHSCOPE_API_KEY='你的 API Key'
./start_embodied.sh --llm cloud
```

具身入口默认 Gazebo；建图、独立抓取入口默认 Isaac Sim。用 `--sim isaac|gazebo` 切换。常用选项为 `--world`、`--bundle`、`--headless`、`--build` / `--no-build`，完整选项见各脚本 `--help`。建图和抓取默认构建，具身入口默认复用已有构建。每次启动会先停止上次项目会话。

ACT 需准备外部 ACT 环境与权重，路径见 [config.json](src/embodied/act_policy/config.json)；可用 `EMBODIED_ACT_CHECKPOINT` 覆盖所选权重。实验性 VIO 的配置和限制见[定位说明](src/localization/README.md)。

## 任务执行与地图

明确的取物指令由代码按阶段执行：

**地图就绪 → 房间区域或 VLM 估计搜寻点 → Nav2 导航 → 局部相机建图与目标搜寻 → 视觉伺服进入候选抓取工作区 → 抓取、抬起、夹持回 Home。**

普通指令使用 Qwen 原生工具调用循环；VLM 根据地图中的物体类别、位置和相机观察推断区域。导航与抓取的具体动作由 Nav2、GraspNet／MoveIt 或 ACT 执行。

- **持久地图**：建图和具身入口共用默认路径 `maps/<后端>/semantic_<场景>`；必须指定相同的 `--sim`、`--world` 和 `--bundle`。建图保存几何地图及 `.semantic.json`／`.semantic.json.gz`（物体XYZ、房间区域、VLM估计搜寻点、证据及几何指纹）。房间范围未确认时，允许导航到VLM选出的可通行估计点，到达后开启局部相机建图搜物。详见 [地图交接与保存格式](src/embodied/semantic_navigation/README.md)。
- **局部地图**：相机深度输入，默认暂停；到达目标区域开始搜物时清空并开启，发布 `/local_map/octomap_full` 供 MoveIt 避障，任务结束后暂停。
- **全局地图**：LIO 模式持续融合 FAST-LIO 几何，并通过相机掩码投影与 raycast 给可见 LiDAR 占据体素打语义；显示话题为 `/global_map/occupied_cells_vis_array`，兼容 `/occupied_cells_vis_array`。
- **视觉伺服**：使用新鲜 RGB-D 检测锁定目标实例，低速调整底盘朝向和距离；目标丢失时停止平移并重新观测。进入候选抓取工作区且底盘停稳后，交给 GraspNet／MoveIt 或 ACT；速度指令仍经过平滑、避障与定位安全门。
- **执行检查**：定位、观测新鲜度、停车误差及视觉确认不满足要求时停止当前阶段。导航地图须匹配仿真后端与场景。

建图和具身入口可通过相同的 `--bundle` 交接地图。探索保存器不覆盖已有目录，重新建图时指定新路径；已有完整几何地图时，具身入口直接重定位。几何地图、语义摘要和本地验证数据不纳入 Git。

2026-10-24，Gazebo 单次“厨房拿可乐”完成 VLM 估计点交接、搜寻、视觉伺服、抓取、抬起、回 Home 及2秒夹持检查，总耗时约71秒；伺服目标距离从1.05米收敛至0.72米。前一轮因检测丢失停车，VLM 区域边界仍可能误判，单次成功不代表多起点或遮挡场景的成功率。详情见[语义导航验证记录](src/embodied/semantic_navigation/README.md)。

## 模块与实验

| 模块 | 说明 |
|---|---|
| [语义导航](src/embodied/semantic_navigation/README.md) | 持久地图、VLM 区域推断、目标跟踪与当前验证状态 |
| [Agent](src/embodied/llm_agent/README.md) | 自然语言编排及历史实验 |
| [ACT](src/embodied/act_policy/README.md) | RGB／RGB＋深度／RGB＋XYZ 结果与回放 |
| [语义建图](src/mapping/semantic_voxel_mapping/README.md) | 局部／全局 OctoMap 配置 |
| [手眼标定](src/manipulation/handeye_demo/README.md) | AprilGrid 标定、深度抓取及误差统计 |

源码按 `robot`、`localization`、`perception`、`planning`、`control`、`mapping`、`manipulation`、`simulation`、`embodied` 分组；启动配置在根目录脚本，运行工具在 `scripts/runtime/`。模型、地图、缓存和运行日志不纳入 Git。
