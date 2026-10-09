# LIO / VIO 定位

默认 `lio` 使用 FAST-LIO + MID360；实验性 `vio` 使用 NVIDIA Isaac ROS Visual SLAM
4.4（ROS 2 Jazzy），融合双目灰度图和 `/livox/imu`，`tracking_mode: 1`。

**当前状态（2026-10-09）：本机 GPU 短程测试通过，长时间 VIO 跟踪仍不稳定。**
最新三分钟自主探索中，机器人最大真值位移 5.147 m，随后连续位姿异常触发安全停车。
默认继续使用 LIO；启动正常和偶尔到达目标不代表完成稳定的全场景探索。

## 依赖与构建

本机验证环境：Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.1、RTX 5060 Ti 16 GB，
NVIDIA 驱动 580.178.04、主机 CUDA 13.3，Isaac ROS 4.4 / cuVSLAM 15.0 / VPI 4.0.5。
本集成使用 `isaac_ros_visual_slam` 的 release-4 组件接口，升级上游时须核对接口及依赖。

在项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/setup/setup_vio_dependencies.py
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

安装器按 [固定版本清单](../../scripts/setup/vio_runtime_packages.json) 下载约 415 MB
运行库并验证 SHA256，仅解包到 `.cache/vslam/root`，无需 sudo。
主机仍需已有 Jazzy、CUDA 13 运行库及兼容驱动；安装器检查动态链接依赖。
根目录脚本只为 VIO 的原生 ROS 子进程加载 `.cache/vslam/env.bash`，
Isaac Sim 与 LIO 保持各自环境。单独运行 VIO ROS 命令时，在 ROS 和工作区 setup
之后执行 `source .cache/vslam/env.bash`；Zsh 使用对应的 `setup.zsh`。
LIO 无需安装这些运行库，缓存、测试日志及生成地图均不纳入 Git。

## 启动

```bash
# 默认 Isaac Sim，自动探索与建图
./start_mapping_and_explore.sh --odometry vio
# 保留 LIO
./start_mapping_and_explore.sh --odometry lio
# 关闭自动探索，供短程诊断使用
./start_mapping_and_explore.sh --odometry vio --headless --no-explore --bundle maps/vio_test
# 使用同场景完整地图包导航
./start_mapping_and_explore.sh --mode navigation --odometry vio --bundle maps/my_vio_map
# 单独启动定位，需已有仿真器和 robot_state_publisher
ros2 launch x_bot_bringup localization.launch.py odometry_source:=vio mode:=mapping bundle:=maps/vio_session
```

三个根目录入口均支持 `--odometry`，也可修改脚本顶部 `ODOMETRY_SOURCE`。
`--mode` 选择任务，`--odometry` 独立选择里程计；缺少 VIO 依赖时在停止当前仿真前退出。
Gazebo 可组合 `--sim gazebo --odometry vio`，但这里记录的 GPU 实测使用 Isaac Sim。
VIO 抓取与具身流程尚未验证；相机安装在机械臂上，VIO 的固定相机/IMU 外参要求
测试时保持机械臂姿态不变。

VIO 导航最大前进速度 0.3 m/s、最大角速度 0.5 rad/s，速度平滑器限制加速度
0.3 m/s² / 0.5 rad/s²；LIO 保留原配置。单独启动导航也需传入
`ros2 launch x_bot_navigation navigation.launch.py odometry_source:=vio`。

## 核心实现与数据流

| 位置 | 职责 |
|---|---|
| [vio.launch.py](x_bot_localization/launch/vio.launch.py) / [vio.yaml](x_bot_localization/config/vio.yaml) | 加载 NVIDIA VisualSlamNode，配置双目、IMU、TF 与平面约束 |
| `.cache/vslam/root/opt/ros/jazzy/lib/libvisual_slam_node.so` / `libcuvslam` | NVIDIA 原生组件及 VIO 算法二进制；本仓库不包含 cuVSLAM 算法源码 |
| [vio_stereo.py](x_bot_localization/x_bot_localization/vio_stereo.py) | mono8 转换、可靠 QoS、右相机光学帧与夹爪 ROI |
| [vio_adapter.py](x_bot_localization/x_bot_localization/vio_adapter.py) | 跟踪状态/位姿检查、统一 odom/TF、深度与位姿配对 |
| [vio_geometry.py](x_bot_localization/x_bot_localization/vio_geometry.py) | 深度投影 |
| [vio_cloud_filter.cpp](x_bot_localization/src/vio_cloud_filter.cpp) | 按 URDF 碰撞模型和采样时刻 TF 去除机器人自身点，生成 scan |
| [localization.launch.py](../robot/x_bot_bringup/launch/localization.launch.py) | 选择 LIO/VIO、建图与 ICP 定位、安全门 |

双目输入为 `/x_bot/camera_{left,right}/image_raw` 与对应 `camera_info`，
保留采集时间，可靠 QoS 接收大图像。右相机使用 `camera_right_optical_frame`。
CameraInfo ROI 排除图像底部 40% 夹爪区域，`masked_bottom_fraction` 可调；
图像尺寸与相机内参保持不变。当前平地配置启用 `enable_ground_constraint_in_odometry`。
IMU 来自 `/livox/imu`，坐标系 `mid360_imu_link`，配置采样频率 200 Hz；
真实传感器需重新标定噪声及外参。

cuVSLAM 输出 `/vio/odometry`，坐标系为 `vio_odom`，不直接发布 TF，关闭回环建图。
adapter 以初始底盘位姿建立 `odom`，发布 `/odom`、`/x_bot/odom` 和
`odom → base_footprint`；已有流程提供 `map → odom`。

左相机仿真深度（32FC1 米 / 16UC1 毫米）投影为点云，默认每 8 像素采样，
距离 0.1–10 m，与 VIO 位姿最大时间差 20 ms。同一 VIO 时间戳发布
`/vio/cloud_unfiltered`、`/vio/sensor_odom` 和 odom；自体过滤后输出
`/vio/cloud_odom`、`/x_bot/scan`，缺少模型或 TF 时等待。
几何建图使用仿真深度，`--depth-source` 只选择语义建图深度；
VIO 自动把 `fastlio` 语义点云选项切为 `depth`，不会启动 FAST-LIO。

前向深度视野小于 360° 激光。地图边界包含底盘周围至少 1 m 未知区域，
不将未观测区域标为自由空间；探索节点在底盘仍位于地图外时等待更新。
地图 bundle 记录里程计和传感器来源，沿用 PCD/栅格地图保存与 ICP 校正流程。

## 安全与排障

每帧位姿必须匹配同时间戳的 `/vio/diagnostics` 成功状态。
首次正常定位后报告 Lost、无效位姿、大幅跳变、连续三帧运动异常或高度漂移时，
adapter 锁定故障，停止发布定位/建图样本，并通过 `/vio/healthy=false` 触发安全门停车。
轻微孤立跳变只丢弃该帧并暂时停车，下一帧与最后接受的位姿比较，通过后恢复。
健康状态过期也禁止运动；锁定故障或仿真 reset 后须重启定位，不能仅修改 TF 时间戳。

| 日志 | 含义与处理 |
|---|---|
| `Robot transform is stale; waiting for localization` | 检查 adapter 故障、`/vio/healthy`、`/localization/ready` 和 TF。跟踪故障后的等待是保护行为，须解决跟踪问题后重启 |
| `Smoothed path leads to a collision` | 平滑候选路径未通过代价地图检查；现有行为树回退原规划路径，本地控制器继续检查碰撞，保留 `check_for_collisions=true` |
| `change_state ... failed to send response (timeout)` | Nav2 默认加载 [Fast DDS 服务配置](../planning/x_bot_navigation/config/fastdds_services.xml)，将服务响应等待从 100 ms 增至 2 s；本机重启已确认 `Managed nodes are active`。已设置 `FASTRTPS_DEFAULT_PROFILES_FILE` 时尊重用户配置 |
| `Invalid tracking thresholds` | YOLOE 跟踪参数须满足 `low_threshold < high_threshold <= new_track_threshold <= 1`。high 是高置信检测分界，new_track 是新建轨迹所需分数；不能配置 high=0.50、new_track=0.40 |

DDS 配置仅作用于服务响应发布端，不启用全局 `RMW_FASTRTPS_USE_QOS_FROM_XML`，
图像、点云和里程计 QoS 不受影响。

## 本机验证结果（2026-10-09）

在 Isaac Sim `simple_room` 中经根目录脚本运行 VIO、Nav2、YOLOE 与 CUDA 语义建图。
真值仅供测试对比，不参与定位输入。以下短程结果来自早期配置，不能作为最新配置
长期稳定性的证据；最后一行是启用当前安全检查、温和速度、平面约束和 ROI 的复测。

| 测试 | 结果 | 本机报告（不纳入 Git） |
|---|---|---|
| 10 仿真秒短程运动 | 通过；GPU/IMU 已启用，真值平移 0.380 m / 转向 0.191 rad；VIO 16.57 Hz，平移 RMSE 0.00948 m / 最大 0.02108 m，航向 RMSE 0.00160 rad，ready 100%，115 组配对，TF/scan/地图保存通过 | `.cache/vslam/startup_validation.json` |
| 自主探索启动 | 通过；真值位移 0.302 m，51 条自动前进指令，315 个 ready 样本全为 true，首个目标成功 | `.cache/vslam/exploration_validation.json` |
| 三分钟持续探索 | **失败**；最大真值位移 5.147 m，连续三帧异常达 90 ms / 0.314 m 后停车；140 秒质量观测平移 RMSE 0.355 m / 最大 1.618 m | `.cache/vslam/plane_validation.json`、`plane_quality.json` |
| 故障停车检查 | ready=false，安全速度为零 | `.cache/vslam/plane_stop_result.json` |

最新复测 Nav2 正常进入 active，未复现生命周期回复超时；VIO 长时间位姿异常仍待解决。
逐帧数据为对应 `.trace.json`，最新原始日志在 `.cache/vslam/plane_logs/`。

短程诊断会驱动仿真机器人并保存地图，先使用 `--no-explore` 启动，指定新的地图目录：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .cache/vslam/env.bash
python3 scripts/diagnostics/check_vio_runtime.py --drive --output .cache/vslam/short_test.json
# 自主探索只读观测，不发速度或目标；另行启动自动探索后运行
python3 scripts/diagnostics/check_autonomous_exploration.py \
    --timeout 180 --observe-duration 180 --output .cache/vslam/exploration_test.json
# 离线回归，无需 GPU 或仿真器；VIO 测试使用隔离的 ROS domain 83
python3 -m unittest discover -s tests -p 'test_vio.py'
python3 -m unittest discover -s tests -p 'test_mid360_offline.py'
```
