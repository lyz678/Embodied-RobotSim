# CUDA 多帧语义体素地图

探索入口默认使用本包。直接运行 `./start_explore_and_mapping.sh`；入口顶部的 `OCTOMAP_BACKEND=semantic_cuda` 和 `SEMANTIC_MAP_CONFIG` 控制后端和配置路径。改为 `OCTOMAP_BACKEND=legacy` 可回到原 ColorOctoMap；导航、抓取和 LLM 入口暂时仍用原后端。

本包使用自己的稀疏体素状态融合地图，CUDA 执行三维 DDA 射线遍历、排序和去重。CPU 负责每帧证据合并、占据概率、语义投票和 ROS 输出。`ColorOcTree` 只用于兼容导出，不参与融合；不会通过平均 RGB 生成不存在的类别颜色。必须有可用 CUDA 设备，启动失败会明确报错，不自动回退 CPU。

## 输入与融合

YOLOE 新话题 `/yoloe_multi_text_prompt/pointcloud_semantic` 包含 `x/y/z`（FLOAT32）、`rgb`（UINT32，0xRRGGBB）和 `confidence`（FLOAT32）。探索入口顶部两个来源开关相互独立：

```bash
SEMANTIC_CLOUD_SOURCE=fastlio # fastlio 或 depth
DEPTH_SOURCE=isaac           # isaac 或 lsm，仅 depth 模式使用
```

默认 `fastlio` 模式订阅 `/fastlio/cloud_odom`：这是 FAST-LIO 去畸变后的单帧扫描经 adapter 转到 `odom` 的点云，不是累积地图。同步 RGB、CameraInfo 和单帧扫描，最大时间差默认 0.15 个仿真秒；按图像时间查询相机姿态、按扫描时间查询点云姿态，通过固定 `odom` 坐标系补偿机器人运动。相机坐标中 z>0 且投影落在图像范围内的点才有资格上色；检测 bbox 仅确定 mask 的偏移，只有 mask 非零的像素赋类别 RGB。重叠 mask 选置信度更高的类别，同一像素比最前点远 0.15 m 以上的返回不赋类别。相机 CameraInfo 必须匹配输入图像；当前 Isaac 图像无畸变，使用 PinholeCameraModel 的校正相机投影。

所有有限 FAST-LIO 端点都保留，包括视野外、mask 外和遮挡点，以白色 RGB 和零置信度表示未知。输出点坐标转换到扫描时间的 `mid360_link`，时间戳仍为扫描时间，因此 raycast 使用雷达原点，不能将 `odom` 原点或相机原点当作激光原点。不会用相机深度替换激光点坐标，也不会把缺少激光返回的方向当作空闲观测；地图自身的距离/高度筛选仍生效。

`SEMANTIC_CLOUD_SOURCE=depth` 保留原有深度点云路径：`DEPTH_SOURCE=isaac` 使用仿真器深度，`DEPTH_SOURCE=lsm` 使用双目推理深度。有效背景深度保留为未知，检测掩膜及深度 MAD 过滤决定语义颜色。旧点云和 Detection3D 输出在 depth 模式保持原行为；导航抓取/LLM 的 legacy 后端自动采用 depth 模式。fastlio 模式用于探索语义建图，只发布语义点云和检测图像，不发布旧 RGB-D 点云/Detection3D；不会启动未使用的 LSM 节点。

投影参数在 `src/yoloe_infer/configs/config.yaml`，也可用同名 ROS 启动参数覆盖：

| 参数 | 默认值 | 用途 |
|---|---|---|
| `semantic_cloud_source` | depth，探索入口覆盖为 fastlio | 几何来源 |
| `semantic_lidar_topic` | `/fastlio/cloud_odom` | 单帧点云话题，需要可用 TF |
| `semantic_lidar_frame` | `mid360_link` | 输出坐标系与 raycast 原点 |
| `projection_fixed_frame` | `odom` | 跨时间投影的固定坐标系 |
| `projection_sync_slop` | 0.15 s | RGB/扫描同步容差 |
| `projection_occlusion_tolerance` | 0.15 m | 同像素前后遮挡容差 |

投影不会绕过地图的置信度和跨帧投票门槛。激光点比深度图点稀疏，颜色覆盖率取决于相机与激光重叠视野、扫描密度和检测置信度。

`rgb` 是类别颜色编码，输入不需要额外 `class_id`。调色板来自安装后的 `yoloe_infer/configs/config.yaml`，或 `palette_file` 指定的同格式配置。每个类别必须有唯一 RGB，白色保留为未知。相机 RGB 纹理点云不能当作本节点的语义输入。

每帧、每个体素最多一次 hit/miss 更新，hit 优先；像素再多也不会增加单帧语义票数。置信度合格的像素先在体素内选出唯一最多的类别，平票弃权，之后每帧最多记一票。默认保留最近 30 次有效语义观测，至少 3 次且最多类别占比达到 60% 才输出类别颜色，否则输出白色。未知和低置信度观测仍更新几何，不参与语义投票。miss 使体素变为空闲时清空其语义历史。

占据使用 log-odds：默认 hit=0.7、miss=0.4，概率限制为 0.12–0.97，超过 0.5 为占据。超过最远距离的观测只清空截断射线，不生成端点占据。端点超出高度范围时只更新范围内经过的空闲体素；高度筛选按体素中心进行。

使用点云时间戳查询精确 TF 到 `odom`，最多等待 0.5 秒，失败整帧丢弃。输入队列深度 2，默认按墙钟最多融合 5 Hz、发布 1 Hz。CUDA 按块处理，默认临时显存预算 512 MiB；最多保存 200 万个体素。预算不足或 CUDA 失败时整帧拒绝，不提交半帧状态。仿真时间倒退会暂停融合，需要重启定位链路后 clear 或 load。

多数投票能抑制偶发误检，持续误分类仍可能被确认。本包不区分同类不同实例，不提供回环校正；`odom` 的漂移会影响地图。

## 参数与输出

修改 [config/map.yaml](config/map.yaml) 后重启，参数在运行中只读。主要接口：

| 参数 | 默认值 | 用途 |
|---|---|---|
| `cloud_topic` | `/yoloe_multi_text_prompt/pointcloud_semantic` | 语义输入 |
| `map_frame` | `odom` | 融合坐标系 |
| `resolution` | 0.10 m | 体素边长，最小 0.005 m |
| `min_range/max_range` | 0.2 / 8.0 m | 相机观测距离 |
| `min_z/max_z` | 0.1 / 2.5 m | 地图高度范围 |
| `confidence_threshold` | 0.5 | 有效检测置信度下限 |
| `vote_window/min_observations/majority_threshold` | 30 / 3 / 0.6 | 多帧确认规则 |
| `hit/miss/clamp_min/clamp_max/occupied_threshold` | 0.7 / 0.4 / 0.12 / 0.97 / 0.5 | 占据更新 |
| `cuda_device/gpu_scratch_mib/max_voxels` | 0 / 512 / 2000000 | 资源限制 |
| `integration_rate/publish_rate` | 5 / 1 Hz | 墙钟融合与发布频率 |
| `palette_file` | 空，自动查询 YOLOE 配置 | 类别调色板 |
| `map_file` | `maps/semantic_map.svm` | 完整状态保存路径，相对启动目录 |

| 话题 | 内容 |
|---|---|
| `/semantic_map/voxels` | 占据体素中心，RGB、occupancy 和 semantic_confidence |
| `/occupied_cells_vis_array` | 带类别颜色的 RViz 体素方块，兼容现有显示 |
| `/octomap_full` | 标准完整 ColorOcTree 导出；不包含投票历史 |
| `/projected_map` | 体素地图的二维投影，有占据列为 100、观测空闲列为 0、未观测为 -1 |
| `/semantic_map/diagnostics` | 融合/丢弃/错误计数、体素数、射线与整帧时间、临时显存估算 |

`semantic_confidence` 是历史多数类别的票数比例，即使尚未满足确认条件也可能非零，不是 YOLOE 检测置信度。二维导航的 `/map` 仍来自 MID360/FAST-LIO 建图链路，不切换到本包的 `/projected_map`。几何来源由 `SEMANTIC_CLOUD_SOURCE` 选择，depth 模式的深度来源再由 `DEPTH_SOURCE` 决定。新后端应在 YAML 里指定 `cloud_topic`，不使用旧后端的 `OCTOMAP_CLOUD_TOPIC`。

## 保存、加载和清空

```bash
ros2 service call /semantic_voxel_map/save std_srvs/srv/Trigger '{}'
ros2 service call /semantic_voxel_map/load std_srvs/srv/Trigger '{}'
ros2 service call /semantic_voxel_map/clear std_srvs/srv/Trigger '{}'
```

save 通过临时文件替换 `map_file`，会覆盖该路径的旧快照。保存格式包含每个体素的 log-odds、完整有序投票历史及坐标系/融合参数/类别调色板签名。load 检查兼容性和数据完整性后整体替换状态，失败保留原图。不会自动加载旧地图，也不隐式恢复定位坐标；用户须确保新的 `odom` 与保存时一致。v1 二进制采用本机数值表示，面向当前 x86_64 Linux 环境。

## 构建和验证

```bash
source /opt/ros/jazzy/setup.bash
export PATH=/usr/local/cuda/bin:$PATH
colcon build --symlink-install --packages-select semantic_voxel_mapping yoloe_infer x_bot
ctest --test-dir build/semantic_voxel_mapping --output-on-failure
ctest --test-dir build/yoloe_infer --output-on-failure
build/semantic_voxel_mapping/benchmark_raycast
```

CUDA 编译默认生成本机 GPU 的原生指令，避免驱动 PTX JIT 版本不匹配。交叉编译可显式传 `--cmake-args -DCMAKE_CUDA_ARCHITECTURES=<目标架构>`。

核心测试覆盖重复证据、hit 优先、误检投票、未知/平票、历史淘汰、空闲清除、持久化和预算拒绝。benchmark 检查 CUDA 与 CPU 的空闲体素集合完全一致，包含负坐标、轴向和边界射线。报告中的 `gpu_ms` 包括每块射线、排序去重、回传及部分主机处理，不代表整帧融合或仿真实时倍率。

YOLOE 投影测试验证 FOV、mask（不是 bbox）、同像素遮挡、重叠检测置信度、原始端点与雷达时间戳保留，以及旧深度模式兼容。
