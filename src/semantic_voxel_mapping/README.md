# CUDA 多帧语义体素地图

探索入口默认使用本包。直接运行 `./start_explore_and_mapping.sh`；入口顶部的 `OCTOMAP_BACKEND=semantic_cuda` 和 `SEMANTIC_MAP_CONFIG` 控制后端和配置路径。改为 `OCTOMAP_BACKEND=legacy` 可回到原 ColorOctoMap；导航、抓取和 LLM 入口暂时仍用原后端。

本包使用自己的稀疏体素状态融合地图，CUDA 执行三维 DDA 射线遍历、排序和去重。CPU 负责每帧证据合并、占据概率、语义投票和 ROS 输出。`ColorOcTree` 只用于兼容导出，不参与融合；不会通过平均 RGB 生成不存在的类别颜色。必须有可用 CUDA 设备，启动失败会明确报错，不自动回退 CPU。

## 输入与融合

YOLOE 新话题 `/yoloe_multi_text_prompt/pointcloud_semantic` 包含 `x/y/z`（FLOAT32）、`rgb`（UINT32，0xRRGGBB）和 `confidence`（FLOAT32）。旧点云话题保持原行为。有效背景深度也进入新点云，以白色 RGB 和零置信度表示未知；检测掩膜、深度离群点过滤及重叠区域的最高检测置信度决定语义颜色。

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

`semantic_confidence` 是历史多数类别的票数比例，即使尚未满足确认条件也可能非零，不是 YOLOE 检测置信度。二维导航的 `/map` 仍来自 MID360/FAST-LIO 建图链路，不切换到本包的 `/projected_map`。语义深度来源由探索入口的 `DEPTH_SOURCE` 决定：`isaac` 使用仿真器深度，`lsm` 使用双目推理深度。新后端应在 YAML 里指定 `cloud_topic`，不使用旧后端的 `OCTOMAP_CLOUD_TOPIC`。

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
build/semantic_voxel_mapping/benchmark_raycast
```

CUDA 编译默认生成本机 GPU 的原生指令，避免驱动 PTX JIT 版本不匹配。交叉编译可显式传 `--cmake-args -DCMAKE_CUDA_ARCHITECTURES=<目标架构>`。

核心测试覆盖重复证据、hit 优先、误检投票、未知/平票、历史淘汰、空闲清除、持久化和预算拒绝。benchmark 检查 CUDA 与 CPU 的空闲体素集合完全一致，包含负坐标、轴向和边界射线。报告中的 `gpu_ms` 包括每块射线、排序去重、回传及部分主机处理，不代表整帧融合或仿真实时倍率。
