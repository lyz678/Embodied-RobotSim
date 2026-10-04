# LSM 双目深度

ROS 包名是 `stereo_matching`。`start_explore_and_mapping.sh` 默认启动本节点：左右 RGB → TensorRT 视差 → 深度图和彩色点云。OctoMap 输入 `/x_bot/camera_left/nn_pointcloud`；YOLOE 深度输入 `/x_bot/camera_left/nn_depth`。FAST-LIO、`/map` 和导航虚拟扫描继续来自 MID360。

直接运行探索脚本即可，无需参数。脚本顶部暴露：

| 配置 | 用途 |
|---|---|
| `DEPTH_SOURCE` | `lsm`（默认）或 `isaac` |
| `LSM_CONFIG_FILE` | 基础配置 YAML：引擎、CUDA 设备、输入尺寸、内参、基线、话题 |
| `LSM_PARAMS_FILE` | 标准 ROS 参数 YAML，覆盖基础配置 |
| `DEPTH_IMAGE_TOPIC` | YOLOE 深度输入；空值按来源选择默认话题 |
| `OCTOMAP_CLOUD_TOPIC` | OctoMap 点云输入；空值按来源选择默认话题 |

`config/isaac_params.yaml` 是探索默认使用的参数接口：

```yaml
stereo_matching_node:
  ros__parameters:
    min_depth: 0.2
    max_depth: 8.0
    enable_pointcloud: true
    use_confidence_filter: true
    filter_percentile: 20.0
    use_gradient_filter: true
    gradient_threshold: 2.0
    gradient_dilation_size: 0
```

同一参数文件还可以覆盖 `engine_path`、`camera.fx/fy/cx/cy/baseline`、`topics.left_image/right_image/depth/pointcloud/disparity/confidence`。例如键 `camera.fx: 337.22194822727283`。`filter_percentile` 范围为 0–100，深度范围单位为米，基线单位为米。相机参数必须匹配真实输入；当前 Isaac 双目为 640×640、fx/fy≈337.222、cx/cy=320、基线 0.05 m。更改输出话题后也要调整探索脚本里的下游输入话题。

这些配置会在启动时创建模型、缓冲区和订阅，因此 ROS 参数为只读，修改 YAML 后重启。实时启停推理使用服务：

```bash
ros2 service call /stereo_matching_node/enable_inference std_srvs/srv/SetBool '{data: false}'
ros2 service call /stereo_matching_node/enable_inference std_srvs/srv/SetBool '{data: true}'
```

单独启动（先 source ROS 和工作空间环境）：

```bash
ros2 launch stereo_matching stereo_matching.launch.py \
  params_file:=/home/lyz/Embodied-RobotSim/src/LSM_depth_infer/config/isaac_params.yaml \
  use_sim_time:=true use_rviz:=true
```

引擎文件不提交到 Git；构建时将本目录已有的 `LSM_conf_640_fp16.engine` 安装至包的 `engines/`。相对引擎路径以安装后的包 share 目录为基准。
