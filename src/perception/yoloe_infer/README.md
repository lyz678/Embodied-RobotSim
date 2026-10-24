# YOLOE + FAST-LIO 三维多目标跟踪

在原 C++ / TensorRT 检测节点中实现 BoT-SORT 风格扩展，不依赖 Python
Ultralytics/FastReID，也没有移植官方实现的全部细节。算法依据：
[BoT-SORT 论文及官方实现](https://github.com/NirAharon/BoT-SORT)。
保留高/低置信度两阶段关联、XYWH Kalman 预测、平滑外观特征和 lost 轨迹恢复；
使用 FAST-LIO 的固定坐标系投影代替图像特征 GMC，并加入三维距离门控。

## 启动

先启动机器人相机和 FAST-LIO adapter，使 `/fastlio/cloud_odom`、CameraInfo
和带历史时间戳的 `odom → base_footprint → camera` TF 可用。
此功能使用单帧去畸变点云，不能用累积地图代替。相机须提供与图像一致的
针孔/已校正内参（当前仿真相机满足此要求）；实机畸变图像应先校正。

原 engine 未暴露 Neck 特征。构建新的模型文件，不覆盖原模型：

```bash
python3 src/perception/yoloe_infer/scripts/build_tracking_engine.py \
  --source src/perception/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.onnx \
  --output src/perception/yoloe_infer/models/yoloe-tracking.onnx
source /opt/ros/jazzy/setup.bash
colcon build --packages-select yoloe_infer --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run yoloe_infer ros2_trt_infer_text_prompt_multi_node --ros-args \
  -p config_path:="$(ros2 pkg prefix yoloe_infer)/share/yoloe_infer/configs/tracking.yaml" \
  -p use_sim_time:=true
```

需要 `onnx` Python 包和 TensorRT `trtexec`。默认暴露 bundled YOLOE 的 P3
`/model.15/cv2/act/Mul_output_0`，输出名 `reid_p3`、形状 `[1,256,80,80]`。
其他架构使用 `--tensor` 指定检测头之前的实际 Neck 张量；不要把检测头或
mask prototype 当作 Neck。脚本校验形状，支持 TRT 10 的 FP16 builder 和
TRT 11 的 strongly typed builder。`--onnx-only` 可仅导出 ONNX。
TensorRT engine 与 GPU/运行时相关，应在目标机器构建。

`start_mapping_and_explore.sh` 和直接使用 `robot_services.sh explore` 默认选择
`tracking.yaml`，启用 FAST-LIO 三维跟踪及图像 ID。可用 `--yoloe-config` 覆盖配置。

`tracking.yaml` 默认开启 FAST-LIO 三维跟踪；`manipulation.yaml` 开启 depth 模式的
深度三维/外观跟踪，在同一图像话题显示 ID，保留原深度几何处理。`config.yaml`
也默认开启跟踪，所有内置配置统一使用 `yoloe-tracking.engine`。配置未填写
`tracking_enabled` 时同样默认开启；需要关闭可显式设置 `tracking_enabled: false`
或通过 ROS 参数 `-p tracking_enabled:=false` 覆盖。开启跟踪却缺少 Neck 输出时，
节点会明确报错。

## 数据流和输出

1. 图像、CameraInfo、点云近似同步，默认最大时间差 0.15 秒。TF 分别查询
   图像时刻的相机位姿与扫描时刻的点云位姿，通过 `projection_fixed_frame`
   （默认 `odom`）补偿自车运动，不使用最新 TF 代替历史位姿。
2. 点云投影到检测框/实例 mask；同像素遮挡剔除、深度分组、至少 5 个有效点，
   取最近有效组的三轴中位数。估计是可见物体表面代表位置，不是完整物体
   几何中心；无有效点时使用二维关联，不伪造三维测量。
3. 复用同一次检测推理的 P3 特征，根据实际 letterbox 参数映射 ROI，
   通道平均池化并 L2 归一化为 256 维向量，再以 EMA 平滑轨迹外观。
   该向量是未额外训练的 Neck 描述子，不能等同于专门训练过的 ReID 网络。
4. 14 维联合常速度状态：`[cx,cy,w,h,X,Y,Z,vcx,vcy,vw,vh,VX,VY,VZ]`。
   使用实际图像时间差预测，测量缺失时只更新可用维度，Joseph 形式更新协方差。
   将世界坐标预测位置投影到当前相机，补偿 IoU 关联中的自车平移/旋转。
5. 第一阶段融合 `w_iou*(1-IoU) + w_app*(1-cos)/2 + w_3d*distance/gate`，
   按实际可用项归一化。类别、三维欧氏距离、Mahalanobis 距离及外观门控
   排除明显错误匹配；Hungarian 分配显式允许不匹配。第二阶段用低分检测
   延续近期已确认轨迹，不依赖低分外观。只有高分检测能新建轨迹。

- `~/image_result`：检测图像，确认轨迹显示 `ID: 12`。
- `~/tracks_3d`：`vision_msgs/Detection3DArray`，图像时间戳、固定世界坐标系。
  `Detection3D.id` 是稳定轨迹 ID；`bbox.center` / `results.pose.pose` 为滤波位置，
  `results.pose.covariance` 的平移部分为位置协方差，class_id/score 保留检测语义。
  不估计三维包围盒尺寸/方向。仅发布当前检测匹配且已确认、有三维状态的轨迹；
  短暂缺少 LiDAR 测量时位置由运动模型预测，超过 max_age 后停止使用旧三维状态。
  整帧漏检时仍内部预测保留 lost 轨迹，但发布空数组，避免下游把预测当成检测。
- 原有语义点云话题继续输出。关闭 `enable_pointcloud` 后跟踪仍工作；关闭推理
  会清空轨迹，恢复时重新分配 ID。时间回退清空轨迹，进程内不回收 ID。
- 原有 `~/detections_3d` 是 depth 路径输出；两种跟踪路径均使用 `~/tracks_3d`。

阈值与三项权重见 `configs/tracking.yaml`。三维 measurement_variance 单位为
平方米，包含点云、投影、标定与 FAST-LIO 误差的固定近似，并非传播 FAST-LIO
完整位姿协方差。两传感器时间差仍会引入目标自身运动误差，必要时收紧同步窗口。
未观察到的二维大小继续由 Kalman 预测，自车运动下的尺度变化不做精确三维建模。
实际稳定性增益需用移动相机、交叉目标和遮挡场景测量 IDF1 / ID switches；
当前自动测试不构成准确率或性能提升结论。

## 验证

```bash
colcon test --packages-select yoloe_infer --event-handlers console_direct+
colcon test-result --verbose
```

`bot_sort_3d` 覆盖交叉身份保持、低分延续、遮挡恢复、过期、时间回退、二维降级、
协方差、自车运动投影、全局分配、Neck ROI 池化和 LiDAR 前景定位。
`semantic_projection` 覆盖原有语义点云几何与投影兼容性。

## 抓取模式类别与 ID 抖动排查

图像上的 `ID:` 是实例跟踪 ID；类别编号在 `results[].hypothesis.class_id`。
二者用途不同。抓取配置显式使用 low=0.10、high=0.20、birth=0.25、min_hits=3：
只有达到 birth 的检测能初始化轨迹，后续连续匹配三次才显示 ID。
此前省略阈值时使用 high=0.50 / birth=0.60，约 0.2–0.3 的瓶体检测无法建轨。
这些较低阈值是针对当前仿真抓取场景的选择，可能增加误检轨迹，不能当作通用准确率提升。

跟踪输入仍保留 low 以上的检测；对外图像、深度检测和语义点云仅保留
达到 `conf_thres` 的当前检测，包括已确认轨迹。
低于 `conf_thres` 的检测只在跟踪器内部维持身份，不进入图像和抓取语义输出。
`manipulation.yaml` 的 `suppress_cross_class_duplicates` 会抑制 IoU 大于
max(iou_thres, 0.85) 的跨类别重复框，避免瓶体同时被当作 bottle 和 coke；
普通同类 NMS 仍使用 `iou_thres`。其他配置默认不启用跨类抑制。

抓取模式关闭 `tolerate_class_jitter`，严格按类别关联，防止杯子和可乐串轨。
若自行开启跨类关联，仍须满足 IoU≥0.7、外观余弦相似度≥0.85 和其他门控。
轨迹内部类别须连续 `class_switch_hits` 次达到 high 的观测才更正；
历史类别仅保留在轨迹状态和 `tracks_3d` 中。图像、`detections_3d`、语义点云
始终使用当前 YOLOE 检测的类别和对应置信度，ReID 不得改写分类结果。

深度抓取路径对对齐的 32FC1 深度每四像素采样，在固定坐标系中估计实例位置，
并使用图像时刻的 TF 投影补偿相机运动。要求 RGB、深度和内参的坐标系及尺寸一致，
RGB/depth 时间差不超过 50 ms；深度或 TF 不可用时打印警告并降级二维关联。
快速转动、长时间遮挡、目标完全离开视野或重启节点仍可能换 ID；
上述修改不提供跨进程或任意遮挡下的永久身份保证。


## 厨房任务的图像卡顿与分类诊断（2026-10-12）

`publish_lidar_semantics:=true` 可在深度抓取模式同时输出独立的
`~/pointcloud_semantic_lidar`，供全局 LiDAR 图进行语义 raycast。
RGB/LiDAR 与 RGB/depth 分别按时间同步，同一幅 RGB 的检测结果缓存复用；
关闭 `~/enable_pointcloud` 只暂停相机/局部点云，不会关闭全局 LiDAR 语义输出。

深度模式将配对的RGB/depth/CameraInfo放入最多两帧的短队列，等待采集时刻的精确odom TF可用后再生成三维检测。10ms定时器非阻塞重试；墙钟等待超过0.3秒或图像仿真年龄超过0.5秒则丢弃。这样可以容纳定位TF常见的10–35ms延迟，不必在每帧做二维关联降级。历史图像测量仍使用原采集时间；视觉伺服另用年龄≤0.35秒的最新相机TF预测当前视野。

- 当前 web_video_server 不解码 query 中的 `%2F`，会把编码后的话题当作非法 ROS
  话题拒绝。前端保留 `/`，请求 `qos_profile=sensor_data`；编码请求实测只有
  22 字节结束边界，修正请求正常返回 JPEG。MJPEG 的 `<img>` 不提供每帧事件，
  界面显示 MJPEG，不再用 80 ms 定时器伪造帧率。
- TF 暂时断开时，旧深度坐标和地面过滤每处最多等待 0.5 仿真秒，日志可见
  单帧 0.8–1.6 墙钟秒。图像路径现使用不阻塞的指定时刻 TF；缺失时跳过
  对应三维结果并发布空检测数组，不使用最新位姿代替旧图像位姿。
  输入队列为 2、深度同步队列为 3、图像输出队列为 1，减少旧帧堆积。
- `Frame ms` 分别记录 detect / track / preview / semantic / coordinates /
  legacy / filter-publish，便于区分检测、关联和点云耗时。
- 原 ONNX 与 tracking ONNX 的 426 个权重完全一致，原节点也完全一致，后者
  只追加一个特征 Cast 输出。厨房回放 15/20/25/35 秒的四帧原 engine 与 tracking
  engine 原始检测类别一致，置信度近似一致；其中杯状物被两者识别为 book。
  这说明此次样本的分类错误存在于原检测模型，不能归因于 ReID 网络改了分类。
  四帧对照不是分类准确率评测，也未复现用户报告的每一次杯子/可乐互换。
- 之前把历史轨迹类别覆盖到检测框上的实现已移除，并增加原始类别保留回归测试。
  检测错误仍需用场景数据验证提示词或调整检测模型；修复身份关联不等于修复分类。
