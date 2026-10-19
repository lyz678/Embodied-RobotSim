# AprilGrid 手眼标定与深度抓取

只保留已验证的 **4×4 AprilGrid + OpenCV Daniilidis**。FR3 末端相机使用 Isaac `SingleViewDepthCameraSensor`，基线 50 mm；抓取位置必须来自配准后的带噪声深度。

本次保存的仿真结果：标定外参 RMSE **1.184 mm / 0.111°**，闭爪前 TCP RMSE **0.814 mm / 0.354°**，物理抓取 **10/10**。原始观测、逐次结果和报告保存在 `.cache/handeye/aprilgrid/`。这是该仿真配置的一次实验，不代表真实 D435 精度或每次运行的保证。

## 运行

依赖现有 ROS 2 Jazzy、Isaac Sim 6.1 和已构建的机器人/MoveIt 包；系统 Python 使用 OpenCV ArUco、NumPy、SciPy、PyYAML、Matplotlib、cv_bridge。首次可加 `--build`。

```bash
# 新实验：30 个有效训练姿态、10 个独立测试姿态，然后执行 10 次抓取
./start_handeye_demo.sh all --output .cache/handeye/new-run --headless
# 显示 Isaac Sim 窗口：去掉 --headless，并使用新目录
./start_handeye_demo.sh all --output .cache/handeye/visible-run

# 分步执行；collect 同时求解外参并评分
./start_handeye_demo.sh collect --output .cache/handeye/new-run --headless
./start_handeye_demo.sh calibrate --output .cache/handeye/new-run
./start_handeye_demo.sh evaluate --output .cache/handeye/new-run
./start_handeye_demo.sh grasp --output .cache/handeye/new-run --headless
./start_handeye_demo.sh report --output .cache/handeye/new-run

# 中断后只续跑缺失试次，校验冻结的配置与外参，保留已有记录
./start_handeye_demo.sh grasp --resume --output .cache/handeye/new-run --headless
```

完整实验和分步实验应各使用新输出目录。省略 `--headless` 显示窗口；`--config FILE` 指定配置，`--domain-id` 默认 73。入口只清理自己的进程，使用域锁防止重复启动，日志分次保存。按一次 Ctrl+C 后等待数秒退出；清理会按 SIGINT → SIGTERM → SIGKILL 处理整个进程组，连续按键不会中断清理。

## 流程与数据

- AprilGrid：180 mm 标定板，16 个 `36h11` Tag（ID 0–15），标签边长 30 mm、间距 10 mm。角点联合 PnP，末端 TF 按图像时间戳查询；Daniilidis 仅使用训练集，真值仅用于独立测试评分。
- 深度：原生虚拟双目距离不能直接按 RGB 像素取值。利用原生点云的左相机射线，把**带噪声距离**重投影到 RGB，使用最近 Z 缓冲处理遮挡，不填补空洞。
- 抓取：40 mm / 60 g 方块顶面使用独立 ID 100。内部有效深度反投影、稳健拟合顶面，求中心与朝向，再用标定外参转换到基座。无效深度最多重观测三帧，不回退到理想深度或 PnP 平移。
- 几何：`fingers_center` 的 x 为接近轴、y 为夹持轴。当前夹指碰撞末端前伸约 64 mm，TCP 沿外法线退后 55 mm；方块允许四种等价夹持朝向，预检查接近/抓取/抬升的 IK 与碰撞。
- 验收：抬升指令 100 mm，物体实际抬升至少 80 mm，保持 2 秒且相对 TCP 滑移不超过 10 mm。每次打开夹爪并回到同一关节初始姿态，再重置物体。失败计入总试次，超时或复位失败则中止。

矩阵 `T_a_b` 将 b 系点转换到 a 系；抓取使用 `T_base_hand @ T_hand_camera @ T_camera_grasp`。不覆盖 URDF 静态 TF。

|用途|话题|
|---|---|
|RGB / 配准深度 / 内参|`/handeye/rgb/image_raw`、`/handeye/depth/image_raw`、`/handeye/camera_info`|
|评分数据|`/handeye/debug/ideal_depth/image_raw`、`/handeye/debug/truth`|
|原生未配准距离|`/handeye/debug/raw_sensor_depth/image_raw`|

深度为 `32FC1` 光学 Z、单位米，无效值 NaN；检查帧、尺寸、内参和时间戳一致。内置单视图模型的噪声/置信度使用本机默认参数；近场范围设为 0.10–3.0 m，参数未按真机 D435 拟合。

输出：`grid/` 保存图像、深度、角点、机器人位姿及独立评分真值；`grid/extrinsics.yaml` 保存外参；`grasp_frames/` 和 `grasps.json/csv` 保存逐次抓取；`evaluation.json`、`evaluation_samples.csv`、`report.md`、`errors.png` 保存精度统计。配置与续跑 manifest 冻结在输出目录。已有 10/10 归档中的标定附带深度采于配准修复前，仅用于归档；标定求解使用 RGB/PnP，抓取已使用正确配准深度。

```bash
python3 -m unittest discover -s tests -p test_handeye_demo.py -v
~/isaacsim/python.sh tests/isaac_handeye_depth_check.py --output .cache/handeye/depth-check
```

独立倾斜平面检查验证虚拟双目像素偏移被消除且保留距离噪声。API 依据：[OpenCV](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)、[Isaac 深度传感器](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_camera_depth.html)。
