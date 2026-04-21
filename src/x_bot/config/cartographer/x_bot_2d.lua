-- x_bot Cartographer 2D SLAM 配置
-- 使用 2D LiDAR + IMU 进行实时定位与建图

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 坐标系配置
  map_frame = "map",                    -- 地图坐标系
  tracking_frame = "imu_frame",         -- 跟踪坐标系 (IMU 所在坐标系)
  published_frame = "base_footprint",   -- 发布的坐标系 (机器人基座)
  odom_frame = "odom",                  -- 里程计坐标系
  
  -- Cartographer 生成 odom frame (map -> odom -> base_footprint)
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true, -- 投影到 2D 平面
  
  -- 位姿外推器
  use_pose_extrapolator = true,
  
  -- 传感器数据源配置
  use_odometry = false,                 -- 不使用外部里程计 (Cartographer 自己估计)
  use_nav_sat = false,                  -- 不使用 GPS
  use_landmarks = false,                -- 不使用地标
  
  -- 激光扫描配置
  num_laser_scans = 1,                  -- 1 个 2D 激光雷达
  num_multi_echo_laser_scans = 0,       -- 无多回波激光
  num_subdivisions_per_laser_scan = 1,  -- 每次扫描的细分数
  num_point_clouds = 0,                 -- 无 3D 点云
  
  -- 超时和发布频率配置
  lookup_transform_timeout_sec = 0.2,   -- TF 查询超时
  submap_publish_period_sec = 0.3,      -- 子地图发布周期
  pose_publish_period_sec = 5e-3,       -- 位姿发布周期 (200Hz)
  trajectory_publish_period_sec = 30e-3, -- 轨迹发布周期
  
  -- 传感器采样率
  rangefinder_sampling_ratio = 1.,      -- 测距仪采样率
  odometry_sampling_ratio = 1.,         -- 里程计采样率
  fixed_frame_pose_sampling_ratio = 1., -- 固定坐标系位姿采样率
  imu_sampling_ratio = 1.,              -- IMU 采样率
  landmarks_sampling_ratio = 1.,        -- 地标采样率
}

-- 使用 2D 轨迹构建器
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4  -- 后台线程数

-- 2D 轨迹构建器参数
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 使用 IMU 数据

-- 激光雷达参数 (匹配 x_bot URDF 配置)
TRAJECTORY_BUILDER_2D.min_range = 0.55     -- 最小测量范围 (米)
TRAJECTORY_BUILDER_2D.max_range = 16.0     -- 最大测量范围 (米)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- 缺失数据射线长度
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 累积的扫描数据量

-- 子地图配置
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90    -- 每个子地图的扫描数
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 地图分辨率 (米/像素)

-- 在线相关性扫描匹配 (更鲁棒但更慢)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- 线性搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- 角度搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres 扫描匹配参数
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- 运动滤波器 (减少不必要的计算)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- 位姿图优化参数
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35       -- 每 35 个节点优化一次
POSE_GRAPH.constraint_builder.min_score = 0.55  -- 约束最小分数
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

-- 回环检测参数
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

return options
