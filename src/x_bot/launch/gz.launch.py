#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable

# Gazebo仿真环境启动文件
# 功能：启动Gazebo仿真器并生成x机器人模型
# 用于ROS2 + Gazebo的机器人仿真环境配置


def generate_launch_description():
    # 定义启动参数配置
    # use_sim_time: 是否使用仿真时间（Gazebo时间）
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    # 传感器使能配置
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)  # 2D激光雷达
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)            # 相机使能

    # 机器人初始姿态配置
    position_x = LaunchConfiguration("position_x", default="0.0")  # 初始X坐标
    position_y = LaunchConfiguration("position_y", default="0.0")  # 初始Y坐标
    orientation_yaw = LaunchConfiguration("orientation_yaw", default="0.0")  # 初始偏航角（朝东）

    # 里程计源配置
    odometry_source = LaunchConfiguration("odometry_source", default="world")  # 里程计参考系
    publish_odom_tf = LaunchConfiguration("publish_odom_tf", default="false")

    # 世界文件配置（默认为ware_house）
    world_name = LaunchConfiguration("world_name", default="ware_house")  # 仿真世界名称
    world_file = PythonExpression(["'", join(get_package_share_directory("x_bot"), "worlds"), "/", world_name, ".sdf'"])

    # 获取包共享目录路径
    x_bot_path = get_package_share_directory("x_bot")  # x机器人包路径
    gz_sim_share = get_package_share_directory("ros_gz_sim")  # ROS-Gazebo桥接包路径

    # 启动Gazebo仿真器
    # 使用ros_gz_sim包的标准启动文件
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            # Gazebo启动参数：指定世界文件并自动开始仿真(-r)
            "gz_args" : PythonExpression(["'", world_file, " -r'"])
        }.items()
    )

    # 启动x机器人生成节点
    # 该节点负责在Gazebo中生成机器人模型并配置传感器
    spawn_x_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(x_bot_path, "launch", "x_bot_gz_spawn.launch.py")),
        launch_arguments={
            # 传递传感器使能参数
            "two_d_lidar_enabled": two_d_lidar_enabled,
            "camera_enabled": camera_enabled,

            # 传递机器人初始姿态参数
            "position_x": position_x,
            "position_y": position_y,
            "orientation_yaw": orientation_yaw,

            # 传递里程计配置参数
            "odometry_source": odometry_source,
            "publish_odom_tf": publish_odom_tf,

            # 传递世界名称
            "world_name": world_name,
        }.items()
    )

    # 返回启动描述，包含所有配置和动作
    return LaunchDescription([
        # 声明所有启动参数，使其可通过命令行覆盖默认值
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value=two_d_lidar_enabled),
        DeclareLaunchArgument("camera_enabled", default_value=camera_enabled),
        DeclareLaunchArgument("position_x", default_value=position_x),
        DeclareLaunchArgument("position_y", default_value=position_y),
        DeclareLaunchArgument("orientation_yaw", default_value=orientation_yaw),
        DeclareLaunchArgument("odometry_source", default_value=odometry_source),
        DeclareLaunchArgument("publish_odom_tf", default_value=publish_odom_tf),
        DeclareLaunchArgument("world_name", default_value=world_name),
        DeclareLaunchArgument("world_file", default_value=world_file),

        # 设置Gazebo资源路径，使其能找到自定义世界文件和模型
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(x_bot_path, "worlds")),  # 添加世界文件路径

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(x_bot_path, "models")),  # 添加模型文件路径

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(x_bot_path, "models", "small_house")),  # aws_robomaker模型路径

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(x_bot_path, "models", "simple_house")),  # simple_house模型路径

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(x_bot_path, "models", "ware_house")),  # ware_house模型路径

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(get_package_share_directory("franka_description"), "..")), # franka_description parent path for model:// resolution

        # 包含的启动动作：Gazebo仿真器和机器人生成节点
        gz_sim, spawn_x_bot_node
    ])