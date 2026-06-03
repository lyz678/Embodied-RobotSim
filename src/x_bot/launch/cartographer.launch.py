#!/usr/bin/python3

"""
Cartographer 2D SLAM 启动文件
功能：使用 2D LiDAR + IMU 进行实时定位与建图
输出：/map 占用栅格地图, TF (map -> odom -> base_footprint)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Cartographer SLAM 启动描述"""

    # 获取包路径
    x_bot_path = get_package_share_directory('x_bot')
    cartographer_ros_path = get_package_share_directory('cartographer_ros')

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # 参数声明
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the published occupancy grid'
    )

    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='0.1',  # 提高到 10Hz
        description='OccupancyGrid publishing period'
    )

    # Cartographer 核心节点
    # 执行 SLAM：接收 scan 和 imu 数据，输出 TF 和子地图
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(x_bot_path, 'config', 'cartographer'),
            '-configuration_basename', 'x_bot_2d.lua'
        ],
        remappings=[
            ('scan', '/x_bot/scan'),     # 激光雷达数据
            ('imu', '/x_bot/imu'),        # IMU 数据
        ]
    )

    # 占用栅格地图生成节点
    # 将 Cartographer 子地图转换为标准 OccupancyGrid 消息发布到 /map
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': resolution},
            {'publish_period_sec': publish_period_sec}
        ]
    )

    # TF 转 Odometry 节点
    # 将 Cartographer 发布的 TF (odom -> base_footprint) 转换为 /odom 话题
    # 供 Nav2 等需要 odom 话题的组件使用
    tf_to_odom_node = Node(
        package='x_bot',
        executable='tf_to_odom.py',
        name='tf_to_odom_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_footprint'},
            {'odom_topic': '/odom'},
            {'publish_rate': 50.0}
        ]
    )

    # 构建启动描述
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_resolution)
    ld.add_action(declare_publish_period_sec)

    # 添加节点
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(tf_to_odom_node)

    return ld
