#!/usr/bin/python3

"""
Nav2 导航系统启动文件 (Cartographer SLAM 版本)
功能：启动 Nav2 导航栈，搭配外部 Cartographer SLAM
区别：不启动内置 SLAM，使用 Cartographer 提供的 /map 和 TF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Nav2 导航系统启动描述 (Cartographer 版本)"""

    # 获取包路径
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_x = get_package_share_directory('x_bot')

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # 参数声明
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Nav2 导航启动（不启动 SLAM，使用外部 Cartographer）
    # 注意：这里使用 navigation_launch.py 而不是 bringup_launch.py
    # 因为 bringup_launch.py 会启动 SLAM 或 map_server，我们不需要
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': os.path.join(pkg_x, 'config', 'nav2_params_cartographer.yaml'),
        }.items()
    )

    # RViz 可视化工具
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )
        ]
    )

    # 构建启动描述
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_autostart_arg)

    # 添加核心启动动作
    ld.add_action(nav2_launch)
    ld.add_action(rviz_launch_cmd)

    return ld
