#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition

# RViz可视化启动文件
# 功能：启动RViz可视化工具，支持实体机器人和Isaac Sim仿真环境

def get_xacro_to_doc(xacro_file_path, mappings):
    """将XACRO文件解析为URDF文档对象"""
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    """生成RViz可视化启动描述"""

    # 启动配置参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')  # 使用仿真时间（默认false，适用于实体机器人）
    isaac_sim = LaunchConfiguration('isaac_sim')                         # Isaac Sim模式标志

    # 处理XACRO文件
    # 解析机器人URDF描述，设置轮子里程计话题
    xacro_path = os.path.join(get_package_share_directory('x_bot'), 'urdf', 'x_bot.xacro')
    doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom"})

    # 节点定义
    # 机器人状态发布器（仅在非Isaac Sim模式下启动）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},        # 仿真时间配置
            {'robot_description': doc.toxml()}     # 机器人URDF描述
        ],
        condition=UnlessCondition(isaac_sim)  # 仅在isaac_sim为false时启动
    )

    # RViz可视化节点
    # 使用完整的系统设置配置文件，显示所有传感器和状态信息
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('x_bot'), 'rviz', 'entire_setup.rviz')]
    )

    # 返回启动描述
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'  # 如果为true则使用仿真时间
        ),
        DeclareLaunchArgument(
            'isaac_sim',
            default_value='false',
            description='Set to true when using Isaac Sim'  # 使用Isaac Sim时设为true
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value=doc.toxml(),
            description='Robot description in URDF/XACRO format'  # URDF/XACRO格式的机器人描述
        ),
        # 启动的节点
        robot_state_publisher,  # 机器人状态发布器
        rviz                  # RViz可视化工具
    ])