#!/usr/bin/env python3
"""Unified CUDA semantic mapping, with optional per-demo parameter overrides."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def semantic_node(context):
    config = LaunchConfiguration('semantic_config').perform(context)
    if not config:
        config = str(Path(get_package_share_directory('semantic_voxel_mapping')) / 'config/map.yaml')
    overrides = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    resolution = LaunchConfiguration('resolution').perform(context)
    topic = LaunchConfiguration('cloud_topic').perform(context)
    palette = LaunchConfiguration('palette_file').perform(context)
    if palette:
        overrides['palette_file'] = palette
    if resolution:
        overrides['resolution'] = float(resolution)
    if topic:
        overrides['cloud_topic'] = topic
    return [Node(package='semantic_voxel_mapping', executable='semantic_voxel_node',
                 output='screen', parameters=[config, overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='semantic_cuda', choices=['semantic_cuda']),
        DeclareLaunchArgument('semantic_config', default_value=''),
        DeclareLaunchArgument('palette_file', default_value=''),
        DeclareLaunchArgument('resolution', default_value='', description='Optional voxel size override in meters'),
        DeclareLaunchArgument('cloud_topic', default_value='', description='Optional semantic cloud topic override'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=semantic_node),
        Node(package='rviz2', executable='rviz2', name='rviz2_octomap',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', str(Path(get_package_share_directory('x_bot')) / 'rviz/octomap.rviz')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
