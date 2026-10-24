#!/usr/bin/env python3
"""Unified CUDA semantic mapping, with optional per-demo parameter overrides."""
from pathlib import Path
import yaml
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
    scope=LaunchConfiguration('map_scope').perform(context)
    with open(config) as f:settings=yaml.safe_load(f)['semantic_voxel_map']['ros__parameters']
    overrides = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    resolution = LaunchConfiguration('resolution').perform(context)
    topic = LaunchConfiguration('cloud_topic').perform(context)
    palette = LaunchConfiguration('palette_file').perform(context)
    map_file = LaunchConfiguration('map_file').perform(context)
    enabled = LaunchConfiguration('enabled_on_start').perform(context)
    if enabled:
        overrides['enabled_on_start'] = enabled == 'true'
    if map_file:
        overrides['map_file'] = map_file
    if palette:
        overrides['palette_file'] = palette
    if resolution:
        overrides['resolution'] = float(resolution)
    if topic:
        overrides['cloud_topic'] = topic
    remaps=[]
    if scope!='legacy':
        for source,suffix in (('/semantic_map/voxels','voxels'),('/semantic_map/diagnostics','diagnostics'),('/occupied_cells_vis_array','occupied_cells_vis_array'),('/octomap_full','octomap_full'),('/projected_map','projected_map')):
            remaps.append((source,f'/{scope}_map/{suffix}'))
    if scope=='global':
        overrides['legacy_visualization']=True
        overrides['publish_moveit_scene']=False
        remaps.append(('/legacy_occupied_cells_vis_array','/occupied_cells_vis_array'))
    return [Node(package='semantic_voxel_mapping', executable='semantic_voxel_node',
                 name='semantic_voxel_map',namespace='' if scope=='legacy' else f'{scope}_map',
                 output='screen', parameters=[settings, overrides],remappings=remaps)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_scope', default_value='legacy',choices=['legacy','local','global']),
        DeclareLaunchArgument('backend', default_value='semantic_cuda', choices=['semantic_cuda']),
        DeclareLaunchArgument('semantic_config', default_value=''),
        DeclareLaunchArgument('palette_file', default_value=''),
        DeclareLaunchArgument('map_file', default_value='', description='Optional semantic map save path'),
        DeclareLaunchArgument('enabled_on_start', default_value='', choices=['','true','false']),
        DeclareLaunchArgument('resolution', default_value='', description='Optional voxel size override in meters'),
        DeclareLaunchArgument('cloud_topic', default_value='', description='Optional semantic cloud topic override'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=semantic_node),
        Node(package='rviz2', executable='rviz2', name='rviz2_octomap',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', str(Path(get_package_share_directory('x_bot_navigation')) / 'rviz/octomap.rviz')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
