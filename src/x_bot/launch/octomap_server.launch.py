#!/usr/bin/env python3
"""
Launch file for independent Octomap Server with RViz visualization.
This maintains a persistent 3D occupancy map in the global coordinate frame (odom).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def semantic_node(context):
    if LaunchConfiguration('backend').perform(context) != 'semantic_cuda':
        return []
    config = LaunchConfiguration('semantic_config').perform(context)
    if not config:
        config = str(Path(get_package_share_directory('semantic_voxel_mapping'))/'config/map.yaml')
    return [Node(package='semantic_voxel_mapping', executable='semantic_voxel_node', name='semantic_voxel_map',
                 output='screen', parameters=[config, {'use_sim_time':
                    ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])]


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('x_bot'),
        'config',
        'octomap_server.yaml'
    ])
    
    # Path to RViz config
    rviz_config = PathJoinSubstitution([
        FindPackageShare('x_bot'),
        'rviz',
        'octomap.rviz'
    ])
    
    # Octomap Server Node
    octomap_server_node = Node(
        package='octomap_server',
        executable='color_octomap_server_node',
        name='octomap_server',
        condition=LaunchConfigurationEquals('backend', 'legacy'),
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Remap point cloud topic to camera depth points
            # ('cloud_in', '/x_bot/camera_left/depth/points'),
            ('cloud_in', LaunchConfiguration('cloud_topic')),
        ],
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_octomap',
        condition=IfCondition(use_rviz),
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='legacy', choices=['legacy', 'semantic_cuda']),
        DeclareLaunchArgument('semantic_config', default_value=''),
        DeclareLaunchArgument('cloud_topic', default_value='/yoloe_multi_text_prompt/pointcloud_colored',
                              description='Legacy colored PointCloud2 input; semantic_cuda uses its YAML cloud_topic'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        OpaqueFunction(function=semantic_node),
        octomap_server_node,
        rviz_node,
    ])
