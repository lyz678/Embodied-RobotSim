#!/usr/bin/env python3
"""
Launch file for independent Octomap Server with RViz visualization.
This maintains a persistent 3D occupancy map in the global coordinate frame (odom).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Remap point cloud topic to camera depth points
            # ('cloud_in', '/x_bot/camera_left/depth/points'),
            ('cloud_in', '/yoloe_multi_text_prompt/pointcloud_colored'),
        ],
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_octomap',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
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
        octomap_server_node,
        rviz_node,
    ])
