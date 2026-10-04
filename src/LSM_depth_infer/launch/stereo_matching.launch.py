"""Launch LSM stereo depth with YAML and startup ROS parameter overrides."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    value = lambda name: context.launch_configurations[name]
    params = [{'config_file': value('config_file'), 'use_sim_time': value('use_sim_time') == 'true'}]
    if value('params_file'):
        params.append(value('params_file'))
    nodes = [Node(package='stereo_matching', executable='stereo_matching_node',
                  name='stereo_matching_node', output='screen', parameters=params)]
    if value('use_rviz') == 'true':
        share = Path(get_package_share_directory('stereo_matching'))
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(share/'launch/pointcloud.rviz')],
                          parameters=[{'use_sim_time': value('use_sim_time') == 'true'}]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('stereo_matching'))
    defaults = {'config_file': str(share/'config/config.yaml'), 'params_file': '',
                'use_rviz': 'false', 'use_sim_time': 'true'}
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
                              OpaqueFunction(function=setup)])
