from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('semantic_voxel_mapping'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(share/'config/map.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='semantic_voxel_mapping', executable='semantic_voxel_node', name='semantic_voxel_map',
             output='screen', parameters=[LaunchConfiguration('config_file'),
                 {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
