"""Navigation-only launch, map and localization supplied externally."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    here = Path(get_package_share_directory('x_bot_navigation'))
    params = RewrittenYaml(
        source_file=str(here/'config/nav2.yaml'),
        param_rewrites={
            'default_nav_to_pose_bt_xml': str(here/'behavior_trees/navigate_to_pose.xml'),
            'default_nav_through_poses_bt_xml': str(here/'behavior_trees/navigate_through_poses.xml'),
        },
        convert_types=True,
    )
    return LaunchDescription([
        Node(package='x_bot_control', executable='contact_scan',
             parameters=[{'use_sim_time': True}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2/'launch/navigation_launch.py')),
            launch_arguments={'use_sim_time':'true','autostart':'true','params_file':params}.items()),
    ])
