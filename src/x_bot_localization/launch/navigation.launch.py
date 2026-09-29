"""Navigation-only launch, map and localization supplied externally."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    here = Path(get_package_share_directory('x_bot_localization'))
    params = str(here/'config/nav2_isaac.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2/'launch/navigation_launch.py')),
            launch_arguments={'use_sim_time':'true','autostart':'true','params_file':params}.items()),
    ])
