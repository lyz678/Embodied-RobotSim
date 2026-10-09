"""Navigation-only launch, map and localization supplied externally."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def setup(context):
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    here = Path(get_package_share_directory('x_bot_navigation'))
    source = LaunchConfiguration('odometry_source').perform(context)
    if source not in ('lio', 'vio'):
        raise ValueError('odometry_source must be lio or vio')
    config = yaml.safe_load((here/'config/nav2.yaml').read_text())
    if source == 'vio':
        # The hand-mounted stereo rig needs gentler motion than the roof lidar.
        config['controller_server']['ros__parameters']['FollowPath'].update({
            'v_linear_max': .3, 'v_angular_max': .5, 'vx_max': .3, 'wz_max': .5,
            'rotate_to_heading_angular_vel': .5, 'max_angular_accel': .5})
        config['velocity_smoother']['ros__parameters'].update({
            'max_velocity': [.3, 0., .5], 'min_velocity': [-.2, 0., -.5],
            'max_accel': [.3, 0., .5], 'max_decel': [-.6, 0., -.75]})
    with tempfile.NamedTemporaryFile(mode='w', prefix='x_bot_nav2_', suffix='.yaml', delete=False) as file:
        yaml.safe_dump(config, file)
        config_file = file.name
    params = RewrittenYaml(
        source_file=config_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': str(here/'behavior_trees/navigate_to_pose.xml'),
            'default_nav_through_poses_bt_xml': str(here/'behavior_trees/navigate_through_poses.xml'),
        },
        convert_types=True,
    )
    return [
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            EnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                                default_value=str(here/'config/fastdds_services.xml'))),
        Node(package='x_bot_control', executable='contact_scan',
             parameters=[{'use_sim_time': True}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2/'launch/navigation_launch.py')),
            launch_arguments={'use_sim_time':'true','autostart':'true','params_file':params}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odometry_source', default_value='lio'),
        OpaqueFunction(function=setup),
    ])
