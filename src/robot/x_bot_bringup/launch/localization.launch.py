"""Only this launch owns the Isaac localization TF chain; no AMCL/Cartographer."""
import json
import math
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('mode')
    odometry_source = value('odometry_source')
    if odometry_source not in ('lio', 'vio'):
        raise ValueError('odometry_source must be lio or vio')
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    share = Path(get_package_share_directory('x_bot_localization'))
    initial = [float(value(n)) for n in ('initial_x','initial_y','initial_yaw')]
    if not all(math.isfinite(v) for v in initial):
        raise ValueError('Initial pose must be finite')
    common = {'use_sim_time':True}
    nodes = [
        Node(package='x_bot_control', executable='safety_gate', parameters=[common, {'require_vio_health': odometry_source == 'vio',
              'max_forward_speed': .3 if odometry_source == 'vio' else 1.,
              'max_angular_speed': .5 if odometry_source == 'vio' else 1.2}], output='screen'),
    ]
    if odometry_source == 'lio':
        sensor_config = yaml.safe_load((Path(get_package_share_directory('x_bot_sensors'))/
                                        'config/mid360.yaml').read_text())
        rate = sensor_config['publish_hz']
        if isinstance(rate, bool) or rate not in (10, 20, 25, 40):
            raise ValueError('MID360 publish_hz must be 10, 20, 25 or 40')
        nodes.extend([
            Node(package='fast_lio', executable='fastlio_mapping', output='screen',
                 parameters=[str(share/'config/fastlio_mid360.yaml'), {'preprocess.scan_rate': int(rate)}],
                 remappings=[('/tf','/fastlio/tf_internal'), ('/tf_static','/fastlio/tf_static_internal'),
                             ('/Odometry','/fastlio/odometry'), ('/cloud_registered','/fastlio/cloud_registered'),
                             ('/cloud_registered_body','/fastlio/cloud_body'), ('/path','/fastlio/path')]),
            Node(package='x_bot_sensors', executable='bridge', parameters=[common], output='screen'),
            Node(package='x_bot_localization', executable='adapter', parameters=[common], output='screen'),
        ])
    else:
        # Resolve the optional GPU dependency only when VIO is requested.
        get_package_share_directory('isaac_ros_visual_slam')
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/vio.launch.py'))))
    cloud_remappings = [] if odometry_source == 'lio' else [
        ('/fastlio/cloud_odom', '/vio/cloud_odom'),
        ('/fastlio/sensor_odom', '/vio/sensor_odom')]
    if mode == 'mapping':
        nodes.append(Node(package='x_bot_mapping', executable='map_builder', output='screen',
                          remappings=cloud_remappings,
                          parameters=[common, {'initial_pose':initial, 'output_bundle':value('bundle'),
                                               'odometry_source':odometry_source}]))
    else:
        bundle = Path(value('bundle')).expanduser().resolve()
        metadata = json.loads((bundle/'bundle.json').read_text())
        if metadata.get('version') != 1 or metadata.get('frame') != 'map':
            raise ValueError('Unsupported map bundle frame/version')
        for filename in ('map.pcd','map.yaml','map.pgm'):
            if not (bundle/filename).is_file():
                raise ValueError('Missing paired map file: '+filename)
        nodes.extend([
            Node(package='x_bot_localization', executable='map_registration', output='screen',
                 remappings=cloud_remappings,
                 parameters=[common, {'pcd_map':str(bundle/'map.pcd'), 'initial_pose':initial,
                                      'use_initial_pose':value('use_initial_pose') == 'true'}]),
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[common, {'yaml_filename':str(bundle/'map.yaml')}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle',
                 parameters=[common, {'autostart':True, 'node_names':['map_server']}]),
            Node(package='x_bot_mapping', executable='map_activation_guard',
                 parameters=[common], output='screen'),
        ])
    return nodes


def generate_launch_description():
    defaults = {'mode':'mapping', 'odometry_source':'lio', 'bundle':'maps/session', 'initial_x':'0.0', 'initial_y':'0.0',
                'initial_yaw':'0.0', 'use_initial_pose':'true'}
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()], OpaqueFunction(function=setup)])
