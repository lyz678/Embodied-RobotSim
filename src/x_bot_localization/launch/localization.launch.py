"""Only this launch owns the Isaac localization TF chain; no AMCL/Cartographer."""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('mode')
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    share = Path(get_package_share_directory('x_bot_localization'))
    initial = [float(value(n)) for n in ('initial_x','initial_y','initial_yaw')]
    if not all(math.isfinite(v) for v in initial):
        raise ValueError('Initial pose must be finite')
    common = {'use_sim_time':True}
    nodes = [
        Node(package='rviz2', executable='rviz2', arguments=['-d',str(share/'rviz/localization.rviz')],
             parameters=[common]),
        Node(package='fast_lio', executable='fastlio_mapping', output='screen',
             parameters=[str(share/'config/fastlio_mid360.yaml')],
             remappings=[('/tf','/fastlio/tf_internal'), ('/tf_static','/fastlio/tf_static_internal'),
                         ('/Odometry','/fastlio/odometry'), ('/cloud_registered','/fastlio/cloud_registered'),
                         ('/cloud_registered_body','/fastlio/cloud_body'), ('/path','/fastlio/path')]),
        *[Node(package='x_bot_localization', executable=name, parameters=[common], output='screen')
          for name in ('bridge','adapter','safety_gate')],
    ]
    if mode == 'mapping':
        nodes.append(Node(package='x_bot_localization', executable='map_builder', output='screen',
                          parameters=[common, {'initial_pose':initial, 'output_bundle':value('bundle')}]))
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
                 parameters=[common, {'pcd_map':str(bundle/'map.pcd'), 'initial_pose':initial,
                                      'use_initial_pose':value('use_initial_pose') == 'true'}]),
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[common, {'yaml_filename':str(bundle/'map.yaml')}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle',
                 parameters=[common, {'autostart':True, 'node_names':['map_server']}]),
        ])
    return nodes


def generate_launch_description():
    defaults = {'mode':'mapping', 'bundle':'maps/session', 'initial_x':'0.0', 'initial_y':'0.0',
                'initial_yaw':'0.0', 'use_initial_pose':'true'}
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()], OpaqueFunction(function=setup)])
