import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Nav2导航系统启动文件
# 功能：启动ROS2 Navigation2导航栈，包括地图服务器、定位、路径规划等组件
# 支持两种模式：
#   1. SLAM 模式（use_slam:=true）：用于自动探索建图
#   2. 静态地图模式（use_slam:=false）：用于已知地图导航

def launch_nav2(context, *args, **kwargs):
    """根据参数动态启动 Nav2"""
    
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_x = get_package_share_directory('x_bot')
    
    # 获取参数值
    use_sim_time_value = context.launch_configurations['use_sim_time']
    autostart_value = context.launch_configurations['autostart']
    use_slam_value = context.launch_configurations['use_slam']
    map_file_value = context.launch_configurations['map_file']
    
    # 根据 use_slam 参数构建 launch_arguments
    if use_slam_value.lower() == 'true':
        # SLAM 模式：传递 slam=True，不传递 map
        launch_args = [
            ('use_sim_time', use_sim_time_value),
            ('autostart', autostart_value),
            ('slam', 'True'),
            ('slam_params_file', os.path.join(pkg_x, 'config', 'mapper_params_online_async.yaml')),
            ('params_file', os.path.join(pkg_x, 'config', 'nav2_params.yaml')),
        ]
    else:
        # 静态地图模式：传递 map，不传递 slam
        launch_args = [
            ('use_sim_time', use_sim_time_value),
            ('autostart', autostart_value),
            ('map', map_file_value),
            ('params_file', os.path.join(pkg_x, 'config', 'nav2_params.yaml')),
        ]
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments=launch_args
    )
    
    return [nav2_launch]

def generate_launch_description():
    """生成Nav2导航系统启动描述"""

    # 获取包路径
    pkg_x = get_package_share_directory('x_bot')

    # 声明启动参数
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Whether to run SLAM (true) or use a pre-built map (false)'
    )

    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_x, 'config', 'ware_house_map.yaml'),
        description='Full path to map yaml file to load (only used when use_slam=false)'
    )

    # 使用 OpaqueFunction 动态构建 Nav2 启动
    nav2_launch_cmd = OpaqueFunction(function=launch_nav2)

    # RViz可视化工具启动命令
    # 用于可视化导航过程中的地图、路径规划、传感器数据等
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{'use_sim_time': True}],  # 使用仿真时间
        arguments=[
            '-d' + os.path.join(  # 指定RViz配置文件
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )
        ]
    )

    # 地图服务器节点（已定义但未在启动描述中使用）
    # 功能：加载和提供静态地图数据
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(pkg_x, 'config', 'ware_house_map.yaml')  # 仓库地图文件
        }],
    )

    # 静态坐标变换发布器：map -> odom
    # 功能：建立地图坐标系与里程计坐标系之间的固定变换关系
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0',    # X, Y, Z偏移（均为0）
                   '0', '0', '0',    # 偏航角、俯仰角、翻滚角（均为0）
                   'map', 'odom']    # 父坐标系->子坐标系
    )

    # 里程计话题中继：/x_bot/odom → /odom
    # Gazebo diff_drive 插件发布在 /x_bot/odom，AMCL/Nav2 订阅 /odom
    odom_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        output='screen',
        arguments=['/x_bot/odom', '/odom'],
        parameters=[{'use_sim_time': True}]
    )

    # 话题重映射节点（已定义但未在启动描述中使用）
    # 功能：处理x机器人特有的话题重映射需求
    remapper_node = Node(
        package='x_bot',
        executable='remapper.py',
        name='remapper',
        output='screen',
    )

    # 创建启动描述对象
    ld = LaunchDescription()

    # 添加参数声明（必须先声明才能通过命令行传递）
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_autostart_arg)
    ld.add_action(declare_use_slam_arg)
    ld.add_action(declare_map_file_arg)

    # 添加核心启动动作
    ld.add_action(nav2_launch_cmd)               # Nav2导航栈
    ld.add_action(rviz_launch_cmd)               # RViz可视化工具
    ld.add_action(odom_relay_node)               # odom 话题中继（/x_bot/odom → /odom）

    # 注意：map_server_node和remapper_node已定义但未添加到启动描述中

    return ld
