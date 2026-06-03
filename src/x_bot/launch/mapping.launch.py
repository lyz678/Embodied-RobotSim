import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 使用slam_toolbox进行实时同步定位与建图(SLAM)

def generate_launch_description():
    """生成SLAM建图系统启动描述"""

    pkg_slam_toolbox_dir = get_package_share_directory('slam_toolbox')  # SLAM工具箱包路径
    pkg_x = get_package_share_directory('x_bot')                    # x机器人包路径
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')  # 仿真时间配置
    autostart = LaunchConfiguration('autostart', default='True')         # 自动启动配置

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'  # 如果为true则使用Gazebo仿真时钟
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the slam_toolbox stack'  # 自动启动SLAM工具箱
    )

    # 使用在线异步建图模式，支持实时建图和定位
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,  # 仿真时间模式
            'slam_params_file': os.path.join(pkg_x, 'config', 'mapper_params_online_async.yaml'),  # SLAM参数配置文件
        }.items()
    )

    # 创建启动描述并添加所有动作
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_use_sim_time)      # 仿真时间参数
    ld.add_action(declare_autostart)         # 自动启动参数

    # 添加核心启动动作
    ld.add_action(slam_toolbox_launch_cmd)   # SLAM工具箱
    
    return ld

