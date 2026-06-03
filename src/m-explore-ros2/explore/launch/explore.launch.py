import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Explore Lite 自主探索启动文件
# 功能：启动机器人自主探索节点，实现未知环境的自动探索和地图构建

def generate_launch_description():
    """生成Explore Lite自主探索启动描述"""

    # 创建启动描述对象
    ld = LaunchDescription()

    # 获取探索参数配置文件路径
    # params_costmap.yaml包含探索算法的参数配置
    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params_costmap.yaml"
    )

    # 获取启动参数配置
    use_sim_time = LaunchConfiguration("use_sim_time")  # 是否使用仿真时间
    namespace = LaunchConfiguration("namespace")         # 节点命名空间

    # 声明启动参数，使其可通过命令行配置
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",  # 默认为空，无命名空间
        description="Namespace for the explore node",  # 探索节点的命名空间
    )

    # 话题重映射配置
    # 将全局TF话题重映射为相对话题，支持多机器人场景下的命名空间隔离
    # 这是ROS2多机器人系统的标准做法
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # 自主探索核心节点
    # Explore Lite算法实现机器人自主探索功能
    node = Node(
        package="explore_lite",          # 包名
        name="explore_node",             # 节点名称
        namespace=namespace,             # 命名空间（支持多机器人）
        executable="explore",            # 可执行文件名
        parameters=[
            config,                     # 探索算法参数配置文件
            {"use_sim_time": use_sim_time}  # 仿真时间配置
        ],
        output="screen",                # 输出到屏幕
        remappings=remappings,          # 话题重映射配置
    )

    # 构建启动描述
    ld.add_action(declare_use_sim_time_argument)  # 添加仿真时间参数声明
    ld.add_action(declare_namespace_argument)      # 添加命名空间参数声明
    ld.add_action(node)                           # 添加探索节点

    return ld
