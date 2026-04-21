#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from ament_index_python.packages import get_package_share_directory

# x机器人Gazebo生成和ROS桥接启动文件
# 功能：将机器人模型生成到Gazebo仿真环境中，并建立ROS2与Gazebo的通信桥接

def get_xacro_to_doc(xacro_file_path, mappings):
    """将XACRO文件解析为URDF文档对象"""
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    """生成x机器人Gazebo生成和桥接的启动描述"""

    # 获取x机器人包路径
    x_bot_path = get_package_share_directory("x_bot")

    # 获取启动参数配置
    position_x = LaunchConfiguration("position_x")  # 机器人初始X位置
    position_y = LaunchConfiguration("position_y")  # 机器人初始Y位置
    orientation_yaw = LaunchConfiguration("orientation_yaw")  # 机器人初始朝向

    # 传感器使能配置
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)  # 2D激光雷达使能
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)            # 相机使能
    odometry_source = LaunchConfiguration("odometry_source")  # 里程计源配置
    world_name = LaunchConfiguration("world_name")  # 世界名称配置


    # 机器人状态发布器节点
    # 功能：解析XACRO文件生成URDF，并发布机器人状态信息
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': ParameterValue(
                    Command(  # 动态解析XACRO文件为URDF
                        ['xacro ', join(x_bot_path, 'urdf/x_bot.xacro'),
                         ' two_d_lidar_enabled:=', two_d_lidar_enabled,      # 激光雷达参数
                         ' camera_enabled:=', camera_enabled,                # 相机参数
                         ' odometry_source:=', odometry_source,       # 里程计源参数
                         ' sim_gz:=', "true"                          # 启用Gazebo仿真模式
                        ]),
                    value_type=str
                )
            }
        ],
        remappings=[
            ('/joint_states', 'x_bot/joint_states'),
        ]
    )

    # Load initial joint positions from YAML
    initial_positions_file = join(x_bot_path, "config", "initial_positions.yaml")
    initial_positions = {}
    with open(initial_positions_file, 'r') as f:
        initial_positions = yaml.safe_load(f)["initial_positions"]

    # Construct spawn arguments
    spawn_args = [
        "-topic", "/robot_description",    # 从此话题读取机器人URDF描述
        "-name", "x_bot",                # 在Gazebo中的模型名称
        "-allow_renaming", "true",         # 允许重命名（处理名称冲突）
        "-z", "0.15",                      # Z轴高度偏移（机器人底座高度）
        "-x", position_x,                  # X轴初始位置
        "-y", position_y,                  # Y轴初始位置
        "-Y", orientation_yaw,             # 偏航角初始朝向
    ]

    # Append joint initial positions
    for joint_name, value in initial_positions.items():
        spawn_args.extend(["-J", joint_name, str(value)])


    # Gazebo实体生成节点
    # 功能：从robot_description话题读取URDF并在Gazebo中生成机器人模型
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=spawn_args
    )

    # ROS2-Gazebo桥接节点
    # 功能：建立ROS2与Gazebo之间的消息通信桥接
    # 将Gazebo的消息转换为ROS2格式，反之亦然
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # 运动控制命令桥接：ROS2 Twist -> Gazebo Twist
            "/x_bot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",

            # 时钟同步桥接：Gazebo时钟 -> ROS2时钟
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # 里程计桥接：Gazebo里程计 -> ROS2里程计
            "/gazebo/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            # TF变换桥接：Gazebo位姿 -> ROS2 TF消息
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

            # 激光雷达数据桥接：Gazebo激光扫描 -> ROS2激光扫描
            "/x_bot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            # IMU传感器桥接：Gazebo IMU -> ROS2 IMU
            "/x_bot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",

            # 关节状态桥接：Gazebo模型关节状态 -> ROS2关节状态
            # 直接桥接namespaced的关节状态话题
            "/x_bot/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",

            # 双目相机桥接 (Left, Right)
            "/x_bot/camera_left/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/x_bot/camera_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/x_bot/camera_left/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/x_bot/camera_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/x_bot/camera_right@sensor_msgs/msg/Image[gz.msgs.Image",
            "/x_bot/camera_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[
            ('/x_bot/camera_left/image', '/x_bot/camera_left/image_raw'),
            ('/x_bot/camera_left/depth_image', '/x_bot/camera_left/depth/image_raw'),
            ('/x_bot/camera_right', '/x_bot/camera_right/image_raw'),
        ]
    )

    # Spawn Joint State Broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )

    # Spawn Arm Controller
    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_arm_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # Spawn Gripper Controller
    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_gripper_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # 返回启动描述
    return LaunchDescription([
        # 声明所有启动参数，使其可通过命令行配置
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        DeclareLaunchArgument("world_name", default_value="ware_house"),

        # 启动的核心节点
        robot_state_publisher,      # 机器人状态发布器
        gz_spawn_entity,            # Gazebo实体生成器
        gz_ros2_bridge,             # ROS-Gazebo通信桥接
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller
    ])