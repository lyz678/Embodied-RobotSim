"""Isaac ROS 4.x stereo-inertial odometry, with no visual SLAM map/TF owner."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = Path(get_package_share_directory('x_bot_localization'))
    common = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_adapter', default_value='true',
                              description='Publish navigation odometry/cloud/TF from VIO'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='vio_right_optical_tf', parameters=[common], arguments=[
                 '--frame-id', 'camera_right_frame', '--child-frame-id', 'camera_right_optical_frame',
                 '--roll', '-1.5707963267948966', '--pitch', '0', '--yaw', '-1.5707963267948966']),
        Node(package='x_bot_localization', executable='vio_stereo', parameters=[common], output='screen'),
        ComposableNodeContainer(
            name='vio_container', namespace='', package='rclcpp_components',
            executable='component_container', output='screen',
            composable_node_descriptions=[ComposableNode(
                name='visual_slam_node', package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                parameters=[str(share/'config/vio.yaml'), common],
                remappings=[('visual_slam/image_0', '/vio/image_0'),
                            ('visual_slam/image_1', '/vio/image_1'),
                            ('visual_slam/camera_info_0', '/vio/camera_info_0'),
                            ('visual_slam/camera_info_1', '/vio/camera_info_1'),
                            ('visual_slam/imu', '/livox/imu'),
                            ('/diagnostics', '/vio/diagnostics'),
                            ('visual_slam/tracking/odometry', '/vio/odometry')])]),
        Node(package='x_bot_localization', executable='vio_adapter',
             parameters=[common, {'depth_stride': 8}], output='screen',
             remappings=[('/vio/cloud_odom','/vio/cloud_unfiltered'),
                         ('/x_bot/scan','/vio/scan_unfiltered')],
             condition=IfCondition(LaunchConfiguration('publish_adapter'))),
        Node(package='x_bot_localization', executable='vio_cloud_filter',
             parameters=[common], output='screen',
             condition=IfCondition(LaunchConfiguration('publish_adapter'))),
    ])
