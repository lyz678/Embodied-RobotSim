#!/usr/bin/env python3
"""
Launch MoveIt 2 for the x_bot robotic arm.

Based on mycobot_moveit_config launch file structure.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Generate a launch description for MoveIt 2 with x_bot robot."""
    
    package_name = 'x_bot'

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Get the package share directory
    pkg_share_temp = FindPackageShare(package=package_name)

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Whether to start RViz')

    def configure_setup(context):
        """Configure MoveIt and create nodes with proper string conversions."""
        
        # Get package path
        pkg_share = pkg_share_temp.find(package_name)

        # Define all config file paths
        config_path = os.path.join(pkg_share, 'config')
        initial_positions_file_path = os.path.join(config_path, 'initial_positions.yaml')
        
        # Use Franka FR3 official joint limits configuration
        fr3_moveit_config_pkg = FindPackageShare(package='franka_fr3_moveit_config').find('franka_fr3_moveit_config')
        joint_limits_file_path = os.path.join(fr3_moveit_config_pkg, 'config', 'fr3_joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model_path = os.path.join(config_path, 'x_bot.srdf')
        pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

        # Create MoveIt configuration using MoveItConfigsBuilder
        moveit_config = (
            MoveItConfigsBuilder("x_bot", package_name=package_name)
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=["ompl"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )

        # MoveIt capabilities
        move_group_capabilities = {
            "capabilities": "move_group/ExecuteTaskSolutionCapability move_group/MoveGroupGetPlanningSceneService move_group/ClearOctomapService move_group/MoveGroupCartesianPathService move_group/MoveGroupKinematicsService move_group/MoveGroupMoveAction MoveGroupPlanService move_group/MoveGroupQueryPlannersService move_group/MoveGroupStateValidationService"
        }

        # Manually load joint limits to ensure acceleration limits are applied
        with open(joint_limits_file_path, 'r') as file:
            joint_limits_content = yaml.safe_load(file)
        
        moveit_config_dict = moveit_config.to_dict()
        if 'robot_description_planning' not in moveit_config_dict:
            moveit_config_dict['robot_description_planning'] = {}
        moveit_config_dict['robot_description_planning']['joint_limits'] = joint_limits_content['joint_limits']

        # Create move_group node
        start_move_group_node_cmd = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config_dict,
                os.path.join(config_path, 'sensors_3d_x_bot.yaml'),
                {'use_sim_time': use_sim_time},
                {'start_state': {'content': initial_positions_file_path}},
                move_group_capabilities,
            ],
            # Suppress noisy shape_mask and planning_scene_monitor errors
            ros_arguments=[
                '--log-level', 'move_group.moveit.moveit.ros.shape_mask:=fatal',
                '--log-level', 'move_group.moveit.moveit.ros.planning_scene_monitor:=error',
            ],
        )

        # Create RViz node (optional)
        rviz_config_file = os.path.join(pkg_share, 'rviz', 'moveit.rviz')
        
        nodes = [start_move_group_node_cmd]
        
        if os.path.exists(rviz_config_file):
            start_rviz_node_cmd = Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    os.path.join(config_path, 'sensors_3d_x_bot.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
            )
            
            exit_event_handler = RegisterEventHandler(
                condition=IfCondition(use_rviz),
                event_handler=OnProcessExit(
                    target_action=start_rviz_node_cmd,
                    on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
                ),
            )
            
            nodes.extend([start_rviz_node_cmd, exit_event_handler])

        return nodes

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add the setup and node creation
    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld
