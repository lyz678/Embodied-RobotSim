#!/usr/bin/env python3
"""Load the controllers hosted by Isaac Sim's in-process controller manager."""

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _spawner():
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "fr3_arm_controller",
            "fr3_gripper_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "600",
            "--service-call-timeout",
            "300",
            "--switch-timeout",
            "120",
        ],
        output="screen",
    )


def generate_launch_description():
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare("x_bot"), "urdf", "x_bot.xacro"]),
            " sim_gz:=false sim_gazebo:=false sim_ign:=false sim_isaac:=true",
            " two_d_lidar_enabled:=false camera_enabled:=true stereo_camera_enabled:=false",
            " mid360_xyz:='",
            LaunchConfiguration("mid360_xyz"),
            "'",
            " mid360_rpy:='",
            LaunchConfiguration("mid360_rpy"),
            "'",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mid360_xyz", default_value="0.25 0 0.0875"),
            DeclareLaunchArgument("mid360_rpy", default_value="0 0 0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            robot_description, value_type=str
                        ),
                        "use_sim_time": True,
                    }
                ],
            ),
            # Cold RTX shader compilation can exceed two minutes. A single
            # spawner waits once and avoids the competing spawner lock timeout.
            TimerAction(period=2.0, actions=[_spawner()]),
        ]
    )
