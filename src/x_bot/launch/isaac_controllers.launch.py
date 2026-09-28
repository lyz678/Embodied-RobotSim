#!/usr/bin/env python3
"""Load the controllers hosted by Isaac Sim's in-process controller manager."""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawner(controller_name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
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
            " two_d_lidar_enabled:=true camera_enabled:=true stereo_camera_enabled:=false",
        ]
    )
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            ),
            TimerAction(period=2.0, actions=[_spawner("joint_state_broadcaster")]),
            TimerAction(period=3.0, actions=[_spawner("fr3_arm_controller")]),
            TimerAction(period=4.0, actions=[_spawner("fr3_gripper_controller")]),
        ]
    )
