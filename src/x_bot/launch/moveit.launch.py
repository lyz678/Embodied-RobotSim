from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Load URDF via xacro
    robot_description_content = os.popen("xacro " + 
        os.path.join(get_package_share_directory("x_bot"), "urdf", "x_bot.xacro") +
        " sim_gz:=true").read()
    robot_description = {"robot_description": robot_description_content}

    # Load SRDF
    robot_description_semantic_config = load_yaml("franka_fr3_moveit_config", "config/fr3.srdf") # This helper seems to force yaml load. Let's just read file.
    
    srdf_file = os.path.join(get_package_share_directory("franka_fr3_moveit_config"), "config", "fr3.srdf")
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # Kinematics
    kinematics_yaml = load_yaml("franka_fr3_moveit_config", "config/kinematics.yaml")

    # MoveIt Controllers
    # We need to remap the controller names to what we have in Gazebo (arm_controller)
    moveit_controllers = {
        "controller_names": ["arm_controller", "gripper_action_controller"],
        "arm_controller": {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": [
                "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                "fr3_joint5", "fr3_joint6", "fr3_joint7"
            ]
        },
        "gripper_action_controller": {
            "action_ns": "gripper_action",
            "type": "GripperCommand",
            "default": True,
            "joints": [
                "fr3_finger_joint1", "fr3_finger_joint2"
            ]
        }
    }

    # Joint Limits
    robot_description_planning = {
        "robot_description_planning": load_yaml("franka_fr3_moveit_config", "config/fr3_joint_limits.yaml")
    }

    # Planning Pipelines
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        # Map panda_arm groups to fr3 groups if they exist
        if "panda_arm" in ompl_planning_yaml:
            ompl_planning_yaml["fr3_arm"] = ompl_planning_yaml["panda_arm"]
        if "panda_arm_hand" in ompl_planning_yaml:
            ompl_planning_yaml["hand"] = ompl_planning_yaml["panda_arm_hand"]
        
        # Merge loaded yaml into base config
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Kinematics config wrapper
    kinematics_config = {"robot_description_kinematics": kinematics_yaml}

    # Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_config,
            ompl_planning_pipeline_config,
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"moveit_simple_controller_manager": moveit_controllers},
            {"use_sim_time": True},
        ],
    )
    
    # RViz
    rviz_config_file = os.path.join(get_package_share_directory("franka_fr3_moveit_config"), "rviz", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_config,
            ompl_planning_pipeline_config,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        run_move_group_node,
        rviz_node
    ])
