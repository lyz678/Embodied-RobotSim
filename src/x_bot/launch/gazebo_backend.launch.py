"""Gazebo Harmonic backend; public localization/control interfaces match Isaac."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda n: LaunchConfiguration(n).perform(context)
    share = Path(get_package_share_directory("x_bot"))
    urdf = subprocess.check_output(
        [
            "xacro",
            str(share / "urdf/x_bot.xacro"),
            "sim_gz:=true",
            "sim_isaac:=false",
            "mid360_enabled:=true",
            "two_d_lidar_enabled:=false",
            "stereo_camera_enabled:=false",
            "camera_enabled:=true",
        ],
        text=True,
    )
    root = ET.fromstring(urdf)
    physics = yaml.safe_load((share / "config/gazebo_physics.yaml").read_text())
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        if name.startswith("fr3_joint") and joint.find("limit") is not None:
            # Isaac's drive override is 300 Nm despite the source URDF limits.
            # Match that physical actuator envelope for native Gazebo servos.
            joint.find("limit").set("effort", str(physics["arm_max_effort"]))
        if name.startswith("fr3_finger_joint"):
            joint.find("limit").set("effort", str(physics["gripper_max_effort"]))
            dynamics = joint.find("dynamics")
            if dynamics is None:
                dynamics = ET.SubElement(joint, "dynamics")
            dynamics.set("damping", str(physics["finger_joint_damping"]))
            dynamics.set("friction", str(physics["finger_joint_friction"]))
    for finger in ("fr3_leftfinger", "fr3_rightfinger"):
        tag = ET.SubElement(root, "gazebo", reference=finger)
        for name in ("mu1", "mu2"):
            ET.SubElement(tag, name).text = str(physics["finger_friction"])
    # Keep lidar pose and hand telemetry links instead of losing them to fixed-joint lumping.
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed" and (
            joint.find("child").get("link") in ("mid360_link", "fr3_hand")
        ):
            tag = ET.SubElement(root, "gazebo", reference=joint.get("name"))
            ET.SubElement(tag, "preserveFixedJoint").text = "true"
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename", "")
        for scheme in ("package://", "model://"):
            if uri.startswith(scheme):
                package, relative = uri[len(scheme) :].split("/", 1)
                mesh.set(
                    "filename",
                    (Path(get_package_share_directory(package)) / relative).as_uri(),
                )
                break
    urdf = ET.tostring(root, encoding="unicode")
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": urdf}],
        remappings=[],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "x_bot",
            "-x",
            value("x"),
            "-y",
            value("y"),
            "-z",
            "0.0",
            "-Y",
            value("yaw"),
        ],
        output="screen",
    )
    args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/x_bot/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        "/x_bot/camera_left/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/x_bot/camera_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/x_bot/camera_left/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/x_bot/camera_right@sensor_msgs/msg/Image[gz.msgs.Image",
        "/x_bot/camera_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    ]
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=args,
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("/x_bot/joint_states", "/joint_states"),
            ("/x_bot/camera_left/image", "/x_bot/camera_left/image_raw"),
            ("/x_bot/camera_left/depth_image", "/x_bot/camera_left/depth/image_raw"),
            ("/x_bot/camera_right", "/x_bot/camera_right/image_raw"),
        ],
    )
    flags = " -r" + (" -s --headless-rendering" if value("headless") == "true" else "")
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("ros_gz_sim"))
                / "launch/gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": value("world_file") + flags}.items(),
    )
    controllers = [
        Node(
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
        )
    ]
    return [
        gz,
        rsp,
        bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=controllers)),
        spawn,
    ]


def generate_launch_description():
    defaults = {
        "world_file": "",
        "x": "0.0",
        "y": "0.0",
        "yaw": "0.0",
        "headless": "false",
    }
    return LaunchDescription(
        [
            *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
            OpaqueFunction(function=setup),
        ]
    )
