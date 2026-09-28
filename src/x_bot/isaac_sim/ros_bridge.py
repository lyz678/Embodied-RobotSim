"""ROS 2 publishers, subscribers, and ros2_control wiring for Isaac Sim 6.1."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf, Sdf, Usd, UsdGeom

from robot_importer import find_prim


def _target(path: str | Usd.Prim) -> list[usdrt.Sdf.Path]:
    value = str(path.GetPath()) if isinstance(path, Usd.Prim) else path
    return [usdrt.Sdf.Path(value)]


def create_control_graph(
    stage: Usd.Stage,
    articulation_path: str,
    controller_config: Path,
    publish_odom_tf: bool,
) -> Any:
    """Create clock, base control, odometry, IMU, joint-state and arm-control nodes."""
    keys = og.Controller.Keys
    chassis_path = str(find_prim(stage, "chassis_link").GetPath())

    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BreakLinear", "omni.graph.nodes.BreakVector3"),
        ("BreakAngular", "omni.graph.nodes.BreakVector3"),
        ("FrontDifferential", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("RearDifferential", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("FrontWheels", "isaacsim.core.nodes.IsaacArticulationController"),
        ("RearWheels", "isaacsim.core.nodes.IsaacArticulationController"),
        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
        ("PublishNamespacedJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("ROS2ControlManager", "isaacsim.ros2.control.ROS2ControlManager"),
    ]
    if publish_odom_tf:
        nodes.append(("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))

    connections = [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
        ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
        ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
        ("SubscribeTwist.outputs:execOut", "FrontDifferential.inputs:execIn"),
        ("SubscribeTwist.outputs:execOut", "RearDifferential.inputs:execIn"),
        ("BreakLinear.outputs:x", "FrontDifferential.inputs:linearVelocity"),
        ("BreakLinear.outputs:x", "RearDifferential.inputs:linearVelocity"),
        ("BreakAngular.outputs:z", "FrontDifferential.inputs:angularVelocity"),
        ("BreakAngular.outputs:z", "RearDifferential.inputs:angularVelocity"),
        ("OnPlaybackTick.outputs:tick", "FrontWheels.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "RearWheels.inputs:execIn"),
        ("FrontDifferential.outputs:velocityCommand", "FrontWheels.inputs:velocityCommand"),
        ("RearDifferential.outputs:velocityCommand", "RearWheels.inputs:velocityCommand"),
        ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "PublishImu.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
        ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
        ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
        ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
        ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
        ("ComputeOdometry.outputs:orientation", "PublishImu.inputs:orientation"),
        ("ComputeOdometry.outputs:linearAcceleration", "PublishImu.inputs:linearAcceleration"),
        ("ComputeOdometry.outputs:angularVelocity", "PublishImu.inputs:angularVelocity"),
        ("OnPlaybackTick.outputs:tick", "PublishNamespacedJointState.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishNamespacedJointState.inputs:timeStamp"),
        ("OnPlaybackTick.outputs:tick", "ROS2ControlManager.inputs:execIn"),
    ]
    if publish_odom_tf:
        connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
            ]
        )

    values = [
        ("SubscribeTwist.inputs:topicName", "/x_bot/cmd_vel"),
        ("FrontDifferential.inputs:wheelRadius", 0.06),
        ("FrontDifferential.inputs:wheelDistance", 0.45),
        ("RearDifferential.inputs:wheelRadius", 0.06),
        ("RearDifferential.inputs:wheelDistance", 0.45),
        ("FrontWheels.inputs:jointNames", ["front_left_wheel_joint", "front_right_wheel_joint"]),
        ("RearWheels.inputs:jointNames", ["back_left_wheel_joint", "back_right_wheel_joint"]),
        ("FrontWheels.inputs:targetPrim", _target(articulation_path)),
        ("RearWheels.inputs:targetPrim", _target(articulation_path)),
        ("ComputeOdometry.inputs:chassisPrim", _target(chassis_path)),
        ("PublishOdometry.inputs:topicName", "/x_bot/odom"),
        ("PublishOdometry.inputs:odomFrameId", "odom"),
        ("PublishOdometry.inputs:chassisFrameId", "base_footprint"),
        ("PublishImu.inputs:topicName", "/x_bot/imu"),
        ("PublishImu.inputs:frameId", "imu_frame"),
        ("PublishNamespacedJointState.inputs:topicName", "/x_bot/joint_states"),
        ("PublishNamespacedJointState.inputs:targetPrim", _target(articulation_path)),
        ("ROS2ControlManager.inputs:targetPrim", _target(articulation_path)),
        ("ROS2ControlManager.inputs:controllerConfig", str(controller_config)),
        ("ROS2ControlManager.inputs:namespace", ""),
        ("ROS2ControlManager.inputs:publishRobotDescription", True),
    ]
    if publish_odom_tf:
        values.extend(
            [
                ("PublishOdomTF.inputs:topicName", "/tf"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:childFrameId", "base_footprint"),
            ]
        )

    graph, _, _, _ = og.Controller.edit(
        {"graph_path": "/World/ROS2ControlGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: nodes,
            keys.CONNECT: connections,
            keys.SET_VALUES: values,
        },
    )
    return graph


def _create_camera(stage: Usd.Stage, frame_name: str, local_rotation: tuple[float, float, float]) -> str:
    frame = find_prim(stage, frame_name)
    path = f"{frame.GetPath()}/isaac_camera"
    camera = UsdGeom.Camera.Define(stage, path)
    camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateVerticalApertureAttr(20.955)
    # Match the 1.5184-radian horizontal field of view in the Gazebo sensor.
    camera.CreateFocalLengthAttr(20.955 / (2.0 * math.tan(1.5184 / 2.0)))
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))
    camera.GetPrim().CreateAttribute("exposure:time", Sdf.ValueTypeNames.Float).Set(0.01)
    UsdGeom.XformCommonAPI(camera.GetPrim()).SetRotate(
        local_rotation, UsdGeom.XformCommonAPI.RotationOrderXYZ
    )
    return path


def create_camera_graphs(stage: Usd.Stage) -> list[Any]:
    """Create RGB/RGB-D camera publishers with the Gazebo backend's topic names."""
    graphs = []
    for side in ("left", "right"):
        # A USD camera looks down -Z with +Y up.  The left URDF frame is a ROS
        # optical frame; the right frame retains camera_link axes.
        rotation = (180.0, 0.0, 0.0) if side == "left" else (90.0, 0.0, -90.0)
        camera_path = _create_camera(stage, f"camera_{side}_frame", rotation)
        node_specs = [
            ("OnTick", "omni.graph.action.OnTick"),
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("PublishRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PublishInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ]
        helper_names = ["PublishRgb", "PublishInfo"]
        if side == "left":
            node_specs.extend(
                [
                    ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("PublishPointCloud", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ]
            )
            helper_names.extend(["PublishDepth", "PublishPointCloud"])

        connections = [("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn")]
        for helper in helper_names:
            connections.extend(
                [
                    ("CreateRenderProduct.outputs:execOut", f"{helper}.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", f"{helper}.inputs:renderProductPath"),
                ]
            )

        frame_id = f"camera_{side}_frame"
        values = [
            ("CreateRenderProduct.inputs:cameraPrim", _target(camera_path)),
            ("CreateRenderProduct.inputs:width", 640),
            ("CreateRenderProduct.inputs:height", 640),
            ("PublishRgb.inputs:frameId", frame_id),
            ("PublishRgb.inputs:topicName", f"/x_bot/camera_{side}/image_raw"),
            ("PublishRgb.inputs:type", "rgb"),
            ("PublishInfo.inputs:frameId", frame_id),
            ("PublishInfo.inputs:topicName", f"/x_bot/camera_{side}/camera_info"),
        ]
        if side == "left":
            values.extend(
                [
                    ("PublishDepth.inputs:frameId", frame_id),
                    ("PublishDepth.inputs:topicName", "/x_bot/camera_left/depth/image_raw"),
                    ("PublishDepth.inputs:type", "depth"),
                    ("PublishPointCloud.inputs:frameId", frame_id),
                    ("PublishPointCloud.inputs:topicName", "/x_bot/camera_left/points"),
                    ("PublishPointCloud.inputs:type", "depth_pcl"),
                ]
            )

        graph, _, _, _ = og.Controller.edit(
            {
                "graph_path": f"/World/ROS2Camera{side.title()}Graph",
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                og.Controller.Keys.CREATE_NODES: node_specs,
                og.Controller.Keys.CONNECT: connections,
                og.Controller.Keys.SET_VALUES: values,
            },
        )
        og.Controller.evaluate_sync(graph)
        graphs.append(graph)
    return graphs


def _laser_scan_metadata(prim: Usd.Prim) -> dict[str, float | list[float]]:
    rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError("RTX lidar configuration has no positive scan/firing rate")
    return {
        "horizontalFov": 360.0,
        "horizontalResolution": 360.0 * rotation_rate / firing_rate,
        "depthRange": [near_range, far_range],
        "rotationRate": rotation_rate,
        "azimuthRange": [-180.0, 180.0],
    }


def create_lidar(stage: Usd.Stage) -> Any:
    """Attach an RTX 2-D lidar and publish the existing /x_bot/scan interface."""
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    parent = find_prim(stage, "two_d_lidar")
    lidar = Lidar.create(
        path=f"{parent.GetPath()}/isaac_rtx_lidar",
        config="Example_Rotary_2D",
        tick_rate=10.0,
        translations=[[0.0, 0.0, 0.04]],
    )
    lidar_prim = stage.GetPrimAtPath(lidar.paths[0])
    lidar_prim.GetAttribute("omni:sensor:Core:nearRangeM").Set(0.55)
    lidar_prim.GetAttribute("omni:sensor:Core:farRangeM").Set(16.0)
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        "RtxLidarROS2PublishLaserScan",
        topicName="/x_bot/scan",
        frameId="two_d_lidar",
        **_laser_scan_metadata(lidar_prim),
    )
    return sensor
