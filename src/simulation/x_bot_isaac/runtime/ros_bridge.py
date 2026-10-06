"""ROS 2 publishers, subscribers, and ros2_control wiring for Isaac Sim 6.1."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf, Sdf, Usd, UsdGeom

from robot_importer import ROBOT_PRIM_PATH, find_prim
from x_bot_control.base_motion import WHEEL_NAMES


def _target(path: str | Usd.Prim) -> list[usdrt.Sdf.Path]:
    value = str(path.GetPath()) if isinstance(path, Usd.Prim) else path
    return [usdrt.Sdf.Path(value)]


def create_control_graph(
    stage: Usd.Stage,
    articulation_path: str,
    controller_config: Path,
    source_urdf: Path,
) -> Any:
    """Create clock, base control, debug odometry, joint-state and arm-control nodes."""
    # Isaac 6.1's live USD exporter mutates collider prims and omits the arm
    # branch across fixed frame-only mount links. Reuse our generated, connected
    # URDF for kinematics and synthesize hardware interfaces from the live USD.
    from isaacsim.ros2.control import urdf_synth

    original_builder = urdf_synth.build_full_urdf
    def build_control_urdf(stage, target_prim_path, sensor_overlay_urdf_path=None):
        if target_prim_path != ROBOT_PRIM_PATH:
            return original_builder(stage, target_prim_path, sensor_overlay_urdf_path)
        roots = urdf_synth.discover_articulations(stage, target_prim_path)
        if len(roots) != 1:
            raise RuntimeError(f"Expected one x_bot articulation, found {len(roots)}")
        root = ET.parse(source_urdf).getroot()
        block = urdf_synth.synthesize_hardware_block(
            stage, roots[0], stage.GetPrimAtPath(target_prim_path),
            mimic_map=urdf_synth._extract_mimic_map(root),
        )
        names = {joint.get("name") for joint in root.findall("joint")}
        missing = {joint.name for joint in block.joints} - names
        if missing:
            raise RuntimeError(f"USD control joints missing from source URDF: {sorted(missing)}")
        urdf_synth._fix_joint_limits(root)
        root.append(urdf_synth.hardware_block_to_xml(block))
        return ET.tostring(root, encoding="unicode")

    # setup() imports the builder into its own module namespace.
    from isaacsim.ros2.control import ros2_control_manager
    ros2_control_manager.build_full_urdf = build_control_urdf
    keys = og.Controller.Keys
    chassis_path = str(find_prim(stage, "chassis_link").GetPath())

    nodes = [
        ("BaseWheels", "isaacsim.core.nodes.IsaacArticulationController"),
        ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PublishNamespacedJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("ROS2ControlManager", "isaacsim.ros2.control.ROS2ControlManager"),
    ]

    connections = [
        ("OnPhysicsStep.outputs:step", "BaseWheels.inputs:execIn"),
        ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("OnPhysicsStep.outputs:step", "ComputeOdometry.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
        ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
        ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
        ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
        ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
        ("OnPhysicsStep.outputs:step", "PublishNamespacedJointState.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishNamespacedJointState.inputs:timeStamp"),
        ("OnPhysicsStep.outputs:step", "ROS2ControlManager.inputs:execIn"),
    ]

    values = [
        ("BaseWheels.inputs:jointNames", list(WHEEL_NAMES)),
        ("BaseWheels.inputs:targetPrim", _target(articulation_path)),
        ("BaseWheels.inputs:velocityCommand", [0.0] * 4),
        ("ReadSimTime.inputs:resetOnStop", True),
        ("ComputeOdometry.inputs:chassisPrim", _target(chassis_path)),
        ("PublishOdometry.inputs:topicName", "/debug/ground_truth/odom"),
        ("PublishOdometry.inputs:odomFrameId", "sim_world"),
        ("PublishOdometry.inputs:chassisFrameId", "chassis_link"),
        ("PublishNamespacedJointState.inputs:topicName", "/x_bot/joint_states"),
        ("PublishNamespacedJointState.inputs:targetPrim", _target(articulation_path)),
        ("ROS2ControlManager.inputs:targetPrim", _target(ROBOT_PRIM_PATH)),
        ("ROS2ControlManager.inputs:controllerConfig", str(controller_config)),
        ("ROS2ControlManager.inputs:namespace", ""),
        ("ROS2ControlManager.inputs:publishRobotDescription", True),
    ]

    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": "/World/ROS2ControlGraph",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
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
