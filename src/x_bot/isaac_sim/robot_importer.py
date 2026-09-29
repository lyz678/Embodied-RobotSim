"""Generate an Isaac-specific URDF and import it into the current USD stage."""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path
from pxr import Gf, Usd, UsdGeom, UsdPhysics


ROBOT_PRIM_PATH = "/World/x_bot"
WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "back_left_wheel_joint",
    "front_right_wheel_joint",
    "back_right_wheel_joint",
]
ARM_JOINTS = [f"fr3_joint{index}" for index in range(1, 8)]
GRIPPER_JOINTS = ["fr3_finger_joint1", "fr3_finger_joint2"]
INITIAL_JOINT_POSITIONS = [0.0, -1.5, 0.0, -2.3561, 0.0, 2.0, 0.7853, 0.04, 0.04]


def generate_urdf(package_share: Path, mid360_xyz: str = '0.25 0 0.0875', mid360_rpy: str = '0 0 0') -> tuple[Path, Path]:
    """Expand x_bot.xacro into a temporary URDF.

    Returns the URDF and its owning temporary directory.  The caller keeps the
    directory alive for as long as the imported USD is referenced.
    """
    xacro = shutil.which("xacro")
    if not xacro:
        raise RuntimeError("xacro was not found; source ROS 2 Jazzy and the built workspace before Isaac Sim")
    temp_dir = Path(tempfile.mkdtemp(prefix="x_bot_isaac_"))
    urdf_path = temp_dir / "x_bot.urdf"
    command = [
        xacro,
        str(package_share / "urdf" / "x_bot.xacro"),
        "sim_gz:=false",
        "sim_gazebo:=false",
        "sim_ign:=false",
        "sim_isaac:=true",
        f"mid360_xyz:={mid360_xyz}",
        f"mid360_rpy:={mid360_rpy}",
        "two_d_lidar_enabled:=false",
        "camera_enabled:=true",
        "stereo_camera_enabled:=false",
        "odometry_source:=world",
    ]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed ({result.returncode}):\n{result.stderr.strip()}")
    urdf_path.write_text(result.stdout, encoding="utf-8")
    return urdf_path, temp_dir


def import_robot_asset(urdf_path: Path, output_dir: Path, package_paths: dict[str, Path]) -> Path:
    """Convert the generated URDF with the Isaac Sim 6.1 importer."""
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

    # URDFImporterConfig is a pybind configuration object in Isaac Sim 6.1;
    # assign its fields explicitly instead of relying on keyword construction.
    config = URDFImporterConfig()
    config.urdf_path = str(urdf_path)
    config.usd_path = str(output_dir / "usd")
    config.merge_fixed_joints = False
    config.merge_mesh = False
    config.collision_from_visuals = False
    config.allow_self_collision = False
    config.fix_base = False
    config.link_density = 1000.0
    config.joint_drive_type = "force"
    config.joint_target_type = "position"
    config.override_joint_stiffness = 400.0
    config.override_joint_damping = 40.0
    config.robot_type = "Default"
    config.run_asset_transformer = True
    config.run_multi_physics_conversion = True
    config.ros_package_paths = [
        {"name": name, "path": str(path)} for name, path in package_paths.items()
    ]
    output = URDFImporter(config).import_urdf()
    if not output:
        raise RuntimeError("Isaac Sim URDF importer did not return an output USD path")
    return Path(str(output))


def add_robot_reference(stage: Usd.Stage, usd_path: Path, x: float, y: float, yaw: float) -> Usd.Prim:
    robot = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    robot.GetReferences().AddReference(str(usd_path))
    xform = UsdGeom.XformCommonAPI(robot)
    xform.SetTranslate(Gf.Vec3d(x, y, 0.15))
    xform.SetRotate((0.0, 0.0, yaw * 180.0 / 3.141592653589793), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    return robot


def find_prim(stage: Usd.Stage, name: str) -> Usd.Prim:
    """Find an imported link or joint by basename, independent of asset layout."""
    matches = [prim for prim in stage.Traverse() if prim.GetName() == name]
    if not matches:
        raise RuntimeError(f"Imported x_bot asset has no prim named '{name}'")
    # Prefer the prim below the robot reference when an importer helper prim has
    # the same name elsewhere in the stage.
    matches.sort(key=lambda prim: (not str(prim.GetPath()).startswith(ROBOT_PRIM_PATH), len(str(prim.GetPath()))))
    return matches[0]


def find_articulation_root(stage: Usd.Stage) -> Usd.Prim:
    robot = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    for prim in Usd.PrimRange(robot):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    raise RuntimeError("Imported x_bot asset contains no ArticulationRootAPI prim")


def configure_joint_drives(stage: Usd.Stage) -> None:
    """Use velocity drives for wheels and position drives for arm joints."""
    for name in WHEEL_JOINTS:
        joint = find_prim(stage, name)
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(80.0)
        drive.CreateMaxForceAttr(20000.0)
    for name in ARM_JOINTS:
        joint = find_prim(stage, name)
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateTypeAttr("force")
        joint_number = int(name.removeprefix("fr3_joint"))
        drive.CreateStiffnessAttr(600.0 if joint_number <= 4 else 300.0)
        drive.CreateDampingAttr(40.0)
        drive.CreateMaxForceAttr(300.0)
    for name in GRIPPER_JOINTS:
        joint = find_prim(stage, name)
        drive = UsdPhysics.DriveAPI.Apply(joint, "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(300.0)
        drive.CreateDampingAttr(20.0)
        drive.CreateMaxForceAttr(80.0)


def set_initial_joint_state(articulation_path: str) -> None:
    """Place the FR3 in the same initial configuration as the Gazebo backend."""
    from isaacsim.core.experimental.prims import Articulation

    articulation = Articulation(articulation_path)
    names = ARM_JOINTS + GRIPPER_JOINTS
    indices = articulation.get_dof_indices(names).numpy().flatten().tolist()
    articulation.set_dof_positions(INITIAL_JOINT_POSITIONS, dof_indices=indices)
    articulation.set_dof_position_targets(INITIAL_JOINT_POSITIONS, dof_indices=indices)


def cleanup_temp_dir(path: Path) -> None:
    shutil.rmtree(path, ignore_errors=True)
