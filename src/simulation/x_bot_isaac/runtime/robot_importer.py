"""Generate an Isaac-specific URDF and import it into the current USD stage."""

from __future__ import annotations

import math
import shutil
import subprocess
import tempfile
from pathlib import Path
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema


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


def add_robot_reference(stage: Usd.Stage, usd_path: Path, x: float, y: float, yaw: float, z: float = 0.0) -> Usd.Prim:
    robot = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    robot.GetReferences().AddReference(str(usd_path))
    # Isaac Sim 6.1's asset transformer separates physics into variants and
    # leaves the variant unselected in the generated asset interface.
    variants = robot.GetVariantSets()
    if variants.HasVariantSet("Physics"):
        physics = variants.GetVariantSet("Physics")
        if "physx" not in physics.GetVariantNames():
            raise RuntimeError(f"Imported x_bot asset has no physx Physics variant: {usd_path}")
        physics.SetVariantSelection("physx")
        robot.Load()
    xform = UsdGeom.XformCommonAPI(robot)
    # The Isaac URDF places base_footprint at the wheel contact plane.
    xform.SetTranslate(Gf.Vec3d(x, y, z))
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


def round_chassis_geometry(stage: Usd.Stage) -> None:
    """Use matching smooth visuals and fine convex collision rings for the base."""
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(ROBOT_PRIM_PATH + '/') or not prim.IsA(UsdGeom.Cylinder):
            continue
        cylinder = UsdGeom.Cylinder(prim)
        radius = cylinder.GetRadiusAttr().Get()
        if abs(radius - .35) > 1e-6:
            continue  # chassis and roof only; preserve tested tire contacts
        half_height = cylinder.GetHeightAttr().Get() / 2
        count = 96
        angles = [2 * math.pi * i / count for i in range(count)]
        points = [Gf.Vec3f(radius * math.cos(a), radius * math.sin(a), z)
                  for z in (-half_height, half_height) for a in angles]
        faces = [[i, (i+1) % count, (i+1) % count + count, i+count] for i in range(count)]
        faces += [list(reversed(range(count))), list(range(count, 2*count))]
        normals = [Gf.Vec3f(math.cos(angles[i % count]), math.sin(angles[i % count]), 0)
                   for face in faces[:-2] for i in face]
        normals += [Gf.Vec3f(0, 0, -1)] * count + [Gf.Vec3f(0, 0, 1)] * count
        prim.SetTypeName('Mesh')
        mesh = UsdGeom.Mesh(prim)
        mesh.CreatePointsAttr(points)
        mesh.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -half_height), Gf.Vec3f(radius, radius, half_height)])
        mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
        mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateNormalsAttr(normals)
        mesh.SetNormalsInterpolation('faceVarying')
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr(255)
            PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(.002)


def configure_joint_drives(stage: Usd.Stage) -> None:
    """Configure stable tire contacts and wheel/arm/gripper drives."""
    root = PhysxSchema.PhysxArticulationAPI.Apply(find_articulation_root(stage))
    root.CreateSolverPositionIterationCountAttr(32)
    root.CreateSolverVelocityIterationCountAttr(32)
    root.CreateEnabledSelfCollisionsAttr(False)
    round_chassis_geometry(stage)
    # Compliant rubber contacts absorb small polygon/contact corrections
    # instead of transmitting each impulse through the unsuspended chassis.
    tire_material = UsdShade.Material.Define(stage, "/World/Materials/x_bot_tire")
    material = UsdPhysics.MaterialAPI.Apply(tire_material.GetPrim())
    material.CreateStaticFrictionAttr(0.8)
    material.CreateDynamicFrictionAttr(0.7)
    material.CreateRestitutionAttr(0.0)
    rubber = PhysxSchema.PhysxMaterialAPI.Apply(tire_material.GetPrim())
    rubber.CreateCompliantContactStiffnessAttr(100000.0)
    rubber.CreateCompliantContactDampingAttr(2000.0)
    # PhysX's analytic USD cylinders rock and sink for these small, thin
    # wheels. Use explicit convex tire meshes; keep the visual cylinders.
    for name in ("front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"):
        for prim in Usd.PrimRange(find_prim(stage, name)):
            if not prim.IsA(UsdGeom.Cylinder) or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            cylinder = UsdGeom.Cylinder(prim)
            radius, half_width = cylinder.GetRadiusAttr().Get(), cylinder.GetHeightAttr().Get() / 2
            # Preserve the circular profile using PhysX's full hull budget;
            # the default 64-vertex budget flattens these tires considerably.
            count = 128
            # Start on a horizontal facet, with its support plane exactly at
            # the nominal tire radius, rather than balancing on a vertex.
            vertex_radius = radius / math.cos(math.pi/count)
            points = [Gf.Vec3f(vertex_radius * math.cos((2*i+1)*math.pi/count),
                              vertex_radius * math.sin((2*i+1)*math.pi/count), z)
                      for z in (-half_width, half_width) for i in range(count)]
            faces = [[i, (i+1) % count, (i+1) % count + count, i+count] for i in range(count)]
            faces += [list(reversed(range(count))), list(range(count, 2*count))]
            prim.SetTypeName("Mesh")
            mesh = UsdGeom.Mesh(prim)
            mesh.CreatePointsAttr(points)
            mesh.CreateExtentAttr([Gf.Vec3f(-vertex_radius, -vertex_radius, -half_width),
                                   Gf.Vec3f(vertex_radius, vertex_radius, half_width)])
            mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
            mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
            mesh.CreateSubdivisionSchemeAttr("none")
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr(255)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(tire_material, materialPurpose="physics")
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(0.002)
            collision.CreateRestOffsetAttr(0.0)
    for name in WHEEL_JOINTS:
        joint = find_prim(stage, name)
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        # Bounded velocity-servo torque prevents violent contact corrections.
        drive.CreateDampingAttr(8.0)
        drive.CreateMaxForceAttr(12.0)
    for name in ARM_JOINTS:
        joint = find_prim(stage, name)
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateTypeAttr("force")
        joint_number = int(name.removeprefix("fr3_joint"))
        # Position-only ros2_control commands need sufficient servo bandwidth:
        # the old gains lagged moving targets beyond the 0.1 rad path tolerance.
        drive.CreateStiffnessAttr(2000.0 if joint_number <= 4 else 1200.0)
        drive.CreateDampingAttr(80.0 if joint_number <= 4 else 60.0)
        drive.CreateMaxForceAttr(300.0)
    fingertip_material = UsdShade.Material.Define(stage, "/World/Materials/GripperContact")
    friction = UsdPhysics.MaterialAPI.Apply(fingertip_material.GetPrim())
    friction.CreateStaticFrictionAttr(1.5)
    friction.CreateDynamicFrictionAttr(1.2)
    friction.CreateRestitutionAttr(0.0)
    # Soft rubber pads reduce solver velocity chatter during sustained closure.
    pad = PhysxSchema.PhysxMaterialAPI.Apply(fingertip_material.GetPrim())
    pad.CreateCompliantContactStiffnessAttr(20000.0)
    pad.CreateCompliantContactDampingAttr(100.0)
    for name in ("fr3_leftfinger", "fr3_rightfinger"):
        for prim in Usd.PrimRange(find_prim(stage, name)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(fingertip_material, materialPurpose="physics")
                # Rubber pads have finite area; resist twisting about a single
                # contact line instead of treating a held object as a hinge.
                contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                contact.CreateTorsionalPatchRadiusAttr(0.008)
                contact.CreateMinTorsionalPatchRadiusAttr(0.004)
    for name in GRIPPER_JOINTS:
        joint = find_prim(stage, name)
        drive = UsdPhysics.DriveAPI.Apply(joint, "linear")
        drive.CreateTypeAttr("force")
        # A compliant 300 N/m drive could close on a 1 kg object yet slip
        # during lift. Raise normal force bandwidth, bounded to 40 N per finger.
        drive.CreateStiffnessAttr(2000.0)
        drive.CreateDampingAttr(80.0)
        drive.CreateMaxForceAttr(40.0)


def set_initial_joint_state(articulation_path: str) -> None:
    """Place the FR3 in the same initial configuration as the Gazebo backend."""
    from isaacsim.core.experimental.prims import Articulation

    articulation = Articulation(articulation_path)
    names = ARM_JOINTS + GRIPPER_JOINTS
    indices = articulation.get_dof_indices(names).numpy().flatten().tolist()
    articulation.set_dof_positions(INITIAL_JOINT_POSITIONS, dof_indices=indices)
    articulation.set_dof_position_targets(INITIAL_JOINT_POSITIONS, dof_indices=indices)
    articulation.set_dof_velocities(0.0)
    articulation.set_velocities([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])


def cleanup_temp_dir(path: Path) -> None:
    shutil.rmtree(path, ignore_errors=True)
