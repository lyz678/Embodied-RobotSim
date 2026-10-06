"""Downloaded and procedural USD scenes used by the Isaac Sim backend."""

from __future__ import annotations

import math
import os
import re
from pathlib import Path
from typing import Sequence

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics


Color = tuple[float, float, float]


def _safe_name(name: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not value or value[0].isdigit():
        value = f"object_{value}"
    return value


def _set_transform(prim: Usd.Prim, position: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
    xform = UsdGeom.XformCommonAPI(prim)
    xform.SetTranslate(Gf.Vec3d(*position))
    xform.SetRotate(tuple(math.degrees(v) for v in rpy), UsdGeom.XformCommonAPI.RotationOrderXYZ)


def _finish_geometry(
    geom: UsdGeom.Gprim,
    color: Color,
    dynamic: bool,
    mass: float,
    semantic_class: str | None,
) -> Usd.Prim:
    prim = geom.GetPrim()
    geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(prim)
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    if semantic_class:
        prim.CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set(semantic_class)
    return prim


def add_box(
    stage: Usd.Stage,
    path: str,
    size: Sequence[float],
    position: Sequence[float],
    color: Color = (0.75, 0.75, 0.75),
    rpy: Sequence[float] = (0.0, 0.0, 0.0),
    dynamic: bool = False,
    mass: float = 1.0,
    semantic_class: str | None = None,
) -> Usd.Prim:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    prim = _finish_geometry(cube, color, dynamic, mass, semantic_class)
    _set_transform(prim, position, rpy)
    UsdGeom.XformCommonAPI(prim).SetScale(Gf.Vec3f(*size))
    return prim


def add_cylinder(
    stage: Usd.Stage,
    path: str,
    radius: float,
    height: float,
    position: Sequence[float],
    color: Color,
    rpy: Sequence[float] = (0.0, 0.0, 0.0),
    dynamic: bool = False,
    mass: float = 0.2,
    semantic_class: str | None = None,
) -> Usd.Prim:
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr("Z")
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    prim = _finish_geometry(cylinder, color, dynamic, mass, semantic_class)
    _set_transform(prim, position, rpy)
    return prim


def _assets_root() -> Path:
    return Path(os.environ.get("ISAAC_ASSETS_PATH", str(Path.home() / "isaacsim_assets/6.1")))


def spawn_floor_height(stage: Usd.Stage, x: float, y: float) -> float:
    """Start wheel contacts on a thin floor covering, without a drop.

    Only broad, low, static colliders supporting the whole base qualify;
    furniture and loose objects cannot change the robot's spawn height.
    """
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'guide'])
    height = 0.0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not prim.IsA(UsdGeom.Gprim):
            continue
        parent = prim
        dynamic = False
        while parent and not parent.IsPseudoRoot():
            if parent.HasAPI(UsdPhysics.RigidBodyAPI):
                dynamic = True
                break
            parent = parent.GetParent()
        if dynamic:
            continue
        box = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
        low, high = box.GetMin(), box.GetMax()
        if (low[0] <= x-.25 and high[0] >= x+.25 and low[1] <= y-.25 and high[1] >= y+.25
                and high[2]-low[2] <= .12 and -.05 <= high[2] <= .10):
            height = max(height, float(high[2]))
    return height


def _reference_environment(stage: Usd.Stage, folder: str, filename: str) -> Usd.Prim:
    asset = _assets_root() / folder / filename
    if not asset.is_file():
        raise FileNotFoundError(f"Missing downloaded Isaac environment: {asset}; run scripts/assets/download_isaac_environments.py with Isaac python.sh")
    prim = UsdGeom.Xform.Define(stage, "/World/Scene/Environment").GetPrim()
    prim.GetReferences().AddReference(str(asset))
    return prim


def _main_grasp_object(stage: Usd.Stage, name: str, xy: tuple[float, float], floor: float, yaw: float, mass: float) -> None:
    asset = _assets_root() / "PickObjects" / f"{name}.usd"
    if not asset.is_file():
        raise FileNotFoundError(f"Missing converted main-branch grasp object: {asset}")
    body = UsdGeom.Xform.Define(stage, f"/World/Scene/objects/{name}").GetPrim()
    _set_transform(body, (xy[0], xy[1], floor), (0.0, 0.0, yaw))
    UsdPhysics.RigidBodyAPI.Apply(body)
    UsdPhysics.MassAPI.Apply(body).CreateMassAttr(mass)
    body.CreateAttribute("semantic:class", Sdf.ValueTypeNames.String).Set(name)
    visual = UsdGeom.Xform.Define(stage, f"{body.GetPath()}/model").GetPrim()
    visual.GetReferences().AddReference(str(asset))
    for prim in Usd.PrimRange(visual):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")


def build_office(stage: Usd.Stage) -> None:
    _reference_environment(stage, "Office", "office.usd")
    # The enclosed roof blocks skylight. Shadow-free ambient fill keeps the
    # interior and robot camera readable, with a few broad ceiling panels.
    ambient = UsdLux.DomeLight.Define(stage, "/World/Lights/OfficeAmbient")
    ambient.CreateIntensityAttr(1200.0)
    ambient.CreateColorAttr(Gf.Vec3f(0.95, 0.97, 1.0))
    UsdLux.ShadowAPI.Apply(ambient.GetPrim()).CreateShadowEnableAttr(False)
    positions = [(0.0, 0.0), (-7.0, 0.0), (-7.0, -16.0), (-20.0, 8.0),
                 (0.0, 10.0), (-3.0, 30.0), (-14.0, 32.0), (-14.0, 52.0)]
    for index, (x, y) in enumerate(positions):
        panel = UsdLux.RectLight.Define(stage, f"/World/Lights/OfficeCeiling_{index}")
        panel.CreateWidthAttr(3.0)
        panel.CreateHeightAttr(3.0)
        panel.CreateIntensityAttr(2500.0)
        panel.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
        # RectLight emits along local -Z, towards the floor.
        _set_transform(panel.GetPrim(), (x, y, 2.75))
        UsdLux.ShadowAPI.Apply(panel.GetPrim()).CreateShadowEnableAttr(False)


def build_official_simple_room(stage: Usd.Stage) -> None:
    environment = _reference_environment(stage, "Simple_Room", "simple_room.usd")
    # The official scene's floor is at -0.7695 m. Normalize it to z=0.
    _set_transform(environment, (0.0, 0.0, 0.7695))
    table = stage.GetPrimAtPath(f"{environment.GetPath()}/table_low_327")
    if not table:
        raise RuntimeError("Downloaded Simple Room is missing its expected table")
    # Reuse its textured table, orienting it like main's manipulation_test.
    table.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.801, 0.0, -0.7695))
    table.GetAttribute("xformOp:rotateZYX").Set(Gf.Vec3d(0.0, 0.0, 90.0))
    table.GetAttribute("xformOp:scale").Set(Gf.Vec3d(0.5, 0.8, 1.15))
    top = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(table).ComputeAlignedRange().GetMax()[2]
    # Same meshes, xy positions and yaw as main's grasping test.
    _main_grasp_object(stage, "coke", (0.710, -0.224), top + 0.002, 2.77149, 0.35)
    _main_grasp_object(stage, "cup", (0.654, 0.038), top + 0.002, -1.01751, 0.25)
    _main_grasp_object(stage, "book", (0.641, 0.304), top + 0.002, -0.10202, 0.4)
    # Open bin under main's place pose; no solid top blocking released objects.
    center = (-0.02, -0.81)
    add_cylinder(stage, "/World/Scene/furniture/trash_bin/bottom", 0.22, 0.02,
                 (center[0], center[1], 0.01), (0.12, 0.12, 0.12))
    for i in range(24):
        angle = i * 2.0 * math.pi / 24
        add_box(stage, f"/World/Scene/furniture/trash_bin/wall_{i}", (0.06, 0.02, 0.55),
                (center[0] + 0.22*math.cos(angle), center[1] + 0.22*math.sin(angle), 0.275),
                (0.12, 0.12, 0.12), (0.0, 0.0, angle + math.pi/2))


def build_scene(stage: Usd.Stage, world_name: str, package_share: Path) -> None:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, "/World/Scene")
    UsdGeom.Xform.Define(stage, "/World/Lights")
    if world_name == "office":
        build_office(stage)
    elif world_name == "isaac_simple_room":
        build_official_simple_room(stage)
    elif world_name in ("simple_room", "legacy_room", "gazebo_simple_room", "manipulation_test", "small_house", "ware_house", "obstacle_avoidance_test", "empty"):
        source_name = "simple_room" if world_name in ("legacy_room", "gazebo_simple_room") else world_name
        asset = _assets_root() / "GazeboMain" / f"{source_name}.usd"
        manifest = asset.with_suffix(".json")
        if not asset.is_file() or not manifest.is_file():
            raise FileNotFoundError(f"Missing migrated main-branch scene: {asset}; run scripts/assets/migrate_gazebo_scenes.py with Isaac python.sh")
        root = UsdGeom.Xform.Define(stage, "/World/Scene/Environment").GetPrim()
        root.GetReferences().AddReference(str(asset))
    else:
        raise ValueError(
            f"Unsupported Isaac Sim world '{world_name}'. "
            "Supported worlds: office, simple_room, isaac_simple_room, legacy_room, gazebo_simple_room, manipulation_test, small_house, ware_house, obstacle_avoidance_test, empty"
        )


def configure_manipulation_contacts(stage: Usd.Stage) -> None:
    """Apply material and damping after referenced scene assets finish loading."""
    from pxr import PhysxSchema, UsdShade
    material = UsdShade.Material.Define(stage, "/World/Materials/PlasticBottle")
    friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(0.8)
    friction.CreateDynamicFrictionAttr(0.6)
    friction.CreateRestitutionAttr(0.0)
    count = 0
    for prim in stage.Traverse():
        if '/water_bottle/' not in str(prim.GetPath()):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body.CreateAngularDampingAttr(2.0)
            body.CreateLinearDampingAttr(1.0)
            count += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")
    # Dynamic whole-table convex hull filled the space between the legs.
    # Decompose the source mesh, retaining the table's original mass and motion.
    for prim in stage.Traverse():
        if '/kitchen_table/' in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Mesh):
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexDecomposition')
    if count != 1:
        raise RuntimeError(f"Expected one dynamic bottle body, found {count}")
