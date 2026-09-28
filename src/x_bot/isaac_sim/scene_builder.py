"""Procedural USD scenes used by the Isaac Sim backend.

Only the two scenarios used by the end-to-end demos are authored here.  The
large Gazebo model library is intentionally not required by this backend.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
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


def _add_ground_and_lights(stage: Usd.Stage, extent: float) -> None:
    add_box(stage, "/World/Scene/ground", (extent, extent, 0.10), (0.0, 0.0, -0.05), (0.72, 0.69, 0.62))
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(450.0)
    dome.CreateColorAttr(Gf.Vec3f(0.92, 0.95, 1.0))
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(1800.0)
    sun.CreateAngleAttr(0.6)
    _set_transform(sun.GetPrim(), (0.0, 0.0, 8.0), (-0.8, 0.4, 0.3))


def _pose(text: str | None) -> list[float]:
    values = [float(v) for v in (text or "0 0 0 0 0 0").split()]
    return (values + [0.0] * 6)[:6]


def _add_sdf_boxes(stage: Usd.Stage, sdf_path: Path) -> None:
    """Import the static box geometry from an SDF without a Gazebo dependency."""
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        return
    for model in world.findall("model"):
        box = model.find("./link/collision/geometry/box/size")
        if box is None:
            box = model.find("./link/visual/geometry/box/size")
        if box is None or not box.text:
            continue
        size = [float(value) for value in box.text.split()]
        model_pose = _pose(model.findtext("pose"))
        link_pose = _pose(model.findtext("./link/pose"))
        position = [model_pose[index] + link_pose[index] for index in range(3)]
        rpy = [model_pose[index] + link_pose[index] for index in range(3, 6)]
        name = _safe_name(model.get("name", "sdf_box"))
        add_box(stage, f"/World/Scene/structure/{name}", size, position, (0.91, 0.90, 0.84), rpy)


def _add_table(stage: Usd.Stage, path: str, center: Sequence[float], size: Sequence[float]) -> None:
    x, y, z = center
    sx, sy, sz = size
    add_box(stage, f"{path}/top", (sx, sy, 0.08), (x, y, z), (0.42, 0.24, 0.11), semantic_class="table")
    for index, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        add_box(
            stage,
            f"{path}/leg_{index}",
            (0.07, 0.07, sz),
            (x + dx * (sx / 2 - 0.08), y + dy * (sy / 2 - 0.08), z / 2),
            (0.30, 0.16, 0.07),
        )


def _add_grasp_objects(stage: Usd.Stage, root: str, positions: dict[str, Sequence[float]]) -> None:
    coke = positions["coke"]
    add_cylinder(stage, f"{root}/coke", 0.033, 0.122, coke, (0.86, 0.03, 0.03), dynamic=True, mass=0.34, semantic_class="coke")
    # A white band makes the procedural can visually distinguishable from other cylinders.
    band = UsdGeom.Cylinder.Define(stage, f"{root}/coke/label")
    band.CreateAxisAttr("Z")
    band.CreateRadiusAttr(0.034)
    band.CreateHeightAttr(0.025)
    band.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.95, 0.95)])
    _set_transform(band.GetPrim(), (0.0, 0.0, 0.0))

    cup = positions["cup"]
    add_cylinder(stage, f"{root}/cup", 0.045, 0.095, cup, (0.12, 0.28, 0.82), dynamic=True, mass=0.25, semantic_class="cup")

    book = positions["book"]
    add_box(stage, f"{root}/book", (0.22, 0.16, 0.035), book, (0.95, 0.74, 0.10), dynamic=True, mass=0.45, semantic_class="book")


def build_simple_room(stage: Usd.Stage, package_share: Path) -> None:
    _add_ground_and_lights(stage, 32.0)
    _add_sdf_boxes(stage, package_share / "worlds" / "simple_room.sdf")

    # Lightweight stand-ins for the missing Gazebo model collection.  Their
    # poses preserve the navigation and LLM coordinates used by this project.
    add_box(stage, "/World/Scene/furniture/bed", (2.0, 1.5, 0.55), (-4.5, 4.0, 0.275), (0.35, 0.55, 0.82), semantic_class="bed")
    add_box(stage, "/World/Scene/furniture/wardrobe", (0.65, 1.7, 2.0), (-6.45, -0.38, 1.0), (0.40, 0.22, 0.12))
    _add_table(stage, "/World/Scene/furniture/reading_desk", (-6.45, 1.62, 0.72), (1.25, 0.60, 0.72))
    add_box(stage, "/World/Scene/furniture/sofa", (2.0, 0.85, 0.78), (1.59, 4.49, 0.39), (0.22, 0.52, 0.30), semantic_class="sofa")
    add_box(stage, "/World/Scene/furniture/tv_cabinet", (1.6, 0.45, 0.65), (-1.31, 3.84, 0.325), (0.20, 0.14, 0.10))
    add_box(stage, "/World/Scene/furniture/tv", (1.1, 0.08, 0.68), (-1.23, 3.68, 1.0), (0.03, 0.04, 0.05), semantic_class="TV")
    add_box(stage, "/World/Scene/furniture/fridge", (0.8, 0.78, 1.9), (6.45, -0.06, 0.95), (0.82, 0.84, 0.86))
    add_box(stage, "/World/Scene/furniture/kitchen_cabinet", (0.65, 2.2, 0.95), (5.25, 4.63, 0.475), (0.70, 0.66, 0.56))
    _add_table(stage, "/World/Scene/furniture/kitchen_table", (4.73, 2.65, 0.76), (1.55, 0.95, 0.76))
    add_box(stage, "/World/Scene/furniture/kitchen_chair", (0.50, 0.50, 0.85), (5.92, 2.52, 0.425), (0.54, 0.32, 0.16), semantic_class="chair")
    add_box(stage, "/World/Scene/furniture/shoe_rack", (1.2, 0.35, 0.8), (2.0, -5.64, 0.4), (0.44, 0.30, 0.18))
    _add_grasp_objects(
        stage,
        "/World/Scene/objects",
        {
            "coke": (4.448, 2.325, 0.861),
            "cup": (4.588, 2.919, 0.848),
            "book": (-6.45, 1.62, 0.78),
        },
    )


def build_manipulation_test(stage: Usd.Stage, _package_share: Path) -> None:
    _add_ground_and_lights(stage, 12.0)
    _add_table(stage, "/World/Scene/furniture/kitchen_table", (0.801, 0.0, 0.76), (1.45, 0.82, 0.76))
    wall_color = (0.86, 0.85, 0.79)
    add_box(stage, "/World/Scene/structure/north", (5.8, 0.12, 2.4), (1.3, 2.5, 1.2), wall_color)
    add_box(stage, "/World/Scene/structure/south", (5.8, 0.12, 2.4), (1.3, -2.58, 1.2), wall_color)
    add_box(stage, "/World/Scene/structure/east", (0.12, 5.2, 2.4), (3.24, 0.0, 1.2), wall_color)
    _add_grasp_objects(
        stage,
        "/World/Scene/objects",
        {
            "coke": (0.710, -0.224, 0.861),
            "cup": (0.654, 0.038, 0.848),
            "book": (0.641, 0.304, 0.818),
        },
    )
    add_cylinder(stage, "/World/Scene/furniture/trash_bin", 0.22, 0.55, (0.0, -0.81, 0.275), (0.12, 0.12, 0.12), semantic_class="trash bin")


def build_scene(stage: Usd.Stage, world_name: str, package_share: Path) -> None:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, "/World/Scene")
    UsdGeom.Xform.Define(stage, "/World/Lights")
    if world_name == "simple_room":
        build_simple_room(stage, package_share)
    elif world_name == "manipulation_test":
        build_manipulation_test(stage, package_share)
    else:
        raise ValueError(
            f"Unsupported Isaac Sim world '{world_name}'. "
            "Supported worlds: simple_room, manipulation_test"
        )
