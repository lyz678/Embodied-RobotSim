#!/usr/bin/env python3
"""Prepare pinned SDF assets without importing Isaac or mutating tracked files."""

import argparse
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml
from scene_sources import REPOSITORY, SOURCE_REVISION, extract_sources

from x_bot_scene_assets.sdf_scene import SdfAssets


def configure_book_inertia(model):
    """Give the unit-mass scanned book a measured COM and positive inertia."""
    if model.get("name") != "book":
        return
    for link in model.findall("link"):
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            if any(
                abs(float(v)) > 1e-12
                for v in collision.findtext("pose", "0 0 0 0 0 0").split()
            ):
                raise ValueError("Book collision pose requires explicit transform")
            scale = [float(v) for v in mesh.findtext("scale", "1 1 1").split()]
            vertices = [
                [float(v) * scale[i] for i, v in enumerate(row.split()[1:4])]
                for row in Path(mesh.findtext("uri")).read_text().splitlines()
                if row.startswith("v ")
            ]
            bounds = [
                (min(v[i] for v in vertices), max(v[i] for v in vertices))
                for i in range(3)
            ]
            size = [hi - lo for lo, hi in bounds]
            center = [(lo + hi) / 2 for lo, hi in bounds]
            if link.find("inertial") is None:
                inertial = ET.SubElement(link, "inertial")
                ET.SubElement(inertial, "mass").text = "1.0"
                ET.SubElement(inertial, "pose").text = (
                    " ".join(map(str, center)) + " 0 0 0"
                )
                tensor = ET.SubElement(inertial, "inertia")
                for i, name in enumerate(("ixx", "iyy", "izz")):
                    ET.SubElement(tensor, name).text = str(
                        sum(size[j] ** 2 for j in range(3) if i != j) / 12
                    )
                for name in ("ixy", "ixz", "iyz"):
                    ET.SubElement(tensor, name).text = "0"


def prepare(world, destination):
    source, revision = extract_sources(destination, SOURCE_REVISION)
    assets = SdfAssets(source)
    scene = assets.world(world)
    physics_config = yaml.safe_load(
        (Path(get_package_share_directory("x_bot_control")) / "config/gazebo_physics.yaml").read_text()
    )
    # Resolve URIs before dropping parser-only source-directory annotations.
    for model in scene.iter("model"):
        base = Path(model.get("_asset_base", str(source / "worlds")))
        for link in model.findall("link"):
            for mesh in link.findall("./visual/geometry/mesh") + link.findall(
                "./collision/geometry/mesh"
            ):
                uri = mesh.find("uri")
                uri.text = str(assets.resolve(uri.text, base))
        model.attrib.pop("_asset_base", None)
        configure_book_inertia(model)
        if model.findtext("static", "false") != "true":
            # Same closed contact approximation as Isaac dynamic mesh assets.
            for mesh in model.findall("link/collision/geometry/mesh"):
                mesh.set("optimization", "convex_decomposition")
                decomposition = ET.SubElement(mesh, "convex_decomposition")
                ET.SubElement(decomposition, "max_convex_hulls").text = "64"
                ET.SubElement(decomposition, "voxel_resolution").text = "400000"
        if model.get("name") in (
            "book",
            "coffee_mug",
            "coke_can",
            "water_bottle",
            "shoe",
            "trash_bin",
        ):
            coefficient = physics_config[
                (
                    "bottle_friction"
                    if model.get("name") == "water_bottle"
                    else (
                        "bin_friction"
                        if model.get("name") == "trash_bin"
                        else "object_friction"
                    )
                )
            ]
            for collision in model.findall("link/collision"):
                surface = collision.find("surface")
                if surface is None:
                    surface = ET.SubElement(collision, "surface")
                friction = surface.find("friction")
                if friction is None:
                    friction = ET.SubElement(surface, "friction")
                ode = friction.find("ode")
                if ode is None:
                    ode = ET.SubElement(friction, "ode")
                for name in ("mu", "mu2"):
                    child = ode.find(name)
                    if child is None:
                        child = ET.SubElement(ode, name)
                    child.text = str(coefficient)
    for plugin in list(scene.findall("plugin")):
        scene.remove(plugin)
    for filename, name in [
        ("gz-sim-physics-system", "Physics"),
        ("gz-sim-user-commands-system", "UserCommands"),
        ("gz-sim-scene-broadcaster-system", "SceneBroadcaster"),
        ("gz-sim-sensors-system", "Sensors"),
    ]:
        plugin = ET.SubElement(
            scene, "plugin", filename=filename, name="gz::sim::systems::" + name
        )
        if name == "Sensors":
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    plugin = ET.SubElement(
        scene,
        "plugin",
        filename="libembodied_gazebo_backend.so",
        name="embodied::GazeboBackend",
    )
    ET.SubElement(plugin, "robot_name").text = "x_bot"
    physics = scene.find("physics")
    if physics is None:
        physics = ET.SubElement(
            scene, "physics", name="default_physics", type="ignored"
        )
    physics.set("default", "true")
    for name, value in [("max_step_size", "0.005"), ("real_time_factor", "1.0")]:
        child = physics.find(name)
        if child is None:
            child = ET.SubElement(physics, name)
        child.text = value
    scene.find("gravity").text = "0 0 -9.81"
    root = ET.Element("sdf", version="1.11")
    root.append(scene)
    output = destination / "gazebo" / f"{world}.sdf"
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="unicode", xml_declaration=True)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--world", choices=("simple_room", "manipulation_test"), required=True
    )
    args = parser.parse_args()
    destination = (
        Path(
            os.environ.get(
                "ISAAC_ASSETS_PATH", str(Path.home() / "isaacsim_assets/6.1")
            )
        )
        / "GazeboMain"
    )
    print(prepare(args.world, destination))
