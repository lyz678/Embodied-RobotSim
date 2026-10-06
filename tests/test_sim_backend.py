"""Backend selection and pinned asset preparation regressions, without simulators."""

import source_packages

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/assets"))
from scene_sources import SOURCE_REVISION
from prepare_gazebo_scene import prepare, configure_book_inertia


class BackendTests(unittest.TestCase):
    def test_scanned_book_preserves_mesh_and_has_measured_inertia(self):
        with tempfile.TemporaryDirectory() as temp:
            mesh = Path(temp) / "book.obj"
            mesh.write_text("v -1 -2 0\nv 1 2 3\n")
            model = ET.fromstring(
                f'<model name="book"><link name="link"><collision><geometry><mesh><uri>{mesh}</uri></mesh></geometry></collision></link></model>'
            )
            configure_book_inertia(model)
            self.assertEqual(model.findtext(".//mesh/uri"), str(mesh))
            self.assertEqual(model.findtext(".//inertial/pose"), "0.0 0.0 1.5 0 0 0")
            self.assertAlmostEqual(float(model.findtext(".//inertia/ixx")), 25 / 12)
            self.assertEqual(mesh.read_text(), "v -1 -2 0\nv 1 2 3\n")

    def test_all_entrypoints_select_default_and_override(self):
        for name in (
            "start_mapping.sh",
            "start_pick.sh",
            "start_embodied.sh",
        ):
            with tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                (root / "scripts/runtime").mkdir(parents=True)
                entry = root / name
                entry.write_text((ROOT / name).read_text())
                (root / "scripts/runtime/robot_services.sh").write_text('printf "%s\\n" "$@"\n')
                baseline = subprocess.check_output(
                    ["bash", str(entry)], text=True
                ).splitlines()
                override = subprocess.check_output(
                    ["bash", str(entry), "--sim", "gazebo"], text=True
                ).splitlines()
                self.assertEqual(baseline[1:3], ["--sim", "isaac"])
                self.assertEqual(override[-2:], ["--sim", "gazebo"])

    def test_task_modes_dispatch_and_reject_invalid_modes(self):
        for name, expected in (("start_mapping.sh", "navigation"), ("start_pick.sh", "navigation_pick")):
            with tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                (root / "scripts/runtime").mkdir(parents=True)
                entry = root / name
                entry.write_text((ROOT / name).read_text())
                (root / "scripts/runtime/robot_services.sh").write_text('printf "%s\\n" "$@"\n')
                result = subprocess.check_output(["bash", str(entry), "--mode", "navigation", "--sim", "gazebo"], text=True).splitlines()
                self.assertEqual(result[0], expected)
                self.assertEqual(result[-2:], ["--sim", "gazebo"])
                invalid = subprocess.run(["bash", str(entry), "--mode", "invalid"], capture_output=True)
                self.assertEqual(invalid.returncode, 2)

    def test_prepared_world_resolves_collision_mesh_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "source"
            (source / "worlds").mkdir(parents=True)
            (source / "models/box").mkdir(parents=True)
            mesh = source / "models/box/collision.obj"
            mesh.write_text("fixture")
            (source / "models/box/model.sdf").write_text(
                '<sdf><model name="box"><link name="link"><collision name="c"><geometry><mesh><uri>model://box/collision.obj</uri></mesh></geometry></collision></link></model></sdf>'
            )
            world = source / "worlds/simple_room.sdf"
            original = '<sdf><world name="simple_room"><gravity>0 0 -9.8</gravity><include><uri>model://box</uri></include></world></sdf>'
            world.write_text(original)
            with patch(
                "prepare_gazebo_scene.extract_sources",
                return_value=(source, SOURCE_REVISION),
            ):
                output = prepare("simple_room", Path(temp))
            scene = ET.parse(output).getroot().find("world")
            self.assertEqual(scene.find(".//mesh/uri").text, str(mesh))
            self.assertEqual(
                scene.find(".//mesh").get("optimization"), "convex_decomposition"
            )
            self.assertEqual(scene.find("physics/max_step_size").text, "0.005")
            self.assertTrue(
                any(
                    p.get("name") == "embodied::GazeboBackend"
                    for p in scene.findall("plugin")
                )
            )
            self.assertEqual(world.read_text(), original)

    def test_cpp_sampler_matches_isaac_reference(self):
        binary = Path(os.environ.get("ROBOT_BUILD_BASE", ROOT / "build")) / "x_bot_gazebo/test_sampling"
        if not binary.exists():
            self.skipTest("Build optional Gazebo tests first")
        import numpy as np

        sys.path.insert(0, str(ROOT / "src/simulation/x_bot_isaac/runtime"))
        from x_bot_sensors.mid360_sampling import direction_array

        for first in (0, 19999, 123456789):
            result = subprocess.check_output(
                [str(binary), str(first), "1000"], text=True
            )
            actual = np.array(
                [[float(v) for v in row.split()] for row in result.splitlines()]
            )
            np.testing.assert_allclose(
                actual, direction_array(first), atol=1e-12, rtol=0
            )

    def test_invalid_backend_rejected_before_cleanup(self):
        result = subprocess.run(
            [
                "bash",
                str(ROOT / "scripts/runtime/robot_services.sh"),
                "explore",
                "--sim",
                "invalid",
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertNotIn("正在清理", result.stdout)

    def test_gazebo_rejects_usd_only_world_before_cleanup(self):
        result = subprocess.run(
            [
                "bash",
                str(ROOT / "scripts/runtime/robot_services.sh"),
                "explore",
                "--sim",
                "gazebo",
                "--world",
                "office",
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertNotIn("正在清理", result.stdout)


if __name__ == "__main__":
    unittest.main()
