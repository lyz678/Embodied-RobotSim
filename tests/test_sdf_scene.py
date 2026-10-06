"""SDF include/asset resolution regressions without Kit or Gazebo."""

import source_packages
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/simulation/x_bot_isaac/runtime'))
from x_bot_scene_assets.sdf_scene import SdfAssets, sdf_bool
from x_bot_sensors.mid360_sampling import ray_matches_surface, native_hit_mask
from scene_contacts import visible_surface_height


class SdfSceneTests(unittest.TestCase):
    def test_visible_tabletop_height_uses_mesh_surface_not_whole_bbox(self):
        # Sloping tabletop plus a higher decoration outside the support point.
        points = [[0,0,.8],[1,0,.9],[1,1,.9],[0,1,.8], [2,2,1.2],[3,2,1.2],[2,3,1.2]]
        self.assertAlmostEqual(visible_surface_height(points,[4,3],[0,1,2,3,4,5,6],.5,.5), .85)
        self.assertIsNone(visible_surface_height(points,[4,3],[0,1,2,3,4,5,6],-1,-1))

    def test_native_triangle_hit_without_valid_depth_is_kept(self):
        points = [[2.7,0,0], [2,0,0], [2,0,0], [40.1,0,0], [float('nan'),0,0]]
        paths = ['/World/Wall/mesh', '', '/World/x_bot/chassis', '/World/Wall/mesh', '/World/Wall/mesh']
        self.assertEqual(native_hit_mask(points, paths).tolist(), [True,False,False,False,False])

    def test_grazing_ray_check_bounds_surface_error_and_rejects_wrong_pattern(self):
        # 5 mm sensor-pose disagreement creates 10 cm range disagreement at
        # a shallow floor incidence. It must not look like a stale pattern.
        self.assertTrue(ray_matches_surface([7.9,0,0], [1,0,0], 8.0, .05))
        self.assertFalse(ray_matches_surface([7.9,.01,0], [1,0,0], 8.0, .05))
        self.assertFalse(ray_matches_surface([7.9,0,0], [1,0,0], 8.0, 1.0))
        self.assertFalse(ray_matches_surface([float('nan'),0,0], [1,0,0], 8.0, .05))

    def test_numeric_sdf_booleans_keep_static_furniture_static(self):
        for value in ('true', '1', ' 1 ', 'TRUE'):
            self.assertTrue(sdf_bool(value))
        for value in ('false', '0', ' 0 '):
            self.assertFalse(sdf_bool(value))
        with self.assertRaises(ValueError):
            sdf_bool('2')

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.model = self.root / 'models/small_house/chair'
        self.model.mkdir(parents=True)
        (self.model / 'mesh.dae').write_text('fixture')
        (self.model / 'model.sdf').write_text('''<sdf><model name="chair"><pose>0 0 0.2 0 0 0</pose>
        <link name="body"><pose>1 0 0 0 0 0</pose><visual name="v"><geometry><mesh>
        <uri>model://chair/mesh.dae</uri></mesh></geometry></visual></link></model></sdf>''')
        (self.root / 'worlds').mkdir()

    def world(self, text):
        (self.root / 'worlds/room.sdf').write_text('<sdf><world name="room">'+text+'</world></sdf>')
        return SdfAssets(self.root).world('room')

    def test_nested_include_keeps_pose_hierarchy(self):
        world = self.world('''<model name="rotated"><static>true</static><pose>4 5 0 0 0 1.5708</pose>
        <include><uri>model://chair</uri></include></model>''')
        parent = world.find('model')
        child = parent.find('model')
        self.assertEqual(parent.findtext('pose'), '4 5 0 0 0 1.5708')
        self.assertEqual(child.findtext('pose'), '0 0 0.2 0 0 0')
        self.assertEqual(child.findtext('link/pose'), '1 0 0 0 0 0')
        self.assertEqual(SdfAssets(self.root).meshes(world), [self.model / 'mesh.dae'])

    def test_world_include_overrides_model_pose_and_name(self):
        world = self.world('''<include><uri>model://small_house/chair</uri><name>desk_chair</name>
        <pose>2 3 4 0 0 1</pose><static>true</static></include>''')
        model = world.find('model')
        self.assertEqual(model.get('name'), 'desk_chair')
        self.assertEqual(model.findtext('pose'), '2 3 4 0 0 1')
        self.assertEqual(model.findtext('static'), 'true')

    def test_missing_assets_fail_instead_of_substitution(self):
        with self.assertRaises(FileNotFoundError):
            self.world('<include><uri>model://missing</uri></include>')

    def test_unsupported_frames_fail(self):
        with self.assertRaises(ValueError):
            self.world('<model name="m"><pose relative_to="another">0 0 0 0 0 0</pose></model>')

    def test_cyclic_include_fails(self):
        (self.model / 'model.sdf').write_text('<sdf><model name="chair"><include><uri>model://chair</uri></include></model></sdf>')
        with self.assertRaises(ValueError):
            self.world('<include><uri>model://chair</uri></include>')


if __name__ == '__main__':
    unittest.main()
