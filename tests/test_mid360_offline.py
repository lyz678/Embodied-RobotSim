"""Run: python3 -m unittest discover -s tests -v (no ROS2/Isaac/PCL)."""
import ast
import math
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import json
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/x_bot_localization'), str(ROOT/'src/x_bot/isaac_sim')]
from mid360_sampling import Packet, POINT, period_for_rate, directions, direction_array, settled_at_rest
from x_bot_localization.core import matrix, quaternion, transform, decode_cloud, Grid, ray_cells, Health, save_bundle, VelocityArbiter


def cloud(data, endian=False):
    fields = [NS(name=n, offset=o, datatype=t, count=1) for n,o,t in
              [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('offset_time',16,6),('line',20,2),('tag',21,2)]]
    return NS(header=NS(frame_id='mid360_link'), fields=fields, data=data,
              is_bigendian=endian, width=len(data)//24, height=1, point_step=24, row_step=len(data))


class Sampling(unittest.TestCase):
    def test_vectorized_pattern_matches_reference(self):
        for first in (0, 19999, 123456789):
            actual = direction_array(first)
            np.testing.assert_allclose(actual, list(directions(first)), atol=1e-12)
            np.testing.assert_allclose(np.linalg.norm(actual, axis=1), 1, atol=1e-12)

    def test_batch_packet_wire_format_and_rewind(self):
        reference, batched = Packet(), Packet()
        xyz = np.array([[1., 2., 3.], [math.nan, 0., 1.], [4., 5., 6.]])
        lines = np.array([0, 1, 3])
        rows = [(*point, 100., line) for point, line in zip(xyz, lines)]
        for ns in (5_000_000, 10_000_000, 105_000_000, 5_000_000, 105_000_000):
            self.assertEqual(reference.add(ns, rows), batched.add_arrays(ns, xyz, lines))
            self.assertEqual(reference.data, batched.data)

    def test_layout(self):
        self.assertEqual(POINT.size,24)
        msg=cloud(POINT.pack(1,2,3,42,95_000_000,3,16))
        self.assertEqual(decode_cloud(msg)[0]['offset_time'],95_000_000)

    def test_endian(self):
        msg=cloud(struct.pack('>ffffIBBxx',1,2,3,42,5_000_000,0,16),True)
        self.assertEqual(decode_cloud(msg)[0]['x'],1.)

    def test_real_sample_offsets(self):
        packet=Packet()
        for step in range(20):
            self.assertIsNone(packet.add(5_000_000+step*5_000_000,[(1,0,0,100,0)]))
        ns,data=packet.add(105_000_000,[(1,0,0,100,0)])
        self.assertEqual(ns,5_000_000)
        self.assertEqual([p['offset_time'] for p in decode_cloud(cloud(data))],list(range(0,100_000_000,5_000_000)))

    def test_faster_frames_keep_true_time_and_all_samples(self):
        for hz in (10, 20, 25, 40):
            period = period_for_rate(hz)
            packet = Packet(period)
            frames = []
            for ns in range(0, 1_000_000_001, 5_000_000):
                completed = packet.add(ns, [(1, 0, 0, 100, 0)])
                if completed: frames.append(completed)
            self.assertEqual(len(frames), hz)
            self.assertEqual(sum(len(data)//POINT.size for _, data in frames), 200)
            for start, data in frames:
                offsets = [p['offset_time'] for p in decode_cloud(cloud(data))]
                self.assertEqual(offsets, list(range(0, period, 5_000_000)))
            packet.reset()
            self.assertEqual(packet.period_ns, period)

    def test_frame_rate_validation(self):
        for hz in (0, 6, 15, 19.5, 50, True, math.nan, math.inf):
            with self.assertRaises(ValueError): period_for_rate(hz)
        for period in (0, 4_000_000, 105_000_000):
            with self.assertRaises(ValueError): Packet(period)

    def test_clock_rewind(self):
        p=Packet()
        p.add(50_000_000,[(1,0,0,1,0)])
        p.add(5_000_000,[(2,0,0,1,0)])
        self.assertEqual(p.start,5_000_000)
        self.assertEqual(len(p.data),24)

    def test_packet_reset(self):
        p=Packet(); p.add(0,[(1,0,0,1,0)]); p.reset()
        self.assertIsNone(p.last)
        self.assertEqual(p.data,bytearray())

    def test_pattern_coverage(self):
        rays=np.array(list(directions(0,20000)))
        np.testing.assert_allclose(np.linalg.norm(rays,axis=1),1,atol=1e-12)
        elevation=np.degrees(np.arcsin(rays[:,2]))
        self.assertGreaterEqual(elevation.min(),-7.000001)
        self.assertLessEqual(elevation.max(),52)
        self.assertEqual(len(set(np.floor((np.arctan2(rays[:,1],rays[:,0])+math.pi)*180/math.pi).astype(int))),360)
        self.assertFalse(np.allclose(list(directions(0,100)),list(directions(20000,100))))

    def test_bad_layout_rejected(self):
        msg=cloud(POINT.pack(1,2,3,42,0,0,16)); msg.fields[4].datatype=7
        with self.assertRaises(ValueError): decode_cloud(msg)

    def test_truncated_cloud(self):
        msg=cloud(POINT.pack(1,2,3,42,0,0,16)); msg.data=msg.data[:-1]
        with self.assertRaises(ValueError): decode_cloud(msg)

    def test_nonfinite_filtered(self):
        self.assertEqual(decode_cloud(cloud(POINT.pack(float('nan'),2,3,42,0,0,16))),[])

    def test_bad_offsets_and_line(self):
        for offset,line in [(100_000_000,0),(0,4)]:
            with self.assertRaises(ValueError): decode_cloud(cloud(POINT.pack(1,2,3,42,offset,line,16)))

    def test_order_and_intensity(self):
        data=POINT.pack(1,2,3,999,10,0,16)+POINT.pack(1,2,3,-1,0,1,16)
        decoded=decode_cloud(cloud(data))
        self.assertEqual([p['offset_time'] for p in decoded],[0,10])
        self.assertEqual([p['intensity'] for p in decoded],[0,255])

    def test_row_padding(self):
        data=POINT.pack(1,2,3,42,0,0,16)+b'xxxx'+POINT.pack(4,5,6,42,5,1,16)+b'xxxx'
        msg=cloud(data); msg.width=1; msg.height=2; msg.row_step=28
        self.assertEqual([p['x'] for p in decode_cloud(msg)],[1.,4.])


class Geometry(unittest.TestCase):
    def test_quaternion_roundtrip(self):
        rng=np.random.default_rng(1)
        for _ in range(100):
            m=matrix([1,2,3],rng.normal(size=4))
            np.testing.assert_allclose(matrix([1,2,3],quaternion(m[:3,:3])),m,atol=1e-12)

    def test_bad_quaternion(self):
        with self.assertRaises(ValueError): matrix([0,0,0],[0,0,0,0])

    def test_imu_to_base_lever_arm(self):
        base_imu=matrix([.25,0,.2875],[0,0,0,1])
        camera_imu0=np.eye(4)
        camera_base0=camera_imu0@np.linalg.inv(base_imu)
        odom_camera=np.linalg.inv(camera_base0)
        np.testing.assert_allclose(odom_camera@camera_base0,np.eye(4))
        imu_turned=matrix([0,0,0],[0,0,math.sqrt(.5),math.sqrt(.5)])
        moved_base=odom_camera@imu_turned@np.linalg.inv(base_imu)
        np.testing.assert_allclose(moved_base[:3,3],[.25,-.25,0],atol=1e-12)

    def test_point_transform_inverse(self):
        points=np.array([[1,2,3],[-1,2,0]])
        pose=matrix([4,5,6],[0,0,.5,.5])
        np.testing.assert_allclose(transform(transform(points,pose),np.linalg.inv(pose)),points)


class Mapping(unittest.TestCase):
    def test_bundle_roundtrip_and_no_overwrite(self):
        grid=Grid(1)
        grid.insert([0,0,.3],[[3,0,1],[0,3,1]])
        points=[(i*.05,0.,1.) for i in range(100)]
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)/'bundle'
            save_bundle(folder,grid,points,[1.,2.,.3])
            self.assertEqual(set(p.name for p in folder.iterdir()),{'map.pcd','map.pgm','map.yaml','bundle.json'})
            metadata=json.loads((folder/'bundle.json').read_text())
            self.assertEqual(metadata['initial_pose'],[1.,2.,.3])
            pcd=(folder/'map.pcd').read_text().split('DATA ascii\n')[1]
            np.testing.assert_allclose(np.loadtxt(pcd.splitlines()),points)
            header=(folder/'map.pgm').read_bytes().split(b'\n',3)
            self.assertEqual(header[:3],[b'P5',b'4 4',b'255'])
            image,_=grid.image()
            actual=np.frombuffer(header[3],dtype='uint8').reshape(4,4)[::-1]
            self.assertEqual(actual[2,2],205)
            self.assertEqual(actual[0,3],0)
            self.assertEqual(actual[0,1],254)
            self.assertEqual(yaml.safe_load((folder/'map.yaml').read_text())['resolution'],1)
            before=(folder/'map.pcd').read_bytes()
            with self.assertRaises(ValueError): save_bundle(folder,grid,points,[0,0,0])
            self.assertEqual(before,(folder/'map.pcd').read_bytes())

    def test_empty_bundle_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)/'bundle'
            with self.assertRaises(ValueError): save_bundle(folder,Grid(),[],[0,0,0])
            self.assertFalse(folder.exists())

    def test_raycast(self):
        self.assertEqual(list(ray_cells((0,0),(3,0))),[(0,0),(1,0),(2,0),(3,0)])
        self.assertEqual(list(ray_cells((2,2),(0,0))),[(2,2),(1,1),(0,0)])

    def test_unknown_free_and_occupied(self):
        grid=Grid(resolution=1)
        grid.insert([0,0,.3],[[3,0,1],[0,3,1]])
        image,origin=grid.image()
        self.assertEqual(origin,[0,0,0])
        self.assertEqual(image[0,1],0)
        self.assertEqual(image[0,3],100)
        self.assertEqual(image[2,2],-1)

    def test_ceiling_does_not_clear_floor(self):
        grid=Grid()
        grid.insert([0,0,.3],[[10,0,3]])
        self.assertFalse(grid.cells)

    def test_negative_coordinates(self):
        self.assertEqual(Grid().cell([-.01,-.06]),(-1,-2))

    def test_occupied_wins_same_scan(self):
        g=Grid(1); g.insert([0,0,.3],[[2,0,1],[3,0,1]])
        image,_=g.image()
        self.assertEqual(image[0,2],100)


class CommandPriority(unittest.TestCase):
    def test_manual_overrides_navigation_and_stops_before_resuming(self):
        arbiter = VelocityArbiter()
        arbiter.update('navigation', 1., .1, 10.)
        self.assertEqual(arbiter.select(10.1), ((1., .1), 'navigation'))
        arbiter.update('manual', -.15, 0., 10.2)
        arbiter.update('navigation', 2., .5, 10.3)
        self.assertEqual(arbiter.select(10.4), ((-.15, 0.), 'manual'))
        self.assertEqual(arbiter.select(10.8), ((0., 0.), 'manual_stop'))
        arbiter.update('navigation', .2, 0., 12.3)
        self.assertEqual(arbiter.select(12.4), ((.2, 0.), 'navigation'))

    def test_manual_stop_invalid_input_and_deadman(self):
        arbiter = VelocityArbiter()
        arbiter.update('navigation', 1., 0., 1.)
        arbiter.update('manual', 0., 0., 1.1)
        self.assertEqual(arbiter.select(1.2), ((0., 0.), 'manual'))
        arbiter.update('manual', float('nan'), 1., 1.3)
        self.assertEqual(arbiter.select(1.4), ((0., 0.), 'manual'))
        self.assertEqual(arbiter.select(4.), ((0., 0.), 'idle'))


class Readiness(unittest.TestCase):
    def test_compliant_contact_requires_stationary_pose(self):
        state = ([0, 0, -.015], [0, 0, 0], [0, 0, 0], [0, 0, 9.81])
        self.assertTrue(settled_at_rest(*state, .0001))
        self.assertFalse(settled_at_rest(*state, .012))
        self.assertFalse(settled_at_rest(*state, math.inf))
        self.assertFalse(settled_at_rest([0, 0, -.04], *state[1:], .0001))
        self.assertFalse(settled_at_rest(*state[:3], [0, 0, 8.], .0001))
        self.assertFalse(settled_at_rest(*state[:3], [0, 0, math.nan], .0001))

    def test_stale_invalid_and_future(self):
        h=Health()
        self.assertFalse(h.ready(0,0))
        h.update(True,10,20)
        self.assertTrue(h.ready(10.1,20.1))
        self.assertFalse(h.ready(11,20.1))
        self.assertFalse(h.ready(10.1,23))
        self.assertFalse(h.ready(9,20.1))
        h.update(False,10.2,20.2)
        self.assertFalse(h.ready(10.3,20.3))

    def test_rewind_latches(self):
        h=Health(); h.update(True,10,20); h.update(True,1,21)
        self.assertFalse(h.ready(1.1,21.1))
        h.update(True,11,22)
        self.assertFalse(h.ready(11,22))


class Contracts(unittest.TestCase):
    def test_default_entrypoint_starts_isaac_exploration(self):
        with tempfile.TemporaryDirectory() as folder:
            entry = Path(folder)/'start_explore_and_mapping.sh'
            entry.write_text((ROOT/'start_explore_and_mapping.sh').read_text())
            (Path(folder)/'scripts').mkdir()
            (Path(folder)/'scripts/robot_services.sh').write_text('printf "%s\\n" "$@"\n')
            environment = dict(os.environ)
            environment.pop('SIM_BACKEND', None)
            result = subprocess.run(['bash', str(entry)], env=environment,
                                    check=True, capture_output=True, text=True)
            self.assertEqual(result.stdout.splitlines(),
                             ['explore', '--sim', 'isaac', '--world', 'simple_room', '--bundle', str(Path(folder)/'maps/gazebo_simple_room'),
                              '--initial-x', '0.0', '--initial-y', '0.0',
                              '--initial-yaw', '1.5708', '--depth-source', 'sim',
                              '--lsm-config', str(Path(folder)/'src/LSM_depth_infer/config/config.yaml'),
                              '--lsm-params', str(Path(folder)/'src/LSM_depth_infer/config/isaac_params.yaml'),
                              '--semantic-cloud-source', 'fastlio',
                              '--octomap-backend', 'semantic_cuda', '--semantic-map-config',
                              str(Path(folder)/'src/semantic_voxel_mapping/config/map.yaml'), '--build'])

    def test_pick_entrypoint_uses_main_manipulation_scene(self):
        with tempfile.TemporaryDirectory() as folder:
            entry = Path(folder)/'start_pick_and_place_demo.sh'
            entry.write_text((ROOT/'start_pick_and_place_demo.sh').read_text())
            (Path(folder)/'scripts').mkdir()
            (Path(folder)/'scripts/robot_services.sh').write_text('printf "%s\\n" "$@"\n')
            result = subprocess.run(['bash', str(entry)], check=True, capture_output=True, text=True)
            self.assertEqual(result.stdout.splitlines(),
                             ['pick', '--sim', 'isaac', '--world', 'manipulation_test', '--bundle', str(Path(folder)/'maps/manipulation_test'),
                              '--initial-x', '0.0', '--initial-y', '0.0', '--initial-yaw', '0.0',
                              '--yoloe-config', str(Path(folder)/'src/yoloe_infer/configs/manipulation.yaml'),
                              '--depth-source', 'sim', '--semantic-cloud-source', 'depth',
                              '--octomap-resolution', '0.02', '--semantic-map-config',
                              str(Path(folder)/'src/semantic_voxel_mapping/config/manipulation.yaml'),
                              '--lsm-config', str(Path(folder)/'src/LSM_depth_infer/config/config.yaml'),
                              '--lsm-params', str(Path(folder)/'src/LSM_depth_infer/config/isaac_params.yaml'), '--build'])

    def test_python_syntax(self):
        paths=list((ROOT/'src/x_bot/isaac_sim').glob('*.py'))+list((ROOT/'src/x_bot_localization').rglob('*.py'))
        paths += [path for path in (ROOT/'src/x_bot_localization/scripts').iterdir() if path.is_file()]
        for path in paths:
            ast.parse(path.read_text(),filename=str(path))

    def test_shell_syntax(self):
        for path in list(ROOT.glob('*.sh')) + list((ROOT/'scripts').glob('*.sh')):
            subprocess.run(['bash','-n',str(path)],check=True)

    def test_yaml_and_xml(self):
        for path in (ROOT/'src/x_bot_localization').rglob('*.yaml'):
            self.assertIsInstance(yaml.safe_load(path.read_text()),dict)
        for path in [ROOT/'src/x_bot/urdf/x_bot.xacro',ROOT/'src/x_bot_localization/package.xml',ROOT/'src/isaac_livox_interfaces/package.xml']:
            ET.parse(path)

    def test_nav2_limits(self):
        params=yaml.safe_load((ROOT/'src/x_bot_localization/config/nav2_isaac.yaml').read_text())
        controller=params['controller_server']['ros__parameters']['FollowPath']
        limits=params['velocity_smoother']['ros__parameters']['max_velocity']
        self.assertLessEqual(controller['vx_max'], limits[0])
        self.assertLessEqual(controller['wz_max'], limits[2])
        self.assertAlmostEqual(controller['model_dt'], 1.0/params['controller_server']['ros__parameters']['controller_frequency'])
        smoother=params['velocity_smoother']['ros__parameters']
        self.assertLessEqual(controller['ax_max'], smoother['max_accel'][0])
        self.assertGreaterEqual(controller['ax_min'], smoother['max_decel'][0])
        self.assertLessEqual(controller['az_max'], smoother['max_accel'][2])
        self.assertTrue(smoother['scale_velocities'])
        self.assertEqual(params['velocity_smoother']['ros__parameters']['max_velocity'],[1.,0,1.2])

    def test_planning_margin_prevents_collision_monitor_contact_deadlock(self):
        params=yaml.safe_load((ROOT/'src/x_bot_localization/config/nav2_isaac.yaml').read_text())
        local=params['local_costmap']['local_costmap']['ros__parameters']
        global_map=params['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(local['robot_radius'], global_map['robot_radius'])
        points=ast.literal_eval(params['collision_monitor']['ros__parameters']['FootprintApproach']['points'])
        radii=[math.hypot(x,y) for x,y in points]
        self.assertGreaterEqual(min(radii),.42)
        self.assertGreater(local['robot_radius']+local['footprint_padding']-max(radii),.04)

    def test_rviz_goal_tool_has_navigation_panel(self):
        # GoalTool updates GoalUpdater; Navigation 2 consumes that event and
        # sends NavigateToPose. A toolbar tool alone cannot start navigation.
        config = yaml.safe_load((ROOT/'src/x_bot/rviz/octomap.rviz').read_text())
        tools = {tool['Class'] for tool in config['Visualization Manager']['Tools']}
        panels = {panel['Class'] for panel in config['Panels']}
        self.assertIn('nav2_rviz_plugins/GoalTool', tools)
        self.assertIn('nav2_rviz_plugins/Navigation 2', panels)

    def test_single_tf_authority(self):
        bridge=(ROOT/'src/x_bot/isaac_sim/ros_bridge.py').read_text()
        self.assertNotIn('PublishRawTransformTree',bridge)
        self.assertNotIn('Example_Rotary_2D',bridge)
        self.assertIn('/debug/ground_truth/odom',bridge)
        launch=(ROOT/'src/x_bot_localization/launch/localization.launch.py').read_text()
        self.assertIn("('/tf','/fastlio/tf_internal')",launch)
        start=(ROOT/'scripts/robot_services.sh').read_text()
        self.assertNotIn('cartographer.launch.py',start)
        self.assertNotIn('nav2.launch.py',start)

    def test_fastlio_calibration(self):
        p=yaml.safe_load((ROOT/'src/x_bot_localization/config/fastlio_mid360.yaml').read_text())['/**']['ros__parameters']
        self.assertFalse(p['mapping']['extrinsic_est_en'])
        self.assertEqual(p['mapping']['extrinsic_T'],[0.,0.,0.])
        self.assertEqual(p['preprocess']['timestamp_unit'],3)

    def test_mount_geometry(self):
        tree=ET.parse(ROOT/'src/x_bot/urdf/x_bot.xacro')
        joint=tree.find('.//joint[@name="mid360_joint"]')
        self.assertEqual(joint.find('parent').get('link'),'roof_link')
        arg=tree.find('.//{http://www.ros.org/wiki/xacro}arg[@name="mid360_xyz"]')
        self.assertEqual(arg.get('default'),'0.25 0 0.0875')

    def test_xacro_backend_expansion(self):
        try:
            import xacro
        except ImportError:
            self.skipTest('Optional xacro package not installed')
        original=xacro.eval_extension
        def local_packages(text):
            for name in ('x_bot','franka_description'):
                text=text.replace('$(find '+name+')',str(ROOT/'src'/name))
            return original(text)
        with patch.object(xacro,'eval_extension',side_effect=local_packages):
            for isaac in ('true','false'):
                doc=xacro.process_file(str(ROOT/'src/x_bot/urdf/x_bot.xacro'),mappings={
                    'sim_isaac':isaac,'two_d_lidar_enabled':'true','camera_enabled':'true'})
                tree=ET.fromstring(doc.toxml())
                names={n.get('name') for n in tree.findall('link')}
                self.assertEqual('mid360_link' in names,isaac=='true')
                self.assertEqual('two_d_lidar' in names,isaac=='false')
                joints = {j.get('name'): j for j in tree.findall('joint')}
                base_height = float(joints['base_joint'].find('origin').get('xyz').split()[2])
                self.assertAlmostEqual(base_height, .15 if isaac == 'true' else .12)
                if isaac == 'true':
                    # A zero-height footprint must put all four tire bottoms
                    # on the ground, and the root/chassis mass must stay 70 kg.
                    for side in ('front_left', 'front_right', 'back_left', 'back_right'):
                        joint = joints[side + '_wheel_joint']
                        wheel_z = float(joint.find('origin').get('xyz').split()[2])
                        wheel = tree.find(f'link[@name="{side}_wheel"]')
                        radius = float(wheel.find('collision/geometry/cylinder').get('radius'))
                        self.assertAlmostEqual(base_height + wheel_z - radius, 0.)
                    masses = [float(tree.find(f'link[@name="{name}"]/inertial/mass').get('value'))
                              for name in ('base_link', 'chassis_link')]
                    self.assertAlmostEqual(sum(masses), 70.)
                    self.assertGreaterEqual(masses[0], 1.)


if __name__=='__main__':
    unittest.main()
