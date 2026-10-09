"""VIO frame, projection, synchronization and mode-selection regressions.

Run after sourcing ROS 2 Jazzy; no GPU or simulator is required.
"""
import source_packages
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from launch import LaunchContext
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from x_bot_control.safety_gate import Gate
from sensor_msgs.msg import CameraInfo
from sensor_msgs_py import point_cloud2
from x_bot_common.geometry import matrix, transform
from x_bot_localization.adapter import Adapter, pose_matrix, set_pose
from x_bot_localization.vio_adapter import VioAdapter
from x_bot_localization.vio_geometry import depth_points
from x_bot_localization.vio_stereo import StereoInputs
from x_bot_mapping.grid import Grid, save_bundle

ROOT = Path(__file__).resolve().parents[1]


class VioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=83)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = VioAdapter()
        for name in ('cloud_pub', 'sensor_pub', 'scan_pub', 'tf'):
            setattr(self.node, name, Mock())
        self.node.odom_pubs = [Mock(), Mock()]
        self.base_camera = matrix([.3, 0, .8], [-.5, .5, -.5, .5])
        extrinsic = TransformStamped()
        set_pose_proxy = Odometry()
        set_pose(set_pose_proxy.pose.pose, self.base_camera)
        extrinsic.transform.translation.x = .3
        extrinsic.transform.translation.z = .8
        extrinsic.transform.rotation = set_pose_proxy.pose.pose.orientation
        self.node.buffer.lookup_transform = Mock(return_value=extrinsic)
        self.info = CameraInfo(width=16, height=16, k=[10.,0.,8.,0.,10.,8.,0.,0.,1.])
        self.node.receive_info(self.info)
        self.depth = CvBridge().cv2_to_imgmsg(np.full((16,16), 2., np.float32), '32FC1')
        self.depth.header.frame_id = 'camera_left_frame'
        self.depth.header.stamp.sec = 10
        self.odom = Odometry()
        self.odom.header.frame_id = 'vio_odom'
        self.odom.child_frame_id = 'base_footprint'
        self.odom.header.stamp.sec = 10
        # Nontrivial initial translation and heading must disappear from odom.
        self.initial = matrix([4, 3, 0], [0, 0, math.sin(.4), math.cos(.4)])
        set_pose(self.odom.pose.pose, self.initial)

    def tearDown(self):
        self.node.destroy_node()

    def test_projection_filters_missing_and_clipped_depth(self):
        depth = np.array([[2., np.nan, np.inf], [0., 10., 3.]])
        points = depth_points(depth, [2.,0.,1.,0.,2.,0.,0.,0.,1.], stride=1)
        np.testing.assert_allclose(points, [[-1,0,2], [1.5,1.5,3]])
        with self.assertRaises(ValueError):
            depth_points(depth, [0.]*9)

    def test_pairs_initial_base_camera_origin_cloud_and_scan(self):
        self.node.receive_vio(self.odom, self.depth)
        out = self.node.odom_pubs[0].publish.call_args.args[0]
        np.testing.assert_allclose(pose_matrix(out.pose.pose), np.eye(4), atol=1e-12)
        sensor = self.node.sensor_pub.publish.call_args.args[0]
        np.testing.assert_allclose(pose_matrix(sensor.pose.pose), self.base_camera, atol=1e-12)
        self.assertEqual(sensor.child_frame_id, 'camera_left_frame')
        cloud = self.node.cloud_pub.publish.call_args.args[0]
        expected = transform(depth_points(np.full((16,16), 2.), self.info.k), self.base_camera)
        np.testing.assert_allclose(point_cloud2.read_points_numpy(cloud), expected, atol=1e-6)
        scan = self.node.scan_pub.publish.call_args.args[0]
        self.assertTrue(any(math.isfinite(v) for v in scan.ranges))
        for msg in (out, sensor, cloud, scan):
            self.assertEqual(msg.header.stamp, self.odom.header.stamp)
        tf = self.node.tf.sendTransform.call_args.args[0]
        self.assertEqual((tf.header.frame_id, tf.child_frame_id), ('odom', 'base_footprint'))

    def test_body_velocity_and_time_rewind_inhibit_future_samples(self):
        self.node.receive_vio(self.odom, self.depth)
        moved = copy.deepcopy(self.odom)
        moved.header.stamp.sec += 1
        delta = np.eye(4)
        delta[0,3] = .2
        set_pose(moved.pose.pose, self.initial @ delta)
        self.node.receive_vio(moved, self.depth)
        out = self.node.odom_pubs[0].publish.call_args.args[0]
        self.assertAlmostEqual(out.pose.pose.position.x, .2)
        self.assertAlmostEqual(out.twist.twist.linear.x, .2)
        self.node.receive_vio(self.odom, self.depth)
        self.assertTrue(self.node.fault)
        moved.header.stamp.sec += 1
        self.node.receive_vio(moved, self.depth)
        self.assertEqual(self.node.cloud_pub.publish.call_count, 2)

    def test_millimeter_depth_and_bad_frames(self):
        depth = CvBridge().cv2_to_imgmsg(np.full((16,16), 2000, np.uint16), '16UC1')
        depth.header = self.depth.header
        self.node.receive_vio(self.odom, depth)
        cloud = self.node.cloud_pub.publish.call_args.args[0]
        expected = transform(depth_points(np.full((16,16), 2.), self.info.k), self.base_camera)
        np.testing.assert_allclose(point_cloud2.read_points_numpy(cloud), expected, atol=1e-6)
        self.odom.child_frame_id = 'camera_link'
        self.node.receive_vio(self.odom, depth)
        self.assertEqual(self.node.cloud_pub.publish.call_count, 1)

    def diagnostics(self, state='OK'):
        return DiagnosticArray(header=copy.deepcopy(self.odom.header), status=[DiagnosticStatus(
            hardware_id='visual_slam', values=[KeyValue(key='vo_status', value=state)])])

    def test_sync_drops_depth_without_close_vio_pose(self):
        self.node.diagnostics_sub.signalMessage(self.diagnostics())
        self.node.odom_sub.signalMessage(self.odom)
        depth = copy.deepcopy(self.depth)
        depth.header.stamp.nanosec = 50_000_000
        self.node.depth_sub.signalMessage(depth)
        self.node.cloud_pub.publish.assert_not_called()
        depth.header.stamp.nanosec = 10_000_000
        self.node.depth_sub.signalMessage(depth)
        self.node.cloud_pub.publish.assert_called_once()

    def test_missing_or_failed_tracking_status_never_releases_pose(self):
        self.node.odom_sub.signalMessage(self.odom)
        self.node.depth_sub.signalMessage(self.depth)
        self.node.cloud_pub.publish.assert_not_called()
        self.node.diagnostics_sub.signalMessage(self.diagnostics('Lost'))
        self.node.cloud_pub.publish.assert_not_called()
        self.assertFalse(self.node.fault)  # Startup before the first valid estimate.

    def test_status_from_another_frame_cannot_release_pose(self):
        diagnostics = self.diagnostics()
        diagnostics.header.stamp.nanosec = 30_000_000
        self.node.odom_sub.signalMessage(self.odom)
        self.node.depth_sub.signalMessage(self.depth)
        self.node.diagnostics_sub.signalMessage(diagnostics)
        self.node.cloud_pub.publish.assert_not_called()

    def test_nonfinite_pose_latches_fault(self):
        self.odom.pose.pose.position.x = float('nan')
        self.node.receive_vio(self.odom, self.depth)
        self.assertTrue(self.node.fault)
        self.node.tf.sendTransform.assert_not_called()
        self.node.cloud_pub.publish.assert_not_called()

    def test_tracking_loss_latches_stop_even_after_recovery(self):
        self.node.receive_vio(self.odom, self.depth)
        self.node.receive_tracking_state(self.diagnostics('Lost'))
        self.assertTrue(self.node.fault)
        self.node.healthy_pub = Mock()
        self.node.receive_tracking_state(self.diagnostics('OK'))
        self.node.receive_vio(self.odom, self.depth)
        self.assertEqual(self.node.cloud_pub.publish.call_count, 1)
        self.assertTrue(self.node.fault)

    def test_pose_jump_is_rejected_before_tf_or_map_publication(self):
        self.node.receive_vio(self.odom, self.depth)
        moved = copy.deepcopy(self.odom)
        moved.header.stamp.nanosec = 30_000_000
        delta = np.eye(4)
        delta[0,3] = 30.
        set_pose(moved.pose.pose, self.initial @ delta)
        self.node.receive_vio(moved, self.depth)
        self.assertTrue(self.node.fault)
        self.assertEqual(self.node.tf.sendTransform.call_count, 1)
        self.assertEqual(self.node.cloud_pub.publish.call_count, 1)

    def test_small_isolated_correction_is_dropped_and_recovers(self):
        self.node.healthy_pub = Mock()
        self.node.receive_vio(self.odom, self.depth)
        moved = copy.deepcopy(self.odom)
        delta = np.eye(4)
        delta[0,3] = .112  # Exact runtime failure: 112 mm over 30 ms.
        moved.header.stamp.nanosec = 30_000_000
        set_pose(moved.pose.pose, self.initial @ delta)
        self.node.receive_vio(moved, self.depth)
        self.assertFalse(self.node.fault)
        self.assertFalse(self.node.healthy_pub.publish.call_args.args[0].data)
        self.assertEqual(self.node.cloud_pub.publish.call_count, 1)
        self.assertEqual(self.node.previous[0], 10_000_000_000)
        # The next frame is checked against the last GOOD pose (60 ms apart).
        moved.header.stamp.nanosec = 60_000_000
        delta[0,3] = .13
        set_pose(moved.pose.pose, self.initial @ delta)
        self.node.receive_vio(moved, self.depth)
        self.assertFalse(self.node.fault)
        self.assertTrue(self.node.healthy_pub.publish.call_args.args[0].data)
        self.assertEqual(self.node.cloud_pub.publish.call_count, 2)
        self.assertEqual(self.node.pose_rejections, 0)

    def test_repeated_small_jumps_latch_fault_against_last_good_pose(self):
        self.node.receive_vio(self.odom, self.depth)
        for i in (1, 2, 3):
            moved = copy.deepcopy(self.odom)
            moved.header.stamp.nanosec = i*30_000_000
            delta = np.eye(4)
            delta[0,3] = i*.12
            set_pose(moved.pose.pose, self.initial @ delta)
            self.node.receive_vio(moved, self.depth)
        self.assertTrue(self.node.fault)
        self.assertEqual(self.node.cloud_pub.publish.call_count, 1)

    def test_vertical_divergence_is_rejected(self):
        self.node.receive_vio(self.odom, self.depth)
        moved = copy.deepcopy(self.odom)
        moved.header.stamp.sec += 1
        moved.pose.pose.position.z = .4
        self.node.receive_vio(moved, self.depth)
        self.assertTrue(self.node.fault)
        self.assertEqual(self.node.cloud_pub.publish.call_count, 1)

    def test_vio_unhealthy_overrides_fresh_mapping_and_velocity(self):
        gate = Gate()
        try:
            gate.require_vio_health = True
            gate.health.ready = Mock(return_value=True)
            gate.pub, gate.ready_pub = Mock(), Mock()
            gate.status(Bool(data=True))
            cmd = Twist()
            cmd.linear.x = .4
            gate.nav_cmd(cmd)
            gate.vio_status(Bool(data=True))
            gate.tick()
            self.assertAlmostEqual(gate.pub.publish.call_args.args[0].linear.x, .4)
            gate.vio_status(Bool(data=False))
            gate.tick()
            self.assertFalse(gate.ready_pub.publish.call_args.args[0].data)
            self.assertEqual(gate.pub.publish.call_args.args[0].linear.x, 0.)
        finally:
            gate.destroy_node()

    def test_stereo_preserves_stamp_and_uses_optical_frames(self):
        node = StereoInputs()
        try:
            image = CvBridge().cv2_to_imgmsg(np.full((8,8,3), 100, np.uint8), 'rgb8')
            image.header.stamp.sec = 42
            pub = Mock()
            node.image(image, pub, 'camera_right_optical_frame')
            out = pub.publish.call_args.args[0]
            self.assertEqual(out.encoding, 'mono8')
            self.assertEqual(out.header.stamp.sec, 42)
            self.assertEqual(out.header.frame_id, 'camera_right_optical_frame')
            node.info(self.info, pub, 'camera_right_optical_frame')
            self.assertEqual(pub.publish.call_args.args[0].k.tolist(), self.info.k.tolist())
            out_info = pub.publish.call_args.args[0]
            self.assertEqual((out_info.width, out_info.height), (16, 16))
            self.assertEqual((out_info.roi.width, out_info.roi.height), (16, 10))
        finally:
            node.destroy_node()

    def test_lio_keeps_imu_lever_arm_and_initial_base_origin(self):
        node = Adapter()
        try:
            node.odom_pubs = [Mock()]
            node.sensor_pub, node.cloud_pub, node.scan_pub, node.tf = Mock(), Mock(), Mock(), Mock()
            extrinsic = TransformStamped()
            extrinsic.transform.translation.z = -.5
            extrinsic.transform.rotation.w = 1.
            node.buffer.lookup_transform = Mock(return_value=extrinsic)
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'camera_init', 'body'
            odom.header.stamp.sec = 10
            odom.pose.pose.position.z = .5
            odom.pose.pose.orientation.w = 1.
            cloud = point_cloud2.create_cloud_xyz32(odom.header, [[2.,0.,.5]])
            node.receive(odom, cloud)
            np.testing.assert_allclose(pose_matrix(node.odom_pubs[0].publish.call_args.args[0].pose.pose), np.eye(4))
            self.assertEqual(node.sensor_pub.publish.call_args.args[0].child_frame_id, 'mid360_imu_link')
        finally:
            node.destroy_node()


class VioModeTests(unittest.TestCase):
    def test_vio_enables_imu_fusion_and_has_no_competing_tf_publisher(self):
        config = yaml.safe_load((ROOT/'src/localization/x_bot_localization/config/vio.yaml').read_text())['/**']['ros__parameters']
        self.assertEqual(config['tracking_mode'], 1)
        for key in ('enable_localization_n_mapping', 'publish_map_to_odom_tf', 'publish_odom_to_base_tf'):
            self.assertFalse(config[key])
        self.assertEqual(config['base_frame'], 'base_footprint')
        self.assertTrue(config['enable_ground_constraint_in_odometry'])

    def test_launch_selects_one_odometry_source_and_cloud_contract(self):
        spec = importlib.util.spec_from_file_location('localization_launch', ROOT/'src/robot/x_bot_bringup/launch/localization.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for source in ('lio', 'vio'):
            context = LaunchContext()
            context.launch_configurations.update(dict(mode='mapping', odometry_source=source,
                bundle='maps/test', initial_x='0', initial_y='0', initial_yaw='0'))
            def share(name):
                return str(ROOT/'src'/('robot' if name == 'x_bot_sensors' else 'localization')/name)
            with patch.object(module, 'get_package_share_directory', side_effect=share) as lookup, \
                    patch.object(module, 'Node', side_effect=lambda **kw: kw):
                nodes = module.setup(context)
            packages = [n['package'] for n in nodes if isinstance(n, dict)]
            self.assertEqual('fast_lio' in packages, source == 'lio')
            self.assertEqual('isaac_ros_visual_slam' in [c.args[0] for c in lookup.call_args_list], source == 'vio')
            gate = next(n for n in nodes if isinstance(n, dict) and n.get('executable') == 'safety_gate')
            self.assertEqual(gate['parameters'][1]['require_vio_health'], source == 'vio')
            self.assertEqual(gate['parameters'][1]['max_forward_speed'], .3 if source == 'vio' else 1.)
            builder = next(n for n in nodes if isinstance(n, dict) and n.get('executable') == 'map_builder')
            self.assertEqual(builder['remappings'], [] if source == 'lio' else [
                ('/fastlio/cloud_odom','/vio/cloud_odom'), ('/fastlio/sensor_odom','/vio/sensor_odom')])

    def test_navigation_vio_limits_are_numeric_and_lio_keeps_defaults(self):
        spec = importlib.util.spec_from_file_location('navigation_launch',
            ROOT/'src/planning/x_bot_navigation/launch/navigation.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        create_temp = tempfile.NamedTemporaryFile
        with tempfile.TemporaryDirectory() as tmp:
            for source, speed, accel in [('lio', 1., 1.), ('vio', .3, .3)]:
                context = LaunchContext()
                context.launch_configurations['odometry_source'] = source
                with patch.object(module, 'get_package_share_directory',
                                  return_value=str(ROOT/'src/planning/x_bot_navigation')), \
                        patch.object(module, 'IncludeLaunchDescription', side_effect=lambda *args, **kw: kw), \
                        patch.object(module, 'RewrittenYaml', side_effect=lambda **kw: kw) as rewritten, \
                        patch.object(module.tempfile, 'NamedTemporaryFile',
                                     side_effect=lambda **kw: create_temp(dir=tmp, **kw)):
                    module.setup(context)
                config = yaml.safe_load(Path(rewritten.call_args.kwargs['source_file']).read_text())
                self.assertEqual(config['controller_server']['ros__parameters']['FollowPath']['v_linear_max'], speed)
                velocity = config['velocity_smoother']['ros__parameters']
                self.assertEqual(velocity['max_velocity'][0], speed)
                self.assertEqual(velocity['max_accel'][0], accel)
                self.assertIsInstance(velocity['max_velocity'], list)

    def test_bundle_records_vio_sensor(self):
        with tempfile.TemporaryDirectory() as tmp:
            grid = Grid()
            grid.insert([0,0,0], [[2,0,.5]])
            folder = Path(tmp)/'map'
            save_bundle(folder, grid, [[2,i*.01,.5] for i in range(100)], [0,0,0], 'vio')
            metadata = json.loads((folder/'bundle.json').read_text())
            self.assertEqual(metadata['odometry_source'], 'vio')
            self.assertEqual(metadata['sensor_frame'], 'camera_left_frame')

    def test_invalid_odometry_rejected_before_cleanup(self):
        result = subprocess.run(['bash', str(ROOT/'scripts/runtime/robot_services.sh'),
                                 'explore', '--odometry', 'invalid'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertNotIn('清理', result.stdout)


if __name__ == '__main__':
    unittest.main()
