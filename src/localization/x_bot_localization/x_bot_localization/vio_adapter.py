"""Pair cuVSLAM base odometry with rectified depth for navigation/mapping."""
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from cv_bridge import CvBridge
from message_filters import Subscriber, SimpleFilter, TimeSynchronizer, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformException
from .adapter import Adapter, pose_matrix
from .vio_geometry import depth_points
from x_bot_common.geometry import matrix, transform


class VioAdapter(Adapter):
    def __init__(self):
        super().__init__('vio')
        self.bridge = CvBridge()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.stride = self.declare_parameter('depth_stride', 4).value
        self.max_range = self.declare_parameter('depth_max_range', 10.0).value
        self.slop = self.declare_parameter('sync_slop', .02).value
        if self.stride < 1 or not 0 < self.slop <= .05 or self.max_range <= .1:
            raise ValueError('Invalid VIO depth/synchronization configuration')
        self.info = None
        self.pose_rejections = 0
        self.healthy_pub = self.create_publisher(Bool, '/vio/healthy', 10)
        self.max_linear_speed = self.declare_parameter('max_pose_speed', 2.0).value
        self.max_angular_speed = self.declare_parameter('max_pose_angular_speed', 3.0).value
        self.max_height = self.declare_parameter('max_base_height_change', .3).value
        if not np.isfinite([self.max_linear_speed, self.max_angular_speed, self.max_height]).all() or min(
                self.max_linear_speed, self.max_angular_speed, self.max_height) <= 0:
            raise ValueError('VIO motion limits must be finite and positive')
        self.info_sub = self.create_subscription(CameraInfo, '/x_bot/camera_left/camera_info',
                                                 self.receive_info, qos)
        self.odom_sub = Subscriber(self, Odometry, '/vio/odometry')
        self.diagnostics_sub = Subscriber(self, DiagnosticArray, '/vio/diagnostics')
        self.diagnostics_sub.registerCallback(self.receive_tracking_state)
        self.validated_odom = SimpleFilter()
        self.tracking_sync = TimeSynchronizer([self.odom_sub, self.diagnostics_sub], 30)
        self.tracking_sync.registerCallback(self.validate_tracking)
        self.depth_sub = Subscriber(self, Image, '/x_bot/camera_left/depth/image_raw',
                                    qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer([self.validated_odom, self.depth_sub], 30, self.slop)
        self.sync.registerCallback(self.receive_vio)

    @staticmethod
    def tracking_state(msg):
        for status in msg.status:
            if status.hardware_id == 'visual_slam':
                for value in status.values:
                    if value.key == 'vo_status':
                        return value.value
        return None

    def inhibit(self, reason):
        if not self.fault:
            self.get_logger().error(reason + '; restart localization; navigation inhibited')
        self.fault = True
        self.healthy_pub.publish(Bool(data=False))

    def receive_tracking_state(self, msg):
        if self.previous is not None and self.tracking_state(msg) == 'Lost':
            # Recovery can use a new VIO origin; never splice it into the old map.
            self.inhibit('cuVSLAM lost visual tracking')

    def validate_tracking(self, odom, diagnostics):
        if not self.fault and self.tracking_state(diagnostics) == 'OK':
            self.validated_odom.signalMessage(odom)

    def receive_info(self, msg):
        self.info = msg

    def receive_vio(self, odom, depth):
        if self.fault or self.info is None:
            return
        if odom.header.frame_id != 'vio_odom' or odom.child_frame_id != 'base_footprint':
            self.get_logger().error('Unexpected cuVSLAM odometry frame contract')
            return
        if (depth.header.frame_id != 'camera_left_frame' or
                (depth.width, depth.height) != (self.info.width, self.info.height)):
            self.get_logger().error('Depth must match rectified left optical camera calibration')
            return
        if depth.encoding not in ('32FC1', '16UC1'):
            self.get_logger().error('Unsupported depth encoding: ' + depth.encoding)
            return
        try:
            extrinsic = self.buffer.lookup_transform('base_footprint', 'camera_left_frame',
                                                     Time.from_msg(depth.header.stamp))
            t = extrinsic.transform
            base_camera = matrix([getattr(t.translation, a) for a in 'xyz'],
                                 [getattr(t.rotation, a) for a in 'xyzw'])
            image = np.asarray(self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough'), dtype=float)
            if depth.encoding == '16UC1':
                image *= .001
            points = depth_points(image, self.info.k, self.stride, self.max_range)
            if len(points) < 10:
                return
            try:
                world_base = pose_matrix(odom.pose.pose)
            except ValueError as exc:
                self.inhibit('Invalid VIO pose: ' + str(exc))
                return
            if self.origin is not None:
                pose = self.origin @ world_base
                if abs(pose[2, 3]) > self.max_height:
                    self.inhibit('VIO base height exceeds planar robot limit')
                    return
                if self.previous is not None:
                    old_ns, old_pose = self.previous
                    dt = (Time.from_msg(odom.header.stamp).nanoseconds-old_ns)*1e-9
                    if dt > 0:
                        distance = np.linalg.norm(pose[:3, 3]-old_pose[:3, 3])
                        relative = old_pose[:3, :3].T @ pose[:3, :3]
                        angle = np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))
                        if (distance > .05+self.max_linear_speed*dt or
                                angle > .05+self.max_angular_speed*dt):
                            self.pose_rejections += 1
                            reason = (f'VIO pose jump exceeds robot motion limits: '
                                      f'distance={distance:.3f} m angle={angle:.3f} rad dt={dt:.3f} s')
                            # A small one-frame correction must not permanently
                            # disable navigation. Do not publish it or advance
                            # previous: the next pose must fit the LAST ACCEPTED
                            # pose, not the rejected outlier.
                            if (self.pose_rejections >= 3 or
                                    distance > 1.+self.max_linear_speed*dt or
                                    angle > .5+self.max_angular_speed*dt):
                                self.inhibit(reason)
                            else:
                                self.healthy_pub.publish(Bool(data=False))
                                self.get_logger().warning('Discarding transient ' + reason)
                            return
            self.pose_rejections = 0
            world_camera = world_base @ base_camera
            # Use the paired VIO stamp on every output: consumers require exact pairs.
            # Depth/pose pairing is bounded by sync_slop (20 ms), without extrapolation.
            self.publish_estimate(odom, world_base, world_camera,
                                  transform(points, world_camera), 'camera_left_frame')
            self.healthy_pub.publish(Bool(data=not self.fault))
        except TransformException:
            return
        except (ValueError, TypeError) as exc:
            self.get_logger().error(str(exc))


def main():
    rclpy.init()
    node = VioAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
