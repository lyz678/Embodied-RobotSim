import copy
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from std_msgs.msg import Header
from message_filters import Subscriber, TimeSynchronizer
from .core import matrix, quaternion, transform


def pose_matrix(pose):
    return matrix([getattr(pose.position, a) for a in 'xyz'], [getattr(pose.orientation, a) for a in 'xyzw'])


def set_pose(pose, m):
    for a, v in zip('xyz', m[:3, 3]):
        setattr(pose.position, a, float(v))
    for a, v in zip('xyzw', quaternion(m[:3, :3])):
        setattr(pose.orientation, a, float(v))


def tf_message(m, stamp, parent, child):
    msg = TransformStamped()
    msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, parent, child
    for a, v in zip('xyz', m[:3, 3]):
        setattr(msg.transform.translation, a, float(v))
    for a, v in zip('xyzw', quaternion(m[:3, :3])):
        setattr(msg.transform.rotation, a, float(v))
    return msg


class Adapter(Node):
    def __init__(self):
        super().__init__('fastlio_adapter')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf = TransformBroadcaster(self)
        self.odom_pubs = [self.create_publisher(Odometry, t, 10) for t in ('/odom', '/x_bot/odom')]
        self.cloud_pub = self.create_publisher(PointCloud2, '/fastlio/cloud_odom', 10)
        self.sensor_pub = self.create_publisher(Odometry, '/fastlio/sensor_odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/x_bot/scan', 10)
        self.odom_sub = Subscriber(self, Odometry, '/fastlio/odometry')
        self.cloud_sub = Subscriber(self, PointCloud2, '/fastlio/cloud_registered')
        self.sync = TimeSynchronizer([self.odom_sub, self.cloud_sub], 30)
        self.sync.registerCallback(self.receive)
        self.origin = None
        self.previous = None
        self.fault = False

    def receive(self, odom, cloud):
        if self.fault:
            return
        if odom.header.frame_id != 'camera_init' or odom.child_frame_id != 'body' or cloud.header.frame_id != 'camera_init':
            self.get_logger().error('Unexpected FAST-LIO frame contract')
            return
        ns = Time.from_msg(odom.header.stamp).nanoseconds
        if self.previous and ns <= self.previous[0]:
            self.fault = True
            self.get_logger().error('Time rewind: restart FAST-LIO and adapter; navigation inhibited')
            return
        try:
            extrinsic = self.buffer.lookup_transform('mid360_imu_link', 'base_footprint', Time())
        except TransformException:
            return
        t = extrinsic.transform
        imu_base = matrix([getattr(t.translation, a) for a in 'xyz'], [getattr(t.rotation, a) for a in 'xyzw'])
        camera_imu = pose_matrix(odom.pose.pose)
        camera_base = camera_imu @ imu_base
        if self.origin is None:
            # odom origin = initial base pose, not initial elevated IMU pose.
            self.origin = np.linalg.inv(camera_base)
        odom_base = self.origin @ camera_base
        out = Odometry()
        out.header = Header(stamp=odom.header.stamp, frame_id='odom')
        out.child_frame_id = 'base_footprint'
        set_pose(out.pose.pose, odom_base)
        # Conservative covariance until a fully propagated lever-arm model is available.
        out.pose.covariance = [0.] * 36
        for i in (0, 7, 14, 21, 28, 35):
            out.pose.covariance[i] = max(.02, float(odom.pose.covariance[i]))
            out.twist.covariance[i] = .1
        if self.previous:
            old_ns, old_pose = self.previous
            dt = (ns-old_ns) * 1e-9
            velocity = odom_base[:3, :3].T @ (odom_base[:3, 3]-old_pose[:3, 3]) / dt
            relative = old_pose[:3, :3].T @ odom_base[:3, :3]
            for a, v in zip('xyz', velocity):
                setattr(out.twist.twist.linear, a, float(v))
            out.twist.twist.angular.z = math.atan2(relative[1, 0], relative[0, 0]) / dt
        self.previous = ns, odom_base
        self.tf.sendTransform(tf_message(odom_base, odom.header.stamp, 'odom', 'base_footprint'))
        for pub in self.odom_pubs:
            pub.publish(out)
        sensor = copy.deepcopy(out)
        sensor.child_frame_id = 'mid360_imu_link'
        set_pose(sensor.pose.pose, self.origin @ camera_imu)
        self.sensor_pub.publish(sensor)
        raw = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        points = transform(raw, self.origin)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(out.header, points.tolist()))
        # Derive a navigation scan from DESKEWED points at scan-end base pose.
        local = transform(points, np.linalg.inv(odom_base))
        scan = LaserScan()
        scan.header = Header(stamp=odom.header.stamp, frame_id='base_footprint')
        scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/720
        scan.angle_max = scan.angle_min + 719*scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .1, 40., .1
        scan.ranges = [math.inf] * 720
        for x, y, z in local:
            r = math.hypot(x, y)
            if .1 <= z <= 1.8 and .1 <= r <= 40:
                i = min(719, int((math.atan2(y, x)+math.pi)/scan.angle_increment))
                scan.ranges[i] = min(scan.ranges[i], r)
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
