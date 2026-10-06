"""Keep collision monitoring active while allowing checked separating retreat."""

import copy
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from .core import matrix, transform
from std_msgs.msg import Bool
from .contact_escape import separating_contacts


class ContactScan(Node):
    def __init__(self):
        super().__init__("contact_scan_guard")
        self.command = Twist()
        self.command_wall = self.odom_wall = self.ready_wall = -math.inf
        self.yaw = math.inf
        self.ready = False
        self.active = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.points_pub = self.create_publisher(
            PointCloud2, "/x_bot/collision_points", 10
        )
        self.pending_scan = None
        self.create_timer(0.01, self.publish_points)
        self.pub = self.create_publisher(LaserScan, "/x_bot/scan_collision", 10)
        self.create_subscription(Twist, "/cmd_vel_smoothed", self.cmd, 10)
        self.create_subscription(Odometry, "/odom", self.odom, 10)
        self.create_subscription(Bool, "/localization/ready", self.health, 10)
        self.create_subscription(
            LaserScan, "/x_bot/scan", self.scan, qos_profile_sensor_data
        )

    def cmd(self, msg):
        self.command, self.command_wall = msg, time.monotonic()

    def odom(self, msg):
        self.yaw, self.odom_wall = msg.twist.twist.angular.z, time.monotonic()

    def health(self, msg):
        self.ready, self.ready_wall = msg.data, time.monotonic()

    def scan(self, msg):
        wall = time.monotonic()
        age = self.get_clock().now().nanoseconds * 1e-9 - (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        indices = []
        if (
            msg.header.frame_id == "base_footprint"
            and 0 <= age <= 0.3
            and self.ready
            and wall - self.ready_wall < 0.3
            and wall - self.command_wall < 0.2
            and wall - self.odom_wall < 0.3
            and abs(self.yaw) < 0.05
            and self.command.linear.y == 0.0
        ):
            indices = separating_contacts(
                msg.ranges,
                msg.angle_min,
                msg.angle_increment,
                msg.range_min,
                msg.range_max,
                self.command.linear.x,
                self.command.angular.z,
            )
        active = bool(indices)
        if active != self.active:
            self.get_logger().info(
                "Checked straight reverse escape"
                if active
                else "Full collision scan restored"
            )
            self.active = active
        if indices:
            msg = copy.deepcopy(msg)
            for i in indices:
                msg.ranges[i] = math.inf
        self.pub.publish(msg)
        self.pending_scan = msg
        self.publish_points()

    def publish_points(self):
        """Freeze hits in odom at acquisition time, without waiting for future TF.

        CollisionMonitor can transform these fixed points using its latest body
        pose on EVERY command. The scan stamp stays unchanged for source timeout.
        Retry a missing scan-time TF asynchronously; never block command delivery.
        """
        msg = self.pending_scan
        if msg is None:
            return
        age = (
            self.get_clock().now() - Time.from_msg(msg.header.stamp)
        ).nanoseconds * 1e-9
        if not 0 <= age <= 0.3:
            self.pending_scan = None
            return
        try:
            tf = self.tf.lookup_transform(
                "odom", msg.header.frame_id, Time.from_msg(msg.header.stamp)
            )
        except TransformException:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = (
            np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        )
        points = np.column_stack(
            (
                ranges[valid] * np.cos(angles[valid]),
                ranges[valid] * np.sin(angles[valid]),
                np.zeros(np.count_nonzero(valid)),
            )
        )
        t, q = tf.transform.translation, tf.transform.rotation
        try:
            points = transform(points, matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
        except ValueError:
            self.pending_scan = None
            return
        cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=msg.header.stamp, frame_id="odom"), points.astype(np.float32)
        )
        self.points_pub.publish(cloud)
        self.pending_scan = None


def main():
    rclpy.init()
    node = ContactScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
