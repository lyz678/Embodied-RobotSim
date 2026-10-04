"""Ray-cleared occupancy + voxelized PCD map bundle, saved atomically to NEW directory."""
import math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from std_msgs.msg import Header, Bool, String
from tf2_ros import StaticTransformBroadcaster
from message_filters import Subscriber, TimeSynchronizer
from .core import Grid, matrix, transform, save_bundle
from .adapter import tf_message


class MapBuilder(Node):
    def __init__(self):
        super().__init__('fastlio_map_builder')
        self.output = Path(self.declare_parameter('output_bundle', 'maps/session').value).expanduser().resolve()
        self.initial = self.declare_parameter('initial_pose', [0., 0., 0.]).value
        x, y, yaw = self.initial
        self.origin = matrix([x,y,0], [0,0,math.sin(yaw/2),math.cos(yaw/2)])
        self.grid = Grid()
        self.voxels = {}
        self.last_stamp = None
        self.dirty = False
        # Mapping keeps the supplied map origin fixed; it must remain valid
        # between lidar packets rather than expiring at the last scan stamp.
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(tf_message(self.origin, self.get_clock().now().to_msg(), 'map', 'odom'))
        self.pub = self.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ready = self.create_publisher(Bool, '/localization/valid', 10)
        self.status = self.create_publisher(String, '/localization/status', 10)
        self.cloud = Subscriber(self, PointCloud2, '/fastlio/cloud_odom')
        self.sensor = Subscriber(self, Odometry, '/fastlio/sensor_odom')
        self.sync = TimeSynchronizer([self.cloud, self.sensor], 30)
        self.sync.registerCallback(self.insert)
        self.save_srv = self.create_service(Trigger, '/localization/save_map', self.save)
        self.timer = self.create_timer(1., self.publish)

    def insert(self, cloud, sensor):
        stamp = cloud.header.stamp
        if self.last_stamp and (stamp.sec,stamp.nanosec) <= (self.last_stamp.sec,self.last_stamp.nanosec):
            self.ready.publish(Bool(data=False))
            self.status.publish(String(data='clock_rewind_restart_required'))
            return
        points = point_cloud2.read_points_numpy(cloud, field_names=('x','y','z'), skip_nans=True)
        points = transform(points, self.origin)
        p = sensor.pose.pose.position
        origin = transform([[p.x,p.y,p.z]], self.origin)[0]
        self.grid.insert(origin, points)
        for p in points:
            key = tuple(np.floor(p/.05).astype(int))
            self.voxels[key] = tuple(float(v) for v in p)
        self.last_stamp = stamp
        self.dirty = True
        self.ready.publish(Bool(data=len(self.voxels) >= 100 and bool(self.grid.cells)))
        self.status.publish(String(data=f'mapping points={len(self.voxels)} cells={len(self.grid.cells)}'))

    def publish(self):
        if not self.dirty or not self.grid.cells:
            return
        image, origin = self.grid.image()
        msg = OccupancyGrid()
        msg.header = Header(frame_id='map', stamp=self.last_stamp)
        msg.info.resolution = self.grid.resolution
        msg.info.height, msg.info.width = image.shape
        msg.info.origin.position.x, msg.info.origin.position.y = origin[:2]
        msg.info.origin.orientation.w = 1.
        msg.data = image.reshape(-1).tolist()
        self.pub.publish(msg)
        self.dirty = False

    def save(self, request, response):
        try:
            save_bundle(self.output, self.grid, self.voxels.values(), self.initial)
            response.success, response.message = True, str(self.output)
        except (ValueError, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response


def main():
    rclpy.init()
    node = MapBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
