"""Camera collision mapping is a per-search session, independent of global LiDAR."""
import time
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, DurabilityPolicy


class LocalMapSession:
    def __init__(self, node):
        self.node = node
        self.clear = node.create_client(Trigger, '/local_map/semantic_voxel_map/clear')
        self.enable = node.create_client(SetBool, '/local_map/semantic_voxel_map/set_enabled')
        self.cloud = node.create_client(SetBool, '/yoloe_multi_text_prompt/enable_pointcloud')
        self.received = -float('inf')
        self.started_stamp = 0
        self.sub = node.create_subscription(PointCloud2, '/local_map/voxels', self.observe,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def observe(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        # A measured free-space scene may contain zero occupied voxels. A clear
        # operation has stamp zero, and is never evidence of a new sensor frame.
        if stamp > 0 and stamp >= self.started_stamp:
            self.received = time.monotonic()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=2.):
            return False
        future = client.call_async(request)
        end = time.monotonic() + 5.
        while not future.done() and time.monotonic() < end:
            time.sleep(.05)
        try:
            return future.done() and future.result().success
        except Exception as exc:
            self.node.get_logger().error('Local collision map service: ' + str(exc))
            return False

    def pause(self):
        # Do not clear the scene: the arm still needs its obstacle snapshot.
        cloud = self.call(self.cloud, SetBool.Request(data=False))
        mapping = self.call(self.enable, SetBool.Request(data=False))
        return cloud and mapping

    def start(self, timeout=12.):
        if getattr(self.node, '_nav_cancel_requested', False):
            return False
        if not self.pause() or not self.call(self.clear, Trigger.Request()):
            return False
        self.received = -float('inf')
        self.started_stamp = self.node.get_clock().now().nanoseconds
        start = time.monotonic()
        if not self.call(self.enable, SetBool.Request(data=True)) or not self.call(self.cloud, SetBool.Request(data=True)):
            self.pause()
            return False
        while time.monotonic() - start < timeout:
            if getattr(self.node, '_nav_cancel_requested', False):
                self.pause()
                return False
            if self.received >= start:
                self.node.get_logger().info('LOCAL_MAP_SEARCH_READY: fresh camera collision map')
                return True
            time.sleep(.05)
        self.pause()
        self.node.get_logger().error('No fresh camera collision map for object search')
        return False
