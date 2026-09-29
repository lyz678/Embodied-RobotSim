import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from .core import decode_cloud


class Bridge(Node):
    def __init__(self):
        super().__init__('mid360_livox_bridge')
        self.pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.sub = self.create_subscription(PointCloud2, '/x_bot/mid360/points', self.convert, qos_profile_sensor_data)
        self.last = None

    def convert(self, msg):
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.last is not None and ns <= self.last:
            self.get_logger().error('Clock reset/duplicate cloud; restart localization after simulator reset')
            return
        try:
            points = decode_cloud(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        if len(points) < 10:
            return
        out = CustomMsg()
        out.header, out.timebase, out.lidar_id = msg.header, ns, 0
        out.points = [CustomPoint(x=float(p['x']), y=float(p['y']), z=float(p['z']),
                                  reflectivity=p['intensity'], offset_time=p['offset_time'],
                                  line=p['line'], tag=p['tag']) for p in points]
        out.point_num = len(out.points)
        self.last = ns
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
