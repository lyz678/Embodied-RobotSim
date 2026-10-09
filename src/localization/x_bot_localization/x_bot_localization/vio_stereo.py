"""Provide mono8 stereo inputs with explicit optical CameraInfo frames."""
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class StereoInputs(Node):
    def __init__(self):
        super().__init__('vio_stereo_inputs')
        self.bridge = CvBridge()
        # The hand-mounted cameras see both gripper fingers in the lower 40%.
        # cuVSLAM uses CameraInfo ROI as feature-tracking borders, preserving K
        # and full-resolution images for all downstream consumers.
        self.masked_bottom_fraction = self.declare_parameter('masked_bottom_fraction', .4).value
        if not 0 <= self.masked_bottom_fraction < .8:
            raise ValueError('masked_bottom_fraction must be in [0, .8)')
        # Both simulator bridges publish reliably. Large RGB frames must be
        # retransmitted as a stereo pair instead of losing different fragments.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions_ = []
        for index, side in enumerate(('left', 'right')):
            frame = 'camera_left_frame' if side == 'left' else 'camera_right_optical_frame'
            image_pub = self.create_publisher(Image, f'/vio/image_{index}', 10)
            info_pub = self.create_publisher(CameraInfo, f'/vio/camera_info_{index}', 10)
            self.subscriptions_.append(self.create_subscription(
                Image, f'/x_bot/camera_{side}/image_raw',
                lambda msg, pub=image_pub, frame=frame: self.image(msg, pub, frame),
                qos))
            self.subscriptions_.append(self.create_subscription(
                CameraInfo, f'/x_bot/camera_{side}/camera_info',
                lambda msg, pub=info_pub, frame=frame: self.info(msg, pub, frame),
                qos))

    def image(self, msg, pub, frame):
        out = self.bridge.cv2_to_imgmsg(self.bridge.imgmsg_to_cv2(msg, 'mono8'), 'mono8')
        out.header = copy.deepcopy(msg.header)
        out.header.frame_id = frame
        pub.publish(out)

    def info(self, msg, pub, frame):
        out = copy.deepcopy(msg)
        out.header.frame_id = frame
        out.roi.x_offset = out.roi.y_offset = 0
        out.roi.width = out.width
        out.roi.height = max(1, int(round(out.height*(1-self.masked_bottom_fraction))))
        out.roi.do_rectify = False
        pub.publish(out)


def main():
    rclpy.init()
    node = StereoInputs()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
