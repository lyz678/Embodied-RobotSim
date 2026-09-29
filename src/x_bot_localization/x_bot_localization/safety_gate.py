import time
import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from .core import Health


class Gate(Node):
    def __init__(self):
        super().__init__('localization_safety_gate')
        self.health = Health()
        self.localization = False
        self.valid_wall = -float('inf')
        self.last_command = -float('inf')
        self.command = Twist()
        self.pub = self.create_publisher(Twist, '/x_bot/cmd_vel_safe', 10)
        self.ready_pub = self.create_publisher(Bool, '/localization/ready', 10)
        self.create_subscription(Twist, '/x_bot/cmd_vel', self.cmd, 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(Bool, '/localization/valid', self.status, 10)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def cmd(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.command = Twist()
            return
        self.command = Twist()
        ratio = max(1., abs(msg.linear.x)/(.5 if msg.linear.x >= 0 else .2), abs(msg.angular.z))
        self.command.linear.x = msg.linear.x / ratio
        self.command.angular.z = msg.angular.z / ratio
        self.last_command = time.monotonic()

    def odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.health.update(True, stamp, time.monotonic())

    def status(self, msg):
        self.localization, self.valid_wall = msg.data, time.monotonic()

    def tick(self):
        wall = time.monotonic()
        sim = self.get_clock().now().nanoseconds * 1e-9
        ready = self.localization and wall-self.valid_wall < 2 and self.health.ready(sim, wall)
        self.ready_pub.publish(Bool(data=ready))
        self.pub.publish(self.command if ready and wall-self.last_command < .5 else Twist())


def main():
    rclpy.init()
    node = Gate()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
