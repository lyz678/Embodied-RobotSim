import time
import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from .core import Health, VelocityArbiter


class Gate(Node):
    def __init__(self):
        super().__init__('localization_safety_gate')
        self.health = Health()
        self.localization = False
        self.valid_wall = -float('inf')
        self.arbiter = VelocityArbiter()
        self.last_state = None
        self.max_forward = self.declare_parameter('max_forward_speed', 1.0).value
        self.max_angular = self.declare_parameter('max_angular_speed', 1.2).value
        self.pub = self.create_publisher(Twist, '/x_bot/cmd_vel_safe', 10)
        self.ready_pub = self.create_publisher(Bool, '/localization/ready', 10)
        self.state_pub = self.create_publisher(String, '/x_bot/control_state', 10)
        self.create_subscription(Twist, '/x_bot/cmd_vel', self.cmd, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd, 10)
        self.create_subscription(Twist, '/x_bot/cmd_vel_nav', self.nav_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(Bool, '/localization/valid', self.status, 10)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def cmd(self, msg):
        self.receive_command(msg, 'manual')

    def nav_cmd(self, msg):
        self.receive_command(msg, 'navigation')

    def receive_command(self, msg, source):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.arbiter.update(source, 0., 0., time.monotonic())
            return
        ratio = max(1., abs(msg.linear.x)/(self.max_forward if msg.linear.x >= 0 else .2), abs(msg.angular.z)/self.max_angular)
        self.arbiter.update(source, msg.linear.x / ratio, msg.angular.z / ratio, time.monotonic())

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
        (linear, angular), state = self.arbiter.select(wall)
        out = Twist()
        if ready:
            out.linear.x, out.angular.z = linear, angular
        else:
            state = 'localization_not_ready'
        self.pub.publish(out)
        if state != self.last_state:
            self.state_pub.publish(String(data=state))
            self.get_logger().info('Base control: '+state)
            self.last_state = state


def main():
    rclpy.init()
    node = Gate()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
