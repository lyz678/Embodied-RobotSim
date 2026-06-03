#!/usr/bin/env python3
"""
Obstacle Avoidance Demo - 机械臂避障规划演示（键盘交互控制）
通过 CollisionObject 在 MoveIt Planning Scene 中添加墙体障碍物
OMPL 规划器自动规划绕障路径

按键控制：
  1 - 回到原始位置 (Home)
  2 - 准备位置 (墙左侧)
  3 - 避障到墙右侧
  4 - 避障到墙左侧
  q - 退出
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
import time
import threading
import sys
import termios
import tty


class ObstacleAvoidanceDemo(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_demo')

        self.pub_arm = self.create_publisher(PoseStamped, '/arm_command/pose', 10)
        self.pub_planning_scene = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/yoloe_multi_text_prompt/pointcloud_colored', 10)
        self.create_subscription(String, '/arm_command/status', self.status_callback, 10)
        self.create_timer(0.1, self._publish_wall_cloud)  # 10Hz

        self.go_home_client = self.create_client(Trigger, '/robot_actions/go_home')

        self.last_motion_status = None

        # 障碍物参数 - 墙放在机械臂工作空间内，挡住左右直线路径
        # 墙在机械臂正前方，Y=0处，机械臂要从Y<0到Y>0必须绕过这面墙
        self.WALL_CENTER_X = 0.4
        self.WALL_CENTER_Y = 0.0
        self.WALL_LENGTH = 0.3     # X方向长度（较短，只挡住中间）
        self.WALL_THICKNESS = 0.05
        self.WALL_HEIGHT = 0.8     # 高度覆盖目标点区域

        # 位姿定义 - 左右两点在墙两侧，高度在墙范围内
        self.POSES = {
            'left': {
                'pos': [0.4, -0.3, 0.5],
                'rot': [0.0, 0.0, 0.0, 1.0]
            },
            'right': {
                'pos': [0.4, 0.3, 0.5],
                'rot': [0.0, 0.0, 0.0, 1.0]
            },
        }

        # 键盘控制线程
        self.logic_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.logic_thread.start()

    def status_callback(self, msg):
        self.last_motion_status = msg.data

    def _publish_wall_cloud(self):
        """发布墙体点云到 /yoloe_multi_text_prompt/pointcloud_colored，供 OctoMap/MoveIt 使用"""
        # 在墙体表面均匀采样点
        xs = np.full(400, self.WALL_CENTER_X)
        ys = np.linspace(
            self.WALL_CENTER_Y - self.WALL_LENGTH / 2,
            self.WALL_CENTER_Y + self.WALL_LENGTH / 2, 20
        )
        zs = np.linspace(0.0, self.WALL_HEIGHT, 20)
        yy, zz = np.meshgrid(ys, zs)
        points = np.column_stack([
            xs, yy.ravel(), zz.ravel(),
            np.zeros(400, dtype=np.float32),  # r
            np.zeros(400, dtype=np.float32),  # g
            np.ones(400, dtype=np.float32),   # b
        ]).astype(np.float32)

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        msg = PointCloud2()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        msg.is_dense = True
        self.pub_cloud.publish(msg)

    def add_wall_to_planning_scene(self):
        """通过 PlanningScene 消息直接添加墙体碰撞物体"""
        co = CollisionObject()
        co.header.frame_id = 'odom'
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = 'obstacle_wall'
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # 墙沿X方向(yaw=90°): X方向长, Y方向薄, Z方向高
        box.dimensions = [self.WALL_LENGTH, self.WALL_THICKNESS, self.WALL_HEIGHT]

        box_pose = Pose()
        box_pose.position.x = self.WALL_CENTER_X
        box_pose.position.y = self.WALL_CENTER_Y
        box_pose.position.z = self.WALL_HEIGHT / 2.0
        box_pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(box_pose)

        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.collision_objects.append(co)
        planning_scene_msg.is_diff = True

        for _ in range(10):
            self.pub_planning_scene.publish(planning_scene_msg)
            time.sleep(0.1)

        self.get_logger().info(
            f"Wall added to planning scene: center=({self.WALL_CENTER_X}, {self.WALL_CENTER_Y}, {self.WALL_HEIGHT/2:.1f}), "
            f"size=({self.WALL_LENGTH}, {self.WALL_THICKNESS}, {self.WALL_HEIGHT})")

    def send_arm_pose(self, x, y, z, qx, qy, qz, qw, timeout=30.0):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info(f"Moving arm to: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.last_motion_status = None
        self.pub_arm.publish(pose)

        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.last_motion_status:
                if "SUCCESS" in self.last_motion_status:
                    self.get_logger().info("Arm motion SUCCESS")
                    return True
                else:
                    self.get_logger().error(f"Arm motion failed: {self.last_motion_status}")
                    return False
            time.sleep(0.1)

        self.get_logger().error(f"Arm motion timeout after {timeout}s")
        return False

    def call_go_home(self, timeout=30.0):
        self.get_logger().info("Moving to HOME position...")
        if not self.go_home_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("go_home service not available")
            return False

        req = Trigger.Request()
        future = self.go_home_client.call_async(req)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error("go_home timeout")
                return False
            time.sleep(0.1)

        res = future.result()
        if res.success:
            self.get_logger().info(f"go_home success: {res.message}")
            return True
        else:
            self.get_logger().error(f"go_home failed: {res.message}")
            return False

    def move_to_pose(self, name):
        p = self.POSES[name]
        return self.send_arm_pose(
            p['pos'][0], p['pos'][1], p['pos'][2],
            p['rot'][0], p['rot'][1], p['rot'][2], p['rot'][3])

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def print_menu(self):
        print("\n" + "=" * 50)
        print("  机械臂避障规划演示 - 键盘控制")
        print("=" * 50)
        print("  [1] 回到原始位置 (Home)")
        print("  [2] 准备位置 (墙左侧)")
        print("  [3] 避障到墙右侧")
        print("  [4] 避障到墙左侧")
        print("  [q] 退出")
        print("=" * 50)
        print("  等待按键输入...")

    def keyboard_loop(self):
        self.get_logger().info("=== Obstacle Avoidance Demo (Keyboard Control) ===")
        self.get_logger().info("Waiting for MoveIt to be ready...")
        time.sleep(5.0)

        self.get_logger().info("Adding wall collision object to planning scene...")
        self.add_wall_to_planning_scene()
        time.sleep(2.0)

        self.get_logger().info("Ready! Use keyboard to control the arm.")
        self.print_menu()

        while rclpy.ok():
            key = self.get_key()

            if key == '1':
                print("\n>>> [1] 回到原始位置...")
                self.call_go_home()
                self.print_menu()

            elif key == '2':
                print("\n>>> [2] 移动到准备位置（墙左侧）...")
                self.move_to_pose('left')
                self.print_menu()

            elif key == '3':
                print("\n>>> [3] 避障移动到墙右侧...")
                self.move_to_pose('right')
                self.print_menu()

            elif key == '4':
                print("\n>>> [4] 避障移动到墙左侧...")
                self.move_to_pose('left')
                self.print_menu()

            elif key == 'q' or key == '\x03':
                print("\n退出演示...")
                rclpy.shutdown()
                break


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
