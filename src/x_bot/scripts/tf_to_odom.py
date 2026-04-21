#!/usr/bin/env python3
"""
TF 转 Odometry 节点
功能：从 TF (odom -> base_footprint) 转换为 nav_msgs/Odometry 消息
用途：为 Nav2 等需要 odom 话题的组件提供里程计数据

当使用 Cartographer SLAM 时，Cartographer 只发布 TF，不发布 odom 话题。
此节点监听 TF 并发布标准的 Odometry 消息。
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math


class TfToOdomNode(Node):
    def __init__(self):
        super().__init__('tf_to_odom_node')
        
        # 参数声明
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # 获取参数
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # TF2 Buffer 和 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Odometry 发布器
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        
        # 用于计算速度的历史数据
        self.last_transform = None
        self.last_time = None
        
        # 定时器
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f'TF to Odom node started: {self.odom_frame} -> {self.base_frame} => {self.odom_topic}'
        )
    
    def timer_callback(self):
        try:
            # 查询 TF
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),  # 获取最新的变换
                Duration(seconds=0.1)
            )
            
            # 创建 Odometry 消息
            odom_msg = Odometry()
            odom_msg.header.stamp = transform.header.stamp
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            # 设置位置
            odom_msg.pose.pose.position.x = transform.transform.translation.x
            odom_msg.pose.pose.position.y = transform.transform.translation.y
            odom_msg.pose.pose.position.z = transform.transform.translation.z
            odom_msg.pose.pose.orientation = transform.transform.rotation
            
            # 计算速度 (通过位置差分)
            current_time = Time.from_msg(transform.header.stamp)
            if self.last_transform is not None and self.last_time is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt > 0.001:  # 避免除零
                    # 线速度
                    dx = transform.transform.translation.x - self.last_transform.transform.translation.x
                    dy = transform.transform.translation.y - self.last_transform.transform.translation.y
                    
                    # 获取当前朝向角
                    q = transform.transform.rotation
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    
                    # 上一次朝向角
                    q_last = self.last_transform.transform.rotation
                    yaw_last = math.atan2(2.0 * (q_last.w * q_last.z + q_last.x * q_last.y),
                                          1.0 - 2.0 * (q_last.y * q_last.y + q_last.z * q_last.z))
                    
                    # 世界坐标系下的速度
                    vx_world = dx / dt
                    vy_world = dy / dt
                    
                    # 转换到机器人坐标系
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    odom_msg.twist.twist.linear.x = vx_world * cos_yaw + vy_world * sin_yaw
                    odom_msg.twist.twist.linear.y = -vx_world * sin_yaw + vy_world * cos_yaw
                    
                    # 角速度
                    dyaw = yaw - yaw_last
                    # 处理角度跳变
                    if dyaw > math.pi:
                        dyaw -= 2 * math.pi
                    elif dyaw < -math.pi:
                        dyaw += 2 * math.pi
                    odom_msg.twist.twist.angular.z = dyaw / dt
            
            # 更新历史数据
            self.last_transform = transform
            self.last_time = current_time
            
            # 设置协方差 (简单的对角矩阵)
            # Pose 协方差
            odom_msg.pose.covariance[0] = 0.01   # x
            odom_msg.pose.covariance[7] = 0.01   # y
            odom_msg.pose.covariance[14] = 0.01  # z
            odom_msg.pose.covariance[21] = 0.01  # roll
            odom_msg.pose.covariance[28] = 0.01  # pitch
            odom_msg.pose.covariance[35] = 0.01  # yaw
            
            # Twist 协方差
            odom_msg.twist.covariance[0] = 0.01   # vx
            odom_msg.twist.covariance[7] = 0.01   # vy
            odom_msg.twist.covariance[14] = 0.01  # vz
            odom_msg.twist.covariance[21] = 0.01  # wx
            odom_msg.twist.covariance[28] = 0.01  # wy
            odom_msg.twist.covariance[35] = 0.01  # wz
            
            # 发布
            self.odom_pub.publish(odom_msg)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TF 还未准备好，静默等待
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdomNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
