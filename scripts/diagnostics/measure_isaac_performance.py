#!/usr/bin/env python3
"""Read-only ROS telemetry: run with system ROS Python, not Isaac python.sh."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 5:
        parser.error('--seconds must be finite and at least 5')
    rclpy.init()
    node = rclpy.create_node('measure_isaac_performance')
    clock = []
    odometry = []
    counts = {'imu': 0, 'lidar': 0, 'camera': 0}
    previous = {}
    commands = []
    state = {'ready': False}
    offsets = []
    widths = []
    subscriptions = []
    def seconds(stamp):
        return stamp.sec + stamp.nanosec / 1e9
    def on_clock(msg):
        clock.append((time.monotonic(), seconds(msg.clock)))
    def on_odom(msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        odometry.append((seconds(msg.header.stamp), p.x, p.y, math.hypot(v.x, v.y)))
    def count(name, msg):
        stamp = seconds(msg.header.stamp)
        if previous.get(name) != stamp:
            counts[name] += 1
            previous[name] = stamp
        if name == 'lidar' and msg.width:
            field = next(f for f in msg.fields if f.name == 'offset_time')
            values = np.ndarray((msg.width,), dtype='>u4' if msg.is_bigendian else '<u4',
                                buffer=bytes(msg.data), offset=field.offset, strides=(msg.point_step,))
            offsets.append((int(values.min()), int(values.max())))
            widths.append(msg.width)
    for topic, kind, callback in [
        ('/clock', Clock, on_clock),
        ('/debug/ground_truth/odom', Odometry, on_odom),
        ('/livox/imu', Imu, lambda msg: count('imu', msg)),
        ('/x_bot/mid360/points', PointCloud2, lambda msg: count('lidar', msg)),
        ('/x_bot/camera_left/image_raw', Image, lambda msg: count('camera', msg)),
        ('/x_bot/cmd_vel_safe', Twist, lambda msg: commands.append(msg.linear.x)),
    ]:
        subscriptions.append(node.create_subscription(kind, topic, callback, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Bool, '/localization/ready', lambda msg: state.update(ready=msg.data), 10))
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if len(clock) < 2:
            raise RuntimeError('No advancing /clock; start the simulator before measuring')
        wall = clock[-1][0] - clock[0][0]
        sim = clock[-1][1] - clock[0][1]
        if sim <= 0:
            raise RuntimeError('Simulation clock is paused or reset during measurement')
        distance = sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(odometry, odometry[1:]))
        result = dict(wall_seconds=wall, sim_seconds=sim, real_time_factor=sim/wall,
                      localization_ready=state['ready'], path_m=distance,
                      mean_speed_wall_m_s=distance/wall, mean_speed_sim_m_s=distance/sim,
                      peak_speed_sim_m_s=max((r[3] for r in odometry), default=0),
                      max_command_m_s=max(commands, default=0),
                      rates_per_sim_second={k: v/sim for k, v in counts.items()},
                      lidar_mean_points=sum(widths)/len(widths) if widths else None,
                      lidar_offset_range_ns=[min(r[0] for r in offsets), max(r[1] for r in offsets)] if offsets else None)
        text = json.dumps(result, indent=2)
        print(text)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text + '\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
