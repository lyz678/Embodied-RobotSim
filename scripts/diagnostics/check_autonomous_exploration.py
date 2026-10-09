#!/usr/bin/env python3
"""Observe autonomous exploration goals, automatic commands and actual motion.

This diagnostic never publishes velocity commands or navigation goals.
Run after starting start_mapping_and_explore.sh with automatic exploration.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=120.)
    parser.add_argument('--min-distance', type=float, default=.3)
    parser.add_argument('--observe-duration', type=float, default=0.,
                        help='Keep observing this many wall seconds even after movement starts')
    parser.add_argument('--output', type=Path, default=Path('.cache/vslam/exploration_validation.json'))
    args = parser.parse_args()
    rclpy.init()
    node = Node('autonomous_exploration_validation')
    report = dict(automatic_commands=0, automatic_forward_commands=0, safe_commands=0,
                  executing_goals=[], controller_goals=[],
                  max_displacement_m=0., max_turn_rad=0., ready_samples=0, ready_true=0,
                  explore_events=[], localization_faults=[], publishes_commands=False)
    initial = [None]
    trace = []
    subscriptions = []
    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if initial[0] is None:
            initial[0] = (p.x,p.y,yaw)
        x,y,a = initial[0]
        report['max_displacement_m'] = max(report['max_displacement_m'], math.hypot(p.x-x,p.y-y))
        report['max_turn_rad'] = max(report['max_turn_rad'],abs(math.atan2(math.sin(yaw-a),math.cos(yaw-a))))
        trace.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,yaw])
    subscriptions.append(node.create_subscription(Odometry,'/debug/ground_truth/odom',odom,100))
    def command(msg, key):
        if abs(msg.linear.x) > .01 or abs(msg.angular.z) > .01:
            report[key] += 1
        if key == 'automatic_commands' and msg.linear.x > .01:
            report['automatic_forward_commands'] += 1
    for topic,key in (('/x_bot/cmd_vel_nav','automatic_commands'),('/x_bot/cmd_vel_safe','safe_commands')):
        subscriptions.append(node.create_subscription(Twist,topic,lambda m,k=key:command(m,k),100))
    def status(msg, key):
        for goal in msg.status_list:
            identifier = bytes(goal.goal_info.goal_id.uuid).hex()
            if goal.status == 2 and identifier not in report[key]:
                report[key].append(identifier)
    for topic,key in (('/navigate_to_pose/_action/status','executing_goals'),
                      ('/follow_path/_action/status','controller_goals')):
        subscriptions.append(node.create_subscription(GoalStatusArray,topic,lambda m,k=key:status(m,k),
            QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def ready(msg):
        report['ready_samples'] += 1
        report['ready_true'] += int(msg.data)
    subscriptions.append(node.create_subscription(Bool,'/localization/ready',ready,100))
    def log(msg):
        if msg.name == 'vio_adapter' and 'inhibited' in msg.msg:
            report['localization_faults'].append(msg.msg)
        if msg.name == 'explore_node':
            report['explore_events'].append(msg.msg)
    subscriptions.append(node.create_subscription(Log,'/rosout',log,100))
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.timeout:
            rclpy.spin_once(node,timeout_sec=.05)
            if (report['executing_goals'] and report['controller_goals']
                    and report['automatic_forward_commands'] >= 10
                    and report['safe_commands'] >= 10 and report['max_displacement_m'] >= args.min_distance
                    and time.monotonic()-started >= args.observe_duration):
                break
        report['wall_seconds'] = time.monotonic()-started
        report['required_displacement_m'] = args.min_distance
        report['required_observation_seconds'] = args.observe_duration
        report['passed'] = bool(report['executing_goals'] and report['controller_goals']
            and report['automatic_forward_commands'] >= 10
            and report['safe_commands'] >= 10 and report['max_displacement_m'] >= args.min_distance
            and not report['localization_faults']
            and report['wall_seconds'] >= args.observe_duration)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        args.output.with_suffix('.trace.json').write_text(json.dumps(trace)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
