#!/usr/bin/env python3
"""Measure running simulator VIO against debug ground truth (never an input).

Requires the VIO localization stack and a simulator. --drive explicitly enables
a short 0.1 m/s forward and 0.15 rad/s turn test through the existing safety gate.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def pose_sample(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return [stamp(msg), p.x, p.y, yaw]


def tracking_error(estimate, truth):
    estimate, truth = np.array(estimate), np.array(truth)
    estimate = estimate[(estimate[:,0] >= truth[0,0]) & (estimate[:,0] <= truth[-1,0])]
    target = np.column_stack([np.interp(estimate[:,0], truth[:,0], truth[:,i]) for i in (1,2)])
    target_yaw = np.interp(estimate[:,0], truth[:,0], np.unwrap(truth[:,3]))
    def relative(points, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return (points-points[0]) @ np.array([[c,-s],[s,c]])
    delta = relative(estimate[:,1:3], estimate[0,3])-relative(target, target_yaw[0])
    errors = np.linalg.norm(delta, axis=1)
    angle = (np.unwrap(estimate[:,3])-estimate[0,3])-(target_yaw-target_yaw[0])
    angle = np.arctan2(np.sin(angle), np.cos(angle))
    return dict(samples=len(estimate), position_rmse_m=float(np.sqrt(np.mean(errors**2))),
                position_max_m=float(errors.max()), yaw_rmse_rad=float(np.sqrt(np.mean(angle**2))),
                truth_displacement_m=float(np.linalg.norm(target[-1]-target[0])),
                truth_turn_rad=float(target_yaw[-1]-target_yaw[0]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('.cache/vslam/validation.json'))
    parser.add_argument('--timeout', type=float, default=180.)
    args = parser.parse_args()
    rclpy.init()
    node = Node('vio_runtime_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    command = node.create_publisher(Twist, '/x_bot/cmd_vel', 10)
    save = node.create_client(Trigger, '/localization/save_map')
    data = {k: [] for k in ('vio', 'odom', 'truth', 'cloud', 'sensor', 'scan', 'ready', 'map')}
    sim, ready = [None], [False]
    subscriptions = []
    subscriptions.append(node.create_subscription(Clock, '/clock', lambda m: sim.__setitem__(0, m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data))
    for key, topic in (('vio','/vio/odometry'), ('odom','/odom'), ('truth','/debug/ground_truth/odom')):
        subscriptions.append(node.create_subscription(Odometry, topic, lambda m, k=key: data[k].append(pose_sample(m)), 100))
    for key, topic, kind in (('cloud','/vio/cloud_odom',PointCloud2), ('sensor','/vio/sensor_odom',Odometry), ('scan','/x_bot/scan',LaserScan), ('map','/map',OccupancyGrid)):
        subscriptions.append(node.create_subscription(kind, topic, lambda m, k=key: data[k].append([stamp(m), time.monotonic()]), qos_profile_sensor_data))
    def readiness(msg):
        ready[0] = msg.data
        data['ready'].append([sim[0], msg.data])
    subscriptions.append(node.create_subscription(Bool, '/localization/ready', readiness, 100))
    started, begin, end, last_command = time.monotonic(), None, None, 0.
    report = {'drive_requested': args.drive}
    try:
        while time.monotonic()-started < args.timeout:
            rclpy.spin_once(node, timeout_sec=.02)
            if begin is None and ready[0] and sim[0] is not None and data['vio'] and data['truth']:
                begin = sim[0]
                print(f'Localization ready; test begins at simulation {begin:.3f}', flush=True)
            elapsed = 0. if begin is None else sim[0]-begin
            if time.monotonic()-last_command >= .05:
                cmd = Twist()
                if args.drive and begin is not None:
                    if 2 <= elapsed < 6:
                        cmd.linear.x = .1
                    elif 6 <= elapsed < 8:
                        cmd.angular.z = .15
                if args.drive:
                    command.publish(cmd)
                last_command = time.monotonic()
            if begin is not None and elapsed >= 10:
                end = sim[0]
                break
        if args.drive:
            command.publish(Twist())
        report.update(wall_seconds=time.monotonic()-started, sim_begin=begin, sim_end=end,
                      counts={key: len(value) for key,value in data.items()})
        if begin is None or end is None:
            raise RuntimeError('Timed out waiting for readiness or 10 simulation seconds')
        vio = [row for row in data['vio'] if begin <= row[0] <= end]
        truth = data['truth']
        if len(vio) < 20 or len(truth) < 20:
            raise RuntimeError('Insufficient VIO/ground truth samples')
        report['tracking'] = tracking_error(vio, truth)
        report['vio_sim_hz'] = (len(vio)-1)/(vio[-1][0]-vio[0][0])
        status = [row[1] for row in data['ready'] if row[0] is not None and row[0] >= begin]
        report['ready_fraction'] = sum(status)/len(status)
        clouds = {row[0] for row in data['cloud']}
        report['cloud_pairs'] = len(clouds & {row[0] for row in data['sensor']} & {row[0] for row in data['odom']})
        report['tf'] = {}
        for target, source in (('map','odom'), ('odom','base_footprint')):
            msg = buffer.lookup_transform(target, source, rclpy.time.Time())
            report['tf'][target+'->'+source] = dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                xyz=[msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z])
        if save.wait_for_service(timeout_sec=3):
            future = save.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=15)
            if future.done() and future.result():
                report['map_save'] = dict(success=future.result().success, message=future.result().message)
        checks = {
            'tracking_position': report['tracking']['position_rmse_m'] < .1,
            'tracking_heading': report['tracking']['yaw_rmse_rad'] < .1,
            'ready': report['ready_fraction'] > .9,
            'paired_clouds': report['cloud_pairs'] >= 20,
            'scan_and_map': bool(data['scan'] and data['map']),
            'saved_map': report.get('map_save',{}).get('success',False),
        }
        if args.drive:
            checks['actual_translation'] = report['tracking']['truth_displacement_m'] > .15
            checks['actual_rotation'] = abs(report['tracking']['truth_turn_rad']) > .15
        report['checks'] = checks
        report['passed'] = all(checks.values())
    except (RuntimeError, TransformException) as exc:
        report.update(passed=False, error=str(exc))
    finally:
        if args.drive:
            command.publish(Twist())
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        args.output.with_suffix('.trace.json').write_text(json.dumps(data)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
