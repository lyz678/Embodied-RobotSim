"""Isolated collision-monitor regression: 10 Hz TF must not throttle 20 Hz commands.

Run after sourcing install/setup.bash:
ROS_DOMAIN_ID=79 python3 tests/check_collision_timing.py
Checks scan-time geometry, contact stopping and source timeout; no simulator needed.
"""

import os, time, math, subprocess, tempfile, yaml, json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from lifecycle_msgs.srv import ChangeState
from x_bot_control.contact_scan import ContactScan

assert os.environ.get("ROS_DOMAIN_ID") == "79"
rclpy.init()
n = Node("collision_timing_fixture")
guard = ContactScan()
ex = MultiThreadedExecutor(num_threads=3)
ex.add_node(n)
ex.add_node(guard)
tf = TransformBroadcaster(n)
st = StaticTransformBroadcaster(n)
s = TransformStamped()
s.header.frame_id = "base_footprint"
s.child_frame_id = "base_link"
s.transform.rotation.w = 1.0
st.sendTransform(s)
scan = n.create_publisher(LaserScan, "/x_bot/scan", 10)
odom = n.create_publisher(Odometry, "/odom", 10)
ready = n.create_publisher(Bool, "/localization/ready", 10)
cmd = n.create_publisher(Twist, "/cmd_vel_smoothed", 10)
outputs = []
clouds = []
scan_poses = {}
n.create_subscription(
    Twist,
    "/x_bot/cmd_vel_nav",
    lambda m: outputs.append((time.monotonic(), m.linear.x)),
    10,
)
n.create_subscription(
    PointCloud2, "/x_bot/collision_points", lambda m: clouds.append(m), 10
)
phase = "free"
started = time.monotonic()


def sensor():
    stamp = (n.get_clock().now() - rclpy.duration.Duration(seconds=0.05)).to_msg()
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = 1.0 + 0.1 * (time.monotonic() - started)
    scan_poses[(stamp.sec, stamp.nanosec)] = t.transform.translation.x
    t.transform.rotation.w = 1.0
    tf.sendTransform(t)
    o = Odometry()
    o.header = t.header
    o.child_frame_id = "base_footprint"
    odom.publish(o)
    if phase == "stale":
        return
    m = LaserScan()
    m.header.stamp = stamp
    m.header.frame_id = "base_footprint"
    m.angle_min = -0.08
    m.angle_increment = 0.005
    m.range_min = 0.1
    m.range_max = 40.0
    m.ranges = [0.35 if phase == "contact" else 4.0] * 32
    scan.publish(m)


def command():
    m = Twist()
    m.linear.x = 0.4
    cmd.publish(m)
    ready.publish(Bool(data=True))


n.create_timer(0.1, sensor)
n.create_timer(0.05, command)


def wait(f, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not f.done() and time.monotonic() < deadline:
        ex.spin_once(timeout_sec=0.02)
    assert f.done(), "Service timeout"
    return f.result()


def spin_for(duration):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)


cfg = yaml.safe_load(
    (
        Path(__file__).resolve().parents[1]
        / "src/planning/x_bot_navigation/config/nav2.yaml"
    ).read_text()
)["collision_monitor"]["ros__parameters"]
cfg["use_sim_time"] = False
with tempfile.TemporaryDirectory() as temp:
    p = Path(temp) / "params.yaml"
    p.write_text(yaml.safe_dump({"collision_monitor": {"ros__parameters": cfg}}))
    log = open("/tmp/collision-timing.log", "w")
    proc = subprocess.Popen(
        [
            "/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor",
            "--ros-args",
            "--params-file",
            str(p),
        ],
        stdout=log,
        stderr=log,
    )
    try:
        c = n.create_client(ChangeState, "/collision_monitor/change_state")
        deadline = time.monotonic() + 8
        while not c.wait_for_service(timeout_sec=0.05):
            ex.spin_once(timeout_sec=0.02)
            assert time.monotonic() < deadline
        for state in (1, 3):
            req = ChangeState.Request()
            req.transition.id = state
            assert wait(c.call_async(req)).success
        spin_for(1.0)
        outputs.clear()
        clouds.clear()
        spin_for(2.0)
        hz = (len(outputs) - 1) / (outputs[-1][0] - outputs[0][0])
        assert hz > 18.0, hz
        assert outputs and all(v > 0.39 for _, v in outputs), outputs
        points = list(point_cloud2.read_points(clouds[-1], field_names=("x", "y", "z")))
        stamp = clouds[-1].header.stamp
        assert clouds[-1].header.frame_id == "odom"
        scan_x = scan_poses[(stamp.sec, stamp.nanosec)]
        assert abs(float(points[0][0]) - (scan_x + 4 * math.cos(-0.08))) < 1e-5, points[
            0
        ]
        assert (
            n.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
        ) >= 0.04
        phase = "contact"
        spin_for(0.6)
        assert all(v == 0 for _, v in outputs[-5:]), outputs[-5:]
        phase = "free"
        spin_for(0.6)
        assert outputs[-1][1] > 0.39
        phase = "stale"
        spin_for(0.7)
        assert all(v == 0 for _, v in outputs[-5:]), outputs[-5:]
        print(
            json.dumps(
                {
                    "command_hz": hz,
                    "scan_hz": 10,
                    "scan_pose_translation_preserved": True,
                    "front_contact_stops": True,
                    "stale_source_stops": True,
                },
                indent=2,
            )
        )
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        log.close()
ex.shutdown()
guard.destroy_node()
n.destroy_node()
rclpy.shutdown()
