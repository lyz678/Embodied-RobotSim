"""Isolated Nav2 integration check with an ideal differential-drive plant.

Run after sourcing ROS: ROS_DOMAIN_ID=79 python3 tests/check_nav2_motion.py
This checks curved path tracking and cancel-stop, not Isaac/Gazebo contacts.
The selected domain must be unused by any other ROS session.
"""

import math, time, json, subprocess, yaml, os, tempfile, argparse
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--initial-yaw", type=float, default=0.0)
parser.add_argument("--odom-hz", type=float, default=50.0)
parser.add_argument("--tf-lag", type=float, default=0.0)
parser.add_argument("--goal-yaw", type=float, default=None)
parser.add_argument("--controller", choices=("configured", "mppi", "graceful"), default="configured")
args = parser.parse_args()
assert args.odom_hz > 0 and math.isfinite(args.initial_yaw)
ROOT = Path(__file__).resolve().parents[1]
assert os.environ.get("ROS_DOMAIN_ID") not in (
    None,
    "",
    "0",
), "Use an isolated ROS_DOMAIN_ID (for example 79)."
TEMP = tempfile.TemporaryDirectory(prefix="nav2-motion-")
PARAMS = Path(TEMP.name) / "params.yaml"
LOG = Path(TEMP.name) / "controller.log"
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient

cfg = yaml.safe_load(open(ROOT / "src/planning/x_bot_navigation/config/nav2.yaml"))
p = cfg["controller_server"]["ros__parameters"]
p["use_sim_time"] = False
if args.controller == "mppi":
    p["FollowPath"].update({"primary_controller": "nav2_mppi_controller::MPPIController", "tracking_goal_tolerance_margin": 0.})
if args.controller == "graceful":
    p["FollowPath"].update({
        "primary_controller": "nav2_graceful_controller::GracefulController",
        "initial_rotation": False, "prefer_final_rotation": True,
        "allow_backward": False, "min_lookahead": .3, "max_lookahead": 1.,
        "k_phi": 1., "k_delta": 2., "beta": .2, "lambda": 2.,
        "v_linear_min": .05, "v_linear_max": 1., "v_angular_max": 1.2,
        "slowdown_radius": .5,
    })
c = cfg["local_costmap"]["local_costmap"]["ros__parameters"]
c["plugins"] = ["inflation_layer"]
c["use_sim_time"] = False
open(PARAMS, "w").write(
    yaml.safe_dump(
        {
            "controller_server": {"ros__parameters": p},
            "local_costmap": {"local_costmap": {"ros__parameters": c}},
        }
    )
)
rclpy.init()
node = Node("mppi_motion_fixture")
tf = TransformBroadcaster(node)
odom = node.create_publisher(Odometry, "/odom", 10)
x = y = v = w = 0.0
yaw = args.initial_yaw
last_odom = 0.0
trajectory = []
commands = []
last = time.monotonic()


def command(m):
    global v, w
    v, w = m.linear.x, m.angular.z
    commands.append((time.monotonic(), v, w))


node.create_subscription(Twist, "/cmd_vel", command, 10)


def tick():
    global x, y, yaw, last, last_odom
    now = time.monotonic()
    dt = now - last
    last = now
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += w * dt
    stamp = (
        node.get_clock().now() - rclpy.duration.Duration(seconds=args.tf_lag)
    ).to_msg()
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.z = math.sin(yaw / 2)
    t.transform.rotation.w = math.cos(yaw / 2)
    tf.sendTransform(t)
    o = Odometry()
    o.header = t.header
    o.child_frame_id = "base_link"
    o.pose.pose.position.x = x
    o.pose.pose.position.y = y
    o.pose.pose.orientation = t.transform.rotation
    o.twist.twist.linear.x = v
    o.twist.twist.angular.z = w
    if now - last_odom >= 1.0 / args.odom_hz:
        odom.publish(o)
        last_odom = now
    trajectory.append((x, y, yaw, v, w))


node.create_timer(0.02, tick)


def wait(f, timeout=10):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not f.done():
        rclpy.spin_once(node, timeout_sec=0.02)
    assert f.done(), "service/action timeout"
    return f.result()


log = open(LOG, "w")
proc = subprocess.Popen(
    [
        "/opt/ros/jazzy/lib/nav2_controller/controller_server",
        "--ros-args",
        "--params-file",
        str(PARAMS),
    ],
    stdout=log,
    stderr=log,
)
try:
    client = node.create_client(ChangeState, "/controller_server/change_state")
    end = time.monotonic() + 15
    while not client.wait_for_service(timeout_sec=0.1):
        rclpy.spin_once(node, timeout_sec=0.02)
        assert time.monotonic() < end, "controller unavailable"
    for state in (1, 3):
        req = ChangeState.Request()
        req.transition.id = state
        assert wait(client.call_async(req), 15).success, "lifecycle failed"
    action = ActionClient(node, FollowPath, "/follow_path")
    assert action.wait_for_server(timeout_sec=10)
    path = Path()
    path.header.frame_id = "odom"
    path.header.stamp = node.get_clock().now().to_msg()
    # Straight -> continuous quarter-circle -> straight; no forced intermediate stops.
    points = [(i * 0.05, 0.0, 0.0) for i in range(21)]
    points += [
        (1 + math.sin(a), 1 - math.cos(a), a)
        for a in [i * math.pi / 80 for i in range(1, 41)]
    ]
    points += [(2.0, 1 + i * 0.05, math.pi / 2) for i in range(1, 21)]
    for px, py, a in points:
        q = PoseStamped()
        q.header = path.header
        q.pose.position.x = px
        q.pose.position.y = py
        q.pose.orientation.z = math.sin(a / 2)
        q.pose.orientation.w = math.cos(a / 2)
        path.poses.append(q)
    if args.goal_yaw is not None:
        path.poses[-1].pose.orientation.z = math.sin(args.goal_yaw / 2)
        path.poses[-1].pose.orientation.w = math.cos(args.goal_yaw / 2)
    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = "FollowPath"
    goal.goal_checker_id = "general_goal_checker"
    handle = wait(action.send_goal_async(goal))
    assert handle.accepted, "goal rejected"
    result = wait(handle.get_result_async(), 60)
    moving = [z for z in commands if abs(z[1]) > 0.05 or abs(z[2]) > 0.05]
    both = [z for z in moving if z[1] > 0.05 and abs(z[2]) > 0.05]
    approach_turn = [r for r in trajectory if abs(r[2]) > .6 and r[3] > .05 and abs(r[4]) > .05]
    path_deviation = max(
        min(math.hypot(r[0]-px, r[1]-py) for px, py, _ in points)
        for r in trajectory
    )
    out = {
        "status": result.status,
        "error_code": result.result.error_code,
        "position": [x, y, yaw],
        "commands": len(commands),
        "turn_and_move_fraction": len(both) / max(1, len(moving)),
        "max_v": max(z[1] for z in commands),
        "max_w": max(abs(z[2]) for z in commands),
        "min_v": min(z[1] for z in commands),
        "yaw_at_first_fast_motion": next(
            (r[2] for r in trajectory if r[3] > 0.2), None
        ),
        "moving_approach_turn_samples": len(approach_turn),
        "max_path_deviation": path_deviation,
    }
    print(json.dumps(out))
    assert result.status == 4, out
    assert len(both) > 20, out
    assert out["min_v"] >= -1e-6, out
    if abs(args.initial_yaw) > 0.8:
        # Turn while approaching rather than requiring stationary alignment.
        assert out["moving_approach_turn_samples"] > 5, out
    assert path_deviation < .65, out
    # A free curved path should not settle into the previous low-speed crawl.
    assert (.5 if abs(args.initial_yaw) > .8 else .65) < out["max_v"] <= 1.0 + 1e-6, out
    # Cancel a second goal and ensure zero command is emitted.
    goal.path.poses[-1].pose.position.y = 4.0
    handle = wait(action.send_goal_async(goal))
    wait(handle.cancel_goal_async())
    wait(handle.get_result_async())
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert abs(commands[-1][1]) + abs(commands[-1][2]) < 1e-6, "cancel failed to stop"
    print("PASS: curved-path tracking, joint v/w commands, cancellation stop")
finally:
    print(
        "fixture telemetry", x, y, yaw, "commands", len(commands), "last", commands[-5:]
    )
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    log.close()
    if proc.returncode != 0:
        print(LOG.read_text()[-8000:])
    node.destroy_node()
    rclpy.shutdown()
    TEMP.cleanup()
