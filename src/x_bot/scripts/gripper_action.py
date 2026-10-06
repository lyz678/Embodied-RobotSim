"""Shared FR3 single-finger GripperCommand handling for Isaac demos."""
import math
import time
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand


def control_gripper(node, client, position, timeout=20.0):
    # The action controls one finger (0..0.06 m in this project), not the total jaw opening.
    if not math.isfinite(position) or position < 0.0:
        node.get_logger().error(f"Invalid gripper position: {position}")
        return False
    target = min(float(position), 0.06)
    node.get_logger().info(f"Setting gripper finger to {target:.4f} m (requested {position:.4f})")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Gripper action server not available")
        return False
    goal = GripperCommand.Goal()
    goal.command.position = target
    goal.command.max_effort = 10.0
    sent = client.send_goal_async(goal)
    deadline = time.monotonic() + timeout
    while not sent.done():
        if time.monotonic() >= deadline:
            # Cancel even when acceptance arrives after the client deadline.
            def cancel_late(future):
                handle = future.result()
                if handle and handle.accepted:
                    handle.cancel_goal_async()
            sent.add_done_callback(cancel_late)
            node.get_logger().error("Gripper goal acceptance timeout; late goal will be canceled")
            return False
        time.sleep(0.05)
    handle = sent.result()
    if not handle or not handle.accepted:
        node.get_logger().error("Gripper goal rejected")
        return False
    result = handle.get_result_async()
    deadline = time.monotonic() + timeout
    while not result.done():
        if time.monotonic() >= deadline:
            cancel = handle.cancel_goal_async()
            cancel_deadline = time.monotonic() + 2.0
            while not cancel.done() and time.monotonic() < cancel_deadline:
                time.sleep(0.05)
            node.get_logger().error("Gripper result timeout; cancel requested to stop the active goal")
            return False
        time.sleep(0.05)
    wrapped = result.result()
    outcome = wrapped.result
    node.last_gripper_position = outcome.position
    node.get_logger().info(
        f"Gripper result: status={wrapped.status}, position={outcome.position:.4f}, "
        f"reached={outcome.reached_goal}, stalled={outcome.stalled}")
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
        return False
    if outcome.reached_goal:
        return True
    # Stable contact can stop closure before the requested empty-jaw position.
    # A stalled opening is a failure, even if the controller allows stalling.
    if target <= 0.01 and outcome.stalled:
        node.get_logger().info("Gripper closure stopped on contact; continuing pick sequence")
        return True
    node.get_logger().error("Gripper did not reach the requested opening")
    return False
