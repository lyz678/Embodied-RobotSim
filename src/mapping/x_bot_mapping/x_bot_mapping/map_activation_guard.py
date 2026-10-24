"""Recover map-server startup when a lifecycle service response is lost."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import GetState, ChangeState
from nav_msgs.msg import OccupancyGrid


def required_transition(state):
    return {State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
            State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE}.get(state)


class MapActivationGuard(Node):
    def __init__(self):
        super().__init__('map_activation_guard')
        self.received = False
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state = self.create_client(GetState, '/map_server/get_state')
        self.change = self.create_client(ChangeState, '/map_server/change_state')

    def map(self, msg):
        self.received = msg.header.frame_id == 'map' and bool(msg.info.width * msg.info.height)

    def call(self, client, request):
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.)
        if not future.done():
            client.remove_pending_request(future)
            future.cancel()
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warn('Map lifecycle service: ' + str(exc))
            return None

    def step(self):
        # Always re-read state. A timed-out transition can have succeeded.
        response = self.call(self.state, GetState.Request())
        if response is None or self.received:
            return
        transition = required_transition(response.current_state.id)
        if transition is not None:
            request = ChangeState.Request()
            request.transition.id = transition
            self.get_logger().warn(f'Recovering map_server from {response.current_state.label}')
            self.call(self.change, request)

    def run(self, timeout=180., grace=8.):
        started = time.monotonic()
        while rclpy.ok() and time.monotonic() - started < timeout:
            rclpy.spin_once(self, timeout_sec=.2)
            if self.received:
                self.get_logger().info('Map server is publishing /map')
                return True
            if time.monotonic() - started >= grace:
                self.step()
                # Give the normal lifecycle manager time to finish a transition.
                end = time.monotonic() + .5
                while not self.received and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=.1)
        self.get_logger().error('Map server did not publish /map within startup deadline')
        return False


def main():
    rclpy.init()
    node = MapActivationGuard()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)
