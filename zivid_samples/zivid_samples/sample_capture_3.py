from zivid_interfaces.srv import Capture

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from lifecycle_msgs.msg import (
    State, Transition, TransitionDescription, TransitionEvent)
from lifecycle_msgs.srv import (
    ChangeState, GetAvailableStates, GetAvailableTransitions, GetState)


class ZividCameraClient:
    def __init__(self):
        self._node = rclpy.create_node("zivid_camera_client")

        self._available_transitions_client = self._node.create_client(
            GetAvailableTransitions, '/zivid_camera/get_available_transitions')
        while not self._available_transitions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._change_state_client = self._node.create_client(
            ChangeState, '/zivid_camera/change_state')
        while not self._change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._capture_client = None

    def available_transitions(self):
        req = GetAvailableTransitions.Request()
        future = self._available_transitions_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result()

    def _change_camera_state(self, transition_id):
        req = ChangeState.Request()
        req.transition = Transition()
        req.transition.id = transition_id
        future = self._change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result()

    def configure(self):
        return self._change_camera_state(Transition.TRANSITION_CONFIGURE)

    def activate(self):
        return self._change_camera_state(Transition.TRANSITION_ACTIVATE)

    def deactivate(self):
        return self._change_camera_state(Transition.TRANSITION_DEACTIVATE)

    def cleanup(self):
        return self._change_camera_state(Transition.TRANSITION_CLEANUP)

    def shutdown(self):
        return self._change_camera_state(Transition.TRANSITION_INACTIVE_SHUTDOWN)

    def capture(self):
        if self._capture_client is None:
            self._capture_client = self._node.create_client(
                Capture, '/capture')
            while not self._capture_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Capture service not available, waiting again...')
        req = Capture.Request()
        future = self._capture_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result()

    def get_logger(self):
        return self._node.get_logger()


def main(args=None):
    rclpy.init(args=args)

    try:
        zivid_camera_client = ZividCameraClient()
        zivid_camera_client.configure()
        zivid_camera_client.activate()
        zivid_camera_client.capture()
        zivid_camera_client.deactivate()
        zivid_camera_client.cleanup()
        # zivid_camera_client.shutdown()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
