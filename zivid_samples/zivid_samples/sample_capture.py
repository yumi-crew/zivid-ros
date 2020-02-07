# Copyright (c) 2020 Norwegian University of Science and Technology
# Copyright (c) 2019, Zivid AS
# Use of this source code is governed by the BSD 3-Clause license, see LICENSE

from zivid_interfaces.srv import Capture

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


from lifecycle_msgs.msg import (
    State, Transition, TransitionDescription, TransitionEvent)
from lifecycle_msgs.srv import (
    ChangeState, GetAvailableStates, GetAvailableTransitions, GetState)


class ZividCameraClient(Node):

    def __init__(self):
        super().__init__('zivid_camera_client')

        self._get_available_transitions_client = self.create_client(
            GetAvailableTransitions, '/zivid_camera/get_available_transitions')
        while not self._get_available_transitions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._get_camera_available_transitions()

        self._change_state_client = self.create_client(
            ChangeState, '/zivid_camera/change_state')
        while not self._change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._capture_client = self.create_client(Capture, '/capture')
        while not self._capture_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Capture service not available, waiting again...')

        self._capture_timer = self.create_timer(10.0, self.capture)

    def _get_camera_available_transitions(self):
        req = GetAvailableTransitions.Request()
        future = self._get_available_transitions_client.call_async(req)
        while not future.done():
            print("still not done")
            pass
        try:
            available_transitions = future.result().available_transitions
            for available_transition in available_transitions:
                self.get_logger().info("{}".format(str(available_transition)))
        except:
            self.get_logger().info('Failed')
        else:
            self.get_logger().info('Succeeded')


    def _change_camera_state(self, transition_id):
        req = ChangeState.Request()
        req.transition = Transition()
        req.transition.id = transition_id
        future = self._change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            result = future.result()
        except:
            self.get_logger().info('State transition failed')
        else:
            self.get_logger().info('State transition succeeded')

    def configure_camera(self):
        self.get_logger().info("Configuring camera")
        return self._change_camera_state(Transition.TRANSITION_CONFIGURE)

    def activate_camera(self):
        self.get_logger().info("Activating camera")
        return self._change_camera_state(Transition.TRANSITION_ACTIVATE)

    def capture(self):
        req = Capture.Request()
        future = self._capture_client.call_async(req)
        try:
            future.result()
        except Exception as e:
            self.get_logger().info('Capture failed %r' % (e,))
        else:
            self.get_logger().info('Capture succeeded')


def main(args=None):
    rclpy.init(args=args)
    try:
        zivid_camera_client = ZividCameraClient()
        executor = SingleThreadedExecutor()
        executor.add_node(zivid_camera_client)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            zivid_camera_client.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
