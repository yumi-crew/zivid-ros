from zivid_interfaces.srv import Capture

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from lifecycle_msgs.msg import (
    State, Transition, TransitionDescription, TransitionEvent)
from lifecycle_msgs.srv import (
    ChangeState, GetAvailableStates, GetAvailableTransitions, GetState)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('zivid_camera_client')

    available_transitions_client = node.create_client(
        GetAvailableTransitions, '/zivid_camera/get_available_transitions')
    while not available_transitions_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    req = GetAvailableTransitions.Request()
    future = available_transitions_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        available_transitions = future.result().available_transitions
        for available_transition in available_transitions:
            node.get_logger().info("{}".format(str(available_transition)))
    else:
        node.get_logger().error('Exception while calling service: %r' % future.exception())

    change_state_client = node.create_client(
        ChangeState, '/zivid_camera/change_state')
    while not change_state_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    req = ChangeState.Request()
    req.transition = Transition()
    req.transition.id = Transition.TRANSITION_CONFIGURE

    future = change_state_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('Camera configured')
    else:
        node.get_logger().error('Exception while calling service: %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
