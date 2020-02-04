
"""Launch a zivid_camera component in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        node_name='zivid_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='zivid_camera',
                node_plugin='zivid_camera::ZividCamera',
                node_name='zivid_camera',
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
