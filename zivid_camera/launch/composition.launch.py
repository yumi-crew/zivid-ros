
"""Launch a zivid_camera component in a component container."""
###########
# put here by markus 
import os
from ament_index_python.packages import get_package_share_directory
###########

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode



def generate_launch_description():
    ######put here by markus
    rviz_config_dir = os.path.join(get_package_share_directory(
        'zivid_camera'), 'rviz', 'camera_view.rviz')
    assert os.path.exists(rviz_config_dir)
    ############


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


    return launch.LaunchDescription([container,
    #########################
    # put here by markus    
        Node(package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
            )
    ########################
    ])
