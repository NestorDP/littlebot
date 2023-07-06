
"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='base_container',
            namespace='littlebot',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='littlebot_teleop',
                    plugin='littlebot_teleop::Teleop',
                    name='Teleop')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])