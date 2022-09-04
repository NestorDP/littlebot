
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
                    package='composition',
                    plugin='composition::Talker',
                    name='talker'),
                ComposableNode(
                    package='littlebot_base',
                    plugin='littlebot_base::Listener',
                    name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
