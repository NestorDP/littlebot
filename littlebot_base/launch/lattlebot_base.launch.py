
"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='composition',
                    node_plugin='composition::Talker',
                    node_name='talker'),
                ComposableNode(
                    package='littlebot_base',
                    plugin='littlebot_base::Listener',
                    name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
