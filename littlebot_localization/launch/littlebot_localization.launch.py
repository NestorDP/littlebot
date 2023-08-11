from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_littlebot_ekf = PathJoinSubstitution(
        [FindPackageShare('littlebot_localization'),
         'config', 'littlebot_localization.yaml'],
    )

    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[config_littlebot_ekf],
        )

    # if (primary_imu_enable.perform(lc)) == 'true':
    #     config_imu_filter = PathJoinSubstitution(
    #         [FindPackageShare('littlebot_control'),
    #         'config',
    #         'imu_filter.yaml'],
    #     )
    #     node_imu_filter = Node(
    #         package='imu_filter_madgwick',
    #         executable='imu_filter_madgwick_node',
    #         name='imu_filter',
    #         output='screen',
    #         parameters=[config_imu_filter]
    #     )
    #     ld.add_action(node_imu_filter)

    ld = LaunchDescription()
    ld.add_action(node_ekf)

    return ld
