# from ament_index_python.packages import get_package_share_path

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.conditions import IfCondition, UnlessCondition
# from launch.substitutions import Command, LaunchConfiguration

# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue


# def generate_launch_description():

#     description_path = get_package_share_path('littlebot_description')
#     model_path = description_path / 'urdf/littlebot_description.urdf.xacro'
#     rviz_config_path = description_path / 'config/littlebot_description.rviz'
#     use_sim_time     = LaunchConfiguration('use_sim_time', default='false')

#     # Launch arguments
#     #-----------------------------------------------------------------------------
#     time_arg  = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false',
#         choices=['true', 'false'],
#         description='Use simulation (Gazebo) clock if true')

#     joint_gui_arg   = DeclareLaunchArgument(
#         name='joint_gui',
#         default_value='true',
#         choices=['true', 'false'],
#         description='Flag to enable joint_state_publisher_gui')

#     model_arg = DeclareLaunchArgument(
#         name='model',
#         default_value=str(urdf_model_path),
#         description='Absolute path to robot urdf file')

#     rviz_arg  = DeclareLaunchArgument(
#         name='rvizconfig',
#         default_value=str(rviz_config_path),
#         description='Absolute path to rviz config file')

#     # Parameters
#     #-----------------------------------------------------------------------------
#     robot_description = ParameterValue(
#         Command(['xacro ', LaunchConfiguration('model')]),
#         value_type=str)

#     # Nodes
#     #-----------------------------------------------------------------------------
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         parameters=[
#             {'robot_description': robot_description},
#             {'use_sim_time': use_sim_time}],
#         output='screen'
#     )

#     joint_state_publisher_node = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         condition=UnlessCondition(LaunchConfiguration('joint_gui'))
#     )

#     joint_state_publisher_gui_node = Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         condition=IfCondition(LaunchConfiguration('joint_gui'))
#     )

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', LaunchConfiguration('rvizconfig')],
#     )

#     # Return
#     #-----------------------------------------------------------------------------
#     return LaunchDescription([
#         time_arg,
#         joint_gui_arg,
#         model_arg,
#         rviz_arg,
#         joint_state_publisher_node,
#         joint_state_publisher_gui_node,
#         robot_state_publisher_node,
#         rviz_node
#     ])
