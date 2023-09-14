import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    state_config_file_name = 'config/robot_state.rviz'

    state_config = os.path.join(
        get_package_share_directory('robot_visualization'),
        state_config_file_name)

    return LaunchDescription([
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_desc},
        #                 {'publish_frequency': 200.0}],
        #     arguments=[urdf_path], remappings=[
        #         ('/joint_states', '/rviz_debugger/joint_states')]
        # ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='robot_state',
            output='screen',
            arguments=['-d', state_config],

        ),


    ])
