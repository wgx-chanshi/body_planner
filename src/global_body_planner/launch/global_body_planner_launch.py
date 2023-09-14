from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='global_body_planner',
            executable='main_planner',
            output='screen'
        )
    ])
