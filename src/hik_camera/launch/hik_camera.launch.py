from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera',
            output='screen'
        )
    ])
