from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scan_timestamp_relay',
            executable='scan_timestamp_relay',
            name='scan_timestamp_relay',
            output='screen',
        ),
    ])
