from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('slam_gmapping'),
                'params',
                'slam_gmapping.yaml')],
            remappings=[('scan', '/scan_synced')]),
    ])
