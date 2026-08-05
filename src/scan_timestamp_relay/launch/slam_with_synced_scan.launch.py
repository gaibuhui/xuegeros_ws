import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam_parameters = os.path.join(
        get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")
    return LaunchDescription([
        Node(
            package="scan_timestamp_relay",
            executable="scan_timestamp_relay",
            name="scan_timestamp_relay",
            output="screen",
        ),
        Node(
            package="slam_gmapping",
            executable="slam_gmapping",
            name="slam_gmapping_synced",
            output="screen",
            parameters=[slam_parameters],
            remappings=[("scan", "scan_synced")],
        ),
    ])
