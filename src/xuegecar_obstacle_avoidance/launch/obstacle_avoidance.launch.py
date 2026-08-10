"""Launch xuegecar's standalone LaserScan obstacle-avoidance controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        ("scan_topic", "/scan_synced", "LaserScan topic to monitor."),
        ("cmd_vel_topic", "/cmd_vel", "Exclusive Twist command topic."),
        ("odom_topic", "/odom", "Odometry topic used to detect a stuck chassis."),
        ("camera_topic", "/camera/image_raw/compressed", "Camera topic for visual motion checks."),
        ("imu_topic", "/imu", "IMU topic used to verify turning motion."),
        ("forward_speed", "0.10", "Forward speed in metres per second."),
        ("reverse_speed", "0.08", "Reverse speed in metres per second."),
        ("turn_speed", "0.60", "In-place turn speed in radians per second."),
        ("stop_distance", "0.35", "Obstacle stop distance in metres."),
        ("clear_distance", "0.50", "Required forward clearance in metres."),
        ("front_sector_half_angle_deg", "30.0", "Front sector half-angle in degrees."),
        ("side_sector_start_angle_deg", "30.0", "Side sector inner angle in degrees."),
        ("side_sector_end_angle_deg", "100.0", "Side sector outer angle in degrees."),
        ("rear_sector_half_angle_deg", "30.0", "Rear sector half-angle in degrees."),
        ("turn_timeout", "2.0", "Seconds before trying another turn direction."),
        ("reverse_duration", "1.0", "Maximum reverse duration in seconds."),
        ("escape_turn_duration", "2.4", "Committed turn duration after a stuck recovery."),
        ("scan_timeout", "0.50", "Maximum allowed LaserScan age in seconds."),
        ("control_frequency", "10.0", "Twist publish rate in hertz."),
        ("stuck_timeout", "2.0", "No-motion duration before recovery starts."),
        ("stuck_distance", "0.02", "Minimum odometry movement in metres."),
        ("scan_stuck_distance", "0.04", "Minimum nearby lidar scene change in metres."),
        ("scan_stuck_max_range", "3.0", "Maximum lidar range used for skid detection."),
        ("scan_stuck_min_samples", "30", "Minimum valid nearby lidar beams for skid detection."),
        ("vision_stuck_distance", "3.0", "Minimum camera feature motion in pixels."),
        ("vision_min_features", "20", "Minimum tracked camera features for skid detection."),
        ("imu_turn_angle_deg", "8.0", "Minimum IMU yaw change during a turn."),
        ("imu_turn_timeout", "2.0", "Maximum no-yaw-change duration while turning."),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in arguments
    ] + [
        Node(
            package="xuegecar_obstacle_avoidance",
            executable="obstacle_avoidance",
            name="obstacle_avoidance",
            output="screen",
            parameters=[{name: LaunchConfiguration(name) for name, _, _ in arguments}],
        ),
    ])
