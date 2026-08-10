"""ROS 2 node that continuously drives forward while avoiding obstacles."""

from __future__ import annotations

import math
import time
from typing import Any, Optional, Tuple

import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

from xuegecar_obstacle_avoidance.controller import AvoidanceConfig
from xuegecar_obstacle_avoidance.controller import AvoidanceController
from xuegecar_obstacle_avoidance.controller import MotionState
from xuegecar_obstacle_avoidance.controller import VelocityCommand
from xuegecar_obstacle_avoidance.controller import median_scan_change
from xuegecar_obstacle_avoidance.vision import angular_distance
from xuegecar_obstacle_avoidance.vision import median_optical_flow
from xuegecar_obstacle_avoidance.vision import yaw_from_quaternion


class ObstacleAvoidanceNode(Node):
    """Publish exclusive cmd_vel commands from the latest safe laser scan."""

    def __init__(self) -> None:
        super().__init__("obstacle_avoidance")
        self.declare_parameter("scan_topic", "/scan_synced")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("camera_topic", "/camera/image_raw/compressed")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("forward_speed", 0.10)
        self.declare_parameter("reverse_speed", 0.08)
        self.declare_parameter("turn_speed", 0.60)
        self.declare_parameter("stop_distance", 0.35)
        self.declare_parameter("clear_distance", 0.50)
        self.declare_parameter("front_sector_half_angle_deg", 30.0)
        self.declare_parameter("side_sector_start_angle_deg", 30.0)
        self.declare_parameter("side_sector_end_angle_deg", 100.0)
        self.declare_parameter("rear_sector_half_angle_deg", 30.0)
        self.declare_parameter("turn_timeout", 2.0)
        self.declare_parameter("reverse_duration", 1.0)
        self.declare_parameter("escape_turn_duration", 2.4)
        self.declare_parameter("scan_timeout", 0.50)
        self.declare_parameter("control_frequency", 10.0)
        self.declare_parameter("stuck_timeout", 2.0)
        self.declare_parameter("stuck_distance", 0.02)
        self.declare_parameter("scan_stuck_distance", 0.04)
        self.declare_parameter("scan_stuck_max_range", 3.0)
        self.declare_parameter("scan_stuck_min_samples", 30)
        self.declare_parameter("vision_stuck_distance", 3.0)
        self.declare_parameter("vision_min_features", 20)
        self.declare_parameter("imu_turn_angle_deg", 8.0)
        self.declare_parameter("imu_turn_timeout", 2.0)

        config = AvoidanceConfig(
            forward_speed=self._float_parameter("forward_speed"),
            reverse_speed=self._float_parameter("reverse_speed"),
            turn_speed=self._float_parameter("turn_speed"),
            stop_distance=self._float_parameter("stop_distance"),
            clear_distance=self._float_parameter("clear_distance"),
            front_sector_half_angle=math.radians(
                self._float_parameter("front_sector_half_angle_deg")
            ),
            side_sector_start_angle=math.radians(
                self._float_parameter("side_sector_start_angle_deg")
            ),
            side_sector_end_angle=math.radians(
                self._float_parameter("side_sector_end_angle_deg")
            ),
            rear_sector_half_angle=math.radians(
                self._float_parameter("rear_sector_half_angle_deg")
            ),
            turn_timeout=self._float_parameter("turn_timeout"),
            reverse_duration=self._float_parameter("reverse_duration"),
            escape_turn_duration=self._float_parameter("escape_turn_duration"),
        )
        self._controller = AvoidanceController(config)
        self._scan_timeout = self._float_parameter("scan_timeout")
        control_frequency = self._float_parameter("control_frequency")
        self._stuck_timeout = self._float_parameter("stuck_timeout")
        self._stuck_distance = self._float_parameter("stuck_distance")
        self._scan_stuck_distance = self._float_parameter("scan_stuck_distance")
        self._scan_stuck_max_range = self._float_parameter("scan_stuck_max_range")
        self._scan_stuck_min_samples = int(
            self.get_parameter("scan_stuck_min_samples").value
        )
        self._vision_stuck_distance = self._float_parameter("vision_stuck_distance")
        self._vision_min_features = int(
            self.get_parameter("vision_min_features").value
        )
        self._imu_turn_angle = math.radians(
            self._float_parameter("imu_turn_angle_deg")
        )
        self._imu_turn_timeout = self._float_parameter("imu_turn_timeout")
        if (
            self._scan_timeout <= 0.0
            or control_frequency <= 0.0
            or self._stuck_timeout <= 0.0
            or self._stuck_distance <= 0.0
            or self._scan_stuck_distance <= 0.0
            or self._scan_stuck_max_range <= 0.0
            or self._scan_stuck_min_samples <= 0
            or self._vision_stuck_distance <= 0.0
            or self._vision_min_features <= 0
            or self._imu_turn_angle <= 0.0
            or self._imu_turn_timeout <= 0.0
        ):
            raise ValueError("safety timeouts and distances must be positive")

        scan_topic = self._string_parameter("scan_topic")
        cmd_vel_topic = self._string_parameter("cmd_vel_topic")
        odom_topic = self._string_parameter("odom_topic")
        camera_topic = self._string_parameter("camera_topic")
        imu_topic = self._string_parameter("imu_topic")
        self._publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )
        self._camera_subscription = self.create_subscription(
            CompressedImage,
            camera_topic,
            self._camera_callback,
            qos_profile_sensor_data,
        )
        self._imu_subscription = self.create_subscription(
            Imu,
            imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self._last_scan_time = None
        self._latest_odom_position: Optional[Tuple[float, float]] = None
        self._motion_origin: Optional[Tuple[float, float]] = None
        self._motion_started_at: Optional[float] = None
        self._latest_scan_ranges: Optional[Tuple[float, ...]] = None
        self._latest_scan_range_min: Optional[float] = None
        self._scan_origin: Optional[Tuple[float, ...]] = None
        self._latest_camera_frame: Optional[np.ndarray] = None
        self._vision_origin: Optional[np.ndarray] = None
        self._latest_yaw: Optional[float] = None
        self._turn_origin_yaw: Optional[float] = None
        self._turn_started_at: Optional[float] = None
        self._timer = self.create_timer(1.0 / control_frequency, self._control_tick)
        self.get_logger().info(
            f"Obstacle avoidance controls {cmd_vel_topic} from {scan_topic}; "
            "stop other cmd_vel publishers."
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _string_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f"{name} must not be empty")
        return value

    def _scan_callback(self, scan: LaserScan) -> None:
        command = self._controller.update_scan(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            time.monotonic(),
        )
        self._last_scan_time = self.get_clock().now()
        self._latest_scan_ranges = tuple(scan.ranges)
        self._latest_scan_range_min = scan.range_min
        self._publish(command)

    def _odom_callback(self, odom: Odometry) -> None:
        position = odom.pose.pose.position
        self._latest_odom_position = (position.x, position.y)

    def _camera_callback(self, image: CompressedImage) -> None:
        encoded = np.frombuffer(image.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
        if frame is not None:
            self._latest_camera_frame = frame

    def _imu_callback(self, imu: Imu) -> None:
        orientation = imu.orientation
        self._latest_yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def _control_tick(self) -> None:
        if self._last_scan_time is None:
            self._publish(self._controller.stop())
            self._reset_motion_monitor()
            self._reset_turn_monitor()
            return

        elapsed = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
        if elapsed > self._scan_timeout:
            if self._controller.state is not MotionState.STOPPED:
                self.get_logger().warn("LaserScan timed out; stopping the vehicle.")
            self._publish(self._controller.stop())
            self._reset_motion_monitor()
            self._reset_turn_monitor()
            return

        now = time.monotonic()
        if self._controller.command.linear_x != 0.0:
            self._reset_turn_monitor()
            self._detect_stuck(now)
        elif self._controller.command.angular_z != 0.0:
            self._reset_motion_monitor()
            self._detect_stuck_turn(now)
        else:
            self._reset_motion_monitor()
            self._reset_turn_monitor()
        self._publish(self._controller.command)

    def _detect_stuck(self, now: float) -> None:
        command = self._controller.command
        if command.linear_x == 0.0:
            self._reset_motion_monitor()
            return

        if self._motion_started_at is None:
            self._motion_origin = self._latest_odom_position
            self._motion_started_at = now
            self._scan_origin = self._latest_scan_ranges
            self._vision_origin = self._latest_camera_frame
            return

        movement = self._odometry_movement_since_motion_started()
        odometry_stuck = (
            movement is not None and movement < self._stuck_distance
        )
        scan_change = self._scan_change_since_motion_started()
        lidar_stuck = (
            scan_change is not None and scan_change < self._scan_stuck_distance
        )
        vision_change = self._vision_change_since_motion_started()
        vision_stuck = (
            vision_change is not None and vision_change < self._vision_stuck_distance
        )
        if (
            movement is not None
            and movement >= self._stuck_distance
            and not lidar_stuck
            and not vision_stuck
        ):
            self._motion_origin = self._latest_odom_position
            self._motion_started_at = now
            self._scan_origin = self._latest_scan_ranges
            self._vision_origin = self._latest_camera_frame
            return

        if (
            now - self._motion_started_at >= self._stuck_timeout
            and (odometry_stuck or lidar_stuck or vision_stuck)
        ):
            reason = self._stuck_reason(
                movement, odometry_stuck, lidar_stuck, vision_stuck
            )
            self.get_logger().warn(f"Stuck recovery: {reason} while translating.")
            self._controller.report_stuck(now)
            self._motion_origin = self._latest_odom_position
            self._motion_started_at = now
            self._scan_origin = self._latest_scan_ranges
            self._vision_origin = self._latest_camera_frame

    def _odometry_movement_since_motion_started(self) -> Optional[float]:
        if self._motion_origin is None or self._latest_odom_position is None:
            return None
        return math.dist(self._latest_odom_position, self._motion_origin)

    def _detect_stuck_turn(self, now: float) -> None:
        if self._latest_yaw is None:
            return
        if self._turn_origin_yaw is None:
            self._turn_origin_yaw = self._latest_yaw
            self._turn_started_at = now
            return
        yaw_change = angular_distance(self._turn_origin_yaw, self._latest_yaw)
        if yaw_change >= self._imu_turn_angle:
            self._turn_origin_yaw = self._latest_yaw
            self._turn_started_at = now
            return
        if now - self._turn_started_at >= self._imu_turn_timeout:
            self.get_logger().warn("Stuck recovery: IMU yaw unchanged while turning.")
            self._controller.report_stuck(now)
            self._turn_origin_yaw = self._latest_yaw
            self._turn_started_at = now

    def _scan_change_since_motion_started(self) -> Optional[float]:
        if (
            self._scan_origin is None
            or self._latest_scan_ranges is None
            or self._latest_scan_range_min is None
        ):
            return None
        return median_scan_change(
            self._scan_origin,
            self._latest_scan_ranges,
            self._latest_scan_range_min,
            self._scan_stuck_max_range,
            self._scan_stuck_min_samples,
        )

    def _vision_change_since_motion_started(self) -> Optional[float]:
        if self._vision_origin is None or self._latest_camera_frame is None:
            return None
        return median_optical_flow(
            self._vision_origin,
            self._latest_camera_frame,
            self._vision_min_features,
        )

    def _stuck_reason(
        self,
        movement: Optional[float],
        odometry_stuck: bool,
        lidar_stuck: bool,
        vision_stuck: bool,
    ) -> str:
        reasons = []
        if odometry_stuck:
            reasons.append("no odometry movement")
        if lidar_stuck:
            reasons.append("lidar scene unchanged")
        if vision_stuck:
            reasons.append("camera scene unchanged")
        return ", ".join(reasons)

    def _reset_motion_monitor(self) -> None:
        self._motion_origin = None
        self._motion_started_at = None
        self._scan_origin = None
        self._vision_origin = None

    def _reset_turn_monitor(self) -> None:
        self._turn_origin_yaw = None
        self._turn_started_at = None

    def _publish(self, command: VelocityCommand) -> None:
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        self._publisher.publish(message)

    def stop(self) -> None:
        """Publish an explicit brake command before the node is destroyed."""
        self._publish(self._controller.stop())


def main(args: Any = None) -> None:
    """Run obstacle avoidance until ROS receives a shutdown request."""
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
