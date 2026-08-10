"""Pure obstacle-avoidance decision logic independent of ROS transport."""

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Iterable, Optional


class MotionState(Enum):
    """The motion modes published by the autonomous controller."""

    STOPPED = "stopped"
    FORWARD = "forward"
    TURNING = "turning"
    REVERSING = "reversing"


@dataclass(frozen=True)
class AvoidanceConfig:
    """Tunable safety and motion limits for obstacle avoidance."""

    forward_speed: float = 0.10
    reverse_speed: float = 0.08
    turn_speed: float = 0.60
    stop_distance: float = 0.35
    clear_distance: float = 0.50
    front_sector_half_angle: float = math.radians(30.0)
    side_sector_start_angle: float = math.radians(30.0)
    side_sector_end_angle: float = math.radians(100.0)
    rear_sector_half_angle: float = math.radians(30.0)
    turn_timeout: float = 2.0
    reverse_duration: float = 1.0
    escape_turn_duration: float = 2.4


@dataclass(frozen=True)
class VelocityCommand:
    """A planar velocity command, expressed in ROS Twist units."""

    linear_x: float = 0.0
    angular_z: float = 0.0


def sector_minimum(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    start_angle: float,
    end_angle: float,
    range_min: float,
    range_max: float,
) -> Optional[float]:
    """Return the nearest valid range in an inclusive angular sector."""
    distances = []
    for index, distance in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if start_angle <= angle <= end_angle and (
            math.isfinite(distance) and range_min <= distance <= range_max
        ):
            distances.append(distance)
    return min(distances) if distances else None


def median_scan_change(
    reference_ranges: Iterable[float],
    current_ranges: Iterable[float],
    range_min: float,
    comparison_max_range: float,
    minimum_samples: int,
) -> Optional[float]:
    """Return median range change for nearby valid beams, or None if insufficient."""
    changes = sorted(
        abs(current - reference)
        for reference, current in zip(reference_ranges, current_ranges)
        if (
            math.isfinite(reference)
            and math.isfinite(current)
            and range_min <= reference <= comparison_max_range
            and range_min <= current <= comparison_max_range
        )
    )
    if len(changes) < minimum_samples:
        return None
    midpoint = len(changes) // 2
    if len(changes) % 2:
        return changes[midpoint]
    return (changes[midpoint - 1] + changes[midpoint]) / 2.0


class AvoidanceController:
    """Select safe forward motion or an in-place turn from a laser scan."""

    def __init__(self, config: AvoidanceConfig) -> None:
        if config.stop_distance <= 0.0:
            raise ValueError("stop_distance must be positive")
        if config.clear_distance < config.stop_distance:
            raise ValueError("clear_distance must be at least stop_distance")
        if (
            config.forward_speed <= 0.0
            or config.reverse_speed <= 0.0
            or config.turn_speed <= 0.0
        ):
            raise ValueError("motion speeds must be positive")
        if not 0.0 < config.front_sector_half_angle < math.pi:
            raise ValueError("front_sector_half_angle must be between 0 and pi")
        if not 0.0 <= config.side_sector_start_angle < config.side_sector_end_angle <= math.pi:
            raise ValueError("side sector angles must be ordered within [0, pi]")
        if not 0.0 < config.rear_sector_half_angle < math.pi / 2:
            raise ValueError("rear_sector_half_angle must be between 0 and pi/2")
        if (
            config.turn_timeout <= 0.0
            or config.reverse_duration <= 0.0
            or config.escape_turn_duration <= 0.0
        ):
            raise ValueError("recovery durations must be positive")

        self.config = config
        self.state = MotionState.STOPPED
        self._turn_left = True
        self._turn_started_at: Optional[float] = None
        self._reverse_started_at: Optional[float] = None
        self._tried_opposite_turn = False
        self._force_turn_until: Optional[float] = None
        self._latest_rear: Optional[float] = None
        self._next_escape_turn_left = False

    @property
    def command(self) -> VelocityCommand:
        """Return the currently selected velocity command."""
        if self.state is MotionState.FORWARD:
            return VelocityCommand(linear_x=self.config.forward_speed)
        if self.state is MotionState.TURNING:
            direction = 1.0 if self._turn_left else -1.0
            return VelocityCommand(angular_z=direction * self.config.turn_speed)
        if self.state is MotionState.REVERSING:
            return VelocityCommand(linear_x=-self.config.reverse_speed)
        return VelocityCommand()

    def stop(self) -> VelocityCommand:
        """Immediately enter the fail-safe stopped state."""
        self.state = MotionState.STOPPED
        self._turn_started_at = None
        self._reverse_started_at = None
        self._force_turn_until = None
        return self.command

    def report_stuck(self, now: Optional[float] = None) -> VelocityCommand:
        """Start a recovery manoeuvre after commanded translation made no progress."""
        now = time.monotonic() if now is None else now
        if self.state is MotionState.REVERSING:
            self._start_escape_turn(now)
        elif self._rear_allows_escape_reverse(self._latest_rear):
            self.state = MotionState.REVERSING
            self._reverse_started_at = now
            self._force_turn_until = None
        else:
            self._start_escape_turn(now)
        return self.command

    def update_scan(
        self,
        ranges: Iterable[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        now: Optional[float] = None,
    ) -> VelocityCommand:
        """Update state from one LaserScan and return the next command."""
        now = time.monotonic() if now is None else now
        ranges = tuple(ranges)
        front = sector_minimum(
            ranges,
            angle_min,
            angle_increment,
            -self.config.front_sector_half_angle,
            self.config.front_sector_half_angle,
            range_min,
            range_max,
        )
        left = sector_minimum(
            ranges,
            angle_min,
            angle_increment,
            self.config.side_sector_start_angle,
            self.config.side_sector_end_angle,
            range_min,
            range_max,
        )
        right = sector_minimum(
            ranges,
            angle_min,
            angle_increment,
            -self.config.side_sector_end_angle,
            -self.config.side_sector_start_angle,
            range_min,
            range_max,
        )
        rear = min(
            (
                distance
                for distance in (
                    sector_minimum(
                        ranges,
                        angle_min,
                        angle_increment,
                        math.pi - self.config.rear_sector_half_angle,
                        math.pi,
                        range_min,
                        range_max,
                    ),
                    sector_minimum(
                        ranges,
                        angle_min,
                        angle_increment,
                        -math.pi,
                        -math.pi + self.config.rear_sector_half_angle,
                        range_min,
                        range_max,
                    ),
                )
                if distance is not None
            ),
            default=None,
        )
        self._latest_rear = rear

        if front is None:
            return self.stop()

        if self._fully_blocked(front, left, right, rear):
            return self.stop()

        if self.state is MotionState.REVERSING:
            self._recover_from_reverse(left, right, rear, now)
            return self.command

        if self._force_turn_until is not None:
            if now < self._force_turn_until:
                return self.command
            self._force_turn_until = None

        if front >= self.config.clear_distance:
            self.state = MotionState.FORWARD
            self._turn_started_at = None
            self._reverse_started_at = None
            self._tried_opposite_turn = False
            return self.command

        if self.state in (MotionState.STOPPED, MotionState.FORWARD):
            self._start_turn(self._prefer_left(left, right), now, False)
        elif self.state is MotionState.TURNING:
            self._recover_from_turn(left, right, rear, now)
        return self.command

    def _recover_from_turn(
        self,
        left: Optional[float],
        right: Optional[float],
        rear: Optional[float],
        now: float,
    ) -> None:
        if self._turn_started_at is None:
            self._start_turn(self._prefer_left(left, right), now, False)
            return
        if now - self._turn_started_at < self.config.turn_timeout:
            return
        if not self._tried_opposite_turn:
            self._start_turn(not self._turn_left, now, True)
            return
        if self._rear_allows_escape_reverse(rear):
            self.state = MotionState.REVERSING
            self._reverse_started_at = now
            return
        self._start_turn(self._prefer_left(left, right), now, False)

    def _recover_from_reverse(
        self,
        left: Optional[float],
        right: Optional[float],
        rear: Optional[float],
        now: float,
    ) -> None:
        if self._reverse_started_at is None:
            self._start_turn(self._prefer_left(left, right), now, False)
            return
        if rear is not None and rear < self.config.stop_distance:
            self._start_turn(self._prefer_left(left, right), now, False)
            return
        if now - self._reverse_started_at >= self.config.reverse_duration:
            self._start_escape_turn(now)

    def _start_escape_turn(self, now: float) -> None:
        """Commit to an alternating large turn so recovery changes heading."""
        self._start_turn(self._next_escape_turn_left, now, False)
        self._next_escape_turn_left = not self._next_escape_turn_left
        self._force_turn_until = now + self.config.escape_turn_duration

    def _start_turn(self, turn_left: bool, now: float, tried_opposite: bool) -> None:
        self.state = MotionState.TURNING
        self._turn_left = turn_left
        self._turn_started_at = now
        self._reverse_started_at = None
        self._tried_opposite_turn = tried_opposite

    def _fully_blocked(
        self,
        front: float,
        left: Optional[float],
        right: Optional[float],
        rear: Optional[float],
    ) -> bool:
        directions = (front, left, right, rear)
        return all(
            distance is not None and distance < self.config.stop_distance
            for distance in directions
        )

    def _rear_allows_escape_reverse(self, rear: Optional[float]) -> bool:
        """Allow a bounded escape reverse unless the rear is known to be tight."""
        return rear is None or rear >= self.config.clear_distance

    @staticmethod
    def _prefer_left(left: Optional[float], right: Optional[float]) -> bool:
        """Prefer the wider side, using left as the deterministic tie-breaker."""
        if left is None and right is None:
            return True
        if left is None:
            return False
        if right is None:
            return True
        return left >= right
