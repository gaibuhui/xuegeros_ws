import math
import unittest

from xuegecar_obstacle_avoidance.controller import AvoidanceConfig
from xuegecar_obstacle_avoidance.controller import AvoidanceController
from xuegecar_obstacle_avoidance.controller import MotionState
from xuegecar_obstacle_avoidance.controller import sector_minimum


def make_scan(front=2.0, left=2.0, right=2.0, rear=2.0):
    angle_min = -math.pi
    increment = math.pi / 18
    ranges = []
    for index in range(37):
        angle = angle_min + index * increment
        if abs(angle) <= math.radians(30):
            ranges.append(front)
        elif abs(abs(angle) - math.pi) <= math.radians(30):
            ranges.append(rear)
        elif angle > 0:
            ranges.append(left)
        else:
            ranges.append(right)
    return ranges, angle_min, increment


def update(controller, now=0.0, front=2.0, left=2.0, right=2.0, rear=2.0):
    ranges, angle_min, increment = make_scan(front, left, right, rear)
    return controller.update_scan(ranges, angle_min, increment, 0.05, 10.0, now)


class TestAvoidanceController(unittest.TestCase):
    def test_sector_minimum_ignores_non_finite_and_out_of_range_values(self):
        minimum = sector_minimum(
            [float("nan"), float("inf"), 0.01, 0.8],
            0.0,
            0.1,
            0.0,
            0.4,
            0.05,
            10.0,
        )

        self.assertEqual(minimum, 0.8)

    def test_clear_front_drives_forward(self):
        controller = AvoidanceController(AvoidanceConfig())

        command = update(controller)

        self.assertIs(controller.state, MotionState.FORWARD)
        self.assertAlmostEqual(command.linear_x, 0.10)
        self.assertEqual(command.angular_z, 0.0)

    def test_obstacle_prefers_clearer_left_sector(self):
        controller = AvoidanceController(AvoidanceConfig())

        command = update(controller, front=0.2, left=1.5, right=0.4)

        self.assertIs(controller.state, MotionState.TURNING)
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_obstacle_prefers_clearer_right_sector(self):
        controller = AvoidanceController(AvoidanceConfig())

        command = update(controller, front=0.2, left=0.4, right=1.5)

        self.assertIs(controller.state, MotionState.TURNING)
        self.assertEqual(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)

    def test_missing_side_measurements_use_deterministic_left_turn(self):
        controller = AvoidanceController(AvoidanceConfig())
        ranges, angle_min, increment = make_scan(front=0.2)
        for index, distance in enumerate(ranges):
            angle = angle_min + index * increment
            if abs(angle) > math.radians(30):
                ranges[index] = float("inf")

        command = controller.update_scan(ranges, angle_min, increment, 0.05, 10.0, 0.0)

        self.assertIs(controller.state, MotionState.TURNING)
        self.assertGreater(command.angular_z, 0.0)

    def test_turning_resumes_only_after_clearance_is_restored(self):
        controller = AvoidanceController(AvoidanceConfig())

        update(controller, now=0.0, front=0.2)
        still_turning = update(controller, now=0.5, front=0.4)
        forward = update(controller, now=1.0, front=0.6)

        self.assertNotEqual(still_turning.angular_z, 0.0)
        self.assertIs(controller.state, MotionState.FORWARD)
        self.assertAlmostEqual(forward.linear_x, 0.10)

    def test_missing_front_measurements_stops_vehicle(self):
        controller = AvoidanceController(AvoidanceConfig())
        ranges, angle_min, increment = make_scan()
        controller.update_scan(ranges, angle_min, increment, 0.05, 10.0, 0.0)

        command = controller.update_scan(
            [float("inf")] * len(ranges), angle_min, increment, 0.05, 10.0, 1.0
        )

        self.assertIs(controller.state, MotionState.STOPPED)
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_recovery_tries_both_turn_directions_before_reversing(self):
        controller = AvoidanceController(
            AvoidanceConfig(turn_timeout=1.0, reverse_duration=0.5)
        )

        first_turn = update(controller, now=0.0, front=0.2, left=1.0, right=0.4)
        opposite_turn = update(controller, now=1.1, front=0.2, left=1.0, right=0.4)
        reverse = update(controller, now=2.2, front=0.2, left=1.0, right=0.4, rear=1.0)

        self.assertGreater(first_turn.angular_z, 0.0)
        self.assertLess(opposite_turn.angular_z, 0.0)
        self.assertIs(controller.state, MotionState.REVERSING)
        self.assertLess(reverse.linear_x, 0.0)

    def test_reverse_returns_to_direction_search_after_timeout(self):
        controller = AvoidanceController(
            AvoidanceConfig(turn_timeout=1.0, reverse_duration=0.5)
        )

        update(controller, now=0.0, front=0.2, rear=1.0)
        update(controller, now=1.1, front=0.2, rear=1.0)
        update(controller, now=2.2, front=0.2, rear=1.0)
        command = update(controller, now=2.8, front=0.2, left=1.0, rear=1.0)

        self.assertIs(controller.state, MotionState.TURNING)
        self.assertEqual(command.linear_x, 0.0)
        self.assertNotEqual(command.angular_z, 0.0)

    def test_all_blocked_directions_stop_the_vehicle(self):
        controller = AvoidanceController(AvoidanceConfig())

        command = update(controller, front=0.2, left=0.2, right=0.2, rear=0.2)

        self.assertIs(controller.state, MotionState.STOPPED)
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_stuck_report_reverses_when_rear_is_clear(self):
        controller = AvoidanceController(AvoidanceConfig(reverse_duration=1.0))

        update(controller, now=0.0, front=2.0, rear=1.0)
        recovery = controller.report_stuck(now=2.0)
        sustained_recovery = update(controller, now=2.5, front=2.0, rear=1.0)

        self.assertIs(controller.state, MotionState.REVERSING)
        self.assertLess(recovery.linear_x, 0.0)
        self.assertLess(sustained_recovery.linear_x, 0.0)

    def test_stuck_report_reverses_when_rear_is_unknown(self):
        controller = AvoidanceController(AvoidanceConfig(turn_timeout=1.0))

        update(controller, now=0.0, front=2.0, rear=float("inf"))
        recovery = controller.report_stuck(now=2.0)
        sustained_recovery = update(
            controller, now=2.5, front=2.0, rear=float("inf")
        )

        self.assertIs(controller.state, MotionState.REVERSING)
        self.assertLess(recovery.linear_x, 0.0)
        self.assertLess(sustained_recovery.linear_x, 0.0)

    def test_turn_recovery_reverses_when_rear_is_unknown(self):
        controller = AvoidanceController(AvoidanceConfig(turn_timeout=1.0))

        update(controller, now=0.0, front=0.2, rear=float("inf"))
        update(controller, now=1.1, front=0.2, rear=float("inf"))
        recovery = update(controller, now=2.2, front=0.2, rear=float("inf"))

        self.assertIs(controller.state, MotionState.REVERSING)
        self.assertLess(recovery.linear_x, 0.0)

    def test_turn_recovery_does_not_reverse_into_known_close_rear_obstacle(self):
        controller = AvoidanceController(AvoidanceConfig(turn_timeout=1.0))

        update(controller, now=0.0, front=0.2, rear=0.4)
        update(controller, now=1.1, front=0.2, rear=0.4)
        recovery = update(controller, now=2.2, front=0.2, rear=0.4)

        self.assertIs(controller.state, MotionState.TURNING)
        self.assertEqual(recovery.linear_x, 0.0)
        self.assertNotEqual(recovery.angular_z, 0.0)

    def test_stuck_recovery_reverses_then_commits_to_an_escape_turn(self):
        controller = AvoidanceController(
            AvoidanceConfig(reverse_duration=1.0, escape_turn_duration=2.4)
        )

        update(controller, now=0.0, front=2.0, rear=1.0)
        controller.report_stuck(now=0.1)
        escape_turn = update(controller, now=1.2, front=2.0, rear=1.0)

        self.assertIs(controller.state, MotionState.TURNING)
        self.assertLess(escape_turn.angular_z, 0.0)
        still_escaping = update(controller, now=3.0, front=2.0, rear=1.0)
        self.assertLess(still_escaping.angular_z, 0.0)

    def test_escape_turn_directions_alternate_between_recoveries(self):
        controller = AvoidanceController(AvoidanceConfig(escape_turn_duration=1.0))

        update(controller, now=0.0, front=2.0, rear=float("inf"))
        controller.report_stuck(now=0.1)
        first = controller.report_stuck(now=0.2)
        controller.report_stuck(now=0.3)
        second = controller.report_stuck(now=0.4)

        self.assertLess(first.angular_z, 0.0)
        self.assertGreater(second.angular_z, 0.0)
