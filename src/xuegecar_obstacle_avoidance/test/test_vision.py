import math
import unittest

import cv2
import numpy as np

from xuegecar_obstacle_avoidance.vision import angular_distance
from xuegecar_obstacle_avoidance.vision import median_optical_flow
from xuegecar_obstacle_avoidance.vision import yaw_from_quaternion


class TestVisionMotion(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros((160, 200), dtype=np.uint8)
        for x in range(20, 181, 40):
            for y in range(20, 141, 30):
                cv2.circle(self.frame, (x, y), 4, 255, -1)

    def test_identical_frame_has_no_optical_flow(self):
        flow = median_optical_flow(self.frame, self.frame.copy(), 10)

        self.assertIsNotNone(flow)
        self.assertLess(flow, 0.1)

    def test_translated_frame_has_optical_flow(self):
        transform = np.float32([[1, 0, 8], [0, 1, 0]])
        translated = cv2.warpAffine(self.frame, transform, (200, 160))

        flow = median_optical_flow(self.frame, translated, 10)

        self.assertIsNotNone(flow)
        self.assertGreater(flow, 6.0)

    def test_yaw_helpers_handle_quarter_turn_and_wraparound(self):
        yaw = yaw_from_quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))

        self.assertAlmostEqual(yaw, math.pi / 2)
        self.assertAlmostEqual(angular_distance(math.radians(179), math.radians(-179)), math.radians(2))
