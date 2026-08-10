"""Camera and IMU motion measurements for chassis-stuck detection."""

import math
from typing import Optional

import cv2
import numpy as np


def median_optical_flow(
    reference: np.ndarray,
    current: np.ndarray,
    minimum_features: int,
) -> Optional[float]:
    """Return median tracked-feature motion in pixels, if enough features track."""
    if reference.shape != current.shape:
        return None
    points = cv2.goodFeaturesToTrack(
        reference,
        maxCorners=160,
        qualityLevel=0.01,
        minDistance=7.0,
        blockSize=7,
    )
    if points is None:
        return None
    next_points, status, error = cv2.calcOpticalFlowPyrLK(
        reference,
        current,
        points,
        None,
        winSize=(21, 21),
        maxLevel=3,
    )
    if next_points is None or status is None or error is None:
        return None
    valid = (status.reshape(-1) == 1) & (error.reshape(-1) < 20.0)
    if int(np.count_nonzero(valid)) < minimum_features:
        return None
    movement = np.linalg.norm(
        next_points.reshape(-1, 2)[valid] - points.reshape(-1, 2)[valid], axis=1
    )
    return float(np.median(movement))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Convert a ROS orientation quaternion to planar yaw in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angular_distance(first: float, second: float) -> float:
    """Return the absolute shortest yaw distance between two angles."""
    return abs((second - first + math.pi) % (2.0 * math.pi) - math.pi)
