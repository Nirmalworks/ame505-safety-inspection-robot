"""Small deterministic ray-cast fixture for testing the complete scan frontend."""

import math
import numpy as np


def base_pose(elapsed):
    moving = max(elapsed - 3., 0.)
    return 0.05 * moving, 0.02 * math.sin(moving), 0.04 * moving


def room_scan(pose, count=360, lidar_x=0.096, lidar_yaw=math.pi):
    """Laser mounted backwards, at the measured approximate JetAuto x offset."""
    x, y, yaw = pose
    origin = np.array([x + lidar_x * math.cos(yaw), y + lidar_x * math.sin(yaw)])
    angles = np.linspace(-math.pi / 2, math.pi / 2, count)
    directions = np.column_stack((np.cos(angles + yaw + lidar_yaw),
                                  np.sin(angles + yaw + lidar_yaw)))
    ranges = np.full(count, np.inf)
    for axis, boundary, low, high in ((0, -2., -1.5, 2.), (0, 3., -1.5, 2.),
                                       (1, -1.5, -2., 3.), (1, 2., -2., 3.)):
        component = directions[:, axis]
        distance = np.divide(boundary - origin[axis], component,
                             out=np.full(count, np.inf), where=np.abs(component) > 1e-10)
        other = origin[1 - axis] + distance * directions[:, 1 - axis]
        good = (distance > 0) & (other >= low) & (other <= high)
        ranges = np.minimum(ranges, np.where(good, distance, np.inf))
    return ranges
