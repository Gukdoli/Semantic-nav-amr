"""Pure-function approach-pose geometry.

Deliberately free of ROS imports so the goal math can be unit tested standalone
(see test/test_approach.py). The node layer (goal_commander_node.py) supplies
object and robot positions already resolved to the map frame via tf2, and wraps
the returned yaw into a quaternion / PoseStamped.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple


# Below this object<->robot distance the direction is undefined; fall back to a
# fixed heading rather than dividing by ~0.
_MIN_SEPARATION = 1e-3


def compute_approach_pose(
    obj_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    approach_distance: float,
) -> Tuple[float, float, float]:
    """Approach pose `approach_distance` metres from the object, toward the robot.

    `obj_xy` and `robot_xy` MUST be in the same frame (the map frame, as used by
    the node). Returns (x, y, yaw) in that frame: a point offset from the object
    along the object->robot direction, with yaw pointing from that point back at
    the object (so the robot faces the object on arrival).

    If the robot is essentially on top of the object (separation < ~1e-3 m) the
    direction is undefined, so we fall back to approaching from the +x side
    (yaw = pi, facing -x toward the object).
    """
    ox, oy = float(obj_xy[0]), float(obj_xy[1])
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    dx, dy = rx - ox, ry - oy
    dist = math.hypot(dx, dy)
    if dist < _MIN_SEPARATION:
        ax, ay = ox + float(approach_distance), oy
        yaw = math.pi  # face -x, i.e. back toward the object
        return (ax, ay, yaw)
    ux, uy = dx / dist, dy / dist
    ax = ox + ux * float(approach_distance)
    ay = oy + uy * float(approach_distance)
    yaw = math.atan2(oy - ay, ox - ax)  # from approach point toward the object
    return (ax, ay, yaw)


def nearest(
    points: Sequence[Tuple[float, float]],
    robot_xy: Tuple[float, float],
) -> Tuple[int, float]:
    """Index of the point nearest `robot_xy` and its distance (same frame).

    Raises ValueError on an empty sequence so the caller handles "not found"
    explicitly before calling.
    """
    if not points:
        raise ValueError("nearest() requires at least one point")
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    dists: List[float] = [math.hypot(float(px) - rx, float(py) - ry) for px, py in points]
    best_idx = min(range(len(dists)), key=lambda i: dists[i])
    return best_idx, dists[best_idx]


def farthest(
    points: Sequence[Tuple[float, float]],
    robot_xy: Tuple[float, float],
) -> Tuple[int, float]:
    """Index of the point farthest from `robot_xy` and its distance (same frame).

    Mirror of `nearest`, used when the command's selector asks for the
    "farthest" instance. Raises ValueError on an empty sequence.
    """
    if not points:
        raise ValueError("farthest() requires at least one point")
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    dists: List[float] = [math.hypot(float(px) - rx, float(py) - ry) for px, py in points]
    best_idx = max(range(len(dists)), key=lambda i: dists[i])
    return best_idx, dists[best_idx]
