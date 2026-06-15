"""Unit tests for the approach-pose geometry (no ROS)."""

import math

import pytest

from language_goal.approach import compute_approach_pose, farthest, nearest


def test_approach_point_is_offset_toward_robot():
    # Object at origin, robot on +x at distance 5; approach 0.7 m -> (0.7, 0).
    ax, ay, yaw = compute_approach_pose((0.0, 0.0), (5.0, 0.0), 0.7)
    assert ax == pytest.approx(0.7)
    assert ay == pytest.approx(0.0)
    # Facing back toward the object (-x) => yaw = pi.
    assert abs(abs(yaw) - math.pi) == pytest.approx(0.0)


def test_approach_distance_from_object_is_exact():
    ax, ay, _ = compute_approach_pose((1.0, 2.0), (4.0, 6.0), 0.7)
    d = math.hypot(ax - 1.0, ay - 2.0)
    assert d == pytest.approx(0.7)


def test_yaw_points_from_approach_to_object():
    obj = (0.0, 0.0)
    ax, ay, yaw = compute_approach_pose(obj, (0.0, 5.0), 0.7)
    # Robot on +y, so approach point is +y of object; yaw should face -y.
    assert ax == pytest.approx(0.0)
    assert ay == pytest.approx(0.7)
    assert yaw == pytest.approx(-math.pi / 2)
    # yaw aimed from approach point toward object.
    assert math.atan2(obj[1] - ay, obj[0] - ax) == pytest.approx(yaw)


def test_robot_on_top_of_object_uses_fallback():
    ax, ay, yaw = compute_approach_pose((2.0, 2.0), (2.0, 2.0), 0.7)
    assert ax == pytest.approx(2.7)
    assert ay == pytest.approx(2.0)
    assert yaw == pytest.approx(math.pi)


def test_nearest_picks_closest_index_and_distance():
    pts = [(10.0, 0.0), (1.0, 0.0), (5.0, 0.0)]
    idx, dist = nearest(pts, (0.0, 0.0))
    assert idx == 1
    assert dist == pytest.approx(1.0)


def test_nearest_empty_raises():
    with pytest.raises(ValueError):
        nearest([], (0.0, 0.0))


def test_farthest_picks_furthest_index_and_distance():
    pts = [(10.0, 0.0), (1.0, 0.0), (5.0, 0.0)]
    idx, dist = farthest(pts, (0.0, 0.0))
    assert idx == 0
    assert dist == pytest.approx(10.0)


def test_farthest_empty_raises():
    with pytest.raises(ValueError):
        farthest([], (0.0, 0.0))
