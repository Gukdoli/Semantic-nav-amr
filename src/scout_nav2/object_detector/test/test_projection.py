"""Unit tests for the pinhole projection math (no ROS, no torch)."""

import math

import pytest

from object_detector.projection import deproject, median_depth

# Matches the workspace's corrected D435i intrinsics (see CLAUDE.md):
# fx=fy=462.3, cx=320, cy=240.
K = [462.3, 0.0, 320.0, 0.0, 462.3, 240.0, 0.0, 0.0, 1.0]


def test_median_depth_basic():
    assert median_depth([1.0, 2.0, 3.0]) == 2.0
    assert median_depth([1.0, 2.0, 3.0, 4.0]) == pytest.approx(2.5)


def test_median_depth_ignores_nan_and_zero():
    vals = [float("nan"), 0.0, 2.0, 2.0, float("nan")]
    assert median_depth(vals) == 2.0


def test_median_depth_all_invalid_returns_none():
    assert median_depth([0.0, float("nan")]) is None
    assert median_depth([]) is None


def test_deproject_principal_point_maps_to_axis():
    # A pixel at the principal point projects straight along +Z.
    x, y, z = deproject(K, 320.0, 240.0, 5.0)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(5.0)


def test_deproject_offset_pixel():
    z = 2.0
    u, v = 420.0, 300.0
    x, y, zz = deproject(K, u, v, z)
    assert x == pytest.approx((u - 320.0) / 462.3 * z)
    assert y == pytest.approx((v - 240.0) / 462.3 * z)
    assert zz == pytest.approx(z)
    # Round-trip: reproject the 3D point back to pixels.
    u_re = x / z * 462.3 + 320.0
    v_re = y / z * 462.3 + 240.0
    assert u_re == pytest.approx(u)
    assert v_re == pytest.approx(v)


def test_deproject_zero_focal_raises():
    bad = [0.0, 0.0, 320.0, 0.0, 0.0, 240.0, 0.0, 0.0, 1.0]
    with pytest.raises(ValueError):
        deproject(bad, 1.0, 1.0, 1.0)
