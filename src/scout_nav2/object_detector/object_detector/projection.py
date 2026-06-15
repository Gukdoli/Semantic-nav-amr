"""Pure-function pinhole projection helpers.

Deliberately free of ROS and torch imports so the 2D->3D math can be unit
tested standalone (see test/test_projection.py). The node layer
(object_detector_node.py) feeds these with data pulled from ROS messages.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence


def median_depth(values: Sequence[float]) -> Optional[float]:
    """Median depth (metres) over a patch, ignoring invalid pixels.

    Depth is 32FC1 in metres on this workspace; failed measurements come back
    as NaN, and 0.0 is also treated as invalid. Returns None when no valid
    pixel remains so the caller can skip that detection.
    """
    valid = [
        float(v)
        for v in values
        if v is not None and not math.isnan(float(v)) and float(v) > 0.0
    ]
    if not valid:
        return None
    valid.sort()
    n = len(valid)
    mid = n // 2
    if n % 2 == 1:
        return valid[mid]
    return 0.5 * (valid[mid - 1] + valid[mid])


def deproject(k: Sequence[float], u: float, v: float, z: float):
    """Back-project pixel (u, v) at depth z (m) to a 3D point in the camera
    optical frame.

    `k` is the 3x3 row-major CameraInfo intrinsics (length 9):
        [fx, 0, cx, 0, fy, cy, 0, 0, 1].
    Returns (x, y, z) in metres using the standard pinhole model.
    """
    fx = float(k[0])
    fy = float(k[4])
    cx = float(k[2])
    cy = float(k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("Invalid intrinsics: fx/fy must be non-zero")
    x = (float(u) - cx) / fx * float(z)
    y = (float(v) - cy) / fy * float(z)
    return (x, y, float(z))
