from __future__ import annotations

from math import atan2, cos, sin


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return atan2(sin(angle), cos(angle))


def angle_difference(target: float, source: float) -> float:
    return wrap_angle(target - source)
