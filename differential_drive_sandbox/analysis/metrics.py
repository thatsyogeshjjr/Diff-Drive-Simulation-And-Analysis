from __future__ import annotations

from dataclasses import dataclass
from math import hypot, sqrt

from differential_drive_sandbox.robot.model import RobotState
from differential_drive_sandbox.simulation.engine import TrajectorySample
from differential_drive_sandbox.utils.angles import angle_difference

Point = tuple[float, float]


@dataclass(frozen=True)
class PoseError:
    position: float
    orientation: float


@dataclass(frozen=True)
class TrackingMetrics:
    rms_error: float
    max_error: float
    completion_time: float
    samples: int


def pose_error(actual: RobotState, expected: RobotState) -> PoseError:
    return PoseError(
        position=hypot(actual.x - expected.x, actual.y - expected.y),
        orientation=abs(angle_difference(actual.theta, expected.theta)),
    )


def compute_tracking_metrics(samples: list[TrajectorySample], desired_path: list[Point]) -> TrackingMetrics:
    if not samples:
        return TrackingMetrics(rms_error=0.0, max_error=0.0, completion_time=0.0, samples=0)
    errors = [_distance_to_path((sample.state.x, sample.state.y), desired_path) for sample in samples]
    rms = sqrt(sum(error * error for error in errors) / len(errors))
    return TrackingMetrics(
        rms_error=rms,
        max_error=max(errors),
        completion_time=samples[-1].time,
        samples=len(samples),
    )


def _distance_to_path(point: Point, path: list[Point]) -> float:
    if len(path) == 1:
        return hypot(point[0] - path[0][0], point[1] - path[0][1])
    return min(_distance_to_segment(point, start, end) for start, end in zip(path, path[1:]))


def _distance_to_segment(point: Point, start: Point, end: Point) -> float:
    px, py = point
    sx, sy = start
    ex, ey = end
    vx = ex - sx
    vy = ey - sy
    length_sq = vx * vx + vy * vy
    if length_sq == 0.0:
        return hypot(px - sx, py - sy)
    t = max(0.0, min(1.0, ((px - sx) * vx + (py - sy) * vy) / length_sq))
    projection = (sx + t * vx, sy + t * vy)
    return hypot(px - projection[0], py - projection[1])
