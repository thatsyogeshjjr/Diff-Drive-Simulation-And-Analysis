from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot, sin

from differential_drive_sandbox.kinematics import BodyVelocity
from differential_drive_sandbox.robot.model import RobotState
from differential_drive_sandbox.utils.angles import angle_difference

Waypoint = tuple[float, float]


@dataclass
class PurePursuitController:
    path: list[Waypoint]
    lookahead_distance: float = 0.5
    target_speed: float = 0.4
    goal_tolerance: float = 0.15

    def __post_init__(self) -> None:
        if len(self.path) < 2:
            raise ValueError("path must contain at least two waypoints")
        if self.lookahead_distance <= 0:
            raise ValueError("lookahead_distance must be positive")
        if self.target_speed <= 0:
            raise ValueError("target_speed must be positive")
        self._target_index = 0

    def __call__(self, state: RobotState, _: float = 0.0) -> BodyVelocity:
        target = self._select_target(state)
        dx = target[0] - state.x
        dy = target[1] - state.y
        heading_to_target = atan2(dy, dx)
        alpha = angle_difference(heading_to_target, state.theta)
        curvature = 2.0 * sin(alpha) / max(self.lookahead_distance, hypot(dx, dy), 1e-9)
        speed = 0.0 if self.is_finished(state) else self.target_speed
        return BodyVelocity(linear=speed, angular=speed * curvature)

    def is_finished(self, state: RobotState) -> bool:
        if self._target_index < len(self.path) - 5:
            return False
        goal = self.path[-1]
        return hypot(goal[0] - state.x, goal[1] - state.y) <= self.goal_tolerance

    def _select_target(self, state: RobotState) -> Waypoint:
        while self._target_index < len(self.path) - 1:
            point = self.path[self._target_index]
            if hypot(point[0] - state.x, point[1] - state.y) >= self.lookahead_distance:
                break
            self._target_index += 1
        return self.path[self._target_index]
