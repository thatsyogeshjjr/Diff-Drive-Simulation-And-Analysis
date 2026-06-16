from __future__ import annotations

from dataclasses import dataclass
from math import pi

from differential_drive_sandbox.utils.angles import wrap_angle


@dataclass(frozen=True)
class RobotParams:
    wheel_radius: float = 0.05
    wheel_separation: float = 0.30
    max_velocity: float = 1.0
    encoder_resolution: int = 2048

    def __post_init__(self) -> None:
        if self.wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")
        if self.wheel_separation <= 0:
            raise ValueError("wheel_separation must be positive")
        if self.max_velocity <= 0:
            raise ValueError("max_velocity must be positive")
        if self.encoder_resolution <= 0:
            raise ValueError("encoder_resolution must be positive")

    @property
    def encoder_radians_per_tick(self) -> float:
        return 2.0 * pi / self.encoder_resolution


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class DifferentialDriveRobot:
    """Mutable robot model with pose and wheel velocity command state."""

    def __init__(self, params: RobotParams | None = None, state: RobotState | None = None) -> None:
        self.params = params or RobotParams()
        self.state = state or RobotState()
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

    def set_velocity(self, left_wheel_velocity: float, right_wheel_velocity: float) -> None:
        limit = self.params.max_velocity / self.params.wheel_radius
        self.left_wheel_velocity = max(-limit, min(limit, left_wheel_velocity))
        self.right_wheel_velocity = max(-limit, min(limit, right_wheel_velocity))

    def update_state(self, state: RobotState) -> None:
        self.state = RobotState(state.x, state.y, wrap_angle(state.theta))

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.state = RobotState(x, y, wrap_angle(theta))
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

    def get_pose(self) -> tuple[float, float, float]:
        return self.state.x, self.state.y, self.state.theta
