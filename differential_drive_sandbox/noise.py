from __future__ import annotations

from dataclasses import dataclass
from random import Random

from differential_drive_sandbox.kinematics import WheelVelocity
from differential_drive_sandbox.robot.model import RobotParams


@dataclass(frozen=True)
class NoiseConfig:
    encoder_std_ticks: float = 0.0
    slip_std: float = 0.0
    wheel_radius_mismatch: float = 0.0
    timestep_jitter_std: float = 0.0


class NoiseModel:
    def __init__(self, config: NoiseConfig | None = None, seed: int | None = None) -> None:
        self.config = config or NoiseConfig()
        self.random = Random(seed)

    def apply_wheel_noise(self, wheels: WheelVelocity, params: RobotParams) -> WheelVelocity:
        left = wheels.left
        right = wheels.right

        if self.config.encoder_std_ticks:
            radians_std = self.config.encoder_std_ticks * params.encoder_radians_per_tick
            left += self.random.gauss(0.0, radians_std)
            right += self.random.gauss(0.0, radians_std)

        if self.config.slip_std:
            left *= max(0.0, 1.0 - self.random.gauss(0.0, self.config.slip_std))
            right *= max(0.0, 1.0 - self.random.gauss(0.0, self.config.slip_std))

        if self.config.wheel_radius_mismatch:
            left *= 1.0 - self.config.wheel_radius_mismatch
            right *= 1.0 + self.config.wheel_radius_mismatch

        return WheelVelocity(left=left, right=right)

    def apply_timestep_jitter(self, dt: float) -> float:
        if not self.config.timestep_jitter_std:
            return dt
        return max(1e-6, dt + self.random.gauss(0.0, self.config.timestep_jitter_std))
