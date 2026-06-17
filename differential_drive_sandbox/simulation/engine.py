from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

from differential_drive_sandbox.integrators import euler_step, rk4_step
from differential_drive_sandbox.kinematics import (
    BodyVelocity,
    WheelVelocity,
    forward_kinematics,
    inverse_kinematics,
)
from differential_drive_sandbox.noise import NoiseModel
from differential_drive_sandbox.robot.model import DifferentialDriveRobot, RobotState

Controller = Callable[[RobotState, float], BodyVelocity | WheelVelocity]


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.02
    duration: float = 50.0
    integrator: str = "rk4"

    def __post_init__(self) -> None:
        if self.dt <= 0:
            raise ValueError("dt must be positive")
        if self.duration <= 0:
            raise ValueError("duration must be positive")
        if self.integrator not in {"euler", "rk4"}:
            raise ValueError("integrator must be 'euler' or 'rk4'")


@dataclass(frozen=True)
class TrajectorySample:
    time: float
    state: RobotState
    command: BodyVelocity


class SimulationEngine:
    def __init__(
        self,
        robot: DifferentialDriveRobot,
        config: SimulationConfig | None = None,
        noise_model: NoiseModel | None = None,
    ) -> None:
        self.robot = robot
        self.config = config or SimulationConfig()
        self.noise_model = noise_model or NoiseModel()
        self._stepper = euler_step if self.config.integrator == "euler" else rk4_step

    def step(self, command: BodyVelocity | WheelVelocity, dt: float | None = None) -> TrajectorySample:
        actual_dt = self.noise_model.apply_timestep_jitter(dt or self.config.dt)
        body_command = self._to_body_velocity(command)
        self.robot.update_state(self._stepper(self.robot.state, body_command, actual_dt))
        return TrajectorySample(time=actual_dt, state=self.robot.state, command=body_command)

    def run(self, controller: Controller) -> list[TrajectorySample]:
        samples: list[TrajectorySample] = []
        elapsed = 0.0
        while elapsed < self.config.duration:
            command = controller(self.robot.state, elapsed)
            sample = self.step(command)
            elapsed += sample.time
            samples.append(TrajectorySample(time=elapsed, state=sample.state, command=sample.command))
        return samples

    def replay(self, commands: Iterable[BodyVelocity | WheelVelocity]) -> list[TrajectorySample]:
        samples: list[TrajectorySample] = []
        elapsed = 0.0
        for command in commands:
            sample = self.step(command)
            elapsed += sample.time
            samples.append(TrajectorySample(time=elapsed, state=sample.state, command=sample.command))
        return samples

    def _to_body_velocity(self, command: BodyVelocity | WheelVelocity) -> BodyVelocity:
        if isinstance(command, BodyVelocity):
            wheels = inverse_kinematics(command, self.robot.params)
            noisy_wheels = self.noise_model.apply_wheel_noise(wheels, self.robot.params)
            return forward_kinematics(noisy_wheels, self.robot.params)
        noisy_wheels = self.noise_model.apply_wheel_noise(command, self.robot.params)
        return forward_kinematics(noisy_wheels, self.robot.params)
