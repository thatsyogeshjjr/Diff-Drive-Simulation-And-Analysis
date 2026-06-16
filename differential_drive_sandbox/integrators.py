from __future__ import annotations

from math import cos, sin
from time import perf_counter
from typing import Callable

from differential_drive_sandbox.kinematics import BodyVelocity
from differential_drive_sandbox.robot.model import RobotState
from differential_drive_sandbox.utils.angles import wrap_angle

Derivative = tuple[float, float, float]


def pose_derivative(state: RobotState, velocity: BodyVelocity) -> Derivative:
    return (
        velocity.linear * cos(state.theta),
        velocity.linear * sin(state.theta),
        velocity.angular,
    )


def euler_step(state: RobotState, velocity: BodyVelocity, dt: float) -> RobotState:
    dx, dy, dtheta = pose_derivative(state, velocity)
    return RobotState(
        x=state.x + dx * dt,
        y=state.y + dy * dt,
        theta=wrap_angle(state.theta + dtheta * dt),
    )


def rk4_step(state: RobotState, velocity: BodyVelocity, dt: float) -> RobotState:
    def add(base: RobotState, derivative: Derivative, scale: float) -> RobotState:
        return RobotState(
            base.x + derivative[0] * scale,
            base.y + derivative[1] * scale,
            wrap_angle(base.theta + derivative[2] * scale),
        )

    k1 = pose_derivative(state, velocity)
    k2 = pose_derivative(add(state, k1, dt / 2.0), velocity)
    k3 = pose_derivative(add(state, k2, dt / 2.0), velocity)
    k4 = pose_derivative(add(state, k3, dt), velocity)
    return RobotState(
        x=state.x + dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
        y=state.y + dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
        theta=wrap_angle(state.theta + dt * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0),
    )


def timed_step(
    stepper: Callable[[RobotState, BodyVelocity, float], RobotState],
    state: RobotState,
    velocity: BodyVelocity,
    dt: float,
) -> tuple[RobotState, float]:
    start = perf_counter()
    next_state = stepper(state, velocity, dt)
    return next_state, perf_counter() - start
