from __future__ import annotations

from dataclasses import dataclass

from differential_drive_sandbox.robot.model import RobotParams


@dataclass(frozen=True)
class BodyVelocity:
    linear: float
    angular: float


@dataclass(frozen=True)
class WheelVelocity:
    left: float
    right: float


def forward_kinematics(wheels: WheelVelocity, params: RobotParams) -> BodyVelocity:
    left_linear = params.wheel_radius * wheels.left
    right_linear = params.wheel_radius * wheels.right
    linear = 0.5 * (right_linear + left_linear)
    angular = (right_linear - left_linear) / params.wheel_separation
    return BodyVelocity(linear=linear, angular=angular)


def inverse_kinematics(body: BodyVelocity, params: RobotParams) -> WheelVelocity:
    half_track = params.wheel_separation / 2.0
    left = (body.linear - body.angular * half_track) / params.wheel_radius
    right = (body.linear + body.angular * half_track) / params.wheel_radius
    return WheelVelocity(left=left, right=right)
