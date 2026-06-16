from math import isclose

from differential_drive_sandbox.kinematics import BodyVelocity, WheelVelocity, forward_kinematics, inverse_kinematics
from differential_drive_sandbox.robot import RobotParams


def test_forward_kinematics_straight_line_motion() -> None:
    params = RobotParams(wheel_radius=0.1, wheel_separation=0.5)
    body = forward_kinematics(WheelVelocity(left=5.0, right=5.0), params)

    assert isclose(body.linear, 0.5)
    assert isclose(body.angular, 0.0)


def test_forward_kinematics_in_place_rotation() -> None:
    params = RobotParams(wheel_radius=0.1, wheel_separation=0.5)
    body = forward_kinematics(WheelVelocity(left=-5.0, right=5.0), params)

    assert isclose(body.linear, 0.0)
    assert isclose(body.angular, 2.0)


def test_inverse_kinematics_round_trip_for_circular_motion() -> None:
    params = RobotParams(wheel_radius=0.05, wheel_separation=0.3)
    desired = BodyVelocity(linear=0.6, angular=0.4)
    wheels = inverse_kinematics(desired, params)
    actual = forward_kinematics(wheels, params)

    assert isclose(actual.linear, desired.linear)
    assert isclose(actual.angular, desired.angular)
