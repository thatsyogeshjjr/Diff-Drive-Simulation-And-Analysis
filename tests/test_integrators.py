from math import isclose, pi

from differential_drive_sandbox.integrators import euler_step, rk4_step
from differential_drive_sandbox.kinematics import BodyVelocity
from differential_drive_sandbox.robot import RobotState


def test_euler_integrator_straight_line() -> None:
    state = euler_step(RobotState(), BodyVelocity(linear=1.0, angular=0.0), dt=1.0)

    assert isclose(state.x, 1.0)
    assert isclose(state.y, 0.0)
    assert isclose(state.theta, 0.0)


def test_rk4_integrator_curved_motion_is_close_to_analytic_solution() -> None:
    state = rk4_step(RobotState(), BodyVelocity(linear=1.0, angular=1.0), dt=pi / 2.0)

    assert isclose(state.x, 1.0, abs_tol=0.01)
    assert isclose(state.y, 1.0, abs_tol=0.01)
    assert isclose(state.theta, pi / 2.0)


def test_rk4_is_more_accurate_than_euler_for_large_curved_step() -> None:
    velocity = BodyVelocity(linear=1.0, angular=1.0)
    euler = euler_step(RobotState(), velocity, dt=1.0)
    rk4 = rk4_step(RobotState(), velocity, dt=1.0)

    euler_error = abs(euler.x - 0.8414709848) + abs(euler.y - 0.4596976941)
    rk4_error = abs(rk4.x - 0.8414709848) + abs(rk4.y - 0.4596976941)
    assert rk4_error < euler_error
