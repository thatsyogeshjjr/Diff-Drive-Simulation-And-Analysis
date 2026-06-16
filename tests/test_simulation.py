from pathlib import Path

from differential_drive_sandbox.controllers import PurePursuitController
from differential_drive_sandbox.experiments.runner import ExperimentConfig, run_experiment
from differential_drive_sandbox.kinematics import WheelVelocity
from differential_drive_sandbox.noise import NoiseConfig, NoiseModel
from differential_drive_sandbox.paths import straight_line
from differential_drive_sandbox.robot import DifferentialDriveRobot
from differential_drive_sandbox.simulation import SimulationConfig, SimulationEngine


def test_noise_injection_changes_wheel_command() -> None:
    robot = DifferentialDriveRobot()
    noise = NoiseModel(NoiseConfig(encoder_std_ticks=4.0, slip_std=0.1), seed=1)
    noisy = noise.apply_wheel_noise(WheelVelocity(left=10.0, right=10.0), robot.params)

    assert noisy.left != 10.0 or noisy.right != 10.0


def test_pure_pursuit_tracks_straight_path() -> None:
    path = straight_line(length=2.0)
    robot = DifferentialDriveRobot()
    controller = PurePursuitController(path=path, lookahead_distance=0.25, target_speed=0.4)
    engine = SimulationEngine(robot, SimulationConfig(duration=7.0, dt=0.02))

    engine.run(controller)

    assert abs(robot.state.y) < 0.05
    assert robot.state.x > 1.8


def test_experiment_pipeline_writes_outputs(tmp_path: Path) -> None:
    metrics = run_experiment(
        ExperimentConfig(
            runs=2,
            duration=3.0,
            dt=0.05,
            output_dir=tmp_path,
        )
    )

    assert len(metrics) == 2
    assert (tmp_path / "metrics.csv").exists()
    assert (tmp_path / "summary.md").exists()
    assert (tmp_path / "trajectory_run_1.csv").exists()
