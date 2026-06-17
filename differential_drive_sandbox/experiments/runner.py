from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean

from differential_drive_sandbox.analysis.metrics import TrackingMetrics, compute_tracking_metrics
from differential_drive_sandbox.controllers import PurePursuitController
from differential_drive_sandbox.noise import NoiseConfig, NoiseModel
from differential_drive_sandbox.paths import named_path
from differential_drive_sandbox.robot import DifferentialDriveRobot, RobotState
from differential_drive_sandbox.simulation import SimulationConfig, SimulationEngine, TrajectorySample


@dataclass(frozen=True)
class ExperimentConfig:
    controller: str = "pure_pursuit"
    path: str = "straight"
    noise: str = "none"
    runs: int = 1
    duration: float = 50.0
    dt: float = 0.02
    integrator: str = "rk4"
    output_dir: Path = Path("outputs")
    seed: int = 7


def run_experiment(config: ExperimentConfig) -> list[TrackingMetrics]:
    config.output_dir.mkdir(parents=True, exist_ok=True)
    path = named_path(config.path)
    metrics: list[TrackingMetrics] = []
    last_samples: list[TrajectorySample] = []

    for run_index in range(config.runs):
        robot = DifferentialDriveRobot(state=RobotState(path[0][0], path[0][1], 0.0))
        controller = _build_controller(config.controller, path)
        engine = SimulationEngine(
            robot=robot,
            config=SimulationConfig(dt=config.dt, duration=config.duration, integrator=config.integrator),
            noise_model=NoiseModel(_noise_config(config.noise), seed=config.seed + run_index),
        )
        samples = engine.run(controller)
        run_metrics = compute_tracking_metrics(samples, path)
        metrics.append(run_metrics)
        last_samples = samples
        _write_trajectory_csv(config.output_dir / f"trajectory_run_{run_index + 1}.csv", samples)

    _write_reference_path_csv(config.output_dir / "path_reference.csv", path)
    _write_metrics_csv(config.output_dir / "metrics.csv", metrics)
    _write_summary(config.output_dir / "summary.md", config, metrics)
    return metrics


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run differential-drive robotics experiments.")
    parser.add_argument("--controller", default="pure_pursuit", choices=["pure_pursuit"])
    parser.add_argument("--path", default="straight", choices=["straight", "circle", "figure-eight"])
    parser.add_argument("--noise", default="none", choices=["none", "encoder", "slip", "mismatch", "jitter", "all"])
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--integrator", default="rk4", choices=["euler", "rk4"])
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args(argv)

    config = ExperimentConfig(
        controller=args.controller,
        path=args.path,
        noise=args.noise,
        runs=args.runs,
        duration=args.duration,
        dt=args.dt,
        integrator=args.integrator,
        output_dir=args.output_dir,
        seed=args.seed,
    )
    metrics = run_experiment(config)
    print(f"completed {len(metrics)} run(s); mean RMS error={mean(m.rms_error for m in metrics):.4f} m")
    print(f"outputs written to {config.output_dir}")
    return 0


def _build_controller(name: str, path: list[tuple[float, float]]) -> PurePursuitController:
    if name == "pure_pursuit":
        return PurePursuitController(path=path)
    raise ValueError(f"unsupported controller: {name}")


def _noise_config(name: str) -> NoiseConfig:
    configs = {
        "none": NoiseConfig(),
        "encoder": NoiseConfig(encoder_std_ticks=1.5),
        "slip": NoiseConfig(slip_std=0.04),
        "mismatch": NoiseConfig(wheel_radius_mismatch=0.015),
        "jitter": NoiseConfig(timestep_jitter_std=0.002),
        "all": NoiseConfig(encoder_std_ticks=1.5, slip_std=0.04, wheel_radius_mismatch=0.015, timestep_jitter_std=0.002),
    }
    return configs[name]


def _write_metrics_csv(path: Path, metrics: list[TrackingMetrics]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["run", "rms_error", "max_error", "completion_time", "samples"])
        writer.writeheader()
        for index, item in enumerate(metrics, start=1):
            writer.writerow({"run": index, **asdict(item)})


def _write_trajectory_csv(path: Path, samples: list[TrajectorySample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["time", "x", "y", "theta", "linear", "angular"])
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "time": sample.time,
                    "x": sample.state.x,
                    "y": sample.state.y,
                    "theta": sample.state.theta,
                    "linear": sample.command.linear,
                    "angular": sample.command.angular,
                }
            )


def _write_reference_path_csv(path: Path, waypoints: list[tuple[float, float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["x", "y"])
        writer.writeheader()
        for x, y in waypoints:
            writer.writerow({"x": x, "y": y})


def _write_summary(path: Path, config: ExperimentConfig, metrics: list[TrackingMetrics]) -> None:
    summary = {
        "mean_rms_error": mean(m.rms_error for m in metrics),
        "mean_max_error": mean(m.max_error for m in metrics),
        "mean_completion_time": mean(m.completion_time for m in metrics),
    }
    lines = [
        "# Experiment Summary",
        "",
        "## Configuration",
        *[f"- {key}: {value}" for key, value in asdict(config).items()],
        "",
        "## Aggregate Metrics",
        *[f"- {key}: {value:.6f}" for key, value in summary.items()],
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
