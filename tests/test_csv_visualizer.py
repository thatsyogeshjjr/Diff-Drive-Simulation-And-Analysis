from pathlib import Path

from differential_drive_sandbox.experiments.runner import ExperimentConfig, run_experiment
from differential_drive_sandbox.visualization.csv_visualizer import build_dashboard, write_dashboard


def test_csv_visualizer_discovers_experiment_outputs(tmp_path: Path) -> None:
    run_experiment(ExperimentConfig(runs=2, duration=1.0, dt=0.1, output_dir=tmp_path))

    dashboard = build_dashboard(tmp_path)

    assert len(dashboard.metrics) == 2
    assert set(dashboard.trajectories) == {"trajectory_run_1", "trajectory_run_2"}


def test_csv_visualizer_writes_self_contained_html(tmp_path: Path) -> None:
    run_experiment(ExperimentConfig(runs=1, duration=1.0, dt=0.1, output_dir=tmp_path))

    output_path = write_dashboard(tmp_path)
    html = output_path.read_text(encoding="utf-8")

    assert output_path == tmp_path / "dashboard.html"
    assert "Differential Drive CSV Dashboard" in html
    assert "Trajectory Overview" in html
    assert "Run Metrics" in html
    assert "<svg" in html
