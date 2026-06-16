# Diff-Drive-Simulation-And-Analysis

A robotics experimentation framework for differential-drive mobile robots focused on simulation, controller evaluation, odometry analysis, numerical integration studies, and reproducible benchmarking.

The project provides a complete workflow for designing experiments, generating trajectory datasets.

---

![Dashboard](assets/dashboard.png)

## Installation

```powershell
python -m pip install -e .[test,viz]
```

The core simulator has no required third-party runtime dependency. The `viz` extra is only needed for trajectory plots.

## Quick Start

```powershell
python examples/quick_start.py
```

Run a benchmark from the CLI:

```powershell
python run_experiment.py --controller pure_pursuit --path straight --noise slip --runs 10 --output-dir outputs --plot
```

Outputs include per-run trajectory CSVs, aggregate `metrics.csv`, a Markdown summary, and an optional trajectory plot.

Build an interactive-free, self-contained HTML dashboard from the CSV outputs:

```powershell
python visualize_csv.py outputs
```

The dashboard summarizes aggregate metrics, compares run-level errors, overlays trajectories, and plots heading over time. If the package is installed, the same command is available as `dds-visualize outputs`.

## Structure

- `differntial_drive_sandbox/robot`: robot parameters, pose state, and mutable robot model
- `differntial_drive_sandbox/kinematics.py`: forward and inverse differential-drive kinematics
- `differntial_drive_sandbox/integrators.py`: Euler and RK4 integration
- `differntial_drive_sandbox/noise.py`: encoder noise, wheel slip, wheel mismatch, and time-step jitter
- `differntial_drive_sandbox/controllers`: controller framework, currently Pure Pursuit
- `differntial_drive_sandbox/simulation`: simulation engine
- `differntial_drive_sandbox/experiments`: CLI experiment runner and reporting
- `differntial_drive_sandbox/analysis`: tracking and pose metrics
- `/tests`: unit and integration tests

## Theory

Differential-drive forward kinematics converts left and right wheel angular velocities into body-frame linear and angular velocity:

```text
v = r * (wr + wl) / 2
omega = r * (wr - wl) / L
```

Inverse kinematics maps a desired body command back to wheel angular velocities. Euler and RK4 integration are both provided so experiments can compare accuracy and runtime under identical motion profiles.

## Results

The experiment runner reports RMS tracking error, maximum error, completion time, and sample count. These metrics are written to CSV to support reproducible benchmarking across noise models, controllers, integration methods, and path scenarios.

## Future Additions

- Stanley controller
- PID path tracking
- EKF localization
- ROS2 bridge
- Occupancy grid mapping
- Package distribution on PyPI
- Library-style scenario registry
