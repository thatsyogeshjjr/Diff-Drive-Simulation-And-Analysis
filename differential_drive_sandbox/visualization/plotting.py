from __future__ import annotations

from pathlib import Path
from math import cos, sin

from differential_drive_sandbox.simulation import TrajectorySample


def plot_trajectory(
    samples: list[TrajectorySample],
    desired_path: list[tuple[float, float]],
    output_path: str | Path | None = None,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plotting; install with the 'viz' extra") from exc

    actual_x = [sample.state.x for sample in samples]
    actual_y = [sample.state.y for sample in samples]
    desired_x = [point[0] for point in desired_path]
    desired_y = [point[1] for point in desired_path]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(desired_x, desired_y, "--", label="desired path", color="tab:gray")
    ax.plot(actual_x, actual_y, label="actual trajectory", color="tab:blue")
    if samples:
        last = samples[-1].state
        ax.arrow(
            last.x,
            last.y,
            0.25 * cos(last.theta),
            0.25 * sin(last.theta),
            head_width=0.08,
            color="tab:orange",
            label="heading",
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    if output_path:
        fig.savefig(output_path, dpi=160, bbox_inches="tight")
    else:
        plt.show()
    plt.close(fig)
