from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from html import escape
from pathlib import Path
from statistics import mean


@dataclass(frozen=True)
class MetricsRow:
    run: int
    rms_error: float
    max_error: float
    completion_time: float
    samples: int


@dataclass(frozen=True)
class TrajectoryRow:
    time: float
    x: float
    y: float
    theta: float
    linear: float
    angular: float


@dataclass(frozen=True)
class CsvDashboard:
    metrics: list[MetricsRow]
    trajectories: dict[str, list[TrajectoryRow]]

    @property
    def run_count(self) -> int:
        return max(len(self.metrics), len(self.trajectories))


def build_dashboard(input_dir: str | Path) -> CsvDashboard:
    directory = Path(input_dir)
    if not directory.exists():
        raise FileNotFoundError(f"CSV directory does not exist: {directory}")

    metrics_path = directory / "metrics.csv"
    metrics = _read_metrics(metrics_path) if metrics_path.exists() else []
    trajectories = {
        path.stem: _read_trajectory(path)
        for path in sorted(directory.glob("trajectory_run_*.csv"), key=_trajectory_sort_key)
    }
    if not metrics and not trajectories:
        raise ValueError(f"no metrics.csv or trajectory_run_*.csv files found in {directory}")
    return CsvDashboard(metrics=metrics, trajectories=trajectories)


def write_dashboard(input_dir: str | Path, output_path: str | Path | None = None) -> Path:
    directory = Path(input_dir)
    destination = Path(output_path) if output_path else directory / "dashboard.html"
    dashboard = build_dashboard(directory)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(render_dashboard(dashboard, directory), encoding="utf-8")
    return destination


def render_dashboard(dashboard: CsvDashboard, source_dir: Path) -> str:
    metrics_cards = _render_metric_cards(dashboard.metrics)
    metrics_table = _render_metrics_table(dashboard.metrics)
    trajectory_svg = _render_trajectory_svg(dashboard.trajectories)
    time_series = _render_time_series(dashboard.trajectories)
    run_cards = _render_run_cards(dashboard.trajectories)
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Differential Drive CSV Dashboard</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f7f8fb;
      --surface: #ffffff;
      --ink: #1d2433;
      --muted: #667085;
      --line: #d9dee8;
      --accent: #2563eb;
      --accent-2: #0f766e;
      --accent-3: #b45309;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--ink);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      line-height: 1.45;
    }}
    header, main {{ max-width: 1180px; margin: 0 auto; padding: 24px; }}
    header {{ padding-bottom: 8px; }}
    h1 {{ margin: 0 0 6px; font-size: 30px; line-height: 1.15; letter-spacing: 0; }}
    h2 {{ margin: 0 0 14px; font-size: 18px; letter-spacing: 0; }}
    p {{ margin: 0; color: var(--muted); }}
    section {{ margin-top: 20px; background: var(--surface); border: 1px solid var(--line); border-radius: 8px; padding: 18px; }}
    .cards {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 12px; margin-top: 18px; }}
    .card {{ border: 1px solid var(--line); border-radius: 8px; padding: 14px; background: #fbfcff; }}
    .label {{ color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: .08em; }}
    .value {{ margin-top: 6px; font-size: 24px; font-weight: 700; }}
    .grid {{ display: grid; grid-template-columns: minmax(0, 1.3fr) minmax(280px, .7fr); gap: 18px; align-items: start; }}
    table {{ width: 100%; border-collapse: collapse; font-size: 14px; }}
    th, td {{ padding: 10px 8px; border-bottom: 1px solid var(--line); text-align: right; }}
    th:first-child, td:first-child {{ text-align: left; }}
    th {{ color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: .06em; }}
    svg {{ display: block; width: 100%; height: auto; min-height: 260px; }}
    .legend {{ display: flex; flex-wrap: wrap; gap: 10px 16px; margin-top: 12px; color: var(--muted); font-size: 13px; }}
    .swatch {{ display: inline-block; width: 12px; height: 12px; border-radius: 2px; margin-right: 6px; vertical-align: -1px; }}
    .run-list {{ display: grid; gap: 10px; }}
    .run-card {{ border: 1px solid var(--line); border-radius: 8px; padding: 12px; }}
    .run-card strong {{ display: block; margin-bottom: 4px; }}
    .muted {{ color: var(--muted); }}
    @media (max-width: 860px) {{
      header, main {{ padding: 18px; }}
      .grid {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>Differential Drive CSV Dashboard</h1>
    <p>Source: {escape(str(source_dir))}</p>
    {metrics_cards}
  </header>
  <main>
    <section>
      <h2>Run Metrics</h2>
      {metrics_table}
    </section>
    <section>
      <div class="grid">
        <div>
          <h2>Trajectory Overview</h2>
          {trajectory_svg}
          {_render_legend(dashboard.trajectories)}
        </div>
        <div>
          <h2>Run Files</h2>
          {run_cards}
        </div>
      </div>
    </section>
    <section>
      <h2>Time Series</h2>
      {time_series}
    </section>
  </main>
</body>
</html>
"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build an HTML dashboard from experiment CSV files.")
    parser.add_argument("input_dir", type=Path, help="Directory containing metrics.csv and trajectory_run_*.csv")
    parser.add_argument("-o", "--output", type=Path, help="Dashboard output path; defaults to input_dir/dashboard.html")
    args = parser.parse_args(argv)
    output = write_dashboard(args.input_dir, args.output)
    print(f"dashboard written to {output}")
    return 0


def _read_metrics(path: Path) -> list[MetricsRow]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [
            MetricsRow(
                run=int(row["run"]),
                rms_error=float(row["rms_error"]),
                max_error=float(row["max_error"]),
                completion_time=float(row["completion_time"]),
                samples=int(row["samples"]),
            )
            for row in csv.DictReader(handle)
        ]


def _read_trajectory(path: Path) -> list[TrajectoryRow]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [
            TrajectoryRow(
                time=float(row["time"]),
                x=float(row["x"]),
                y=float(row["y"]),
                theta=float(row["theta"]),
                linear=float(row["linear"]),
                angular=float(row["angular"]),
            )
            for row in csv.DictReader(handle)
        ]


def _render_metric_cards(metrics: list[MetricsRow]) -> str:
    if not metrics:
        return '<div class="cards"><div class="card"><span class="label">Metrics</span><div class="value">No metrics.csv</div></div></div>'
    return f"""
    <div class="cards">
      <div class="card"><span class="label">Runs</span><div class="value">{len(metrics)}</div></div>
      <div class="card"><span class="label">Mean RMS Error</span><div class="value">{mean(m.rms_error for m in metrics):.4f} m</div></div>
      <div class="card"><span class="label">Mean Max Error</span><div class="value">{mean(m.max_error for m in metrics):.4f} m</div></div>
      <div class="card"><span class="label">Mean Completion</span><div class="value">{mean(m.completion_time for m in metrics):.2f} s</div></div>
    </div>"""


def _render_metrics_table(metrics: list[MetricsRow]) -> str:
    if not metrics:
        return '<p class="muted">No metrics.csv file was found.</p>'
    rows = "\n".join(
        f"<tr><td>Run {m.run}</td><td>{m.rms_error:.6f}</td><td>{m.max_error:.6f}</td><td>{m.completion_time:.3f}</td><td>{m.samples}</td></tr>"
        for m in metrics
    )
    return f"""<table>
      <thead><tr><th>Run</th><th>RMS Error</th><th>Max Error</th><th>Completion Time</th><th>Samples</th></tr></thead>
      <tbody>{rows}</tbody>
    </table>"""


def _render_run_cards(trajectories: dict[str, list[TrajectoryRow]]) -> str:
    if not trajectories:
        return '<p class="muted">No trajectory CSV files were found.</p>'
    cards = []
    for name, rows in trajectories.items():
        if rows:
            distance = _path_length([(row.x, row.y) for row in rows])
            final = rows[-1]
            detail = f"{len(rows)} samples, {rows[-1].time:.2f} s, {distance:.2f} m traveled<br>final pose: ({final.x:.2f}, {final.y:.2f}, {final.theta:.2f})"
        else:
            detail = "empty trajectory file"
        cards.append(f'<div class="run-card"><strong>{escape(name)}</strong><span class="muted">{detail}</span></div>')
    return f'<div class="run-list">{"".join(cards)}</div>'


def _render_trajectory_svg(trajectories: dict[str, list[TrajectoryRow]]) -> str:
    points_by_run = {
        name: [(row.x, row.y) for row in rows]
        for name, rows in trajectories.items()
        if rows
    }
    if not points_by_run:
        return '<p class="muted">No trajectory data available.</p>'
    return _polyline_svg(points_by_run, width=760, height=420, x_label="x [m]", y_label="y [m]")


def _render_time_series(trajectories: dict[str, list[TrajectoryRow]]) -> str:
    series = {
        f"{name} theta": [(row.time, row.theta) for row in rows]
        for name, rows in trajectories.items()
        if rows
    }
    if not series:
        return '<p class="muted">No time-series data available.</p>'
    return _polyline_svg(series, width=1040, height=360, x_label="time [s]", y_label="theta [rad]")


def _render_legend(trajectories: dict[str, list[TrajectoryRow]]) -> str:
    items = []
    for index, name in enumerate(trajectories):
        items.append(f'<span><span class="swatch" style="background:{_color(index)}"></span>{escape(name)}</span>')
    return f'<div class="legend">{"".join(items)}</div>'


def _polyline_svg(
    series: dict[str, list[tuple[float, float]]],
    width: int,
    height: int,
    x_label: str,
    y_label: str,
) -> str:
    margin = 46
    all_points = [point for points in series.values() for point in points]
    min_x, max_x = _bounds(point[0] for point in all_points)
    min_y, max_y = _bounds(point[1] for point in all_points)

    def sx(x: float) -> float:
        return margin + (x - min_x) / (max_x - min_x) * (width - 2 * margin)

    def sy(y: float) -> float:
        return height - margin - (y - min_y) / (max_y - min_y) * (height - 2 * margin)

    grid = _grid_lines(width, height, margin)
    paths = []
    for index, (_, points) in enumerate(series.items()):
        coords = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in points)
        paths.append(f'<polyline points="{coords}" fill="none" stroke="{_color(index)}" stroke-width="2.4" stroke-linejoin="round" stroke-linecap="round" />')
    return f"""<svg viewBox="0 0 {width} {height}" role="img" aria-label="{escape(y_label)} by {escape(x_label)}">
      <rect x="0" y="0" width="{width}" height="{height}" rx="8" fill="#fbfcff" />
      {grid}
      <text x="{width / 2:.1f}" y="{height - 10}" text-anchor="middle" fill="#667085" font-size="13">{escape(x_label)}</text>
      <text x="16" y="{height / 2:.1f}" text-anchor="middle" fill="#667085" font-size="13" transform="rotate(-90 16 {height / 2:.1f})">{escape(y_label)}</text>
      <text x="{margin}" y="{height - margin + 20}" fill="#667085" font-size="12">{min_x:.2f}</text>
      <text x="{width - margin}" y="{height - margin + 20}" text-anchor="end" fill="#667085" font-size="12">{max_x:.2f}</text>
      <text x="{margin - 8}" y="{sy(max_y):.1f}" text-anchor="end" fill="#667085" font-size="12">{max_y:.2f}</text>
      <text x="{margin - 8}" y="{sy(min_y):.1f}" text-anchor="end" fill="#667085" font-size="12">{min_y:.2f}</text>
      {"".join(paths)}
    </svg>"""


def _grid_lines(width: int, height: int, margin: int) -> str:
    lines = []
    for i in range(6):
        x = margin + i * (width - 2 * margin) / 5
        y = margin + i * (height - 2 * margin) / 5
        lines.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{margin}" y2="{height - margin}" stroke="#e7ebf2" />')
        lines.append(f'<line x1="{margin}" x2="{width - margin}" y1="{y:.1f}" y2="{y:.1f}" stroke="#e7ebf2" />')
    lines.append(f'<rect x="{margin}" y="{margin}" width="{width - 2 * margin}" height="{height - 2 * margin}" fill="none" stroke="#cfd6e3" />')
    return "".join(lines)


def _bounds(values: object) -> tuple[float, float]:
    collected = list(values)
    low = min(collected)
    high = max(collected)
    if low == high:
        padding = 1.0 if low == 0 else abs(low) * 0.1
        return low - padding, high + padding
    padding = (high - low) * 0.05
    return low - padding, high + padding


def _path_length(points: list[tuple[float, float]]) -> float:
    return sum(
        ((right[0] - left[0]) ** 2 + (right[1] - left[1]) ** 2) ** 0.5
        for left, right in zip(points, points[1:])
    )


def _trajectory_sort_key(path: Path) -> tuple[int, str]:
    try:
        return int(path.stem.rsplit("_", 1)[1]), path.name
    except (IndexError, ValueError):
        return 0, path.name


def _color(index: int) -> str:
    palette = ["#2563eb", "#0f766e", "#b45309", "#9333ea", "#dc2626", "#475569", "#0891b2", "#65a30d"]
    return palette[index % len(palette)]


if __name__ == "__main__":
    raise SystemExit(main())
