from __future__ import annotations

from math import cos, pi, sin

Point = tuple[float, float]


def straight_line(length: float = 5.0, points: int = 50) -> list[Point]:
    return [(length * i / (points - 1), 0.0) for i in range(points)]


def circle(radius: float = 2.0, points: int = 500) -> list[Point]:
    return [
        (radius * cos(2.0 * pi * i / (points - 1)), radius * sin(2.0 * pi * i / (points - 1)))
        for i in range(points)
    ]


def figure_eight(radius: float = 2.0, points: int = 800) -> list[Point]:
    return [
        (
            radius * sin(2.0 * pi * i / (points - 1)),
            radius * sin(2.0 * 2.0 * pi * i / (points - 1)) / 2.0,
        )
        for i in range(points)
    ]


def named_path(name: str) -> list[Point]:
    if name == "straight":
        return straight_line()
    if name == "circle":
        return circle()
    if name in {"figure-eight", "figure_eight"}:
        return figure_eight()
    raise ValueError(f"unknown path scenario: {name}")
