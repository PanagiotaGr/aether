"""Path validation utilities for bounded 3D studies."""

from __future__ import annotations

from math import sqrt
from typing import Iterable

Point = tuple[float, float, float]


def inside_workspace(point: Point, low: Point, high: Point) -> bool:
    return all(low[i] <= point[i] <= high[i] for i in range(3))


def segment_length(a: Point, b: Point) -> float:
    return sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def validate_path(
    points: Iterable[Point],
    low: Point,
    high: Point,
    max_segment_length: float,
) -> tuple[bool, list[str]]:
    pts = list(points)
    errors: list[str] = []
    for idx, point in enumerate(pts):
        if not inside_workspace(point, low, high):
            errors.append(f"point_{idx}_outside_workspace")
    for idx, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
        if segment_length(a, b) > max_segment_length:
            errors.append(f"segment_{idx}_too_long")
    return (not errors, errors)
