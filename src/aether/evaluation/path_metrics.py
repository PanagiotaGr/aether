"""Small path metrics used by the baseline examples."""

from __future__ import annotations

from math import sqrt
from typing import Sequence

Point = tuple[float, float, float]


def segment_length(a: Point, b: Point) -> float:
    return sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def total_length(points: Sequence[Point]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(segment_length(a, b) for a, b in zip(points[:-1], points[1:]))
