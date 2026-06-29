"""Scenario loading helpers for reproducible grid studies."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

GridPoint = tuple[int, int, int]


@dataclass(frozen=True)
class Scenario:
    name: str
    seed: int
    low: GridPoint
    high: GridPoint
    start: GridPoint
    goal: GridPoint
    blocked_cells: set[GridPoint]


def _as_point(values: list[int]) -> GridPoint:
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, got {values!r}")
    return (int(values[0]), int(values[1]), int(values[2]))


def load_scenario(path: str | Path) -> Scenario:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    raw = data["scenario"]
    return Scenario(
        name=str(raw["name"]),
        seed=int(raw.get("seed", 0)),
        low=_as_point(raw["bounds"]["low"]),
        high=_as_point(raw["bounds"]["high"]),
        start=_as_point(raw["start"]),
        goal=_as_point(raw["goal"]),
        blocked_cells={_as_point(cell) for cell in raw.get("blocked_cells", [])},
    )
