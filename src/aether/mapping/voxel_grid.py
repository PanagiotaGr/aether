"""Lightweight probabilistic voxel grid for simulation experiments."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import floor
from typing import Iterable

import numpy as np


@dataclass
class VoxelGrid:
    """Sparse 3D occupancy grid using log-odds updates.

    The grid stores only observed cells. It is deliberately small and dependency-light
    so it can be tested without a simulator.
    """

    resolution: float = 0.5
    occupied_logodds: float = 0.85
    free_logodds: float = -0.4
    min_logodds: float = -4.0
    max_logodds: float = 4.0
    cells: dict[tuple[int, int, int], float] = field(default_factory=dict)

    def key(self, point: Iterable[float]) -> tuple[int, int, int]:
        x, y, z = point
        return (floor(x / self.resolution), floor(y / self.resolution), floor(z / self.resolution))

    def center(self, key: tuple[int, int, int]) -> np.ndarray:
        return (np.asarray(key, dtype=float) + 0.5) * self.resolution

    def update(self, point: Iterable[float], occupied: bool) -> None:
        k = self.key(point)
        delta = self.occupied_logodds if occupied else self.free_logodds
        self.cells[k] = float(np.clip(self.cells.get(k, 0.0) + delta, self.min_logodds, self.max_logodds))

    def probability(self, point: Iterable[float]) -> float:
        odds = self.cells.get(self.key(point), 0.0)
        return float(1.0 / (1.0 + np.exp(-odds)))

    def is_occupied(self, point: Iterable[float], threshold: float = 0.65) -> bool:
        return self.probability(point) >= threshold

    def occupied_centers(self, threshold: float = 0.65) -> list[np.ndarray]:
        return [self.center(k) for k, v in self.cells.items() if 1.0 / (1.0 + np.exp(-v)) >= threshold]
