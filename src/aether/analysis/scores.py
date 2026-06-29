"""Numerical scoring helpers for 3D simulation studies."""

from __future__ import annotations

from math import exp
from typing import Iterable

import numpy as np


def proximity_score(clearance: float, scale: float = 1.0) -> float:
    if clearance < 0:
        return 1.0
    return float(exp(-clearance / max(scale, 1e-6)))


def covariance_radius(covariance: np.ndarray, multiplier: float = 1.5) -> float:
    eigvals = np.linalg.eigvalsh(covariance[:3, :3])
    return float(multiplier * np.sqrt(max(eigvals.max(), 0.0)))


def tail_mean(samples: Iterable[float], fraction: float = 0.2) -> float:
    values = np.sort(np.asarray(list(samples), dtype=float))
    if values.size == 0:
        return 0.0
    count = max(1, int(np.ceil(values.size * fraction)))
    return float(values[-count:].mean())
