"""Bounded 3D grid search for simulation experiments."""

from __future__ import annotations

from heapq import heappop, heappush
from math import sqrt
from typing import Callable

GridKey = tuple[int, int, int]


def distance(a: GridKey, b: GridKey) -> float:
    return sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def neighbors_26(k: GridKey):
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx or dy or dz:
                    yield (k[0] + dx, k[1] + dy, k[2] + dz)


def build_path(parent: dict[GridKey, GridKey], node: GridKey) -> list[GridKey]:
    out = [node]
    while node in parent:
        node = parent[node]
        out.append(node)
    return list(reversed(out))


def astar(start: GridKey, goal: GridKey, blocked: Callable[[GridKey], bool], low: GridKey, high: GridKey) -> list[GridKey]:
    """Return a 3D grid path from start to goal, or an empty list."""

    def inside(k: GridKey) -> bool:
        return all(low[i] <= k[i] <= high[i] for i in range(3))

    if blocked(start) or blocked(goal):
        return []

    heap: list[tuple[float, GridKey]] = [(0.0, start)]
    parent: dict[GridKey, GridKey] = {}
    cost: dict[GridKey, float] = {start: 0.0}

    while heap:
        _, cur = heappop(heap)
        if cur == goal:
            return build_path(parent, cur)
        for nxt in neighbors_26(cur):
            if not inside(nxt) or blocked(nxt):
                continue
            new_cost = cost[cur] + distance(cur, nxt)
            if new_cost < cost.get(nxt, float("inf")):
                parent[nxt] = cur
                cost[nxt] = new_cost
                heappush(heap, (new_cost + distance(nxt, goal), nxt))
    return []
