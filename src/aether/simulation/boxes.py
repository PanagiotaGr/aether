"""Convert box descriptions into grid cells."""

from __future__ import annotations

GridPoint = tuple[int, int, int]


def fill_box(min_corner: GridPoint, max_corner: GridPoint) -> set[GridPoint]:
    cells: set[GridPoint] = set()
    for x in range(min_corner[0], max_corner[0] + 1):
        for y in range(min_corner[1], max_corner[1] + 1):
            for z in range(min_corner[2], max_corner[2] + 1):
                cells.add((x, y, z))
    return cells


def groups_to_cells(groups: list[dict]) -> set[GridPoint]:
    cells: set[GridPoint] = set()
    for group in groups:
        if group.get("type") != "box":
            continue
        cells |= fill_box(tuple(group["min"]), tuple(group["max"]))
    return cells
