"""Run the minimal 3D grid example."""

from __future__ import annotations

from aether.evaluation.path_metrics import total_length
from aether.planners.grid_search import astar


def main() -> None:
    start = (1, 1, 1)
    goal = (10, 10, 2)
    occupied = {(5, 5, 1), (5, 5, 2), (5, 6, 1), (6, 5, 1)}

    path = astar(start, goal, lambda cell: cell in occupied, (0, 0, 0), (12, 12, 4))
    if not path:
        raise SystemExit("No route found")

    points = [(float(x), float(y), float(z)) for x, y, z in path]
    print(f"steps={len(path)}")
    print(f"length={total_length(points):.3f}")
    print(f"start={path[0]} goal={path[-1]}")


if __name__ == "__main__":
    main()
