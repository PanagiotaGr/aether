from aether.planners.grid_search import astar


def test_astar_reaches_goal():
    path = astar(
        start=(0, 0, 0),
        goal=(4, 4, 1),
        blocked=lambda cell: cell in {(2, 2, 0), (2, 2, 1)},
        low=(0, 0, 0),
        high=(5, 5, 2),
    )
    assert path
    assert path[0] == (0, 0, 0)
    assert path[-1] == (4, 4, 1)
    assert (2, 2, 0) not in path
    assert (2, 2, 1) not in path


def test_astar_returns_empty_when_goal_is_blocked():
    path = astar(
        start=(0, 0, 0),
        goal=(1, 1, 1),
        blocked=lambda cell: cell == (1, 1, 1),
        low=(0, 0, 0),
        high=(2, 2, 2),
    )
    assert path == []
