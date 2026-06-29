from aether.simulation.boxes import fill_box, groups_to_cells


def test_fill_box_counts_cells():
    cells = fill_box((0, 0, 0), (1, 1, 1))
    assert len(cells) == 8
    assert (1, 1, 1) in cells


def test_groups_to_cells_ignores_unknown_types():
    cells = groups_to_cells([
        {"type": "box", "min": [0, 0, 0], "max": [0, 0, 0]},
        {"type": "other", "min": [1, 1, 1], "max": [2, 2, 2]},
    ])
    assert cells == {(0, 0, 0)}
