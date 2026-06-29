from aether.mapping.voxel_grid import VoxelGrid


def test_voxel_probability_increases_after_observation():
    grid = VoxelGrid(resolution=1.0)
    p0 = grid.probability((0.2, 0.2, 0.2))
    grid.update((0.2, 0.2, 0.2), occupied=True)
    p1 = grid.probability((0.2, 0.2, 0.2))
    assert p1 > p0


def test_voxel_center_matches_key():
    grid = VoxelGrid(resolution=0.5)
    center = grid.center((2, 4, 6))
    assert tuple(center) == (1.25, 2.25, 3.25)
