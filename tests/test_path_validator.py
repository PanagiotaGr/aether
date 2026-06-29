from aether.validation.path_validator import validate_path


def test_validate_path_accepts_short_segments_inside_bounds():
    ok, errors = validate_path(
        [(0.0, 0.0, 0.0), (0.5, 0.5, 0.0)],
        low=(0.0, 0.0, 0.0),
        high=(1.0, 1.0, 1.0),
        max_segment_length=1.0,
    )
    assert ok
    assert errors == []


def test_validate_path_rejects_out_of_bounds_point():
    ok, errors = validate_path(
        [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
        low=(0.0, 0.0, 0.0),
        high=(1.0, 1.0, 1.0),
        max_segment_length=3.0,
    )
    assert not ok
    assert "point_1_outside_workspace" in errors
