import numpy as np

from aether.analysis.scores import covariance_radius, proximity_score, tail_mean


def test_proximity_score_decreases_with_clearance():
    assert proximity_score(0.1) > proximity_score(2.0)


def test_tail_mean_uses_largest_values():
    assert tail_mean([0.0, 1.0, 2.0, 10.0], fraction=0.25) == 10.0


def test_covariance_radius_is_positive():
    cov = np.eye(3) * 0.04
    assert covariance_radius(cov) > 0.0
