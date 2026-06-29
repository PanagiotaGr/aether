# Algorithms

This document records the first algorithmic baselines used by AETHER.

## Sparse voxel map

The initial map is a sparse dictionary keyed by integer 3D cell coordinates. Each cell stores a log-odds value. Log-odds updates make repeated observations additive and keep the implementation simple enough for unit tests.

## 3D grid search

The first planner uses a bounded 3D grid with 26-neighbor connectivity. It is intentionally deterministic and easy to inspect. This makes it suitable as a baseline before adding sampling-based or optimization-based methods.

## Path scoring

The first scoring helpers compute proximity and tail-average scores. These are not a complete probabilistic planner; they are lightweight numerical components that can later be connected to richer uncertainty models.

## Evaluation philosophy

Every new algorithm should be compared against a simpler baseline. A result is useful only if the scenario, seed, configuration, and metrics are stored together.
