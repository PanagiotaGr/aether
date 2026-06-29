# AETHER Architecture

AETHER is organized as a simulation research stack with seven layers: perception, mapping, planning, risk analysis, safety validation, scenario management, and evaluation.

The first milestone keeps each layer small and testable. A virtual vehicle moves through a 3D grid world. The planner proposes a path, the validator checks clearance, and the evaluator records path length, success, and risk statistics.

## Layer responsibilities

- Perception: convert virtual sensor observations into map updates.
- Mapping: maintain a probabilistic voxel grid.
- Planning: compute obstacle-aware paths through the grid.
- Risk: score path segments using clearance and uncertainty.
- Safety: reject paths that violate clearance or boundary constraints.
- Scenario: define starts, goals, obstacles, and noise settings.
- Evaluation: compute metrics from repeated runs.
