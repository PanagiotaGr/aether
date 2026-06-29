# Experiments

AETHER experiments are defined as small, reproducible scenario bundles. Each bundle should specify a map, start and target states, blocked regions, observation noise settings, and a random seed.

## Baseline studies

1. Open-space path search.
2. Narrow-passage grid search.
3. Unknown-cell penalty comparison.
4. Repeated random seeds for score stability.
5. Runtime scaling with grid size.

## Required outputs

Each run should save:

- scenario name
- seed
- path length
- number of states
- wall-clock runtime
- minimum clearance estimate
- aggregate proximity score
- success flag

## Reproducibility rules

- Store every scenario as YAML.
- Never overwrite raw run logs.
- Include the commit hash in every result file when possible.
- Use deterministic seeds for benchmark tables.
