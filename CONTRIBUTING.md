# Contributing

AETHER is organized as a simulation-first research codebase.

## Development rules

- Keep modules small and testable.
- Add tests for every numerical utility.
- Prefer deterministic examples before adding complex dependencies.
- Store experiment settings in YAML.
- Document assumptions and limitations near the code that uses them.

## Local checks

```bash
pip install -r requirements.txt
pip install -e .
pytest -q
```

## Naming

Use clear names that describe the simulation component or analysis routine. Avoid hiding assumptions behind broad labels.
