from pathlib import Path

from aether.simulation.scenario import load_scenario


def test_load_scenario(tmp_path: Path):
    scenario_file = tmp_path / "scenario.yaml"
    scenario_file.write_text(
        """
scenario:
  name: tiny
  seed: 3
  bounds:
    low: [0, 0, 0]
    high: [3, 3, 2]
  start: [0, 0, 0]
  goal: [3, 3, 1]
  blocked_cells:
    - [1, 1, 0]
""",
        encoding="utf-8",
    )
    scenario = load_scenario(scenario_file)
    assert scenario.name == "tiny"
    assert scenario.seed == 3
    assert scenario.start == (0, 0, 0)
    assert (1, 1, 0) in scenario.blocked_cells
