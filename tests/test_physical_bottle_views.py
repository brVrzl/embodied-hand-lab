from __future__ import annotations

import json
from pathlib import Path

from episode_dataset.training_materialization import materialize_training_dataset
from episode_dataset.training_views import ActDatasetAdapter, ActForceDatasetAdapter

from test_training_materialization import _config, _synthetic_staging


def test_act_force_keeps_invalid_force_rows_matched(tmp_path: Path) -> None:
    source = tmp_path / "raw"
    row_path, _ = _synthetic_staging(source)
    rows = [json.loads(line) for line in row_path.read_text(encoding="utf-8").splitlines()]
    timing = rows[1]["timing"]
    timing["rh56_force_act_valid"] = False
    timing["source_validity"]["rh56_force_act"] = False
    rows[1]["timing"] = timing
    row_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    result = materialize_training_dataset(_config(tmp_path, source, tmp_path / "training"))
    act = ActDatasetAdapter(result.output_root, action_horizon=4)
    force = ActForceDatasetAdapter(result.output_root, action_horizon=4)

    assert len(act) == len(force) == 3
    force_rows = [force[index] for index in range(len(force))]
    assert any(sample["observation"]["force_valid"] is False for sample in force_rows)
