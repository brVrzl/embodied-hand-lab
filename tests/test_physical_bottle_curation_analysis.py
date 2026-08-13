from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


TOOL_PATH = Path(__file__).parents[1] / "tools/analyze_physical_bottle_curation.py"
SPEC = importlib.util.spec_from_file_location("bottle_curation", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def _rows(count: int, onset: int) -> list[dict]:
    rows = []
    for index in range(count):
        action = np.zeros(12, dtype=float)
        if index >= onset:
            action[6:11] = 0.3
        rows.append({
            "action": action.tolist(),
            "timestamp_ns": 1_000_000_000 + index * 33_333_333,
            "source_frame_index": index,
        })
    return rows


def test_transition_summary_counts_only_chunks_inside_one_trajectory() -> None:
    result = tool.transition_summary(_rows(50, 20))
    assert result["grasp_onset_local_frame"] == 20
    assert result["approach_rows"] == 20
    assert result["transition_containing_chunks"] == 14
    assert result["post_grasp_rows"] == 30


def test_transition_heuristic_allows_one_finger_channel_to_remain_open() -> None:
    rows = _rows(40, 15)
    for row in rows[15:]:
        row["action"][10] = 0.0
    result = tool.transition_summary(rows)
    assert result["grasp_onset_local_frame"] == 15


def test_known_reset_interval_report_quantifies_stationary_rows(tmp_path: Path) -> None:
    data = tmp_path / "data/chunk-000"
    data.mkdir(parents=True)
    rows = _rows(8, 4)
    import json

    (data / "episode_000099.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows),
        encoding="utf-8",
    )
    result = tool._interval_action_report(tmp_path, ((99, 1, 3),), mixed_rows=10)
    assert result["rows"] == 3
    assert result["fraction_of_old_mixed_rows"] == 0.3
    assert result["stationary_action_fraction"] == 1.0
