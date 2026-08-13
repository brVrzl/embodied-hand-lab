from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


TOOL_PATH = Path(__file__).parents[1] / "tools/analyze_act_nominal_checkpoint.py"
SPEC = importlib.util.spec_from_file_location("nominal_checkpoint", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_transition_recall_distinguishes_future_closure_from_copy_state() -> None:
    count = 14
    truth = np.zeros((count, 16, 12), dtype=np.float32)
    for row in range(count):
        onset_horizon = 15 - row
        truth[row, onset_horizon:, 6:11] = 0.4
    prediction = truth.copy()
    arrays = {
        "predictions": prediction,
        "ground_truth": truth,
        "state": np.zeros((count, 12), dtype=np.float32),
        "valid": np.ones((count, 16), dtype=bool),
        "source_frame": np.arange(count, dtype=np.int64),
        "logical_segment": np.asarray(["episode"] * count),
    }
    curation = {
        "views": {"nominal16_task_trimmed": {"per_episode": {"episode": {
            "grasp_onset_source_frame": 15,
            "open_baseline_rh56": [0.0] * 5,
            "closure_threshold": 0.1,
        }}}}
    }
    result = tool.checkpoint_summary(arrays, curation)
    assert result["transition"]["queries"] == 14
    assert result["transition"]["recall"] == 1.0
    assert result["current_state_persistence"]["copy_state_transition_recall"] == 0.0
