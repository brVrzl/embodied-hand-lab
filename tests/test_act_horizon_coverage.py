from __future__ import annotations

import importlib.util
from pathlib import Path


TOOL_PATH = Path(__file__).parents[1] / "tools/analyze_act_horizon_coverage.py"
SPEC = importlib.util.spec_from_file_location("act_horizon_coverage", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_horizon_counts_future_events_and_padding_without_crossing_episode() -> None:
    result = tool._horizon_counts([10, 5], [5, 2], 4)

    assert result["future_event_queries"] == 5
    assert result["event_after_first_two_actions_queries"] == 3
    assert result["rows_with_boundary_padding"] == 6
    assert result["boundary_padded_action_slots"] == 12
