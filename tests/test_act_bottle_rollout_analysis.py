from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def _tool():
    spec = importlib.util.spec_from_file_location(
        "analyze_act_bottle_rollout",
        ROOT / "tools" / "analyze_act_bottle_rollout.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_chunk_diagnostic_detects_discarded_future_hand_closure() -> None:
    tool = _tool()
    chunks = np.zeros((20, 16, 12), dtype=np.float32)
    chunks[:, 4:, 6:11] = 0.3
    result = tool._chunk_diagnostic(chunks)
    assert result["later_grasp_discarded_answer"] == "YES"
    assert result["queries_open_at_0_1_but_grasp_like_at_2_15"] == 20


def test_chunk_diagnostic_accepts_strong_act_sixty_step_chunks() -> None:
    tool = _tool()
    chunks = np.zeros((3, 60, 12), dtype=np.float32)
    chunks[:, 31:, 6:11] = 0.3
    result = tool._chunk_diagnostic(chunks)
    assert result["diagnostic_horizons"] == [0, 1, 2, 4, 8, 15, 31, 59]
    assert result["later_grasp_discarded_answer"] == "YES"


def test_consumption_ablation_uses_configured_chunk_indices() -> None:
    tool = _tool()
    chunks = np.zeros((8, 16, 12), dtype=np.float32)
    for horizon in range(16):
        chunks[:, horizon, :] = horizon
    result = tool._simulate_consumption(chunks, 4)
    assert result["maximum_chunk_index_used"] == 3
    assert result["queries_adopted"] == 4
    assert result["queries_superseded_before_adoption"] == 4
    assert result["within_250ms_policy_age_gate"]


def test_rh56_domain_summary_reports_frequency_and_correction() -> None:
    tool = _tool()
    actions = np.zeros((2, 16, 12), dtype=np.float32)
    actions[..., 6:] = 0.5
    actions[0, 0, 6] = -0.01
    actions[1, 0, 7] = 1.02
    valid = np.ones((2, 16), dtype=bool)
    result = tool._rh56_domain_summary(actions, valid=valid)
    assert result["sample_count_per_channel"] == 32
    assert result["per_channel"]["index"]["fraction_below_legal"] == 1 / 32
    assert result["per_channel"]["index"]["below_correction"]["max"] == pytest.approx(0.01)
    assert result["per_channel"]["middle"]["fraction_above_legal"] == 1 / 32
    assert result["per_channel"]["middle"]["above_correction"]["max"] == pytest.approx(0.02)


def test_jaka_consumer_version_uses_query_command_timestamp_identity() -> None:
    tool = _tool()
    queries = [{"jaka_observation_ns": 10}, {"jaka_observation_ns": 20}]
    fixed = [
        {"policy_query_sequence": 1, "jaka_status": {"observation_ns": 10}},
        {"policy_query_sequence": 2, "jaka_status": {"observation_ns": 20}},
    ]
    stale_consumer = [
        {"policy_query_sequence": 1, "jaka_status": {"observation_ns": 11}},
        {"policy_query_sequence": 2, "jaka_status": {"observation_ns": 21}},
    ]
    assert tool._jaka_status_consumer_version(queries, fixed)["classification"] == (
        "post_fix_query_attached_status"
    )
    assert tool._jaka_status_consumer_version(queries, stale_consumer)["classification"] == (
        "pre_fix_independent_command_socket_consumer"
    )
