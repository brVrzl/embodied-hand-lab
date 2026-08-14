from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _tool():
    spec = importlib.util.spec_from_file_location(
        "act_transition_execution_audit",
        ROOT / "tools" / "act_transition_execution_audit.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_consume_k_replay_never_executes_beyond_configured_prefix() -> None:
    tool = _tool()
    predictions = np.zeros((8, 6, 12), dtype=np.float64)
    for horizon in range(6):
        predictions[:, horizon, 6] = horizon
    output, selected_h, selected_query, query_indices = tool._replay_segment(
        predictions,
        query_hz=15.0,
        mode="consume_k",
        consume_actions=2,
        ensemble_coeff=None,
    )
    assert query_indices.tolist() == [0, 2, 4, 6]
    assert selected_h.tolist() == [0, 1, 0, 1, 0, 1, 0, 1]
    assert selected_query.tolist() == [0, 0, 2, 2, 4, 4, 6, 6]
    assert output[:, 6].tolist() == [0, 1, 0, 1, 0, 1, 0, 1]


def test_temporal_ensemble_blends_overlapping_horizons() -> None:
    tool = _tool()
    predictions = np.zeros((3, 3, 12), dtype=np.float64)
    predictions[0, 1, 6] = 2.0
    predictions[1, 0, 6] = 1.0
    predictions[2, 0, 6] = 3.0
    output, _, _, _ = tool._replay_segment(
        predictions,
        query_hz=30.0,
        mode="temporal_ensemble",
        consume_actions=1,
        ensemble_coeff=0.01,
    )
    # At row 1, the current h0 and the previous query's h1 overlap.
    expected = (1.0 + np.exp(-0.01) * 2.0) / (1.0 + np.exp(-0.01))
    np.testing.assert_allclose(output[1, 6], expected, rtol=1e-12)


def test_transition_metrics_detects_a_future_grasp_horizon() -> None:
    tool = _tool()
    prediction = np.zeros((4, 4, 12), dtype=np.float64)
    truth = np.zeros_like(prediction)
    truth[0, 3, 6:11] = 0.4
    info = {
        "closure_threshold": 0.2,
        "grasp_onset_source_frame": 3,
        "release_onset_source_frame": 7,
        "open_baseline_rh56": [0.0] * 5,
    }
    metrics = tool._transition_metrics(
        prediction, truth, np.ones((4, 4), dtype=bool), np.arange(4), info, event="grasp"
    )
    assert metrics["eligible_queries"] == 3
    assert metrics["recalled_queries"] == 0
