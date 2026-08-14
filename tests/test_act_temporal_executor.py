from __future__ import annotations

import numpy as np
import pytest

from embodiment_core.act_temporal_executor import AbsoluteTimeTemporalEnsembler


def _chunk(value: float, *, horizon: int = 4, action_dim: int = 1) -> np.ndarray:
    return np.full((horizon, action_dim), value, dtype=np.float64)


def test_absolute_horizon_maps_to_command_tick_and_preserves_future() -> None:
    ensemble = AbsoluteTimeTemporalEnsembler(action_dim=1, chunk_size=60)
    chunk = np.zeros((60, 1), dtype=np.float64)
    chunk[21, 0] = 1.0
    ensemble.add_prediction(
        query_id=7,
        query_timestamp_ns=100,
        query_command_tick=100,
        chunk=chunk,
    )
    for tick in range(100, 121):
        selection = ensemble.select(command_tick=tick)
        assert selection.action is not None
    selection = ensemble.select(command_tick=121)
    assert selection.action is not None
    assert selection.action[0] == pytest.approx(1.0)
    assert selection.contributors[0].source_horizon == 21
    assert selection.contributors[0].target_command_tick == 121


def test_positive_coefficient_matches_canonical_oldest_first_weighting() -> None:
    ensemble = AbsoluteTimeTemporalEnsembler(action_dim=1, chunk_size=3, coefficient=0.01)
    ensemble.add_prediction(query_id=1, query_timestamp_ns=1, query_command_tick=0, chunk=_chunk(2.0, horizon=3))
    ensemble.add_prediction(query_id=2, query_timestamp_ns=2, query_command_tick=1, chunk=_chunk(1.0, horizon=3))
    selection = ensemble.select(command_tick=1)
    expected = (2.0 + np.exp(-0.01) * 1.0) / (1.0 + np.exp(-0.01))
    assert selection.action is not None
    assert selection.action[0] == pytest.approx(expected)
    assert selection.contributors[0].query_id == 1
    assert selection.contributors[0].ensemble_weight > selection.contributors[1].ensemble_weight


def test_no_query_tick_uses_time_aligned_prediction_instead_of_repeating_last() -> None:
    ensemble = AbsoluteTimeTemporalEnsembler(action_dim=1, chunk_size=4)
    first = _chunk(0.0, horizon=4)
    first[2, 0] = 2.0
    ensemble.add_prediction(query_id=1, query_timestamp_ns=1, query_command_tick=0, chunk=first)
    assert ensemble.select(command_tick=0).action[0] == pytest.approx(0.0)
    assert ensemble.select(command_tick=1).action[0] == pytest.approx(0.0)
    assert ensemble.select(command_tick=2).action[0] == pytest.approx(2.0)


def test_temporal_procrastination_regression_consume_prefix_would_discard_but_ensemble_keeps() -> None:
    ensemble = AbsoluteTimeTemporalEnsembler(action_dim=1, chunk_size=30)
    # Queries arrive every two command ticks and all place the same transition
    # at h=21.  A K=2 consumer would replace each chunk before h=21.
    tick21_selection = None
    for query_tick in range(0, 30, 2):
        chunk = np.zeros((30, 1), dtype=np.float64)
        chunk[21, 0] = 1.0
        ensemble.add_prediction(
            query_id=query_tick,
            query_timestamp_ns=query_tick,
            query_command_tick=query_tick,
            chunk=chunk,
        )
        ensemble.select(command_tick=query_tick)
        if query_tick + 1 < 30:
            selection = ensemble.select(command_tick=query_tick + 1)
            if query_tick + 1 == 21:
                tick21_selection = selection
    # At tick 21 the old query's h21 is still a contributor, even though new
    # queries have arrived since it was made.
    selection = tick21_selection
    assert selection is not None
    assert selection.action is not None
    assert any(c.query_id == 0 and c.source_horizon == 21 for c in selection.contributors)


def test_episode_reset_and_age_expiry_are_bounded() -> None:
    ensemble = AbsoluteTimeTemporalEnsembler(
        action_dim=1, chunk_size=4, max_prediction_age_ticks=1, capacity=8
    )
    ensemble.add_prediction(query_id=1, query_timestamp_ns=1, query_command_tick=0, chunk=_chunk(1.0, horizon=4))
    assert ensemble.select(command_tick=0).action is not None
    assert ensemble.select(command_tick=1).action is not None
    selection = ensemble.select(command_tick=2)
    assert selection.fallback_used
    ensemble.reset()
    assert ensemble.stats()["buffered_prediction_points"] == 0
