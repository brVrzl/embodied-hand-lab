from __future__ import annotations

import numpy as np
import pytest

from research.exploration.force_history import (
    RH56_CHANNEL_ORDER,
    build_causal_force_history,
    flatten_force_history,
    unique_force_sources,
)


def _cached_rows():
    canonical = np.asarray([50, 90, 150, 190, 250, 290], dtype=np.int64)
    source = np.asarray([0, 0, 100, 100, 200, 200], dtype=np.int64)
    updates = np.asarray(
        [
            [1, 2, 3, 4, 5, 6],
            [11, 12, 13, 14, 15, 16],
            [21, 22, 23, 24, 25, 26],
        ],
        dtype=np.float64,
    )
    force = np.repeat(updates, 2, axis=0)
    valid = np.ones(len(canonical), dtype=np.bool_)
    return canonical, source, valid, force


def test_history_counts_unique_source_updates_not_cached_rows() -> None:
    canonical, source, valid, force = _cached_rows()
    unique = unique_force_sources(canonical, source, valid, force)

    history = build_causal_force_history(
        unique, canonical, history_updates=3, baseline_load=np.zeros(6)
    )

    np.testing.assert_array_equal(unique.first_canonical_row, [0, 2, 4])
    np.testing.assert_array_equal(
        history.source_timestamp_ns,
        [
            [-1, -1, 0],
            [-1, -1, 0],
            [-1, 0, 100],
            [-1, 0, 100],
            [0, 100, 200],
            [0, 100, 200],
        ],
    )
    # Two 30 Hz rows carrying one cached sample yield identical histories;
    # neither row becomes an extra apparent force update.
    np.testing.assert_array_equal(history.raw_load[0], history.raw_load[1])
    np.testing.assert_array_equal(history.raw_load[2], history.raw_load[3])


def test_history_is_causal_and_exposes_age_validity_delta_and_share() -> None:
    canonical, source, valid, force = _cached_rows()
    unique = unique_force_sources(canonical, source, valid, force)
    history = build_causal_force_history(
        unique,
        np.asarray([150, 250], dtype=np.int64),
        history_updates=3,
        baseline_load=np.ones(6),
    )

    present = history.present
    query = np.broadcast_to(history.query_timestamp_ns[:, None], present.shape)
    assert np.all(history.source_timestamp_ns[present] <= query[present])
    np.testing.assert_allclose(history.source_age_s[1], [2.5e-7, 1.5e-7, 0.5e-7])
    np.testing.assert_array_equal(history.load_delta[1, 0], np.zeros(6))
    np.testing.assert_array_equal(history.load_delta[1, 1], np.full(6, 10.0))
    np.testing.assert_allclose(history.actuator_load_magnitude_share[1].sum(axis=1), 1.0)
    assert RH56_CHANNEL_ORDER[-2:] == ("thumb_close", "thumb_lateral")


def test_invalid_source_is_masked_without_hiding_its_timestamp() -> None:
    canonical, source, valid, force = _cached_rows()
    valid[2:4] = False
    unique = unique_force_sources(canonical, source, valid, force)

    history = build_causal_force_history(
        unique, np.asarray([250], dtype=np.int64), history_updates=3
    )

    np.testing.assert_array_equal(history.present[0], [True, True, True])
    np.testing.assert_array_equal(history.valid[0], [True, False, True])
    np.testing.assert_array_equal(history.source_timestamp_ns[0], [0, 100, 200])
    np.testing.assert_array_equal(history.raw_load[0, 1], np.zeros(6))
    np.testing.assert_array_equal(history.load_delta[0], np.zeros((3, 6)))


def test_cached_value_disagreement_is_rejected() -> None:
    canonical, source, valid, force = _cached_rows()
    force[1, 0] += 1

    with pytest.raises(ValueError, match="cached rows disagree"):
        unique_force_sources(canonical, source, valid, force)


def test_flattened_feature_order_is_explicit_and_finite() -> None:
    canonical, source, valid, force = _cached_rows()
    unique = unique_force_sources(canonical, source, valid, force)
    history = build_causal_force_history(unique, canonical, history_updates=2)

    features, names = flatten_force_history(
        history, ("raw_load", "source_age_s", "valid")
    )

    assert features.shape == (len(canonical), 2 * 6 + 2 + 2)
    assert np.isfinite(features).all()
    assert names[0] == "force_history_t_minus_1_raw_load_index"
    assert names[5] == "force_history_t_minus_1_raw_load_thumb_lateral"
    assert names[-1] == "force_history_t_minus_0_valid"
