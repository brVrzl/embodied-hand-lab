from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from research.exploration.act_force_interventions import (
    EpisodeSignals,
    action_chunk,
    build_interventions,
    causal_time_shift,
    dropout_mask,
    masked_action_metrics,
    require_force_order,
    session_derangement,
)


def _episode(
    key: str,
    source_episode: int,
    session: str,
    *,
    canonical_ms: list[int] | None = None,
    source_ms: list[int] | None = None,
    force_values: np.ndarray | None = None,
) -> EpisodeSignals:
    canonical_ms = canonical_ms or [50, 90, 150, 190, 250, 290]
    source_ms = source_ms or [0, 0, 100, 100, 200, 200]
    force = (
        np.repeat(
            np.asarray([[1] * 6, [11] * 6, [21] * 6], dtype=np.float32),
            2,
            axis=0,
        )
        if force_values is None
        else np.asarray(force_values, dtype=np.float32)
    )
    count = len(canonical_ms)
    return EpisodeSignals(
        key=key,
        source_episode=source_episode,
        session=session,
        canonical_timestamp_ns=np.asarray(canonical_ms, dtype=np.int64) * 1_000_000,
        force_timestamp_ns=np.asarray(source_ms, dtype=np.int64) * 1_000_000,
        hand_q=np.linspace(0.0, 1.0, count * 6, dtype=np.float32).reshape(count, 6),
        force=force,
        action=np.zeros((count, 12), dtype=np.float32),
    )


def test_causal_time_shift_uses_elapsed_time_and_unique_sources() -> None:
    episode = _episode("ep1", 1, "session-a")

    shifted, source_time = causal_time_shift(
        episode.force,
        episode.force_timestamp_ns,
        episode.canonical_timestamp_ns,
        lag_s=0.1,
        fill=np.full(6, -5, dtype=np.float32),
    )

    # At 150 ms the cutoff is 50 ms, so the last actual source is 0 ms;
    # at 250 ms the cutoff is 150 ms, so the source is 100 ms. Cached 30 Hz
    # rows do not act like independent lag frames.
    np.testing.assert_array_equal(shifted[2], np.full(6, 1))
    np.testing.assert_array_equal(shifted[4], np.full(6, 11))
    np.testing.assert_array_equal(source_time, [-1, -1, 0, 0, 100_000_000, 100_000_000])
    available = source_time >= 0
    assert np.all(
        source_time[available]
        <= episode.canonical_timestamp_ns[available] - 100_000_000
    )


def test_episode_rejects_future_or_inconsistent_cached_force() -> None:
    episode = _episode("ep1", 1, "session-a")
    bad_force = episode.force.copy()
    bad_force[1, 0] += 1
    with pytest.raises(ValueError, match="cached FORCE_ACT rows disagree"):
        replace(episode, force=bad_force)

    future = episode.force_timestamp_ns.copy()
    future[-1] = episode.canonical_timestamp_ns[-1] + 1
    with pytest.raises(ValueError, match="later than its observation"):
        replace(episode, force_timestamp_ns=future)


def test_time_dropout_is_contiguous_in_seconds() -> None:
    timestamps = np.arange(0, 10_000_000_001, 100_000_000, dtype=np.int64)
    mask = dropout_mask(
        timestamps,
        episode_key="ep1",
        interval_s=0.5,
        period_s=2.0,
        seed=2027,
    )

    starts = np.flatnonzero(mask & ~np.r_[False, mask[:-1]])
    assert len(starts) >= 4
    np.testing.assert_allclose(np.diff(timestamps[starts]) / 1e9, 2.0, atol=0.11)
    for start in starts:
        stop = start
        while stop + 1 < len(mask) and mask[stop + 1]:
            stop += 1
        assert (timestamps[stop] - timestamps[start]) / 1e9 <= 0.5


def test_session_shuffle_never_crosses_session_or_source_episode() -> None:
    episodes = [
        _episode("ep99_seg0", 99, "session-a"),
        _episode("ep99_seg1", 99, "session-a"),
        _episode("ep95", 95, "session-a"),
        _episode("ep96", 96, "session-a"),
        _episode("ep114", 114, "session-b"),
    ]

    donors = session_derangement(episodes, seed=2027)
    by_key = {episode.key: episode for episode in episodes}

    assert "ep114" not in donors
    for target_key, donor_key in donors.items():
        assert by_key[target_key].session == by_key[donor_key].session
        assert by_key[target_key].source_episode != by_key[donor_key].source_episode


def test_intervention_training_mean_weights_unique_force_sources() -> None:
    force = np.asarray([[1] * 6, [1] * 6, [1] * 6, [9] * 6], dtype=np.float32)
    train = _episode(
        "train",
        1,
        "session-a",
        canonical_ms=[50, 90, 150, 190],
        source_ms=[0, 0, 0, 100],
        force_values=force,
    )
    validation = _episode("validation", 2, "session-a")

    interventions = build_interventions(
        [train, validation],
        training_episode_keys=["train"],
        lag_s=0.1,
        dropout_interval_s=0.2,
        dropout_period_s=1.0,
        q_ridge=1e-3,
        seed=2027,
    )

    np.testing.assert_allclose(interventions.metadata["train_force_mean"], [5.0] * 6)
    assert interventions.metadata["training_force_weighting"] == (
        "one row per unique FORCE_ACT source update"
    )
    assert "causal_time_shift" in interventions.values


def test_action_chunk_never_crosses_episode_end() -> None:
    actions = np.arange(36, dtype=np.float32).reshape(3, 12)

    chunk, pad = action_chunk(actions, start=2, horizon=4)

    np.testing.assert_array_equal(chunk, np.repeat(actions[2:3], 4, axis=0))
    np.testing.assert_array_equal(pad, [False, True, True, True])


def test_force_order_and_masked_native_action_metrics_are_explicit() -> None:
    require_force_order(
        ("index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral")
    )
    with pytest.raises(ValueError, match="force order"):
        require_force_order(
            ("pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral")
        )

    target = np.zeros((1, 2, 12), dtype=np.float64)
    prediction = target.copy()
    prediction[0, 0, :6] = 1.0
    prediction[0, 0, 6:] = 2.0
    metrics = masked_action_metrics(
        prediction, target, np.asarray([[True, False]], dtype=np.bool_)
    )

    assert metrics["valid_targets"] == 1
    assert metrics["jaka_mae"] == 1.0
    assert metrics["rh56_mae"] == 2.0
