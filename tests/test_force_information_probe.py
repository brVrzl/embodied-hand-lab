from __future__ import annotations

from dataclasses import replace

import numpy as np

from research.exploration.force_information import (
    ForceEpisode,
    RH56_CHANNEL_ORDER,
    _classification_metrics,
    causal_history_indices,
    session_leave_one_out,
)


def _episode(logical: int, source: int, session: str) -> ForceEpisode:
    count = 6
    return ForceEpisode(
        logical_segment_id=f"ep{source:03d}_seg0",
        logical_episode_index=logical,
        source_episode_index=source,
        collection_session_id=session,
        canonical_timestamp_ns=np.arange(count, dtype=np.int64) * 100 + 50,
        force_timestamp_ns=np.arange(count, dtype=np.int64) * 100,
        force=np.zeros((count, 6)),
        rh56_position=np.zeros((count, 6)),
        rh56_target=np.zeros((count, 6)),
        action_status=np.asarray(["accepted"] * count),
    )


def test_force_channel_order_is_the_canonical_rh56_order() -> None:
    assert RH56_CHANNEL_ORDER == (
        "index",
        "middle",
        "ring",
        "pinky",
        "thumb_close",
        "thumb_lateral",
    )


def test_causal_history_uses_no_future_source_and_marks_incomplete_prefix() -> None:
    source = np.asarray([100, 200, 300, 400], dtype=np.int64)
    query = np.asarray([150, 299, 450], dtype=np.int64)

    indices = causal_history_indices(source, query, history_length=2)

    np.testing.assert_array_equal(indices, [[-1, -1], [0, 1], [2, 3]])
    complete = indices[:, 0] >= 0
    assert np.all(source[indices[complete]] <= query[complete, None])


def test_session_split_keeps_related_logical_segments_together() -> None:
    episodes = [
        _episode(0, 99, "session-a"),
        replace(_episode(1, 99, "session-a"), logical_segment_id="ep099_seg1"),
        _episode(2, 114, "session-b"),
        _episode(3, 116, "session-c"),
    ]

    folds = session_leave_one_out(episodes)

    assert len(folds) == 3
    session_a = next(fold for fold in folds if fold[0] == "session-a")
    assert {episode.logical_episode_index for episode in session_a[2]} == {0, 1}
    assert not ({episode.collection_session_id for episode in session_a[1]} & {"session-a"})


def test_causal_history_is_episode_local_even_when_timestamps_overlap() -> None:
    first = _episode(0, 99, "session-a")
    second = replace(
        _episode(1, 99, "session-a"),
        logical_segment_id="ep099_seg1",
        # A host clock can continue across logical segments.  History is still
        # built independently for each segment rather than concatenated.
        canonical_timestamp_ns=np.arange(6, 12, dtype=np.int64) * 100 + 50,
        force_timestamp_ns=np.arange(6, 12, dtype=np.int64) * 100,
    )

    first_indices = causal_history_indices(
        first.force_timestamp_ns, first.canonical_timestamp_ns, history_length=3
    )
    second_indices = causal_history_indices(
        second.force_timestamp_ns, second.canonical_timestamp_ns, history_length=3
    )

    np.testing.assert_array_equal(first_indices[:2], [[-1, -1, -1]] * 2)
    np.testing.assert_array_equal(second_indices[:2], [[-1, -1, -1]] * 2)
    np.testing.assert_array_equal(second_indices[2], [0, 1, 2])


def test_numeric_event_metrics_report_precision_recall_and_confusion() -> None:
    metrics = _classification_metrics(
        np.asarray([False, False, True, True]),
        np.asarray([0.1, 0.8, 0.7, 0.2]),
    )

    assert metrics["balanced_accuracy"] == 0.5
    assert metrics["precision"] == 0.5
    assert metrics["recall"] == 0.5
    assert metrics["false_positive_rate"] == 0.5
    assert metrics["confusion_matrix"] == {
        "true_negative": 1,
        "false_positive": 1,
        "false_negative": 1,
        "true_positive": 1,
    }
