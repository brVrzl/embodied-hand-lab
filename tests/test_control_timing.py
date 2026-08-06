from __future__ import annotations

from quest_jaka_sim.control_timing import CONTROL_TIMING_FIELDS, ControlTimingRecorder


def _sample(total: int, value: int = 1) -> list[int]:
    values = [value] * len(CONTROL_TIMING_FIELDS)
    values[-1] = total
    return values


def test_control_timing_ring_is_bounded_and_unaccounted_is_exact() -> None:
    recorder = ControlTimingRecorder(capacity=2)
    recorder.record(
        timestamp_ns=1,
        durations_ns=_sample(100),
        over_budget=True,
        continuation_retries=2,
        ik_calls=3,
        context={"hand_state": "engaged"},
    )
    recorder.record(
        timestamp_ns=2,
        durations_ns=_sample(200),
        over_budget=False,
        continuation_retries=1,
        ik_calls=2,
    )
    recorder.add_to_last("rh56_feedback_duration_ns", 5)
    recorder.record(
        timestamp_ns=3,
        durations_ns=_sample(300),
        over_budget=True,
        context={"hand_state": "hold"},
    )

    report = recorder.report()
    assert report["sample_count"] == 2
    assert report["budget_event_count"] == 2
    assert report["budget_exhausted_events"][0]["timestamp_ns"] == 1
    assert report["budget_exhausted_events"][1]["timestamp_ns"] == 3
    assert report["durations_ns"]["control_unaccounted_duration_ns"]["max"] == 287
    assert report["budget_exhausted_events"][0]["hand_state"] == "engaged"
    assert report["budget_exhausted_events"][1]["hand_state"] == "hold"
    assert report["continuation_retries"] == {
        "count": 2,
        "p50": 0,
        "p95": 1,
        "p99": 1,
        "max": 1,
    }
    assert report["ik_calls"]["max"] == 2


def test_outer_total_can_mark_the_last_sample_over_budget() -> None:
    recorder = ControlTimingRecorder(capacity=4)
    values = [0] * len(CONTROL_TIMING_FIELDS)
    recorder.record(timestamp_ns=7, durations_ns=values, over_budget=False)
    recorder.set_last("control_total_duration_ns", 21)
    recorder.mark_last_over_budget()

    report = recorder.report()
    assert report["budget_event_count"] == 1
    assert report["budget_exhausted_events"][0]["timestamp_ns"] == 7
    assert report["budget_exhausted_events"][0]["durations_ns"][
        "control_total_duration_ns"
    ] == 21
