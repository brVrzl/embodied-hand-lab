from __future__ import annotations

import pytest

from teleoperation.timing import timing_statistics


def test_timing_statistics_and_deadline_streak() -> None:
    result = timing_statistics("cycle", [8, 9, 11, 12, 8], unit="ns", requested_period_ns=10)
    assert result.count == 5
    assert result.mean == pytest.approx(9.6)
    assert result.median == 9
    assert result.missed_deadlines == 2
    assert result.max_consecutive_missed_deadlines == 2
    assert result.p999 is None


def test_p999_reported_with_sufficient_sample_count() -> None:
    result = timing_statistics("cycle", range(1000), unit="ns")
    assert result.p999 is not None
