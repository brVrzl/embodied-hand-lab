"""Low-overhead control-tick timing collection.

The live producer records only fixed-width integer tuples.  Distribution
summaries and the bounded over-budget event payload are built after the
episode, outside the control tick.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping, Sequence
from typing import Any


CONTROL_TIMING_FIELDS = (
    "quest_input_duration_ns",
    "clutch_state_duration_ns",
    "mapping_duration_ns",
    "smooth_session_duration_ns",
    "ik_duration_ns",
    "collision_singularity_duration_ns",
    "output_feasibility_duration_ns",
    "target_encode_publish_duration_ns",
    "rh56_feedback_duration_ns",
    "rh56_command_duration_ns",
    "episode_metadata_publish_duration_ns",
    "event_diagnostic_duration_ns",
    "control_accounted_duration_ns",
    "control_unaccounted_duration_ns",
    "control_total_duration_ns",
)

_FIELD_INDEX = {name: index for index, name in enumerate(CONTROL_TIMING_FIELDS)}
_ACCOUNTED_FIELDS = CONTROL_TIMING_FIELDS[:12]


def _summary(values: Sequence[int]) -> dict[str, int]:
    ordered = sorted(int(value) for value in values)
    if not ordered:
        return {"count": 0, "p50": 0, "p95": 0, "p99": 0, "max": 0}
    last = len(ordered) - 1
    return {
        "count": len(ordered),
        "p50": ordered[round(last * 0.50)],
        "p95": ordered[round(last * 0.95)],
        "p99": ordered[round(last * 0.99)],
        "max": ordered[-1],
    }


class ControlTimingRecorder:
    """Bounded integer timing ring plus bounded over-budget diagnostics."""

    def __init__(self, *, capacity: int = 512) -> None:
        if capacity <= 0:
            raise ValueError("control timing capacity must be positive")
        self.capacity = int(capacity)
        self._samples: deque[tuple[int, ...]] = deque(maxlen=self.capacity)
        self._budget_events: deque[dict[str, Any]] = deque(maxlen=self.capacity)
        self._last_budget_event: dict[str, Any] | None = None
        self._last_timestamp_ns = 0
        self._last_context: dict[str, Any] = {}

    @staticmethod
    def _account(values: list[int]) -> None:
        accounted = sum(values[_FIELD_INDEX[name]] for name in _ACCOUNTED_FIELDS)
        total = values[_FIELD_INDEX["control_total_duration_ns"]]
        values[_FIELD_INDEX["control_accounted_duration_ns"]] = accounted
        values[_FIELD_INDEX["control_unaccounted_duration_ns"]] = total - accounted

    def record(
        self,
        *,
        timestamp_ns: int,
        durations_ns: Sequence[int],
        over_budget: bool,
        context: Mapping[str, Any] | None = None,
    ) -> None:
        if len(durations_ns) != len(CONTROL_TIMING_FIELDS):
            raise ValueError("control timing sample has the wrong field count")
        values = [max(0, int(value)) for value in durations_ns]
        self._account(values)
        sample = tuple(values)
        self._samples.append(sample)
        self._last_budget_event = None
        self._last_timestamp_ns = int(timestamp_ns)
        self._last_context = {} if context is None else dict(context)
        if over_budget:
            event = {
                "timestamp_ns": self._last_timestamp_ns,
                "durations_ns": {
                    name: sample[index]
                    for index, name in enumerate(CONTROL_TIMING_FIELDS)
                },
                **self._last_context,
            }
            self._budget_events.append(event)
            self._last_budget_event = event

    def mark_last_over_budget(self) -> None:
        """Create an event after an outer producer extends the tick total."""

        if not self._samples or self._last_budget_event is not None:
            return
        sample = self._samples[-1]
        event = {
            "timestamp_ns": self._last_timestamp_ns,
            "durations_ns": {
                name: sample[index]
                for index, name in enumerate(CONTROL_TIMING_FIELDS)
            },
            **self._last_context,
        }
        self._budget_events.append(event)
        self._last_budget_event = event

    def _replace_last(self, values: list[int]) -> None:
        self._account(values)
        self._samples[-1] = tuple(values)
        if self._last_budget_event is not None:
            event = self._last_budget_event
            event["durations_ns"] = {
                name: values[index]
                for index, name in enumerate(CONTROL_TIMING_FIELDS)
            }

    def add_to_last(self, field: str, duration_ns: int) -> None:
        index = _FIELD_INDEX[field]
        values = list(self._samples[-1])
        values[index] += max(0, int(duration_ns))
        self._replace_last(values)

    def set_last(self, field: str, duration_ns: int) -> None:
        index = _FIELD_INDEX[field]
        values = list(self._samples[-1])
        values[index] = max(0, int(duration_ns))
        self._replace_last(values)

    def update_last_context(self, values: Mapping[str, Any]) -> None:
        if self._last_budget_event is not None:
            self._last_budget_event.update(values)

    def report(self) -> dict[str, Any]:
        columns = {
            name: [sample[index] for sample in self._samples]
            for index, name in enumerate(CONTROL_TIMING_FIELDS)
        }
        return {
            "capacity": self.capacity,
            "sample_count": len(self._samples),
            "durations_ns": {
                name: _summary(values) for name, values in columns.items()
            },
            "budget_event_capacity": self.capacity,
            "budget_event_count": len(self._budget_events),
            "budget_exhausted_events": list(self._budget_events),
        }
