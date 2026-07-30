from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Any, Callable, TextIO


@dataclass(frozen=True, slots=True)
class LoggingFailure:
    operation: str
    exception_type: str
    message: str

    def as_dict(self) -> dict[str, str]:
        return {
            "operation": self.operation,
            "exception_type": self.exception_type,
            "message": self.message,
        }


class BoundedJsonlRecorder:
    """Bounded, periodic JSONL buffering with immediate fault persistence.

    File failures are retained as logging failures and never reclassified as
    serial failures.  The recorder is intentionally synchronous and is called
    only by the single RH56 worker/entry thread; it creates no second serial or
    logging thread.
    """

    def __init__(
        self,
        stream: TextIO | None,
        *,
        capacity: int = 64,
        flush_every_records: int = 16,
        flush_interval_sec: float = 1.0,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if capacity <= 0:
            raise ValueError("RH56 telemetry capacity must be positive")
        if flush_every_records <= 0 or flush_every_records > capacity:
            raise ValueError("RH56 flush batch must be within telemetry capacity")
        if flush_interval_sec <= 0.0:
            raise ValueError("RH56 flush interval must be positive")
        self.stream = stream
        self.capacity = int(capacity)
        self.flush_every_records = int(flush_every_records)
        self.flush_interval_sec = float(flush_interval_sec)
        self._monotonic = monotonic
        self._buffer: deque[str] = deque(maxlen=self.capacity)
        self._last_flush = float(monotonic())
        self.record_count = 0
        self.telemetry_record_count = 0
        self.fault_record_count = 0
        self.dropped_record_count = 0
        self.flush_count = 0
        self.last_record: dict[str, Any] | None = None
        self.last_telemetry_record: dict[str, Any] | None = None
        self.failures: deque[LoggingFailure] = deque(maxlen=16)

    def __call__(self, row: dict[str, Any]) -> None:
        self.record_count += 1
        if row.get("record_type") == "rh56_telemetry":
            self.telemetry_record_count += 1
            self.last_telemetry_record = row
        elif row.get("record_type") in {
            "rh56_worker_failure",
            "rh56_logging_failure",
        }:
            self.fault_record_count += 1
        self.last_record = row
        if self.stream is None:
            return
        try:
            serialized = json.dumps(row, sort_keys=True) + "\n"
        except Exception as exc:
            self._remember_failure("json_serialize", exc)
            return
        if len(self._buffer) == self.capacity:
            self.dropped_record_count += 1
        self._buffer.append(serialized)
        now = float(self._monotonic())
        immediate = row.get("record_type") in {
            "rh56_worker_failure",
            "rh56_logging_failure",
        }
        if (
            immediate
            or len(self._buffer) >= self.flush_every_records
            or now - self._last_flush >= self.flush_interval_sec
        ):
            self.flush()

    def flush(self) -> bool:
        if self.stream is None or not self._buffer:
            self._last_flush = float(self._monotonic())
            return True
        payload = "".join(self._buffer)
        try:
            self.stream.write(payload)
        except Exception as exc:
            self._remember_failure("jsonl_write", exc)
            return False
        # The payload has been accepted by TextIO.  Remove it before flush so a
        # failed flush cannot cause duplicate JSONL lines on a later retry.
        self._buffer.clear()
        try:
            self.stream.flush()
        except Exception as exc:
            self._remember_failure("jsonl_flush", exc)
            return False
        self.flush_count += 1
        self._last_flush = float(self._monotonic())
        return True

    def close(self) -> bool:
        return self.flush()

    @property
    def buffered_record_count(self) -> int:
        return len(self._buffer)

    def summary(self) -> dict[str, Any]:
        return {
            "record_count": self.record_count,
            "telemetry_record_count": self.telemetry_record_count,
            "fault_record_count": self.fault_record_count,
            "buffer_capacity": self.capacity,
            "buffered_record_count": len(self._buffer),
            "dropped_record_count": self.dropped_record_count,
            "flush_count": self.flush_count,
            "logging_failure_count": len(self.failures),
            "last_logging_failure": (
                None if not self.failures else self.failures[-1].as_dict()
            ),
        }

    def _remember_failure(self, operation: str, exc: BaseException) -> None:
        self.failures.append(
            LoggingFailure(operation, type(exc).__name__, str(exc))
        )
