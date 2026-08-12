from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import queue
import threading
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
    serial failures.  The physical production path does not instantiate this
    synchronous primitive; commissioning wraps it with AsyncJsonlRecorder.
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


class AsyncJsonlRecorder:
    """Bounded asynchronous wrapper for explicit commissioning diagnostics."""

    def __init__(self, recorder: BoundedJsonlRecorder, *, capacity: int = 64) -> None:
        if capacity <= 0:
            raise ValueError("asynchronous recorder capacity must be positive")
        self.recorder = recorder
        self._queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=capacity)
        self._thread = threading.Thread(
            target=self._run,
            name="rh56-diagnostic-log",
            daemon=True,
        )
        self._started = False
        self._closed = False
        self.dropped_record_count = 0
        self.error_count = 0

    def start(self) -> None:
        if self._started:
            raise RuntimeError("asynchronous recorder already started")
        self._started = True
        self._thread.start()

    def __call__(self, row: dict[str, Any]) -> None:
        if not self._started or self._closed:
            return
        try:
            self._queue.put_nowait(row)
        except queue.Full:
            self.dropped_record_count += 1

    def close(self, timeout_s: float = 5.0) -> bool:
        if self._closed:
            return self.recorder.close()
        self._closed = True
        if self._started:
            try:
                self._queue.put(None, timeout=timeout_s)
            except queue.Full:
                self.dropped_record_count += self._queue.qsize()
            self._thread.join(timeout=timeout_s)
            if self._thread.is_alive():
                self.error_count += 1
        return self.recorder.close()

    @property
    def telemetry_record_count(self) -> int:
        return self.recorder.telemetry_record_count

    @property
    def last_telemetry_record(self) -> dict[str, Any] | None:
        return self.recorder.last_telemetry_record

    def summary(self) -> dict[str, Any]:
        return {
            **self.recorder.summary(),
            "async": True,
            "async_queue_capacity": self._queue.maxsize,
            "async_queue_dropped_record_count": self.dropped_record_count,
            "async_error_count": self.error_count,
        }

    def _run(self) -> None:
        while True:
            row = self._queue.get()
            try:
                if row is None:
                    return
                self.recorder(row)
            except BaseException:
                self.error_count += 1
            finally:
                self._queue.task_done()
