from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable, Mapping

from .episode import (
    CameraRecord,
    CameraFrameRef,
    CameraFrameUnavailable,
    CanonicalEpisodeWriter,
    CanonicalSample,
    EpisodeStatus,
    StartPrerequisites,
)


class AsyncEpisodeWriter:
    """Move frame serialization off control, camera, and preview threads."""

    def __init__(
        self,
        writer: CanonicalEpisodeWriter,
        *,
        capacity: int = 64,
        batch_size: int = 8,
        flush_interval_s: float = 1.0,
        shutdown_timeout_s: float = 5.0,
        require_frame_references: bool = False,
    ) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("episode writer queue capacity must be a positive integer")
        if isinstance(batch_size, bool) or not isinstance(batch_size, int) or batch_size <= 0:
            raise ValueError("episode writer batch size must be a positive integer")
        if flush_interval_s <= 0 or shutdown_timeout_s <= 0:
            raise ValueError("writer flush and shutdown intervals must be positive")
        self._writer = writer
        self._batch_size = batch_size
        self._flush_interval_s = float(flush_interval_s)
        self._shutdown_timeout_s = float(shutdown_timeout_s)
        self._require_frame_references = bool(require_frame_references)
        self._queue: queue.Queue[tuple[str, tuple[Any, ...]] | None] = queue.Queue(
            maxsize=capacity
        )
        self._error: BaseException | None = None
        self._closed = False
        self._metrics_lock = threading.Lock()
        self._enqueued_count = 0
        self._completed_count = 0
        self._queue_max_depth = 0
        self._overflow_count = 0
        self._drop_count = 0
        self._ring_overwrite_drop_count = 0
        self._ring_reference_expired_count = 0
        self._drop_by_method: dict[str, int] = {}
        self._enqueued_by_method: dict[str, int] = {}
        self._written_by_method: dict[str, int] = {}
        self._write_durations_ns: deque[int] = deque(maxlen=4096)
        self._batch_durations_ns: deque[int] = deque(maxlen=4096)
        self._flush_durations_ns: deque[int] = deque(maxlen=4096)
        self._writer_error_count = 0
        self._begin_failed = False
        self._accepted_sample_count = 0
        self._cached_start_ns: int | None = None
        self._worker_started_ns = time.perf_counter_ns()
        self._thread = threading.Thread(target=self._run, name="episode-writer", daemon=True)
        self._thread.start()

    @property
    def root(self) -> Path:
        return self._writer.root

    @property
    def temporary_id(self) -> str:
        return self._writer.temporary_id

    @property
    def dataset_fps(self) -> int:
        return self._writer.dataset_fps

    @property
    def sample_count(self) -> int:
        return self._accepted_sample_count

    @property
    def start_monotonic_ns(self) -> int | None:
        return self._cached_start_ns

    def set_final_metadata_provider(
        self, provider: Callable[[], Mapping[str, Any]]
    ) -> None:
        self._writer.set_final_metadata_provider(provider)

    def begin(self, prerequisites: StartPrerequisites, *, camera_max_age_ns: int) -> None:
        prerequisites.validate(camera_max_age_ns=camera_max_age_ns)
        self._cached_start_ns = prerequisites.accepted.host_monotonic_ns
        if not self._enqueue("begin", prerequisites, camera_max_age_ns):
            raise OSError("episode writer queue is full during begin")

    def append_raw(self, stream: str, record: Mapping[str, Any]) -> bool:
        return self._enqueue("append_raw", stream, dict(record))

    def append_raw_batch(self, records: list[tuple[str, Mapping[str, Any]]]) -> bool:
        return self._enqueue(
            "append_raw_batch",
            [(stream, dict(record)) for stream, record in records],
        )

    def append_raw_camera(self, camera: CameraRecord) -> bool:
        if self._require_frame_references and not isinstance(camera, CameraFrameRef):
            raise TypeError("live episode writer requires a camera ring reference")
        return self._enqueue("append_raw_camera", camera)

    def append_sample(self, sample: CanonicalSample) -> bool:
        if self._require_frame_references and not all(
            isinstance(camera, CameraFrameRef)
            for camera in (sample.workspace, sample.wrist)
        ):
            raise TypeError("live canonical samples require camera ring references")
        accepted = self._enqueue("append_sample", sample)
        if accepted:
            self._accepted_sample_count += 1
        return accepted

    def finalize(
        self,
        status: EpisodeStatus,
        *,
        termination_reason: str,
        trigger_release_monotonic_ns: int | None,
        report: Mapping[str, Any] | None = None,
    ) -> Path:
        final_report = dict(report or {})
        if self._error is not None:
            status = EpisodeStatus.ABORTED
            termination_reason = "async_data_write_failure"
            final_report["async_data_write_error"] = (
                f"{type(self._error).__name__}: {self._error}"
            )
        result = self._terminal_call(
            "finalize",
            status,
            termination_reason,
            trigger_release_monotonic_ns,
            final_report,
        )
        try:
            assert isinstance(result, Path)
            return result
        finally:
            self._close_worker()

    def discard_rejected_start(self, reason: str) -> Path:
        try:
            result = self._terminal_call("discard_rejected_start", reason)
            assert isinstance(result, Path)
            return result
        finally:
            self._close_worker()

    def close(self) -> None:
        """Close an unused writer without creating a rejected-start record."""

        if self._closed:
            return
        if self._cached_start_ns is not None:
            raise RuntimeError(
                "an active episode must be finalized or discarded before close"
            )
        self._close_worker()

    def diagnostics(self) -> dict[str, object]:
        """Return bounded queue/write telemetry for the capture summary."""

        with self._metrics_lock:
            durations = sorted(self._write_durations_ns)
            batches = sorted(self._batch_durations_ns)
            flushes = sorted(self._flush_durations_ns)
            enqueued = self._enqueued_count
            completed = self._completed_count
            maximum_depth = self._queue_max_depth
            overflow = self._overflow_count
            drop_count = self._drop_count
            ring_expired = self._ring_reference_expired_count
            dropped_by_method = dict(self._drop_by_method)
            enqueued_by_method = dict(self._enqueued_by_method)
            written_by_method = dict(self._written_by_method)
            writer_errors = self._writer_error_count
        return {
            "queue_capacity": self._queue.maxsize,
            "queue_depth": self._queue.qsize(),
            "queue_max_depth": maximum_depth,
            "recorder_queue_size": self._queue.qsize(),
            "recorder_queue_high_watermark": maximum_depth,
            "recorder_queue_full_count": overflow,
            "queue_full_count": overflow,
            "recorder_drop_count": drop_count,
            "ring_overwrite_drop_count": ring_expired,
            "ring_reference_expired_count": ring_expired,
            "recorder_enqueued_count": enqueued,
            "recorder_written_count": completed,
            "recorder_dropped_count": drop_count,
            "canonical_enqueued_count": enqueued_by_method.get("append_sample", 0),
            "canonical_written_count": written_by_method.get("append_sample", 0),
            "canonical_dropped_count": dropped_by_method.get("append_sample", 0),
            "recorder_dropped_by_method": dropped_by_method,
            "recorder_enqueued_by_method": enqueued_by_method,
            "recorder_written_by_method": written_by_method,
            "enqueued_count": enqueued,
            "completed_count": completed,
            "writer_write_duration_ns": _summary(durations),
            "writer_batch_duration_ns": _summary(batches),
            "writer_flush_duration_ns": _summary(flushes),
            "writer_error_count": writer_errors,
            "writer_batch_size": self._batch_size,
            "writer_bytes_per_second": (
                float(getattr(self._writer, "bytes_written", 0))
                / max((time.perf_counter_ns() - self._worker_started_ns) / 1e9, 1e-9)
            ),
            "thread_alive": self._thread.is_alive(),
        }

    def _enqueue(self, method: str, *args: Any) -> bool:
        self._raise_if_failed()
        if self._closed:
            raise OSError("episode writer is closed")
        try:
            self._queue.put_nowait((method, args))
        except queue.Full as exc:
            with self._metrics_lock:
                self._overflow_count += 1
                self._drop_count += 1
                self._drop_by_method[method] = self._drop_by_method.get(method, 0) + 1
            return False
        with self._metrics_lock:
            self._enqueued_count += 1
            self._enqueued_by_method[method] = self._enqueued_by_method.get(method, 0) + 1
            self._queue_max_depth = max(self._queue_max_depth, self._queue.qsize())
        return True

    def _raise_if_failed(self) -> None:
        if self._error is not None:
            raise OSError("episode writer worker failed") from self._error

    def _close_worker(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._queue.put(None, timeout=self._shutdown_timeout_s)
        except queue.Full as exc:
            raise TimeoutError("episode writer shutdown queue remained full") from exc
        self._thread.join(timeout=self._shutdown_timeout_s)
        if self._thread.is_alive():
            raise RuntimeError("episode writer worker did not stop")

    def _terminal_call(self, method: str, *args: Any) -> Any:
        done = threading.Event()
        result: dict[str, Any] = {}
        try:
            self._queue.put(
                ("__terminal__", (method, args, done, result)),
                timeout=self._shutdown_timeout_s,
            )
        except queue.Full as exc:
            raise TimeoutError("episode writer did not accept finalization") from exc
        if not done.wait(self._shutdown_timeout_s):
            # The in-flight filesystem call cannot be cancelled safely. Queue
            # a terminal sentinel so the daemon exits immediately after it
            # returns; the .partial metadata remains the integrity marker.
            try:
                self._queue.put_nowait(None)
                self._closed = True
            except queue.Full:
                pass
            raise TimeoutError("episode writer finalization exceeded bounded shutdown")
        if "error" in result:
            raise result["error"]
        return result.get("value")

    def _run(self) -> None:
        next_flush = time.monotonic() + self._flush_interval_s
        while True:
            items = [self._queue.get()]
            for _ in range(self._batch_size - 1):
                try:
                    items.append(self._queue.get_nowait())
                except queue.Empty:
                    break
            batch_started_ns = time.perf_counter_ns()
            stop = False
            for item in items:
                try:
                    if item is None:
                        stop = True
                        continue
                    self._execute(item)
                finally:
                    self._queue.task_done()
            with self._metrics_lock:
                self._batch_durations_ns.append(
                    time.perf_counter_ns() - batch_started_ns
                )
            if stop:
                return
            if time.monotonic() >= next_flush and self._writer.start_monotonic_ns is not None:
                started_ns = time.perf_counter_ns()
                try:
                    self._writer.flush_pending()
                except BaseException as exc:
                    self._error = exc
                    with self._metrics_lock:
                        self._writer_error_count += 1
                with self._metrics_lock:
                    self._flush_durations_ns.append(time.perf_counter_ns() - started_ns)
                next_flush = time.monotonic() + self._flush_interval_s

    def _execute(self, item: tuple[str, tuple[Any, ...]]) -> None:
        method, args = item
        if method == "__terminal__":
            terminal_method, terminal_args, done, result = args
            try:
                if terminal_method == "finalize":
                    status, reason, release_ns, report = terminal_args
                    if self._begin_failed:
                        result["value"] = self._writer.discard_rejected_start(
                            "async_episode_begin_failure"
                        )
                        return
                    if self._error is not None:
                        status = EpisodeStatus.ABORTED
                        reason = "async_data_write_failure"
                        report = {
                            **report,
                            "async_data_write_error": (
                                f"{type(self._error).__name__}: {self._error}"
                            ),
                        }
                    result["value"] = self._writer.finalize(
                        status,
                        termination_reason=reason,
                        trigger_release_monotonic_ns=release_ns,
                        report=report,
                    )
                else:
                    result["value"] = getattr(self._writer, terminal_method)(
                        *terminal_args
                    )
            except BaseException as exc:
                result["error"] = exc
            finally:
                done.set()
            return
        if self._error is not None:
            return
        try:
            started_ns = time.perf_counter_ns()
            if method == "begin":
                prerequisites, camera_max_age_ns = args
                self._writer.begin(
                    prerequisites,
                    camera_max_age_ns=camera_max_age_ns,
                )
            else:
                if method == "append_sample":
                    args = (
                        replace(args[0], frame_index=self._writer.sample_count),
                    )
                getattr(self._writer, method)(*args)
            elapsed_ns = time.perf_counter_ns() - started_ns
            with self._metrics_lock:
                self._write_durations_ns.append(elapsed_ns)
                self._completed_count += 1
                self._written_by_method[method] = self._written_by_method.get(method, 0) + 1
        except CameraFrameUnavailable as exc:
            with self._metrics_lock:
                self._ring_overwrite_drop_count += 1
                self._ring_reference_expired_count += 1
                self._drop_count += 1
                self._drop_by_method[method] = self._drop_by_method.get(method, 0) + 1
            if method == "append_sample":
                self._write_expired_sample_quality(args[0], exc)
        except BaseException as exc:
            if method == "begin":
                self._begin_failed = True
            self._error = exc
            with self._metrics_lock:
                self._writer_error_count += 1

    def _write_expired_sample_quality(
        self, sample: CanonicalSample, failed: CameraFrameUnavailable
    ) -> None:
        """Persist a metadata-only row; never substitute a newer ring frame."""

        quality: dict[str, Any] = {
            "record_type": "canonical_data_quality",
            "canonical_timestamp_ns": sample.timestamp_ns,
            "nominal_slot_index": sample.nominal_slot_index,
            "metadata_only": True,
            "reason": "ring_reference_expired",
            "control": {
                "host_monotonic_ns": sample.control.host_monotonic_ns,
                "accepted_target_sequence": sample.control.accepted_target_sequence,
                "arm_action_status": sample.control.arm_action_status,
                "hand_target": list(sample.control.hand_target or ()),
            },
        }
        additional_expired = 0
        for role, camera in (("workspace", sample.workspace), ("wrist", sample.wrist)):
            valid = role != failed.role
            sequence = getattr(camera, "sequence", None)
            if valid and isinstance(camera, CameraFrameRef):
                try:
                    camera.snapshot()
                except CameraFrameUnavailable:
                    valid = False
                    additional_expired += 1
            quality[f"{role}_valid"] = valid
            quality[f"{role}_frame_sequence"] = sequence
            quality[f"{role}_stale_reason"] = None if valid else "ring_reference_expired"
            quality[f"{role}_age_ns"] = max(
                0, sample.timestamp_ns - camera.host_monotonic_ns
            )
        try:
            if additional_expired:
                with self._metrics_lock:
                    self._ring_overwrite_drop_count += additional_expired
                    self._ring_reference_expired_count += additional_expired
            self._writer.append_raw("data_quality", quality)
        except BaseException as exc:
            self._error = exc
            with self._metrics_lock:
                self._writer_error_count += 1


def _summary(values: list[int]) -> dict[str, int]:
    if not values:
        return {"count": 0, "p50": 0, "p95": 0, "p99": 0, "max": 0}
    last = len(values) - 1
    return {
        "count": len(values),
        "p50": values[round(last * 0.50)],
        "p95": values[round(last * 0.95)],
        "p99": values[round(last * 0.99)],
        "max": values[-1],
    }
