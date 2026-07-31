from __future__ import annotations

import queue
import threading
from pathlib import Path
from typing import Any, Callable, Mapping

from .episode import (
    CameraSample,
    CanonicalEpisodeWriter,
    CanonicalSample,
    EpisodeStatus,
    StartPrerequisites,
)


class AsyncEpisodeWriter:
    """Move frame serialization off control, camera, and preview threads."""

    def __init__(self, writer: CanonicalEpisodeWriter, *, capacity: int = 256) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("episode writer queue capacity must be a positive integer")
        self._writer = writer
        self._queue: queue.Queue[tuple[str, tuple[Any, ...]] | None] = queue.Queue(
            maxsize=capacity
        )
        self._error: BaseException | None = None
        self._closed = False
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
        return self._writer.sample_count

    @property
    def start_monotonic_ns(self) -> int | None:
        return self._writer.start_monotonic_ns

    def set_final_metadata_provider(
        self, provider: Callable[[], Mapping[str, Any]]
    ) -> None:
        self._writer.set_final_metadata_provider(provider)

    def begin(self, prerequisites: StartPrerequisites, *, camera_max_age_ns: int) -> None:
        self._raise_if_failed()
        self._writer.begin(prerequisites, camera_max_age_ns=camera_max_age_ns)

    def append_raw(self, stream: str, record: Mapping[str, Any]) -> None:
        self._enqueue("append_raw", stream, dict(record))

    def append_raw_batch(self, records: list[tuple[str, Mapping[str, Any]]]) -> None:
        self._enqueue(
            "append_raw_batch",
            [(stream, dict(record)) for stream, record in records],
        )

    def append_raw_camera(self, camera: CameraSample) -> None:
        self._enqueue("append_raw_camera", camera)

    def append_sample(self, sample: CanonicalSample) -> None:
        self._enqueue("append_sample", sample)

    def finalize(
        self,
        status: EpisodeStatus,
        *,
        termination_reason: str,
        trigger_release_monotonic_ns: int | None,
        report: Mapping[str, Any] | None = None,
    ) -> Path:
        self._queue.join()
        final_report = dict(report or {})
        if self._error is not None:
            status = EpisodeStatus.ABORTED
            termination_reason = "async_data_write_failure"
            final_report["async_data_write_error"] = (
                f"{type(self._error).__name__}: {self._error}"
            )
        try:
            return self._writer.finalize(
                status,
                termination_reason=termination_reason,
                trigger_release_monotonic_ns=trigger_release_monotonic_ns,
                report=final_report,
            )
        finally:
            self._close_worker()

    def discard_rejected_start(self, reason: str) -> Path:
        self._queue.join()
        try:
            return self._writer.discard_rejected_start(reason)
        finally:
            self._close_worker()

    def close(self) -> None:
        """Close an unused writer without creating a rejected-start record."""

        if self._closed:
            return
        if self._writer.start_monotonic_ns is not None:
            raise RuntimeError(
                "an active episode must be finalized or discarded before close"
            )
        self._queue.join()
        self._close_worker()

    def _enqueue(self, method: str, *args: Any) -> None:
        self._raise_if_failed()
        if self._closed:
            raise OSError("episode writer is closed")
        try:
            self._queue.put_nowait((method, args))
        except queue.Full as exc:
            raise OSError("episode writer queue is full") from exc

    def _raise_if_failed(self) -> None:
        if self._error is not None:
            raise OSError("episode writer worker failed") from self._error

    def _close_worker(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._queue.put(None)
        self._thread.join(timeout=3.0)
        if self._thread.is_alive():
            raise RuntimeError("episode writer worker did not stop")

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                method, args = item
                if self._error is None:
                    try:
                        getattr(self._writer, method)(*args)
                    except BaseException as exc:
                        self._error = exc
            finally:
                self._queue.task_done()
