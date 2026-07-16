"""Quantitative input-stream diagnostics with clock-safe latency math."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
import math
import statistics
import time
from typing import Any, Callable, Sequence

from .model import MotionInputSample, TrackingState


DIAGNOSTIC_WINDOW_SAMPLES = 100_000


@dataclass(slots=True)
class _StreamStats:
    samples: int = 0
    sequence_gaps: int = 0
    out_of_order_sequences: int = 0
    out_of_order_timestamps: int = 0
    interruptions: int = 0
    recoveries: int = 0
    last_sequence: int | None = None
    last_receive_ns: int | None = None
    receive_clock_id: str | None = None
    last_capture_ns: int | None = None
    capture_clock_id: str | None = None
    last_state: TrackingState | None = None
    interruption_started_ns: int | None = None
    interruption_clock_id: str | None = None
    receive_intervals_ns: deque[int] = field(
        default_factory=lambda: deque(maxlen=DIAGNOSTIC_WINDOW_SAMPLES)
    )
    capture_intervals_ns: deque[int] = field(
        default_factory=lambda: deque(maxlen=DIAGNOSTIC_WINDOW_SAMPLES)
    )
    latency_ns: deque[int] = field(default_factory=lambda: deque(maxlen=DIAGNOSTIC_WINDOW_SAMPLES))
    processing_latency_ns: deque[int] = field(
        default_factory=lambda: deque(maxlen=DIAGNOSTIC_WINDOW_SAMPLES)
    )
    confidence: deque[float] = field(
        default_factory=lambda: deque(maxlen=DIAGNOSTIC_WINDOW_SAMPLES)
    )
    interruption_durations_ns: deque[int] = field(
        default_factory=lambda: deque(maxlen=DIAGNOSTIC_WINDOW_SAMPLES)
    )
    tracking_state_counts: dict[str, int] = field(default_factory=lambda: defaultdict(int))


class StreamingDiagnostics:
    def __init__(
        self,
        *,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        process_time_ns: Callable[[], int] = time.process_time_ns,
    ) -> None:
        self._monotonic_ns = monotonic_ns
        self._process_time_ns = process_time_ns
        self._start_wall_ns = monotonic_ns()
        self._start_cpu_ns = process_time_ns()
        self._streams: dict[str, _StreamStats] = defaultdict(_StreamStats)
        self._noncomparable_latency_samples = 0

    def observe(self, sample: MotionInputSample) -> None:
        key = f"{sample.stream_id}:{sample.side.value}"
        stats = self._streams[key]
        stats.samples += 1
        stats.tracking_state_counts[sample.tracking_state.value] += 1
        if sample.tracking_confidence is not None:
            stats.confidence.append(sample.tracking_confidence)

        if stats.last_sequence is not None:
            if sample.sequence_number <= stats.last_sequence:
                stats.out_of_order_sequences += 1
            elif sample.sequence_number > stats.last_sequence + 1:
                stats.sequence_gaps += sample.sequence_number - stats.last_sequence - 1
        stats.last_sequence = sample.sequence_number

        receive_ns = sample.receive_timestamp.nanoseconds
        receive_clock = sample.receive_timestamp.clock_id
        if stats.receive_clock_id != receive_clock:
            stats.receive_clock_id = receive_clock
            stats.last_receive_ns = None
        if stats.last_receive_ns is not None:
            interval = receive_ns - stats.last_receive_ns
            if interval < 0:
                stats.out_of_order_timestamps += 1
            else:
                stats.receive_intervals_ns.append(interval)
        stats.last_receive_ns = receive_ns

        capture = sample.capture_timestamp
        if stats.capture_clock_id != capture.clock_id:
            stats.capture_clock_id = capture.clock_id
            stats.last_capture_ns = None
        if stats.last_capture_ns is not None:
            interval = capture.nanoseconds - stats.last_capture_ns
            if interval < 0:
                stats.out_of_order_timestamps += 1
            else:
                stats.capture_intervals_ns.append(interval)
        stats.last_capture_ns = capture.nanoseconds

        if capture.comparable_to(sample.receive_timestamp):
            stats.latency_ns.append(sample.receive_timestamp.nanoseconds - capture.nanoseconds)
        else:
            self._noncomparable_latency_samples += 1
        if sample.processing_timestamp is not None:
            if sample.processing_timestamp.comparable_to(sample.receive_timestamp):
                stats.processing_latency_ns.append(
                    sample.processing_timestamp.nanoseconds - receive_ns
                )

        previously_tracking = stats.last_state in (TrackingState.TRACKING, TrackingState.LIMITED)
        currently_tracking = sample.tracking_state in (TrackingState.TRACKING, TrackingState.LIMITED)
        if previously_tracking and not currently_tracking:
            stats.interruptions += 1
            stats.interruption_started_ns = receive_ns
            stats.interruption_clock_id = receive_clock
        elif not previously_tracking and currently_tracking and stats.last_state is not None:
            stats.recoveries += 1
            if (
                stats.interruption_started_ns is not None
                and stats.interruption_clock_id == receive_clock
            ):
                stats.interruption_durations_ns.append(receive_ns - stats.interruption_started_ns)
                stats.interruption_started_ns = None
                stats.interruption_clock_id = None
        stats.last_state = sample.tracking_state

    def report(self) -> dict[str, Any]:
        elapsed_wall_ns = max(1, self._monotonic_ns() - self._start_wall_ns)
        elapsed_cpu_ns = max(0, self._process_time_ns() - self._start_cpu_ns)
        streams: dict[str, Any] = {}
        for key, stats in sorted(self._streams.items()):
            intervals = stats.capture_intervals_ns or stats.receive_intervals_ns
            streams[key] = {
                "samples": stats.samples,
                "tracking_frequency_hz": _frequency(intervals),
                "frame_drops": stats.sequence_gaps,
                "out_of_order_sequences": stats.out_of_order_sequences,
                "out_of_order_timestamps": stats.out_of_order_timestamps,
                "timestamp_jitter_ms": _jitter_ms(intervals),
                "latency_ms": _summary_ms(stats.latency_ns),
                "processing_latency_ms": _summary_ms(stats.processing_latency_ns),
                "tracking_confidence": _summary(stats.confidence),
                "tracking_interruptions": stats.interruptions,
                "tracking_recoveries": stats.recoveries,
                "interruption_duration_ms": _summary_ms(stats.interruption_durations_ns),
                "tracking_state_counts": dict(stats.tracking_state_counts),
            }
        return {
            "schema_version": "1.0",
            "elapsed_seconds": elapsed_wall_ns / 1_000_000_000,
            "process_cpu_percent": 100.0 * elapsed_cpu_ns / elapsed_wall_ns,
            "noncomparable_latency_samples": self._noncomparable_latency_samples,
            "streams": streams,
        }


def _frequency(intervals_ns: Sequence[int]) -> float | None:
    positive = [value for value in intervals_ns if value > 0]
    if not positive:
        return None
    return 1_000_000_000 / statistics.mean(positive)


def _jitter_ms(intervals_ns: Sequence[int]) -> float | None:
    if len(intervals_ns) < 2:
        return None
    median = statistics.median(intervals_ns)
    return math.sqrt(statistics.mean((value - median) ** 2 for value in intervals_ns)) / 1e6


def _summary(values: Sequence[float]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(values)
    p95_index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "min": float(ordered[0]),
        "mean": float(statistics.mean(ordered)),
        "p95": float(ordered[p95_index]),
        "max": float(ordered[-1]),
    }


def _summary_ms(values_ns: Sequence[int]) -> dict[str, float] | None:
    summary = _summary([value / 1e6 for value in values_ns])
    return summary
