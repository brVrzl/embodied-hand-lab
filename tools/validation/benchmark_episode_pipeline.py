#!/usr/bin/env python3
"""Offline load benchmark for bounded RGB-D recording and preview paths."""

from __future__ import annotations

import argparse
from collections import deque
import json
import queue
import random
import resource
import threading
import time
from typing import Any

import numpy as np

from episode_dataset.camera import CameraFrameRing
from episode_dataset.episode import CameraFrameRef, CameraFrameUnavailable, CameraSample


def _summary(values: deque[int] | list[int]) -> dict[str, int]:
    ordered = sorted(values)
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


def _sample(role: str, timestamp_ns: int, sequence: int, rgb: np.ndarray, depth: np.ndarray) -> CameraSample:
    return CameraSample(
        role=role,
        host_monotonic_ns=timestamp_ns,
        rgb=rgb,
        depth_raw=depth,
        depth_aligned_to_rgb=None,
        depth_scale_m=0.001,
        device_rgb_timestamp_ms=sequence / 0.03,
        device_depth_timestamp_ms=sequence / 0.03,
        rgb_frame_number=sequence,
        depth_frame_number=sequence,
        rgb_timestamp_domain="offline_virtual_clock",
        depth_timestamp_domain="offline_virtual_clock",
    )


class _Writer:
    def __init__(self, capacity: int, delay_range_ms: tuple[float, float]) -> None:
        self.queue: queue.Queue[tuple[CameraFrameRef, CameraFrameRef, int] | None] = queue.Queue(capacity)
        self.delay_range_ms = delay_range_ms
        self.stop = threading.Event()
        self.high_watermark = 0
        self.full_count = 0
        self.drop_count = 0
        self.overwrite_drop_count = 0
        self.error_count = 0
        self.written_valid = 0
        self.written_invalid = 0
        self.invalid_run = 0
        self.longest_invalid_run = 0
        self.bytes = 0
        self.durations_ns: deque[int] = deque(maxlen=4096)
        self._thread = threading.Thread(target=self._run, name="offline-benchmark-writer")
        self._thread.start()

    def publish(self, item: tuple[CameraFrameRef, CameraFrameRef, int]) -> None:
        try:
            self.queue.put_nowait(item)
            self.high_watermark = max(self.high_watermark, self.queue.qsize())
        except queue.Full:
            self.full_count += 1
            self.drop_count += 1

    def close(self, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                self.queue.put(None, timeout=max(0.0, deadline - time.monotonic()))
                break
            except queue.Full:
                if time.monotonic() >= deadline:
                    return False
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        return not self._thread.is_alive()

    def _run(self) -> None:
        rng = random.Random(7)
        completed = 0
        while True:
            item = self.queue.get()
            if item is None:
                return
            started_ns = time.perf_counter_ns()
            workspace, wrist, _ = item
            try:
                for reference in (workspace, wrist):
                    frame = reference.snapshot()
                    self.bytes += frame.rgb.nbytes + frame.depth_raw.nbytes
                completed += 1
                self.written_valid += 1
                self.invalid_run = 0
                if completed % 30 == 0 and self.delay_range_ms[1] > 0:
                    time.sleep(rng.uniform(*self.delay_range_ms) / 1000.0)
            except CameraFrameUnavailable:
                self.overwrite_drop_count += 1
                self.drop_count += 1
                self.written_invalid += 1
                self.invalid_run += 1
                self.longest_invalid_run = max(self.longest_invalid_run, self.invalid_run)
            except Exception:
                self.error_count += 1
            self.durations_ns.append(time.perf_counter_ns() - started_ns)


def run_benchmark(
    *,
    samples: int,
    queue_capacity: int,
    ring_capacity: int,
    preview_fps: float,
    writer_delay_range_ms: tuple[float, float],
    paced_seconds: float | None = None,
) -> dict[str, Any]:
    rings = {
        role: CameraFrameRing(ring_capacity) for role in ("workspace", "wrist")
    }
    rgb = {
        role: np.zeros((480, 640, 3), dtype=np.uint8)
        for role in ("workspace", "wrist")
    }
    depth = {
        role: np.zeros((480, 640), dtype=np.uint16)
        for role in ("workspace", "wrist")
    }
    writer = _Writer(queue_capacity, writer_delay_range_ms)
    control_ns: deque[int] = deque(maxlen=4096)
    canonical_ns: deque[int] = deque(maxlen=4096)
    frame_ages_ns: deque[int] = deque(maxlen=4096)
    preview_latencies_ns: deque[int] = deque(maxlen=4096)
    preview_drop_count = 0
    previous_preview_sequence: int | None = None
    preview_period = max(1, round(30.0 / preview_fps))
    period_ns = round(1_000_000_000 / 30)
    virtual_start_ns = time.monotonic_ns()
    wall_started_ns = time.perf_counter_ns()

    paced = paced_seconds is not None
    if paced:
        samples = max(1, round(float(paced_seconds) * 30.0))
    next_deadline = time.monotonic()
    published_frames = 0
    for index in range(samples):
        if paced:
            next_deadline += period_ns / 1e9
            remaining = next_deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        canonical_timestamp_ns = virtual_start_ns + index * period_ns
        timestamp_ns = time.monotonic_ns()
        references = {}
        for role in ("workspace", "wrist"):
            rgb[role][0, 0, 0] = index % 255
            depth[role][0, 0] = index % 65535
            references[role] = rings[role].publish(
                _sample(role, timestamp_ns, index, rgb[role], depth[role])
            )
            published_frames += 1

        control_started_ns = time.perf_counter_ns()
        canonical_started_ns = time.perf_counter_ns()
        writer.publish(
            (
                references["workspace"],
                references["wrist"],
                canonical_timestamp_ns,
            )
        )
        canonical_ns.append(time.perf_counter_ns() - canonical_started_ns)
        control_ns.append(time.perf_counter_ns() - control_started_ns)
        frame_ages_ns.append(max(0, time.monotonic_ns() - references["wrist"].host_monotonic_ns))

        if index % preview_period == 0:
            preview_started_ns = time.monotonic_ns()
            try:
                references["workspace"].snapshot()
                references["wrist"].snapshot()
            except CameraFrameUnavailable:
                preview_drop_count += 1
            if previous_preview_sequence is not None:
                preview_drop_count += max(0, index - previous_preview_sequence - 1)
            previous_preview_sequence = index
            preview_latencies_ns.append(max(0, preview_started_ns - timestamp_ns))
        # Accelerated relative to 30 Hz, but yield enough for the normal
        # writer to establish a meaningful no-backpressure baseline.
        time.sleep(0.001)

    shutdown_started_ns = time.perf_counter_ns()
    bounded_shutdown = writer.close()
    shutdown_time_s = (time.perf_counter_ns() - shutdown_started_ns) / 1e9
    elapsed_s = (time.perf_counter_ns() - wall_started_ns) / 1e9
    return {
        "samples": samples,
        "paced_seconds": paced_seconds,
        "expected_camera_frames_per_role": samples,
        "published_camera_frames_per_role": samples,
        "recorded_valid_canonical_frames": writer.written_valid,
        "recorded_invalid_canonical_frames": writer.written_invalid,
        "validity_ratio": writer.written_valid / max(samples, 1),
        "longest_invalid_run": writer.longest_invalid_run,
        "ring_occupancy": {role: rings[role].capacity for role in rings},
        "queue_occupancy": writer.high_watermark,
        "process_rss_kb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "shutdown_time_s": shutdown_time_s,
        "image_profile": "two 640x480 RGB uint8 plus two 640x480 depth uint16",
        "camera_fps": 30,
        "preview_fps": preview_fps,
        "writer_periodic_delay_ms": list(writer_delay_range_ms),
        "elapsed_s": elapsed_s,
        "control_loop_duration_ns": _summary(control_ns),
        "canonical_compute_duration_ns": _summary(canonical_ns),
        "wrist_frame_age_ns": _summary(frame_ages_ns),
        "preview_latency_ns": _summary(preview_latencies_ns),
        "preview_drop_count": preview_drop_count,
        "recorder_queue_high_watermark": writer.high_watermark,
        "recorder_queue_full_count": writer.full_count,
        "recorder_drop_count": writer.drop_count,
        "camera_ring_overwrite_count": sum(ring.overwrite_count for ring in rings.values()),
        "camera_ring_reference_expired_count": sum(
            ring.reference_expired_count for ring in rings.values()
        ),
        "camera_inconsistent_read_count": sum(ring.inconsistent_read_count for ring in rings.values()),
        "writer_write_duration_ns": _summary(writer.durations_ns),
        "writer_bytes_per_second": writer.bytes / max(elapsed_s, 1e-9),
        "writer_error_count": writer.error_count,
        "bounded_shutdown": bounded_shutdown,
        "global_block_detected": max(control_ns, default=0) > 10_000_000,
        "episode_abort": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--queue-capacity", type=int, default=64)
    parser.add_argument("--ring-capacity", type=int, default=16)
    parser.add_argument("--preview-fps", type=float, default=7.5)
    parser.add_argument(
        "--paced-seconds",
        type=float,
        default=None,
        help="also run wall-clock 30 Hz scenarios for this duration (e.g. 120)",
    )
    args = parser.parse_args()
    if args.samples <= 0:
        parser.error("--samples must be positive")
    if args.paced_seconds is not None and args.paced_seconds <= 0:
        parser.error("--paced-seconds must be positive")
    results = {
        "normal_writer": run_benchmark(
            samples=args.samples,
            queue_capacity=args.queue_capacity,
            ring_capacity=args.ring_capacity,
            preview_fps=args.preview_fps,
            writer_delay_range_ms=(0.0, 0.0),
        ),
        "periodically_slow_writer": run_benchmark(
            samples=args.samples,
            queue_capacity=args.queue_capacity,
            ring_capacity=args.ring_capacity,
            preview_fps=args.preview_fps,
            writer_delay_range_ms=(50.0, 150.0),
        ),
    }
    if args.paced_seconds is not None:
        for delay in (0.0, 50.0, 100.0, 150.0):
            results[f"wall_clock_writer_{int(delay)}ms"] = run_benchmark(
                samples=args.samples,
                queue_capacity=args.queue_capacity,
                ring_capacity=args.ring_capacity,
                preview_fps=args.preview_fps,
                writer_delay_range_ms=(delay, delay),
                paced_seconds=args.paced_seconds,
            )
    print(json.dumps(results, indent=2, sort_keys=True))
    return 0 if all(result["bounded_shutdown"] for result in results.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
