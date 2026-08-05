#!/usr/bin/env python3
"""Short offline A/B/C benchmark for combined teleoperation process isolation."""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import queue
import resource
import time
from math import sin
from typing import Any

import numpy as np

from episode_dataset.episode import CameraSample
from episode_dataset.episode import CameraFrameUnavailable
from episode_dataset.process_runtime import (
    FrameReferenceDescriptor,
    SharedMemoryCameraFrameRing,
    _queue_put_latest,
)


def _summary(values: list[int]) -> dict[str, int]:
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


def _fake_producer(spec, control_queue, recorder_queue, stop, result_queue) -> None:
    ring = SharedMemoryCameraFrameRing.attach(spec)
    publish_durations: list[int] = []
    intervals: list[int] = []
    previous_ns: int | None = None
    try:
        sequence = 0
        while not stop.is_set():
            now_ns = time.monotonic_ns()
            sample = CameraSample(
                role=spec.role,
                host_monotonic_ns=now_ns,
                rgb=np.zeros(spec.rgb_shape, dtype=np.uint8),
                depth_raw=np.zeros(spec.depth_shape, dtype=np.uint16),
                depth_aligned_to_rgb=(
                    None
                    if spec.aligned_depth_shape is None
                    else np.zeros(spec.aligned_depth_shape, dtype=np.uint16)
                ),
                depth_scale_m=0.001,
                device_rgb_timestamp_ms=sequence / 30.0,
                device_depth_timestamp_ms=sequence / 30.0,
                rgb_frame_number=sequence,
                depth_frame_number=sequence,
                rgb_timestamp_domain="offline",
                depth_timestamp_domain="offline",
            )
            started_ns = time.perf_counter_ns()
            reference = ring.publish(sample, sequence)
            publish_durations.append(time.perf_counter_ns() - started_ns)
            if previous_ns is not None:
                intervals.append(now_ns - previous_ns)
            previous_ns = now_ns
            descriptor = FrameReferenceDescriptor.from_reference(reference)
            _queue_put_latest(control_queue, descriptor)
            if recorder_queue is not None:
                _queue_put_latest(recorder_queue, descriptor)
            sequence += 1
            time.sleep(max(0.0, 1.0 / 30.0 - 0.0002))
    finally:
        result_queue.put(
            {
                "role": spec.role,
                "camera_publish_duration_ns": _summary(publish_durations),
                "camera_interframe_interval_ns": _summary(intervals),
                "frames": len(publish_durations),
            }
        )
        ring.close()


def _slow_recorder(ring_specs, recorder_queues, stop, result_queue) -> None:
    rings = {role: SharedMemoryCameraFrameRing.attach(spec) for role, spec in ring_specs.items()}
    high_watermark = 0
    materialization_durations: list[int] = []
    expired = 0
    try:
        while not stop.is_set():
            latest = []
            for role, source in recorder_queues.items():
                try:
                    descriptor = source.get_nowait()
                except queue.Empty:
                    continue
                latest.append((role, descriptor))
                high_watermark = max(high_watermark, source.qsize())
            for role, descriptor in latest:
                started_ns = time.perf_counter_ns()
                try:
                    descriptor.to_reference(rings[role]).snapshot()
                except CameraFrameUnavailable:
                    expired += 1
                materialization_durations.append(time.perf_counter_ns() - started_ns)
            time.sleep(0.04)
    finally:
        result_queue.put(
            {
                "recorder_queue_high_watermark": high_watermark,
                "recorder_expired_reference_count": expired,
                "frame_materialization_duration_ns": _summary(materialization_durations),
            }
        )
        for ring in rings.values():
            ring.close()


def _control_work() -> None:
    accumulator = 0.0
    for index in range(4000):
        accumulator += sin(index * 0.001)
    if accumulator == float("inf"):  # keep the workload observable to the interpreter
        raise AssertionError("unreachable")


def run_scenario(name: str, duration_s: float) -> dict[str, Any]:
    context = mp.get_context("spawn")
    rings: dict[str, SharedMemoryCameraFrameRing] = {}
    control_queues: dict[str, Any] = {}
    recorder_queues: dict[str, Any] = {}
    stop = context.Event()
    producers: list[mp.Process] = []
    results = context.Queue(maxsize=8)
    with_cameras = name in {"B", "C"}
    with_recorder = name == "C"
    try:
        if with_cameras:
            for role in ("workspace", "wrist"):
                rings[role] = SharedMemoryCameraFrameRing.create(
                    role=role,
                    capacity=8,
                    rgb_shape=(480, 640, 3),
                    depth_shape=(480, 640),
                    aligned_depth_shape=(480, 640),
                )
                control_queues[role] = context.Queue(maxsize=8)
                recorder_queues[role] = context.Queue(maxsize=8) if with_recorder else None
                producers.append(
                    context.Process(
                        target=_fake_producer,
                        args=(
                            rings[role].spec,
                            control_queues[role],
                            recorder_queues[role],
                            stop,
                            results,
                        ),
                        name=f"benchmark-camera-{role}",
                    )
                )
            for producer in producers:
                producer.start()
        recorder = None
        if with_recorder:
            recorder = context.Process(
                target=_slow_recorder,
                args=(
                    {role: ring.spec for role, ring in rings.items()},
                    recorder_queues,
                    stop,
                    results,
                ),
                name="benchmark-recorder",
            )
            recorder.start()
        durations: list[int] = []
        budget_exhausted = 0
        started = time.perf_counter_ns()
        deadline = time.monotonic() + duration_s
        next_tick = time.monotonic()
        while time.monotonic() < deadline:
            next_tick += 1.0 / 30.0
            remaining = next_tick - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            tick_started = time.perf_counter_ns()
            if with_cameras:
                for source in control_queues.values():
                    while True:
                        try:
                            source.get_nowait()
                        except queue.Empty:
                            break
            _control_work()
            elapsed = time.perf_counter_ns() - tick_started
            durations.append(elapsed)
            budget_exhausted += int(elapsed > 20_000_000)
        shutdown_started = time.perf_counter_ns()
        stop.set()
        for producer in producers:
            producer.join(timeout=3.0)
        if with_recorder and recorder is not None:
            recorder.join(timeout=3.0)
        shutdown_time_ms = (time.perf_counter_ns() - shutdown_started) / 1e6
        child_results = []
        while True:
            try:
                child_results.append(results.get_nowait())
            except queue.Empty:
                break
        camera_results = [item for item in child_results if "role" in item]
        recorder_results = [item for item in child_results if "recorder_queue_high_watermark" in item]
        return {
            "scenario": name,
            "duration_s": duration_s,
            "control_duration_ns": _summary(durations),
            "control_budget_exhausted_count": budget_exhausted,
            "camera_publish_duration_ns": {
                role: item["camera_publish_duration_ns"] for item in camera_results for role in [item["role"]]
            },
            "camera_interframe_interval_ns": {
                role: item["camera_interframe_interval_ns"] for item in camera_results for role in [item["role"]]
            },
            "recorder_queue_high_watermark": max(
                [item["recorder_queue_high_watermark"] for item in recorder_results] or [0]
            ),
            "recorder_expired_reference_count": sum(
                item.get("recorder_expired_reference_count", 0) for item in recorder_results
            ),
            "frame_materialization_duration_ns": (
                recorder_results[0]["frame_materialization_duration_ns"] if recorder_results else None
            ),
            "rss_kb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            "shutdown_time_ms": shutdown_time_ms,
            "producer_exitcodes": [producer.exitcode for producer in producers],
            "recorder_exitcode": None if not with_recorder else recorder.exitcode,
            "wall_elapsed_s": (time.perf_counter_ns() - started) / 1e9,
        }
    finally:
        stop.set()
        for process in producers:
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
        if "recorder" in locals() and recorder is not None and recorder.is_alive():
            recorder.terminate()
            recorder.join(timeout=1.0)
        for source in control_queues.values():
            source.close()
        for source in recorder_queues.values():
            if source is not None:
                source.close()
        results.close()
        for ring in rings.values():
            ring.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=3.0)
    args = parser.parse_args()
    if not 0.1 <= args.duration_sec <= 30.0:
        parser.error("--duration-sec must be between 0.1 and 30 seconds")
    print(json.dumps({name: run_scenario(name, args.duration_sec) for name in ("A", "B", "C")}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
