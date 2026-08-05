#!/usr/bin/env python3
"""Short offline A/B/C/D control-process timing benchmark.

A: Quest/control workload only
B: A plus bounded episode metadata publication
C: B plus a fake RH56 feedback worker
D: C plus active fake RH56 command/contact work

No device, camera, recorder process, or native JAKA worker is opened.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np

from motion_input import Pose6D, ReceivedHtsDatagram
from quest_jaka_sim import (
    AnalogClutchSample,
    RecordingArmTargetAdapter,
    ReplayConfig,
    SharedJakaTargetGenerator,
    SmoothQuestJakaSession,
)
from quest_jaka_sim.control_timing import CONTROL_TIMING_FIELDS


CONFIG = Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")


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


def _hand_packet(sequence: int, timestamp_ns: int) -> ReceivedHtsDatagram:
    landmarks = ",".join("0" for _ in range(63))
    payload = (
        f"Right wrist | f = {sequence}:, 0.0,0.0,0.0,0.0,0.0,0.0,1.0\n"
        f"Right landmarks | f = {sequence}:, {landmarks}"
    ).encode()
    return ReceivedHtsDatagram(payload, "offline", 9000, timestamp_ns, timestamp_ns)


def _head_packet(sequence: int, timestamp_ns: int) -> ReceivedHtsDatagram:
    payload = (
        f"Head pose | f = {sequence}:, 0,0,0,0.0,0.0,0.0,1.0"
    ).encode()
    return ReceivedHtsDatagram(payload, "offline", 9001, timestamp_ns, timestamp_ns)


class _FakeRh56:
    max_target_normalized = 1.0

    def __init__(self, *, active: bool) -> None:
        self.active = active
        self.feedback = (0.1, 0.1, 0.1, 0.1, 0.05, 0.5)
        self.command_count = 0
        self.contact_count = 0
        self._force = np.zeros(6, dtype=np.float64)

    def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
        return self.feedback

    def submit_target(self, target, monotonic_ns: int) -> None:
        self.command_count += 1
        if not self.active:
            return
        values = np.asarray(tuple(float(value) for value in target), dtype=np.float64)
        # Keep the contact workload deterministic and bounded while retaining
        # the same vector comparisons used by the production safety path.
        self._force = np.abs(values - np.asarray(self.feedback)) * 100.0
        self.contact_count += int(bool(np.any(self._force > 20.0)))
        self.feedback = tuple(float(value) for value in values)

    def hold(self, reason: str) -> None:
        return None


class _MetadataPublisher:
    def __init__(self) -> None:
        self.mailbox: deque[tuple[Any, ...]] = deque(maxlen=16)
        self.published = 0

    def publish(self, tick, sequence: int) -> None:
        target = tick.accepted_target
        self.mailbox.append(
            (
                sequence,
                None if target is None else tuple(target.joint_position_rad),
                tick.reason,
                None
                if tick.feasibility is None
                else tuple(tick.feasibility.metrics.ik_candidate_rad),
                None
                if tick.feasibility is None
                else tuple(tick.feasibility.metrics.joint_delta_rad),
                time.monotonic_ns(),
            )
        )
        self.published += 1


def _run_scenario(name: str, duration_s: float) -> dict[str, Any]:
    config = ReplayConfig.load(CONFIG)
    fake_hand = None if name in {"A", "B"} else _FakeRh56(active=name == "D")
    target_generator = SharedJakaTargetGenerator(config)
    session = SmoothQuestJakaSession(
        config,
        target_generator,
        arm_output=RecordingArmTargetAdapter(),
        normalized_hand_output=fake_hand,
    )
    metadata = _MetadataPublisher() if name in {"B", "C", "D"} else None
    tick_durations: list[int] = []
    budget_exhausted = 0
    started = time.monotonic()
    next_tick = started
    sequence = 0
    while time.monotonic() - started < duration_s:
        next_tick += 1.0 / 60.0
        remaining = next_tick - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        sequence += 1
        now_ns = time.monotonic_ns()
        session.ingest(_hand_packet(sequence, now_ns))
        if sequence == 1:
            session.ingest(_head_packet(sequence, now_ns))
        session.set_clutch_samples(
            index=AnalogClutchSample(
                1.0 if sequence > 2 else 0.0, now_ns, sequence
            ),
            grip=AnalogClutchSample(
                1.0 if name == "D" and sequence > 2 else 0.0,
                now_ns,
                sequence,
            ),
            left_controller_valid=True,
            provider="offline_benchmark",
        )
        outer_started_ns = time.perf_counter_ns()
        tick = session.control_tick(now_ns)
        if fake_hand is not None and name in {"C", "D"}:
            # C includes feedback reads without hand activation; D includes
            # the active command/contact path through submit_target above.
            fake_feedback = tuple(fake_hand.feedback)
            if len(fake_feedback) != 6:
                raise AssertionError("fake RH56 feedback shape changed")
            feedback_started_ns = time.perf_counter_ns()
            feedback_snapshot = np.asarray(fake_feedback, dtype=np.float64).copy()
            if feedback_snapshot.shape != (6,):
                raise AssertionError("fake RH56 feedback snapshot shape changed")
            session.add_control_timing(
                "rh56_feedback_duration_ns",
                time.perf_counter_ns() - feedback_started_ns,
            )
            if name == "D":
                command_started_ns = time.perf_counter_ns()
                fake_hand.submit_target(
                    tuple(float(value) for value in feedback_snapshot), now_ns
                )
                session.add_control_timing(
                    "rh56_command_duration_ns",
                    time.perf_counter_ns() - command_started_ns,
                )
        if metadata is not None:
            metadata_started_ns = time.perf_counter_ns()
            metadata.publish(tick, sequence)
            metadata_duration_ns = time.perf_counter_ns() - metadata_started_ns
        else:
            metadata_duration_ns = 0
        elapsed_ns = time.perf_counter_ns() - outer_started_ns
        session.add_control_timing(
            "episode_metadata_publish_duration_ns",
            metadata_duration_ns,
        )
        session.finalize_control_timing(elapsed_ns)
        tick_durations.append(elapsed_ns)
        budget_exhausted += int(elapsed_ns >= 20_000_000)
    report = session.control_timing_report()
    return {
        "scenario": name,
        "duration_s": duration_s,
        "control_duration_ns": _summary(tick_durations),
        "budget_exhausted_count": budget_exhausted,
        "control_timing": report,
        "metadata_queue_high_watermark": len(metadata.mailbox) if metadata else 0,
        "metadata_published": 0 if metadata is None else metadata.published,
        "fake_rh56_commands": 0 if fake_hand is None else fake_hand.command_count,
        "fake_rh56_contact_count": 0 if fake_hand is None else fake_hand.contact_count,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=3.0)
    args = parser.parse_args()
    if not 0.2 <= args.duration_sec <= 30.0:
        parser.error("--duration-sec must be between 0.2 and 30 seconds")
    result = {
        name: _run_scenario(name, args.duration_sec)
        for name in ("A", "B", "C", "D")
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
