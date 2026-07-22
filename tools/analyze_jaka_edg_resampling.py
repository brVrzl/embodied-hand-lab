#!/usr/bin/env python3
"""Offline Candidate A/B/C comparison and fake-worker replay for EDG targets."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time

from teleoperation.wire import (
    FrameId,
    LatestTargetPublisher,
    TargetFlags,
    TargetKind,
    TargetPacket,
)


PERIOD_NS = 8_000_000
SPEED_BOUNDARY_RAD_S = math.pi
ACCELERATION_DIAGNOSTIC_RAD_S2 = 4.0 * math.pi


@dataclass(frozen=True)
class AcceptedPoint:
    sequence: int
    timestamp_ns: int
    joints: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class EmittedPoint:
    timestamp_ns: int
    joints: tuple[float, float, float, float, float, float]
    source_timestamp_ns: int
    source_sequence: int


def _load(path: Path) -> list[AcceptedPoint]:
    result: list[AcceptedPoint] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        if not row.get("accepted") or row.get("output_applied") is False:
            continue
        joints = row.get("accepted_joint_target_rad")
        timestamp = row.get("control_monotonic_ns")
        sequence = row.get("accepted_target_sequence")
        if joints is None or timestamp is None or sequence is None:
            continue
        point = AcceptedPoint(int(sequence), int(timestamp), tuple(float(v) for v in joints))
        if result and (point.sequence <= result[-1].sequence or point.timestamp_ns <= result[-1].timestamp_ns):
            raise ValueError("accepted target stream is not strictly monotonic")
        result.append(point)
    if len(result) < 2:
        raise ValueError("at least two dispatched AcceptedArmTarget records are required")
    return result


def _candidate_a(points: list[AcceptedPoint]) -> list[EmittedPoint]:
    emitted: list[EmittedPoint] = []
    index = 0
    tick = points[0].timestamp_ns
    while tick <= points[-1].timestamp_ns:
        while index + 1 < len(points) and points[index + 1].timestamp_ns <= tick:
            index += 1
        source = points[index]
        emitted.append(EmittedPoint(tick, source.joints, source.timestamp_ns, source.sequence))
        tick += PERIOD_NS
    if emitted[-1].source_sequence != points[-1].sequence:
        source = points[-1]
        emitted.append(EmittedPoint(tick, source.joints, source.timestamp_ns, source.sequence))
    return emitted


def _candidate_b(points: list[AcceptedPoint]) -> list[EmittedPoint]:
    start = points[0].timestamp_ns
    return [
        EmittedPoint(start + index * 2 * PERIOD_NS, point.joints, point.timestamp_ns, point.sequence)
        for index, point in enumerate(points)
    ]


def _candidate_c(points: list[AcceptedPoint]) -> list[EmittedPoint]:
    emitted = [EmittedPoint(points[0].timestamp_ns, points[0].joints,
                            points[0].timestamp_ns, points[0].sequence)]
    accepted_index = 0
    previous_accepted = points[0]
    start_q = points[0].joints
    destination = points[0]
    segment_start_ns = points[0].timestamp_ns
    segment_end_ns = segment_start_ns
    tick = points[0].timestamp_ns + PERIOD_NS
    while accepted_index + 1 < len(points) or emitted[-1].joints != points[-1].joints:
        newest: AcceptedPoint | None = None
        while accepted_index + 1 < len(points) and points[accepted_index + 1].timestamp_ns <= tick:
            accepted_index += 1
            newest = points[accepted_index]
        if newest is not None:
            duration_ns = newest.timestamp_ns - previous_accepted.timestamp_ns
            start_q = emitted[-1].joints
            segment_start_ns = emitted[-1].timestamp_ns
            segment_end_ns = segment_start_ns + duration_ns
            destination = newest
            previous_accepted = newest
        alpha = (
            1.0
            if segment_end_ns <= segment_start_ns
            else min(1.0, max(0.0, (tick - segment_start_ns) / (segment_end_ns - segment_start_ns)))
        )
        joints = tuple(
            start + alpha * (end - start)
            for start, end in zip(start_q, destination.joints, strict=True)
        )
        emitted.append(
            EmittedPoint(tick, joints, destination.timestamp_ns, destination.sequence)
        )
        tick += PERIOD_NS
        if len(emitted) > 100_000:
            raise RuntimeError("resampling failed to converge")
    return emitted


def _metrics(
    emitted: list[EmittedPoint],
    accepted: list[AcceptedPoint],
    *,
    controller_period_ns: int,
    interpolation_documented: bool,
) -> dict:
    maximum_delta = [0.0] * 6
    maximum_velocity = [0.0] * 6
    maximum_acceleration = [0.0] * 6
    maximum_jerk = [0.0] * 6
    speed_crossings = [0] * 6
    acceleration_crossings = [0] * 6
    velocities: list[tuple[float, ...]] = []
    accelerations: list[tuple[float, ...]] = []
    repeated = 0
    repeat_then_jump = 0
    discontinuous_switches = 0
    previous_changed = False
    for left, right in zip(emitted, emitted[1:]):
        dt_s = controller_period_ns / 1e9
        delta = tuple(b - a for a, b in zip(left.joints, right.joints, strict=True))
        changed = any(value != 0.0 for value in delta)
        if changed and right.source_sequence != left.source_sequence and not interpolation_documented:
            discontinuous_switches += 1
        repeated += int(not changed)
        repeat_then_jump += int(changed and not previous_changed)
        previous_changed = changed
        velocity = tuple(value / dt_s for value in delta)
        velocities.append(velocity)
        for joint in range(6):
            maximum_delta[joint] = max(maximum_delta[joint], abs(delta[joint]))
            maximum_velocity[joint] = max(maximum_velocity[joint], abs(velocity[joint]))
            speed_crossings[joint] += int(abs(velocity[joint]) > SPEED_BOUNDARY_RAD_S)
    for index, velocity in enumerate(velocities):
        prior = (0.0,) * 6 if index == 0 else velocities[index - 1]
        acceleration = tuple((v - p) / (controller_period_ns / 1e9)
                             for v, p in zip(velocity, prior, strict=True))
        accelerations.append(acceleration)
        for joint in range(6):
            maximum_acceleration[joint] = max(maximum_acceleration[joint], abs(acceleration[joint]))
            acceleration_crossings[joint] += int(
                abs(acceleration[joint]) > ACCELERATION_DIAGNOSTIC_RAD_S2
            )
    for index, acceleration in enumerate(accelerations):
        prior = (0.0,) * 6 if index == 0 else accelerations[index - 1]
        jerk = tuple((a - p) / (controller_period_ns / 1e9)
                     for a, p in zip(acceleration, prior, strict=True))
        for joint in range(6):
            maximum_jerk[joint] = max(maximum_jerk[joint], abs(jerk[joint]))
    endpoint_error = [
        emitted[-1].joints[joint] - accepted[-1].joints[joint] for joint in range(6)
    ]
    endpoint_latencies = [
        max(0, point.timestamp_ns - point.source_timestamp_ns) for point in emitted
        if point.joints == accepted[-1].joints
    ]
    return {
        "emitted_point_count": len(emitted),
        "point_timestamps_ns": [point.timestamp_ns for point in emitted],
        "maximum_adjacent_delta_rad": maximum_delta,
        "maximum_joint_velocity_rad_s": maximum_velocity,
        "maximum_joint_acceleration_rad_s2": maximum_acceleration,
        "maximum_joint_jerk_rad_s3": maximum_jerk,
        "source_trajectory_duration_ns": accepted[-1].timestamp_ns - accepted[0].timestamp_ns,
        "emitted_trajectory_duration_ns": emitted[-1].timestamp_ns - emitted[0].timestamp_ns,
        "duration_difference_ns": (
            emitted[-1].timestamp_ns - emitted[0].timestamp_ns
            - (accepted[-1].timestamp_ns - accepted[0].timestamp_ns)
        ),
        "final_endpoint_error_rad": endpoint_error,
        "maximum_added_endpoint_latency_ns": max(endpoint_latencies, default=0),
        "repeated_point_count": repeated,
        "repeat_then_jump_switch_count": repeat_then_jump,
        "discontinuous_target_switch_count": discontinuous_switches,
        "speed_boundary_rad_s": SPEED_BOUNDARY_RAD_S,
        "speed_boundary_crossings_per_joint": speed_crossings,
        "acceleration_diagnostic_boundary_rad_s2": ACCELERATION_DIAGNOSTIC_RAD_S2,
        "acceleration_boundary_crossings_per_joint": acceleration_crossings,
        "controller_visible_period_ns": controller_period_ns,
        "controller_interpolation_assumed": interpolation_documented,
    }


def _fake_replay(
    worker: Path,
    points: list[AcceptedPoint],
    output_dir: Path,
) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics = output_dir / "corrected_fake_worker_metrics.json"
    emitted = output_dir / "corrected_fake_worker_emitted.jsonl"
    with tempfile.TemporaryDirectory(prefix="jaka_edg_offline_replay_") as directory:
        target_socket = Path(directory) / "target.sock"
        duration_s = (points[-1].timestamp_ns - points[0].timestamp_ns) / 1e9 + 0.5
        process = subprocess.Popen(
            [
                str(worker), "--mode", "joint-teleop-dry-run",
                "--duration-s", str(duration_s),
                "--warning-ms", "80", "--hold-ms", "200",
                "--controlled-stop-ms", "300", "--fatal-timeout-ms", "500",
                "--fake-initial-joints-rad", ",".join(str(value) for value in points[0].joints),
                "--target-socket", str(target_socket),
                "--metrics-file", str(metrics),
                "--emitted-points-file", str(emitted),
                "--maximum-output-joint-velocity-rad-s", str(SPEED_BOUNDARY_RAD_S),
                "--diagnostic-joint-acceleration-boundary-rad-s2", str(ACCELERATION_DIAGNOSTIC_RAD_S2),
            ]
        )
        deadline = time.monotonic() + 2.0
        while not target_socket.exists() and time.monotonic() < deadline:
            time.sleep(0.001)
        if not target_socket.exists():
            process.terminate()
            raise RuntimeError("fake worker target socket did not appear")
        processing_base = time.monotonic_ns()
        started = time.monotonic()
        source_base = points[0].timestamp_ns
        with LatestTargetPublisher(target_socket) as publisher:
            for point in points:
                delay_s = (point.timestamp_ns - source_base) / 1e9
                while time.monotonic() - started < delay_s:
                    time.sleep(0.0002)
                processing_ns = processing_base + point.timestamp_ns - source_base
                dispatch_ns = time.monotonic_ns()
                packet = TargetPacket(
                    TargetKind.JOINT_POSITION, TargetFlags.ALLOW_MOTION, FrameId.NONE,
                    point.sequence, 0, processing_ns, processing_ns, dispatch_ns,
                    (*point.joints, 0.0, 0.0),
                )
                if not publisher.publish(packet):
                    raise RuntimeError(f"failed to publish accepted target {point.sequence}")
        return_code = process.wait(timeout=duration_s + 2.0)
    payload = json.loads(metrics.read_text(encoding="utf-8"))
    payload["process_return_code"] = return_code
    payload["emitted_points_file"] = str(emitted)
    payload["emitted_point_rows"] = sum(1 for _ in emitted.open(encoding="utf-8"))
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("accepted_log", type=Path)
    parser.add_argument("--worker", type=Path, default=Path("build/jaka_servo_worker/jaka_servo_worker"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fake-output-dir", type=Path, required=True)
    parser.add_argument(
        "--accepted-snapshot",
        type=Path,
        help="write the compact immutable target stream used by the comparison",
    )
    args = parser.parse_args()
    accepted = _load(args.accepted_log)
    if args.accepted_snapshot is not None:
        args.accepted_snapshot.parent.mkdir(parents=True, exist_ok=True)
        with args.accepted_snapshot.open("w", encoding="utf-8") as snapshot:
            for point in accepted:
                snapshot.write(json.dumps({
                    "accepted": True,
                    "output_applied": True,
                    "accepted_target_sequence": point.sequence,
                    "control_monotonic_ns": point.timestamp_ns,
                    "accepted_joint_target_rad": point.joints,
                }, sort_keys=True) + "\n")
    a = _candidate_a(accepted)
    b = _candidate_b(accepted)
    c = _candidate_c(accepted)
    report = {
        "schema_version": "jaka_edg_resampling_comparison.v1",
        "source": str(args.accepted_snapshot or args.accepted_log),
        "original_source": str(args.accepted_log),
        "accepted_target_count": len(accepted),
        "timestamp_domain": "AcceptedArmTarget.generated_monotonic_ns/CLOCK_MONOTONIC",
        "candidate_a_repeat_latest_step_num_1": _metrics(
            a, accepted, controller_period_ns=PERIOD_NS, interpolation_documented=False
        ),
        "candidate_b_direct_step_num_2": _metrics(
            b, accepted, controller_period_ns=2 * PERIOD_NS, interpolation_documented=False
        ),
        "candidate_c_continuous_resampling_step_num_1": _metrics(
            c, accepted, controller_period_ns=PERIOD_NS, interpolation_documented=True
        ),
        "selected": "candidate_c_continuous_resampling_step_num_1",
        "candidate_b_note": (
            "step_num=2 gives a nominal 16 ms motion period; no undocumented controller "
            "interpolation was assumed, so it is not selected as continuous motion"
        ),
    }
    report["corrected_native_fake_worker"] = _fake_replay(
        args.worker, accepted, args.fake_output_dir
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["corrected_native_fake_worker"]["process_return_code"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
