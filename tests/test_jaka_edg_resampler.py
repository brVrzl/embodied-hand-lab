from __future__ import annotations

import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

from teleoperation.wire import (
    FrameId,
    LatestTargetPublisher,
    TargetFlags,
    TargetKind,
    TargetPacket,
)


ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "build" / "jaka_servo_worker" / "jaka_servo_worker"
pytestmark = pytest.mark.skipif(
    not sys.platform.startswith("linux"),
    reason="the real-time JAKA worker requires Linux scheduling and procfs",
)


@pytest.fixture(scope="module", autouse=True)
def build_worker() -> None:
    subprocess.run(
        [
            "cmake",
            "-S",
            str(ROOT / "native/jaka_servo_worker"),
            "-B",
            str(ROOT / "build/jaka_servo_worker"),
            "-DCMAKE_BUILD_TYPE=Release",
        ],
        check=True,
    )
    subprocess.run(
        ["cmake", "--build", str(ROOT / "build/jaka_servo_worker"), "-j2"],
        check=True,
    )


def _packet(
    sequence: int,
    joints: tuple[float, ...],
    processing_ns: int,
    *,
    stop: bool = False,
) -> TargetPacket:
    dispatch_ns = time.monotonic_ns()
    if stop:
        return TargetPacket(
            TargetKind.STOP,
            TargetFlags.NONE,
            FrameId.NONE,
            sequence,
            0,
            dispatch_ns,
            dispatch_ns,
            dispatch_ns,
            (0.0,) * 8,
        )
    return TargetPacket(
        TargetKind.JOINT_POSITION,
        TargetFlags.ALLOW_MOTION,
        FrameId.NONE,
        sequence,
        0,
        processing_ns,
        processing_ns,
        dispatch_ns,
        (*joints, 0.0, 0.0),
    )


def _read_points(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def _run_e1_zero_motion_fake(
    tmp_path: Path,
    *,
    initial: tuple[float, ...],
    post_edg_offset: tuple[float, ...],
    extra: tuple[str, ...] = (),
) -> tuple[subprocess.CompletedProcess[str], dict, list[dict]]:
    metrics = tmp_path / "e1-metrics.json"
    emitted = tmp_path / "e1-emitted.jsonl"
    result = subprocess.run(
        [
            str(WORKER),
            "--mode", "joint-zero-motion-dry-run",
            "--duration-s", "0.20",
            "--fake-initial-joints-rad", ",".join(map(str, initial)),
            "--fake-post-edg-joint-offset-rad", ",".join(map(str, post_edg_offset)),
            "--target-socket", str(tmp_path / "e1-target.sock"),
            "--metrics-file", str(metrics),
            "--emitted-points-file", str(emitted),
            *extra,
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    return result, json.loads(metrics.read_text()), _read_points(emitted)


def _run_stream(
    tmp_path: Path,
    samples: list[tuple[float, int, tuple[float, ...]]],
    *,
    initial: tuple[float, ...] | None = None,
    stop_after_s: float | None = None,
    extra: tuple[str, ...] = (),
) -> tuple[subprocess.CompletedProcess[str], dict, list[dict]]:
    metrics = tmp_path / "metrics.json"
    emitted = tmp_path / "emitted.jsonl"
    target = tmp_path / "target.sock"
    initial = initial or samples[0][2]
    process = subprocess.Popen(
        [
            str(WORKER),
            "--mode",
            "joint-teleop-dry-run",
            "--duration-s",
            "0.8",
            "--warning-ms",
            "80",
            "--hold-ms",
            "200",
            "--controlled-stop-ms",
            "300",
            "--fatal-timeout-ms",
            "500",
            "--fake-initial-joints-rad",
            ",".join(str(value) for value in initial),
            "--target-socket",
            str(target),
            "--metrics-file",
            str(metrics),
            "--emitted-points-file",
            str(emitted),
            *extra,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    deadline = time.monotonic() + 2.0
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert target.exists()
    maximum_offset_ns = max(offset_ns for _, offset_ns, _ in samples)
    time.sleep(maximum_offset_ns / 1e9 + 0.02)
    started = time.monotonic()
    # Keep generated times behind dispatch while remaining newer than the
    # post-EDG handoff established before the offset-dependent wait above.
    processing_base = time.monotonic_ns() - maximum_offset_ns - 10_000_000
    with LatestTargetPublisher(target) as publisher:
        for sequence, (delay_s, offset_ns, joints) in enumerate(samples, start=1):
            while time.monotonic() - started < delay_s:
                time.sleep(0.0002)
            assert publisher.publish(
                _packet(sequence, joints, processing_base + offset_ns)
            )
        if stop_after_s is not None:
            while time.monotonic() - started < stop_after_s:
                time.sleep(0.0002)
            assert publisher.publish(
                _packet(10_000, (0.0,) * 6, processing_base, stop=True)
            )
    stdout, stderr = process.communicate(timeout=3)
    result = subprocess.CompletedProcess(process.args, process.returncode, stdout, stderr)
    return result, json.loads(metrics.read_text()), _read_points(emitted)


def _stream_samples(
    intervals_ns: list[int],
    values: list[float],
) -> list[tuple[float, int, tuple[float, ...]]]:
    assert len(values) == len(intervals_ns) + 1
    offsets = [0]
    for interval in intervals_ns:
        offsets.append(offsets[-1] + interval)
    return [
        (offset / 1e9, offset, (value, -value, value / 2, -value / 2, value / 4, -value / 4))
        for offset, value in zip(offsets, values, strict=True)
    ]


def test_60_hz_targets_become_continuous_exact_8ms_grid_and_reach_endpoint(tmp_path) -> None:
    samples = _stream_samples([16_666_667] * 3, [0.0, 0.012, 0.024, 0.036])
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 0
    moving = [row for row in points if row["alpha"] > 0.0 and not row["endpoint"]]
    assert moving
    servo_times = [row["servo_time_ns"] for row in points]
    assert all(right > left for left, right in zip(servo_times, servo_times[1:]))
    intervals = [right - left for left, right in zip(servo_times[1:], servo_times[2:])]
    non_grid = sum(interval != 8_000_000 for interval in intervals)
    assert non_grid <= metrics["schedule_realignments"]
    assert all(interval > 1_000_000 for interval in intervals)  # no catch-up burst
    assert points[-1]["joint_position_rad"] == pytest.approx(samples[-1][2], abs=1e-15)
    assert metrics["final_resampler_endpoint_error_rad"] == pytest.approx([0.0] * 6, abs=1e-15)
    assert metrics["ik_calls"] == 0


def test_e1_post_edg_state_is_atomic_q_hold_with_no_startup_convergence(tmp_path) -> None:
    initial = (0.2, -0.3, 0.4, -0.5, 0.6, -0.7)
    offset = (1e-5, -2e-5, 3e-5, -4e-5, 5e-5, -6e-5)
    q_hold = tuple(left + right for left, right in zip(initial, offset, strict=True))

    result, metrics, points = _run_e1_zero_motion_fake(
        tmp_path, initial=initial, post_edg_offset=offset
    )

    assert result.returncode == 0, result.stderr
    assert metrics["pre_edg_measured_joint_position_rad"] == pytest.approx(initial)
    assert metrics["post_edg_authoritative_q_hold_rad"] == pytest.approx(q_hold)
    assert metrics["pre_to_post_edg_difference_rad"] == pytest.approx(offset)
    assert metrics["zero_motion_fixed_destination_rad"] == pytest.approx(q_hold)
    assert metrics["zero_motion_first_command_rad"] == pytest.approx(q_hold)
    assert metrics["zero_motion_last_command_rad"] == pytest.approx(q_hold)
    assert metrics["zero_motion_q_hold_initialized"] is True
    assert metrics["zero_motion_command_count"] == len(points) > 0
    assert metrics["zero_motion_command_mismatch_count"] == 0
    assert metrics["resampler_active_segment"] is False
    assert metrics["resampler_destination_switches"] == 0
    assert metrics["resampler_preemptions"] == 0
    assert metrics["final_resampler_endpoint_error_rad"] == [0.0] * 6
    assert metrics["output_maximum_adjacent_delta_rad"] == [0.0] * 6
    assert metrics["output_maximum_velocity_rad_s"] == [0.0] * 6
    assert metrics["output_maximum_acceleration_rad_s2"] == [0.0] * 6
    assert all(row["joint_position_rad"] == pytest.approx(q_hold) for row in points)
    assert metrics["ik_calls"] == 0

    source = (ROOT / "native/jaka_servo_worker/main.cpp").read_text().lower()
    e1_source = (ROOT / "tools/jaka_edg_e1_zero_motion.py").read_text().lower()
    assert "mujoco" not in source
    assert "quest" not in e1_source
    assert "command_rh56" not in e1_source


def test_e1_recoverable_lateness_realigns_without_command_burst(tmp_path) -> None:
    result, metrics, points = _run_e1_zero_motion_fake(
        tmp_path,
        initial=(0.0,) * 6,
        post_edg_offset=(1e-5,) * 6,
        extra=("--fake-start-delay-once-us", "5000"),
    )
    assert result.returncode == 0
    assert metrics["hard_timing_misses"] == 0
    assert metrics["timing_warning_events"] >= 1
    assert metrics["schedule_realignments"] >= 1
    command_times = [row["command_ns"] for row in points]
    assert all(
        right - left > 1_000_000
        for left, right in zip(command_times, command_times[1:])
    )
    assert metrics["output_maximum_adjacent_delta_rad"] == [0.0] * 6


def test_live_worker_holds_post_edg_state_until_fresh_aligned_target(tmp_path) -> None:
    initial = (0.2, -0.3, 0.4, -0.5, 0.6, -0.7)
    offset = (1e-5, -2e-5, 3e-5, -4e-5, 5e-5, -6e-5)
    q_hold = tuple(left + right for left, right in zip(initial, offset, strict=True))
    metrics = tmp_path / "live-handoff.json"
    emitted = tmp_path / "live-handoff.jsonl"
    target = tmp_path / "live-handoff.sock"
    process = subprocess.Popen(
        [
            str(WORKER),
            "--mode", "joint-teleop-dry-run",
            "--duration-s", "0.30",
            "--fake-initial-joints-rad", ",".join(map(str, initial)),
            "--fake-post-edg-joint-offset-rad", ",".join(map(str, offset)),
            "--target-socket", str(target),
            "--metrics-file", str(metrics),
            "--emitted-points-file", str(emitted),
        ]
    )
    deadline = time.monotonic() + 2.0
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.001)
    time.sleep(0.04)  # stand in for the launcher's post-EDG status handoff
    with LatestTargetPublisher(target) as publisher:
        assert publisher.publish(_packet(1, q_hold, time.monotonic_ns()))
        time.sleep(0.04)
        assert publisher.publish(_packet(2, (0.0,) * 6, time.monotonic_ns(), stop=True))
    assert process.wait(timeout=3) == 0

    payload = json.loads(metrics.read_text())
    points = _read_points(emitted)
    assert payload["pre_edg_measured_joint_position_rad"] == pytest.approx(initial)
    assert payload["post_edg_authoritative_q_hold_rad"] == pytest.approx(q_hold)
    assert payload["pre_to_post_edg_difference_rad"] == pytest.approx(offset)
    assert payload["maximum_intentional_command_delta_rad"] == 0.0
    assert payload["output_maximum_adjacent_delta_rad"] == [0.0] * 6
    assert payload["resampler_destination_switches"] == 0
    assert all(row["joint_position_rad"] == pytest.approx(q_hold) for row in points)


def test_per_joint_and_global_tracking_and_displacement_metrics_agree(tmp_path) -> None:
    samples = _stream_samples([40_000_000], [0.0, 0.02])
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 0

    tracking = metrics["maximum_tracking_difference_rad_per_joint"]
    displacement = metrics["maximum_observed_joint_delta_rad_per_joint"]
    assert metrics["maximum_measured_displacement_from_q_hold_rad_per_joint"] == displacement
    assert any(value > 0.0 for value in tracking)
    assert any(value > 0.0 for value in displacement)
    assert metrics["maximum_tracking_difference_rad"] == pytest.approx(max(tracking))
    assert metrics["maximum_observed_joint_delta_rad"] == pytest.approx(max(displacement))
    assert metrics["output_maximum_adjacent_delta_rad_global"] == pytest.approx(
        max(metrics["output_maximum_adjacent_delta_rad"])
    )
    assert metrics["output_maximum_velocity_rad_s_global"] == pytest.approx(
        max(metrics["output_maximum_velocity_rad_s"])
    )
    assert metrics["output_maximum_acceleration_rad_s2_global"] == pytest.approx(
        max(metrics["output_maximum_acceleration_rad_s2"])
    )
    assert points


def test_alternating_16ms_17ms_intervals_preserve_monotonic_continuity(tmp_path) -> None:
    samples = _stream_samples([16_000_000, 17_000_000, 16_000_000, 17_000_000],
                              [0.0, 0.01, 0.02, 0.03, 0.04])
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 0
    positions = [row["joint_position_rad"][0] for row in points]
    assert positions == sorted(positions)
    assert points[-1]["joint_position_rad"] == pytest.approx(samples[-1][2], abs=1e-15)
    assert metrics["output_speed_boundary_rejections"] == [0] * 6


def test_bursty_arrival_is_latest_only_without_backlog_replay(tmp_path) -> None:
    samples = [
        (0.0, 0, (0.0,) * 6),
        (0.025, 16_000_000, (0.008,) * 6),
        (0.025, 32_000_000, (0.016,) * 6),
        (0.025, 48_000_000, (0.024,) * 6),
    ]
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 0
    assert metrics["accepted_targets"] <= 3
    assert metrics["resampler_destination_switches"] <= 2
    assert points[-1]["joint_position_rad"] == pytest.approx((0.024,) * 6, abs=1e-15)
    assert all(row["to_sequence"] not in (2, 3) for row in points)


def test_repeated_identical_targets_emit_only_repeated_positions(tmp_path) -> None:
    samples = _stream_samples([16_666_667] * 3, [0.0, 0.0, 0.0, 0.0])
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 0
    assert all(row["joint_position_rad"] == [0.0] * 6 for row in points)
    assert metrics["resampler_repeated_points"] == metrics["resampler_emitted_points"]


def test_active_destination_replacement_starts_from_last_emit_without_backward_jump(tmp_path) -> None:
    samples = [
        (0.0, 0, (0.0,) * 6),
        (0.040, 40_000_000, (0.04,) * 6),
        (0.052, 60_000_000, (0.06,) * 6),
    ]
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 0
    positions = [row["joint_position_rad"][0] for row in points]
    assert positions == sorted(positions)
    assert metrics["resampler_preemptions"] >= 1
    assert points[-1]["joint_position_rad"] == pytest.approx((0.06,) * 6, abs=1e-15)


@pytest.mark.parametrize("reason", ["clutch_release", "tracking_loss"])
def test_disengagement_stop_cancels_active_segment_without_draining(reason, tmp_path) -> None:
    del reason  # Both faults use the same STOP packet at the adapter boundary.
    samples = [(0.0, 0, (0.0,) * 6), (0.050, 50_000_000, (0.05,) * 6)]
    result, metrics, points = _run_stream(tmp_path, samples, stop_after_s=0.058)
    assert result.returncode == 0
    assert metrics["outcome"] == "operator_stop_command"
    assert points[-1]["joint_position_rad"][0] < 0.05
    assert not points[-1]["endpoint"]


def test_reengagement_process_initializes_first_point_from_current_measured_state(tmp_path) -> None:
    measured = (0.2, -0.1, 0.3, -0.2, 0.1, -0.3)
    samples = [(0.0, 0, measured)]
    result, metrics, points = _run_stream(tmp_path, samples, initial=measured)
    assert result.returncode == 0
    assert points[0]["joint_position_rad"] == pytest.approx(measured, abs=1e-15)
    assert metrics["maximum_intentional_command_delta_rad"] == 0.0


def test_aligned_but_not_bit_exact_startup_has_jump_free_first_point_then_exact_endpoint(tmp_path) -> None:
    measured = (0.0,) * 6
    aligned = (0.0005, 0.0, 0.0, 0.0, 0.0, 0.0)
    result, metrics, points = _run_stream(
        tmp_path, [(0.0, 0, aligned)], initial=measured
    )
    assert result.returncode == 0
    assert points[0]["joint_position_rad"] == pytest.approx(measured, abs=1e-15)
    assert points[-1]["joint_position_rad"] == pytest.approx(aligned, abs=1e-15)
    assert metrics["final_resampler_endpoint_error_rad"] == pytest.approx([0.0] * 6, abs=1e-15)


def test_nonmonotonic_accepted_generation_timestamp_stops_before_new_sdk_point(tmp_path) -> None:
    samples = [
        (0.0, 20_000_000, (0.0,) * 6),
        (0.020, 10_000_000, (0.01,) * 6),
    ]
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 2
    assert "timestamps are not strictly monotonic" in metrics["outcome"]
    assert all(row["joint_position_rad"] == [0.0] * 6 for row in points)


def test_one_5ms_lateness_realigns_once_without_catchup_burst(tmp_path) -> None:
    samples = _stream_samples([40_000_000], [0.0, 0.02])
    result, metrics, points = _run_stream(
        tmp_path, samples, extra=("--fake-start-delay-once-us", "5000")
    )
    assert result.returncode == 0
    assert metrics["hard_timing_misses"] == 0
    assert metrics["timing_warning_events"] >= 1
    command_times = [row["command_ns"] for row in points]
    assert all(right - left > 1_000_000 for left, right in zip(command_times, command_times[1:]))


def test_one_9ms_lateness_and_consecutive_completion_lateness_hard_stop_nonzero(tmp_path) -> None:
    metrics = tmp_path / "late.json"
    result = subprocess.run(
        [
            str(WORKER), "--mode", "joint-shadow-dry-run", "--duration-s", "0.2",
            "--fake-start-delay-once-us", "9000", "--target-socket", str(tmp_path / "late.sock"),
            "--metrics-file", str(metrics),
        ],
        check=False,
    )
    assert result.returncode == 2
    assert json.loads(metrics.read_text())["outcome"] == "hard_start_timing_miss"

    metrics2 = tmp_path / "consecutive.json"
    result2 = subprocess.run(
        [
            str(WORKER), "--mode", "joint-shadow-dry-run", "--duration-s", "0.2",
            "--fake-read-delay-us", "9000", "--target-socket", str(tmp_path / "consecutive.sock"),
            "--metrics-file", str(metrics2),
        ],
        check=False,
    )
    assert result2.returncode == 2
    assert json.loads(metrics2.read_text())["hard_timing_misses"] >= 1


def test_fake_sdk_gets_j1_to_j6_radians_exactly_and_native_does_no_ik(tmp_path) -> None:
    final = (0.01, -0.012, 0.014, -0.016, 0.018, -0.02)
    samples = [(0.0, 0, (0.0,) * 6), (0.020, 20_000_000, final)]
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 0
    assert points[-1]["joint_position_rad"] == pytest.approx(final, abs=1e-15)
    assert metrics["ik_calls"] == 0
    source = (ROOT / "native/jaka_servo_worker/main.cpp").read_text()
    assert "edg_servo_j(&value, ABS, 1)" in source
    assert "mujoco" not in source.lower()


def test_excessive_output_velocity_is_rejected_before_fake_sdk_call(tmp_path) -> None:
    samples = [(0.0, 0, (0.0,) * 6), (0.020, 1_000_000, (0.05,) * 6)]
    result, metrics, points = _run_stream(tmp_path, samples)
    assert result.returncode == 2
    assert "internal output-feasibility contract violation before SDK call" in metrics["outcome"]
    assert sum(metrics["output_speed_boundary_rejections"]) >= 1
    assert all(row["joint_position_rad"][0] == 0.0 for row in points)


def test_per_joint_output_velocity_boundary_rejects_before_fake_sdk_call(
    tmp_path: Path,
) -> None:
    result, metrics, points = _run_stream(
        tmp_path,
        [
            (0.0, 0, (0.0,) * 6),
            (0.025, 20_000_000, (0.0, 0.0, 0.0, 0.026, 0.0, 0.0)),
        ],
        extra=(
            "--maximum-output-joint-velocity-rad-s-per-joint",
            "1.5,1.5,1.5,1.2,1.2,1.2",
        ),
    )
    assert result.returncode == 2
    assert "internal output-feasibility contract violation" in metrics["outcome"]
    assert metrics["output_joint_velocity_boundary_rad_s_per_joint"] == pytest.approx(
        [1.5, 1.5, 1.5, 1.2, 1.2, 1.2]
    )
    assert metrics["output_speed_boundary_rejections"][3] == 1
    assert all(row["joint_position_rad"][3] < 0.026 for row in points)


def test_recoverable_velocity_crossing_is_limited_progress_not_output_hold(
    tmp_path: Path,
) -> None:
    result, metrics, points = _run_stream(
        tmp_path,
        [
            (0.0, 0, (0.0,) * 6),
            (0.020, 20_000_000, (0.03, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ],
        extra=("--recover-output-acceleration-transition",),
    )
    assert result.returncode == 0, result.stderr
    assert metrics["output_speed_boundary_rejections"] == [0] * 6
    assert metrics["resampler_transition_limited_points"] >= 1
    assert metrics["transition_limited_progress_points"] == metrics[
        "resampler_transition_limited_points"
    ]
    assert metrics["true_output_hold_count"] == 0
    assert metrics["recoverable_output_acceleration_hold_count"] == 0
    assert metrics["output_joint_jerk_tolerance_absolute_rad_s3"] == pytest.approx(
        1e-6
    )
    assert metrics["output_joint_jerk_tolerance_relative"] == pytest.approx(
        2.5e-7
    )
    assert metrics["output_joint_jerk_hard_boundary_with_tolerance_rad_s3"] > (
        metrics["output_joint_jerk_hard_boundary_rad_s3"]
    )
    assert all(
        abs(point["joint_velocity_rad_s"][0]) <= math.pi + 1e-9
        for point in points
    )


def test_final_jerk_boundary_accepts_only_numeric_comparison_envelope(
    tmp_path: Path,
) -> None:
    source = tmp_path / "jerk_boundary_probe.cpp"
    binary = tmp_path / "jerk_boundary_probe"
    source.write_text(
        """
#include \"joint_servo_resampler.hpp\"
#include <cassert>

int main() {
  constexpr double limit = 62.8318530718;
  assert(jaka_servo::output_jerk_within_hard_boundary(limit, limit));
  assert(jaka_servo::output_jerk_within_hard_boundary(limit + 1e-6, limit));
  assert(jaka_servo::output_jerk_within_hard_boundary(limit + 1e-5, limit));
  assert(!jaka_servo::output_jerk_within_hard_boundary(limit + 1e-3, limit));
  assert(!jaka_servo::output_jerk_within_hard_boundary(-limit - 1e-3, limit));
  return 0;
}
""",
        encoding="utf-8",
    )
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "native/jaka_servo_worker"),
            str(source),
            "-o",
            str(binary),
        ],
        check=True,
    )
    subprocess.run([str(binary)], check=True)


def test_nonfinite_output_is_rejected_before_fake_sdk_call(tmp_path) -> None:
    metrics = tmp_path / "nan-metrics.json"
    emitted = tmp_path / "nan-emitted.jsonl"
    target = tmp_path / "nan.sock"
    process = subprocess.Popen(
        [
            str(WORKER), "--mode", "joint-teleop-dry-run", "--duration-s", "0.3",
            "--target-socket", str(target), "--metrics-file", str(metrics),
            "--emitted-points-file", str(emitted),
        ]
    )
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.001)
    now = time.monotonic_ns()
    malformed = _packet(1, (math.nan, 0.0, 0.0, 0.0, 0.0, 0.0), now)
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    from teleoperation.wire import encode_target
    sender.sendto(encode_target(malformed), str(target))
    sender.close()
    assert process.wait(timeout=3) == 0
    payload = json.loads(metrics.read_text())
    assert payload["outcome"] == "invalid_command"
    assert payload["resampler_emitted_points"] == 0
    assert _read_points(emitted) == []


def test_e1_and_e2_require_their_new_exact_separate_authorizations(tmp_path) -> None:
    e1 = subprocess.run(
        [
            str(ROOT / ".venv/bin/python"), str(ROOT / "tools/jaka_edg_e1_zero_motion.py"),
            "--robot-ip", "192.0.2.1", "--approval", "WRONG",
            "--metrics", str(tmp_path / "e1.json"),
        ],
        text=True,
        capture_output=True,
    )
    assert e1.returncode != 0
    assert "I_AUTHORIZE_E1_ZERO_MOTION_EDG_RESAMPLER" in e1.stderr

    e2 = subprocess.run(
        [
            str(ROOT / ".venv/bin/python"), str(ROOT / "tools/quest_jaka_hardware.py"),
            "e2-isolated", "--robot-ip", "192.0.2.1", "--approval", "WRONG",
            "--log", str(tmp_path / "e2.jsonl"), "--summary", str(tmp_path / "e2-summary.json"),
            "--metrics", str(tmp_path / "e2-metrics.json"), "--capture", str(tmp_path / "e2-capture.jsonl"),
        ],
        text=True,
        capture_output=True,
    )
    assert e2.returncode != 0
    assert "I_AUTHORIZE_E2_ONE_SMALL_TCP_TRANSLATION" in e2.stderr
