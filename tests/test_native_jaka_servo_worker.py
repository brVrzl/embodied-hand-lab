from __future__ import annotations

import json
import math
import signal
import socket
import subprocess
import time
from pathlib import Path

import pytest

from teleoperation.wire import (
    FrameId,
    LatestTargetPublisher,
    TargetFlags,
    TargetKind,
    TargetPacket,
    heartbeat_target_packet,
    stop_target_packet,
)


ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "build" / "jaka_servo_worker" / "jaka_servo_worker"
P4_APPROVAL = "I_AUTHORIZE_P4_LIVE_QUEST_JAKA_TELEOPERATION"


def packet(sequence: int) -> TargetPacket:
    now = time.monotonic_ns()
    return TargetPacket(TargetKind.CARTESIAN_POSE, TargetFlags.NONE, FrameId.ROBOT_BASE,
                        sequence, now, now, now, now, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0))


def relative_packet(sequence: int, *, allow_motion: bool) -> TargetPacket:
    now = time.monotonic_ns()
    return TargetPacket(
        TargetKind.CARTESIAN_POSE,
        TargetFlags.ALLOW_MOTION if allow_motion else TargetFlags.NONE,
        FrameId.STARTUP_TCP_RELATIVE,
        sequence,
        0,
        now,
        now,
        now,
        (0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    )


def joint_packet(sequence: int, joints: tuple[float, ...], *, allow_motion: bool) -> TargetPacket:
    now = time.monotonic_ns()
    return TargetPacket(
        TargetKind.JOINT_POSITION,
        TargetFlags.ALLOW_MOTION if allow_motion else TargetFlags.NONE,
        FrameId.NONE,
        sequence,
        0,
        now,
        now,
        now,
        (*joints, 0.0, 0.0),
    )


def heartbeat_packet(sequence: int) -> TargetPacket:
    now = time.monotonic_ns()
    return heartbeat_target_packet(
        sequence=sequence,
        input_sequence=sequence,
        local_receive_ns=now,
        processing_ns=now,
        dispatch_ns=now,
        last_accepted_target_sequence=1,
        control_state_code=1,
        allow_motion=True,
    )


@pytest.fixture(scope="module", autouse=True)
def build_worker() -> None:
    subprocess.run(["cmake", "-S", str(ROOT / "native/jaka_servo_worker"),
                    "-B", str(ROOT / "build/jaka_servo_worker"), "-DCMAKE_BUILD_TYPE=Release"], check=True)
    subprocess.run(["cmake", "--build", str(ROOT / "build/jaka_servo_worker"), "-j2"], check=True)


def run_dry(tmp_path: Path, *extra: str) -> tuple[subprocess.CompletedProcess[str], dict]:
    metrics = tmp_path / "metrics.json"
    target = tmp_path / "target.sock"
    result = subprocess.run([str(WORKER), "--mode", "dry-run", "--duration-s", "0.25",
                             "--target-socket", str(target), "--metrics-file", str(metrics), *extra],
                            text=True, capture_output=True)
    return result, json.loads(metrics.read_text())


def test_native_worker_startup_shutdown_and_timing(tmp_path) -> None:
    result, metrics = run_dry(tmp_path)
    assert result.returncode == 0
    assert metrics["mode"] == "native_no_robot"
    assert metrics["requested_period_ns"] == 8_000_000
    assert metrics["maximum_intentional_command_delta_rad"] == 0.0
    assert metrics["error_code"] == 0
    assert metrics["cleanup_error_code"] == 0
    assert metrics["statistics"]["actual_cycle_period"]["count"] > 10


def test_native_worker_failure_injection_cleans_up(tmp_path) -> None:
    result, metrics = run_dry(tmp_path, "--fake-fail-after", "4")
    assert result.returncode == 2
    assert metrics["outcome"].startswith("fault: injected fake SDK failure")


def test_connected_modes_are_hardware_gated(tmp_path) -> None:
    result = subprocess.run([str(WORKER), "--mode", "zero-motion", "--robot-ip", "127.0.0.1"],
                            text=True, capture_output=True)
    assert result.returncode == 64
    assert "exact acknowledgement" in result.stderr


def test_native_p4_authorization_matches_python_hardware_entry() -> None:
    native_source = (ROOT / "native/jaka_servo_worker/main.cpp").read_text()
    hardware_entry = (ROOT / "tools/quest_jaka_hardware.py").read_text()
    assert f'kQuestMotionAck = "{P4_APPROVAL}"' in native_source
    assert f'P4_APPROVAL = "{P4_APPROVAL}"' in hardware_entry


def test_minimal_motion_requires_explicit_workspace_before_connection() -> None:
    result = subprocess.run([
        str(WORKER), "--mode", "minimal-motion", "--hardware", "--robot-ip", "192.0.2.1",
        "--acknowledgement", "I_ACKNOWLEDGE_JAKA_HARDWARE_RISK",
    ], text=True, capture_output=True)
    assert result.returncode == 64
    assert "workspace" in result.stderr


def test_process_signal_produces_metrics_and_cleanup(tmp_path) -> None:
    metrics = tmp_path / "metrics.json"
    target = tmp_path / "target.sock"
    process = subprocess.Popen([str(WORKER), "--mode", "dry-run", "--duration-s", "10",
                                "--target-socket", str(target), "--metrics-file", str(metrics)])
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    process.send_signal(signal.SIGTERM)
    assert process.wait(timeout=3) == 0
    assert json.loads(metrics.read_text())["outcome"] == "completed"


def test_latest_sequence_wins_and_disconnect_times_out(tmp_path) -> None:
    metrics = tmp_path / "metrics.json"
    target = tmp_path / "target.sock"
    process = subprocess.Popen([str(WORKER), "--mode", "dry-run", "--duration-s", "1",
                                "--fatal-timeout-ms", "300", "--controlled-stop-ms", "150",
                                "--hold-ms", "80", "--warning-ms", "20",
                                "--target-socket", str(target), "--metrics-file", str(metrics)])
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    time.sleep(0.02)
    with LatestTargetPublisher(target) as publisher:
        for sequence in (1, 2, 4, 3, 4):
            publisher.publish(packet(sequence))
    assert process.wait(timeout=3) == 0
    payload = json.loads(metrics.read_text())
    assert payload["accepted_targets"] == 1
    assert payload["rejected_targets"] == 2
    assert payload["target_age_warning_cycles"] > 0
    assert payload["loop_rate_hz"] > 0
    assert payload["accepted_target_rate_hz"] > 0
    assert payload["outcome"] == "controlled_stop_target_timeout"


def test_malformed_command_forces_controlled_exit(tmp_path) -> None:
    metrics = tmp_path / "metrics.json"
    target = tmp_path / "target.sock"
    process = subprocess.Popen([str(WORKER), "--mode", "dry-run", "--duration-s", "1",
                                "--target-socket", str(target), "--metrics-file", str(metrics)])
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sender.sendto(b"invalid", str(target))
    sender.close()
    assert process.wait(timeout=3) == 0
    payload = json.loads(metrics.read_text())
    assert payload["outcome"] == "invalid_command"
    assert payload["rejected_targets"] == 1


def run_new_mode(tmp_path: Path, mode: str, target_packet: TargetPacket) -> tuple[subprocess.CompletedProcess[str], dict]:
    metrics = tmp_path / f"{mode}.json"
    target = tmp_path / f"{mode}.sock"
    process = subprocess.Popen(
        [str(WORKER), "--mode", mode, "--duration-s", "0.25",
         "--target-socket", str(target), "--metrics-file", str(metrics)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    with LatestTargetPublisher(target) as publisher:
        assert publisher.publish(target_packet)
    stdout, stderr = process.communicate(timeout=3)
    result = subprocess.CompletedProcess(process.args, process.returncode, stdout, stderr)
    return result, json.loads(metrics.read_text())


def test_command_shadow_generates_constrained_ik_without_edg_or_commands(tmp_path) -> None:
    result, metrics = run_new_mode(
        tmp_path, "command-shadow-dry-run", relative_packet(1, allow_motion=False)
    )
    assert result.returncode == 0
    assert metrics["mode"] == "command_shadow_fake_no_edg"
    assert metrics["ik_calls"] == 1
    assert metrics["maximum_ik_joint_step_rad"] == pytest.approx(0.01)
    assert metrics["maximum_intentional_command_delta_rad"] == 0.0
    assert metrics["statistics"]["command_write_duration"]["max_ns"] == 0
    assert metrics["cleanup_error_code"] == 0


def test_bounded_teleop_fake_uses_jerk_limited_joint_commands(tmp_path) -> None:
    result, metrics = run_new_mode(
        tmp_path, "bounded-teleop-dry-run", relative_packet(1, allow_motion=True)
    )
    assert result.returncode == 0
    assert metrics["mode"] == "bounded_teleop_fake"
    assert metrics["ik_calls"] == 1
    assert metrics["maximum_intentional_command_delta_rad"] > 0.0
    assert metrics["maximum_joint_velocity_rad_s"] <= 0.03 + 1e-12
    assert metrics["maximum_joint_acceleration_rad_s2"] <= 0.15 + 1e-12
    assert metrics["maximum_joint_jerk_rad_s3"] <= 1.5 + 1e-9
    assert metrics["tracking_hard_crossings"] == 0
    assert metrics["cleanup_error_code"] == 0
    assert metrics["outcome"] == "maximum_session_duration"
    assert abs(metrics["final_joint_velocity_max_rad_s"]) <= 1e-4
    assert abs(metrics["final_joint_acceleration_max_rad_s2"]) <= 1e-3


def test_bounded_mode_rejects_target_without_motion_flag(tmp_path) -> None:
    result, metrics = run_new_mode(
        tmp_path, "bounded-teleop-dry-run", relative_packet(1, allow_motion=False)
    )
    assert result.returncode == 2
    assert "motion flag" in metrics["outcome"]
    assert metrics["maximum_intentional_command_delta_rad"] == 0.0


def test_connected_command_shadow_has_distinct_no_edg_acknowledgement() -> None:
    result = subprocess.run(
        [str(WORKER), "--mode", "command-shadow", "--hardware",
         "--robot-ip", "192.0.2.1", "--acknowledgement",
         "I_ACKNOWLEDGE_JAKA_HARDWARE_RISK"],
        text=True,
        capture_output=True,
    )
    assert result.returncode == 64
    assert "exact acknowledgement" in result.stderr


def test_quest_joint_shadow_accepts_shared_solution_without_ik_edg_or_command(tmp_path) -> None:
    result, metrics = run_new_mode(
        tmp_path,
        "joint-shadow-dry-run",
        joint_packet(1, (0.1, -0.2, 0.3, -0.4, 0.5, -0.6), allow_motion=False),
    )
    assert result.returncode == 0
    assert metrics["mode"] == "quest_joint_shadow_fake_no_edg"
    assert metrics["ik_calls"] == 0
    assert metrics["last_ik_target_rad"] == pytest.approx([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    assert metrics["maximum_intentional_command_delta_rad"] == 0.0
    assert metrics["statistics"]["command_write_duration"]["max_ns"] == 0


def test_stream_timing_starts_after_connection_setup(tmp_path) -> None:
    metrics = tmp_path / "delayed-connect.json"
    target = tmp_path / "delayed-connect.sock"
    process = subprocess.Popen(
        [
            str(WORKER),
            "--mode", "joint-shadow-dry-run",
            "--duration-s", "0.20",
            "--fake-connect-delay-us", "50000",
            "--target-socket", str(target),
            "--metrics-file", str(metrics),
        ]
    )
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    with LatestTargetPublisher(target) as publisher:
        assert publisher.publish(joint_packet(1, (0.0,) * 6, allow_motion=False))
    assert process.wait(timeout=3) == 0
    payload = json.loads(metrics.read_text())
    assert payload["outcome"] == "completed"
    assert payload["hard_timing_misses"] == 0
    assert payload["statistics"]["actual_cycle_period"]["max_ns"] < 12_000_000


def test_single_subperiod_start_delay_realigns_without_fault(tmp_path) -> None:
    metrics = tmp_path / "recoverable-start-delay.json"
    target = tmp_path / "recoverable-start-delay.sock"
    result = subprocess.run(
        [
            str(WORKER),
            "--mode", "joint-shadow-dry-run",
            "--duration-s", "0.20",
            "--fake-start-delay-once-us", "5000",
            "--target-socket", str(target),
            "--metrics-file", str(metrics),
        ],
        check=False,
    )
    payload = json.loads(metrics.read_text())
    assert result.returncode == 0
    assert payload["outcome"] == "completed"
    assert payload["hard_timing_misses"] == 0
    assert payload["timing_warning_events"] >= 1
    assert payload["schedule_realignments"] >= 1
    assert 12_000_000 < payload["statistics"]["actual_cycle_period"]["max_ns"] < 16_000_000


def test_full_period_start_delay_is_a_nonzero_hard_fault(tmp_path) -> None:
    metrics = tmp_path / "hard-start-delay.json"
    target = tmp_path / "hard-start-delay.sock"
    result = subprocess.run(
        [
            str(WORKER),
            "--mode", "joint-shadow-dry-run",
            "--duration-s", "0.20",
            "--fake-start-delay-once-us", "9000",
            "--target-socket", str(target),
            "--metrics-file", str(metrics),
        ],
        check=False,
    )
    payload = json.loads(metrics.read_text())
    assert result.returncode == 2
    assert payload["outcome"] == "hard_start_timing_miss"
    assert payload["hard_timing_misses"] == 1
    assert payload["error_code"] == 1


def test_stream_timing_rearms_after_explicit_edg_activation(tmp_path) -> None:
    metrics = tmp_path / "delayed-edg.json"
    target = tmp_path / "delayed-edg.sock"
    process = subprocess.Popen(
        [
            str(WORKER),
            "--mode", "joint-teleop-dry-run",
            "--duration-s", "0.30",
            "--fake-edg-delay-us", "50000",
            "--target-socket", str(target),
            "--metrics-file", str(metrics),
        ]
    )
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    time.sleep(0.06)
    with LatestTargetPublisher(target) as publisher:
        assert publisher.publish(joint_packet(1, (0.0,) * 6, allow_motion=True))
    assert process.wait(timeout=3) == 0
    payload = json.loads(metrics.read_text())
    assert payload["outcome"] == "command_stream_timeout"
    assert payload["accepted_targets"] == 1
    assert payload["hard_timing_misses"] == 0
    assert payload["maximum_intentional_command_delta_rad"] == 0.0
    assert payload["statistics"]["actual_cycle_period"]["max_ns"] < 12_000_000


def test_quest_joint_teleop_time_resamples_latest_target_without_ik_or_endpoint_change(tmp_path) -> None:
    metrics = tmp_path / "joint-teleop.json"
    target = tmp_path / "joint-teleop.sock"
    process = subprocess.Popen(
        [
            str(WORKER),
            "--mode", "joint-teleop-dry-run",
            "--duration-s", "0.35",
            "--warning-ms", "40",
            "--hold-ms", "120",
            "--controlled-stop-ms", "200",
            "--fatal-timeout-ms", "300",
            "--target-socket", str(target),
            "--metrics-file", str(metrics),
        ]
    )
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    time.sleep(0.02)
    with LatestTargetPublisher(target) as publisher:
        assert publisher.publish(joint_packet(1, (0.0,) * 6, allow_motion=True))
        time.sleep(0.025)
        expected = (0.04, -0.04, 0.032, -0.032, 0.024, -0.024)
        assert publisher.publish(joint_packet(2, expected, allow_motion=True))
    assert process.wait(timeout=3) == 0
    payload = json.loads(metrics.read_text())
    assert payload["mode"] == "quest_joint_teleop_fake"
    assert payload["ik_calls"] == 0
    assert payload["last_ik_target_rad"] == pytest.approx(expected)
    assert payload["maximum_intentional_command_delta_rad"] == pytest.approx(0.04)
    assert payload["maximum_joint_velocity_rad_s"] == 0.0
    assert payload["maximum_joint_acceleration_rad_s2"] == 0.0
    assert payload["maximum_joint_jerk_rad_s3"] == 0.0
    assert payload["resampler_emitted_points"] > payload["accepted_targets"]
    assert payload["resampler_destination_switches"] == 1
    assert payload["final_resampler_endpoint_error_rad"] == pytest.approx([0.0] * 6)
    assert max(payload["output_maximum_velocity_rad_s"]) <= math.pi + 1e-12
    assert payload["outcome"] == "command_stream_timeout"


def test_recoverable_hold_heartbeat_keeps_last_safe_target_live(tmp_path) -> None:
    metrics = tmp_path / "heartbeat-hold.json"
    target = tmp_path / "heartbeat-hold.sock"
    emitted = tmp_path / "heartbeat-hold.jsonl"
    process = subprocess.Popen(
        [
            str(WORKER),
            "--mode", "joint-teleop-dry-run",
            "--duration-s", "0.50",
            "--warning-ms", "40",
            "--hold-ms", "100",
            "--controlled-stop-ms", "200",
            "--fatal-timeout-ms", "300",
            "--target-socket", str(target),
            "--metrics-file", str(metrics),
            "--emitted-points-file", str(emitted),
        ]
    )
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    time.sleep(0.02)
    with LatestTargetPublisher(target) as publisher:
        assert publisher.publish(joint_packet(1, (0.0,) * 6, allow_motion=True))
        for sequence in range(2, 8):
            time.sleep(0.04)
            assert publisher.publish(heartbeat_packet(sequence))
        time.sleep(0.02)
        now = time.monotonic_ns()
        assert publisher.publish(stop_target_packet(sequence=8, monotonic_ns=now))
    assert process.wait(timeout=3) == 0
    payload = json.loads(metrics.read_text())
    assert payload["outcome"] == "operator_stop_command"
    assert payload["producer_heartbeat_packets"] == 6
    assert payload["ik_calls"] == 0
    assert payload["last_ik_target_rad"] == pytest.approx([0.0] * 6)
    assert payload["resampler_destination_switches"] == 0
    assert payload["output_maximum_adjacent_delta_rad"] == pytest.approx([0.0] * 6)
    rows = [json.loads(line) for line in emitted.read_text().splitlines()]
    assert rows
    assert all(row["joint_position_rad"] == pytest.approx([0.0] * 6) for row in rows)


def test_cycle_telemetry_uses_current_emitted_command_for_tracking(tmp_path) -> None:
    metrics = tmp_path / "metrics.json"
    telemetry = tmp_path / "cycles.jsonl"
    target = tmp_path / "target.sock"
    process = subprocess.Popen([
        str(WORKER), "--mode", "joint-teleop-dry-run", "--duration-s", "0.3",
        "--target-socket", str(target), "--metrics-file", str(metrics),
        "--cycle-telemetry-file", str(telemetry),
        "--monitor-controller-health-each-cycle",
    ])
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    time.sleep(0.02)
    with LatestTargetPublisher(target) as publisher:
        assert publisher.publish(joint_packet(1, (0.0,) * 6, allow_motion=True))
        time.sleep(0.04)
        now = time.monotonic_ns()
        assert publisher.publish(stop_target_packet(sequence=2, monotonic_ns=now))
    assert process.wait(timeout=3) == 0
    payload = json.loads(metrics.read_text())
    rows = [json.loads(line) for line in telemetry.read_text().splitlines()]
    assert rows
    assert payload["controller_health_samples"] >= len(rows)
    assert payload["controller_alarm_events"] == 0
    assert payload["joint_specific_servo_alarm_code_available"] is False
    for row in rows:
        expected = [
            math.remainder(command - measured, 2.0 * math.pi)
            for command, measured in zip(
                row["emitted_command_rad"], row["measured_joint_rad"], strict=True
            )
        ]
        assert row["emitted_minus_measured_tracking_difference_rad"] == pytest.approx(expected)


def test_diagnostic_acceleration_abort_occurs_before_fake_sdk_call(tmp_path) -> None:
    metrics = tmp_path / "metrics.json"
    emitted = tmp_path / "emitted.jsonl"
    target = tmp_path / "target.sock"
    process = subprocess.Popen([
        str(WORKER), "--mode", "joint-teleop-dry-run", "--duration-s", "0.3",
        "--target-socket", str(target), "--metrics-file", str(metrics),
        "--emitted-points-file", str(emitted),
        "--diagnostic-joint-acceleration-boundary-rad-s2", str(4.0 * math.pi),
        "--abort-on-diagnostic-acceleration-boundary",
    ])
    deadline = time.monotonic() + 2
    while not target.exists() and time.monotonic() < deadline:
        time.sleep(0.005)
    time.sleep(0.02)
    with LatestTargetPublisher(target) as publisher:
        assert publisher.publish(joint_packet(1, (0.0,) * 6, allow_motion=True))
        time.sleep(0.025)
        assert publisher.publish(joint_packet(2, (0.004,) + (0.0,) * 5, allow_motion=True))
    assert process.wait(timeout=3) == 2
    payload = json.loads(metrics.read_text())
    assert "acceleration boundary crossed before SDK call" in payload["outcome"]
    assert sum(payload["output_acceleration_boundary_rejections"]) >= 1
    rows = [json.loads(line) for line in emitted.read_text().splitlines()]
    assert rows
    assert all(row["joint_position_rad"][0] == pytest.approx(0.0) for row in rows)


def test_connected_health_monitor_uses_only_audited_sdk_calls() -> None:
    source = (ROOT / "native/jaka_servo_worker/main.cpp").read_text()
    assert "get_robot_status_simple(&status)" in source
    assert "is_in_estop(&estop)" in source
    assert "is_in_collision(&collision)" in source
    assert "get_joint_servo_alarm" not in source
    assert "class ControllerHealthMonitor" in source
    assert "cycle_health = health_monitor->snapshot()" in source
    assert "cycle_health = backend->read_controller_health()" not in source


def test_quest_joint_teleop_rejects_nonzero_startup_jump(tmp_path) -> None:
    result, metrics = run_new_mode(
        tmp_path,
        "joint-teleop-dry-run",
        joint_packet(1, (0.02, 0.0, 0.0, 0.0, 0.0, 0.0), allow_motion=True),
    )
    assert result.returncode == 2
    assert "not aligned" in metrics["outcome"]
    assert metrics["statistics"]["command_write_duration"]["max_ns"] == 0
