from __future__ import annotations

import json
import signal
import socket
import subprocess
import time
from pathlib import Path

import pytest

from teleoperation.wire import FrameId, LatestTargetPublisher, TargetFlags, TargetKind, TargetPacket


ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "build" / "jaka_servo_worker" / "jaka_servo_worker"


def packet(sequence: int) -> TargetPacket:
    now = time.monotonic_ns()
    return TargetPacket(TargetKind.CARTESIAN_POSE, TargetFlags.NONE, FrameId.ROBOT_BASE,
                        sequence, now, now, now, now, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0))


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
