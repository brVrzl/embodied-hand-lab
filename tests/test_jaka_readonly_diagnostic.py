from __future__ import annotations

import json
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "native/jaka_readonly_diagnostic"
BUILD = ROOT / "build/jaka_readonly_diagnostic"
BINARY = BUILD / "jaka_readonly_diagnostic"
ACK = "I_ACKNOWLEDGE_JAKA_READ_ONLY_CONNECTION"
pytestmark = pytest.mark.skipif(
    not sys.platform.startswith("linux"),
    reason="the JAKA vendor SDK diagnostic is Linux-only",
)


@pytest.fixture(scope="module", autouse=True)
def build_diagnostic() -> None:
    subprocess.run(["cmake", "-S", str(SOURCE), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"], check=True)
    subprocess.run(["cmake", "--build", str(BUILD), "-j2"], check=True)


def run_fake(tmp_path: Path, *arguments: str) -> tuple[subprocess.CompletedProcess[str], dict]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    metrics = tmp_path / "metrics.json"
    result = subprocess.run([
        str(BINARY), "--mode", "fake", "--robot-ip", "192.0.2.1",
        "--duration-s", "0.12", "--poll-hz", "50", "--slow-poll-hz", "5",
        "--metrics-file", str(metrics), *arguments,
    ], text=True, capture_output=True)
    return result, json.loads(metrics.read_text())


def test_default_fails_closed_without_address() -> None:
    result = subprocess.run([str(BINARY)], text=True, capture_output=True)
    assert result.returncode == 64
    assert "explicit IPv4" in result.stderr


def test_dry_run_validates_without_connection() -> None:
    result = subprocess.run([str(BINARY), "--robot-ip", "192.0.2.1"], text=True, capture_output=True)
    payload = json.loads(result.stdout)
    assert result.returncode == 0
    assert payload["mode"] == "dry_run"
    assert payload["connection_opened"] is False
    assert payload["read_only_calls_only"] is True


@pytest.mark.parametrize("arguments", [
    ("--robot-ip", "not-an-ip"),
    ("--robot-ip", "192.0.2.1", "--poll-hz", "126"),
    ("--robot-ip", "192.0.2.1", "--slow-poll-hz", "20", "--poll-hz", "10"),
    ("--robot-ip", "192.0.2.1", "--max-samples", "0"),
])
def test_configuration_validation(arguments: tuple[str, ...]) -> None:
    assert subprocess.run([str(BINARY), *arguments], capture_output=True).returncode == 64


def test_vendor_boundary_calls_only_approved_read_lifecycle_api() -> None:
    implementation = (SOURCE / "readonly_backend.cpp").read_text()
    calls = set(re.findall(r"client_\.(\w+)\s*\(", implementation))
    assert calls == {
        "login_in", "login_out", "get_sdk_version", "get_actual_joint_position",
        "get_actual_tcp_position", "get_robot_status_simple", "get_robot_state",
        "is_in_servomove", "is_in_estop", "is_in_collision", "get_robot_status",
        "get_tool_id", "get_tool_data", "get_user_frame_id", "get_user_frame_data",
        "get_program_state", "get_program_info",
    }
    public_boundary = (SOURCE / "readonly_backend.hpp").read_text()
    assert "JAKAZuRobot" not in public_boundary
    forbidden = (
        "edg_init", "edg_servo", "joint_move", "linear_move", "circular_move",
        "servo_move_enable", "servo_j", "servo_p", "program_run", "program_pause",
        "program_resume", "program_abort", "program_load", "set_digital_output",
        "set_analog_output", "set_tool", "set_user_frame", "set_collision",
        "enable_robot", "disable_robot", "power_on", "power_off",
    )
    assert not any(name in implementation for name in forbidden)


def test_fake_repeated_initialization_and_cleanup(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "--sessions", "3")
    assert result.returncode == 0
    assert payload["sessions_completed"] == 3
    assert payload["reconnect_attempts"] == 2
    assert payload["call_statistics"]["login_out"]["count"] == 3


def test_fake_timeout_and_disconnect_are_observable(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "--fake-disconnect-after", "2", "--max-consecutive-failures", "2")
    assert result.returncode == 2
    assert payload["outcome"] == "consecutive_read_failure"
    assert payload["failed_reads"] >= 2
    assert payload["timeouts"] >= 2
    assert payload["max_consecutive_failed_cycles"] == 2
    assert payload["call_statistics"]["login_out"]["count"] == 1


def test_fake_unreachable_and_cleanup_failure(tmp_path) -> None:
    failed_login, login_payload = run_fake(tmp_path / "login", "--fake-fail-login")
    assert failed_login.returncode == 2
    assert login_payload["outcome"].startswith("connection_failed")
    assert login_payload["connection_failures"] == 1
    assert login_payload["failed_reads"] == 0
    failed_logout, logout_payload = run_fake(tmp_path / "logout", "--fake-fail-logout")
    assert failed_logout.returncode == 2
    assert logout_payload["outcome"].startswith("disconnect_failed")


def test_bounded_storage_stops_at_capacity(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "--duration-s", "1", "--poll-hz", "125", "--max-samples", "5")
    assert result.returncode == 0
    assert payload["cycles"] == 5
    assert payload["bounded_sample_capacity"] == 5
    assert payload["statistics"]["sdk_calls_per_cycle"]["count"] == 5


def test_slow_fake_calls_report_poll_deadline_misses(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "--fake-delay-us", "15000")
    assert result.returncode == 0
    assert payload["missed_poll_deadlines"] > 0
    assert payload["max_consecutive_missed_poll_deadlines"] > 0


def test_operator_interruption_writes_metrics_after_cleanup(tmp_path) -> None:
    metrics = tmp_path / "interrupt.json"
    process = subprocess.Popen([
        str(BINARY), "--mode", "fake", "--robot-ip", "192.0.2.1", "--duration-s", "10",
        "--poll-hz", "20", "--slow-poll-hz", "2", "--metrics-file", str(metrics),
    ])
    time.sleep(0.15)
    process.send_signal(signal.SIGINT)
    assert process.wait(timeout=3) == 130
    payload = json.loads(metrics.read_text())
    assert payload["outcome"] == "operator_interrupted_cleanly"
    assert payload["call_statistics"]["login_out"]["count"] == 1
    assert payload["post_cleanup_thread_count"] == payload["baseline_thread_count"]
