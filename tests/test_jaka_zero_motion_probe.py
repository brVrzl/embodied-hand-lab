from __future__ import annotations

import json
import re
import signal
import subprocess
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "native/jaka_zero_motion_probe"
BUILD = ROOT / "build/jaka_zero_motion_probe"
BINARY = BUILD / "jaka_zero_motion_probe"
ENTRY_BINARY = BUILD / "jaka_edg_entry_exit_probe"


@pytest.fixture(scope="module", autouse=True)
def build_probe() -> None:
    subprocess.run(["cmake", "-S", str(SOURCE), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"], check=True)
    subprocess.run(["cmake", "--build", str(BUILD), "-j2"], check=True)


def run_fake(tmp_path: Path, stage: str, *arguments: str) -> tuple[subprocess.CompletedProcess[str], dict]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    result_file = tmp_path / "result.json"
    command = [str(BINARY), "--backend", "fake", "--stage", stage, "--joint-units", "radians",
               "--result-file", str(result_file)]
    if stage.startswith("run-"):
        command += ["--raw-timing-file", str(tmp_path / "raw.csv")]
    completed = subprocess.run([*command, *arguments], text=True, capture_output=True)
    return completed, json.loads(result_file.read_text())


def test_default_is_nonconnecting_and_noncommanding() -> None:
    result = subprocess.run([str(BINARY)], text=True, capture_output=True)
    payload = json.loads(result.stdout)
    assert result.returncode == 0
    assert payload == {
        "schema_version": "jaka_zero_motion_gate3b.v1",
        "stage": "dry-run",
        "connection_opened": False,
        "edg_entered": False,
        "commands_issued": 0,
    }


def test_physical_path_fails_closed_before_backend_creation() -> None:
    result = subprocess.run([
        str(BINARY), "--backend", "vendor", "--stage", "preflight",
        "--robot-ip", "192.0.2.1", "--edg-state-ip", "192.0.2.2",
        "--joint-units", "radians",
    ], text=True, capture_output=True)
    assert result.returncode == 64
    assert "confirmations" in result.stderr


def test_physical_stage_cannot_skip_prior_receipt() -> None:
    result = subprocess.run([
        str(BINARY), "--backend", "vendor", "--stage", "run-1s",
        "--physical-hardware", "--zero-motion-ack", "I_ACKNOWLEDGE_INVARIANT_JOINT_COMMAND",
        "--estop-access-confirmed", "--workspace-clear-confirmed",
        "--stage-approval", "I_APPROVE_GATE3B_STAGE_4_ONE_SECOND",
        "--robot-ip", "192.0.2.1", "--edg-state-ip", "192.0.2.2",
        "--joint-units", "radians", "--expected-tool-id", "0", "--expected-user-frame-id", "0",
        "--result-file", "/tmp/should-not-exist.json", "--raw-timing-file", "/tmp/should-not-exist.csv",
    ], text=True, capture_output=True)
    assert result.returncode == 64
    assert "prior-stage-result" in result.stderr


@pytest.mark.parametrize(("arguments", "message"), [
    (("--expected-joint-count", "7"), "joint count"),
    (("--joint-units", "degrees"), "radians"),
])
def test_configuration_and_units_rejected(arguments: tuple[str, ...], message: str) -> None:
    result = subprocess.run([
        str(BINARY), "--backend", "fake", "--stage", "preflight", "--joint-units", "radians", *arguments,
    ], text=True, capture_output=True)
    assert result.returncode == 64
    assert message in result.stderr


def test_vendor_backend_has_exact_narrow_api_surface() -> None:
    implementation = (SOURCE / "zero_motion_backend.cpp").read_text()
    calls = set(re.findall(r"client_\.(\w+)\s*\(", implementation))
    assert calls == {
        "login_in", "login_out", "get_sdk_version", "get_robot_status_simple", "get_robot_state",
        "is_in_estop", "is_in_collision", "is_in_servomove", "get_tool_id", "get_user_frame_id",
        "get_actual_joint_position", "edg_init", "edg_get_stat", "edg_servo_j",
        "servo_move_enable",
    }
    forbidden = (
        "get_robot_status(", "get_actual_tcp_position", "edg_servo_p",
        "servo_p", "joint_move", "linear_move", "circular_move", "program_run", "set_",
    )
    assert not any(token in implementation for token in forbidden)
    all_source = "\n".join(path.read_text().lower() for path in SOURCE.glob("*.*") if path.suffix in {".cpp", ".hpp"})
    assert "teledex" not in all_source
    assert "rh56" not in all_source


def test_entry_exit_binary_has_no_joint_or_cartesian_command_symbol() -> None:
    symbols = subprocess.run(
        ["nm", "-D", "--undefined-only", str(ENTRY_BINARY)], text=True, capture_output=True, check=True
    ).stdout
    assert "edg_servo_j" not in symbols
    assert "edg_servo_p" not in symbols
    assert "servo_move_enable" not in symbols
    result = subprocess.run([
        str(ENTRY_BINARY), "--backend", "fake", "--stage", "run-1s", "--joint-units", "radians",
    ], text=True, capture_output=True)
    assert result.returncode == 64
    assert "compiled without cyclic command capability" in result.stderr


def test_prior_stage_joint_delta_is_checked_before_entry(tmp_path) -> None:
    prior = tmp_path / "prior.json"
    prior.write_text(json.dumps({
        "stage": "preflight",
        "outcome": "completed",
        "physical_execution": True,
        "captured_invariant_joint_rad": [0.01, 0, 0, 0, 0, 0],
    }, indent=2))
    result_file = tmp_path / "result.json"
    result = subprocess.run([
        str(ENTRY_BINARY), "--backend", "fake", "--stage", "entry-exit", "--joint-units", "radians",
        "--prior-stage-result", str(prior), "--result-file", str(result_file),
    ], text=True, capture_output=True)
    payload = json.loads(result_file.read_text())
    assert result.returncode == 2
    assert payload["outcome"] == "prior_stage_joint_delta_exceeded"
    assert "enter_edg" not in payload["lifecycle_trace"]


def test_stage5_records_stage4_target_as_history_and_uses_fresh_capture(tmp_path) -> None:
    prior = tmp_path / "stage4.json"
    prior.write_text(json.dumps({
        "stage": "run-1s",
        "outcome": "completed",
        "physical_execution": True,
        "captured_invariant_joint_rad": [0.00005, 0, 0, 0, 0, 0],
    }))
    result, payload = run_fake(tmp_path / "run", "run-5s", "--prior-stage-result", str(prior))
    assert result.returncode == 0
    assert payload["outcome"] == "completed"
    assert payload["historical_prior_stage_target"] == pytest.approx([0.00005, 0, 0, 0, 0, 0])
    assert payload["captured_start_joint_vector"] == [0.0] * 6
    assert payload["invariant_command_target"] == payload["captured_start_joint_vector"]
    assert payload["inter_run_observation_delta"] == pytest.approx([-0.00005, 0, 0, 0, 0, 0])
    assert payload["intentional_command_delta"] == [0.0] * 6


def test_cyclic_function_contains_no_slow_status_or_dynamic_io() -> None:
    source = (SOURCE / "main.cpp").read_text()
    body = source.split("void run_cycles", 1)[1].split("void write_stats", 1)[0]
    forbidden = ("get_robot_status", "fstream", "cout", "cerr", "filesystem", "new ", "push_back", "python")
    assert not any(token in body.lower() for token in forbidden)


def test_invariant_target_and_bounded_raw_timing(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-1s")
    assert result.returncode == 0
    assert payload["cycle_count"] == 125
    assert payload["maximum_intentional_command_delta_rad"] == 0.0
    assert payload["intentional_command_delta"] == [0.0] * 6
    assert payload["captured_start_joint_vector"] == payload["invariant_command_target"]
    assert payload["final_edg_observation_available"] is True
    assert payload["encoder_drift_during_run"] == [0.0] * 6
    assert payload["completion_misses"] == 0
    assert len((tmp_path / "raw.csv").read_text().splitlines()) == 126


def test_five_second_fake_run_fits_fixed_storage(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-5s")
    assert result.returncode == 0
    assert payload["cycle_count"] == 625
    assert payload["timing"]["command_call"]["count"] == 625
    assert len((tmp_path / "raw.csv").read_text().splitlines()) == 626


def test_nonfinite_target_is_rejected_before_edg(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "entry-exit", "--fake-nonfinite-target")
    assert result.returncode == 2
    assert payload["outcome"] == "invalid_nonfinite_joint_target"
    assert "enter_edg" not in payload["lifecycle_trace"]


def test_initial_edg_delta_is_rejected_before_command(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-1s", "--fake-observed-delta-rad", "0.001")
    assert result.returncode == 2
    assert payload["outcome"] == "cross_api_observation_abort"
    assert payload["cycle_count"] == 0


def test_cross_api_observation_is_serialized_per_joint_and_warned_separately(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "entry-exit", "--fake-observed-delta-rad", "0.00006")
    assert result.returncode == 0
    assert payload["observation_warning"] is True
    assert payload["cross_api_observation_delta"] == pytest.approx([0.00006, 0, 0, 0, 0, 0])
    assert payload["intentional_command_delta"] == [0.0] * 6


def test_entry_exit_requires_servo_state_to_remain_inactive(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "entry-exit", "--fake-servo-active")
    assert result.returncode == 2
    assert payload["outcome"] == "unexpected_external_servo_move_owner"
    assert "enter_edg" not in payload["lifecycle_trace"]


def test_command_stage_owns_paired_servo_lifecycle(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-1s")
    assert result.returncode == 0
    assert payload["servo_enable_code"] == 0
    assert payload["servo_disable_code"] == 0
    assert payload["lifecycle_trace"] == (
        "login,preflight,precommand_check,enter_edg,enable_servo_move,disable_servo_move,exit_edg,logout"
    )


def test_servo_enable_failure_never_commands_and_still_exits_edg(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-1s", "--fake-servo-enable-failure")
    assert result.returncode == 2
    assert payload["outcome"] == "servo_move_enable_failure"
    assert payload["cycle_count"] == 0
    assert payload["lifecycle_trace"].endswith("enable_servo_move,exit_edg,logout")


def test_servo_disable_failure_still_exits_edg_and_logs_out(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-1s", "--fake-servo-disable-failure")
    assert result.returncode == 2
    assert payload["outcome"] == "servo_move_disable_failure"
    assert payload["lifecycle_trace"].endswith("disable_servo_move,exit_edg,logout")


def test_edg_entry_failure_still_logs_out(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "entry-exit", "--fake-entry-failure")
    assert result.returncode == 2
    assert payload["outcome"] == "edg_entry_failure"
    assert payload["lifecycle_trace"] == "login,preflight,enter_edg,logout"


def test_cyclic_read_and_command_failures_stop_without_retry(tmp_path) -> None:
    read_result, read_payload = run_fake(tmp_path / "read", "run-1s", "--fake-read-failure-cycle", "2")
    assert read_result.returncode == 2
    assert read_payload["outcome"] == "final_edg_read_failure"
    assert read_payload["sdk_failures"] == 1
    command_result, command_payload = run_fake(tmp_path / "command", "run-1s", "--fake-command-failure-cycle", "1")
    assert command_result.returncode == 2
    assert command_payload["outcome"] == "edg_command_failure"
    assert command_payload["cycle_count"] == 0


def test_completion_miss_realigns_then_hard_start_miss_is_fail_fast(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-1s", "--fake-command-delay-us", "9000")
    assert result.returncode == 2
    assert payload["outcome"] == "hard_start_period_miss"
    assert payload["completion_misses"] == 1
    assert payload["schedule_realignments"] == 1
    assert payload["hard_deadline_misses"] == 1


def test_isolated_wake_lateness_is_warning_and_realigns_without_backlog(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-1s", "--fake-single-start-lateness-us", "3600")
    assert result.returncode == 0
    assert payload["outcome"] == "completed"
    # The injected lateness guarantees at least one warning. The test runs on
    # the real host scheduler, so an additional isolated warning may coexist
    # without changing the policy under test.
    assert payload["timing_warning_events"] >= 1
    assert payload["hard_deadline_misses"] == 0
    assert payload["schedule_realignments"] >= 1
    assert payload["timing"]["start_to_start_period"]["max_ns"] > 11_000_000


def test_repeated_period_overrun_is_fail_fast(tmp_path) -> None:
    result, payload = run_fake(tmp_path, "run-1s", "--fake-start-lateness-step-us", "900")
    assert result.returncode == 2
    assert payload["outcome"] == "repeated_period_overrun"
    assert payload["period_overruns"] == 2
    assert payload["max_consecutive_period_overruns"] == 2


def test_cleanup_failures_and_order_are_reported(tmp_path) -> None:
    exit_result, exit_payload = run_fake(tmp_path / "exit", "entry-exit", "--fake-exit-failure")
    assert exit_result.returncode == 2
    assert exit_payload["outcome"] == "edg_exit_failure"
    assert exit_payload["lifecycle_trace"] == "login,preflight,enter_edg,exit_edg,logout"
    logout_result, logout_payload = run_fake(tmp_path / "logout", "entry-exit", "--fake-logout-failure")
    assert logout_result.returncode == 2
    assert logout_payload["outcome"] == "logout_failure"
    assert logout_payload["lifecycle_trace"].endswith("exit_edg,logout")


def test_ctrl_c_stops_commands_then_cleans_up_and_exits(tmp_path) -> None:
    result_file = tmp_path / "result.json"
    process = subprocess.Popen([
        str(BINARY), "--backend", "fake", "--stage", "run-5s", "--joint-units", "radians",
        "--result-file", str(result_file), "--raw-timing-file", str(tmp_path / "raw.csv"),
    ])
    time.sleep(0.15)
    process.send_signal(signal.SIGINT)
    assert process.wait(timeout=3) == 130
    payload = json.loads(result_file.read_text())
    assert payload["outcome"] == "operator_interrupted"
    assert payload["cycle_count"] < 625
    assert payload["lifecycle_trace"].endswith("exit_edg,logout")
    assert payload["post_cleanup_thread_count"] == payload["baseline_thread_count"]
