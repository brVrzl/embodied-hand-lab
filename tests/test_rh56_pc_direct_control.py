from __future__ import annotations

import sys
import subprocess
import time
from types import SimpleNamespace

import pytest

from rh56_driver.pc_direct_control import (
    RH56_COMBINED_RUN_APPROVAL,
    RH56_HAND_ONLY_COMMAND_APPROVAL,
    RH56_READ_ONLY_APPROVAL,
    RH56_RUNTIME_CONFIG_APPROVAL,
    FakeRH56PcDirectBackend,
    HandAuthorization,
    HandControlState,
    RH56PcDirectControl,
    parse_rh56_approval,
    require_serial_by_id_path,
)
from rh56_driver.pc_direct_worker import RH56PcDirectWorker
from rh56_driver.serial_backend import RH56SerialBackend


def _config() -> dict:
    return {
        "control_frequency_hz": 15,
        "feedback_stale_timeout_sec": 0.4,
        "serial": {"timeout_sec": 0.2},
        "hand_schema": {
            "protocol_order": ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"],
            "hand_delta_limit": 0.05,
        },
        "safety": {"max_close_strength": 0.8},
    }


def _opened_control(approval: str = RH56_HAND_ONLY_COMMAND_APPROVAL) -> tuple[RH56PcDirectControl, FakeRH56PcDirectBackend]:
    backend = FakeRH56PcDirectBackend()
    control = RH56PcDirectControl(backend, _config())
    control.open(approval)
    control.poll_feedback(1_000_000_000)
    return control, backend


def test_approval_contracts_are_distinct_and_missing_approval_opens_nothing() -> None:
    assert parse_rh56_approval(RH56_READ_ONLY_APPROVAL) is HandAuthorization.READ_ONLY
    assert parse_rh56_approval(RH56_HAND_ONLY_COMMAND_APPROVAL) is HandAuthorization.HAND_ONLY_COMMAND
    assert parse_rh56_approval(RH56_COMBINED_RUN_APPROVAL) is HandAuthorization.COMBINED_RUN
    assert parse_rh56_approval(RH56_RUNTIME_CONFIG_APPROVAL) is HandAuthorization.RUNTIME_CONFIG

    backend = FakeRH56PcDirectBackend()
    control = RH56PcDirectControl(backend, _config())
    with pytest.raises(ValueError, match="approval"):
        control.open("")
    assert backend.connect_count == 0
    assert backend.write_count == 0


def test_serial_backend_connect_only_opens_transport_and_writes_no_register(monkeypatch: pytest.MonkeyPatch) -> None:
    serial_port = SimpleNamespace(close=lambda: None)
    serial_module = SimpleNamespace(Serial=lambda **kwargs: serial_port)
    monkeypatch.setitem(sys.modules, "serial", serial_module)
    backend = RH56SerialBackend({"serial": {"port": "/dev/serial/by-id/fake"}})
    register_writes: list[tuple[int, list[int]]] = []
    backend.write_register = lambda address, values: register_writes.append((address, values)) or True  # type: ignore[method-assign]

    assert backend.connect()
    assert backend.ser is serial_port
    assert register_writes == []


def test_read_only_open_and_feedback_perform_no_register_writes() -> None:
    control, backend = _opened_control(RH56_READ_ONLY_APPROVAL)

    assert control.state is HandControlState.HOLD
    assert backend.write_count == 0
    record = control.episode_record(1_000_000_000)
    assert record["action"]["hand_target"] is None
    assert record["action"]["selected_hand_position_raw"] is None
    with pytest.raises(PermissionError):
        control.activate(1_000_000_001)
    assert backend.write_count == 0


def test_active_commands_are_rate_and_delta_limited_in_canonical_order() -> None:
    control, backend = _opened_control()
    control.activate(1_000_000_000)

    assert control.command([0.8] * 6, 1_000_000_000)
    assert control.last_command_normalized == pytest.approx([0.05] * 6)
    assert backend.position_writes[-1] == [950] * 6
    record = control.episode_record(1_000_000_000, [0.8] * 6)
    assert record["action"]["requested_hand_target"] == [0.8] * 6
    assert record["action"]["hand_target"] == pytest.approx([0.05] * 6)
    assert record["action"]["selected_hand_position_raw"] == [950] * 6
    assert not control.command([0.8] * 6, 1_001_000_000)
    assert len(backend.position_writes) == 1
    assert control.command([0.8] * 6, 1_000_000_000 + control.command_period_ns)
    assert control.last_command_normalized == pytest.approx([0.1] * 6)
    assert backend.position_writes[-1] == [900] * 6


def test_each_activation_rebases_first_target_on_fresh_measured_angle_act() -> None:
    control, backend = _opened_control()
    control.activate(1_000_000_000)
    assert control.command([0.5] * 6, 1_000_000_000)
    control.hold("grip_released")
    backend.position = [700.0] * 6
    feedback = control.poll_feedback(1_100_000_000)
    control.activate(1_100_000_000)
    assert control.last_command_normalized == feedback.position_normalized
    assert control.command(feedback.position_normalized, 1_100_000_000)
    assert backend.position_writes[-1] == [700] * 6


def test_pc_direct_worker_starts_from_measured_and_hold_stops_new_writes() -> None:
    backend = FakeRH56PcDirectBackend()
    backend.position = [650.0] * 6
    config = _config()
    config["control_frequency_hz"] = 100
    control = RH56PcDirectControl(backend, config)
    worker = RH56PcDirectWorker(control)
    first = worker.start(RH56_COMBINED_RUN_APPROVAL)
    try:
        reference = worker.activate_from_measured(first.monotonic_ns)
        worker.submit_target(reference, first.monotonic_ns)
        deadline = time.monotonic() + 0.3
        while not backend.position_writes and time.monotonic() < deadline:
            time.sleep(0.005)
        assert backend.position_writes[0] == [650] * 6
        worker.hold("grip_released")
        time.sleep(0.03)
        writes = backend.write_count
        time.sleep(0.03)
        assert backend.write_count == writes
    finally:
        worker.cleanup()


def test_pc_direct_worker_clamps_measured_activation_to_command_envelope() -> None:
    backend = FakeRH56PcDirectBackend()
    backend.position = [185.0] * 6
    control = RH56PcDirectControl(backend, _config())
    worker = RH56PcDirectWorker(control)
    first = worker.start(RH56_COMBINED_RUN_APPROVAL)
    try:
        reference = worker.activate_from_measured(first.monotonic_ns)
        assert reference == pytest.approx([0.8] * 6)
        deadline = time.monotonic() + 0.3
        while not backend.position_writes and time.monotonic() < deadline:
            time.sleep(0.005)
        assert backend.position_writes[0] == [200] * 6
        diagnostics = worker.diagnostics_snapshot()
        assert diagnostics["measured_activation_target_count"] == 1
        assert diagnostics["clamped_activation_target_count"] == 1
    finally:
        worker.cleanup()


def test_combined_approval_is_only_a_state_machine_contract() -> None:
    control, backend = _opened_control(RH56_COMBINED_RUN_APPROVAL)
    control.activate(1_000_000_000)
    assert control.command([0.1] * 6, 1_000_000_000)
    assert backend.write_count == 1


def test_grip_stale_holds_without_writes_then_requires_reactivation() -> None:
    control, backend = _opened_control()
    control.activate(1_000_000_000)
    assert control.command([0.5] * 6, 1_000_000_000)
    writes = backend.write_count

    assert not control.command([0.5] * 6, 1_100_000_000, grip_fresh=False)
    assert control.state is HandControlState.HOLD
    assert backend.write_count == writes
    assert not control.command([0.5] * 6, 1_200_000_000)
    control.poll_feedback(1_200_000_000)
    control.activate(1_200_000_000)
    assert control.command([0.5] * 6, 1_200_000_000)

def test_feedback_timeout_disconnect_and_arm_hard_stop_fault_without_new_command() -> None:
    control, backend = _opened_control()
    control.activate(1_000_000_000)
    assert control.command([0.5] * 6, 1_000_000_000)
    writes = backend.write_count

    with pytest.raises(RuntimeError, match="stale"):
        control.command([0.5] * 6, 1_500_000_001)
    assert control.state is HandControlState.FAULT
    assert control.fault_reason == "feedback_timeout"
    assert backend.write_count == writes
    record = control.episode_record(1_500_000_001)
    assert record["combined_episode_valid"] is False
    assert record["required_arm_action"] == "safe_hold_or_stop"

    disconnected, backend2 = _opened_control()
    backend2.disconnect = True
    with pytest.raises(RuntimeError, match="disconnect"):
        disconnected.poll_feedback(1_100_000_000)
    assert disconnected.state is HandControlState.FAULT
    assert disconnected.fault_reason == "serial_feedback_failure"

    stopped, backend3 = _opened_control()
    stopped.activate(1_000_000_000)
    assert not stopped.command([0.5] * 6, 1_000_000_000, arm_terminal_stop=True)
    assert stopped.state is HandControlState.FAULT
    assert stopped.fault_reason == "arm_terminal_hard_stop"
    assert backend3.write_count == 0


def test_first_fault_reason_is_not_replaced_by_later_stop_symptoms() -> None:
    control, backend = _opened_control()
    control.activate(1_000_000_000)
    assert control.command([0.5] * 6, 1_000_000_000)
    backend.disconnect = True

    with pytest.raises(RuntimeError, match="disconnect"):
        control.poll_feedback(1_100_000_000)
    writes_at_fault = backend.write_count
    control.arm_terminal_stop("later_arm_transport_symptom")
    control.transport_fault("later_cleanup_symptom")

    assert control.fault_reason == "serial_feedback_failure"
    assert not control.command([0.6] * 6, 1_200_000_000)
    assert backend.write_count == writes_at_fault


def test_nonzero_device_error_feedback_is_a_fault_and_remains_recorded() -> None:
    backend = FakeRH56PcDirectBackend()
    backend.error[2] = 7
    control = RH56PcDirectControl(backend, _config())
    control.open(RH56_READ_ONLY_APPROVAL)
    with pytest.raises(RuntimeError, match="nonzero"):
        control.poll_feedback(1_000_000_000)
    assert control.state is HandControlState.FAULT
    record = control.episode_record(1_000_000_000)
    assert record["hand_error"] == [0, 0, 7, 0, 0, 0]
    assert record["combined_episode_valid"] is False


def test_runtime_configuration_has_separate_authorization_and_cleanup_has_no_write() -> None:
    control, backend = _opened_control(RH56_HAND_ONLY_COMMAND_APPROVAL)
    with pytest.raises(PermissionError):
        control.write_runtime_config([800] * 6, [260] * 6)
    assert backend.speed_writes == []
    assert backend.force_writes == []

    configured, config_backend = _opened_control(RH56_RUNTIME_CONFIG_APPROVAL)
    configured.write_runtime_config([800] * 6, [260] * 6)
    assert config_backend.speed_writes == [[800] * 6]
    assert config_backend.force_writes == [[260] * 6]
    writes = config_backend.write_count
    configured.cleanup()
    assert configured.state is HandControlState.DISABLED
    assert configured.transport_state == "CLOSED"
    assert config_backend.write_count == writes


def test_device_path_contract_rejects_unstable_tty_name() -> None:
    assert str(require_serial_by_id_path("/dev/serial/by-id/usb-rh56", require_exists=False)) == (
        "/dev/serial/by-id/usb-rh56"
    )
    with pytest.raises(ValueError, match="by-id"):
        require_serial_by_id_path("/dev/ttyUSB0", require_exists=False)
    assert str(require_serial_by_id_path(
        "/dev/ttyCH341USB0",
        require_exists=False,
        allow_direct_ch341=True,
    )) == "/dev/ttyCH341USB0"
    with pytest.raises(ValueError, match="CH341"):
        require_serial_by_id_path("/dev/ttyCH341USB0", require_exists=False)


def test_pc_direct_module_import_does_not_import_jaka_backend() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import rh56_driver.pc_direct_control; "
            "assert not any(name.startswith('jaka_driver_adapter') for name in sys.modules)",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
