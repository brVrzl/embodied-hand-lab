from __future__ import annotations

import sys
import subprocess
import time
from types import SimpleNamespace

import pytest

from rh56_driver.pc_direct_control import (
    RH56_FAULT_RESET_APPROVAL,
    RH56_FORCE_SENSOR_CALIBRATION_APPROVAL,
    RH56_RUNTIME_CONFIG_APPROVAL,
    FakeRH56PcDirectBackend,
    HandAuthorization,
    HandControlState,
    RH56PcDirectControl,
    RH56SessionArm,
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


def _contact_stop_config() -> dict:
    config = _config()
    config["safety"] = {
        "max_close_strength": 1.0,
        "contact_stop": {
            "enabled": True,
            "require_fresh_force_before_closure_step": True,
            "force_delta_onset": [250] * 6,
            "force_delta_release": [100] * 6,
            "minimum_closing_gap": 0.015,
            "maximum_stall_progress": 0.005,
            "consecutive_samples": 2,
            "relief_margin": 0.01,
            "release_open_delta": 0.02,
            "baseline_alpha": 0.10,
        },
    }
    return config


def _opened_control(
    approval: str | RH56SessionArm = RH56SessionArm.hand_only(),
) -> tuple[RH56PcDirectControl, FakeRH56PcDirectBackend]:
    backend = FakeRH56PcDirectBackend()
    control = RH56PcDirectControl(backend, _config())
    control.open(approval)
    control.poll_feedback(1_000_000_000)
    return control, backend


def test_approval_contracts_are_distinct_and_missing_approval_opens_nothing() -> None:
    assert parse_rh56_approval(RH56_RUNTIME_CONFIG_APPROVAL) is HandAuthorization.RUNTIME_CONFIG
    assert parse_rh56_approval(RH56_FAULT_RESET_APPROVAL) is HandAuthorization.FAULT_RESET
    assert (
        parse_rh56_approval(RH56_FORCE_SENSOR_CALIBRATION_APPROVAL)
        is HandAuthorization.FORCE_SENSOR_CALIBRATION
    )

    backend = FakeRH56PcDirectBackend()
    control = RH56PcDirectControl(backend, _config())
    with pytest.raises(ValueError, match="approval"):
        control.open("")
    assert backend.connect_count == 0
    assert backend.write_count == 0


def test_session_arm_is_in_memory_hand_only_authorization() -> None:
    session_arm = RH56SessionArm.hand_only()
    control, backend = _opened_control(session_arm)

    assert control.authorization is HandAuthorization.HAND_ONLY_SESSION
    control.activate(1_000_000_000)
    assert control.command([0.05] * 6, 1_000_000_000)
    assert backend.write_count == 1

    with pytest.raises(ValueError, match="hand-only"):
        RH56SessionArm(HandAuthorization.RUNTIME_CONFIG)


def test_combined_session_arm_is_in_memory_authorization() -> None:
    session_arm = RH56SessionArm.combined()
    control, backend = _opened_control(session_arm)

    assert control.authorization is HandAuthorization.COMBINED_RUN
    control.activate(1_000_000_000)
    assert control.command([0.05] * 6, 1_000_000_000)
    assert backend.write_count == 1


def test_session_arm_cannot_authorize_special_writes() -> None:
    control, _ = _opened_control(RH56SessionArm.hand_only())

    with pytest.raises(PermissionError, match="runtime-config"):
        control.write_runtime_config([500] * 6, [200] * 6)


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


def test_session_arm_open_and_feedback_perform_no_register_writes_until_activation() -> None:
    control, backend = _opened_control(RH56SessionArm.hand_only())

    assert control.state is HandControlState.HOLD
    assert backend.write_count == 0
    record = control.episode_record(1_000_000_000)
    assert record["action"]["hand_target"] is None
    assert record["action"]["selected_hand_position_raw"] is None
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


def test_enabled_command_shaper_has_bounded_acceleration_and_no_overshoot() -> None:
    backend = FakeRH56PcDirectBackend()
    config = _config()
    config["safety"] = {"max_close_strength": 1.0}
    config["command_shaping"] = {
        "enabled": True,
        "maximum_closing_velocity": [0.35] * 6,
        "maximum_opening_velocity": [0.60] * 6,
        "maximum_acceleration": [1.40] * 6,
    }
    control = RH56PcDirectControl(backend, config)
    control.open(RH56SessionArm.hand_only())
    control.poll_feedback(1_000_000_000)
    control.activate(1_000_000_000)

    period = control.command_period_ns
    closing_positions: list[float] = []
    for step in range(24):
        timestamp = 1_000_000_000 + step * period
        control.poll_feedback(timestamp)
        assert control.command([0.8] * 6, timestamp)
        closing_positions.append(control.last_command_normalized[0])
    closing_steps = [b - a for a, b in zip(closing_positions, closing_positions[1:])]
    assert closing_positions == sorted(closing_positions)
    assert closing_positions[-1] < 0.8
    assert max(closing_steps) <= 0.05
    assert closing_steps[1] > closing_steps[0]

    opening_positions: list[float] = []
    for step in range(80):
        timestamp = 1_000_000_000 + (24 + step) * period
        control.poll_feedback(timestamp)
        control.command([0.0] * 6, timestamp)
        opening_positions.append(control.last_command_normalized[0])
    assert all(0.0 <= value <= 0.8 for value in opening_positions)
    assert opening_positions[-1] == pytest.approx(0.0)
    opening_steps = [a - b for a, b in zip(opening_positions, opening_positions[1:])]
    assert max(opening_steps) <= 0.05


def test_command_shaper_rejects_malformed_channel_vectors() -> None:
    config = _config()
    config["command_shaping"] = {
        "enabled": True,
        "maximum_acceleration": [1.4] * 5,
    }
    with pytest.raises(ValueError, match="command_shaping maximum_acceleration"):
        RH56PcDirectControl(FakeRH56PcDirectBackend(), config)


def test_contact_stop_only_pauses_once_while_measured_closure_is_progressing() -> None:
    backend = FakeRH56PcDirectBackend()
    control = RH56PcDirectControl(backend, _contact_stop_config())
    control.open(RH56SessionArm.hand_only())
    control.poll_feedback(1_000_000_000)
    control.activate(1_000_000_000)
    assert control.command([0.8, 0, 0, 0, 0, 0], 1_000_000_000)

    backend.position[0] = 930.0
    backend.load[0] = 350.0
    control.poll_feedback_register("ANGLE", 1_070_000_000)
    control.poll_feedback_register("FORCE", 1_100_000_000)

    snapshot = control.contact_stop_snapshot()
    assert snapshot["candidate_count"][0] == 1
    assert snapshot["latched"][0] is False
    assert snapshot["detection_count"] == 0


def test_contact_stop_splits_one_force_sample_budget_into_40hz_steps() -> None:
    backend = FakeRH56PcDirectBackend()
    config = _contact_stop_config()
    config["safety"]["contact_stop"].update(
        {
            "closure_budget_per_force_sample": 0.05,
            "maximum_closure_step": 0.0125,
        }
    )
    control = RH56PcDirectControl(backend, config)
    control.open(RH56SessionArm.hand_only())
    control.poll_feedback(1_000_000_000)
    control.activate(1_000_000_000)

    for step in range(4):
        assert control.command(
            [1, 0, 0, 0, 0, 0],
            1_000_000_000 + step * control.command_period_ns,
        )
        assert control.last_command_normalized[0] == pytest.approx(
            (step + 1) * 0.0125
        )
    assert not control.command(
        [1, 0, 0, 0, 0, 0],
        1_000_000_000 + 4 * control.command_period_ns,
    )
    assert control.last_command_disposition == "contact_feedback_wait"
    assert control.last_command_normalized[0] == pytest.approx(0.05)

    control.poll_feedback_register("FORCE", 1_300_000_000)
    assert control.command([1, 0, 0, 0, 0, 0], 1_300_000_000)
    assert control.last_command_normalized[0] == pytest.approx(0.0625)
    # Opening is never delayed waiting for a force sample.
    assert control.command([0, 0, 0, 0, 0, 0], 1_400_000_000)
    assert control.last_command_normalized[0] == pytest.approx(0.0125)


def test_contact_stop_rejects_one_sample_force_spike_without_latching() -> None:
    backend = FakeRH56PcDirectBackend()
    control = RH56PcDirectControl(backend, _contact_stop_config())
    control.open(RH56SessionArm.hand_only())
    control.poll_feedback(1_000_000_000)
    control.activate(1_000_000_000)
    assert control.command([0.8, 0, 0, 0, 0, 0], 1_000_000_000)

    backend.position[0] = 950.0
    control.poll_feedback_register("ANGLE", 1_040_000_000)
    control.poll_feedback_register("ANGLE", 1_070_000_000)
    backend.load[0] = 350.0
    control.poll_feedback_register("FORCE", 1_100_000_000)
    # First qualified sample pauses at measured contact but is not latched.
    snapshot = control.contact_stop_snapshot()
    assert snapshot["candidate_count"][0] == 1
    assert snapshot["latched"][0] is False
    assert control.contact_limited_target([0.8, 0, 0, 0, 0, 0], allow_release=False)[0] == pytest.approx(0.04)

    control.poll_feedback_register("ANGLE", 1_170_000_000)
    backend.load[0] = 0.0
    control.poll_feedback_register("FORCE", 1_200_000_000)
    snapshot = control.contact_stop_snapshot()
    assert snapshot["candidate_count"][0] == 0
    assert snapshot["latched"][0] is False


def test_contact_stop_latches_after_confirmed_stall_and_opening_releases() -> None:
    backend = FakeRH56PcDirectBackend()
    control = RH56PcDirectControl(backend, _contact_stop_config())
    control.open(RH56SessionArm.hand_only())
    control.poll_feedback(1_000_000_000)
    control.activate(1_000_000_000)
    assert control.command([0.8, 0, 0, 0, 0, 0], 1_000_000_000)

    backend.position[0] = 950.0
    backend.load[0] = 350.0
    control.poll_feedback_register("ANGLE", 1_040_000_000)
    for angle_ns, force_ns in (
        (1_070_000_000, 1_100_000_000),
        (1_170_000_000, 1_200_000_000),
    ):
        control.poll_feedback_register("ANGLE", angle_ns)
        control.poll_feedback_register("FORCE", force_ns)

    snapshot = control.contact_stop_snapshot()
    assert snapshot["latched"][0] is True
    assert snapshot["detection_count"] == 1
    hold = snapshot["hold_target_normalized"][0]
    assert hold == pytest.approx(0.04)
    assert control.contact_limited_target([1, 0, 0, 0, 0, 0], allow_release=False)[0] == pytest.approx(hold)

    control.hold("grip_released")
    feedback = control.poll_feedback(1_300_000_000)
    control.activate(1_300_000_000)
    snapshot = control.contact_stop_snapshot()
    assert snapshot["latched"][0] is True
    assert snapshot["force_baseline"][0] == pytest.approx(0.0)
    assert snapshot["last_activation_mode"] == "preserved_loaded_contact"
    assert control.command(
        feedback.position_normalized,
        1_300_000_000,
        force_write=True,
        measured_activation_write=True,
    )

    assert control.command(
        [0, 0, 0, 0, 0, 0],
        1_300_000_000 + control.command_period_ns,
    )
    snapshot = control.contact_stop_snapshot()
    assert snapshot["latched"][0] is False
    assert control.last_command_normalized[0] == pytest.approx(0.0)


def test_contact_candidate_discards_shaper_closing_momentum_immediately() -> None:
    backend = FakeRH56PcDirectBackend()
    config = _contact_stop_config()
    config["control_frequency_hz"] = 40
    config["command_shaping"] = {
        "enabled": True,
        "maximum_closing_velocity": [0.35] * 6,
        "maximum_opening_velocity": [0.60] * 6,
        "maximum_acceleration": [1.40] * 6,
    }
    control = RH56PcDirectControl(backend, config)
    control.open(RH56SessionArm.hand_only())
    control.poll_feedback(1_000_000_000)
    control.activate(1_000_000_000)

    period = control.command_period_ns
    for step in range(8):
        timestamp = 1_000_000_000 + step * period
        assert control.command([0.8, 0, 0, 0, 0, 0], timestamp)
        backend.position[0] = round(
            1000.0 * (1.0 - control.last_command_normalized[0])
        )
        control.poll_feedback_register("ANGLE", timestamp + period // 2)

    before_contact = control.last_command_normalized[0]
    assert control.command_shaper.velocity[0] > 0.0
    backend.load[0] = 350.0
    control.poll_feedback_register("FORCE", 1_000_000_000 + 8 * period)
    assert control.contact_stop_snapshot()["candidate_count"][0] == 1

    assert control.command(
        [0.8, 0, 0, 0, 0, 0], 1_000_000_000 + 9 * period
    )
    assert control.last_command_normalized[0] < before_contact
    assert control.command_shaper.velocity[0] <= 0.0


def test_loaded_reactivation_preserves_baseline_and_provisional_hold() -> None:
    backend = FakeRH56PcDirectBackend()
    control = RH56PcDirectControl(backend, _contact_stop_config())
    control.open(RH56SessionArm.hand_only())
    control.poll_feedback(1_000_000_000)
    control.activate(1_000_000_000)
    assert control.command([0.8, 0, 0, 0, 0, 0], 1_000_000_000)
    assert control.last_command_normalized[0] == pytest.approx(0.05)

    control.hold("grip_released")
    backend.position[0] = 950.0
    backend.load[0] = 200.0
    feedback = control.poll_feedback(1_100_000_000)
    control.activate(1_100_000_000)

    snapshot = control.contact_stop_snapshot()
    assert snapshot["last_activation_mode"] == "preserved_loaded_contact"
    assert snapshot["activation_preserved_count"] == 1
    assert snapshot["activation_rebased_count"] == 1
    assert snapshot["force_baseline"][0] == pytest.approx(0.0)
    assert snapshot["force_delta"][0] == pytest.approx(200.0)
    assert snapshot["candidate_count"][0] == 1
    assert snapshot["hold_target_normalized"][0] == pytest.approx(0.04)
    assert control.last_command_normalized == feedback.position_normalized

    assert control.command(
        feedback.position_normalized,
        1_100_000_000,
        force_write=True,
        measured_activation_write=True,
    )
    assert control.command(
        [0.8, 0, 0, 0, 0, 0],
        1_100_000_000 + control.command_period_ns,
    )
    assert control.last_command_normalized[0] == pytest.approx(0.04)

    assert control.command(
        [0, 0, 0, 0, 0, 0],
        1_100_000_000 + 2 * control.command_period_ns,
    )
    assert control.contact_stop_snapshot()["candidate_count"][0] == 0


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


def test_forced_measured_reactivation_bypasses_previous_command_window() -> None:
    control, backend = _opened_control()
    control.activate(1_000_000_000)
    assert control.command([0.2] * 6, 1_000_000_000)

    control.hold("grip_released")
    feedback = control.poll_feedback(1_001_000_000)
    control.activate(1_001_000_000)

    assert control.command(
        feedback.position_normalized,
        1_001_000_000,
        force_write=True,
        measured_activation_write=True,
    )
    assert control.last_command_disposition == "serial_write_success"
    assert backend.position_writes[-1] == [
        int(round(value)) for value in feedback.position_raw
    ]


def test_pc_direct_worker_starts_from_measured_and_hold_stops_new_writes() -> None:
    backend = FakeRH56PcDirectBackend()
    backend.position = [650.0] * 6
    config = _config()
    config["control_frequency_hz"] = 100
    control = RH56PcDirectControl(backend, config)
    worker = RH56PcDirectWorker(control)
    first = worker.start(RH56SessionArm.combined())
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


def test_pc_direct_worker_preserves_measured_activation_above_command_envelope() -> None:
    backend = FakeRH56PcDirectBackend()
    backend.position = [185.0] * 6
    control = RH56PcDirectControl(backend, _config())
    worker = RH56PcDirectWorker(control)
    first = worker.start(RH56SessionArm.combined())
    try:
        reference = worker.activate_from_measured(first.monotonic_ns)
        assert reference == pytest.approx([0.815] * 6)
        deadline = time.monotonic() + 0.3
        while not backend.position_writes and time.monotonic() < deadline:
            time.sleep(0.005)
        assert backend.position_writes[0] == [185] * 6
        diagnostics = worker.diagnostics_snapshot()
        assert diagnostics["measured_activation_target_count"] == 1
        assert diagnostics["clamped_activation_target_count"] == 0
    finally:
        worker.cleanup()


def test_combined_session_arm_is_only_a_state_machine_contract() -> None:
    control, backend = _opened_control(RH56SessionArm.combined())
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
    control.open(RH56SessionArm.hand_only())
    with pytest.raises(RuntimeError, match="nonzero"):
        control.poll_feedback(1_000_000_000)
    assert control.state is HandControlState.FAULT
    record = control.episode_record(1_000_000_000)
    assert record["hand_error"] == [0, 0, 7, 0, 0, 0]
    assert record["combined_episode_valid"] is False


def test_runtime_configuration_has_separate_authorization_and_cleanup_has_no_write() -> None:
    control, backend = _opened_control(RH56SessionArm.hand_only())
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


def test_fault_reset_and_force_calibration_have_separate_one_write_authorizations() -> None:
    reset_backend = FakeRH56PcDirectBackend()
    reset = RH56PcDirectControl(reset_backend, _config())
    reset.open(RH56_FAULT_RESET_APPROVAL)
    reset.clear_device_error()
    assert reset_backend.clear_error_write_count == 1
    assert reset_backend.force_calibration_write_count == 0
    with pytest.raises(PermissionError):
        reset.start_force_sensor_calibration()

    calibration_backend = FakeRH56PcDirectBackend()
    calibration = RH56PcDirectControl(calibration_backend, _config())
    calibration.open(RH56_FORCE_SENSOR_CALIBRATION_APPROVAL)
    calibration.start_force_sensor_calibration()
    assert calibration_backend.force_calibration_write_count == 1
    assert calibration_backend.clear_error_write_count == 0
    with pytest.raises(PermissionError):
        calibration.clear_device_error()


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
