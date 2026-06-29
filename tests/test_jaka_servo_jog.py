from __future__ import annotations

import pytest

from embodiment_core.types import JointState
from jaka_driver_adapter.palm_target_ik import safe_joint_limits_rad
from jaka_driver_adapter.servo_jog import (
    JakaPalmTargetJogController,
    JakaServoJogController,
    JointJogCommand,
    PalmTargetJogCommand,
    find_servo_safety_blockers,
    parse_joint_jog_command,
    parse_palm_target_jog_command,
)


class FakeBackend:
    def __init__(self) -> None:
        self.joints = [0.0] * 6
        self.calls: list[tuple[str, tuple[object, ...]]] = []
        self._robot = object()

    def get_joint_state(self) -> JointState:
        return JointState(names=[f"joint_{index}" for index in range(1, 7)], positions=list(self.joints))

    def call_sdk_method(self, name: str, *args: object) -> int:
        self.calls.append((name, args))
        if name == "edg_servo_j":
            self.joints = list(args[0])  # type: ignore[arg-type]
        return 0


def test_parse_joint_jog_command_requires_boolean_deadman_and_six_velocities() -> None:
    command = parse_joint_jog_command(
        '{"deadman": true, "joint_velocity_rad_s": [0, 0.1, 0, 0, 0, 0]}'
    )
    assert command.deadman is True
    assert command.joint_velocity_rad_s == [0.0, 0.1, 0.0, 0.0, 0.0, 0.0]

    with pytest.raises(ValueError, match="6 numeric"):
        parse_joint_jog_command({"deadman": True, "joint_velocity_rad_s": [0.0]})
    with pytest.raises(ValueError, match="boolean"):
        parse_joint_jog_command({"deadman": 1, "joint_velocity_rad_s": [0.0] * 6})


def test_parse_palm_target_jog_command_requires_three_velocities() -> None:
    command = parse_palm_target_jog_command(
        '{"deadman": true, "palm_velocity_m_s": [0.01, 0, -0.02], '
        '"wrist_roll_velocity_rad_s": 0.1}'
    )
    assert command.deadman is True
    assert command.palm_velocity_m_s == [0.01, 0.0, -0.02]
    assert command.wrist_roll_velocity_rad_s == 0.1
    assert command.palm_target_position_m is None

    target_command = parse_palm_target_jog_command(
        {
            "deadman": True,
            "hold_current": True,
            "palm_velocity_m_s": [0.0, 0.0, 0.0],
            "wrist_roll_velocity_rad_s": 0.0,
            "palm_target_position_m": [0.1, -0.2, 0.3],
        }
    )
    assert target_command.palm_target_position_m == [0.1, -0.2, 0.3]
    assert target_command.hold_current is True

    orientation_command = parse_palm_target_jog_command(
        {
            "deadman": True,
            "palm_velocity_m_s": [0.0, 0.0, 0.0],
            "wrist_roll_velocity_rad_s": 0.0,
            "palm_target_position_m": [0.1, -0.2, 0.3],
            "palm_target_quaternion_wxyz": [0.9238795, 0.0, 0.0, 0.3826834],
        }
    )
    assert orientation_command.palm_target_quaternion_wxyz == pytest.approx(
        [0.9238795, 0.0, 0.0, 0.3826834]
    )

    with pytest.raises(ValueError, match="3 numeric"):
        parse_palm_target_jog_command({"deadman": True, "palm_velocity_m_s": [0.0]})
    with pytest.raises(ValueError, match="hold_current"):
        parse_palm_target_jog_command(
            {
                "deadman": True,
                "hold_current": 1,
                "palm_velocity_m_s": [0.0, 0.0, 0.0],
            }
        )
    with pytest.raises(ValueError, match="palm_target_position_m"):
        parse_palm_target_jog_command(
            {
                "deadman": True,
                "palm_velocity_m_s": [0.0, 0.0, 0.0],
                "palm_target_position_m": [0.0],
            }
        )
    with pytest.raises(ValueError, match="palm_target_quaternion_wxyz"):
        parse_palm_target_jog_command(
            {
                "deadman": True,
                "palm_velocity_m_s": [0.0, 0.0, 0.0],
                "palm_target_quaternion_wxyz": [1.0],
            }
        )


def test_servo_jog_enables_clips_streams_and_times_out() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaServoJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        watchdog_sec=0.25,
        max_joint_velocity_rad_s=0.12,
        max_session_excursion_rad=0.02,
        now=lambda: clock[0],
    )
    controller.accept(JointJogCommand(deadman=True, joint_velocity_rad_s=[1.0] * 6))
    assert controller.tick() is True
    clock[0] = 1.1
    assert controller.tick() is True
    assert backend.joints == pytest.approx([0.012] * 6)

    clock[0] = 1.2
    assert controller.tick() is True
    assert backend.joints == pytest.approx([0.02] * 6)

    clock[0] = 1.3
    assert controller.tick() is False
    assert controller.enabled is False
    assert controller.last_disable_reason == "command_timeout"
    assert [name for name, _ in backend.calls].count("servo_move_enable") == 2


def test_servo_jog_limits_joint_acceleration() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaServoJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        watchdog_sec=0.25,
        max_joint_velocity_rad_s=1.0,
        max_joint_acceleration_rad_s2=0.2,
        now=lambda: clock[0],
    )
    controller.accept(JointJogCommand(deadman=True, joint_velocity_rad_s=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert controller.tick() is True
    assert backend.joints == pytest.approx([0.0] * 6)

    clock[0] = 1.1
    assert controller.tick() is True
    assert backend.joints[0] == pytest.approx(0.002)
    assert controller.status()["last_joint_velocity_rad_s"][0] == pytest.approx(0.02)

    clock[0] = 1.2
    assert controller.tick() is True
    assert backend.joints[0] == pytest.approx(0.006)
    assert controller.status()["last_joint_velocity_rad_s"][0] == pytest.approx(0.04)


def test_servo_jog_zero_session_excursion_disables_anchor_clamp() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaServoJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        watchdog_sec=0.25,
        max_joint_velocity_rad_s=1.0,
        max_joint_acceleration_rad_s2=100.0,
        max_session_excursion_rad=0.0,
        now=lambda: clock[0],
    )
    controller.accept(JointJogCommand(deadman=True, joint_velocity_rad_s=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert controller.tick() is True
    clock[0] = 1.1
    assert controller.tick() is True
    assert backend.joints[0] == pytest.approx(0.1)
    assert controller.status()["session_excursion_enabled"] is False


def test_servo_jog_passes_configured_step_num_to_edg_servo_j() -> None:
    backend = FakeBackend()
    controller = JakaServoJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        step_num=3,
    )
    controller.accept(JointJogCommand(deadman=True, joint_velocity_rad_s=[0.0] * 6))
    assert controller.tick() is True
    edg_calls = [args for name, args in backend.calls if name == "edg_servo_j"]
    assert edg_calls[-1][2] == 3


def test_servo_jog_rejects_safety_flags_before_enabling() -> None:
    backend = FakeBackend()
    controller = JakaServoJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {"is_in_estop": 1},
    )
    controller.accept(JointJogCommand(deadman=True, joint_velocity_rad_s=[0.0] * 6))
    with pytest.raises(RuntimeError, match="safety flags"):
        controller.tick()
    assert controller.enabled is False
    assert controller.fault_latched is True
    assert controller.fault_reason == "safety_blocked:robot_in_estop"
    assert controller.tick() is False
    assert find_servo_safety_blockers({"is_in_collision": 1}) == ["robot_in_collision"]
    assert find_servo_safety_blockers({"protective_stop_status": {"protective_stop": 1}}) == [
        "robot_protective_stop"
    ]


def test_servo_jog_latches_collision_during_stream_without_retrying() -> None:
    backend = FakeBackend()
    flags: dict[str, int] = {}
    controller = JakaServoJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: flags,
    )
    controller.accept(JointJogCommand(deadman=True, joint_velocity_rad_s=[0.0] * 6))
    assert controller.tick() is True
    flags["is_in_collision"] = 1
    with pytest.raises(RuntimeError, match="robot_in_collision"):
        controller.tick()
    call_count = len(backend.calls)
    assert controller.tick() is False
    assert len(backend.calls) == call_count
    assert controller.status()["fault_latched"] is True


def test_palm_target_jog_uses_ik_then_bounded_joint_servo_stream() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_palm_velocity_m_s=0.04,
        max_joint_velocity_rad_s=0.12,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    command = PalmTargetJogCommand(
        deadman=True,
        palm_velocity_m_s=[1.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
    )
    controller.accept(command)
    assert controller.tick() is True
    clock[0] = 1.1
    controller.accept(command)
    assert controller.tick() is True
    assert any(abs(value) > 0.0 for value in backend.joints)
    assert max(abs(value) for value in backend.joints) <= 0.012 + 1e-9
    assert controller.status()["mode"] == "tcp_velocity_short_horizon_ik_edg_servo_j"


def test_palm_target_jog_accepts_absolute_palm_position_target() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=1.0,
        max_joint_acceleration_rad_s2=100.0,
        max_joint_tracking_error_rad=0.0,
        max_joint_tracking_error_fault_rad=0.0,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0))
    assert controller.tick() is True
    anchor = controller.status()["palm_preview_position_m"]
    assert isinstance(anchor, list)
    command = PalmTargetJogCommand(
        deadman=True,
        palm_velocity_m_s=[0.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
        palm_target_position_m=[float(anchor[0]) + 0.01, float(anchor[1]), float(anchor[2])],
        palm_target_quaternion_wxyz=[0.999, 0.0, 0.0, 0.0447],
    )
    clock[0] = 1.1
    controller.accept(command)
    assert controller.tick() is True
    assert controller.status()["mode"] == "tcp_position_target_ik_edg_servo_j"
    assert controller.status()["palm_target_quaternion_wxyz"] == pytest.approx(
        [0.9990004545453102, 0.0, 0.0, 0.044700020338966376]
    )
    assert any(abs(value) > 0.0 for value in backend.joints)


def test_palm_target_jog_holds_small_absolute_target_updates() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=1.0,
        max_joint_acceleration_rad_s2=100.0,
        max_joint_tracking_error_rad=0.0,
        max_joint_tracking_error_fault_rad=0.0,
        target_deadband_m=0.002,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0))
    assert controller.tick() is True
    anchor = controller.status()["palm_preview_position_m"]
    assert isinstance(anchor, list)

    first_target = [float(anchor[0]) + 0.010, float(anchor[1]), float(anchor[2])]
    clock[0] = 1.1
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0, first_target))
    assert controller.tick() is True
    first_joints = list(backend.joints)

    small_target = [first_target[0] + 0.001, first_target[1], first_target[2]]
    clock[0] = 1.2
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0, small_target))
    assert controller.tick() is True

    status = controller.status()
    assert backend.joints == pytest.approx(first_joints)
    assert status["target_deadband_hold"] is True
    assert status["watchdog_reason"] == "target_deadband"
    assert status["qdot_cmd"] == pytest.approx([0.0] * 6)


def test_palm_target_jog_resynchronizes_target_to_actual_feedback_each_tick() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_palm_velocity_m_s=0.04,
        max_joint_velocity_rad_s=0.12,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    command = PalmTargetJogCommand(
        deadman=True,
        palm_velocity_m_s=[1.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
    )
    controller.accept(command)
    assert controller.tick() is True
    clock[0] = 1.1
    controller.accept(command)
    assert controller.tick() is True
    backend.joints = [0.0] * 6
    clock[0] = 1.2
    controller.accept(command)
    assert controller.tick() is True
    assert max(abs(value) for value in backend.joints) <= 0.012 + 1e-9
    assert controller.status()["feedback_closed_loop"] is True


def test_palm_target_jog_primes_after_enable_before_streaming_motion() -> None:
    backend = FakeBackend()
    backend.joints = [-1.57079632679, -0.75, 1.10, 0.0, -0.35, 0.0]
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_palm_velocity_m_s=0.04,
        max_joint_velocity_rad_s=0.12,
        prime_after_enable_ticks=2,
        now=lambda: clock[0],
    )
    command = PalmTargetJogCommand(
        deadman=True,
        palm_velocity_m_s=[1.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
    )
    initial_joints = list(backend.joints)
    controller.accept(command)
    assert controller.tick() is False
    assert backend.joints == pytest.approx(initial_joints)
    clock[0] = 1.1
    controller.accept(command)
    assert controller.tick() is False
    assert backend.joints == pytest.approx(initial_joints)
    clock[0] = 1.2
    controller.accept(command)
    assert controller.tick() is True
    assert any(abs(value) > 0.0 for value in backend.joints)
    assert controller.status()["prime_after_enable_ticks"] == 2


def test_palm_target_jog_prime_resets_ik_session_before_motion_ticks() -> None:
    backend = FakeBackend()
    backend.joints = [-1.5605687, -0.6181628, -1.4110744, 0.0775183, 0.4496217, 0.0]
    initial_joints = list(backend.joints)
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_palm_velocity_m_s=0.04,
        max_joint_velocity_rad_s=0.12,
        prime_after_enable_ticks=2,
        now=lambda: clock[0],
    )
    command = PalmTargetJogCommand(
        deadman=True,
        palm_velocity_m_s=[0.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
    )
    for step in range(5):
        clock[0] = 1.0 + 0.1 * step
        controller.accept(command)
        controller.tick()

    assert backend.joints == pytest.approx(initial_joints)
    assert controller.status()["palm_target_error_m"] == pytest.approx(0.0, abs=1e-8)


def test_palm_target_jog_zero_input_holds_previous_command_not_noisy_feedback() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=1.0,
        max_joint_acceleration_rad_s2=100.0,
        max_joint_tracking_error_rad=0.0,
        max_joint_tracking_error_fault_rad=0.0,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    command = PalmTargetJogCommand(
        deadman=True,
        palm_velocity_m_s=[0.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
    )
    controller.accept(command)
    assert controller.tick() is True
    assert backend.joints == pytest.approx([0.0] * 6)

    backend.joints = [0.08, -0.04, 0.03, 0.0, 0.0, 0.0]
    clock[0] = 1.1
    controller.accept(command)
    assert controller.tick() is True

    assert backend.joints == pytest.approx([0.0] * 6)
    assert controller.status()["q_current"] == pytest.approx([0.08, -0.04, 0.03, 0.0, 0.0, 0.0])
    assert controller.status()["q_cmd"] == pytest.approx([0.0] * 6)
    assert controller.status()["qdot_cmd"] == pytest.approx([0.0] * 6)


def test_palm_target_jog_hold_current_uses_actual_joints_without_position_ik() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=1.0,
        max_joint_acceleration_rad_s2=100.0,
        prime_after_enable_ticks=0,
        max_session_palm_excursion_m=0.001,
        max_joint_tracking_error_rad=0.0,
        max_joint_tracking_error_fault_rad=0.0,
        now=lambda: clock[0],
    )
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0))
    assert controller.tick() is True

    backend.joints = [0.08, -0.04, 0.03, 0.02, -0.01, 0.0]
    clock[0] = 1.1
    controller.accept(
        PalmTargetJogCommand(
            deadman=True,
            palm_velocity_m_s=[0.0, 0.0, 0.0],
            wrist_roll_velocity_rad_s=0.0,
            palm_target_position_m=[10.0, 10.0, 10.0],
            hold_current=True,
        )
    )
    assert controller.tick() is True

    status = controller.status()
    assert backend.joints == pytest.approx([0.08, -0.04, 0.03, 0.02, -0.01, 0.0])
    assert status["mode"] == "hold_current_edg_servo_j"
    assert status["hold_current"] is True
    assert status["q_cmd"] == pytest.approx([0.08, -0.04, 0.03, 0.02, -0.01, 0.0])
    assert status["qdot_cmd"] == pytest.approx([0.0] * 6)
    assert status["palm_target_workspace_limited"] is False
    assert status["palm_target_error_m"] == pytest.approx(0.0, abs=1e-8)


def test_palm_target_jog_hold_current_latches_once_instead_of_tracking_feedback_noise() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=1.0,
        max_joint_acceleration_rad_s2=100.0,
        max_joint_tracking_error_rad=0.0,
        max_joint_tracking_error_fault_rad=0.0,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0))
    assert controller.tick() is True

    first_hold = [0.08, -0.04, 0.03, 0.02, -0.01, 0.0]
    backend.joints = list(first_hold)
    clock[0] = 1.1
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0, hold_current=True))
    assert controller.tick() is True
    assert backend.joints == pytest.approx(first_hold)

    backend.joints = [0.12, -0.07, 0.06, 0.03, -0.02, 0.01]
    clock[0] = 1.2
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0, hold_current=True))
    assert controller.tick() is True

    status = controller.status()
    assert backend.joints == pytest.approx(first_hold)
    assert status["q_current"] == pytest.approx([0.12, -0.07, 0.06, 0.03, -0.02, 0.01])
    assert status["q_cmd"] == pytest.approx(first_hold)
    assert status["qdot_cmd"] == pytest.approx([0.0] * 6)


def test_palm_target_jog_advances_from_previous_command_under_lagging_feedback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=0.2,
        max_joint_acceleration_rad_s2=100.0,
        max_joint_tracking_error_rad=0.0,
        max_joint_tracking_error_fault_rad=0.0,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0))
    assert controller.tick() is True

    previous_command = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    controller._last_q_cmd = list(previous_command)  # noqa: SLF001
    controller.servo.target_joints = list(previous_command)
    backend.joints = [0.0] * 6

    def fake_clamp(
        raw_ik_q: list[float],
        *,
        q_current: list[float],
        q_cmd_reference: list[float],
    ) -> tuple[list[float], bool]:
        assert q_current == pytest.approx([0.0] * 6)
        assert q_cmd_reference == pytest.approx(previous_command)
        return [0.06, 0.0, 0.0, 0.0, 0.0, 0.0], False

    monkeypatch.setattr(controller, "_clamp_raw_ik_q", fake_clamp)
    clock[0] = 1.1
    controller.accept(PalmTargetJogCommand(True, [0.01, 0.0, 0.0], 0.0))
    assert controller.tick() is True

    assert backend.joints[0] == pytest.approx(0.06)
    assert controller.status()["q_cmd"][0] == pytest.approx(0.06)


def test_palm_target_jog_holds_actual_when_joint_tracking_error_exceeds_limit() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=0.2,
        max_joint_acceleration_rad_s2=100.0,
        max_joint_tracking_error_rad=0.012,
        max_joint_tracking_error_fault_rad=0.025,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0))
    assert controller.tick() is True

    controller._last_q_cmd = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]  # noqa: SLF001
    controller.servo.target_joints = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]
    backend.joints = [0.0] * 6
    clock[0] = 1.1
    controller.accept(PalmTargetJogCommand(True, [0.01, 0.0, 0.0], 0.0))
    assert controller.tick() is True

    status = controller.status()
    assert backend.joints == pytest.approx([0.0] * 6)
    assert status["watchdog_active"] is True
    assert status["watchdog_reason"] == "joint_tracking_error"
    assert status["joint_tracking_error_limited"] is True
    assert status["joint_tracking_error_indices_1_based"] == [1]
    assert status["qdot_cmd"] == pytest.approx([0.0] * 6)


def test_palm_target_jog_tracking_hold_respects_minimum_hold_time() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=0.2,
        max_joint_acceleration_rad_s2=100.0,
        max_joint_tracking_error_rad=0.012,
        joint_tracking_release_rad=0.008,
        joint_tracking_hold_min_sec=0.2,
        max_joint_tracking_error_fault_rad=0.025,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0))
    assert controller.tick() is True

    controller._last_q_cmd = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]  # noqa: SLF001
    controller.servo.target_joints = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]
    backend.joints = [0.0] * 6
    clock[0] = 1.1
    controller.accept(PalmTargetJogCommand(True, [0.01, 0.0, 0.0], 0.0))
    assert controller.tick() is True
    assert controller.status()["joint_tracking_hold_active"] is True

    clock[0] = 1.2
    controller.accept(PalmTargetJogCommand(True, [0.01, 0.0, 0.0], 0.0))
    assert controller.tick() is True
    assert controller.status()["joint_tracking_hold_active"] is True
    assert controller.status()["qdot_cmd"] == pytest.approx([0.0] * 6)

    clock[0] = 1.35
    controller.accept(PalmTargetJogCommand(True, [0.01, 0.0, 0.0], 0.0))
    assert controller.tick() is True
    assert controller.status()["joint_tracking_hold_active"] is False


def test_palm_target_jog_latches_fault_when_joint_tracking_error_is_large() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=0.2,
        max_joint_acceleration_rad_s2=100.0,
        max_joint_tracking_error_rad=0.012,
        max_joint_tracking_error_fault_rad=0.025,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    controller.accept(PalmTargetJogCommand(True, [0.0, 0.0, 0.0], 0.0))
    assert controller.tick() is True

    controller._last_q_cmd = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]  # noqa: SLF001
    controller.servo.target_joints = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    backend.joints = [0.0] * 6
    clock[0] = 1.1
    controller.accept(PalmTargetJogCommand(True, [0.01, 0.0, 0.0], 0.0))
    assert controller.tick() is False

    status = controller.status()
    assert controller.fault_latched is True
    assert status["fault_reason"] == "joint_tracking_error_fault"
    assert status["joint_tracking_error_faulted"] is True
    assert status["joint_tracking_error_rad"] == pytest.approx(0.05)


def test_palm_target_jog_clamps_raw_ik_away_from_current_and_command_q() -> None:
    backend = FakeBackend()
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_raw_ik_error_rad=0.12,
    )
    bounded, limited = controller._clamp_raw_ik_q(  # noqa: SLF001
        [1.0, -1.0, 0.4, 0.0, 0.0, 0.0],
        q_current=[0.0] * 6,
        q_cmd_reference=[0.02] * 6,
    )
    assert limited is True
    assert bounded[:3] == pytest.approx([0.12, -0.10, 0.12])


def test_palm_target_jog_saturation_watchdog_holds_current_pose() -> None:
    backend = FakeBackend()
    clock = [1.0]
    controller = JakaPalmTargetJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_palm_velocity_m_s=10.0,
        max_joint_velocity_rad_s=0.01,
        max_joint_acceleration_rad_s2=1.0,
        saturation_hold_sec=0.02,
        saturation_min_joints=1,
        prime_after_enable_ticks=0,
        now=lambda: clock[0],
    )
    command = PalmTargetJogCommand(
        deadman=True,
        palm_velocity_m_s=[10.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
    )
    controller.accept(command)
    assert controller.tick() is True
    clock[0] = 1.1
    controller.accept(command)
    controller.tick()
    status = controller.status()
    assert status["watchdog_active"] is True
    assert status["watchdog_reason"] == "saturation_hold"
    assert status["qdot_cmd"] == pytest.approx([0.0] * 6)


def test_servo_jog_clips_targets_to_documented_joint_margin() -> None:
    high = safe_joint_limits_rad()[0][1]
    backend = FakeBackend()
    backend.joints = [high - 0.001, 0.0, 0.0, 0.0, 0.0, 0.0]
    clock = [1.0]
    controller = JakaServoJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
        max_joint_velocity_rad_s=1.0,
        now=lambda: clock[0],
    )
    controller.accept(JointJogCommand(deadman=True, joint_velocity_rad_s=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert controller.tick() is True
    clock[0] = 1.1
    assert controller.tick() is True
    assert backend.joints[0] == pytest.approx(high)
    assert controller.status()["joint_limit_limited"] is True
    assert controller.status()["limited_joint_indices_1_based"] == [1]
    assert controller.status()["nearest_joint_limit_index_1_based"] == 1
    assert controller.status()["nearest_joint_limit_remaining_rad"] == pytest.approx(0.0)


def test_servo_jog_rejects_enable_when_inside_configured_joint_margin() -> None:
    high = safe_joint_limits_rad()[1][1]
    backend = FakeBackend()
    backend.joints = [0.0, high + 0.001, 0.0, 0.0, 0.0, 0.0]
    controller = JakaServoJogController(
        backend,  # type: ignore[arg-type]
        state_flags=lambda: {},
    )
    controller.accept(JointJogCommand(deadman=True, joint_velocity_rad_s=[0.0] * 6))
    with pytest.raises(RuntimeError, match="joint limit margins"):
        controller.tick()
    assert controller.fault_latched is True
    assert "joint_2_above_safe_limit" in controller.fault_reason
