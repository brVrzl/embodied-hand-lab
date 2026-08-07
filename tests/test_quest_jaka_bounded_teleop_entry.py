from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from quest_jaka_sim import ReplayConfig
from teleoperation.wire import StatusFlags, WorkerStatusPacket
from tools.quest_jaka_hardware import (
    COMBINED_CONTROL_REALTIME_PRIORITY,
    RECOVERABLE_CLUTCH_STAGES,
    _parser,
    _native_terminal_reason_if_ready,
    _reconcile_terminal_transport_symptom,
    _require_realtime_priority_limit,
    _apply_runtime_config,
    _apply_target_displacement_policy,
    _resolve_output_jerk_limit,
    _synchronize_paused_stopped_reference,
    _wait_status,
)


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_quest_jaka_bounded_teleop.sh"


def test_combined_realtime_limit_is_checked_before_hardware(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "tools.quest_jaka_hardware.resource.getrlimit",
        lambda _kind: (0, 0),
    )
    with pytest.raises(SystemExit, match="RLIMIT_RTPRIO >= 10"):
        _require_realtime_priority_limit(COMBINED_CONTROL_REALTIME_PRIORITY)

    monkeypatch.setattr(
        "tools.quest_jaka_hardware.resource.getrlimit",
        lambda _kind: (10, 10),
    )
    assert _require_realtime_priority_limit(
        COMBINED_CONTROL_REALTIME_PRIORITY
    ) == {
        "required_priority": 10,
        "soft_limit": 10,
        "hard_limit": 10,
    }


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def test_wait_status_requires_native_startup_reference_ready() -> None:
    q_hold = (0.2, -0.3, 0.4, -0.5, 0.6, -0.7)
    drifted = (0.2, -0.3, 0.4, -0.5, 0.6011, -0.7)
    statuses = iter(
        (
            WorkerStatusPacket(
                state=5,
                flags=int(StatusFlags.CONNECTED | StatusFlags.EDG_ACTIVE),
                last_sequence=0,
                loop_sequence=1,
                worker_monotonic_ns=1,
                command_monotonic_ns=1,
                observation_monotonic_ns=1,
                joint_position_rad=drifted,
                error_code=0,
            ),
            WorkerStatusPacket(
                state=5,
                flags=int(
                    StatusFlags.CONNECTED
                    | StatusFlags.EDG_ACTIVE
                    | StatusFlags.STARTUP_REFERENCE_READY
                ),
                last_sequence=0,
                loop_sequence=2,
                worker_monotonic_ns=2,
                command_monotonic_ns=2,
                observation_monotonic_ns=2,
                joint_position_rad=q_hold,
                error_code=0,
            ),
        )
    )

    class FakeRuntime:
        def latest_status(self):
            return next(statuses, None)

    class FakeNative:
        process = None

    status = _wait_status(FakeRuntime(), FakeNative(), timeout_s=0.1)
    assert status.joint_position_rad == q_hold
    assert StatusFlags(status.flags) & StatusFlags.STARTUP_REFERENCE_READY


def test_shell_syntax_and_help_are_offline() -> None:
    syntax = _run(["bash", "-n", str(SCRIPT)])
    assert syntax.returncode == 0, syntax.stderr
    help_result = _run([str(SCRIPT), "--help"])
    assert help_result.returncode == 0
    assert "bounded normal-speed" in help_result.stdout
    assert "not official JAKA Mini2 maximum speeds" in help_result.stdout


def test_jerk_resolution_uses_typed_config_and_project_default(tmp_path: Path) -> None:
    source = (ROOT / "configs/sim/quest_hts_jaka_mini2_live_demo.yaml").read_text()
    missing_jerk = tmp_path / "missing_jerk.yaml"
    missing_jerk.write_text(
        source.replace("  command_maximum_joint_jerk_rad_s3: 62.8318530718\n", "")
    )
    config = ReplayConfig.load(missing_jerk)
    assert _resolve_output_jerk_limit(config) == pytest.approx(
        20.0 * 3.141592653589793
    )


def test_invalid_config_jerk_is_rejected_before_network(tmp_path: Path) -> None:
    source = (ROOT / "configs/sim/quest_hts_jaka_mini2_live_demo.yaml").read_text()
    invalid = tmp_path / "invalid_jerk.yaml"
    invalid.write_text(
        source.replace(
            "  command_maximum_joint_jerk_rad_s3: 62.8318530718",
            "  command_maximum_joint_jerk_rad_s3: 1001.0",
        )
    )
    with pytest.raises(ValueError, match="output jerk shaper"):
        ReplayConfig.load(invalid)


def test_quest_input_recovery_window_cannot_exceed_ten_seconds(
    tmp_path: Path,
) -> None:
    source = (
        ROOT / "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
    ).read_text()
    invalid = tmp_path / "invalid-recovery-window.yaml"
    invalid.write_text(
        source.replace(
            "  input_recovery_timeout_ms: 10000",
            "  input_recovery_timeout_ms: 10001",
        )
    )
    with pytest.raises(ValueError, match="between 0 and 10000"):
        ReplayConfig.load(invalid)


def test_entry_parser_rejects_yaml_owned_control_overrides() -> None:
    parser = _parser()
    for option, value in (
        ("--config", "config.yaml"),
        ("--worker", "worker"),
        ("--robot-ip", "192.0.2.1"),
        ("--edg-state-ip", "192.0.2.2"),
        ("--bind", "0.0.0.0"),
        ("--port", "9000"),
        ("--rh56-device", "/dev/null"),
        ("--rh56-config", "hand.yaml"),
        ("--output-joint-jerk-limit-rad-s3", "70"),
    ):
        with pytest.raises(SystemExit):
            parser.parse_args(["bounded-normal-teleop", option, value])


@pytest.mark.parametrize(
    "removed_stage",
    (
        "p2-shadow",
        "e2-isolated",
        "p4-live",
        "post-payload-diagnostic",
        "research-thin-bounded",
    ),
)
def test_entry_parser_rejects_removed_validation_stages(removed_stage: str) -> None:
    with pytest.raises(SystemExit):
        _parser().parse_args([removed_stage])


def test_runtime_config_resolves_host_and_collection_values(tmp_path: Path) -> None:
    runtime_path = tmp_path / "physical_collection.yaml"
    runtime_path.write_text(
        """
runtime:
  config: configs/sim/quest_hts_jaka_mini2_live_demo.yaml
  worker: build/jaka_servo_worker/jaka_servo_worker
  robot_ip: 192.0.2.10
  edg_state_ip: 192.0.2.11
  bind: 0.0.0.0
  port: 9000
  duration_sec: 45
  native_control_cpu: 6
  native_control_realtime_priority: 10
  rh56_device: /dev/serial/by-id/test-rh56
  rh56_config: configs/hand/rh56_pc_direct_teleop.yaml
  run_output_joint_velocity_limits_rad_s: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
  enforce_clutch_target_displacement_limit: false
  episode_data_config: configs/data_collection/physical_collection.yaml
  episode_root: data/episodes
  task_name: test_task
  operator: "01"
""",
        encoding="utf-8",
    )
    args = _parser().parse_args(
        [
            "combined-normal-teleop",
            "--runtime-config",
            str(runtime_path),
        ]
    )
    _apply_runtime_config(args)
    assert args.robot_ip == "192.0.2.10"
    assert args.native_control_cpu == 6
    assert args.native_control_realtime_priority == 10
    assert args.duration_sec == 45
    assert args.operator == "01"
    assert args.run_output_joint_velocity_limits_rad_s == (1.5,) * 6
    assert args.enforce_clutch_target_displacement_limit is False


def test_collection_policy_disables_only_clutch_target_envelope() -> None:
    config = ReplayConfig.load(
        ROOT / "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
    )
    collection = _apply_target_displacement_policy(
        config, enabled=False
    )
    assert config.feasibility.target_displacement_limit_enabled is True
    assert collection.feasibility.target_displacement_limit_enabled is False
    assert (
        collection.feasibility.maximum_target_displacement_m
        == config.feasibility.maximum_target_displacement_m
    )
    assert collection.output_contract == config.output_contract


def test_native_acceleration_authority_comes_from_shared_contract() -> None:
    config = ReplayConfig.load(
        ROOT / "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
    )
    assert (
        "native_hard_output_joint_acceleration_rad_s2"
        not in config.raw["hardware_adapter"]
    )
    assert config.output_contract.maximum_acceleration_rad_s2 == pytest.approx(
        12.5663706144
    )


def test_normal_entry_has_no_historical_or_yaml_override_options() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    assert "bounded-normal-teleop" in source
    for option in (
        "--config",
        "--worker",
        "--robot-ip",
        "--edg-state-ip",
        "--bind",
        "--port",
        "--output-generator",
        "--no-auto-retry",
    ):
        assert option not in source
    assert "rh56-command-path-absent" in source
    assert "One exec, no retry loop." in source


def test_normal_entry_uses_native_pause_resume_reference_contract() -> None:
    class TargetGenerator:
        synchronized: list[list[float]] = []

        def synchronize_authoritative_arm_joints(self, joints: list[float]) -> None:
            self.synchronized.append(joints)

    generator = TargetGenerator()
    measured = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert "bounded-normal-teleop" in RECOVERABLE_CLUTCH_STAGES

    _synchronize_paused_stopped_reference(
        stage="bounded-normal-teleop",
        status_flags=StatusFlags.STOPPED_READY,
        target_generator=generator,
        measured_joint_position_rad=measured,
    )
    assert generator.synchronized == [list(measured)]


def test_native_fault_metrics_win_process_reap_ipc_race(tmp_path: Path) -> None:
    metrics = tmp_path / "native-metrics.json"
    metrics.write_text(
        '{"stop_classification":"native_output_jerk_hard_fault"}\n',
        encoding="utf-8",
    )
    assert _native_terminal_reason_if_ready(metrics) == (
        "native_output_jerk_hard_fault"
    )


def test_completed_native_fault_replaces_earlier_heartbeat_transport_symptom() -> None:
    reason, symptom = _reconcile_terminal_transport_symptom(
        "control_heartbeat_transport_failure",
        {
            "error_code": 1,
            "outcome": "consecutive_start_timing_misses",
            "stop_classification": "hard_timing_fault",
        },
    )
    assert reason == "hard_timing_fault"
    assert symptom == "control_heartbeat_transport_failure"


def test_transport_failure_is_not_relabelled_without_native_fault_evidence() -> None:
    reason, symptom = _reconcile_terminal_transport_symptom(
        "control_heartbeat_transport_failure",
        {
            "error_code": 0,
            "outcome": "operator_stop_command",
            "stop_classification": "normal_clutch_release",
        },
    )
    assert reason == "control_heartbeat_transport_failure"
    assert symptom is None
