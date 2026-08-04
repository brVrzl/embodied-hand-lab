from __future__ import annotations

import sys

import pytest

from rh56_driver.pc_direct_control import (
    HandOperation,
    require_serial_by_id_path as real_require_serial_by_id_path,
)
import tools.quest_rh56_hand_test as hand_entry
from tools.quest_rh56_hand_test import (
    _build_parser,
    _load_hand_only_quest_config,
    _write_summary,
    validate_gate,
)


DEVICE = "/dev/serial/by-id/usb-Inspire_RH56DFX-test"


@pytest.fixture(autouse=True)
def _offline_by_id_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        hand_entry,
        "require_serial_by_id_path",
        lambda device, **kwargs: device,
    )


def _parse(*arguments: str):
    return _build_parser().parse_args(["--real", "--device", DEVICE, *arguments])


def test_preflight_is_zero_io_and_does_not_construct_a_backend() -> None:
    args = _parse("--preflight-only")
    assert validate_gate(args) is None


def test_default_entry_is_dry_run_without_a_device_or_backend(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        ["quest_rh56_hand_test.py", "--summary", str(tmp_path / "dry-run.json")],
    )
    monkeypatch.setattr(
        hand_entry,
        "inspect_serial_device",
        lambda *args, **kwargs: pytest.fail("dry-run inspected a hardware device"),
    )
    hand_entry.main()
    assert '"real_hardware": false' in capsys.readouterr().out


def test_preflight_main_does_not_construct_or_open_backend(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "quest_rh56_hand_test.py",
            "--real",
            "--device",
            DEVICE,
            "--preflight-only",
        ],
    )
    monkeypatch.setattr(
        hand_entry,
        "inspect_serial_device",
        lambda device, **kwargs: {"requested_by_id": device, "resolved_tty": "/dev/ttyUSB-test"},
    )
    monkeypatch.setattr(
        hand_entry,
        "RH56SerialBackend",
        lambda config: pytest.fail("preflight constructed a serial backend"),
    )

    hand_entry.main()
    assert '"mode": "preflight-only"' in capsys.readouterr().out


def test_read_only_requires_real_device_and_returns_hand_operation() -> None:
    assert validate_gate(_parse("--read-only")) is HandOperation.HAND_ONLY


def test_bounded_command_requires_short_duration_target_and_operator_checks() -> None:
    base = (
        "--bounded-command",
        "--channel",
        "index",
        "--delta",
        "0.05",
    )
    with pytest.raises(PermissionError, match="manual-stop-accessible"):
        validate_gate(_parse(*base))
    with pytest.raises(ValueError, match="10"):
        validate_gate(
            _parse(
                *base,
                "--manual-stop-accessible",
                "--workspace-clear",
                "--no-auto-retry",
                "--duration-sec",
                "11",
            )
        )
    assert validate_gate(
        _parse(
            *base,
            "--manual-stop-accessible",
            "--workspace-clear",
            "--no-auto-retry",
            "--duration-sec",
            "5",
        )
    ) is HandOperation.HAND_ONLY


def test_bounded_pose_and_channel_target_accept_full_normalized_domain() -> None:
    checks = (
        "--manual-stop-accessible",
        "--workspace-clear",
        "--no-auto-retry",
    )
    with pytest.raises(ValueError, match="six canonical"):
        validate_gate(
            _parse(
                "--bounded-pose",
                "--pose-label",
                "incomplete",
                "--target-normalized",
                "0.1",
                *checks,
            )
        )
    assert validate_gate(
        _parse(
            "--bounded-pose",
            "--pose-label",
            "log_max",
            "--target-normalized",
            "0.437",
            "0.443",
            "0.468",
            "0.569",
            "0.750",
            "1.000",
            *checks,
        )
    ) is HandOperation.HAND_ONLY


def test_full_normalized_endpoint_is_available_to_bounded_tests_and_teleop_has_no_override() -> None:
    checks = (
        "--manual-stop-accessible",
        "--workspace-clear",
        "--no-auto-retry",
    )
    assert validate_gate(
        _parse(
            "--bounded-pose",
            "--pose-label",
            "endpoint_probe",
            "--target-normalized",
            "0.55",
            "0",
            "0",
            "0",
            "0.35",
            "0.90",
            "--duration-sec",
            "3",
            "--hold-sec",
            "1",
            *checks,
        )
    ) is HandOperation.HAND_ONLY
    assert validate_gate(
        _parse(
            "--bounded-channel-target",
            "--channel",
            "thumb_lateral",
            "--target-normalized",
            "1.0",
            *checks,
        )
    ) is HandOperation.HAND_ONLY
    assert validate_gate(
        _parse(
            "--bounded-channel-target",
            "--channel",
            "thumb_lateral",
            "--target-normalized",
            "0.4",
            *checks,
        )
    ) is HandOperation.HAND_ONLY


def test_quest_hand_only_uses_explicit_operation_and_production_mode() -> None:
    args = _parse(
        "--quest-teleop",
        "--manual-stop-accessible",
        "--workspace-clear",
        "--no-auto-retry",
    )
    assert validate_gate(args) is HandOperation.HAND_ONLY
    assert args.channel is None
    assert args.delta is None
    assert args.hand_calibration == "configs/hand/quest_rh56_real_retarget.yaml"
    assert _parse("--preflight-only", "--scheduler-profile", "fast30").scheduler_profile == "fast30"


def test_mapping_check_requires_slow_operator_follow_duration() -> None:
    checks = (
        "--mapping-check",
        "--manual-stop-accessible",
        "--workspace-clear",
        "--no-auto-retry",
    )
    with pytest.raises(ValueError, match="at least 4"):
        validate_gate(_parse(*checks, "--mapping-hold-sec", "3.9"))
    assert validate_gate(_parse(*checks, "--mapping-hold-sec", "5")) is HandOperation.HAND_ONLY


def test_hand_only_and_live_defaults_share_real_physical_calibration() -> None:
    config = _load_hand_only_quest_config(
        "configs/sim/quest_hts_jaka_mini2_live_demo.yaml",
        "configs/hand/quest_rh56_real_retarget.yaml",
    )
    assert config.raw["hand_retargeting"]["calibration_path"] == (
        "configs/hand/quest_rh56_real_retarget.yaml"
    )
    assert config.raw["hand_retargeting"]["align_on_grip"] is True
    assert config.raw["hand_retargeting"]["align_index_pinch_to_validated_pose"] is True
    assert (
        hand_entry.ReplayConfig.load(
            "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
        ).raw["hand_retargeting"]["calibration_path"]
        == "configs/hand/quest_rh56_real_retarget.yaml"
    )


def test_physical_hand_path_rejects_sim_uncalibrated_mapping() -> None:
    with pytest.raises(ValueError, match="quest_rh56dfx_real"):
        _load_hand_only_quest_config(
            "configs/sim/quest_hts_jaka_mini2_live_demo.yaml",
            "configs/sim/quest_rh56_retarget.yaml",
        )


def test_custom_ch341_fallback_requires_explicit_flag(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(hand_entry, "require_serial_by_id_path", real_require_serial_by_id_path)
    direct = "/dev/ttyCH341USB0"
    parser = _build_parser()
    without = parser.parse_args(["--real", "--device", direct, "--preflight-only"])
    with pytest.raises(ValueError, match="CH341"):
        validate_gate(without)
    allowed = parser.parse_args([
        "--real",
        "--device", direct,
        "--allow-direct-ch341-device",
        "--preflight-only",
    ])
    assert validate_gate(allowed) is None


def test_runtime_configuration_has_distinct_operation_and_acknowledgement() -> None:
    base = (
        "--write-runtime-config",
        "--speed",
        "800",
        "800",
        "800",
        "800",
        "800",
        "800",
        "--force",
        "260",
        "260",
        "260",
        "260",
        "260",
        "260",
    )
    with pytest.raises(PermissionError, match="configuration-write-understood"):
        validate_gate(_parse(*base))
    assert validate_gate(
        _parse(*base, "--configuration-write-understood")
    ) is HandOperation.RUNTIME_CONFIG


def test_fault_reset_requires_obstruction_clear() -> None:
    base = (
        "--clear-error",
        "--manual-stop-accessible",
        "--workspace-clear",
        "--no-auto-retry",
    )
    with pytest.raises(PermissionError, match="mechanical-obstruction-cleared"):
        validate_gate(_parse(*base))
    assert validate_gate(
        _parse(*base, "--mechanical-obstruction-cleared")
    ) is HandOperation.FAULT_RESET


def test_force_calibration_requires_no_load_confirmation_and_observation_window() -> None:
    base = (
        "--force-sensor-calibration",
        "--manual-stop-accessible",
        "--workspace-clear",
        "--no-auto-retry",
        "--duration-sec",
        "8",
    )
    with pytest.raises(PermissionError, match="calibration-no-load-confirmed"):
        validate_gate(_parse(*base))
    assert validate_gate(
        _parse(*base, "--calibration-no-load-confirmed")
    ) is HandOperation.FORCE_SENSOR_CALIBRATION
    with pytest.raises(ValueError, match=r"\[8, 15\]"):
        validate_gate(
            _parse(
                "--force-sensor-calibration",
                "--manual-stop-accessible",
                "--workspace-clear",
                "--no-auto-retry",
                "--calibration-no-load-confirmed",
                "--duration-sec",
                "7",
            )
        )


def test_summary_uses_exclusive_create(tmp_path) -> None:
    path = tmp_path / "summary.json"
    _write_summary({"outcome": "first"}, str(path))
    with pytest.raises(FileExistsError):
        _write_summary({"outcome": "second"}, str(path))
