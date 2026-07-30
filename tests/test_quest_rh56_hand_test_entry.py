from __future__ import annotations

import sys

import pytest

from rh56_driver.pc_direct_control import (
    RH56_HAND_ONLY_COMMAND_APPROVAL,
    RH56_READ_ONLY_APPROVAL,
    RH56_RUNTIME_CONFIG_APPROVAL,
    HandAuthorization,
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
    return _build_parser().parse_args(["--device", DEVICE, *arguments])


def test_preflight_is_the_only_zero_approval_mode_and_does_not_construct_a_backend() -> None:
    args = _parse("--preflight-only")
    assert validate_gate(args) is None


def test_preflight_main_does_not_construct_or_open_backend(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        ["quest_rh56_hand_test.py", "--device", DEVICE, "--preflight-only"],
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


def test_read_only_requires_its_exact_rh56_approval() -> None:
    with pytest.raises(ValueError, match="approval"):
        validate_gate(_parse("--read-only"))
    assert validate_gate(
        _parse("--read-only", "--approval", RH56_READ_ONLY_APPROVAL)
    ) is HandAuthorization.READ_ONLY


def test_bounded_command_requires_short_duration_target_and_operator_checks() -> None:
    base = (
        "--bounded-command",
        "--approval",
        RH56_HAND_ONLY_COMMAND_APPROVAL,
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
    ) is HandAuthorization.HAND_ONLY_COMMAND


def test_bounded_pose_and_channel_target_require_explicit_safe_normalized_targets() -> None:
    checks = (
        "--approval",
        RH56_HAND_ONLY_COMMAND_APPROVAL,
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
    with pytest.raises(ValueError, match=r"\[0, 0.8\]"):
        validate_gate(
            _parse(
                "--bounded-pose",
                "--pose-label",
                "unsafe",
                "--target-normalized",
                "0.1",
                "0.2",
                "0.3",
                "0.4",
                "0.5",
                "0.81",
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
            "0.800",
            *checks,
        )
    ) is HandAuthorization.HAND_ONLY_COMMAND
    assert validate_gate(
        _parse(
            "--bounded-channel-target",
            "--channel",
            "thumb_lateral",
            "--target-normalized",
            "0.4",
            *checks,
        )
    ) is HandAuthorization.HAND_ONLY_COMMAND


def test_quest_hand_only_uses_command_approval_and_production_mode() -> None:
    args = _parse(
        "--quest-teleop",
        "--approval",
        RH56_HAND_ONLY_COMMAND_APPROVAL,
        "--manual-stop-accessible",
        "--workspace-clear",
        "--no-auto-retry",
    )
    assert validate_gate(args) is HandAuthorization.HAND_ONLY_COMMAND
    assert args.channel is None
    assert args.delta is None
    assert args.hand_calibration == "configs/hand/quest_rh56_real_retarget.yaml"
    assert _parse("--preflight-only", "--scheduler-profile", "fast30").scheduler_profile == "fast30"


def test_hand_only_path_overrides_sim_calibration_without_mutating_sim_default() -> None:
    config = _load_hand_only_quest_config(
        "configs/sim/quest_hts_jaka_mini2_live_demo.yaml",
        "configs/hand/quest_rh56_real_retarget.yaml",
    )
    assert config.raw["hand_retargeting"]["calibration_path"] == (
        "configs/hand/quest_rh56_real_retarget.yaml"
    )
    assert (
        hand_entry.ReplayConfig.load(
            "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
        ).raw["hand_retargeting"]["calibration_path"]
        == "configs/sim/quest_rh56_retarget.yaml"
    )


def test_custom_ch341_fallback_requires_explicit_flag(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(hand_entry, "require_serial_by_id_path", real_require_serial_by_id_path)
    direct = "/dev/ttyCH341USB0"
    parser = _build_parser()
    without = parser.parse_args(["--device", direct, "--preflight-only"])
    with pytest.raises(ValueError, match="CH341"):
        validate_gate(without)
    allowed = parser.parse_args([
        "--device", direct,
        "--allow-direct-ch341-device",
        "--preflight-only",
    ])
    assert validate_gate(allowed) is None


def test_runtime_configuration_has_distinct_mode_approval_and_acknowledgement() -> None:
    base = (
        "--write-runtime-config",
        "--approval",
        RH56_RUNTIME_CONFIG_APPROVAL,
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
    ) is HandAuthorization.RUNTIME_CONFIG


def test_summary_uses_exclusive_create(tmp_path) -> None:
    path = tmp_path / "summary.json"
    _write_summary({"outcome": "first"}, str(path))
    with pytest.raises(FileExistsError):
        _write_summary({"outcome": "second"}, str(path))
