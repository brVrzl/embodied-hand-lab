from __future__ import annotations

import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_quest_jaka_bounded_teleop.sh"
APPROVAL = "I_AUTHORIZE_BOUNDED_NORMAL_QUEST_JAKA_TELEOPERATION"


def _base_args(tmp_path: Path) -> list[str]:
    return [
        str(SCRIPT),
        "--robot-ip",
        "192.0.2.1",
        "--edg-state-ip",
        "192.0.2.2",
        "--duration-sec",
        "30",
        "--approval",
        APPROVAL,
        "--output-generator",
        "pwl-8ms",
        "--joint-velocity-limits-rad-s",
        "1.5",
        "1.5",
        "1.5",
        "1.2",
        "1.2",
        "1.2",
        "--log-dir",
        str(tmp_path / "logs"),
        "--worker",
        "/bin/true",
        "--no-auto-retry",
        "--estop-accessible",
        "--workspace-clear",
        "--rh56-command-path-absent",
    ]


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def test_shell_syntax_and_help_are_offline() -> None:
    syntax = _run(["bash", "-n", str(SCRIPT)])
    assert syntax.returncode == 0, syntax.stderr
    help_result = _run([str(SCRIPT), "--help"])
    assert help_result.returncode == 0
    assert "bounded normal-speed" in help_result.stdout
    assert "not official JAKA Mini2 maximum speeds" in help_result.stdout


def test_wrong_approval_is_rejected_before_outputs(tmp_path: Path) -> None:
    args = _base_args(tmp_path)
    args[args.index(APPROVAL)] = "WRONG"
    result = _run(args)
    assert result.returncode == 2
    assert APPROVAL in result.stderr
    assert not (tmp_path / "logs").exists()


def test_duration_above_sixty_seconds_is_rejected(tmp_path: Path) -> None:
    args = _base_args(tmp_path)
    args[args.index("30")] = "60.001"
    result = _run(args)
    assert result.returncode == 2
    assert "不超过 60" in result.stderr
    assert not (tmp_path / "logs").exists()


def test_six_joint_limits_and_no_auto_retry_are_required(tmp_path: Path) -> None:
    invalid = _base_args(tmp_path)
    first_limit = invalid.index("1.5")
    invalid[first_limit + 3] = "3.2"
    result = _run(invalid)
    assert result.returncode == 2
    assert "不超过 pi" in result.stderr

    missing_no_retry = _base_args(tmp_path)
    missing_no_retry.remove("--no-auto-retry")
    result = _run(missing_no_retry)
    assert result.returncode == 2
    assert "--no-auto-retry" in result.stderr


def test_complete_plant_free_command_uses_pwl_and_zero_rh56(
    tmp_path: Path,
) -> None:
    args = [*_base_args(tmp_path), "--plant-free-no-network-check"]
    result = _run(args)
    assert result.returncode == 0, result.stderr
    report = json.loads(
        next(
            line
            for line in reversed(result.stdout.splitlines())
            if line.startswith("{")
        )
    )
    assert report["stage"] == "bounded-normal-teleop"
    assert report["validation"] == "plant-free-no-network"
    assert report["network_attempted"] is False
    assert report["hardware_commands_sent"] == 0
    assert report["rh56_commands"] == 0
    assert report["output_generator"] == "pwl-8ms"
    assert report["native_mode"] == "joint-teleop"
    assert report["native_ik_calls"] == 0
    assert report["step_num"] == 1
    assert report["run_output_joint_velocity_limits_rad_s"] == [
        1.5,
        1.5,
        1.5,
        1.2,
        1.2,
        1.2,
    ]
    assert report["no_auto_retry"] is True
    assert not (tmp_path / "logs").exists()


def test_output_generator_is_explicit_and_not_diagnostic_disguise(
    tmp_path: Path,
) -> None:
    missing_generator = _base_args(tmp_path)
    index = missing_generator.index("--output-generator")
    del missing_generator[index : index + 2]
    result = _run(missing_generator)
    assert result.returncode == 2
    assert "pwl-8ms" in result.stderr

    source = SCRIPT.read_text(encoding="utf-8")
    assert "bounded-normal-teleop" in source
    assert "post-payload-diagnostic" not in source
    assert "rh56-command-path-absent" in source
    assert "One exec, no retry loop." in source
