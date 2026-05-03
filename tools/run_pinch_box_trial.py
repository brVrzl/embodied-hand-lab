from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]


def _run(cmd: list[str]) -> None:
    completed = subprocess.run(cmd, cwd=ROOT, text=True)
    if completed.returncode != 0:
        raise RuntimeError(f"Command failed: {' '.join(cmd)}")


def _move(preset: str, speed_scale: float, max_joint_delta_rad: float) -> None:
    _run(
        [
            "./scripts/move_jaka_preset.sh",
            "--config",
            "configs/robot/jaka_mini2_real.yaml",
            "--preset-name",
            preset,
            "--speed-scale",
            str(speed_scale),
            "--max-joint-delta-rad",
            str(max_joint_delta_rad),
            "--execute",
        ]
    )


def _hand(commands: list[str], speed_scale: float) -> None:
    payload = "".join(f"{command}\n" for command in commands) + "quit\n"
    completed = subprocess.run(
        [
            "./scripts/start_grasp_debug_cli.sh",
            "--speed-scale",
            str(speed_scale),
            "--max-delta-mm",
            "20",
            "--max-rot-deg",
            "5",
        ],
        cwd=ROOT,
        input=payload,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"Hand command failed: {commands}")


def _append_trial(record: dict[str, Any]) -> None:
    out = ROOT / "data/real_debug/pinch_box_v1/trials.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one scripted top-down pinch box lift trial.")
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--speed-scale", type=float, default=0.12)
    parser.add_argument("--max-joint-delta-rad", type=float, default=2.0)
    parser.add_argument("--skip-upright", action="store_true")
    parser.add_argument("--hold-sec", type=float, default=2.0)
    parser.add_argument("--manual-success", choices=("true", "false", "unknown"), default="unknown")
    parser.add_argument("--failure-mode", default=None)
    parser.add_argument("--notes", default="")
    args = parser.parse_args()

    start_time = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    _hand(["open"], args.speed_scale)
    if not args.skip_upright:
        _move("upright", args.speed_scale, args.max_joint_delta_rad)
    _move("pinch_grasp_box_v2", args.speed_scale, args.max_joint_delta_rad)
    _hand(["preset pinch_box_thumb_rotate_v2", "preset pinch_box_v4"], args.speed_scale)
    _move("pinch_lift_box_v1", args.speed_scale, args.max_joint_delta_rad)
    time.sleep(max(0.0, args.hold_sec))

    success: bool | None
    if args.manual_success == "unknown":
        success = None
    else:
        success = args.manual_success == "true"
    _append_trial(
        {
            "trial_id": args.trial_id,
            "timestamp": start_time,
            "task": "top_down_pinch_box_lift",
            "object": "small_paper_box",
            "start_preset": "upright" if not args.skip_upright else "current",
            "grasp_preset": "pinch_grasp_box_v2",
            "hand_stages": ["pinch_box_thumb_rotate_v2", "pinch_box_v4"],
            "lift_preset": "pinch_lift_box_v1",
            "speed_scale": args.speed_scale,
            "manual_success": success,
            "failure_mode": args.failure_mode,
            "notes": args.notes,
        }
    )


if __name__ == "__main__":
    main()
