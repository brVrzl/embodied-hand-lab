from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from teleop_tools.hebi_mobile_io import HebiMobileIOSnapshot
from teleop_tools.hebi_rviz_shadow import (
    _actual_palm_pose_from_state,
    _make_shadow_ik_checker,
    _relative_config_from_config,
    _shadow_state_from_config,
)
from teleop_tools.relative_pose_lag_follow import RelativePoseLagFollower


DEFAULT_CONFIG = "configs/teleop/hebi_mobile_io_jaka_rh56.yaml"


def _synthetic_snapshots(start: float) -> list[HebiMobileIOSnapshot]:
    positions = [
        [0.0, 0.0, 0.0],
        [0.0, 0.020, 0.0],
        [0.0, 0.040, 0.0],
        [0.010, 0.045, 0.005],
        [0.012, 0.045, 0.006],
        [0.012, 0.045, 0.006],
    ]
    yaw_deg = [0.0, 8.0, 16.0, 24.0, 24.0, 24.0]
    return [
        HebiMobileIOSnapshot(
            timestamp_sec=start + index * 0.10,
            position_m=position,
            quaternion_wxyz=[
                math.cos(math.radians(yaw_deg[index]) / 2.0),
                0.0,
                0.0,
                math.sin(math.radians(yaw_deg[index]) / 2.0),
            ],
            raw_inputs={"b1": True, "a3": 0.0},
        )
        for index, position in enumerate(positions)
    ]


def run_check(
    *,
    config_path: str | Path,
    jsonl_out: str | Path | None = None,
    viewer_preview: bool = False,
    preview_hold_sec: float = 0.8,
) -> dict[str, Any]:
    config = load_yaml(config_path)
    ik_state = _shadow_state_from_config(config, None)
    follower = RelativePoseLagFollower(
        _relative_config_from_config(config),
        ik_checker=_make_shadow_ik_checker(ik_state),
    )
    viewer_handle: Any | None = None
    if viewer_preview:
        import mujoco.viewer

        viewer_handle = mujoco.viewer.launch_passive(ik_state.model, ik_state.data)
        viewer_handle.cam.azimuth = -130
        viewer_handle.cam.elevation = -25
        viewer_handle.cam.distance = 0.85
        viewer_handle.cam.lookat[:] = [-0.18, 0.0, 0.20]
    records: list[dict[str, Any]] = []
    q_current = ik_state.arm_joints_rad.tolist()
    try:
        for snapshot in _synthetic_snapshots(time.time()):
            actual = _actual_palm_pose_from_state(ik_state)
            output = follower.step(snapshot, actual, q_current, timestamp_sec=snapshot.timestamp_sec)
            if output.command_deadman and output.palm_target_position_m is not None:
                q_cmd = output.log.get("q_cmd")
                if q_cmd is not None:
                    q_current = [float(value) for value in q_cmd]
                    ik_state.set_arm_joints_rad(q_current)
            if viewer_handle is not None and viewer_handle.is_running():
                viewer_handle.sync()
                time.sleep(max(0.0, preview_hold_sec))
            records.append(
                {
                    "phone_position_m": snapshot.position_m,
                    "phone_quaternion_wxyz": snapshot.quaternion_wxyz,
                    "command_deadman": output.command_deadman,
                    "palm_target_position_m": output.palm_target_position_m,
                    "palm_target_quaternion_wxyz": output.palm_target_quaternion_wxyz,
                    "q_cmd": output.log.get("q_cmd"),
                    "reason": output.log.get("reason", "ok"),
                    "tcp_tracking_error_rot": output.log.get("tcp_tracking_error_rot"),
                    "bounded": output.log.get("desired_tcp_pose_workspace_bounded"),
                }
            )
    finally:
        if viewer_handle is not None:
            viewer_handle.close()

    commanded = [record for record in records if record["command_deadman"] and record["q_cmd"] is not None]
    orientation_commanded = [
        record for record in commanded if record["palm_target_quaternion_wxyz"] is not None
    ]
    result = {
        "ok": len(commanded) >= 2 and len(orientation_commanded) >= 2,
        "config": str(config_path),
        "steps": len(records),
        "commanded_steps": len(commanded),
        "orientation_commanded_steps": len(orientation_commanded),
        "final_q": q_current,
        "records": records,
    }
    if jsonl_out is not None:
        out_path = Path(jsonl_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", encoding="utf-8") as handle:
            for record in records:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay synthetic HEBI Mobile I/O poses through RViz shadow IK.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--jsonl-out", default="")
    parser.add_argument("--viewer-preview", action="store_true")
    parser.add_argument("--preview-hold-sec", type=float, default=0.8)
    args = parser.parse_args()
    result = run_check(
        config_path=args.config,
        jsonl_out=args.jsonl_out or None,
        viewer_preview=args.viewer_preview,
        preview_hold_sec=args.preview_hold_sec,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if not result["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
