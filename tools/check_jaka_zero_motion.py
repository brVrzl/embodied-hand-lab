from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend


def _extract_scalar(result: Any) -> Any:
    if isinstance(result, tuple) and len(result) >= 2:
        payload = result[1]
    else:
        payload = result
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


def _max_abs_delta(a: list[float], b: list[float]) -> float:
    return max(abs(x - y) for x, y in zip(a, b))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="JAKA zero-displacement motion validation. Default is precheck only."
    )
    parser.add_argument("--config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--speed-scale", type=float, default=0.05)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--stationary-threshold-rad", type=float, default=0.002)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send move_joints(current_joint_state). Without this flag the script is read-only.",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)
    config["mode"] = "real"
    config["backend_type"] = "jaka_sdk"
    config["default_speed_scale"] = args.speed_scale
    if args.ip:
        config["ip"] = args.ip

    adapter = JakaDriverAdapter(config)
    backend = adapter.backend
    result: dict[str, Any] = {
        "config": str(config_path.resolve()),
        "ip": config["ip"],
        "speed_scale": args.speed_scale,
        "execute": args.execute,
    }

    try:
        adapter.connect()
        result["connect_ok"] = True

        if not isinstance(backend, JakaSDKBackend) or getattr(backend, "_robot", None) is None:
            raise RuntimeError("JAKA zero-motion check requires the official SDK backend.")

        robot = backend._robot
        state_flags = {}
        for method_name in (
            "is_in_estop",
            "is_in_collision",
            "is_on_limit",
            "is_in_drag_mode",
            "is_in_servomove",
            "is_in_pos",
        ):
            if hasattr(robot, method_name):
                state_flags[method_name] = _extract_scalar(backend._invoke(method_name))
        result["state_flags"] = state_flags

        joint_state_1 = adapter.get_joint_state()
        time.sleep(args.settle_seconds)
        joint_state_2 = adapter.get_joint_state()

        result["joint_state_1"] = joint_state_1.to_dict()
        result["joint_state_2"] = joint_state_2.to_dict()
        result["max_joint_delta_rad"] = _max_abs_delta(
            joint_state_1.positions, joint_state_2.positions
        )
        result["stationary_ok"] = (
            result["max_joint_delta_rad"] <= args.stationary_threshold_rad
        )

        blockers: list[str] = []
        if state_flags.get("is_in_estop", 0) != 0:
            blockers.append("robot_in_estop")
        if state_flags.get("is_in_collision", 0) != 0:
            blockers.append("robot_in_collision")
        if state_flags.get("is_on_limit", 0) != 0:
            blockers.append("robot_on_limit")
        if state_flags.get("is_in_drag_mode", 0) != 0:
            blockers.append("robot_in_drag_mode")
        if state_flags.get("is_in_servomove", 0) != 0:
            blockers.append("robot_in_servomove")
        if not result["stationary_ok"]:
            blockers.append("robot_not_stationary")
        result["precheck_blockers"] = blockers
        result["precheck_ok"] = len(blockers) == 0

        if not args.execute:
            result["action"] = "precheck_only"
        elif blockers:
            raise RuntimeError(f"Zero-motion command blocked by precheck: {blockers}")
        else:
            result["action"] = "move_joints_current_position"
            adapter.set_speed_scale(args.speed_scale)
            move_ok = adapter.move_joints(joint_state_2.positions, blocking=True)
            result["move_ok"] = move_ok
            post_state = adapter.get_joint_state()
            result["post_joint_state"] = post_state.to_dict()
            result["post_move_max_delta_rad"] = _max_abs_delta(
                joint_state_2.positions, post_state.positions
            )
    except Exception as exc:
        result["connect_ok"] = result.get("connect_ok", False)
        result["error"] = str(exc)
        raise
    finally:
        if isinstance(backend, JakaSDKBackend):
            try:
                backend.disconnect()
            except Exception as exc:
                result.setdefault("disconnect_error", str(exc))
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
