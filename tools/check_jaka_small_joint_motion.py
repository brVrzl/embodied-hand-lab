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
    payload = result[1] if isinstance(result, tuple) and len(result) >= 2 else result
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


def _max_abs_delta(a: list[float], b: list[float]) -> float:
    return max(abs(x - y) for x, y in zip(a, b))


def _collect_state_flags(backend: JakaSDKBackend) -> dict[str, Any]:
    robot = getattr(backend, "_robot", None)
    flags: dict[str, Any] = {}
    if robot is None:
        return flags
    for method_name in (
        "is_in_estop",
        "is_in_collision",
        "is_on_limit",
        "is_in_drag_mode",
        "is_in_servomove",
        "is_in_pos",
    ):
        if hasattr(robot, method_name):
            flags[method_name] = _extract_scalar(backend.call_sdk_method(method_name))
    if hasattr(robot, "get_last_error"):
        flags["last_error_raw"] = _extract_scalar(backend.call_sdk_method("get_last_error"))
    return flags


def _find_blockers(flags: dict[str, Any]) -> list[str]:
    blockers: list[str] = []
    if flags.get("is_in_estop", 0) != 0:
        blockers.append("robot_in_estop")
    if flags.get("is_in_collision", 0) != 0:
        blockers.append("robot_in_collision")
    if flags.get("is_on_limit", 0) != 0:
        blockers.append("robot_on_limit")
    if flags.get("is_in_drag_mode", 0) != 0:
        blockers.append("robot_in_drag_mode")
    if flags.get("is_in_servomove", 0) != 0:
        blockers.append("robot_in_servomove")
    last_error = flags.get("last_error_raw")
    if isinstance(last_error, list) and last_error and last_error[0] != 0:
        blockers.append("robot_last_error")
    return blockers


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a tiny bounded JAKA joint motion and return to the starting joint state."
    )
    parser.add_argument("--config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--joint-index", type=int, default=6, help="1-based joint index.")
    parser.add_argument("--delta-rad", type=float, default=0.01)
    parser.add_argument("--max-delta-rad", type=float, default=0.03)
    parser.add_argument("--speed-scale", type=float, default=0.02)
    parser.add_argument("--settle-sec", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--no-return", action="store_true")
    args = parser.parse_args()

    if not 1 <= args.joint_index <= 6:
        raise SystemExit("--joint-index must be within [1, 6].")
    if abs(args.delta_rad) > args.max_delta_rad:
        raise SystemExit("--delta-rad exceeds --max-delta-rad.")

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
        "joint_index": args.joint_index,
        "delta_rad": args.delta_rad,
        "max_delta_rad": args.max_delta_rad,
        "speed_scale": args.speed_scale,
        "execute": args.execute,
        "return_to_start": not args.no_return,
    }

    try:
        adapter.connect()
        result["connect_ok"] = True
        if not isinstance(backend, JakaSDKBackend) or getattr(backend, "_robot", None) is None:
            raise RuntimeError("Small joint motion requires the official JAKA SDK backend.")

        state_flags = _collect_state_flags(backend)
        result["state_flags_before"] = state_flags
        blockers = _find_blockers(state_flags)

        start_state = adapter.get_joint_state()
        time.sleep(args.settle_sec)
        confirm_state = adapter.get_joint_state()
        result["start_joint_state"] = start_state.to_dict()
        result["confirm_joint_state"] = confirm_state.to_dict()
        result["stationary_delta_rad"] = _max_abs_delta(
            start_state.positions, confirm_state.positions
        )
        if result["stationary_delta_rad"] > 0.002:
            blockers.append("robot_not_stationary")

        target = [float(v) for v in confirm_state.positions]
        target[args.joint_index - 1] += args.delta_rad
        result["target_joint_state"] = {
            "names": list(confirm_state.names),
            "positions": target,
        }
        result["precheck_blockers"] = blockers
        result["precheck_ok"] = len(blockers) == 0

        if not args.execute:
            result["action"] = "precheck_only"
        elif blockers:
            raise RuntimeError(f"Small joint motion blocked by precheck: {blockers}")
        else:
            adapter.set_speed_scale(args.speed_scale)
            result["action"] = "small_joint_delta_and_return"
            result["move_out_ok"] = adapter.move_joints(target, blocking=True)
            out_state = adapter.get_joint_state()
            result["out_joint_state"] = out_state.to_dict()
            result["out_target_error_rad"] = _max_abs_delta(out_state.positions, target)
            result["state_flags_after_out"] = _collect_state_flags(backend)

            if not args.no_return:
                result["return_ok"] = adapter.move_joints(confirm_state.positions, blocking=True)
                return_state = adapter.get_joint_state()
                result["return_joint_state"] = return_state.to_dict()
                result["return_target_error_rad"] = _max_abs_delta(
                    return_state.positions, confirm_state.positions
                )
                result["state_flags_after_return"] = _collect_state_flags(backend)
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
