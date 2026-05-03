from __future__ import annotations

import argparse
import json
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
    parser = argparse.ArgumentParser(description="Move JAKA to a named joint preset safely.")
    parser.add_argument("--config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--preset-name", required=True)
    parser.add_argument("--speed-scale", type=float, default=0.03)
    parser.add_argument(
        "--max-joint-delta-rad",
        type=float,
        default=0.8,
        help="Block if any joint target-current delta exceeds this threshold.",
    )
    parser.add_argument("--execute", action="store_true", help="Send the motion command.")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)
    config["mode"] = "real"
    config["backend_type"] = "jaka_sdk"
    config["default_speed_scale"] = args.speed_scale
    if args.ip:
        config["ip"] = args.ip

    presets = config.get("joint_presets", {})
    if args.preset_name not in presets:
        raise SystemExit(f"Preset {args.preset_name!r} not found in {config_path}.")
    target = [float(v) for v in presets[args.preset_name]]
    if len(target) != 6:
        raise SystemExit(f"Preset {args.preset_name!r} must contain 6 joint values.")

    adapter = JakaDriverAdapter(config)
    backend = adapter.backend
    result: dict[str, Any] = {
        "config": str(config_path.resolve()),
        "ip": config["ip"],
        "preset_name": args.preset_name,
        "target_joints": target,
        "speed_scale": args.speed_scale,
        "max_joint_delta_rad": args.max_joint_delta_rad,
        "execute": args.execute,
    }

    try:
        adapter.connect()
        result["connect_ok"] = True
        if not isinstance(backend, JakaSDKBackend) or getattr(backend, "_robot", None) is None:
            raise RuntimeError("move_jaka_preset requires the official JAKA SDK backend.")

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

        current = adapter.get_joint_state()
        current_joints = [float(v) for v in current.positions]
        result["current_joint_state"] = current.to_dict()
        result["joint_delta_to_target"] = [t - c for t, c in zip(target, current_joints)]
        result["max_abs_joint_delta_to_target"] = _max_abs_delta(current_joints, target)

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
        if result["max_abs_joint_delta_to_target"] > args.max_joint_delta_rad:
            blockers.append("target_too_far")
        result["precheck_blockers"] = blockers
        result["precheck_ok"] = len(blockers) == 0

        if not args.execute:
            result["action"] = "precheck_only"
        elif blockers:
            raise RuntimeError(f"Preset motion blocked by precheck: {blockers}")
        else:
            result["action"] = "move_joints_preset"
            adapter.set_speed_scale(args.speed_scale)
            result["move_ok"] = adapter.move_joints(target, blocking=True)
            result["post_joint_state"] = adapter.get_joint_state().to_dict()
            result["post_move_max_delta_rad"] = _max_abs_delta(
                [float(v) for v in result["post_joint_state"]["positions"]], target
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
