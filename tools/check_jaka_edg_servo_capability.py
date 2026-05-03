from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend


EDG_METHODS = [
    "edg_init",
    "edg_get_stat",
    "edg_stat_details",
    "edg_servo_j",
    "edg_servo_j_extend",
    "edg_servo_p",
    "edg_servo_p_extend",
]

SERVO_METHODS = [
    "servo_move_enable",
    "is_in_servomove",
    "servo_j",
    "servo_j_extend",
    "servo_p",
    "servo_p_extend",
    "servo_move_use_none_filter",
    "servo_move_use_joint_LPF",
    "servo_move_use_joint_NLF",
    "servo_move_use_carte_NLF",
]

SAFETY_FLAG_METHODS = [
    "is_in_estop",
    "is_in_collision",
    "is_on_limit",
    "is_in_drag_mode",
    "is_in_servomove",
]


def _extract_scalar(result: Any) -> Any:
    payload = result[1] if isinstance(result, tuple) and len(result) >= 2 else result
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


def _collect_method_presence(robot: Any) -> dict[str, bool]:
    return {name: hasattr(robot, name) for name in EDG_METHODS + SERVO_METHODS}


def _collect_state_flags(backend: JakaSDKBackend) -> dict[str, Any]:
    flags: dict[str, Any] = {}
    robot = getattr(backend, "_robot", None)
    if robot is None:
        return flags
    for method_name in SAFETY_FLAG_METHODS:
        if hasattr(robot, method_name):
            try:
                flags[method_name] = _extract_scalar(backend.call_sdk_method(method_name))
            except Exception as exc:
                flags[f"{method_name}_error"] = str(exc)
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
    return blockers


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect JAKA SDK EDG/servo capabilities. Default mode imports the SDK only. "
            "Use --connect for controller diagnostics. Use --execute-enable-cycle only for "
            "a no-motion servo enable/disable cycle after safety prechecks."
        )
    )
    parser.add_argument("--config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--connect", action="store_true")
    parser.add_argument(
        "--execute-enable-cycle",
        action="store_true",
        help="Actually call servo_move_enable(True) then False. Requires --connect.",
    )
    parser.add_argument("--settle-sec", type=float, default=0.2)
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)
    config["mode"] = "real"
    config["backend_type"] = "jaka_sdk"
    if args.ip:
        config["ip"] = args.ip

    backend = JakaSDKBackend(config)
    result: dict[str, Any] = {
        "config": str(config_path.resolve()),
        "ip": config.get("ip"),
        "connect": args.connect,
        "execute_enable_cycle": args.execute_enable_cycle,
    }

    try:
        sdk_mod = backend._load_sdk_module()
        result["sdk_import_ok"] = True
        result["sdk_module"] = getattr(sdk_mod, "__name__", repr(sdk_mod))
        robot_cls = getattr(sdk_mod, "RC", None)
        result["exports_RC"] = robot_cls is not None

        if not args.connect:
            return

        backend.connect()
        result["connect_ok"] = True
        robot = getattr(backend, "_robot", None)
        result["method_presence"] = _collect_method_presence(robot)
        result["state_flags_before"] = _collect_state_flags(backend)
        result["precheck_blockers"] = _find_blockers(result["state_flags_before"])

        required_for_edg = ["edg_init", "servo_move_enable", "edg_servo_j", "edg_servo_p"]
        result["edg_minimum_available"] = all(
            result["method_presence"].get(name, False) for name in required_for_edg
        )

        if args.execute_enable_cycle:
            if result["precheck_blockers"]:
                raise RuntimeError(
                    f"Servo enable cycle blocked by safety flags: {result['precheck_blockers']}"
                )
            if not result["method_presence"].get("servo_move_enable", False):
                raise RuntimeError("SDK object does not expose servo_move_enable().")
            backend.ensure_success(backend.call_sdk_method("servo_move_enable", True), "servo_move_enable(True)")
            time.sleep(args.settle_sec)
            result["state_flags_enabled"] = _collect_state_flags(backend)
            backend.ensure_success(backend.call_sdk_method("servo_move_enable", False), "servo_move_enable(False)")
            time.sleep(args.settle_sec)
            result["state_flags_after_disable"] = _collect_state_flags(backend)
    except Exception as exc:
        result["error"] = str(exc)
        raise
    finally:
        try:
            backend.disconnect()
        except Exception as exc:
            if args.connect:
                result["disconnect_error"] = str(exc)
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
