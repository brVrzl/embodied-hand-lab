from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from embodiment_core.types import Pose
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend


def _extract_payload(result: Any) -> Any:
    if isinstance(result, tuple) and len(result) >= 2:
        return result[1]
    return result


def _extract_scalar(result: Any) -> Any:
    payload = _extract_payload(result)
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


def _extract_pose_raw(result: Any) -> list[float]:
    payload = _extract_payload(result)
    if hasattr(payload, "tran") and hasattr(payload, "rpy"):
        return [
            float(payload.tran.x),
            float(payload.tran.y),
            float(payload.tran.z),
            float(payload.rpy.rx),
            float(payload.rpy.ry),
            float(payload.rpy.rz),
        ]
    if isinstance(payload, (list, tuple)) and len(payload) >= 6:
        return [float(v) for v in payload[:6]]
    raise RuntimeError(f"Unrecognized TCP pose payload: {result!r}")


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _pose_from_mm_rpy(values: list[float]) -> Pose:
    return Pose(
        position=[values[0] / 1000.0, values[1] / 1000.0, values[2] / 1000.0],
        orientation_xyzw=_rpy_to_quat(values[3], values[4], values[5]),
        frame_id="jaka_base",
    )


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
        description="Run a tiny bounded JAKA Cartesian TCP motion and return to the start pose."
    )
    parser.add_argument("--config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--dx-mm", type=float, default=0.0)
    parser.add_argument("--dy-mm", type=float, default=0.0)
    parser.add_argument("--dz-mm", type=float, default=2.0)
    parser.add_argument("--max-translation-mm", type=float, default=5.0)
    parser.add_argument("--speed-scale", type=float, default=0.01)
    parser.add_argument("--settle-sec", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--no-return", action="store_true")
    args = parser.parse_args()

    delta = [args.dx_mm, args.dy_mm, args.dz_mm]
    if max(abs(v) for v in delta) > args.max_translation_mm:
        raise SystemExit("Requested TCP translation exceeds --max-translation-mm.")

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
        "delta_mm": delta,
        "max_translation_mm": args.max_translation_mm,
        "speed_scale": args.speed_scale,
        "execute": args.execute,
        "return_to_start": not args.no_return,
    }

    try:
        adapter.connect()
        result["connect_ok"] = True
        if not isinstance(backend, JakaSDKBackend) or getattr(backend, "_robot", None) is None:
            raise RuntimeError("Small TCP motion requires the official JAKA SDK backend.")

        flags = _collect_state_flags(backend)
        result["state_flags_before"] = flags
        blockers = _find_blockers(flags)

        current = _extract_pose_raw(backend.call_sdk_method("get_actual_tcp_position"))
        time.sleep(args.settle_sec)
        confirm = _extract_pose_raw(backend.call_sdk_method("get_actual_tcp_position"))
        result["current_tcp_pose_mm_rpy"] = current
        result["confirm_tcp_pose_mm_rpy"] = confirm
        result["stationary_pose_delta_mm_rpy"] = [b - a for a, b in zip(current, confirm)]
        if _max_abs_delta(current[:3], confirm[:3]) > 0.5:
            blockers.append("tcp_not_stationary")

        target = list(confirm)
        target[0] += args.dx_mm
        target[1] += args.dy_mm
        target[2] += args.dz_mm
        result["target_tcp_pose_mm_rpy"] = target
        result["precheck_blockers"] = blockers
        result["precheck_ok"] = len(blockers) == 0

        if not args.execute:
            result["action"] = "precheck_only"
        elif blockers:
            raise RuntimeError(f"Small TCP motion blocked by precheck: {blockers}")
        else:
            adapter.set_speed_scale(args.speed_scale)
            result["action"] = "small_tcp_delta_and_return"
            result["move_out_ok"] = adapter.move_pose(_pose_from_mm_rpy(target), blocking=True)
            out_pose = _extract_pose_raw(backend.call_sdk_method("get_actual_tcp_position"))
            result["out_tcp_pose_mm_rpy"] = out_pose
            result["out_target_error_mm_rpy"] = [o - t for o, t in zip(out_pose, target)]
            result["state_flags_after_out"] = _collect_state_flags(backend)

            if not args.no_return:
                result["return_ok"] = adapter.move_pose(_pose_from_mm_rpy(confirm), blocking=True)
                return_pose = _extract_pose_raw(backend.call_sdk_method("get_actual_tcp_position"))
                result["return_tcp_pose_mm_rpy"] = return_pose
                result["return_target_error_mm_rpy"] = [
                    r - c for r, c in zip(return_pose, confirm)
                ]
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
