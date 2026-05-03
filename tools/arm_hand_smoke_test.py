from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend
from rh56_driver.interfaces import HandCommand
from rh56_driver.jaka_tool_backend import RH56JakaToolBackend


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


def _arm_precheck(backend: JakaSDKBackend, stationary_threshold_rad: float, settle_seconds: float) -> dict[str, Any]:
    robot = backend._robot
    state_flags: dict[str, Any] = {}
    for method_name in (
        "is_in_estop",
        "is_in_collision",
        "is_on_limit",
        "is_in_drag_mode",
        "is_in_servomove",
        "is_in_pos",
    ):
        if hasattr(robot, method_name):
            state_flags[method_name] = _extract_scalar(backend.call_sdk_method(method_name))

    joint_state_1 = backend.get_joint_state()
    time.sleep(settle_seconds)
    joint_state_2 = backend.get_joint_state()
    max_joint_delta = _max_abs_delta(joint_state_1.positions, joint_state_2.positions)

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
    if max_joint_delta > stationary_threshold_rad:
        blockers.append("robot_not_stationary")

    return {
        "state_flags": state_flags,
        "joint_state_1": joint_state_1.to_dict(),
        "joint_state_2": joint_state_2.to_dict(),
        "max_joint_delta_rad": max_joint_delta,
        "stationary_ok": max_joint_delta <= stationary_threshold_rad,
        "precheck_blockers": blockers,
        "precheck_ok": len(blockers) == 0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal real-world JAKA + RH56 smoke test using a named joint preset.")
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--hand-config", default="configs/hand/rh56.yaml")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--preset-name", default="upright")
    parser.add_argument("--speed-scale", type=float, default=0.05)
    parser.add_argument("--hand-id", type=int, default=1)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--stationary-threshold-rad", type=float, default=0.002)
    parser.add_argument("--hand-pause-sec", type=float, default=0.8)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send the arm move and hand open/close sequence. Without this flag the script is read-only.",
    )
    args = parser.parse_args()

    robot_config_path = Path(args.robot_config)
    hand_config_path = Path(args.hand_config)
    robot_config = load_yaml(robot_config_path)
    hand_config = load_yaml(hand_config_path)

    robot_config["mode"] = "real"
    robot_config["backend_type"] = "jaka_sdk"
    robot_config["default_speed_scale"] = args.speed_scale
    if args.ip:
        robot_config["ip"] = args.ip

    joint_presets = robot_config.get("joint_presets", {})
    preset = joint_presets.get(args.preset_name)
    if preset is None:
        raise RuntimeError(f"Unknown JAKA joint preset: {args.preset_name}")
    target_joints = [float(v) for v in preset]

    result: dict[str, Any] = {
        "robot_config": str(robot_config_path.resolve()),
        "hand_config": str(hand_config_path.resolve()),
        "ip": robot_config["ip"],
        "preset_name": args.preset_name,
        "target_joints": target_joints,
        "speed_scale": args.speed_scale,
        "hand_id": args.hand_id,
        "execute": args.execute,
    }

    arm = JakaDriverAdapter(robot_config)
    arm_backend = arm.backend

    try:
        arm.connect()
        result["arm_connect_ok"] = True
        if not isinstance(arm_backend, JakaSDKBackend):
            raise RuntimeError("arm_hand_smoke_test requires the official JAKA SDK backend.")

        precheck = _arm_precheck(arm_backend, args.stationary_threshold_rad, args.settle_seconds)
        result["arm_precheck"] = precheck
        result["current_joint_state"] = arm.get_joint_state().to_dict()
        result["target_max_delta_rad"] = _max_abs_delta(result["current_joint_state"]["positions"], target_joints)

        if not args.execute:
            result["action"] = "precheck_only"
        else:
            blockers = precheck["precheck_blockers"]
            if blockers:
                raise RuntimeError(f"Arm move blocked by precheck: {blockers}")

            arm.set_speed_scale(args.speed_scale)
            result["arm_move_ok"] = arm.move_joints(target_joints, blocking=True)
            result["post_arm_joint_state"] = arm.get_joint_state().to_dict()
            result["post_arm_max_delta_rad"] = _max_abs_delta(
                result["post_arm_joint_state"]["positions"],
                target_joints,
            )
    finally:
        if isinstance(arm_backend, JakaSDKBackend):
            try:
                arm_backend.disconnect()
            except Exception as exc:
                result.setdefault("arm_disconnect_error", str(exc))

    if not args.execute:
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return

    hand_config["mode"] = "real"
    hand_config["backend_type"] = "jaka_tool_rs485"
    transport_cfg = hand_config.setdefault("jaka_tool_rs485", {})
    transport_cfg["hand_id"] = args.hand_id
    transport_cfg["command_pause_sec"] = args.hand_pause_sec
    prepare_cfg = transport_cfg.setdefault("prepare_tio", {})
    prepare_cfg["enable_vout"] = True
    prepare_cfg["set_ai_pin_mode"] = True
    prepare_cfg["set_channel_mode"] = True
    prepare_cfg["channel_mode"] = RH56JakaToolBackend.RAW_RS485_MODE
    prepare_cfg["set_comm"] = False
    hand_config.setdefault("serial", {})["hand_id"] = args.hand_id

    hand = RH56JakaToolBackend(hand_config)
    if args.ip:
        hand.jaka_backend.config["ip"] = args.ip

    try:
        hand.connect()
        result["hand_connect_ok"] = True
        result["hand_transport_status"] = hand.get_transport_status()
        result["hand_close_ok"] = hand.execute(HandCommand(command="close"))
        result["hand_state_after_close"] = hand.read_state().to_dict()
        result["hand_open_ok"] = hand.execute(HandCommand(command="open"))
        result["hand_state_after_open"] = hand.read_state().to_dict()
    finally:
        try:
            hand.disconnect()
        except Exception as exc:
            result.setdefault("hand_disconnect_error", str(exc))

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
