from __future__ import annotations

import argparse
import json
import math
import shlex
from pathlib import Path
from typing import Any

import yaml

from embodiment_core.config import load_yaml
from embodiment_core.types import Pose
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend
from rh56_driver.jaka_tio_signal_client import JakaTioSignalClient

CANONICAL_HAND_ORDER: tuple[str, ...] = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)

PHYSICAL_DOF_LABELS: tuple[str, ...] = (
    "pinky_bend",
    "ring_bend",
    "middle_bend",
    "index_bend",
    "thumb_bend",
    "thumb_rotate",
)

REG_ANGLE_SET = 1486


def _extract_payload(result: Any) -> Any:
    if isinstance(result, tuple) and len(result) >= 2:
        return result[1]
    return result


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


def _extract_scalar(result: Any) -> Any:
    payload = _extract_payload(result)
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


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


def _load_arm_config(path: Path, speed_scale: float) -> dict[str, Any]:
    config = load_yaml(path)
    config["mode"] = "real"
    config["backend_type"] = "jaka_sdk"
    config["default_speed_scale"] = speed_scale
    return config


def _state_flags(backend: JakaSDKBackend) -> dict[str, Any]:
    robot = getattr(backend, "_robot", None)
    if robot is None:
        return {}
    flags: dict[str, Any] = {}
    for method_name in (
        "is_in_estop",
        "is_in_collision",
        "is_on_limit",
        "is_in_drag_mode",
        "is_in_servomove",
        "is_in_pos",
    ):
        if hasattr(robot, method_name):
            flags[method_name] = _extract_scalar(backend._invoke(method_name))
    return flags


def _check_motion_flags(flags: dict[str, Any]) -> list[str]:
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
    return blockers


class GraspDebugCli:
    def __init__(
        self,
        *,
        arm_config_path: Path,
        hand_config_path: Path,
        speed_scale: float,
        max_delta_mm: float,
        max_rot_deg: float,
    ) -> None:
        self.arm_config_path = arm_config_path
        self.hand_config_path = hand_config_path
        self.speed_scale = speed_scale
        self.max_delta_mm = abs(max_delta_mm)
        self.max_rot_rad = math.radians(abs(max_rot_deg))
        self.hand_config = load_yaml(hand_config_path)
        self.hand_norm = [0.0] * 6
        self.dof_norm = [0.0] * 6

    def print_help(self) -> None:
        print(
            """
Commands:
  pose                         read current joint state and TCP pose
  dz MM | dx MM | dy MM         move TCP by base-frame delta in mm
  r DEG | p DEG | yaw DEG       rotate TCP R/P/Y by degrees around current RPY
  j IDX DEG                    nudge joint IDX(1-6) by degrees
  open                         send RH56 open
  preset NAME                  send RH56 gesture preset, e.g. power_grasp
  hand raw A B C D E F         send raw RH56 angles in raw order
  hand norm I M R P TC TL      send normalized canonical values, 0=open, 1=closed
  dof IDX VALUE                 set physical dof IDX(0-5), 0=open, 1=closed
  dof IDX +DELTA|-DELTA         increment physical dof IDX(0-5)
  dofs V0 V1 V2 V3 V4 V5       set all physical dofs
  finger NAME VALUE            set one normalized finger value
  finger NAME +DELTA|-DELTA    increment one normalized finger value
  handstate                    read RH56 angle feedback
  save NAME                    save current JAKA joint state as a preset
  help                         show this help
  quit                         exit

Raw order comes from config hand_schema.raw_order, usually:
  thumb_close thumb_lateral index middle ring pinky
Canonical normalized order:
  index middle ring pinky thumb_close thumb_lateral
Physical dof order:
  0 pinky_bend, 1 ring_bend, 2 middle_bend, 3 index_bend, 4 thumb_bend, 5 thumb_rotate
""".strip()
        )

    def read_arm(self) -> dict[str, Any]:
        config = _load_arm_config(self.arm_config_path, self.speed_scale)
        adapter = JakaDriverAdapter(config)
        backend = adapter.backend
        result: dict[str, Any] = {}
        try:
            adapter.connect()
            if not isinstance(backend, JakaSDKBackend):
                raise RuntimeError("Expected JAKA SDK backend.")
            result["joint_state"] = adapter.get_joint_state().to_dict()
            if hasattr(backend._robot, "get_actual_tcp_position"):
                result["actual_tcp_pose_mm_rpy"] = _extract_pose_raw(
                    backend.call_sdk_method("get_actual_tcp_position")
                )
            elif hasattr(backend._robot, "get_tcp_position"):
                result["actual_tcp_pose_mm_rpy"] = _extract_pose_raw(
                    backend.call_sdk_method("get_tcp_position")
                )
            result["state_flags"] = _state_flags(backend)
            return result
        finally:
            if isinstance(backend, JakaSDKBackend):
                try:
                    backend.disconnect()
                except Exception as exc:
                    result["disconnect_error"] = str(exc)

    def move_tcp_delta(
        self,
        *,
        dx_mm: float = 0.0,
        dy_mm: float = 0.0,
        dz_mm: float = 0.0,
        dr: float = 0.0,
        dp: float = 0.0,
        dyaw: float = 0.0,
    ) -> dict[str, Any]:
        if max(abs(dx_mm), abs(dy_mm), abs(dz_mm)) > self.max_delta_mm:
            raise ValueError(f"TCP translation step exceeds max_delta_mm={self.max_delta_mm}.")
        if max(abs(dr), abs(dp), abs(dyaw)) > self.max_rot_rad:
            raise ValueError(f"TCP rotation step exceeds max_rot_deg={math.degrees(self.max_rot_rad)}.")

        config = _load_arm_config(self.arm_config_path, self.speed_scale)
        adapter = JakaDriverAdapter(config)
        backend = adapter.backend
        result: dict[str, Any] = {
            "delta_mm": [dx_mm, dy_mm, dz_mm],
            "delta_rpy_rad": [dr, dp, dyaw],
        }
        try:
            adapter.connect()
            if not isinstance(backend, JakaSDKBackend):
                raise RuntimeError("Expected JAKA SDK backend.")
            flags = _state_flags(backend)
            blockers = _check_motion_flags(flags)
            result["state_flags"] = flags
            result["precheck_blockers"] = blockers
            if blockers:
                raise RuntimeError(f"Motion blocked: {blockers}")
            current = _extract_pose_raw(backend.call_sdk_method("get_actual_tcp_position"))
            target = [
                current[0] + dx_mm,
                current[1] + dy_mm,
                current[2] + dz_mm,
                current[3] + dr,
                current[4] + dp,
                current[5] + dyaw,
            ]
            result["current_tcp_pose_mm_rpy"] = current
            result["target_tcp_pose_mm_rpy"] = target
            pose = Pose(
                position=[target[0] / 1000.0, target[1] / 1000.0, target[2] / 1000.0],
                orientation_xyzw=_rpy_to_quat(target[3], target[4], target[5]),
                frame_id="jaka_base",
            )
            result["move_ok"] = adapter.move_pose(pose, blocking=True)
            result["post_tcp_pose_mm_rpy"] = _extract_pose_raw(
                backend.call_sdk_method("get_actual_tcp_position")
            )
            return result
        finally:
            if isinstance(backend, JakaSDKBackend):
                try:
                    backend.disconnect()
                except Exception as exc:
                    result["disconnect_error"] = str(exc)

    def nudge_joint(self, index_one_based: int, delta_deg: float) -> dict[str, Any]:
        if not 1 <= index_one_based <= 6:
            raise ValueError("Joint index must be within 1..6.")
        if abs(math.radians(delta_deg)) > self.max_rot_rad:
            raise ValueError(f"Joint step exceeds max_rot_deg={math.degrees(self.max_rot_rad)}.")
        config = _load_arm_config(self.arm_config_path, self.speed_scale)
        adapter = JakaDriverAdapter(config)
        backend = adapter.backend
        result: dict[str, Any] = {"joint": index_one_based, "delta_deg": delta_deg}
        try:
            adapter.connect()
            if not isinstance(backend, JakaSDKBackend):
                raise RuntimeError("Expected JAKA SDK backend.")
            flags = _state_flags(backend)
            blockers = _check_motion_flags(flags)
            result["state_flags"] = flags
            result["precheck_blockers"] = blockers
            if blockers:
                raise RuntimeError(f"Motion blocked: {blockers}")
            current = adapter.get_joint_state()
            target = [float(v) for v in current.positions]
            target[index_one_based - 1] += math.radians(delta_deg)
            result["current_joint_state"] = current.to_dict()
            result["target_joints"] = target
            result["move_ok"] = adapter.move_joints(target, blocking=True)
            result["post_joint_state"] = adapter.get_joint_state().to_dict()
            return result
        finally:
            if isinstance(backend, JakaSDKBackend):
                try:
                    backend.disconnect()
                except Exception as exc:
                    result["disconnect_error"] = str(exc)

    def _hand_transport_cfg(self) -> dict[str, Any]:
        return self.hand_config.get("jaka_tool_rs485", {})

    def _hand_tcp_client(self) -> JakaTioSignalClient:
        transport = self._hand_transport_cfg()
        feedback = transport.get("state_feedback", {})
        robot_config = load_yaml(transport.get("robot_config_path", "configs/robot/jaka_mini2_real.yaml"))
        host = str(feedback.get("host", robot_config.get("ip", "192.168.71.50")))
        port = int(feedback.get("port", 10001))
        timeout = float(feedback.get("timeout_sec", 0.2))
        terminator_name = str(feedback.get("terminator", "none"))
        terminator = {"none": b"", "lf": b"\n", "crlf": b"\r\n"}.get(terminator_name, b"")
        return JakaTioSignalClient(host=host, port=port, timeout=timeout, terminator=terminator)

    @staticmethod
    def _build_hand_frame(values: list[int], hand_id: int) -> bytes:
        if len(values) != 6:
            raise ValueError("RH56 command vector must have length 6.")
        data_bytes: list[int] = []
        for value in values:
            if not 0 <= int(value) <= 1000:
                raise ValueError(f"RH56 angle value {value} outside [0, 1000].")
            data_bytes.extend([int(value) & 0xFF, (int(value) >> 8) & 0xFF])
        payload = [hand_id, len(data_bytes) + 3, 0x12, REG_ANGLE_SET & 0xFF, (REG_ANGLE_SET >> 8) & 0xFF]
        payload.extend(data_bytes)
        checksum = sum(payload) & 0xFF
        return bytes([0xEB, 0x90] + payload + [checksum])

    def _send_hand_raw(self, raw_values: list[int], mode_name: str = "manual_raw") -> dict[str, Any]:
        result: dict[str, Any] = {"raw_values": raw_values, "mode": mode_name}
        transport = self._hand_transport_cfg()
        channel_id = int(transport.get("channel_id", 1))
        hand_id = int(transport.get("hand_id", self.hand_config.get("serial", {}).get("hand_id", 1)))
        command_channel_mode = int(transport.get("command_channel_mode", 1))
        feedback_channel_mode = int(transport.get("feedback_channel_mode", 0))
        command_pause_sec = float(transport.get("command_pause_sec", 0.8))
        feedback_settle_sec = float(transport.get("feedback_settle_sec", 0.3))
        frame = self._build_hand_frame([int(v) for v in raw_values], hand_id)
        client = self._hand_tcp_client()
        client.set_channel_mode(channel_id, command_channel_mode)
        try:
            client.send_raw_rs485(channel_id, frame)
            if command_pause_sec > 0:
                import time

                time.sleep(command_pause_sec)
        finally:
            client.set_channel_mode(channel_id, feedback_channel_mode)
            if feedback_settle_sec > 0:
                import time

                time.sleep(feedback_settle_sec)
        result["hand_state"] = {"mode": mode_name, "finger_positions": raw_values}
        return result

    def send_hand_preset(self, name: str) -> dict[str, Any]:
        result: dict[str, Any] = {"preset": name}
        gestures = self.hand_config.get("gesture_presets", {})
        if name == "open":
            raw = [int(v) for v in gestures.get("open", [1000] * 6)]
            self.hand_norm = [0.0] * 6
            self.dof_norm = [0.0] * 6
        else:
            preset = gestures.get(name)
            if preset is None:
                raise ValueError(f"Unknown RH56 preset: {name}")
            raw = [int(v) for v in preset]
        result.update(self._send_hand_raw(raw, mode_name=name))
        result["ok"] = True
        return result

    def send_hand_norm(self, values: list[float]) -> dict[str, Any]:
        raw_order = self.hand_config.get("hand_schema", {}).get(
            "raw_order", ["thumb_close", "thumb_lateral", "index", "middle", "ring", "pinky"]
        )
        schema = self.hand_config.get("hand_schema", {})
        calibration = schema.get("dof_calibration", {})
        canonical_raw: dict[str, float] = {}
        self.hand_norm = [float(max(0.0, min(1.0, v))) for v in values]
        for name, value in zip(CANONICAL_HAND_ORDER, self.hand_norm, strict=True):
            item = calibration.get(name, {})
            raw_open = float(item.get("raw_open", 1000))
            raw_close = float(item.get("raw_close", 0))
            safe_min = float(item.get("safe_min", min(raw_open, raw_close)))
            safe_max = float(item.get("safe_max", max(raw_open, raw_close)))
            raw_value = raw_open + value * (raw_close - raw_open)
            canonical_raw[name] = max(safe_min, min(safe_max, raw_value))
        raw = [canonical_raw[name] for name in raw_order]
        return self._send_hand_raw([int(round(v)) for v in raw], mode_name="manual_norm")

    def send_dof_norm(self, values: list[float]) -> dict[str, Any]:
        self.dof_norm = [float(max(0.0, min(1.0, v))) for v in values]
        raw = [int(round(1000.0 * (1.0 - value))) for value in self.dof_norm]
        result = self._send_hand_raw(raw, mode_name="manual_dof_norm")
        result["physical_dof_labels"] = list(PHYSICAL_DOF_LABELS)
        result["physical_dof_norm"] = list(self.dof_norm)
        return result

    def read_hand_state(self) -> dict[str, Any]:
        client = self._hand_tcp_client()
        return {"hand_signal_values": client.get_signal_values()}

    def save_current_preset(self, name: str) -> dict[str, Any]:
        arm_state = self.read_arm()
        joints = [float(v) for v in arm_state["joint_state"]["positions"]]
        config = load_yaml(self.arm_config_path)
        previous = config.setdefault("joint_presets", {}).get(name)
        config["joint_presets"][name] = joints
        with self.arm_config_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle, sort_keys=False, allow_unicode=True)
        return {"preset": name, "previous": previous, "saved_joints": joints}

    def handle(self, line: str) -> bool:
        parts = shlex.split(line)
        if not parts:
            return True
        cmd = parts[0].lower()
        if cmd in {"quit", "exit", "q"}:
            return False
        if cmd == "help":
            self.print_help()
        elif cmd == "pose":
            print(json.dumps(self.read_arm(), indent=2, ensure_ascii=False))
        elif cmd in {"dx", "dy", "dz"} and len(parts) == 2:
            value = float(parts[1])
            kwargs = {f"{cmd}_mm": value}
            print(json.dumps(self.move_tcp_delta(**kwargs), indent=2, ensure_ascii=False))
        elif cmd in {"r", "p", "yaw"} and len(parts) == 2:
            value = math.radians(float(parts[1]))
            key = {"r": "dr", "p": "dp", "yaw": "dyaw"}[cmd]
            print(json.dumps(self.move_tcp_delta(**{key: value}), indent=2, ensure_ascii=False))
        elif cmd == "j" and len(parts) == 3:
            print(json.dumps(self.nudge_joint(int(parts[1]), float(parts[2])), indent=2, ensure_ascii=False))
        elif cmd == "open":
            print(json.dumps(self.send_hand_preset("open"), indent=2, ensure_ascii=False))
        elif cmd == "preset" and len(parts) == 2:
            print(json.dumps(self.send_hand_preset(parts[1]), indent=2, ensure_ascii=False))
        elif cmd == "hand" and len(parts) >= 2:
            mode = parts[1].lower()
            if mode == "raw" and len(parts) == 8:
                values = [int(float(v)) for v in parts[2:]]
                print(json.dumps(self._send_hand_raw(values), indent=2, ensure_ascii=False))
            elif mode == "norm" and len(parts) == 8:
                values = [float(v) for v in parts[2:]]
                print(json.dumps(self.send_hand_norm(values), indent=2, ensure_ascii=False))
            else:
                raise ValueError("Use `hand raw A B C D E F` or `hand norm I M R P TC TL`.")
        elif cmd == "dof" and len(parts) == 3:
            index = int(parts[1])
            if not 0 <= index < 6:
                raise ValueError("dof index must be within 0..5.")
            token = parts[2]
            if token.startswith(("+", "-")):
                self.dof_norm[index] = max(0.0, min(1.0, self.dof_norm[index] + float(token)))
            else:
                self.dof_norm[index] = max(0.0, min(1.0, float(token)))
            print(json.dumps(self.send_dof_norm(self.dof_norm), indent=2, ensure_ascii=False))
        elif cmd == "dofs" and len(parts) == 7:
            values = [float(v) for v in parts[1:]]
            print(json.dumps(self.send_dof_norm(values), indent=2, ensure_ascii=False))
        elif cmd == "finger" and len(parts) == 3:
            name = parts[1]
            if name not in CANONICAL_HAND_ORDER:
                raise ValueError(f"Unknown finger {name!r}; use {list(CANONICAL_HAND_ORDER)}")
            index = list(CANONICAL_HAND_ORDER).index(name)
            token = parts[2]
            if token.startswith(("+", "-")):
                self.hand_norm[index] = max(0.0, min(1.0, self.hand_norm[index] + float(token)))
            else:
                self.hand_norm[index] = max(0.0, min(1.0, float(token)))
            print(json.dumps(self.send_hand_norm(self.hand_norm), indent=2, ensure_ascii=False))
        elif cmd == "handstate":
            print(json.dumps(self.read_hand_state(), indent=2, ensure_ascii=False))
        elif cmd == "save" and len(parts) == 2:
            print(json.dumps(self.save_current_preset(parts[1]), indent=2, ensure_ascii=False))
        else:
            raise ValueError(f"Unknown command: {line!r}. Type `help`.")
        return True

    def loop(self) -> None:
        print("JAKA + RH56 grasp debug CLI. Type `help` for commands, `quit` to exit.")
        self.print_help()
        while True:
            try:
                line = input("grasp-debug> ")
            except EOFError:
                print()
                return
            try:
                if not self.handle(line):
                    return
            except Exception as exc:
                print(f"ERROR: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive grasp pose debug CLI for JAKA + RH56.")
    parser.add_argument("--arm-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--hand-config", default="configs/hand/rh56_real.yaml")
    parser.add_argument("--speed-scale", type=float, default=0.02)
    parser.add_argument("--max-delta-mm", type=float, default=20.0)
    parser.add_argument("--max-rot-deg", type=float, default=5.0)
    args = parser.parse_args()

    cli = GraspDebugCli(
        arm_config_path=Path(args.arm_config),
        hand_config_path=Path(args.hand_config),
        speed_scale=args.speed_scale,
        max_delta_mm=args.max_delta_mm,
        max_rot_deg=args.max_rot_deg,
    )
    cli.loop()


if __name__ == "__main__":
    main()
