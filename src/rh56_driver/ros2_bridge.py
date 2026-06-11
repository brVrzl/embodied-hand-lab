from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from embodiment_core.types import HandState

from .hand_schema import CANONICAL_HAND_ORDER
from .interfaces import HandBackend, HandCommand
from .node import RH56Driver

STATE_SCHEMA_VERSION = "rh56_ros2_json_state_v0.1"
COMMAND_SCHEMA_VERSION = "rh56_ros2_json_command_v0.1"
DEFAULT_STATE_TOPIC = "/hand/state"
DEFAULT_RAW_FEEDBACK_TOPIC = "/hand/raw_feedback"
DEFAULT_BACKEND_MODE_TOPIC = "/hand/backend_mode"
DEFAULT_COMMAND_ANGLES_TOPIC = "/hand/command_angles"
DEFAULT_COMMAND_CODE_TOPIC = "/hand/command_code"
DEFAULT_COMMAND_FORCE_TOPIC = "/hand/command_force"


@dataclass(frozen=True, slots=True)
class HandAngleCommand:
    values: list[float]
    unit: str = "rh56_angle_raw_0_1000"
    order: str = "canonical"


def _as_six_float_list(value: Any, *, field_name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{field_name} must be a list of 6 numeric values.")
    output: list[float] = []
    for item in value:
        if not isinstance(item, (int, float)):
            raise ValueError(f"{field_name} must contain only numeric values.")
        output.append(float(item))
    return output


def _json_payload(message: Any) -> dict[str, Any]:
    if isinstance(message, str):
        payload = json.loads(message)
    elif hasattr(message, "data"):
        payload = json.loads(str(message.data))
    elif isinstance(message, dict):
        payload = dict(message)
    elif isinstance(message, list):
        payload = {"values": message}
    else:
        raise TypeError(f"Unsupported JSON command payload type: {type(message)!r}")
    if not isinstance(payload, dict):
        raise ValueError("JSON command payload must decode to an object or a list.")
    return payload


def build_state_payload(
    state: HandState,
    *,
    backend_mode: str,
    timestamp: float | None = None,
    position_unit: str = "backend_native",
    raw_feedback: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": STATE_SCHEMA_VERSION,
        "timestamp": time.time() if timestamp is None else float(timestamp),
        "backend_mode": backend_mode,
        "canonical_hand_order": list(CANONICAL_HAND_ORDER),
        "hand": {
            "mode": state.mode,
            "position": list(state.finger_positions),
            "position_unit": position_unit,
            "current": list(state.finger_currents),
            "current_unit": "backend_native",
            "force_estimate": list(state.force_estimate),
            "force_unit": "backend_native",
            "contact_binary": [bool(value) for value in state.contact_flags],
            "order": list(CANONICAL_HAND_ORDER),
        },
        "raw_feedback": raw_feedback,
    }


def build_raw_feedback_payload(
    backend: HandBackend,
    *,
    timestamp: float | None = None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": STATE_SCHEMA_VERSION,
        "timestamp": time.time() if timestamp is None else float(timestamp),
        "available": False,
    }
    if not all(hasattr(backend, name) for name in ("get_angles", "get_forces", "get_currents", "REG", "read_register")):
        return payload

    try:
        reg = getattr(backend, "REG")
        payload.update(
            {
                "available": True,
                "protocol_order": list(getattr(backend, "protocol_order", [])),
                "angles_raw": backend.get_angles(),  # type: ignore[attr-defined]
                "forces_raw": backend.get_forces(),  # type: ignore[attr-defined]
                "currents_raw": backend.get_currents(),  # type: ignore[attr-defined]
                "status_raw": backend.read_register(reg["STATUS"], 6),  # type: ignore[attr-defined]
                "errors_raw": backend.read_register(reg["ERROR"], 6),  # type: ignore[attr-defined]
                "temps_raw": backend.read_register(reg["TEMP"], 6),  # type: ignore[attr-defined]
            }
        )
    except Exception as exc:
        payload["error"] = str(exc)
    return payload


def parse_angle_command(message: Any) -> HandAngleCommand:
    payload = _json_payload(message)
    values = payload.get("values", payload.get("hand_cmd", payload.get("angles")))
    command = HandAngleCommand(
        values=_as_six_float_list(values, field_name="values"),
        unit=str(payload.get("unit", "rh56_angle_raw_0_1000")),
        order=str(payload.get("order", "canonical")),
    )
    if command.order != "canonical":
        raise ValueError("RH56 JSON command topics accept only canonical hand order.")
    if command.unit not in {"rh56_angle_raw_0_1000", "normalized_0_1"}:
        raise ValueError(f"Unsupported hand angle command unit: {command.unit!r}")
    return command


def parse_force_command(message: Any) -> HandAngleCommand:
    payload = _json_payload(message)
    values = payload.get("values", payload.get("force", payload.get("force_limit")))
    command = HandAngleCommand(
        values=_as_six_float_list(values, field_name="values"),
        unit=str(payload.get("unit", "rh56_force_raw_0_1000")),
        order=str(payload.get("order", "canonical")),
    )
    if command.order != "canonical":
        raise ValueError("RH56 JSON force command accepts only canonical hand order.")
    if command.unit != "rh56_force_raw_0_1000":
        raise ValueError(f"Unsupported hand force command unit: {command.unit!r}")
    return command


def parse_code_command(message: Any) -> HandCommand:
    payload = _json_payload(message)
    command = str(payload.get("command", payload.get("preset", ""))).strip()
    if not command:
        raise ValueError("command or preset is required.")
    strength = float(payload.get("strength", payload.get("close_strength", 0.4)))
    if command in {"open", "close", "pinch"}:
        return HandCommand(command=command, strength=strength)
    return HandCommand(command="preset_grasp", preset_name=command, strength=strength)


def _backend_config(backend: HandBackend) -> dict[str, Any]:
    config = getattr(backend, "config", {})
    return dict(config) if isinstance(config, dict) else {}


def _clamp_raw_angle_commands(values: list[int], config: dict[str, Any]) -> list[int]:
    schema = config.get("hand_schema", {})
    safety = config.get("safety", {})
    dof_cfg = schema.get("dof_calibration", {})
    max_close = float(safety.get("max_close_strength", 1.0))
    max_close = max(0.0, min(1.0, max_close))
    clamped: list[int] = []
    for name, value in zip(CANONICAL_HAND_ORDER, values, strict=True):
        cfg = dof_cfg.get(name, {}) if isinstance(dof_cfg, dict) else {}
        raw_open = float(cfg.get("raw_open", 1000.0))
        raw_close = float(cfg.get("raw_close", 0.0))
        safe_min = float(cfg.get("safe_min", min(raw_open, raw_close)))
        safe_max = float(cfg.get("safe_max", max(raw_open, raw_close)))
        max_close_raw = raw_open + (raw_close - raw_open) * max_close
        low = max(safe_min, min(raw_open, max_close_raw))
        high = min(safe_max, max(raw_open, max_close_raw))
        clamped.append(int(round(max(low, min(high, float(value))))))
    return clamped


def _clamp_force_commands(values: list[int], config: dict[str, Any]) -> list[int]:
    schema = config.get("hand_schema", {})
    safety = config.get("safety", {})
    dof_cfg = schema.get("dof_calibration", {})
    default_limit = safety.get("max_force_limit", None)
    clamped: list[int] = []
    for name, value in zip(CANONICAL_HAND_ORDER, values, strict=True):
        cfg = dof_cfg.get(name, {}) if isinstance(dof_cfg, dict) else {}
        limit = cfg.get("default_force_limit", default_limit)
        if default_limit is not None:
            limit = min(float(limit), float(default_limit))
        if limit is None:
            clamped.append(int(round(value)))
        else:
            clamped.append(int(round(max(0.0, min(float(limit), float(value))))))
    return clamped


def apply_angle_command(backend: HandBackend, command: HandAngleCommand) -> bool:
    if command.unit == "normalized_0_1":
        raise ValueError("normalized_0_1 commands require a calibrated policy adapter; send rh56_angle_raw_0_1000 here.")
    values = _clamp_raw_angle_commands(
        [int(round(value)) for value in command.values],
        _backend_config(backend),
    )
    if hasattr(backend, "set_canonical_angles"):
        return bool(backend.set_canonical_angles(values))  # type: ignore[attr-defined]
    raise TypeError("Backend does not support canonical angle commands.")


def apply_force_command(backend: HandBackend, command: HandAngleCommand) -> bool:
    values = _clamp_force_commands(
        [int(round(value)) for value in command.values],
        _backend_config(backend),
    )
    if hasattr(backend, "set_canonical_forces"):
        return bool(backend.set_canonical_forces(values))  # type: ignore[attr-defined]
    raise TypeError("Backend does not support canonical force commands.")


def run_ros2_json_node(
    *,
    config_path: str | Path,
    state_hz: float = 20.0,
    position_unit: str = "backend_native",
) -> None:
    try:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from std_msgs.msg import String  # type: ignore
    except Exception as exc:
        raise RuntimeError("ROS2 Python packages are required. Source ROS2 Humble and install std_msgs first.") from exc

    class RH56JsonNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("rh56_json_bridge")
            self.driver = RH56Driver.from_yaml(config_path)
            self.driver.connect()
            self.backend = self.driver.backend
            self.backend_mode = self.driver.config.get("backend_type", self.driver.config.get("mode", "unknown"))
            self.state_pub = self.create_publisher(String, DEFAULT_STATE_TOPIC, 10)
            self.raw_pub = self.create_publisher(String, DEFAULT_RAW_FEEDBACK_TOPIC, 10)
            self.mode_pub = self.create_publisher(String, DEFAULT_BACKEND_MODE_TOPIC, 10)
            self.create_subscription(String, DEFAULT_COMMAND_ANGLES_TOPIC, self._on_angles, 10)
            self.create_subscription(String, DEFAULT_COMMAND_CODE_TOPIC, self._on_code, 10)
            self.create_subscription(String, DEFAULT_COMMAND_FORCE_TOPIC, self._on_force, 10)
            period = 1.0 / max(float(state_hz), 1e-6)
            self.create_timer(period, self._publish_state)

        def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
            message = String()
            message.data = json.dumps(payload, ensure_ascii=False)
            try:
                publisher.publish(message)
            except Exception:
                if rclpy.ok():
                    raise

        def _publish_state(self) -> None:
            now = time.time()
            raw = build_raw_feedback_payload(self.backend, timestamp=now)
            state = self.driver.read_state()
            self._publish_json(
                self.state_pub,
                build_state_payload(
                    state,
                    backend_mode=str(self.backend_mode),
                    timestamp=now,
                    position_unit=position_unit,
                    raw_feedback=None,
                ),
            )
            self._publish_json(self.raw_pub, raw)
            self._publish_json(
                self.mode_pub,
                {
                    "schema_version": STATE_SCHEMA_VERSION,
                    "timestamp": now,
                    "backend_mode": self.backend_mode,
                },
            )

        def _handle_command(self, callback: Callable[[Any], bool], message: Any, label: str) -> None:
            try:
                callback(message)
            except Exception as exc:
                self.get_logger().error("%s command failed: %s", label, exc)

        def _on_angles(self, message: Any) -> None:
            self._handle_command(lambda msg: apply_angle_command(self.backend, parse_angle_command(msg)), message, "angle")

        def _on_code(self, message: Any) -> None:
            self._handle_command(lambda msg: self.backend.execute(parse_code_command(msg)), message, "code")

        def _on_force(self, message: Any) -> None:
            self._handle_command(lambda msg: apply_force_command(self.backend, parse_force_command(msg)), message, "force")

    rclpy.init()
    node = RH56JsonNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Run the RH56 JSON ROS2 bridge.")
    parser.add_argument("--config", default="configs/hand/rh56.yaml")
    parser.add_argument("--state-hz", type=float, default=20.0)
    parser.add_argument("--position-unit", default="backend_native")
    args = parser.parse_args(argv)
    run_ros2_json_node(config_path=args.config, state_hz=args.state_hz, position_unit=args.position_unit)


if __name__ == "__main__":
    main()
