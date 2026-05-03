from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from embodiment_core.logger import get_logger
from embodiment_core.types import HandState
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend

from .interfaces import HandBackend, HandCommand
from .jaka_tio_signal_client import JakaTioSignalClient


class RH56JakaToolBackend(HandBackend):
    """RH56 backend using JAKA tool-end RS485 passthrough.

    This backend is real-hardware oriented and grounded in the local JAKA SDK:
    - `set_tio_vout_param`
    - `set_tio_pin_mode`
    - `set_rs485_chn_mode`
    - `set_rs485_chn_comm`
    - `send_tio_rs_command`

    State feedback can be read through JAKA TCP/IP TIO RS485 semaphores. The
    controller supports only a small number of semaphores, so the production
    profile keeps six `rh56_angle_*` signals for ANGLE_ACT.
    """

    REG_ANGLE_SET = 1486
    MODBUS_RTU_MODE = 0
    RAW_RS485_MODE = 1
    PIN_TYPE_AI = 2
    PARITY_NONE = 78

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.logger = get_logger("RH56JakaToolBackend")
        self.transport_cfg = config.get("jaka_tool_rs485", {})
        self.channel_id = int(self.transport_cfg.get("channel_id", 1))
        self.hand_id = int(self.transport_cfg.get("hand_id", config.get("serial", {}).get("hand_id", 1)))
        self.command_pause_sec = float(self.transport_cfg.get("command_pause_sec", 0.8))
        self.robot_config_path = Path(self.transport_cfg.get("robot_config_path", "configs/robot/jaka_mini2.yaml"))
        self.jaka_backend = JakaSDKBackend(self._load_robot_config())
        self.state_feedback_cfg = self.transport_cfg.get("state_feedback", {})
        self.state_feedback_enabled = bool(self.state_feedback_cfg.get("enabled", False))
        self.signal_names = list(
            self.state_feedback_cfg.get("angle_signal_names", [f"rh56_angle_{index}" for index in range(6)])
        )
        self.signal_addresses = [
            int(value) for value in self.state_feedback_cfg.get("angle_signal_addresses", [1546 + 2 * index for index in range(6)])
        ]
        self.signal_type = int(self.state_feedback_cfg.get("signal_type", 3))
        self.signal_frequency_hz = float(self.state_feedback_cfg.get("frequency_hz", 0.0))
        self.rebuild_signals_after_command = bool(self.state_feedback_cfg.get("rebuild_after_command", True))
        self.switch_mode_for_commands = bool(self.transport_cfg.get("switch_mode_for_commands", False))
        self.mode_switch_transport = str(self.transport_cfg.get("mode_switch_transport", "tcp"))
        self.command_send_transport = str(
            self.transport_cfg.get(
                "command_send_transport",
                "tcp" if self.mode_switch_transport == "tcp" else "sdk",
            )
        )
        self.command_channel_mode = int(self.transport_cfg.get("command_channel_mode", self.RAW_RS485_MODE))
        self.feedback_channel_mode = int(self.transport_cfg.get("feedback_channel_mode", self.MODBUS_RTU_MODE))
        self.mode_switch_retries = int(self.transport_cfg.get("mode_switch_retries", 5))
        self.mode_switch_retry_sec = float(self.transport_cfg.get("mode_switch_retry_sec", 0.2))
        self.feedback_settle_sec = float(self.transport_cfg.get("feedback_settle_sec", 0.3))
        self.feedback_read_retries = int(self.state_feedback_cfg.get("read_retries", 5))
        self.feedback_read_retry_sec = float(self.state_feedback_cfg.get("read_retry_sec", 0.2))
        self.signal_client: JakaTioSignalClient | None = None
        self._connected = False
        self._last_mode = "idle"
        self._last_angles = [float(v) for v in self.config.get("gesture_presets", {}).get("open", [1000] * 6)]

    def connect(self) -> bool:
        self.jaka_backend.connect()
        self._connected = True
        self._connect_state_feedback()
        self._prepare_transport()
        self.logger.info(
            "Connected RH56 via JAKA tool RS485 channel %s, hand_id=%s",
            self.channel_id,
            self.hand_id,
        )
        return True

    def execute(self, command: HandCommand) -> bool:
        self._require_connected()
        angles, mode_name = self._resolve_command(command)
        frame = self._build_set_angles_frame([int(v) for v in angles])
        if self.switch_mode_for_commands:
            self._set_channel_mode(self.command_channel_mode)
        try:
            self._send_command_frame(frame)
            self._last_mode = mode_name
            self._last_angles = [float(v) for v in angles]
            if self.command_pause_sec > 0.0:
                time.sleep(self.command_pause_sec)
        finally:
            if self.switch_mode_for_commands:
                self._set_channel_mode(self.feedback_channel_mode)
                if self.rebuild_signals_after_command:
                    self._ensure_feedback_signals()
                if self.feedback_settle_sec > 0.0:
                    time.sleep(self.feedback_settle_sec)
        return True

    def read_state(self) -> HandState:
        angles = self._read_feedback_angles()
        if angles is not None:
            self._last_angles = angles
        return HandState(
            mode=self._last_mode,
            finger_positions=list(self._last_angles),
            finger_currents=[],
            contact_flags=[],
            force_estimate=[],
        )

    def stop(self) -> None:
        self.logger.warning("RH56 over JAKA RS485 has no dedicated stop command; issuing open is safer than aborting serial.")

    def disconnect(self) -> None:
        self.jaka_backend.disconnect()
        self._connected = False

    def get_transport_status(self) -> dict[str, Any]:
        self._require_connected()
        status: dict[str, Any] = {
            "channel_id": self.channel_id,
            "hand_id": self.hand_id,
        }
        try:
            status["ai_pin_mode"] = self.jaka_backend.call_sdk_method("get_tio_pin_mode", self.PIN_TYPE_AI)[1]
        except Exception as exc:
            status["ai_pin_mode_error"] = str(exc)
        try:
            status["channel_mode"] = self.jaka_backend.call_sdk_method("get_rs485_chn_mode", self.channel_id)[1]
        except Exception as exc:
            status["channel_mode_error"] = str(exc)
        try:
            status["rs485_comm"] = self.jaka_backend.call_sdk_method("get_rs485_chn_comm")[1]
        except Exception as exc:
            status["rs485_comm_error"] = str(exc)
        return status

    def send_raw_angles(self, values: list[int], mode_name: str = "manual_raw") -> None:
        self._require_connected()
        frame = self._build_set_angles_frame([int(v) for v in values])
        if self.switch_mode_for_commands:
            self._set_channel_mode(self.command_channel_mode)
        try:
            self._send_command_frame(frame)
            self._last_mode = mode_name
            self._last_angles = [float(v) for v in values]
            if self.command_pause_sec > 0.0:
                time.sleep(self.command_pause_sec)
        finally:
            if self.switch_mode_for_commands:
                self._set_channel_mode(self.feedback_channel_mode)
                if self.rebuild_signals_after_command:
                    self._ensure_feedback_signals()
                if self.feedback_settle_sec > 0.0:
                    time.sleep(self.feedback_settle_sec)

    def build_open_frame(self) -> bytes:
        return self._build_set_angles_frame(self.config.get("gesture_presets", {}).get("open", [1000] * 6))

    def build_close_frame(self) -> bytes:
        return self._build_set_angles_frame(self.config.get("gesture_presets", {}).get("close", [0] * 6))

    def _send_command_frame(self, frame: bytes) -> None:
        if self.command_send_transport == "tcp":
            if self.signal_client is None:
                self._connect_state_feedback()
            if self.signal_client is None:
                raise RuntimeError("JAKA TCP command sending requires state_feedback.enabled=true.")
            self.signal_client.send_raw_rs485(self.channel_id, frame)
            return

        result = self.jaka_backend.call_sdk_method("send_tio_rs_command", self.channel_id, bytearray(frame))
        self.jaka_backend.ensure_success(result, "send_tio_rs_command")

    def _load_robot_config(self) -> dict[str, Any]:
        robot_config = load_yaml(self.robot_config_path)
        robot_config["mode"] = "real"
        robot_config["backend_type"] = "jaka_sdk"
        return robot_config

    def _prepare_transport(self) -> None:
        prepare_cfg = self.transport_cfg.get("prepare_tio", {})

        if prepare_cfg.get("enable_vout", False):
            vout_enable = int(prepare_cfg.get("vout_enable", 1))
            vout_vol = int(prepare_cfg.get("vout_vol", 0))
            result = self.jaka_backend.call_sdk_method("set_tio_vout_param", vout_enable, vout_vol)
            self.jaka_backend.ensure_success(result, "set_tio_vout_param")

        if prepare_cfg.get("set_ai_pin_mode", False):
            ai_pin_mode = int(prepare_cfg.get("ai_pin_mode", 1))
            result = self.jaka_backend.call_sdk_method("set_tio_pin_mode", self.PIN_TYPE_AI, ai_pin_mode)
            self.jaka_backend.ensure_success(result, "set_tio_pin_mode")

        if prepare_cfg.get("set_channel_mode", False):
            channel_mode = int(prepare_cfg.get("channel_mode", self.RAW_RS485_MODE))
            self._set_channel_mode(channel_mode)

        if prepare_cfg.get("set_comm", False):
            channel_mode = int(prepare_cfg.get("channel_mode", self.RAW_RS485_MODE))
            if channel_mode == self.RAW_RS485_MODE:
                self.logger.info("Skipping set_rs485_chn_comm because channel_mode=RAW_RS485.")
                return
            comm_cfg = dict(prepare_cfg.get("comm", {}))
            comm_cfg.setdefault("chn_id", self.channel_id)
            comm_cfg.setdefault("slave_id", self.hand_id)
            comm_cfg.setdefault("baudrate", 115200)
            comm_cfg.setdefault("databit", 8)
            comm_cfg.setdefault("stopbit", 1)
            comm_cfg.setdefault("parity", self.PARITY_NONE)
            result = self.jaka_backend.call_sdk_method("set_rs485_chn_comm", comm_cfg)
            self.jaka_backend.ensure_success(result, "set_rs485_chn_comm")

    def _connect_state_feedback(self) -> None:
        if not self.state_feedback_enabled:
            return
        robot_config = self.jaka_backend.config
        host = str(self.state_feedback_cfg.get("host", robot_config.get("ip", "192.168.71.50")))
        port = int(self.state_feedback_cfg.get("port", 10001))
        timeout = float(self.state_feedback_cfg.get("timeout_sec", 0.2))
        terminator_name = str(self.state_feedback_cfg.get("terminator", "none"))
        terminator = {"none": b"", "lf": b"\n", "crlf": b"\r\n"}.get(terminator_name, b"")
        self.signal_client = JakaTioSignalClient(host=host, port=port, timeout=timeout, terminator=terminator)

    def _read_feedback_angles(self) -> list[float] | None:
        if not self.state_feedback_enabled or self.signal_client is None:
            return None
        try:
            values = self.signal_client.get_signal_values()
        except Exception as exc:
            self.logger.warning("Failed to read RH56 TIO signal feedback: %s", exc)
            return None
        missing = [name for name in self.signal_names if name not in values]
        if not missing:
            return [values[name] for name in self.signal_names]

        for _ in range(max(self.feedback_read_retries - 1, 0)):
            time.sleep(self.feedback_read_retry_sec)
            try:
                values = self.signal_client.get_signal_values()
            except Exception as exc:
                self.logger.warning("Failed to read RH56 TIO signal feedback: %s", exc)
                return None
            missing = [name for name in self.signal_names if name not in values]
            if not missing:
                return [values[name] for name in self.signal_names]

        self.logger.warning("Missing RH56 TIO angle feedback signals: %s", missing)
        return None

    def _ensure_feedback_signals(self) -> None:
        if self.signal_client is None:
            return
        if len(self.signal_names) != len(self.signal_addresses):
            raise RuntimeError("RH56 angle signal names and addresses must have the same length.")
        for name, address in zip(self.signal_names, self.signal_addresses):
            self.signal_client.add_signal(
                name=name,
                channel_id=self.channel_id,
                signal_type=self.signal_type,
                address=address,
                frequency_hz=self.signal_frequency_hz,
            )

    def _set_channel_mode(self, channel_mode: int) -> None:
        last_error: Exception | None = None
        for attempt in range(max(self.mode_switch_retries, 1)):
            try:
                if self.mode_switch_transport == "tcp":
                    if self.signal_client is None:
                        self._connect_state_feedback()
                    if self.signal_client is None:
                        raise RuntimeError("JAKA TCP mode switching requires state_feedback.enabled=true.")
                    self.signal_client.set_channel_mode(self.channel_id, channel_mode)
                else:
                    result = self.jaka_backend.call_sdk_method("set_rs485_chn_mode", self.channel_id, channel_mode)
                    self.jaka_backend.ensure_success(result, "set_rs485_chn_mode")
                return
            except RuntimeError as exc:
                last_error = exc
                if attempt + 1 >= max(self.mode_switch_retries, 1):
                    break
                time.sleep(self.mode_switch_retry_sec)
        if last_error is not None:
            raise last_error

    def _resolve_command(self, command: HandCommand) -> tuple[list[int], str]:
        gestures = self.config.get("gesture_presets", {})
        if command.command == "open":
            return [int(v) for v in gestures.get("open", [1000] * 6)], "open"
        if command.command == "close":
            return [int(v) for v in gestures.get("close", [0] * 6)], "close"
        if command.command == "pinch":
            return [int(v) for v in gestures.get("pinch", [400, 400, 500, 200, 200, 500])], "pinch"
        if command.command == "preset_grasp":
            preset_name = command.preset_name or "power_grasp"
            preset = gestures.get(preset_name)
            if preset is None:
                raise ValueError(f"Unknown RH56 preset: {preset_name}")
            return [int(v) for v in preset], preset_name
        raise ValueError(f"Unsupported hand command: {command.command}")

    def _build_set_angles_frame(self, values: list[int]) -> bytes:
        self._validate_vector(values)
        data_bytes = self._u16_list_to_bytes(values)
        payload = [self.hand_id, len(data_bytes) + 3, 0x12, self.REG_ANGLE_SET & 0xFF, (self.REG_ANGLE_SET >> 8) & 0xFF]
        payload.extend(data_bytes)
        return self._build_frame(payload)

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("RH56 JAKA tool backend is not connected.")

    @staticmethod
    def _build_frame(payload: list[int]) -> bytes:
        checksum = sum(payload) & 0xFF
        return bytes([0xEB, 0x90] + payload + [checksum])

    @staticmethod
    def _u16_list_to_bytes(values: list[int]) -> list[int]:
        output: list[int] = []
        for value in values:
            output.append(value & 0xFF)
            output.append((value >> 8) & 0xFF)
        return output

    @staticmethod
    def _validate_vector(values: list[int]) -> None:
        if len(values) != 6:
            raise ValueError("RH56 command vector must have length 6.")
        for value in values:
            if not (0 <= int(value) <= 1000):
                raise ValueError(f"RH56 angle value {value} outside [0, 1000].")
