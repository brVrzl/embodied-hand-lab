from __future__ import annotations

import time
from typing import Any

from embodiment_core.logger import get_logger
from embodiment_core.types import HandState

from .hand_schema import RH56_PROTOCOL_ORDER, canonical_to_raw, raw_to_canonical
from .interfaces import HandBackend, HandCommand


class RH56SerialBackend(HandBackend):
    """RS485 backend grounded in the local RH56 protocol reference code."""

    REG = {
        "CLEAR_ERROR": 1004,
        "POS_SET": 1474,
        "ANGLE_SET": 1486,
        "FORCE_SET": 1498,
        "SPEED_SET": 1522,
        "POS_ACT": 1534,
        "ANGLE_ACT": 1546,
        "FORCE_ACT": 1582,
        "CURRENT": 1594,
        "ERROR": 1606,
        "STATUS": 1612,
        "TEMP": 1618,
        "ACTION_SEQ_IDX": 2320,
        "ACTION_SEQ_RUN": 2322,
    }

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.logger = get_logger("RH56SerialBackend")
        self.ser: Any | None = None
        serial_cfg = config.get("serial", {})
        self.port = serial_cfg.get("port", "/dev/ttyUSB0")
        self.baudrate = int(serial_cfg.get("baudrate", 115200))
        self.timeout = float(serial_cfg.get("timeout_sec", 0.2))
        self.hand_id = int(serial_cfg.get("hand_id", 1))
        schema_cfg = config.get("hand_schema", {})
        self.protocol_order = tuple(schema_cfg.get("protocol_order", RH56_PROTOCOL_ORDER))
        self.gesture_order = schema_cfg.get("gesture_order", "canonical")
        self._last_mode = "idle"

    def connect(self) -> bool:
        try:
            import serial  # type: ignore
        except Exception as exc:
            raise RuntimeError("pyserial is required for RH56 serial backend.") from exc

        self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        self.clear_error()
        self.set_canonical_speeds(self.config.get("speed_default", [800] * 6))
        self.set_canonical_forces(self.config.get("force_default", [500] * 6))
        self.logger.info("Connected to RH56 serial backend on %s", self.port)
        return True

    def execute(self, command: HandCommand) -> bool:
        gestures = self.config.get("gesture_presets", {})
        if command.command == "open":
            ok = self.set_command_angles(gestures.get("open", [1000] * 6))
        elif command.command == "close":
            ok = self.set_command_angles(gestures.get("close", [0] * 6))
        elif command.command == "pinch":
            ok = self.set_command_angles(gestures.get("pinch", [400, 400, 500, 200, 200, 500]))
        elif command.command == "preset_grasp":
            preset_name = command.preset_name or "power_grasp"
            preset = gestures.get(preset_name)
            if preset is None:
                raise ValueError(f"Unknown RH56 preset: {preset_name}")
            ok = self.set_command_angles(preset)
        else:
            raise ValueError(f"Unsupported hand command: {command.command}")
        self._last_mode = command.command if command.command != "preset_grasp" else command.preset_name
        return ok

    def read_state(self) -> HandState:
        forces = self.get_canonical_forces()
        return HandState(
            mode=self._last_mode,
            finger_positions=self.get_canonical_angles(),
            finger_currents=self.get_canonical_currents(),
            contact_flags=[value > 0 for value in forces],
            force_estimate=forces,
        )

    def stop(self) -> None:
        self.logger.warning("RH56 serial backend stop requested; no dedicated protocol stop is defined.")

    def close_port(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None
            self.logger.info("Closed RH56 serial port %s", self.port)

    def clear_error(self) -> bool:
        return self.write_register(self.REG["CLEAR_ERROR"], [1])

    def set_speeds(self, values: list[int]) -> bool:
        self._validate_vector(values, 0, 1000)
        return self.write_register(self.REG["SPEED_SET"], self._u16_list_to_bytes(values))

    def set_canonical_speeds(self, values: list[int]) -> bool:
        return self.set_speeds(self._canonical_to_protocol_ints(values))

    def set_forces(self, values: list[int]) -> bool:
        self._validate_vector(values, 0, 1000)
        return self.write_register(self.REG["FORCE_SET"], self._u16_list_to_bytes(values))

    def set_canonical_forces(self, values: list[int]) -> bool:
        return self.set_forces(self._canonical_to_protocol_ints(values))

    def set_angles(self, values: list[int]) -> bool:
        self._validate_vector(values, 0, 1000)
        return self.write_register(self.REG["ANGLE_SET"], self._u16_list_to_bytes(values))

    def set_canonical_angles(self, values: list[int]) -> bool:
        return self.set_angles(self._canonical_to_protocol_ints(values))

    def set_command_angles(self, values: list[int]) -> bool:
        if isinstance(self.gesture_order, str):
            if self.gesture_order == "canonical":
                return self.set_canonical_angles(values)
            if self.gesture_order in {"protocol", "raw"}:
                return self.set_angles(values)
            raise ValueError(f"Unsupported RH56 gesture_order={self.gesture_order!r}")
        return self.set_angles([int(round(value)) for value in canonical_to_raw(values, raw_order=self.gesture_order)])

    def get_angles(self) -> list[float]:
        return [float(v) for v in self._u16_list_from_bytes(self.read_register(self.REG["ANGLE_ACT"], 12), 6)]

    def get_canonical_angles(self) -> list[float]:
        return raw_to_canonical(self.get_angles(), raw_order=self.protocol_order)

    def get_forces(self) -> list[float]:
        return [float(v) for v in self._u16_list_from_bytes(self.read_register(self.REG["FORCE_ACT"], 12), 6)]

    def get_canonical_forces(self) -> list[float]:
        return raw_to_canonical(self.get_forces(), raw_order=self.protocol_order)

    def get_currents(self) -> list[float]:
        return [float(v) for v in self._u16_list_from_bytes(self.read_register(self.REG["CURRENT"], 12), 6)]

    def get_canonical_currents(self) -> list[float]:
        return raw_to_canonical(self.get_currents(), raw_order=self.protocol_order)

    def read_register(self, address: int, length: int) -> list[int]:
        payload = [self.hand_id, 0x04, 0x11, address & 0xFF, (address >> 8) & 0xFF, length]
        frames = self._exchange(payload, expected_frames=1)
        if not frames:
            raise RuntimeError(f"RH56 read_register timeout at address {address}.")
        frame = frames[0]
        if not self._validate_checksum(frame):
            raise RuntimeError("RH56 read_register checksum failure.")
        reg_len = frame[3] - 3
        return list(frame[7 : 7 + reg_len])

    def write_register(self, address: int, data_bytes: list[int]) -> bool:
        payload = [self.hand_id, len(data_bytes) + 3, 0x12, address & 0xFF, (address >> 8) & 0xFF] + data_bytes
        frames = self._exchange(payload, expected_frames=1)
        if not frames:
            raise RuntimeError(f"RH56 write_register timeout at address {address}.")
        frame = frames[0]
        return self._validate_checksum(frame) and frame[4] == 0x12

    def _exchange(self, payload: list[int], expected_frames: int) -> list[bytes]:
        if self.ser is None:
            raise RuntimeError("RH56 serial backend is not connected.")
        frame = self._build_frame(payload)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        time.sleep(0.005)
        deadline = time.time() + max(self.timeout * 4.0, 0.1)
        buffer = bytearray()
        frames: list[bytes] = []
        while time.time() < deadline:
            chunk = self.ser.read(64)
            if chunk:
                buffer.extend(chunk)
                frames = self._split_frames(bytes(buffer))
                if len(frames) >= expected_frames:
                    return frames
            else:
                time.sleep(0.001)
        return frames

    @staticmethod
    def _build_frame(payload: list[int]) -> bytes:
        checksum = sum(payload) & 0xFF
        return bytes([0xEB, 0x90] + payload + [checksum])

    @staticmethod
    def _split_frames(buffer: bytes) -> list[bytes]:
        frames: list[bytes] = []
        i = 0
        while i + 5 <= len(buffer):
            if buffer[i] != 0x90 or buffer[i + 1] != 0xEB:
                i += 1
                continue
            data_len = buffer[i + 3]
            total_len = data_len + 5
            if i + total_len > len(buffer):
                break
            frames.append(buffer[i : i + total_len])
            i += total_len
        return frames

    @staticmethod
    def _validate_checksum(frame: bytes) -> bool:
        return (sum(frame[2:-1]) & 0xFF) == frame[-1]

    @staticmethod
    def _u16_list_to_bytes(values: list[int]) -> list[int]:
        output: list[int] = []
        for value in values:
            output.append(value & 0xFF)
            output.append((value >> 8) & 0xFF)
        return output

    @staticmethod
    def _u16_list_from_bytes(data: list[int], count: int) -> list[int]:
        if len(data) < 2 * count:
            raise RuntimeError("Insufficient RH56 response payload length.")
        values: list[int] = []
        for index in range(count):
            lo = data[2 * index] & 0xFF
            hi = data[2 * index + 1] & 0xFF
            values.append(lo | (hi << 8))
        return values

    def _canonical_to_protocol_ints(self, values: list[int]) -> list[int]:
        return [int(round(value)) for value in canonical_to_raw(values, raw_order=self.protocol_order)]

    @staticmethod
    def _validate_vector(values: list[int], min_value: int, max_value: int) -> None:
        if len(values) != 6:
            raise ValueError("RH56 command vector must have length 6.")
        for value in values:
            if not (min_value <= value <= max_value):
                raise ValueError(f"RH56 value {value} outside [{min_value}, {max_value}].")


class RH56Ros2ServiceBackend(HandBackend):
    """Official ROS2 service naming shim.

    The service names are grounded in the local vendor workspace, but the concrete client
    implementation depends on the target ROS2 environment and generated service packages.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config

    def connect(self) -> bool:
        raise NotImplementedError(
            "此处为待替换适配点: RH56 ROS2 service backend requires the vendor service_interfaces package in the active ROS2 workspace."
        )

    def execute(self, command: HandCommand) -> bool:
        raise NotImplementedError

    def read_state(self) -> HandState:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError
