from __future__ import annotations

import grp
import os
import subprocess
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence

import numpy as np

from .hand_schema import (
    CANONICAL_HAND_ORDER,
    DEFAULT_HAND_DELTA_LIMIT,
    DEFAULT_RH56_CALIBRATION,
    HandDofCalibration,
    denormalize_canonical,
    normalize_raw,
    raw_to_canonical,
)

RH56_READ_ONLY_APPROVAL = "I_AUTHORIZE_ONE_RH56_PC_DIRECT_READ_ONLY_PROBE"
RH56_HAND_ONLY_COMMAND_APPROVAL = "I_AUTHORIZE_ONE_RH56_PC_DIRECT_BOUNDED_HAND_TEST"
RH56_COMBINED_RUN_APPROVAL = "I_AUTHORIZE_ONE_JAKA_RH56_PC_DIRECT_COMBINED_RUN"
RH56_RUNTIME_CONFIG_APPROVAL = "I_AUTHORIZE_ONE_RH56_PC_DIRECT_RUNTIME_CONFIG_WRITE"

RH56_PC_DIRECT_SCHEMA_VERSION = "rh56_pc_direct_episode.v1"
RH56_SPEED_MIN = 0
RH56_SPEED_MAX = 1000
RH56_FORCE_MIN = 0
RH56_FORCE_MAX = 1000


class HandControlState(str, Enum):
    DISABLED = "HAND_DISABLED"
    HOLD = "HAND_HOLD"
    ACTIVE = "HAND_ACTIVE"
    FAULT = "HAND_FAULT"


class HandAuthorization(str, Enum):
    READ_ONLY = "read_only_probe"
    HAND_ONLY_COMMAND = "hand_only_command_test"
    COMBINED_RUN = "arm_hand_combined_run"
    RUNTIME_CONFIG = "runtime_config_write"


_APPROVALS = {
    RH56_READ_ONLY_APPROVAL: HandAuthorization.READ_ONLY,
    RH56_HAND_ONLY_COMMAND_APPROVAL: HandAuthorization.HAND_ONLY_COMMAND,
    RH56_COMBINED_RUN_APPROVAL: HandAuthorization.COMBINED_RUN,
    RH56_RUNTIME_CONFIG_APPROVAL: HandAuthorization.RUNTIME_CONFIG,
}


def parse_rh56_approval(token: str) -> HandAuthorization:
    try:
        return _APPROVALS[token]
    except KeyError as exc:
        raise ValueError("Missing or incorrect RH56 PC-direct approval token.") from exc


def require_serial_by_id_path(
    device: str,
    *,
    require_exists: bool = True,
    allow_direct_ch341: bool = False,
) -> Path:
    path = Path(device)
    stable_by_id = (
        str(path).startswith("/dev/serial/by-id/")
        and path.parent == Path("/dev/serial/by-id")
    )
    direct_ch341 = (
        allow_direct_ch341
        and path.parent == Path("/dev")
        and path.name.startswith("ttyCH341USB")
        and path.name.removeprefix("ttyCH341USB").isdigit()
    )
    if not (stable_by_id or direct_ch341):
        raise ValueError(
            "RH56 device must use /dev/serial/by-id/...; the explicitly "
            "acknowledged fallback accepts only /dev/ttyCH341USB<N>."
        )
    if require_exists:
        if stable_by_id and not path.is_symlink():
            raise ValueError(f"RH56 by-id device is missing or not a symlink: {path}")
        resolved = path.resolve(strict=True)
        if not str(resolved).startswith("/dev/"):
            raise ValueError(f"RH56 by-id device resolves outside /dev: {resolved}")
    return path


def inspect_serial_device(
    device: str,
    *,
    allow_direct_ch341: bool = False,
) -> dict[str, Any]:
    path = require_serial_by_id_path(
        device,
        allow_direct_ch341=allow_direct_ch341,
    )
    resolved = path.resolve(strict=True)
    completed = subprocess.run(
        ["udevadm", "info", "--query=property", f"--name={resolved}"],
        check=False,
        capture_output=True,
        text=True,
    )
    properties: dict[str, str] = {}
    if completed.returncode == 0:
        for line in completed.stdout.splitlines():
            key, separator, value = line.partition("=")
            if separator:
                properties[key] = value
    stat_result = resolved.stat()
    occupied_pids: list[int] = []
    fuser = subprocess.run(
        ["fuser", str(resolved)],
        check=False,
        capture_output=True,
        text=True,
    )
    if fuser.returncode == 0:
        occupied_pids = [
            int(value)
            for value in fuser.stdout.split()
            if value.isdigit()
        ]
    return {
        "requested_device": str(path),
        "requested_by_id": str(path),
        "resolved_tty": str(resolved),
        "usb_vid": properties.get("ID_VENDOR_ID"),
        "usb_pid": properties.get("ID_MODEL_ID"),
        "usb_serial": properties.get("ID_SERIAL_SHORT"),
        "usb_identity": properties.get("ID_SERIAL"),
        "usb_vendor": properties.get("ID_VENDOR_FROM_DATABASE", properties.get("ID_VENDOR")),
        "usb_model": properties.get("ID_MODEL_FROM_DATABASE", properties.get("ID_MODEL")),
        "usb_driver": properties.get("ID_USB_DRIVER"),
        "device_owner_uid": stat_result.st_uid,
        "device_group_gid": stat_result.st_gid,
        "device_group": grp.getgrgid(stat_result.st_gid).gr_name,
        "current_user_can_read": os.access(resolved, os.R_OK),
        "current_user_can_write": os.access(resolved, os.W_OK),
        "occupied_pids": occupied_pids,
        "fuser_check_available": fuser.returncode in {0, 1},
        "udevadm_ok": completed.returncode == 0,
        "udevadm_error": completed.stderr.strip() if completed.returncode else None,
    }


class PcDirectBackend(Protocol):
    REG: Mapping[str, int]

    def connect(self) -> bool: ...

    def close_port(self) -> None: ...

    def get_canonical_angles(self) -> list[float]: ...

    def get_canonical_currents(self) -> list[float]: ...

    def get_canonical_forces(self) -> list[float]: ...

    def read_register(self, address: int, length: int) -> list[int]: ...

    def set_canonical_angles(self, values: list[int]) -> bool: ...

    def set_canonical_speeds(self, values: list[int]) -> bool: ...

    def set_canonical_forces(self, values: list[int]) -> bool: ...


@dataclass(frozen=True, slots=True)
class PcDirectFeedback:
    monotonic_ns: int
    position_raw: tuple[float, ...]
    position_normalized: tuple[float, ...]
    current_raw_count: tuple[float, ...]
    load_or_force_raw_count: tuple[float, ...]
    status: tuple[int, ...]
    error: tuple[int, ...]
    read_latency_ms: float
    state_source: str = "rh56_angle_act_register"


def _calibration_from_config(config: Mapping[str, Any]) -> dict[str, HandDofCalibration]:
    rows = config.get("hand_schema", {}).get("dof_calibration")
    if not rows:
        return dict(DEFAULT_RH56_CALIBRATION)
    calibration: dict[str, HandDofCalibration] = {}
    for name in CANONICAL_HAND_ORDER:
        row = rows[name]
        calibration[name] = HandDofCalibration(
            raw_open=float(row["raw_open"]),
            raw_close=float(row["raw_close"]),
            direction_sign=int(row["direction_sign"]),
            safe_min=float(row["safe_min"]),
            safe_max=float(row["safe_max"]),
            default_speed=float(row["default_speed"]),
            default_force_limit=float(row["default_force_limit"]),
        )
    return calibration


class RH56PcDirectControl:
    """Authorized PC-direct hand control, independent from the JAKA arm path."""

    def __init__(self, backend: PcDirectBackend, config: Mapping[str, Any]) -> None:
        self.backend = backend
        self.config = config
        self.state = HandControlState.DISABLED
        self.transport_state = "CLOSED"
        self.authorization: HandAuthorization | None = None
        self.control_frequency_hz = float(config.get("control_frequency_hz", 15.0))
        if self.control_frequency_hz <= 0.0:
            raise ValueError("RH56 control_frequency_hz must be positive.")
        self.command_period_ns = int(round(1e9 / self.control_frequency_hz))
        serial_timeout_sec = float(config.get("serial", {}).get("timeout_sec", 0.2))
        self.feedback_stale_timeout_ns = int(
            round(float(config.get("feedback_stale_timeout_sec", 2.0 * serial_timeout_sec)) * 1e9)
        )
        self.delta_limit = float(
            config.get("hand_schema", {}).get("hand_delta_limit", DEFAULT_HAND_DELTA_LIMIT)
        )
        self.max_close = float(config.get("safety", {}).get("max_close_strength", 1.0))
        self.calibration = _calibration_from_config(config)
        self.last_feedback: PcDirectFeedback | None = None
        self.last_command_normalized: tuple[float, ...] | None = None
        self.last_command_raw: tuple[int, ...] | None = None
        self.last_command_monotonic_ns: int | None = None
        self.next_command_monotonic_ns: int | None = None
        self.fault_reason: str | None = None

    def open(self, approval_token: str) -> None:
        authorization = parse_rh56_approval(approval_token)
        try:
            connected = self.backend.connect()
        except Exception:
            self._fault("serial_connect_failure")
            raise
        if not connected:
            self._fault("serial_connect_failure")
            raise RuntimeError("RH56 PC-direct backend did not connect.")
        self.authorization = authorization
        self.transport_state = "CONNECTED_READ_ONLY"
        self.state = HandControlState.HOLD

    def poll_feedback(self, monotonic_ns: int) -> PcDirectFeedback:
        self._require_open()
        started_ns = time.perf_counter_ns()
        try:
            position = tuple(self.backend.get_canonical_angles())
            currents = tuple(self.backend.get_canonical_currents())
            loads = tuple(self.backend.get_canonical_forces())
            status = tuple(
                int(value)
                for value in raw_to_canonical(
                    self.backend.read_register(self.backend.REG["STATUS"], 6),
                    raw_order=self.config.get("hand_schema", {}).get(
                        "protocol_order", CANONICAL_HAND_ORDER
                    ),
                )
            )
            errors = tuple(
                int(value)
                for value in raw_to_canonical(
                    self.backend.read_register(self.backend.REG["ERROR"], 6),
                    raw_order=self.config.get("hand_schema", {}).get(
                        "protocol_order", CANONICAL_HAND_ORDER
                    ),
                )
            )
        except Exception:
            self._fault("serial_feedback_failure")
            raise
        normalized = tuple(
            normalize_raw(
                position,
                raw_order=CANONICAL_HAND_ORDER,
                calibration=self.calibration,
            )
        )
        read_latency_ms = (time.perf_counter_ns() - started_ns) / 1e6
        feedback = PcDirectFeedback(
            monotonic_ns=int(monotonic_ns),
            position_raw=position,
            position_normalized=normalized,
            current_raw_count=currents,
            load_or_force_raw_count=loads,
            status=status,
            error=errors,
            read_latency_ms=read_latency_ms,
        )
        self.last_feedback = feedback
        if any(errors):
            self._fault("device_error_status")
            raise RuntimeError(f"RH56 device error registers are nonzero: {list(errors)}")
        if read_latency_ms * 1e6 > self.feedback_stale_timeout_ns:
            self._fault("feedback_stale_during_read")
            raise RuntimeError(
                f"RH56 feedback read took {read_latency_ms:.3f} ms, exceeding the stale boundary."
            )
        if self.last_command_normalized is None:
            self.last_command_normalized = normalized
            self.last_command_raw = tuple(int(round(value)) for value in position)
        return feedback

    def activate(self, monotonic_ns: int) -> None:
        if self.authorization not in {
            HandAuthorization.HAND_ONLY_COMMAND,
            HandAuthorization.COMBINED_RUN,
        }:
            raise PermissionError("This RH56 authorization does not allow position commands.")
        self._require_fresh_feedback(monotonic_ns)
        if self.state is HandControlState.FAULT:
            raise RuntimeError(f"RH56 is faulted: {self.fault_reason}")
        # Every new grip engagement starts from fresh measured ANGLE_ACT.
        # This prevents a held target from a previous clutch cycle/session
        # becoming the first command of the new engagement.
        assert self.last_feedback is not None
        self.last_command_normalized = self.last_feedback.position_normalized
        self.last_command_raw = tuple(
            int(round(value)) for value in self.last_feedback.position_raw
        )
        self.state = HandControlState.ACTIVE
        self.transport_state = "CONNECTED_COMMAND_AUTHORIZED"
        self.next_command_monotonic_ns = int(monotonic_ns)

    def command(
        self,
        target_normalized: Sequence[float],
        monotonic_ns: int,
        *,
        grip_fresh: bool = True,
        arm_terminal_stop: bool = False,
    ) -> bool:
        if arm_terminal_stop:
            self._fault("arm_terminal_hard_stop")
            return False
        if not grip_fresh:
            self.hold("grip_stale")
            return False
        if self.state is not HandControlState.ACTIVE:
            return False
        self._require_fresh_feedback(monotonic_ns)
        if self.state is HandControlState.FAULT:
            return False
        if self.next_command_monotonic_ns is not None and monotonic_ns < self.next_command_monotonic_ns:
            return False
        requested = np.asarray(target_normalized, dtype=np.float64).reshape(-1)
        if requested.size != len(CANONICAL_HAND_ORDER):
            raise ValueError("RH56 normalized target must have six canonical channels.")
        if not np.all(np.isfinite(requested)) or np.any(requested < 0.0) or np.any(requested > self.max_close):
            raise ValueError(f"RH56 normalized target must remain within [0, {self.max_close}].")
        assert self.last_command_normalized is not None
        previous = np.asarray(self.last_command_normalized, dtype=np.float64)
        selected = previous + np.clip(requested - previous, -self.delta_limit, self.delta_limit)
        raw = denormalize_canonical(
            selected,
            raw_order=CANONICAL_HAND_ORDER,
            calibration=self.calibration,
        )
        raw_int = [int(round(value)) for value in raw]
        try:
            ok = self.backend.set_canonical_angles(raw_int)
        except Exception:
            self._fault("serial_command_failure")
            raise
        if not ok:
            self._fault("serial_command_rejected")
            raise RuntimeError("RH56 rejected the position command.")
        self.last_command_normalized = tuple(float(value) for value in selected)
        self.last_command_raw = tuple(raw_int)
        self.last_command_monotonic_ns = int(monotonic_ns)
        self.next_command_monotonic_ns = int(monotonic_ns) + self.command_period_ns
        return True

    def write_runtime_config(
        self,
        speeds: Sequence[int],
        forces: Sequence[int],
    ) -> None:
        if self.authorization is not HandAuthorization.RUNTIME_CONFIG:
            raise PermissionError("Independent RH56 runtime-config authorization is required.")
        speed_values = self._validated_int_vector(speeds, RH56_SPEED_MIN, RH56_SPEED_MAX, "speed")
        force_values = self._validated_int_vector(forces, RH56_FORCE_MIN, RH56_FORCE_MAX, "force")
        try:
            if not self.backend.set_canonical_speeds(speed_values):
                raise RuntimeError("RH56 rejected runtime speed configuration.")
            if not self.backend.set_canonical_forces(force_values):
                raise RuntimeError("RH56 rejected runtime force configuration.")
        except Exception:
            self._fault("runtime_config_write_failure")
            raise

    def hold(self, reason: str) -> None:
        if self.state is HandControlState.FAULT:
            return
        self.state = HandControlState.HOLD
        self.transport_state = f"CONNECTED_HOLD:{reason}"
        self.next_command_monotonic_ns = None

    def arm_terminal_stop(self, reason: str) -> None:
        self._fault(f"arm_terminal_hard_stop:{reason}")

    def transport_fault(self, reason: str) -> None:
        self._fault(reason)

    def mark_feedback_timeout(self, monotonic_ns: int) -> bool:
        if self.last_feedback is None or monotonic_ns - self.last_feedback.monotonic_ns > self.feedback_stale_timeout_ns:
            self._fault("feedback_timeout")
            return True
        return False

    def cleanup(self) -> None:
        try:
            self.backend.close_port()
        finally:
            self.state = HandControlState.DISABLED
            self.transport_state = "CLOSED"
            self.next_command_monotonic_ns = None

    def episode_record(
        self,
        monotonic_ns: int,
        requested_target: Sequence[float] | None = None,
    ) -> dict[str, Any]:
        feedback = self.last_feedback
        return {
            "schema_version": RH56_PC_DIRECT_SCHEMA_VERSION,
            "monotonic_ns": int(monotonic_ns),
            "canonical_hand_order": list(CANONICAL_HAND_ORDER),
            "action": {
                "hand_target": None
                if self.last_command_monotonic_ns is None
                else list(self.last_command_normalized),
                "hand_target_unit": "normalized_closure_0_to_1",
                "requested_hand_target": None
                if requested_target is None
                else list(requested_target),
                "selected_hand_position_raw": None
                if self.last_command_monotonic_ns is None
                else list(self.last_command_raw),
            },
            "observation": {
                "hand_position": None if feedback is None else list(feedback.position_raw),
                "hand_position_unit": "rh56_angle_act_raw_count",
                "hand_position_normalized": None
                if feedback is None
                else list(feedback.position_normalized),
                "hand_current_or_load": None
                if feedback is None
                else list(feedback.load_or_force_raw_count),
                "hand_current_or_load_unit": "rh56_force_act_raw_count",
                "hand_current_raw_count": None
                if feedback is None
                else list(feedback.current_raw_count),
                "hand_state_source": None if feedback is None else feedback.state_source,
            },
            "hand_transport_state": self.transport_state,
            "hand_control_state": self.state.value,
            "hand_command_timestamp": self.last_command_monotonic_ns,
            "hand_feedback_timestamp": None if feedback is None else feedback.monotonic_ns,
            "hand_feedback_read_latency_ms": None if feedback is None else feedback.read_latency_ms,
            "hand_error": None if feedback is None else list(feedback.error),
            "hand_status": None if feedback is None else list(feedback.status),
            "hand_fault_reason": self.fault_reason,
            "combined_episode_valid": self.state is not HandControlState.FAULT,
            "required_arm_action": None
            if self.state is not HandControlState.FAULT
            else "safe_hold_or_stop",
        }

    def _require_open(self) -> None:
        if self.transport_state == "CLOSED" or self.state is HandControlState.DISABLED:
            raise RuntimeError("RH56 PC-direct transport is not open.")

    def _require_fresh_feedback(self, monotonic_ns: int) -> None:
        if self.mark_feedback_timeout(monotonic_ns):
            raise RuntimeError("RH56 feedback is stale or absent.")

    def _fault(self, reason: str) -> None:
        self.state = HandControlState.FAULT
        self.transport_state = "FAULT"
        self.fault_reason = reason
        self.next_command_monotonic_ns = None

    @staticmethod
    def _validated_int_vector(
        values: Sequence[int], min_value: int, max_value: int, name: str
    ) -> list[int]:
        result = [int(value) for value in values]
        if len(result) != len(CANONICAL_HAND_ORDER):
            raise ValueError(f"RH56 {name} vector must have six channels.")
        if any(value < min_value or value > max_value for value in result):
            raise ValueError(f"RH56 {name} values must remain within [{min_value}, {max_value}].")
        return result


class FakeRH56PcDirectBackend:
    """Deterministic no-hardware backend for the PC-direct safety contract."""

    REG = {"STATUS": 1612, "ERROR": 1606}

    def __init__(self) -> None:
        self.connected = False
        self.disconnect = False
        self.connect_count = 0
        self.close_count = 0
        self.position = [1000.0] * 6
        self.current = [0.0] * 6
        self.load = [0.0] * 6
        self.status = [0] * 6
        self.error = [0] * 6
        self.position_writes: list[list[int]] = []
        self.speed_writes: list[list[int]] = []
        self.force_writes: list[list[int]] = []
        self.timeout_count = 0
        self.checksum_failure_count = 0
        self.protocol_error_count = 0

    @property
    def write_count(self) -> int:
        return len(self.position_writes) + len(self.speed_writes) + len(self.force_writes)

    @property
    def register_write_count(self) -> int:
        return self.write_count

    def connect(self) -> bool:
        self.connect_count += 1
        self.connected = True
        return True

    def close_port(self) -> None:
        self.close_count += 1
        self.connected = False

    def _check(self) -> None:
        if not self.connected or self.disconnect:
            raise RuntimeError("fake RH56 serial disconnect")

    def get_canonical_angles(self) -> list[float]:
        self._check()
        return self.position.copy()

    def get_canonical_currents(self) -> list[float]:
        self._check()
        return self.current.copy()

    def get_canonical_forces(self) -> list[float]:
        self._check()
        return self.load.copy()

    def read_register(self, address: int, length: int) -> list[int]:
        self._check()
        if length != 6:
            raise ValueError("fake RH56 status/error reads require six bytes")
        if address == self.REG["STATUS"]:
            return self.status.copy()
        if address == self.REG["ERROR"]:
            return self.error.copy()
        raise ValueError(f"unsupported fake RH56 register {address}")

    def set_canonical_angles(self, values: list[int]) -> bool:
        self._check()
        self.position_writes.append(values.copy())
        self.position = [float(value) for value in values]
        return True

    def set_canonical_speeds(self, values: list[int]) -> bool:
        self._check()
        self.speed_writes.append(values.copy())
        return True

    def set_canonical_forces(self, values: list[int]) -> bool:
        self._check()
        self.force_writes.append(values.copy())
        return True
