from __future__ import annotations

import grp
import math
import os
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol, Sequence

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


class HandOperation(str, Enum):
    HAND_ONLY = "hand_only"
    COMBINED = "arm_hand_combined"
    RUNTIME_CONFIG = "runtime_config_write"
    FAULT_RESET = "fault_reset"
    FORCE_SENSOR_CALIBRATION = "force_sensor_calibration"


class RH56CommandShaper:
    """Bound normalized position commands by velocity and acceleration.

    This is deliberately a position shaper at the RH56 command boundary, not
    a feedback controller.  The latest target remains authoritative; the
    shaper only limits how quickly the hand command approaches it.  Hardware
    safety gates and the outer normalized delta limit remain in
    :class:`RH56PcDirectControl`.
    """

    def __init__(
        self,
        policy: Mapping[str, Any] | None,
        *,
        channel_count: int,
        command_period_ns: int,
    ) -> None:
        values = {} if policy is None else policy
        if not isinstance(values, Mapping):
            raise ValueError("RH56 command_shaping must be a mapping.")
        self.enabled = bool(values.get("enabled", False))
        self.channel_count = int(channel_count)
        self.command_period_ns = int(command_period_ns)

        def vector(name: str, default: float) -> np.ndarray:
            raw = values.get(name, default)
            items = [raw] * self.channel_count if np.isscalar(raw) else raw
            result = np.asarray(items, dtype=np.float64).reshape(-1)
            if (
                result.size != self.channel_count
                or not np.all(np.isfinite(result))
                or np.any(result <= 0.0)
            ):
                raise ValueError(
                    f"RH56 command_shaping {name} must be six positive finite values."
                )
            return result

        self.maximum_closing_velocity = vector(
            "maximum_closing_velocity", 0.35
        )
        self.maximum_opening_velocity = vector(
            "maximum_opening_velocity", 0.60
        )
        self.maximum_acceleration = vector("maximum_acceleration", 1.40)
        self.position: np.ndarray | None = None
        self.velocity = np.zeros(self.channel_count, dtype=np.float64)
        self.last_update_ns: int | None = None
        self.last_dt_sec: float | None = None
        self.step_count = 0
        self.reset_count = 0

    def reset(
        self,
        position: Sequence[float] | None = None,
        monotonic_ns: int | None = None,
    ) -> None:
        self.position = (
            None
            if position is None
            else np.asarray(position, dtype=np.float64).reshape(-1).copy()
        )
        if self.position is not None and self.position.size != self.channel_count:
            raise ValueError("RH56 command shaper position must have six values.")
        self.velocity.fill(0.0)
        self.last_update_ns = None if monotonic_ns is None else int(monotonic_ns)
        self.last_dt_sec = None
        self.reset_count += 1

    def reconcile(
        self,
        position: Sequence[float],
        monotonic_ns: int,
    ) -> None:
        """Synchronize state after an outer safety clamp or successful write."""

        value = np.asarray(position, dtype=np.float64).reshape(-1)
        if value.size != self.channel_count:
            raise ValueError("RH56 command shaper position must have six values.")
        if self.position is None or not np.allclose(value, self.position, atol=1e-12, rtol=0.0):
            self.position = value.copy()
            self.velocity.fill(0.0)
        self.last_update_ns = int(monotonic_ns)

    def step(
        self,
        previous: Sequence[float],
        target: Sequence[float],
        monotonic_ns: int,
        *,
        contact_closing_mask: Sequence[bool] | None = None,
    ) -> np.ndarray:
        previous_value = np.asarray(previous, dtype=np.float64).reshape(-1)
        target_value = np.asarray(target, dtype=np.float64).reshape(-1)
        if (
            previous_value.size != self.channel_count
            or target_value.size != self.channel_count
        ):
            raise ValueError("RH56 command shaper positions must have six values.")
        if not self.enabled:
            self.reconcile(previous_value, monotonic_ns)
            return target_value.copy()
        if self.position is None or self.last_update_ns is None:
            self.position = previous_value.copy()
            self.velocity.fill(0.0)
            dt_sec = self.command_period_ns / 1e9
        else:
            if not np.allclose(
                previous_value, self.position, atol=1e-12, rtol=0.0
            ):
                # A contact hold, measured activation, or other safety gate
                # changed the actual command.  Do not carry stale momentum
                # across that discontinuity.
                self.position = previous_value.copy()
                self.velocity.fill(0.0)
            elapsed_ns = int(monotonic_ns) - int(self.last_update_ns)
            dt_sec = (
                min(elapsed_ns / 1e9, 0.25)
                if elapsed_ns > 0
                else self.command_period_ns / 1e9
            )
        dt_sec = max(dt_sec, 1e-6)
        self.last_dt_sec = dt_sec
        contact_mask = np.zeros(self.channel_count, dtype=bool)
        if contact_closing_mask is not None:
            contact_mask = np.asarray(contact_closing_mask, dtype=bool).reshape(-1)
            if contact_mask.size != self.channel_count:
                raise ValueError("RH56 contact closing mask must have six values.")

        error = target_value - self.position
        closing = error > 0.0
        # A contact hold is an outer safety clamp.  Do not carry positive
        # (closing) momentum into the lower hold target: acceleration-limited
        # reversal would otherwise keep closing for several cycles after
        # contact had already been detected.
        self.velocity[contact_mask] = np.minimum(
            self.velocity[contact_mask], 0.0
        )
        speed_limit = np.where(
            closing,
            self.maximum_closing_velocity,
            self.maximum_opening_velocity,
        )
        stopping_speed = np.sqrt(
            2.0 * self.maximum_acceleration * np.abs(error)
        )
        desired_velocity = np.sign(error) * np.minimum(speed_limit, stopping_speed)
        acceleration_step = self.maximum_acceleration * dt_sec
        self.velocity = self.velocity + np.clip(
            desired_velocity - self.velocity,
            -acceleration_step,
            acceleration_step,
        )
        self.velocity[contact_mask] = np.minimum(
            self.velocity[contact_mask], 0.0
        )
        step = self.velocity * dt_sec
        step = np.where(closing, np.minimum(step, error), np.maximum(step, error))
        output = self.position + step
        reached = np.abs(error) <= np.abs(step) + 1e-12
        output = np.where(reached, target_value, output)
        self.velocity[reached] = 0.0
        self.position = np.clip(output, 0.0, 1.0)
        self.last_update_ns = int(monotonic_ns)
        self.step_count += 1
        return self.position.copy()

    def snapshot(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "maximum_closing_velocity": self.maximum_closing_velocity.tolist(),
            "maximum_opening_velocity": self.maximum_opening_velocity.tolist(),
            "maximum_acceleration": self.maximum_acceleration.tolist(),
            "position": None if self.position is None else self.position.tolist(),
            "velocity": self.velocity.tolist(),
            "last_update_ns": self.last_update_ns,
            "last_dt_sec": self.last_dt_sec,
            "step_count": self.step_count,
            "reset_count": self.reset_count,
        }


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

    def clear_error(self) -> bool: ...

    def calibrate_force_sensors(self) -> bool: ...


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
    register_latency_ms: Mapping[str, float] | None = None
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
    """Bounded PC-direct hand control, independent from the JAKA arm path."""

    def __init__(
        self,
        backend: PcDirectBackend,
        config: Mapping[str, Any],
        *,
        perf_counter_ns: Callable[[], int] = time.perf_counter_ns,
    ) -> None:
        self.backend = backend
        self.config = config
        self.state = HandControlState.DISABLED
        self.transport_state = "CLOSED"
        self.operation: HandOperation | None = None
        profile_name = str(config.get("scheduler_profile", "baseline"))
        profiles = config.get("scheduler_profiles", {})
        profile = profiles.get(profile_name, {}) if isinstance(profiles, Mapping) else {}
        if profiles and not profile:
            raise ValueError(f"Unknown RH56 scheduler_profile={profile_name!r}.")
        if not isinstance(profile, Mapping):
            raise ValueError(f"RH56 scheduler profile {profile_name!r} must be a mapping.")
        self.scheduler_profile = profile_name

        def scheduler_value(name: str, fallback: Any) -> Any:
            return profile.get(name, config.get(name, fallback))

        self.command_rate_hz = float(
            scheduler_value("command_rate_hz", config.get("control_frequency_hz", 15.0))
        )
        if self.command_rate_hz <= 0.0:
            raise ValueError("RH56 command_rate_hz must be positive.")
        # Retain the old public name while command and feedback rates are now
        # independently scheduled by RH56PcDirectWorker.
        self.control_frequency_hz = self.command_rate_hz
        self.command_period_ns = int(round(1e9 / self.command_rate_hz))
        self.command_shaper = RH56CommandShaper(
            config.get("command_shaping"),
            channel_count=len(CANONICAL_HAND_ORDER),
            command_period_ns=self.command_period_ns,
        )
        self.feedback_rate_hz = {
            name: float(
                scheduler_value(
                    f"{name.lower()}_feedback_rate_hz",
                    config.get("control_frequency_hz", 15.0),
                )
            )
            for name in ("ANGLE", "CURRENT", "FORCE", "STATUS", "ERROR")
        }
        if any(rate <= 0.0 for rate in self.feedback_rate_hz.values()):
            raise ValueError("RH56 feedback rates must be positive.")
        warning_defaults_ms = {
            "ANGLE": 150.0,
            "CURRENT": 250.0,
            "FORCE": 250.0,
            "STATUS": 250.0,
            "ERROR": 250.0,
        }
        configured_warning_ms = config.get("feedback_warning_age_ms", {})
        self.feedback_warning_age_ns = {
            name: int(
                round(
                    float(configured_warning_ms.get(name.lower(), default_ms))
                    * 1e6
                )
            )
            for name, default_ms in warning_defaults_ms.items()
        }
        if any(age <= 0 for age in self.feedback_warning_age_ns.values()):
            raise ValueError("RH56 feedback warning ages must be positive.")
        serial_timeout_sec = float(config.get("serial", {}).get("timeout_sec", 0.2))
        self.feedback_stale_timeout_ns = int(
            round(float(config.get("feedback_stale_timeout_sec", 2.0 * serial_timeout_sec)) * 1e9)
        )
        self.delta_limit = float(
            config.get("hand_schema", {}).get("hand_delta_limit", DEFAULT_HAND_DELTA_LIMIT)
        )
        self.max_close = float(config.get("safety", {}).get("max_close_strength", 1.0))
        if not math.isfinite(self.max_close) or not 0.0 < self.max_close <= 1.0:
            raise ValueError("RH56 max_close_strength must be in (0, 1].")
        contact_values = config.get("safety", {}).get("contact_stop", {})
        self.contact_stop_enabled = bool(contact_values.get("enabled", False))
        self.contact_require_fresh_force_before_closure = bool(
            contact_values.get("require_fresh_force_before_closure_step", False)
        )

        def contact_vector(name: str, default: float) -> np.ndarray:
            raw = contact_values.get(name, default)
            values = [raw] * len(CANONICAL_HAND_ORDER) if np.isscalar(raw) else raw
            result = np.asarray(values, dtype=np.float64).reshape(-1)
            if (
                result.size != len(CANONICAL_HAND_ORDER)
                or not np.all(np.isfinite(result))
                or np.any(result < 0.0)
            ):
                raise ValueError(f"RH56 contact_stop {name} must be six nonnegative values.")
            return result

        self.contact_force_delta_onset = contact_vector(
            "force_delta_onset", 250.0
        )
        self.contact_force_delta_release = contact_vector(
            "force_delta_release", 100.0
        )
        self.contact_closure_budget_per_force_sample = contact_vector(
            "closure_budget_per_force_sample", self.delta_limit
        )
        self.contact_maximum_closure_step = contact_vector(
            "maximum_closure_step", self.delta_limit
        )
        self.contact_minimum_closing_gap = float(
            contact_values.get("minimum_closing_gap", 0.008)
        )
        self.contact_maximum_stall_progress = float(
            contact_values.get("maximum_stall_progress", 0.005)
        )
        self.contact_relief_margin = float(
            contact_values.get("relief_margin", 0.01)
        )
        self.contact_release_open_delta = float(
            contact_values.get("release_open_delta", 0.02)
        )
        self.contact_consecutive_samples = int(
            contact_values.get("consecutive_samples", 2)
        )
        self.contact_baseline_alpha = float(
            contact_values.get("baseline_alpha", 0.10)
        )
        if (
            not math.isfinite(self.contact_minimum_closing_gap)
            or self.contact_minimum_closing_gap <= 0.0
            or not math.isfinite(self.contact_maximum_stall_progress)
            or self.contact_maximum_stall_progress < 0.0
            or np.any(
                self.contact_force_delta_release
                >= self.contact_force_delta_onset
            )
            or np.any(self.contact_closure_budget_per_force_sample <= 0.0)
            or np.any(
                self.contact_closure_budget_per_force_sample > self.delta_limit
            )
            or np.any(self.contact_maximum_closure_step <= 0.0)
            or np.any(
                self.contact_maximum_closure_step
                > self.contact_closure_budget_per_force_sample
            )
            or not math.isfinite(self.contact_relief_margin)
            or not 0.0 <= self.contact_relief_margin <= self.delta_limit
            or not math.isfinite(self.contact_release_open_delta)
            or self.contact_release_open_delta <= 0.0
            or self.contact_consecutive_samples < 2
            or not math.isfinite(self.contact_baseline_alpha)
            or not 0.0 < self.contact_baseline_alpha <= 1.0
        ):
            raise ValueError("Malformed RH56 contact_stop policy.")
        self.calibration = _calibration_from_config(config)
        diagnostics = config.get("diagnostics", {})
        self.diagnostics_enabled = bool(diagnostics.get("enabled", False))
        self.diagnostics_window_size = int(diagnostics.get("window_size", 256))
        if self.diagnostics_window_size <= 0:
            raise ValueError("RH56 diagnostics window_size must be positive.")
        self.exact_duplicate_suppression = bool(
            diagnostics.get("exact_duplicate_suppression", True)
        )
        self._perf_counter_ns = perf_counter_ns
        self.last_feedback: PcDirectFeedback | None = None
        self.last_command_normalized: tuple[float, ...] | None = None
        self.last_command_raw: tuple[int, ...] | None = None
        self.last_command_monotonic_ns: int | None = None
        self.next_command_monotonic_ns: int | None = None
        self.fault_reason: str | None = None
        self.last_failure_record: dict[str, Any] | None = None
        self.last_command_disposition = "never_evaluated"
        self.last_submitted_sequence: int | None = None
        self.last_written_sequence: int | None = None
        self.last_command_age_ms: float | None = None
        self.last_write_latency_ms: float | None = None
        self._force_next_command = False
        self.command_evaluation_count = 0
        self.command_rate_limited_count = 0
        self.command_write_attempt_count = 0
        self.successful_command_write_count = 0
        self.duplicate_suppressed_count = 0
        self.feedback_record_count = 0
        self._first_write_attempt_ns: int | None = None
        self._last_write_attempt_ns: int | None = None
        self._command_age_ms: deque[float] = deque(maxlen=self.diagnostics_window_size)
        self._write_latency_ms: deque[float] = deque(maxlen=self.diagnostics_window_size)
        self._feedback_latency_ms: deque[float] = deque(maxlen=self.diagnostics_window_size)
        self._register_latency_ms: dict[str, deque[float]] = {
            name: deque(maxlen=self.diagnostics_window_size)
            for name in ("ANGLE", "CURRENT", "FORCE", "STATUS", "ERROR")
        }
        self._feedback_values: dict[str, tuple[Any, ...] | None] = {
            name: None for name in ("ANGLE", "CURRENT", "FORCE", "STATUS", "ERROR")
        }
        self._feedback_success_ns: dict[str, int | None] = {
            name: None for name in self._feedback_values
        }
        self._feedback_latest_latency_ms: dict[str, float] = {}
        self.last_requested_target_normalized: tuple[float, ...] | None = None
        self._contact_force_baseline: np.ndarray | None = None
        self._contact_force_delta = np.zeros(6, dtype=np.float64)
        self._contact_last_angle: np.ndarray | None = None
        self._contact_angle_progress = np.zeros(6, dtype=np.float64)
        self._contact_angle_generation = 0
        self._contact_force_angle_generation = 0
        self._contact_candidate_count = np.zeros(6, dtype=np.int64)
        self._contact_hold_target = np.full(6, np.nan, dtype=np.float64)
        self._contact_latched = np.zeros(6, dtype=bool)
        self._contact_last_closing_request_ns = np.full(6, -1, dtype=np.int64)
        self._contact_force_generation = 0
        self._contact_last_closure_force_generation = -1
        self._contact_closure_since_force = np.zeros(6, dtype=np.float64)
        self.contact_detection_count = 0
        self.contact_activation_preserved_count = 0
        self.contact_activation_rebased_count = 0
        self._contact_relief_pending = False
        self.contact_relief_write_count = 0
        self._contact_last_activation_mode = "never_activated"

    def open(self, operation: HandOperation) -> None:
        if not isinstance(operation, HandOperation):
            raise TypeError("RH56 operation must be a HandOperation")
        try:
            connected = self.backend.connect()
        except Exception:
            self._fault("serial_connect_failure")
            raise
        if not connected:
            self._fault("serial_connect_failure")
            raise RuntimeError("RH56 PC-direct backend did not connect.")
        self.operation = operation
        self.transport_state = "CONNECTED_READ_ONLY"
        self.state = HandControlState.HOLD

    def poll_feedback(self, monotonic_ns: int) -> PcDirectFeedback:
        self._require_open()
        started_ns = self._perf_counter_ns()
        register_latency_ms: dict[str, float] = {}

        def timed(name: str, operation: Callable[[], Sequence[float] | Sequence[int]]) -> tuple[Any, ...]:
            register_started_ns = self._perf_counter_ns()
            try:
                return tuple(operation())
            finally:
                if self.diagnostics_enabled:
                    elapsed_ms = (self._perf_counter_ns() - register_started_ns) / 1e6
                    register_latency_ms[name] = elapsed_ms
                    self._register_latency_ms[name].append(elapsed_ms)

        try:
            position = timed("ANGLE", self.backend.get_canonical_angles)
            currents = timed("CURRENT", self.backend.get_canonical_currents)
            loads = timed("FORCE", self.backend.get_canonical_forces)
            status = tuple(
                int(value)
                for value in raw_to_canonical(
                    timed(
                        "STATUS",
                        lambda: self.backend.read_register(self.backend.REG["STATUS"], 6),
                    ),
                    raw_order=self.config.get("hand_schema", {}).get(
                        "protocol_order", CANONICAL_HAND_ORDER
                    ),
                )
            )
            errors = tuple(
                int(value)
                for value in raw_to_canonical(
                    timed(
                        "ERROR",
                        lambda: self.backend.read_register(self.backend.REG["ERROR"], 6),
                    ),
                    raw_order=self.config.get("hand_schema", {}).get(
                        "protocol_order", CANONICAL_HAND_ORDER
                    ),
                )
            )
        except Exception as exc:
            self._fault("serial_feedback_failure")
            self._capture_failure(
                "feedback_poll",
                exc,
                monotonic_ns,
                {"completed_register_latency_ms": register_latency_ms},
            )
            raise
        normalized = tuple(
            normalize_raw(
                position,
                raw_order=CANONICAL_HAND_ORDER,
                calibration=self.calibration,
            )
        )
        read_latency_ms = (self._perf_counter_ns() - started_ns) / 1e6
        feedback = PcDirectFeedback(
            monotonic_ns=int(monotonic_ns),
            position_raw=position,
            position_normalized=normalized,
            current_raw_count=currents,
            load_or_force_raw_count=loads,
            status=status,
            error=errors,
            read_latency_ms=read_latency_ms,
            register_latency_ms=register_latency_ms if self.diagnostics_enabled else None,
        )
        self.last_feedback = feedback
        for name, values in {
            "ANGLE": position,
            "CURRENT": currents,
            "FORCE": loads,
            "STATUS": status,
            "ERROR": errors,
        }.items():
            self._feedback_values[name] = values
            self._feedback_success_ns[name] = int(monotonic_ns)
        self._feedback_latest_latency_ms.update(register_latency_ms)
        self.feedback_record_count += 1
        self._observe_contact_angle(normalized)
        self._observe_contact_force(loads, monotonic_ns)
        if self.diagnostics_enabled:
            self._feedback_latency_ms.append(read_latency_ms)
        if any(errors):
            self._fault("device_error_status")
            exc = RuntimeError(f"RH56 device error registers are nonzero: {list(errors)}")
            self._capture_failure(
                "device_error_status",
                exc,
                monotonic_ns,
                {"error": list(errors), "status": list(status)},
            )
            raise exc
        if read_latency_ms * 1e6 > self.feedback_stale_timeout_ns:
            self._fault("feedback_stale_during_read")
            exc = RuntimeError(
                f"RH56 feedback read took {read_latency_ms:.3f} ms, exceeding the stale boundary."
            )
            self._capture_failure(
                "feedback_stale_during_read",
                exc,
                monotonic_ns,
                {"read_latency_ms": read_latency_ms},
            )
            raise exc
        if self.last_command_normalized is None:
            self.last_command_normalized = normalized
            self.last_command_raw = tuple(int(round(value)) for value in position)
            self.command_shaper.reset(normalized, monotonic_ns)
        return feedback

    def poll_feedback_register(
        self, register: str, monotonic_ns: int
    ) -> PcDirectFeedback:
        """Read exactly one feedback register and refresh the cached snapshot."""

        self._require_open()
        name = str(register).upper()
        if name not in self._feedback_values:
            raise ValueError(f"Unknown RH56 feedback register {register!r}.")
        started_ns = self._perf_counter_ns()
        try:
            if name == "ANGLE":
                values: tuple[Any, ...] = tuple(self.backend.get_canonical_angles())
            elif name == "CURRENT":
                values = tuple(self.backend.get_canonical_currents())
            elif name == "FORCE":
                values = tuple(self.backend.get_canonical_forces())
            else:
                values = tuple(
                    int(value)
                    for value in raw_to_canonical(
                        self.backend.read_register(self.backend.REG[name], 6),
                        raw_order=self.config.get("hand_schema", {}).get(
                            "protocol_order", CANONICAL_HAND_ORDER
                        ),
                    )
                )
        except Exception as exc:
            self._fault("serial_feedback_failure")
            self._capture_failure(
                "feedback_register_poll",
                exc,
                monotonic_ns,
                {"register": name},
            )
            raise
        latency_ms = (self._perf_counter_ns() - started_ns) / 1e6
        self._feedback_values[name] = values
        self._feedback_success_ns[name] = int(monotonic_ns)
        self._feedback_latest_latency_ms[name] = latency_ms
        if self.diagnostics_enabled:
            self._register_latency_ms[name].append(latency_ms)
        if name == "ERROR" and any(values):
            self._fault("device_error_status")
            exc = RuntimeError(
                f"RH56 device error registers are nonzero: {list(values)}"
            )
            self._capture_failure(
                "device_error_status",
                exc,
                monotonic_ns,
                {
                    "error": list(values),
                    "status": None
                    if self._feedback_values["STATUS"] is None
                    else list(self._feedback_values["STATUS"]),
                },
            )
            raise exc
        if latency_ms * 1e6 > self.feedback_stale_timeout_ns:
            self._fault("feedback_stale_during_read")
            exc = RuntimeError(
                f"RH56 {name} feedback read took {latency_ms:.3f} ms, "
                "exceeding the stale boundary."
            )
            self._capture_failure(
                "feedback_stale_during_read",
                exc,
                monotonic_ns,
                {"register": name, "read_latency_ms": latency_ms},
            )
            raise exc
        feedback = self._feedback_from_cache(latency_ms)
        self.last_feedback = feedback
        if name == "ANGLE":
            self._observe_contact_angle(feedback.position_normalized)
        elif name == "FORCE":
            self._observe_contact_force(
                feedback.load_or_force_raw_count, monotonic_ns
            )
        if name == "ANGLE" and self.last_command_normalized is None:
            self.last_command_normalized = feedback.position_normalized
            self.last_command_raw = tuple(
                int(round(value)) for value in feedback.position_raw
            )
            self.command_shaper.reset(feedback.position_normalized, monotonic_ns)
        return feedback

    def _feedback_from_cache(self, read_latency_ms: float) -> PcDirectFeedback:
        if any(value is None for value in self._feedback_values.values()):
            raise RuntimeError("RH56 feedback cache is incomplete.")
        position = self._feedback_values["ANGLE"]
        assert position is not None
        angle_ns = self._feedback_success_ns["ANGLE"]
        assert angle_ns is not None
        return PcDirectFeedback(
            monotonic_ns=angle_ns,
            position_raw=tuple(float(value) for value in position),
            position_normalized=tuple(
                normalize_raw(
                    position,
                    raw_order=CANONICAL_HAND_ORDER,
                    calibration=self.calibration,
                )
            ),
            current_raw_count=tuple(
                float(value) for value in self._feedback_values["CURRENT"] or ()
            ),
            load_or_force_raw_count=tuple(
                float(value) for value in self._feedback_values["FORCE"] or ()
            ),
            status=tuple(int(value) for value in self._feedback_values["STATUS"] or ()),
            error=tuple(int(value) for value in self._feedback_values["ERROR"] or ()),
            read_latency_ms=float(read_latency_ms),
            register_latency_ms=(
                dict(self._feedback_latest_latency_ms)
                if self.diagnostics_enabled
                else None
            ),
        )

    def activate(self, monotonic_ns: int) -> None:
        if self.operation not in {
            HandOperation.HAND_ONLY,
            HandOperation.COMBINED,
        }:
            raise PermissionError("This RH56 operation does not allow position commands.")
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
        self.last_requested_target_normalized = self.last_command_normalized
        self.command_shaper.reset(self.last_command_normalized, monotonic_ns)
        self._prepare_contact_stop_activation(self.last_feedback)
        self.state = HandControlState.ACTIVE
        self.transport_state = "CONNECTED_COMMANDING"
        self.next_command_monotonic_ns = int(monotonic_ns)
        self._force_next_command = True

    def command(
        self,
        target_normalized: Sequence[float],
        monotonic_ns: int,
        *,
        grip_fresh: bool = True,
        arm_terminal_stop: bool = False,
        submitted_monotonic_ns: int | None = None,
        target_sequence: int | None = None,
        force_write: bool = False,
        measured_activation_write: bool = False,
        contact_relief: bool = False,
    ) -> bool:
        self.command_evaluation_count += 1
        self.last_submitted_sequence = target_sequence
        if arm_terminal_stop:
            self._fault("arm_terminal_hard_stop")
            return False
        if not grip_fresh:
            self.hold("grip_stale")
            return False
        if self.state is not HandControlState.ACTIVE and not (
            contact_relief and self.state is HandControlState.HOLD
        ):
            self.last_command_disposition = "inactive"
            return False
        self._require_fresh_feedback(monotonic_ns)
        if self.state is HandControlState.FAULT:
            self.last_command_disposition = "faulted"
            return False
        # A measured activation is the one safety-significant exception to the
        # ordinary command cadence: it must be written exactly once from fresh
        # ANGLE_ACT even when grip is re-engaged inside the previous 40 Hz
        # command window.  Deferring it would consume the worker's one-shot
        # force flag and turn the same activation target into an invalid normal
        # write on the next cycle.
        if (
            not force_write
            and self.next_command_monotonic_ns is not None
            and monotonic_ns < self.next_command_monotonic_ns
        ):
            self.command_rate_limited_count += 1
            self.last_command_disposition = "rate_limited"
            return False
        requested = np.asarray(target_normalized, dtype=np.float64).reshape(-1)
        if requested.size != len(CANONICAL_HAND_ORDER):
            raise ValueError("RH56 normalized target must have six canonical channels.")
        target_ceiling = 1.0 if measured_activation_write else self.max_close
        if (
            not np.all(np.isfinite(requested))
            or np.any(requested < 0.0)
            or np.any(requested > target_ceiling)
        ):
            raise ValueError(
                f"RH56 normalized target must remain within [0, {target_ceiling}]."
            )
        if measured_activation_write:
            if not force_write or self.last_feedback is None:
                raise ValueError(
                    "Measured activation requires a forced write with fresh feedback."
                )
            measured = np.clip(
                np.asarray(self.last_feedback.position_normalized, dtype=np.float64),
                0.0,
                1.0,
            )
            if not np.allclose(requested, measured, atol=1e-12, rtol=0.0):
                raise ValueError(
                    "Measured activation target must equal current ANGLE_ACT."
                )
        assert self.last_command_normalized is not None
        previous_command = np.asarray(
            self.last_command_normalized, dtype=np.float64
        )
        self.last_requested_target_normalized = tuple(float(value) for value in requested)
        if not measured_activation_write:
            requested = self.contact_limited_target(requested, allow_release=True)
            closing_requested = requested > previous_command + 1e-12
            if (
                self.contact_stop_enabled
                and self.contact_require_fresh_force_before_closure
                and np.any(closing_requested)
            ):
                remaining_closure = np.maximum(
                    0.0,
                    self.contact_closure_budget_per_force_sample
                    - self._contact_closure_since_force,
                )
                requested = np.where(
                    closing_requested,
                    np.minimum(requested, previous_command + remaining_closure),
                    requested,
                )
                closure_blocked = requested <= previous_command + 1e-12
                opening_requested = requested < previous_command - 1e-12
                if np.all(closure_blocked[closing_requested]) and not np.any(
                    opening_requested
                ):
                    self.last_command_disposition = "contact_feedback_wait"
                    return False
            # The mask represents an outer contact hold, not merely a target
            # that still points in the closing direction.  The hold target is
            # normally below the last command, and the shaper must discard any
            # residual positive velocity before starting that relief motion.
            contact_closing_mask = (
                self._contact_latched | (self._contact_candidate_count > 0)
            )
            requested = self.command_shaper.step(
                previous_command,
                requested,
                monotonic_ns,
                contact_closing_mask=contact_closing_mask,
            )
        else:
            self.command_shaper.reconcile(requested, monotonic_ns)
        requested_tuple = tuple(float(value) for value in requested)
        if (
            self.exact_duplicate_suppression
            and not force_write
            and not self._force_next_command
            and requested_tuple == self.last_command_normalized
        ):
            self.duplicate_suppressed_count += 1
            self.last_command_disposition = "exact_duplicate_suppressed"
            self.next_command_monotonic_ns = int(monotonic_ns) + self.command_period_ns
            if submitted_monotonic_ns is not None:
                self.last_command_age_ms = max(
                    0.0, (monotonic_ns - submitted_monotonic_ns) / 1e6
                )
            return False
        previous = np.asarray(self.last_command_normalized, dtype=np.float64)
        command_delta = requested - previous
        if (
            not measured_activation_write
            and self.contact_stop_enabled
            and self.contact_require_fresh_force_before_closure
        ):
            selected_delta = np.where(
                command_delta >= 0.0,
                np.minimum(command_delta, self.contact_maximum_closure_step),
                np.maximum(command_delta, -self.delta_limit),
            )
        else:
            selected_delta = np.clip(
                command_delta, -self.delta_limit, self.delta_limit
            )
        selected = previous + selected_delta
        self.command_shaper.reconcile(selected, monotonic_ns)
        raw = denormalize_canonical(
            selected,
            raw_order=CANONICAL_HAND_ORDER,
            calibration=self.calibration,
        )
        raw_int = [int(round(value)) for value in raw]
        command_age_ms = (
            None
            if submitted_monotonic_ns is None
            else max(0.0, (monotonic_ns - submitted_monotonic_ns) / 1e6)
        )
        write_started_ns = self._perf_counter_ns()
        self.command_write_attempt_count += 1
        if self._first_write_attempt_ns is None:
            self._first_write_attempt_ns = int(monotonic_ns)
        self._last_write_attempt_ns = int(monotonic_ns)
        self._force_next_command = False
        try:
            ok = self.backend.set_canonical_angles(raw_int)
        except Exception as exc:
            self.last_write_latency_ms = (self._perf_counter_ns() - write_started_ns) / 1e6
            self._fault("serial_command_failure")
            self._capture_failure(
                "command_write",
                exc,
                monotonic_ns,
                {
                    "target_sequence": target_sequence,
                    "command_age_ms": command_age_ms,
                    "selected_raw": raw_int,
                    "write_latency_ms": self.last_write_latency_ms,
                },
            )
            raise
        self.last_write_latency_ms = (self._perf_counter_ns() - write_started_ns) / 1e6
        if not ok:
            self._fault("serial_command_rejected")
            exc = RuntimeError("RH56 rejected the position command.")
            self._capture_failure(
                "command_write",
                exc,
                monotonic_ns,
                {
                    "target_sequence": target_sequence,
                    "command_age_ms": command_age_ms,
                    "selected_raw": raw_int,
                    "write_latency_ms": self.last_write_latency_ms,
                },
            )
            raise exc
        self.last_command_normalized = tuple(float(value) for value in selected)
        self.last_command_raw = tuple(raw_int)
        self.last_command_monotonic_ns = int(monotonic_ns)
        self.next_command_monotonic_ns = int(monotonic_ns) + self.command_period_ns
        self.last_written_sequence = target_sequence
        self.last_command_age_ms = command_age_ms
        self.last_command_disposition = "serial_write_success"
        if contact_relief:
            self.contact_relief_write_count += 1
        closure_written = selected > previous + 1e-12
        if np.any(closure_written):
            self._contact_closure_since_force += np.maximum(
                selected - previous, 0.0
            )
            self._contact_last_closing_request_ns[closure_written] = int(
                monotonic_ns
            )
            self._contact_last_closure_force_generation = (
                self._contact_force_generation
            )
        self.successful_command_write_count += 1
        if self.diagnostics_enabled:
            self._write_latency_ms.append(self.last_write_latency_ms)
            if command_age_ms is not None:
                self._command_age_ms.append(command_age_ms)
        return True

    def write_runtime_config(
        self,
        speeds: Sequence[int],
        forces: Sequence[int],
    ) -> None:
        if self.operation is not HandOperation.RUNTIME_CONFIG:
            raise PermissionError("Runtime configuration requires the runtime-config operation.")
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

    def clear_device_error(self) -> None:
        if self.operation is not HandOperation.FAULT_RESET:
            raise PermissionError(
                "Fault reset requires the clear-error operation."
            )
        try:
            if not self.backend.clear_error():
                raise RuntimeError("RH56 rejected the fault-reset command.")
        except Exception:
            self._fault("fault_reset_write_failure")
            raise

    def start_force_sensor_calibration(self) -> None:
        if self.operation is not HandOperation.FORCE_SENSOR_CALIBRATION:
            raise PermissionError(
                "Force calibration requires the force-sensor-calibration operation."
            )
        try:
            if not self.backend.calibrate_force_sensors():
                raise RuntimeError(
                    "RH56 rejected the force-sensor-calibration command."
                )
        except Exception:
            self._fault("force_sensor_calibration_write_failure")
            raise

    def hold(self, reason: str) -> None:
        if self.state is HandControlState.FAULT:
            return
        self.state = HandControlState.HOLD
        self.transport_state = f"CONNECTED_HOLD:{reason}"
        self.next_command_monotonic_ns = None
        self.command_shaper.reset(self.last_command_normalized)

    def arm_terminal_stop(self, reason: str) -> None:
        self._fault(f"arm_terminal_hard_stop:{reason}")

    def transport_fault(self, reason: str) -> None:
        self._fault(reason)

    def mark_feedback_timeout(self, monotonic_ns: int) -> bool:
        if self.last_feedback is None or monotonic_ns - self.last_feedback.monotonic_ns > self.feedback_stale_timeout_ns:
            self._fault("feedback_timeout")
            exc = RuntimeError("RH56 feedback is stale or absent.")
            self._capture_failure(
                "feedback_freshness",
                exc,
                monotonic_ns,
                {
                    "last_feedback_monotonic_ns": (
                        None
                        if self.last_feedback is None
                        else self.last_feedback.monotonic_ns
                    ),
                    "feedback_stale_timeout_ns": self.feedback_stale_timeout_ns,
                },
            )
            return True
        return False

    def cleanup(self) -> None:
        try:
            self.backend.close_port()
        finally:
            self.state = HandControlState.DISABLED
            self.transport_state = "CLOSED"
            self.next_command_monotonic_ns = None
            self.command_shaper.reset(self.last_command_normalized)

    def episode_record(
        self,
        monotonic_ns: int,
        requested_target: Sequence[float] | None = None,
        *,
        include_diagnostics: bool = True,
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
                "contact_stop": self.contact_stop_snapshot(),
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
            "hand_feedback_register_latency_ms": (
                None if feedback is None else feedback.register_latency_ms
            ),
            "hand_feedback_register_timestamps_ns": {
                "ANGLE_ACT": self._feedback_success_ns["ANGLE"],
                "CURRENT": self._feedback_success_ns["CURRENT"],
                "FORCE_ACT": self._feedback_success_ns["FORCE"],
                "ERROR": self._feedback_success_ns["ERROR"],
                "STATUS": self._feedback_success_ns["STATUS"],
            },
            "rh56_registers": {
                "ANGLE_ACT": None if feedback is None else list(feedback.position_raw),
                "CURRENT": None if feedback is None else list(feedback.current_raw_count),
                "FORCE_ACT": None
                if feedback is None
                else list(feedback.load_or_force_raw_count),
                "ERROR": None if feedback is None else list(feedback.error),
                "STATUS": None if feedback is None else list(feedback.status),
            },
            "hand_error": None if feedback is None else list(feedback.error),
            "hand_status": None if feedback is None else list(feedback.status),
            "hand_fault_reason": self.fault_reason,
            "hand_failure": self.last_failure_record,
            "hand_target_sequence": self.last_submitted_sequence,
            "hand_written_sequence": self.last_written_sequence,
            "hand_command_disposition": self.last_command_disposition,
            "hand_command_age_ms": self.last_command_age_ms,
            "rh56_diagnostics": (
                self.diagnostics_snapshot()
                if self.diagnostics_enabled and include_diagnostics
                else None
            ),
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
        # Fault is terminal for this control instance. Preserve the first cause
        # so a later arm-stop, transport, or cleanup symptom cannot replace the
        # device/protocol failure that actually stopped command output.
        if self.fault_reason is None:
            self.fault_reason = reason
        self.next_command_monotonic_ns = None
        self.command_shaper.reset(self.last_command_normalized)

    def diagnostics_snapshot(self) -> dict[str, Any]:
        return {
            "enabled": self.diagnostics_enabled,
            "window_size": self.diagnostics_window_size,
            "scheduler_profile": self.scheduler_profile,
            "requested_command_rate_hz": self.command_rate_hz,
            "requested_feedback_rate_hz": dict(self.feedback_rate_hz),
            "exact_duplicate_suppression": self.exact_duplicate_suppression,
            "command_evaluation_count": self.command_evaluation_count,
            "command_rate_limited_count": self.command_rate_limited_count,
            "serial_write_attempt_count": self.command_write_attempt_count,
            "serial_write_attempt_rate_hz": _rate(
                self.command_write_attempt_count,
                self._first_write_attempt_ns,
                self._last_write_attempt_ns,
            ),
            "successful_serial_write_count": self.successful_command_write_count,
            "exact_duplicate_suppressed_count": self.duplicate_suppressed_count,
            "complete_feedback_record_count": self.feedback_record_count,
            "last_target_sequence": self.last_submitted_sequence,
            "last_written_sequence": self.last_written_sequence,
            "last_command_age_ms": self.last_command_age_ms,
            "maximum_command_age_ms": max(self._command_age_ms, default=None),
            "command_age_ms": _distribution(self._command_age_ms),
            "last_write_latency_ms": self.last_write_latency_ms,
            "maximum_write_latency_ms": max(self._write_latency_ms, default=None),
            "serial_write_latency_ms": _distribution(self._write_latency_ms),
            "maximum_feedback_latency_ms": max(self._feedback_latency_ms, default=None),
            "complete_feedback_latency_ms": _distribution(
                self._feedback_latency_ms
            ),
            "feedback_register_maximum_latency_ms": {
                name: max(values, default=None)
                for name, values in self._register_latency_ms.items()
            },
            "feedback_register_latency_ms": {
                name: _distribution(values)
                for name, values in self._register_latency_ms.items()
            },
            "command_shaping": self.command_shaper.snapshot(),
            "contact_stop": self.contact_stop_snapshot(),
        }

    def contact_limited_target(
        self,
        target_normalized: Sequence[float],
        *,
        allow_release: bool,
    ) -> np.ndarray:
        """Apply provisional/latched contact holds without changing open commands."""

        requested = np.asarray(target_normalized, dtype=np.float64).reshape(-1).copy()
        if not self.contact_stop_enabled:
            return requested
        for index in range(len(CANONICAL_HAND_ORDER)):
            active = bool(
                self._contact_latched[index]
                or self._contact_candidate_count[index] > 0
            )
            if not active:
                continue
            hold = float(self._contact_hold_target[index])
            if allow_release and requested[index] <= hold - self.contact_release_open_delta:
                self._contact_latched[index] = False
                self._contact_candidate_count[index] = 0
                self._contact_hold_target[index] = math.nan
                self._contact_last_closing_request_ns[index] = -1
                continue
            requested[index] = min(requested[index], hold)
        return requested

    def pop_contact_relief_target(self) -> tuple[float, ...] | None:
        """Consume one feedback-qualified opening target while hand is held.

        Ordinary grip release remains a no-write hold. A pending target is
        created only after the contact detector observes a qualified loaded
        channel while the previous target is still being held; the one-shot
        opening is the safety exception that relieves that contact before a
        controller-side device error becomes terminal.
        """

        if (
            not self._contact_relief_pending
            or self.state is not HandControlState.HOLD
            or self.last_command_normalized is None
        ):
            return None
        active = self._contact_latched | (self._contact_candidate_count > 0)
        if not np.any(active):
            self._contact_relief_pending = False
            return None
        requested = np.asarray(self.last_command_normalized, dtype=np.float64)
        relief = requested.copy()
        for index in np.flatnonzero(active):
            hold = float(self._contact_hold_target[index])
            if math.isfinite(hold):
                relief[index] = min(relief[index], max(0.0, hold))
        if not np.any(relief < requested - 1e-12):
            self._contact_relief_pending = False
            return None
        self._contact_relief_pending = False
        return tuple(float(value) for value in relief)

    def _reset_contact_stop(self, feedback: PcDirectFeedback) -> None:
        self._contact_force_baseline = np.asarray(
            feedback.load_or_force_raw_count, dtype=np.float64
        )
        self._contact_force_delta.fill(0.0)
        self._contact_last_angle = np.asarray(
            feedback.position_normalized, dtype=np.float64
        )
        self._contact_angle_progress.fill(0.0)
        self._contact_angle_generation += 1
        self._contact_force_angle_generation = self._contact_angle_generation
        self._contact_candidate_count.fill(0)
        self._contact_hold_target.fill(math.nan)
        self._contact_latched.fill(False)
        self._contact_last_closing_request_ns.fill(-1)
        self._contact_relief_pending = False
        self._contact_last_closure_force_generation = (
            self._contact_force_generation - 1
        )
        self._contact_closure_since_force.fill(0.0)

    def _prepare_contact_stop_activation(
        self, feedback: PcDirectFeedback
    ) -> None:
        """Rebase only after an unloaded clutch cycle; preserve loaded holds."""

        if not self.contact_stop_enabled or self._contact_force_baseline is None:
            self._reset_contact_stop(feedback)
            self.contact_activation_rebased_count += 1
            self._contact_last_activation_mode = "rebased"
            return

        current_force = np.asarray(
            feedback.load_or_force_raw_count, dtype=np.float64
        )
        force_delta = np.abs(current_force - self._contact_force_baseline)
        contact_active = self._contact_latched | (
            self._contact_candidate_count > 0
        )
        load_persists = force_delta >= self.contact_force_delta_release
        if not np.any(contact_active | load_persists):
            self._reset_contact_stop(feedback)
            self.contact_activation_rebased_count += 1
            self._contact_last_activation_mode = "rebased_unloaded"
            return

        # A grip release stops new writes but the RH56 continues servoing its
        # last position target.  Reacquiring while an object is still loaded
        # must therefore retain the no-load baseline and all provisional or
        # latched holds instead of treating the loaded FORCE_ACT values as zero.
        self._contact_force_delta = force_delta
        measured = np.asarray(
            feedback.position_normalized, dtype=np.float64
        )
        self._contact_last_angle = measured.copy()
        self._contact_angle_progress.fill(0.0)
        self._contact_angle_generation += 1
        self._contact_force_angle_generation = self._contact_angle_generation
        self._contact_last_closure_force_generation = (
            self._contact_force_generation - 1
        )
        self._contact_closure_since_force.fill(0.0)

        # If release raced the first qualified FORCE sample, conservatively
        # restore a provisional hold on a still-loaded channel that had been
        # closing.  A later fresh sample confirms it; an explicit opening
        # command remains able to release it through the normal hysteresis.
        provisional = (
            load_persists
            & ~self._contact_latched
            & (self._contact_candidate_count == 0)
            & (self._contact_last_closing_request_ns >= 0)
        )
        for index in np.flatnonzero(provisional):
            self._contact_candidate_count[index] = 1
            self._contact_hold_target[index] = max(
                0.0, float(measured[index]) - self.contact_relief_margin
            )

        self.contact_activation_preserved_count += 1
        self._contact_last_activation_mode = "preserved_loaded_contact"

    def _observe_contact_angle(self, values: Sequence[float]) -> None:
        current = np.asarray(values, dtype=np.float64)
        if self._contact_last_angle is not None:
            self._contact_angle_progress = current - self._contact_last_angle
        self._contact_last_angle = current
        self._contact_angle_generation += 1

    def _observe_contact_force(
        self, values: Sequence[float], monotonic_ns: int
    ) -> None:
        self._contact_force_generation += 1
        self._contact_closure_since_force.fill(0.0)
        current_force = np.asarray(values, dtype=np.float64)
        if self._contact_force_baseline is None:
            self._contact_force_baseline = current_force.copy()
            return
        self._contact_force_delta = np.abs(
            current_force - self._contact_force_baseline
        )
        if (
            not self.contact_stop_enabled
            or self.state not in {
                HandControlState.ACTIVE,
                HandControlState.HOLD,
            }
            or self.last_requested_target_normalized is None
            or self._contact_last_angle is None
            or self._contact_angle_generation
            <= self._contact_force_angle_generation
        ):
            return
        requested = np.asarray(
            self.last_requested_target_normalized, dtype=np.float64
        )
        measured = self._contact_last_angle
        assert measured is not None
        for index in range(len(CANONICAL_HAND_ORDER)):
            if self._contact_latched[index]:
                continue
            closing_gap = requested[index] - measured[index]
            closure_seen = self._contact_last_closing_request_ns[index] >= 0
            stalled = (
                self._contact_angle_progress[index]
                <= self.contact_maximum_stall_progress
            )
            onset_force_seen = (
                self._contact_force_delta[index]
                >= self.contact_force_delta_onset[index]
            )
            release_force_seen = (
                self._contact_force_delta[index]
                >= self.contact_force_delta_release[index]
            )
            first_candidate = (
                self._contact_candidate_count[index] == 0
                and closure_seen
                and onset_force_seen
                and (
                    closing_gap >= self.contact_minimum_closing_gap
                    or requested[index] + self.contact_minimum_closing_gap
                    >= measured[index]
                )
            )
            confirmed_candidate = (
                self._contact_candidate_count[index] > 0
                and release_force_seen
                and stalled
            )
            if first_candidate or confirmed_candidate:
                if first_candidate:
                    last_command = (
                        measured[index]
                        if self.last_command_normalized is None
                        else self.last_command_normalized[index]
                    )
                    self._contact_hold_target[index] = max(
                        0.0,
                        min(last_command, measured[index])
                        - self.contact_relief_margin,
                    )
                self._contact_candidate_count[index] += 1
                if (
                    self._contact_candidate_count[index]
                    >= self.contact_consecutive_samples
                ):
                    self._contact_latched[index] = True
                    self.contact_detection_count += 1
                if self.state is HandControlState.HOLD:
                    self._contact_relief_pending = True
            else:
                self._contact_candidate_count[index] = 0
                self._contact_hold_target[index] = math.nan
                if not closure_seen:
                    alpha = self.contact_baseline_alpha
                    self._contact_force_baseline[index] = (
                        (1.0 - alpha) * self._contact_force_baseline[index]
                        + alpha * current_force[index]
                    )
        self._contact_force_angle_generation = self._contact_angle_generation

    def contact_stop_snapshot(self) -> dict[str, Any]:
        return {
            "enabled": self.contact_stop_enabled,
            "detection_count": self.contact_detection_count,
            "activation_preserved_count": self.contact_activation_preserved_count,
            "activation_rebased_count": self.contact_activation_rebased_count,
            "last_activation_mode": self._contact_last_activation_mode,
            "relief_pending": self._contact_relief_pending,
            "relief_write_count": self.contact_relief_write_count,
            "latched": self._contact_latched.tolist(),
            "candidate_count": self._contact_candidate_count.tolist(),
            "hold_target_normalized": [
                None if not math.isfinite(value) else float(value)
                for value in self._contact_hold_target
            ],
            "force_baseline": None
            if self._contact_force_baseline is None
            else self._contact_force_baseline.tolist(),
            "force_delta": self._contact_force_delta.tolist(),
            "angle_progress": self._contact_angle_progress.tolist(),
            "force_generation": self._contact_force_generation,
            "fresh_force_before_closure_step": (
                self.contact_require_fresh_force_before_closure
            ),
            "closure_budget_per_force_sample": (
                self.contact_closure_budget_per_force_sample.tolist()
            ),
            "maximum_closure_step": self.contact_maximum_closure_step.tolist(),
            "closure_since_force": self._contact_closure_since_force.tolist(),
        }

    def _capture_failure(
        self,
        stage: str,
        exc: BaseException,
        monotonic_ns: int,
        context: Mapping[str, Any] | None = None,
    ) -> None:
        serial_context = getattr(exc, "as_dict", None)
        if callable(serial_context):
            serial_details = serial_context()
        else:
            serial_details = getattr(self.backend, "last_failure_context", None)
        self.last_failure_record = {
            "schema_version": "rh56_control_failure.v1",
            "stage": stage,
            "monotonic_ns": int(monotonic_ns),
            "exception_type": type(exc).__name__,
            "message": str(exc),
            "fault_reason": self.fault_reason,
            "control_state": self.state.value,
            "transport_state": self.transport_state,
            "serial": serial_details,
            "context": dict(context or {}),
        }

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


def _distribution(values: Sequence[float]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)

    def percentile(fraction: float) -> float:
        index = int(round((len(ordered) - 1) * fraction))
        return ordered[index]

    return {
        "p50": percentile(0.50),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "max": ordered[-1],
    }


def _rate(count: int, first_ns: int | None, last_ns: int | None) -> float | None:
    if count < 2 or first_ns is None or last_ns is None or last_ns <= first_ns:
        return None
    return (count - 1) * 1e9 / (last_ns - first_ns)


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
        self.clear_error_write_count = 0
        self.force_calibration_write_count = 0
        self.timeout_count = 0
        self.checksum_failure_count = 0
        self.protocol_error_count = 0

    @property
    def write_count(self) -> int:
        return (
            len(self.position_writes)
            + len(self.speed_writes)
            + len(self.force_writes)
            + self.clear_error_write_count
            + self.force_calibration_write_count
        )

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

    def clear_error(self) -> bool:
        self._check()
        self.clear_error_write_count += 1
        self.error = [0] * 6
        self.status = [0] * 6
        return True

    def calibrate_force_sensors(self) -> bool:
        self._check()
        self.force_calibration_write_count += 1
        return True
