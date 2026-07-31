from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Iterable, Mapping, Sequence

import numpy as np

CANONICAL_HAND_ORDER: tuple[str, ...] = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)

RH56_INTERNAL_ORDER: tuple[str, ...] = (
    "thumb_close",
    "thumb_lateral",
    "index",
    "middle",
    "ring",
    "pinky",
)

RH56_PROTOCOL_ORDER: tuple[str, ...] = (
    "pinky",
    "ring",
    "middle",
    "index",
    "thumb_close",
    "thumb_lateral",
)

HAND_SCHEMA_VERSION = "inspire6_v1"
DEFAULT_HAND_DELTA_LIMIT = 0.05


@dataclass(frozen=True, slots=True)
class HandDofCalibration:
    raw_open: float
    raw_close: float
    direction_sign: int
    safe_min: float
    safe_max: float
    default_speed: float
    default_force_limit: float

    @classmethod
    def from_open_close(
        cls,
        *,
        raw_open: float,
        raw_close: float,
        safe_min: float | None = None,
        safe_max: float | None = None,
        default_speed: float = 800.0,
        default_force_limit: float = 500.0,
    ) -> "HandDofCalibration":
        low = min(raw_open, raw_close) if safe_min is None else safe_min
        high = max(raw_open, raw_close) if safe_max is None else safe_max
        return cls(
            raw_open=float(raw_open),
            raw_close=float(raw_close),
            direction_sign=1 if raw_close >= raw_open else -1,
            safe_min=float(low),
            safe_max=float(high),
            default_speed=float(default_speed),
            default_force_limit=float(default_force_limit),
        )

    def to_dict(self) -> dict[str, float | int]:
        return asdict(self)


DEFAULT_RH56_CALIBRATION: dict[str, HandDofCalibration] = {
    name: HandDofCalibration.from_open_close(raw_open=1000.0, raw_close=0.0)
    for name in CANONICAL_HAND_ORDER
}


def _as_array(values: Sequence[float] | np.ndarray, *, expected: int | None = None) -> np.ndarray:
    array = np.asarray(values, dtype=np.float32).reshape(-1)
    if expected is not None and array.size != expected:
        raise ValueError(f"Expected {expected} values, got {array.size}.")
    return array


def _validate_order(order: Iterable[str]) -> tuple[str, ...]:
    order_tuple = tuple(order)
    if len(order_tuple) != len(set(order_tuple)):
        raise ValueError(f"Order contains duplicate names: {order_tuple!r}.")
    missing = set(CANONICAL_HAND_ORDER) - set(order_tuple)
    extra = set(order_tuple) - set(CANONICAL_HAND_ORDER)
    if missing or extra:
        raise ValueError(f"Order must contain canonical names. missing={sorted(missing)}, extra={sorted(extra)}")
    return order_tuple


def reorder_values(
    values: Sequence[float] | np.ndarray,
    *,
    source_order: Iterable[str],
    target_order: Iterable[str],
) -> list[float]:
    source = _validate_order(source_order)
    target = _validate_order(target_order)
    array = _as_array(values, expected=len(source))
    by_name = dict(zip(source, array.tolist(), strict=True))
    return [float(by_name[name]) for name in target]


def raw_to_canonical(
    values: Sequence[float] | np.ndarray,
    *,
    raw_order: Iterable[str] = RH56_INTERNAL_ORDER,
) -> list[float]:
    return reorder_values(values, source_order=raw_order, target_order=CANONICAL_HAND_ORDER)


def canonical_to_raw(
    values: Sequence[float] | np.ndarray,
    *,
    raw_order: Iterable[str] = RH56_INTERNAL_ORDER,
) -> list[float]:
    return reorder_values(values, source_order=CANONICAL_HAND_ORDER, target_order=raw_order)


def normalize_raw(
    raw_values: Sequence[float] | np.ndarray,
    *,
    raw_order: Iterable[str] = RH56_INTERNAL_ORDER,
    calibration: Mapping[str, HandDofCalibration] = DEFAULT_RH56_CALIBRATION,
) -> list[float]:
    canonical_raw = raw_to_canonical(raw_values, raw_order=raw_order)
    normalized: list[float] = []
    for name, raw in zip(CANONICAL_HAND_ORDER, canonical_raw, strict=True):
        dof = calibration[name]
        denom = dof.raw_close - dof.raw_open
        if abs(denom) < 1e-9:
            raise ValueError(f"Invalid calibration for {name}: raw_open equals raw_close.")
        normalized.append(float(np.clip((float(raw) - dof.raw_open) / denom, 0.0, 1.0)))
    return normalized


def denormalize_canonical(
    normalized_values: Sequence[float] | np.ndarray,
    *,
    raw_order: Iterable[str] = RH56_INTERNAL_ORDER,
    calibration: Mapping[str, HandDofCalibration] = DEFAULT_RH56_CALIBRATION,
) -> list[float]:
    normalized = _as_array(normalized_values, expected=len(CANONICAL_HAND_ORDER))
    canonical_raw: list[float] = []
    for name, value in zip(CANONICAL_HAND_ORDER, normalized, strict=True):
        dof = calibration[name]
        clipped = float(np.clip(value, 0.0, 1.0))
        raw = dof.raw_open + clipped * (dof.raw_close - dof.raw_open)
        canonical_raw.append(float(np.clip(raw, dof.safe_min, dof.safe_max)))
    return canonical_to_raw(canonical_raw, raw_order=raw_order)


def compute_delta(
    current_cmd: Sequence[float] | np.ndarray,
    target_cmd: Sequence[float] | np.ndarray,
    *,
    limit: float = DEFAULT_HAND_DELTA_LIMIT,
) -> list[float]:
    current = _as_array(current_cmd, expected=len(CANONICAL_HAND_ORDER))
    target = _as_array(target_cmd, expected=len(CANONICAL_HAND_ORDER))
    return np.clip(target - current, -abs(limit), abs(limit)).astype(np.float32).tolist()


def apply_delta(
    current_cmd: Sequence[float] | np.ndarray,
    delta: Sequence[float] | np.ndarray,
    *,
    limit: float = DEFAULT_HAND_DELTA_LIMIT,
) -> list[float]:
    current = _as_array(current_cmd, expected=len(CANONICAL_HAND_ORDER))
    clipped_delta = np.clip(_as_array(delta, expected=len(CANONICAL_HAND_ORDER)), -abs(limit), abs(limit))
    return np.clip(current + clipped_delta, 0.0, 1.0).astype(np.float32).tolist()


def moving_direction(
    last_cmd: Sequence[float] | np.ndarray,
    current_cmd: Sequence[float] | np.ndarray,
) -> list[int]:
    last = _as_array(last_cmd, expected=len(CANONICAL_HAND_ORDER))
    current = _as_array(current_cmd, expected=len(CANONICAL_HAND_ORDER))
    return np.sign(current - last).astype(np.int8).tolist()
def calibration_to_dict(calibration: Mapping[str, HandDofCalibration]) -> dict[str, dict[str, float | int]]:
    return {name: calibration[name].to_dict() for name in CANONICAL_HAND_ORDER}


def build_hand_state(
    *,
    raw_positions: Sequence[float] | np.ndarray,
    raw_velocities: Sequence[float] | np.ndarray | None = None,
    raw_currents: Sequence[float] | np.ndarray | None = None,
    raw_forces: Sequence[float] | np.ndarray | None = None,
    raw_contact_binary: Sequence[bool] | None = None,
    raw_order: Iterable[str] = RH56_INTERNAL_ORDER,
    calibration: Mapping[str, HandDofCalibration] | None = None,
    command: Sequence[float] | np.ndarray | None = None,
    last_command: Sequence[float] | np.ndarray | None = None,
    mode: str = "rh56",
) -> dict[str, object]:
    raw_order_tuple = _validate_order(raw_order)
    raw_pos = _as_array(raw_positions, expected=len(raw_order_tuple))
    raw_vel = _as_array(np.zeros_like(raw_pos) if raw_velocities is None else raw_velocities, expected=len(raw_order_tuple))
    raw_cur = _as_array(np.zeros_like(raw_pos) if raw_currents is None else raw_currents, expected=len(raw_order_tuple))
    raw_force = _as_array(np.zeros_like(raw_pos) if raw_forces is None else raw_forces, expected=len(raw_order_tuple))
    contact = list(raw_contact_binary) if raw_contact_binary is not None else [False] * len(raw_order_tuple)
    if len(contact) != len(raw_order_tuple):
        raise ValueError(f"Expected {len(raw_order_tuple)} contact flags, got {len(contact)}.")

    canonical_pos = raw_to_canonical(raw_pos, raw_order=raw_order_tuple)
    canonical_vel = raw_to_canonical(raw_vel, raw_order=raw_order_tuple)
    canonical_cur = raw_to_canonical(raw_cur, raw_order=raw_order_tuple)
    canonical_force = raw_to_canonical(raw_force, raw_order=raw_order_tuple)
    canonical_contact = reorder_values(
        [1.0 if value else 0.0 for value in contact],
        source_order=raw_order_tuple,
        target_order=CANONICAL_HAND_ORDER,
    )
    normalized = normalize_raw(raw_pos, raw_order=raw_order_tuple, calibration=calibration) if calibration else None

    command_list = _as_array(command, expected=len(CANONICAL_HAND_ORDER)).tolist() if command is not None else None
    last_command_list = (
        _as_array(last_command, expected=len(CANONICAL_HAND_ORDER)).tolist() if last_command is not None else None
    )
    direction = moving_direction(last_command_list, command_list) if command_list is not None and last_command_list is not None else None
    position_error = (
        (np.asarray(command_list, dtype=np.float32) - np.asarray(normalized, dtype=np.float32)).tolist()
        if command_list is not None and normalized is not None
        else None
    )

    return {
        "mode": mode,
        "schema_version": HAND_SCHEMA_VERSION,
        "canonical_order": list(CANONICAL_HAND_ORDER),
        "raw_order": list(raw_order_tuple),
        "finger_positions": raw_pos.tolist(),
        "finger_currents": raw_cur.tolist(),
        "contact_flags": [bool(value) for value in contact],
        "force_estimate": raw_force.tolist(),
        "inspire6": {
            "positions": canonical_pos,
            "normalized_positions": normalized,
            "velocities": canonical_vel,
            "currents": canonical_cur,
            "forces": canonical_force,
            "position_error": position_error,
            "contact_binary": [bool(round(value)) for value in canonical_contact],
            "slip_binary": False,
            "last_cmd": last_command_list,
            "current_cmd": command_list,
            "moving_direction": direction,
        },
        "rh56_raw": {
            "positions": raw_pos.tolist(),
            "velocities": raw_vel.tolist(),
            "currents": raw_cur.tolist(),
            "forces": raw_force.tolist(),
            "position_error": position_error,
            "contact_binary": [bool(value) for value in contact],
            "slip_binary": False,
            "calibration": calibration_to_dict(calibration) if calibration else None,
        },
    }
