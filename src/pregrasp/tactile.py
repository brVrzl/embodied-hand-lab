from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER


@dataclass(frozen=True, slots=True)
class TactileCorrection:
    confidence: float
    hand_delta: list[float]
    wrist_delta_xyz_m: list[float]
    status: str
    reasons: list[str]

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["canonical_hand_order"] = list(CANONICAL_HAND_ORDER)
        return result


def estimate_tactile_correction(
    hand_state: Mapping[str, Any],
    *,
    target_contacts: Sequence[str],
    max_hand_delta: float = 0.04,
    force_threshold: float = 30.0,
) -> TactileCorrection:
    inspire = hand_state.get("inspire6") if isinstance(hand_state, Mapping) else None
    source = inspire if isinstance(inspire, Mapping) else hand_state
    contacts = _canonical_contact_flags(source)
    forces = _canonical_forces(source)

    target_set = set(target_contacts)
    active = {name for name, flag in zip(CANONICAL_HAND_ORDER, contacts, strict=True) if flag}
    missing = sorted((target_set & set(CANONICAL_HAND_ORDER)) - active)
    unexpected = sorted(active - (target_set & set(CANONICAL_HAND_ORDER)))

    hand_delta = np.zeros(len(CANONICAL_HAND_ORDER), dtype=np.float64)
    reasons: list[str] = []
    if missing:
        reasons.append("missing_expected_contacts")
        for idx, name in enumerate(CANONICAL_HAND_ORDER):
            if name in missing:
                hand_delta[idx] = max_hand_delta
    if unexpected and not missing:
        reasons.append("unexpected_contact_bias")
        for idx, name in enumerate(CANONICAL_HAND_ORDER):
            if name in unexpected:
                hand_delta[idx] = -0.5 * max_hand_delta

    wrist_delta = np.zeros(3, dtype=np.float64)
    left_force = forces[0] + forces[1]
    right_force = forces[2] + forces[3]
    thumb_force = forces[4] + forces[5]
    if max(left_force, right_force, thumb_force) > force_threshold and abs(left_force - right_force) > force_threshold:
        wrist_delta[1] = -0.003 if left_force > right_force else 0.003
        reasons.append("lateral_force_imbalance")
    if thumb_force > 1.8 * max(left_force + right_force, 1.0):
        wrist_delta[0] = -0.003
        reasons.append("thumb_dominant_contact")

    if not reasons and active:
        status = "contact_ok"
        confidence = 0.85
    elif missing:
        status = "close_or_reseat"
        confidence = 0.45
    elif unexpected:
        status = "relieve_contact"
        confidence = 0.55
    else:
        status = "seek_contact"
        confidence = 0.25
        hand_delta[:] = max_hand_delta
        reasons.append("no_contact")

    return TactileCorrection(
        confidence=confidence,
        hand_delta=hand_delta.astype(float).tolist(),
        wrist_delta_xyz_m=wrist_delta.astype(float).tolist(),
        status=status,
        reasons=reasons,
    )


def _canonical_contact_flags(state: Mapping[str, Any]) -> list[bool]:
    raw = state.get("contact_binary", state.get("contact_flags"))
    if raw is None:
        return [False] * len(CANONICAL_HAND_ORDER)
    if len(raw) != len(CANONICAL_HAND_ORDER):
        raise ValueError(f"Expected {len(CANONICAL_HAND_ORDER)} contact flags, got {len(raw)}.")
    return [bool(value) for value in raw]


def _canonical_forces(state: Mapping[str, Any]) -> np.ndarray:
    raw = state.get("forces", state.get("force_estimate"))
    if raw is None:
        return np.zeros(len(CANONICAL_HAND_ORDER), dtype=np.float64)
    array = np.asarray(raw, dtype=np.float64).reshape(-1)
    if array.size != len(CANONICAL_HAND_ORDER):
        raise ValueError(f"Expected {len(CANONICAL_HAND_ORDER)} force values, got {array.size}.")
    return array
