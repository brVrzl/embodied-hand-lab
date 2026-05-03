from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER, denormalize_canonical

UNIFUC_INSPIRE_TARGET_DIM = 19
UNIFUC_INSPIRE_WRIST_DIM = 7

UNIFUC_INSPIRE_JOINT_ORDER: tuple[str, ...] = (
    "right_thumb_1_joint",
    "right_thumb_2_joint",
    "right_thumb_3_joint",
    "right_thumb_4_joint",
    "right_index_1_joint",
    "right_index_2_joint",
    "right_middle_1_joint",
    "right_middle_2_joint",
    "right_ring_1_joint",
    "right_ring_2_joint",
    "right_little_1_joint",
    "right_little_2_joint",
)

# Joint limits from UniFucGrasp's InspireHand force-sensor URDF.
UNIFUC_INSPIRE_JOINT_LOWER = np.asarray([0.0] * 12, dtype=np.float32)
UNIFUC_INSPIRE_JOINT_UPPER = np.asarray(
    [
        1.1641,
        0.5864,
        0.5,
        3.14,
        1.4381,
        3.14,
        1.4381,
        3.14,
        1.4381,
        3.14,
        1.4381,
        3.14,
    ],
    dtype=np.float32,
)


@dataclass(frozen=True, slots=True)
class UniFucInspirePose:
    position_m: list[float]
    quat_wxyz: list[float]
    joints_12d: list[float]
    rh56_canonical_norm: list[float]
    rh56_raw_order: list[float]


def _as_array(values: Sequence[float] | np.ndarray, *, expected: int) -> np.ndarray:
    array = np.asarray(values, dtype=np.float32).reshape(-1)
    if array.size != expected:
        raise ValueError(f"Expected {expected} values, got {array.size}.")
    return array


def normalize_unifuc_inspire_joints(joints_12d: Sequence[float] | np.ndarray) -> np.ndarray:
    joints = _as_array(joints_12d, expected=len(UNIFUC_INSPIRE_JOINT_ORDER))
    denom = np.maximum(UNIFUC_INSPIRE_JOINT_UPPER - UNIFUC_INSPIRE_JOINT_LOWER, 1e-6)
    return np.clip((joints - UNIFUC_INSPIRE_JOINT_LOWER) / denom, 0.0, 1.0)


def inspire12_to_rh56_canonical_norm(joints_12d: Sequence[float] | np.ndarray) -> list[float]:
    """Map UniFucGrasp's 12D Inspire joint pose to this project's 6D RH56 canonical command.

    The mapping is intentionally conservative. It uses normalized joint closure
    averages for each coupled finger group, preserving the project-level order:
    index, middle, ring, pinky, thumb_close, thumb_lateral.
    """

    norm = normalize_unifuc_inspire_joints(joints_12d)
    thumb_lateral = norm[0]
    thumb_close = float(np.mean(norm[[1, 2, 3]]))
    index = float(np.mean(norm[[4, 5]]))
    middle = float(np.mean(norm[[6, 7]]))
    ring = float(np.mean(norm[[8, 9]]))
    pinky = float(np.mean(norm[[10, 11]]))
    return [index, middle, ring, pinky, thumb_close, float(thumb_lateral)]


def parse_unifuc_inspire_target(target_q: Sequence[float] | np.ndarray) -> UniFucInspirePose:
    target = _as_array(target_q, expected=UNIFUC_INSPIRE_TARGET_DIM)
    position_m = target[:3].astype(np.float32).tolist()
    quat_wxyz = target[3:7].astype(np.float32).tolist()
    joints = target[7:].astype(np.float32)
    rh56_norm = inspire12_to_rh56_canonical_norm(joints)
    rh56_raw = denormalize_canonical(rh56_norm)
    return UniFucInspirePose(
        position_m=position_m,
        quat_wxyz=quat_wxyz,
        joints_12d=joints.tolist(),
        rh56_canonical_norm=rh56_norm,
        rh56_raw_order=rh56_raw,
    )


def mapping_metadata() -> dict[str, object]:
    return {
        "source": "UniFucGrasp InspireHand 12D joint-level annotations",
        "target": "project RH56 canonical 6D normalized command",
        "source_joint_order": list(UNIFUC_INSPIRE_JOINT_ORDER),
        "source_joint_lower": UNIFUC_INSPIRE_JOINT_LOWER.tolist(),
        "source_joint_upper": UNIFUC_INSPIRE_JOINT_UPPER.tolist(),
        "target_canonical_order": list(CANONICAL_HAND_ORDER),
        "groups": {
            "index": ["right_index_1_joint", "right_index_2_joint"],
            "middle": ["right_middle_1_joint", "right_middle_2_joint"],
            "ring": ["right_ring_1_joint", "right_ring_2_joint"],
            "pinky": ["right_little_1_joint", "right_little_2_joint"],
            "thumb_close": ["right_thumb_2_joint", "right_thumb_3_joint", "right_thumb_4_joint"],
            "thumb_lateral": ["right_thumb_1_joint"],
        },
        "warning": "Approximate mapping for dataset bootstrapping; validate against real RH56 photos and replay before sim-to-real claims.",
    }
