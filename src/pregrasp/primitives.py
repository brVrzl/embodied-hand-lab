from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER


@dataclass(frozen=True, slots=True)
class PregraspPrimitive:
    name: str
    task_modes: tuple[str, ...]
    hand_command: list[float]
    wrist_offset_xyz_m: list[float]
    approach_axis_xyz: list[float]
    contact_strategy: str
    shape_tags: tuple[str, ...]
    min_object_width_m: float
    max_object_width_m: float
    expected_contacts: tuple[str, ...]
    notes: str = ""

    def __post_init__(self) -> None:
        if len(self.hand_command) != len(CANONICAL_HAND_ORDER):
            raise ValueError(f"{self.name}: hand_command must have {len(CANONICAL_HAND_ORDER)} values.")
        if len(self.wrist_offset_xyz_m) != 3:
            raise ValueError(f"{self.name}: wrist_offset_xyz_m must have 3 values.")
        if len(self.approach_axis_xyz) != 3:
            raise ValueError(f"{self.name}: approach_axis_xyz must have 3 values.")
        command = np.asarray(self.hand_command, dtype=np.float64)
        if not np.isfinite(command).all() or np.any(command < 0.0) or np.any(command > 1.0):
            raise ValueError(f"{self.name}: hand_command values must be normalized in [0, 1].")
        axis = np.asarray(self.approach_axis_xyz, dtype=np.float64)
        if np.linalg.norm(axis) < 1e-9:
            raise ValueError(f"{self.name}: approach_axis_xyz cannot be zero.")
        if self.min_object_width_m < 0.0 or self.max_object_width_m <= self.min_object_width_m:
            raise ValueError(f"{self.name}: invalid object width range.")

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["canonical_hand_order"] = list(CANONICAL_HAND_ORDER)
        return result


def rh56_default_primitives() -> list[PregraspPrimitive]:
    return [
        PregraspPrimitive(
            name="power_envelope",
            task_modes=("pick", "hold"),
            hand_command=[0.62, 0.68, 0.72, 0.72, 0.70, 0.42],
            wrist_offset_xyz_m=[-0.055, 0.0, 0.018],
            approach_axis_xyz=[1.0, 0.0, -0.15],
            contact_strategy="palm_envelope",
            shape_tags=("round", "box", "elongated"),
            min_object_width_m=0.030,
            max_object_width_m=0.095,
            expected_contacts=("thumb_close", "index", "middle", "ring", "pinky", "palm"),
            notes="Default robust RH56 grasp for ball-sized or box-like objects.",
        ),
        PregraspPrimitive(
            name="tripod_support",
            task_modes=("pick", "hold"),
            hand_command=[0.55, 0.56, 0.28, 0.22, 0.62, 0.52],
            wrist_offset_xyz_m=[-0.045, -0.006, 0.010],
            approach_axis_xyz=[1.0, -0.10, -0.05],
            contact_strategy="thumb_index_middle",
            shape_tags=("box", "round"),
            min_object_width_m=0.018,
            max_object_width_m=0.060,
            expected_contacts=("thumb_close", "index", "middle"),
            notes="Smaller-object grasp that avoids over-relying on coupled ring/pinky closure.",
        ),
        PregraspPrimitive(
            name="lateral_clamp",
            task_modes=("pick", "hold", "pre_align"),
            hand_command=[0.42, 0.45, 0.38, 0.32, 0.48, 0.70],
            wrist_offset_xyz_m=[-0.042, 0.018, 0.008],
            approach_axis_xyz=[1.0, 0.25, -0.05],
            contact_strategy="side_clamp",
            shape_tags=("flat", "box", "elongated"),
            min_object_width_m=0.010,
            max_object_width_m=0.055,
            expected_contacts=("thumb_lateral", "index", "middle"),
            notes="For thin or side-clamped objects; useful after pushing to expose an edge.",
        ),
        PregraspPrimitive(
            name="palm_push",
            task_modes=("push", "pre_align"),
            hand_command=[0.18, 0.18, 0.18, 0.18, 0.20, 0.38],
            wrist_offset_xyz_m=[-0.065, 0.0, 0.012],
            approach_axis_xyz=[1.0, 0.0, 0.0],
            contact_strategy="open_palm_push",
            shape_tags=("round", "box", "flat", "elongated"),
            min_object_width_m=0.010,
            max_object_width_m=0.140,
            expected_contacts=("palm",),
            notes="Nonprehensile pre-alignment primitive before attempting closure.",
        ),
        PregraspPrimitive(
            name="hook_pull",
            task_modes=("pull", "pre_align"),
            hand_command=[0.72, 0.70, 0.62, 0.55, 0.30, 0.36],
            wrist_offset_xyz_m=[0.035, 0.0, 0.014],
            approach_axis_xyz=[-1.0, 0.0, 0.0],
            contact_strategy="finger_hook",
            shape_tags=("box", "elongated", "flat"),
            min_object_width_m=0.012,
            max_object_width_m=0.090,
            expected_contacts=("index", "middle", "ring"),
            notes="Repositioning primitive for pulling an object into the reachable grasp zone.",
        ),
    ]


def load_primitive_config(path: str | Path) -> list[PregraspPrimitive]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, Mapping):
        raise ValueError("Primitive config must be a mapping.")
    primitives = data.get("primitives")
    if not isinstance(primitives, list):
        raise ValueError("Primitive config must contain a 'primitives' list.")
    return [_primitive_from_mapping(item) for item in primitives]


def _primitive_from_mapping(data: Mapping[str, Any]) -> PregraspPrimitive:
    return PregraspPrimitive(
        name=str(data["name"]),
        task_modes=_as_tuple(data["task_modes"]),
        hand_command=[float(value) for value in data["hand_command"]],
        wrist_offset_xyz_m=[float(value) for value in data["wrist_offset_xyz_m"]],
        approach_axis_xyz=[float(value) for value in data["approach_axis_xyz"]],
        contact_strategy=str(data["contact_strategy"]),
        shape_tags=_as_tuple(data["shape_tags"]),
        min_object_width_m=float(data["min_object_width_m"]),
        max_object_width_m=float(data["max_object_width_m"]),
        expected_contacts=_as_tuple(data["expected_contacts"]),
        notes=str(data.get("notes", "")),
    )


def _as_tuple(values: Iterable[object] | object) -> tuple[str, ...]:
    if isinstance(values, str):
        return (values,)
    if not isinstance(values, Sequence):
        raise ValueError(f"Expected a sequence of strings, got {values!r}.")
    return tuple(str(value) for value in values)
