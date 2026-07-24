from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np


DEFAULT_OUTPUT_AXES = ("x", "y", "z")


@dataclass(frozen=True, slots=True)
class AxisMapping:
    output: str
    source: str
    sign: float = 1.0
    scale: float = 1.0

    def apply(self, values: Mapping[str, float]) -> float:
        return float(values.get(self.source, 0.0)) * float(self.sign) * float(self.scale)

    def to_dict(self) -> dict[str, float | str]:
        return {
            "output": self.output,
            "source": self.source,
            "sign": self.sign,
            "scale": self.scale,
        }


DEFAULT_PHONE_TO_ROBOT_TRANSLATION_MAP: tuple[AxisMapping, ...] = (
    AxisMapping("x", "y", -1.0),
    AxisMapping("y", "x", 1.0),
    AxisMapping("z", "z", 1.0),
)
DEFAULT_PHONE_WRIST_ROLL_MAP = AxisMapping("wrist_roll", "rot_z", -1.0)


def parse_axis_mapping(
    output: str,
    payload: Mapping[str, Any] | str,
    *,
    default_source: str | None = None,
) -> AxisMapping:
    if isinstance(payload, str):
        return AxisMapping(output=output, source=payload)
    if not isinstance(payload, Mapping):
        raise ValueError(f"Axis mapping for {output!r} must be a mapping or source string.")
    source = str(payload.get("source", default_source or output))
    return AxisMapping(
        output=str(payload.get("output", output)),
        source=source,
        sign=float(payload.get("sign", 1.0)),
        scale=float(payload.get("scale", 1.0)),
    )


def parse_vector_axis_map(
    payload: Mapping[str, Any] | None,
    *,
    default: Sequence[AxisMapping],
    output_axes: Sequence[str] = DEFAULT_OUTPUT_AXES,
) -> tuple[AxisMapping, ...]:
    if payload is None:
        return tuple(default)
    mappings: list[AxisMapping] = []
    for axis in output_axes:
        if axis in payload:
            mappings.append(parse_axis_mapping(axis, payload[axis], default_source=axis))
        else:
            fallback = next((item for item in default if item.output == axis), None)
            if fallback is None:
                raise ValueError(f"Missing axis mapping for output {axis!r}.")
            mappings.append(fallback)
    return tuple(mappings)


def parse_scalar_axis_map(
    payload: Mapping[str, Any] | str | None,
    *,
    default: AxisMapping,
) -> AxisMapping:
    if payload is None:
        return default
    return parse_axis_mapping(default.output, payload, default_source=default.source)


def apply_vector_axis_map(
    values: Mapping[str, float],
    mappings: Sequence[AxisMapping],
    *,
    output_axes: Sequence[str] = DEFAULT_OUTPUT_AXES,
) -> np.ndarray:
    by_output = {mapping.output: mapping for mapping in mappings}
    return np.asarray([by_output[axis].apply(values) for axis in output_axes], dtype=np.float64)


def axis_map_to_config(mappings: Sequence[AxisMapping]) -> dict[str, dict[str, float | str]]:
    return {mapping.output: mapping.to_dict() for mapping in mappings}
