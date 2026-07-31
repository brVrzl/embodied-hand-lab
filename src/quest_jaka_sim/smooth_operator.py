"""Configuration for the shared SE(3) filters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

@dataclass(frozen=True, slots=True)
class Se3FilterProfile:
    name: str
    translation_min_cutoff: float
    translation_beta: float
    translation_derivative_cutoff: float
    rotation_min_cutoff: float
    rotation_beta: float
    rotation_derivative_cutoff: float
    maximum_filter_dt: float

    @classmethod
    def from_mapping(cls, name: str, values: Mapping[str, Any]) -> "Se3FilterProfile":
        return cls(
            name=name,
            translation_min_cutoff=float(values["translation_min_cutoff"]),
            translation_beta=float(values["translation_beta"]),
            translation_derivative_cutoff=float(values["translation_derivative_cutoff"]),
            rotation_min_cutoff=float(values["rotation_min_cutoff"]),
            rotation_beta=float(values["rotation_beta"]),
            rotation_derivative_cutoff=float(values["rotation_derivative_cutoff"]),
            maximum_filter_dt=float(values["maximum_filter_dt"]),
        )
