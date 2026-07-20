from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(slots=True)
class JointState:
    names: list[str]
    positions: list[float]
    velocities: list[float] = field(default_factory=list)
    efforts: list[float] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class Pose:
    position: list[float]
    orientation_xyzw: list[float]
    frame_id: str = "base_link"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class HandState:
    mode: str
    finger_positions: list[float]
    finger_currents: list[float] = field(default_factory=list)
    contact_flags: list[bool] = field(default_factory=list)
    force_estimate: list[float] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class QuadrupedState:
    battery_percent: float
    mode: str
    estopped: bool
    position_xyz: list[float]
    orientation_xyzw: list[float]
    linear_velocity_xyz: list[float]
    angular_velocity_xyz: list[float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str
    distortion_model: str = "none"
    distortion_coefficients: list[float] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class ActionRecord:
    timestamp: float
    source: str
    action_type: str
    payload: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
