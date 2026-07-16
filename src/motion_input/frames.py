"""Central coordinate-frame definitions and explicit basis conversions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable

from .errors import ProtocolValidationError
from .model import Pose6D


class Handedness(str, Enum):
    RIGHT = "right"
    LEFT = "left"


@dataclass(frozen=True, slots=True)
class FrameDefinition:
    frame_id: str
    parent_frame_id: str | None
    handedness: Handedness
    x_axis: str
    y_axis: str
    z_axis: str
    origin: str
    units: str = "meter"
    dynamic: bool = False

    def __post_init__(self) -> None:
        if not self.frame_id.strip():
            raise ProtocolValidationError("frame_id must not be empty")
        if self.units != "meter":
            raise ProtocolValidationError("UMIP pose frames must use meters")


class FrameRegistry:
    """Rejects ambiguous frame reuse and keeps frame facts in one place."""

    def __init__(self, definitions: Iterable[FrameDefinition] = ()) -> None:
        self._definitions: dict[str, FrameDefinition] = {}
        for definition in definitions:
            self.register(definition)

    def register(self, definition: FrameDefinition) -> None:
        existing = self._definitions.get(definition.frame_id)
        if existing is not None and existing != definition:
            raise ProtocolValidationError(
                f"frame {definition.frame_id!r} is already registered differently"
            )
        self._definitions[definition.frame_id] = definition

    def get(self, frame_id: str) -> FrameDefinition:
        try:
            return self._definitions[frame_id]
        except KeyError as exc:
            raise ProtocolValidationError(f"unknown coordinate frame {frame_id!r}") from exc

    def __contains__(self, frame_id: str) -> bool:
        return frame_id in self._definitions

    def definitions(self) -> tuple[FrameDefinition, ...]:
        return tuple(self._definitions.values())


OPENXR_AXES = FrameDefinition(
    frame_id="openxr/reference",
    parent_frame_id=None,
    handedness=Handedness.RIGHT,
    x_axis="right",
    y_axis="up",
    z_axis="backward (forward is -Z)",
    origin="defined by the selected OpenXR reference space",
)

UNITY_AXES = FrameDefinition(
    frame_id="unity/world",
    parent_frame_id=None,
    handedness=Handedness.LEFT,
    x_axis="right",
    y_axis="up",
    z_axis="forward",
    origin="Unity scene origin",
)


def unity_to_openxr_pose(pose: Pose6D) -> Pose6D:
    """Reflect a Unity pose into the OpenXR right-handed basis.

    This is a coordinate-basis conversion only. It performs no calibration,
    smoothing, scaling, prediction, or control transform.
    """

    x, y, z = pose.position_m
    qx, qy, qz, qw = pose.orientation_xyzw
    return Pose6D(
        position_m=(x, y, -z),
        orientation_xyzw=(-qx, -qy, qz, qw),
    )


def openxr_to_unity_pose(pose: Pose6D) -> Pose6D:
    """Inverse of :func:`unity_to_openxr_pose`."""

    return unity_to_openxr_pose(pose)
