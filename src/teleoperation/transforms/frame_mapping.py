from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from ..contracts import Pose3D
from .se3 import (
    compose_pose,
    matrix_to_quaternion_xyzw,
    quaternion_exp,
    quaternion_log,
    quaternion_multiply,
    quaternion_to_matrix,
)


@dataclass(frozen=True, slots=True)
class CentralFrameMapping:
    """Single owner of source-axis, handedness, and control-frame transforms.

    ``source_to_normalized_basis`` maps source-world vectors into the normalized
    operator frame.  It may have determinant -1 for an explicit handedness
    conversion; orientations use ``C R C^-1`` and therefore remain proper.

    Frame chain:
    ``TeleDex source -> normalized input -> operator control -> robot base -> TCP``.
    The output target is the configured JAKA TCP (tool/user IDs are checked at
    the native boundary).  No axis swap is permitted downstream of this class.
    """

    source_frame_id: str
    normalized_frame_id: str
    robot_base_frame_id: str
    tcp_frame_id: str
    source_to_normalized_basis: tuple[tuple[float, float, float], ...]
    device_to_operator_control: Pose3D = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def __post_init__(self) -> None:
        if not all(
            isinstance(value, str) and value.strip()
            for value in (
                self.source_frame_id,
                self.normalized_frame_id,
                self.robot_base_frame_id,
                self.tcp_frame_id,
            )
        ):
            raise ValueError("frame identifiers must be non-empty")
        basis = np.asarray(self.source_to_normalized_basis, dtype=np.float64)
        if basis.shape != (3, 3) or not np.all(np.isfinite(basis)):
            raise ValueError("source_to_normalized_basis must be finite 3x3")
        if not np.allclose(basis @ basis.T, np.eye(3), atol=1e-6):
            raise ValueError("source_to_normalized_basis must be orthogonal")
        if not math.isclose(abs(float(np.linalg.det(basis))), 1.0, abs_tol=1e-6):
            raise ValueError("source_to_normalized_basis determinant must be +/-1")

    @property
    def basis(self) -> np.ndarray:
        return np.asarray(self.source_to_normalized_basis, dtype=np.float64)

    def source_pose_to_operator(self, pose: Pose3D) -> Pose3D:
        basis = self.basis
        position = basis @ np.asarray(pose.position_m)
        rotation = basis @ quaternion_to_matrix(pose.quaternion_xyzw) @ basis.T
        normalized = Pose3D(
            tuple(float(value) for value in position),
            matrix_to_quaternion_xyzw(rotation),
        )
        return compose_pose(normalized, self.device_to_operator_control)

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "CentralFrameMapping":
        extrinsic = payload.get("device_to_operator_control", {})
        return cls(
            source_frame_id=str(payload["source_frame_id"]),
            normalized_frame_id=str(payload["normalized_frame_id"]),
            robot_base_frame_id=str(payload["robot_base_frame_id"]),
            tcp_frame_id=str(payload["tcp_frame_id"]),
            source_to_normalized_basis=tuple(
                tuple(float(value) for value in row)
                for row in payload["source_to_normalized_basis"]
            ),
            device_to_operator_control=Pose3D(
                tuple(float(value) for value in extrinsic.get("position_m", (0.0, 0.0, 0.0))),
                tuple(
                    float(value)
                    for value in extrinsic.get("quaternion_xyzw", (0.0, 0.0, 0.0, 1.0))
                ),
            ),
        )


class RelativePoseMapper:
    def __init__(
        self,
        frames: CentralFrameMapping,
        *,
        translation_scale: float,
        rotation_scale: float,
    ) -> None:
        if not 0.0 <= translation_scale <= 0.10:
            raise ValueError("translation_scale must be in [0, 0.10]")
        if not 0.0 <= rotation_scale <= 0.10:
            raise ValueError("rotation_scale must be in [0, 0.10]")
        self.frames = frames
        self.translation_scale = float(translation_scale)
        self.rotation_scale = float(rotation_scale)
        self._source_anchor: Pose3D | None = None
        self._robot_anchor: Pose3D | None = None
        self.anchor_id = 0

    @property
    def anchored(self) -> bool:
        return self._source_anchor is not None and self._robot_anchor is not None

    def clear(self) -> None:
        self._source_anchor = None
        self._robot_anchor = None

    def anchor(self, source_pose: Pose3D, robot_tcp_pose: Pose3D) -> int:
        return self.anchor_operator(self.frames.source_pose_to_operator(source_pose), robot_tcp_pose)

    def anchor_operator(self, operator_pose: Pose3D, robot_tcp_pose: Pose3D) -> int:
        self._source_anchor = operator_pose
        self._robot_anchor = robot_tcp_pose
        self.anchor_id += 1
        return self.anchor_id

    def map(self, source_pose: Pose3D) -> Pose3D:
        return self.map_operator(self.frames.source_pose_to_operator(source_pose))

    def map_operator(self, current: Pose3D) -> Pose3D:
        if self._source_anchor is None or self._robot_anchor is None:
            raise RuntimeError("relative mapper has no clutch anchor")
        delta_position = np.asarray(current.position_m) - np.asarray(self._source_anchor.position_m)
        target_position = np.asarray(self._robot_anchor.position_m) + self.translation_scale * delta_position

        current_rotation = quaternion_to_matrix(current.quaternion_xyzw)
        anchor_rotation = quaternion_to_matrix(self._source_anchor.quaternion_xyzw)
        world_delta = current_rotation @ anchor_rotation.T
        delta_q = matrix_to_quaternion_xyzw(world_delta)
        scaled_delta_q = quaternion_exp(quaternion_log(delta_q) * self.rotation_scale)
        target_q = quaternion_multiply(scaled_delta_q, self._robot_anchor.quaternion_xyzw)
        return Pose3D(tuple(float(value) for value in target_position), target_q)
