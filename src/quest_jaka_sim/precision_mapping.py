"""Latched-head-yaw, full relative-SE(3) arm mapping primitives."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from motion_input import Pose6D

from .mapping import ProvisionalMappingConfig
from .se3 import (
    OneEuroQuaternionFilter,
    OneEuroVectorFilter,
    compose_pose,
    matrix_to_quaternion_xyzw,
    quaternion_angle_rad,
    quaternion_to_matrix,
    relative_pose,
)
from .smooth_operator import Se3FilterProfile


@dataclass(frozen=True, slots=True)
class ArmMappingTelemetry:
    hand_local_delta: Pose6D
    horizontal_delta: Pose6D
    robot_delta: Pose6D
    comfort_translation_warning: bool
    comfort_rotation_warning: bool


def gravity_aligned_head_yaw(head_orientation_xyzw: tuple[float, float, float, float]) -> tuple[float, np.ndarray]:
    """Return yaw and ``R_quest_horizontal`` for a Y-up Quest world.

    Quest/OpenXR local forward is -Z.  Its projection into the gravity-normal
    XZ plane defines yaw.  If forward is nearly vertical, projected local +X is
    used instead.  No Euler pitch/roll subtraction and no head translation are
    involved.
    """

    rotation = quaternion_to_matrix(head_orientation_xyzw)
    forward = rotation @ np.asarray((0.0, 0.0, -1.0))
    horizontal = np.asarray((forward[0], 0.0, forward[2]))
    if float(np.linalg.norm(horizontal)) < 1e-8:
        right = rotation @ np.asarray((1.0, 0.0, 0.0))
        right_horizontal = np.asarray((right[0], 0.0, right[2]))
        if float(np.linalg.norm(right_horizontal)) < 1e-8:
            raise ValueError("head yaw is undefined because horizontal axes are degenerate")
        right_horizontal /= np.linalg.norm(right_horizontal)
        horizontal = np.asarray((right_horizontal[2], 0.0, -right_horizontal[0]))
    horizontal /= np.linalg.norm(horizontal)
    yaw = math.atan2(-float(horizontal[0]), -float(horizontal[2]))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    # Active rotation whose columns are horizontal-frame axes in Quest world.
    yaw_rotation = np.asarray(
        ((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine)),
        dtype=np.float64,
    )
    return yaw, yaw_rotation


class LatchedHeadYawArmMapper:
    """Maps a filtered wrist delta onto a captured authoritative robot TCP."""

    def __init__(self, config: ProvisionalMappingConfig, profile: Se3FilterProfile) -> None:
        self.config = config
        self.profile = profile
        self._position_filter = OneEuroVectorFilter(
            min_cutoff_hz=profile.translation_min_cutoff,
            beta=profile.translation_beta,
            derivative_cutoff_hz=profile.translation_derivative_cutoff,
            maximum_dt_s=profile.maximum_filter_dt,
        )
        self._rotation_filter = OneEuroQuaternionFilter(
            min_cutoff_hz=profile.rotation_min_cutoff,
            beta=profile.rotation_beta,
            derivative_cutoff_hz=profile.rotation_derivative_cutoff,
            maximum_dt_s=profile.maximum_filter_dt,
        )
        self.hand_reference: Pose6D | None = None
        self.robot_reference: Pose6D | None = None
        self.latched_head_yaw_rad: float | None = None
        self._yaw_rotation: np.ndarray | None = None
        self.raw_wrist: Pose6D | None = None
        self.filtered_wrist: Pose6D | None = None
        self.last_telemetry: ArmMappingTelemetry | None = None

    def clear(self) -> None:
        self.hand_reference = None
        self.robot_reference = None
        self.latched_head_yaw_rad = None
        self._yaw_rotation = None
        self.filtered_wrist = None
        self.last_telemetry = None
        self._position_filter.reset()
        self._rotation_filter.reset()

    def capture(self, *, wrist: Pose6D, robot_tcp: Pose6D, head: Pose6D, timestamp_ns: int) -> Pose6D:
        yaw, yaw_rotation = gravity_aligned_head_yaw(head.orientation_xyzw)
        self._position_filter.reset()
        self._rotation_filter.reset()
        position = self._position_filter.filter(timestamp_ns, wrist.position_m)
        orientation = self._rotation_filter.filter(timestamp_ns, wrist.orientation_xyzw)
        filtered = Pose6D(tuple(float(v) for v in position), orientation)
        self.raw_wrist = wrist
        self.filtered_wrist = filtered
        self.hand_reference = filtered
        self.robot_reference = robot_tcp
        self.latched_head_yaw_rad = yaw
        self._yaw_rotation = yaw_rotation
        identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.last_telemetry = ArmMappingTelemetry(identity, identity, identity, False, False)
        return robot_tcp

    def target(self, wrist: Pose6D, *, timestamp_ns: int) -> Pose6D:
        if self.hand_reference is None or self.robot_reference is None or self._yaw_rotation is None:
            raise RuntimeError("arm references have not been captured")
        self.raw_wrist = wrist
        position = self._position_filter.filter(timestamp_ns, wrist.position_m)
        orientation = self._rotation_filter.filter(timestamp_ns, wrist.orientation_xyzw)
        current = Pose6D(tuple(float(v) for v in position), orientation)
        self.filtered_wrist = current

        # Required local relative transform: inv(T_q_ref_hand) @ T_q_current_hand.
        local_delta = relative_pose(self.hand_reference, current)

        # Express that same rigid displacement in the gravity-aligned horizontal
        # frame latched at engagement.  R_h_ref converts reference-hand-local
        # vectors to the latched horizontal frame.  This is a fixed change of
        # basis for the engagement; later head motion is never sampled here.
        r_q_ref = quaternion_to_matrix(self.hand_reference.orientation_xyzw)
        r_h_ref = self._yaw_rotation.T @ r_q_ref
        horizontal_translation = r_h_ref @ np.asarray(local_delta.position_m)
        horizontal_rotation = (
            r_h_ref @ quaternion_to_matrix(local_delta.orientation_xyzw) @ r_h_ref.T
        )
        horizontal_delta = Pose6D(
            tuple(float(v) for v in horizontal_translation),
            matrix_to_quaternion_xyzw(horizontal_rotation),
        )

        translation_gain = np.asarray(self.config.translation_scale_per_axis)
        rotation_gain = np.asarray(
            self.config.orientation_scale_per_axis
            or (self.config.orientation_scale,) * 3
        )
        if not np.allclose(translation_gain, 1.0) or not np.allclose(rotation_gain, 1.0):
            raise ValueError("precision default requires fixed translation and rotation gains of 1.0")
        robot_translation = np.asarray(self.config.operator_to_robot_basis) @ horizontal_translation
        rotation_basis = np.asarray(
            self.config.rotation_operator_to_robot_basis
            or self.config.operator_to_robot_basis
        )
        robot_rotation = rotation_basis @ horizontal_rotation @ rotation_basis.T
        robot_delta = Pose6D(
            tuple(float(v) for v in robot_translation),
            matrix_to_quaternion_xyzw(robot_rotation),
        )
        self.last_telemetry = ArmMappingTelemetry(
            local_delta,
            horizontal_delta,
            robot_delta,
            float(np.linalg.norm(horizontal_translation)) >= 0.8 * self.config.maximum_operator_displacement_m,
            quaternion_angle_rad(horizontal_delta.orientation_xyzw, (0.0, 0.0, 0.0, 1.0))
            >= 0.8 * self.config.maximum_relative_rotation_rad,
        )
        return compose_pose(self.robot_reference, robot_delta)
