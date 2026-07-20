from __future__ import annotations

import math

import numpy as np

from ..contracts import ArmPoseSample, Pose3D
from ..transforms.se3 import (
    quaternion_conjugate,
    quaternion_log,
    quaternion_multiply,
    slerp,
)


def _alpha(cutoff_hz: float, dt_s: float) -> float:
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return 1.0 / (1.0 + tau / dt_s)


class OneEuroSE3Filter:
    """Timestamp-aware One Euro translation and geodesic SO(3) filter."""

    def __init__(
        self,
        *,
        translation_min_cutoff_hz: float = 2.0,
        translation_beta_s_m: float = 30.0,
        rotation_min_cutoff_hz: float = 2.0,
        rotation_beta_s_rad: float = 3.0,
        derivative_cutoff_hz: float = 1.0,
        minimum_dt_s: float = 1e-4,
        maximum_dt_s: float = 0.1,
    ) -> None:
        values = (
            translation_min_cutoff_hz,
            rotation_min_cutoff_hz,
            derivative_cutoff_hz,
            minimum_dt_s,
            maximum_dt_s,
        )
        if min(values) <= 0.0 or translation_beta_s_m < 0.0 or rotation_beta_s_rad < 0.0:
            raise ValueError("One Euro parameters are invalid")
        if minimum_dt_s >= maximum_dt_s:
            raise ValueError("minimum_dt_s must be less than maximum_dt_s")
        self.translation_min_cutoff_hz = float(translation_min_cutoff_hz)
        self.translation_beta_s_m = float(translation_beta_s_m)
        self.rotation_min_cutoff_hz = float(rotation_min_cutoff_hz)
        self.rotation_beta_s_rad = float(rotation_beta_s_rad)
        self.derivative_cutoff_hz = float(derivative_cutoff_hz)
        self.minimum_dt_s = float(minimum_dt_s)
        self.maximum_dt_s = float(maximum_dt_s)
        self.reset()

    def reset(self) -> None:
        self._timestamp_ns: int | None = None
        self._raw_position: np.ndarray | None = None
        self._filtered_position: np.ndarray | None = None
        self._raw_quaternion: tuple[float, float, float, float] | None = None
        self._filtered_quaternion: tuple[float, float, float, float] | None = None
        self._translation_derivative = np.zeros(3)
        self._angular_derivative = np.zeros(3)

    def filter(self, sample: ArmPoseSample) -> ArmPoseSample:
        timestamp = sample.timestamps.local_receive_ns
        position = np.asarray(sample.pose.position_m, dtype=np.float64)
        quaternion = sample.pose.quaternion_xyzw
        if self._timestamp_ns is None:
            self._timestamp_ns = timestamp
            self._raw_position = position
            self._filtered_position = position
            self._raw_quaternion = quaternion
            self._filtered_quaternion = quaternion
            return sample
        dt_raw = (timestamp - self._timestamp_ns) / 1e9
        if dt_raw <= 0.0 or dt_raw > self.maximum_dt_s:
            self.reset()
            return self.filter(sample)
        dt = min(self.maximum_dt_s, max(self.minimum_dt_s, dt_raw))
        assert self._raw_position is not None
        assert self._filtered_position is not None
        assert self._raw_quaternion is not None
        assert self._filtered_quaternion is not None

        derivative_alpha = _alpha(self.derivative_cutoff_hz, dt)
        raw_velocity = (position - self._raw_position) / dt
        self._translation_derivative += derivative_alpha * (
            raw_velocity - self._translation_derivative
        )
        translation_cutoff = self.translation_min_cutoff_hz + self.translation_beta_s_m * float(
            np.linalg.norm(self._translation_derivative)
        )
        translation_alpha = _alpha(translation_cutoff, dt)
        filtered_position = self._filtered_position + translation_alpha * (
            position - self._filtered_position
        )

        raw_delta = quaternion_multiply(quaternion, quaternion_conjugate(self._raw_quaternion))
        raw_angular_velocity = quaternion_log(raw_delta) / dt
        self._angular_derivative += derivative_alpha * (
            raw_angular_velocity - self._angular_derivative
        )
        rotation_cutoff = self.rotation_min_cutoff_hz + self.rotation_beta_s_rad * float(
            np.linalg.norm(self._angular_derivative)
        )
        filtered_quaternion = slerp(
            self._filtered_quaternion,
            quaternion,
            _alpha(rotation_cutoff, dt),
        )

        self._timestamp_ns = timestamp
        self._raw_position = position
        self._filtered_position = filtered_position
        self._raw_quaternion = quaternion
        self._filtered_quaternion = filtered_quaternion
        return ArmPoseSample(
            source_id=sample.source_id,
            sequence=sample.sequence,
            frame_id=sample.frame_id,
            pose=Pose3D(tuple(float(value) for value in filtered_position), filtered_quaternion),
            timestamps=sample.timestamps,
            tracking_valid=sample.tracking_valid,
            tracking_quality=sample.tracking_quality,
            tracking_state=sample.tracking_state,
            validity_reason=sample.validity_reason,
            sample_age_ns=sample.sample_age_ns,
            connection_epoch=sample.connection_epoch,
            discontinuity=sample.discontinuity,
            source_sequence=sample.source_sequence,
        )
