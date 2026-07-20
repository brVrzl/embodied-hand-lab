from __future__ import annotations

import enum
import math
from dataclasses import dataclass

import numpy as np

from ..contracts import ArmPoseSample, DiscontinuityKind, TrackingState
from ..sequence import SequenceDisposition, SequenceTracker
from ..transforms.se3 import quaternion_angle


class ValidationAction(str, enum.Enum):
    ACCEPT = "accept"
    WARNING = "warning"
    HOLD = "hold"
    CONTROLLED_STOP = "controlled_stop"
    ABORT = "abort"
    RECLUTCH_REQUIRED = "reclutch_required"
    REJECT = "reject"


@dataclass(frozen=True, slots=True)
class ValidationResult:
    action: ValidationAction
    reason: str
    sample: ArmPoseSample | None


@dataclass(frozen=True, slots=True)
class PoseValidationConfig:
    expected_frame_id: str
    warning_age_ns: int = 40_000_000
    hold_age_ns: int = 100_000_000
    controlled_stop_age_ns: int = 500_000_000
    fatal_age_ns: int = 2_000_000_000
    maximum_translation_jump_m: float = 0.20
    maximum_rotation_jump_rad: float = math.radians(35.0)
    maximum_linear_speed_m_s: float = 3.0
    maximum_angular_speed_rad_s: float = math.radians(720.0)

    def __post_init__(self) -> None:
        ages = (self.warning_age_ns, self.hold_age_ns, self.controlled_stop_age_ns, self.fatal_age_ns)
        if ages[0] < 0 or not all(left < right for left, right in zip(ages, ages[1:])):
            raise ValueError("pose age thresholds must be strictly increasing")
        if not self.expected_frame_id:
            raise ValueError("expected_frame_id is required")
        if min(
            self.maximum_translation_jump_m,
            self.maximum_rotation_jump_rad,
            self.maximum_linear_speed_m_s,
            self.maximum_angular_speed_rad_s,
        ) <= 0.0:
            raise ValueError("pose discontinuity limits must be positive")


class PoseValidator:
    def __init__(self, config: PoseValidationConfig) -> None:
        self.config = config
        self._sequence = SequenceTracker()
        self._source_sequence = SequenceTracker()
        self._previous: ArmPoseSample | None = None
        self._previous_tracking_valid = False
        self._connection_epoch: int | None = None

    def reset(self) -> None:
        self._sequence.reset()
        self._source_sequence.reset()
        self._previous = None
        self._previous_tracking_valid = False
        self._connection_epoch = None

    def validate(self, sample: ArmPoseSample | None, *, now_ns: int) -> ValidationResult:
        if sample is None:
            return ValidationResult(ValidationAction.HOLD, "no_sample", None)
        if sample.frame_id != self.config.expected_frame_id:
            return ValidationResult(ValidationAction.ABORT, "unexpected_source_frame", sample)
        disposition = self._sequence.observe(sample.sequence)
        if disposition == SequenceDisposition.DUPLICATE:
            return ValidationResult(ValidationAction.REJECT, "duplicate_sequence", sample)
        if disposition == SequenceDisposition.REORDERED:
            return ValidationResult(ValidationAction.REJECT, "reordered_sequence", sample)
        if sample.source_sequence is not None:
            source_disposition = self._source_sequence.observe(sample.source_sequence)
            if source_disposition == SequenceDisposition.DUPLICATE:
                return ValidationResult(ValidationAction.REJECT, "duplicate_source_sequence", sample)
            if source_disposition == SequenceDisposition.REORDERED:
                return ValidationResult(ValidationAction.RECLUTCH_REQUIRED, "source_sequence_reset", sample)

        receive_ns = sample.timestamps.local_receive_ns
        if receive_ns > now_ns + 5_000_000:
            return ValidationResult(ValidationAction.ABORT, "future_receive_timestamp", sample)
        age = max(sample.sample_age_ns, max(0, now_ns - receive_ns))
        if age >= self.config.fatal_age_ns:
            return ValidationResult(ValidationAction.ABORT, "fatal_sample_age", sample)
        if age >= self.config.controlled_stop_age_ns:
            return ValidationResult(ValidationAction.CONTROLLED_STOP, "prolonged_dropout", sample)
        if age >= self.config.hold_age_ns:
            return ValidationResult(ValidationAction.HOLD, "short_dropout", sample)

        if not sample.tracking_valid or sample.tracking_state == TrackingState.INVALID:
            self._previous = sample
            self._previous_tracking_valid = False
            return ValidationResult(ValidationAction.RECLUTCH_REQUIRED, "tracking_invalid", sample)
        if self._connection_epoch is not None and sample.connection_epoch != self._connection_epoch:
            self._connection_epoch = sample.connection_epoch
            self._previous = sample
            self._previous_tracking_valid = True
            return ValidationResult(ValidationAction.RECLUTCH_REQUIRED, "connection_epoch_changed", sample)
        self._connection_epoch = sample.connection_epoch
        if sample.discontinuity in {
            DiscontinuityKind.RECONNECT,
            DiscontinuityKind.SEQUENCE_RESET,
            DiscontinuityKind.TRACKING_RECOVERY,
            DiscontinuityKind.RELOCALIZATION,
        }:
            self._previous = sample
            self._previous_tracking_valid = True
            return ValidationResult(
                ValidationAction.RECLUTCH_REQUIRED,
                f"discontinuity:{sample.discontinuity.value}",
                sample,
            )
        if not self._previous_tracking_valid and self._previous is not None:
            self._previous = sample
            self._previous_tracking_valid = True
            return ValidationResult(ValidationAction.RECLUTCH_REQUIRED, "tracking_recovered", sample)

        previous = self._previous
        if previous is not None:
            dt_ns = receive_ns - previous.timestamps.local_receive_ns
            if dt_ns <= 0:
                return ValidationResult(ValidationAction.REJECT, "nonmonotonic_receive_timestamp", sample)
            dt = dt_ns / 1e9
            translation = float(
                np.linalg.norm(np.asarray(sample.pose.position_m) - np.asarray(previous.pose.position_m))
            )
            rotation = quaternion_angle(sample.pose.quaternion_xyzw, previous.pose.quaternion_xyzw)
            if translation > self.config.maximum_translation_jump_m:
                return ValidationResult(ValidationAction.RECLUTCH_REQUIRED, "translation_discontinuity", sample)
            if rotation > self.config.maximum_rotation_jump_rad:
                return ValidationResult(ValidationAction.RECLUTCH_REQUIRED, "rotation_discontinuity", sample)
            if translation / dt > self.config.maximum_linear_speed_m_s:
                return ValidationResult(ValidationAction.RECLUTCH_REQUIRED, "impossible_linear_speed", sample)
            if rotation / dt > self.config.maximum_angular_speed_rad_s:
                return ValidationResult(ValidationAction.RECLUTCH_REQUIRED, "impossible_angular_speed", sample)

        self._previous = sample
        self._previous_tracking_valid = True
        if age >= self.config.warning_age_ns:
            return ValidationResult(ValidationAction.WARNING, "sample_age_warning", sample)
        return ValidationResult(ValidationAction.ACCEPT, "ok", sample)
