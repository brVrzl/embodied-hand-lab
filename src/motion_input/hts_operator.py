"""Offline right-hand validity and relative-pose preparation for Quest HTS.

This module terminates at a bounded operator-space transform.  It has no robot,
IK, ROS, JAKA SDK, or Inspire hand dependency and cannot emit hardware commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Mapping

from .errors import ProtocolValidationError
from .hts_canonical import CANONICAL_OPERATOR_FRAME, CanonicalQuestState
from .model import Pose6D, Side


class OperatorInputState(str, Enum):
    DISENGAGED = "disengaged"
    ARMED_REFERENCE_CAPTURE = "armed_reference_capture"
    ENGAGED = "engaged"


@dataclass(frozen=True, slots=True)
class OperatorStateTransition:
    timestamp_monotonic_ns: int
    previous: OperatorInputState
    current: OperatorInputState
    reason: str


@dataclass(frozen=True, slots=True)
class RightHandOperatorConfig:
    """Input-only production semantics and operator-space safety bounds."""

    required_hand: Side = Side.RIGHT
    left_hand_required: bool = False
    head_pose_required: bool = False
    required_joint_count: int = 21
    stale_after_s: float = 0.25
    translation_scale: tuple[float, float, float] = (0.35, 0.35, 0.35)
    orientation_mapping: str = "relative"
    orientation_scale: float = 1.0
    filter_time_constant_s: float = 0.02
    jump_reject_translation_m: float = 0.25
    jump_reject_rotation_rad: float = math.radians(45.0)
    workspace_min_m: tuple[float, float, float] = (-0.20, -0.20, -0.20)
    workspace_max_m: tuple[float, float, float] = (0.20, 0.20, 0.20)

    def __post_init__(self) -> None:
        if self.required_hand is not Side.RIGHT:
            raise ProtocolValidationError("HTS production required_hand must be right")
        if self.left_hand_required or self.head_pose_required:
            raise ProtocolValidationError(
                "the current HTS production contract requires neither left hand nor head"
            )
        if self.required_joint_count != 21:
            raise ProtocolValidationError("HTS production requires exactly 21 right joints")
        for name in (
            "stale_after_s",
            "orientation_scale",
            "jump_reject_translation_m",
            "jump_reject_rotation_rad",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ProtocolValidationError(f"{name} must be finite and positive")
        if not math.isfinite(self.filter_time_constant_s) or self.filter_time_constant_s < 0:
            raise ProtocolValidationError(
                "filter_time_constant_s must be finite and non-negative"
            )
        if self.orientation_mapping not in ("relative", "disabled"):
            raise ProtocolValidationError(
                "orientation_mapping must be 'relative' or 'disabled'"
            )
        for name, values in (
            ("translation_scale", self.translation_scale),
            ("workspace_min_m", self.workspace_min_m),
            ("workspace_max_m", self.workspace_max_m),
        ):
            if len(values) != 3 or not all(math.isfinite(value) for value in values):
                raise ProtocolValidationError(f"{name} must contain three finite values")
        if any(value < 0 for value in self.translation_scale):
            raise ProtocolValidationError("translation_scale values must be non-negative")
        if any(low >= high for low, high in zip(self.workspace_min_m, self.workspace_max_m)):
            raise ProtocolValidationError(
                "each workspace_min_m value must be below workspace_max_m"
            )

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "RightHandOperatorConfig":
        """Load the existing repository's YAML/dict configuration style."""

        required_hand = Side(str(values.get("required_hand", "right")).lower())
        return cls(
            required_hand=required_hand,
            left_hand_required=bool(values.get("left_hand_required", False)),
            head_pose_required=bool(values.get("head_pose_required", False)),
            required_joint_count=int(values.get("required_joint_count", 21)),
            stale_after_s=float(values.get("stale_after_ms", 250.0)) / 1000.0,
            translation_scale=_triple(values.get("translation_scale", (0.35,) * 3)),
            orientation_mapping=str(values.get("orientation_mapping", "relative")),
            orientation_scale=float(values.get("orientation_scale", 1.0)),
            filter_time_constant_s=float(values.get("filter_time_constant_ms", 20.0))
            / 1000.0,
            jump_reject_translation_m=float(
                values.get("jump_reject_translation_m", 0.25)
            ),
            jump_reject_rotation_rad=math.radians(
                float(values.get("jump_reject_rotation_deg", 45.0))
            ),
            workspace_min_m=_triple(
                values.get("operator_workspace_min_m", (-0.20,) * 3)
            ),
            workspace_max_m=_triple(
                values.get("operator_workspace_max_m", (0.20,) * 3)
            ),
        )


@dataclass(frozen=True, slots=True)
class OfflineOperatorTarget:
    """A relative operator transform, never a robot or TCP command."""

    timestamp_monotonic_ns: int
    state: OperatorInputState
    valid_for_mapping: bool
    emergency_neutral: bool
    frame_id: str
    translation_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    reference_host_sequence: int | None
    current_host_sequence: int | None
    reason: str


class RightHandOperatorPipeline:
    """Fail-disengaged state machine and bounded relative-pose preparation."""

    def __init__(self, config: RightHandOperatorConfig | None = None) -> None:
        self.config = config or RightHandOperatorConfig()
        self.state = OperatorInputState.DISENGAGED
        self.reference_pose: Pose6D | None = None
        self.reference_host_sequence: int | None = None
        self.transitions: list[OperatorStateTransition] = []
        self._last_input_pose: Pose6D | None = None
        self._last_input_sequence: int | None = None
        self._filtered_translation: tuple[float, float, float] | None = None
        self._filtered_orientation: tuple[float, float, float, float] | None = None
        self._last_output_ns: int | None = None

    def step(
        self,
        quest: CanonicalQuestState,
        *,
        engage_request: bool = False,
        capture_reference_request: bool = False,
        disengage_request: bool = False,
        frozen_stream: bool = False,
        malformed_data: bool = False,
    ) -> OfflineOperatorTarget:
        now_ns = quest.host_monotonic_ns
        hand = quest.right

        unsafe_reason = self._unsafe_reason(
            quest, frozen_stream=frozen_stream, malformed_data=malformed_data
        )
        if unsafe_reason is not None:
            self._force_disengaged(now_ns, unsafe_reason)
            return self._neutral(now_ns, hand.host_sequence_number, unsafe_reason)
        assert hand.wrist_pose is not None

        if disengage_request:
            self._force_disengaged(now_ns, "explicit_disengage_request")
            return self._neutral(now_ns, hand.host_sequence_number, "explicit_disengage_request")

        is_new_sample = hand.host_sequence_number != self._last_input_sequence
        if is_new_sample and self._last_input_pose is not None and self.state in (
            OperatorInputState.ARMED_REFERENCE_CAPTURE,
            OperatorInputState.ENGAGED,
        ):
            translation_jump = _distance(
                hand.wrist_pose.position_m, self._last_input_pose.position_m
            )
            rotation_jump = _quaternion_distance(
                hand.wrist_pose.orientation_xyzw,
                self._last_input_pose.orientation_xyzw,
            )
            if translation_jump > self.config.jump_reject_translation_m:
                self._force_disengaged(now_ns, "excessive_translation_jump")
                self._remember_input(hand.wrist_pose, hand.host_sequence_number)
                return self._neutral(
                    now_ns, hand.host_sequence_number, "excessive_translation_jump"
                )
            if rotation_jump > self.config.jump_reject_rotation_rad:
                self._force_disengaged(now_ns, "excessive_orientation_jump")
                self._remember_input(hand.wrist_pose, hand.host_sequence_number)
                return self._neutral(
                    now_ns, hand.host_sequence_number, "excessive_orientation_jump"
                )

        self._remember_input(hand.wrist_pose, hand.host_sequence_number)

        if self.state is OperatorInputState.DISENGAGED:
            if engage_request:
                self._transition(
                    now_ns,
                    OperatorInputState.ARMED_REFERENCE_CAPTURE,
                    "explicit_engage_request_with_fresh_right_hand",
                )
                return self._neutral(
                    now_ns, hand.host_sequence_number, "awaiting_reference_capture"
                )
            return self._neutral(now_ns, hand.host_sequence_number, "disengaged")

        if self.state is OperatorInputState.ARMED_REFERENCE_CAPTURE:
            if capture_reference_request:
                self.reference_pose = hand.wrist_pose
                self.reference_host_sequence = hand.host_sequence_number
                self._filtered_translation = (0.0, 0.0, 0.0)
                self._filtered_orientation = (0.0, 0.0, 0.0, 1.0)
                self._last_output_ns = now_ns
                self._transition(
                    now_ns,
                    OperatorInputState.ENGAGED,
                    "fresh_right_hand_reference_captured",
                )
                return self._target(
                    now_ns,
                    hand.host_sequence_number,
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    "reference_captured",
                )
            return self._neutral(now_ns, hand.host_sequence_number, "awaiting_reference_capture")

        assert self.reference_pose is not None
        translation = tuple(
            (current - reference) * scale
            for current, reference, scale in zip(
                hand.wrist_pose.position_m,
                self.reference_pose.position_m,
                self.config.translation_scale,
            )
        )
        if not _inside_workspace(
            translation, self.config.workspace_min_m, self.config.workspace_max_m
        ):
            self._force_disengaged(now_ns, "operator_workspace_envelope_violation")
            return self._neutral(
                now_ns, hand.host_sequence_number, "operator_workspace_envelope_violation"
            )

        if self.config.orientation_mapping == "disabled":
            orientation = (0.0, 0.0, 0.0, 1.0)
        else:
            relative = _quaternion_multiply(
                hand.wrist_pose.orientation_xyzw,
                _quaternion_conjugate(self.reference_pose.orientation_xyzw),
            )
            orientation = _quaternion_scaled(relative, self.config.orientation_scale)

        translation, orientation = self._filter(now_ns, translation, orientation)
        return self._target(
            now_ns,
            hand.host_sequence_number,
            translation,
            orientation,
            "engaged_relative_operator_transform",
        )

    def force_fault(self, *, timestamp_monotonic_ns: int, reason: str) -> None:
        """Inject a parser/transport safety fault that has no canonical sample."""

        if not reason.strip():
            raise ValueError("fault reason must not be empty")
        self._force_disengaged(timestamp_monotonic_ns, reason)

    def _unsafe_reason(
        self,
        quest: CanonicalQuestState,
        *,
        frozen_stream: bool,
        malformed_data: bool,
    ) -> str | None:
        hand = quest.right
        if malformed_data:
            return "malformed_right_hand_data"
        if frozen_stream:
            return "frozen_right_hand_stream"
        if not hand.tracking_valid or hand.wrist_pose is None:
            if hand.stream_age_s is not None and hand.stream_age_s > self.config.stale_after_s:
                return "right_hand_stale"
            return "right_hand_not_tracking"
        if hand.stream_age_s is None or hand.stream_age_s > self.config.stale_after_s:
            return "right_hand_stale"
        if len(hand.joints) != self.config.required_joint_count:
            return "right_hand_joint_count_invalid"
        return None

    def _remember_input(self, pose: Pose6D, sequence: int | None) -> None:
        if sequence != self._last_input_sequence:
            self._last_input_pose = pose
            self._last_input_sequence = sequence

    def _filter(
        self,
        now_ns: int,
        translation: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        if (
            self.config.filter_time_constant_s == 0
            or self._filtered_translation is None
            or self._filtered_orientation is None
            or self._last_output_ns is None
        ):
            alpha = 1.0
        else:
            dt_s = max(0.0, (now_ns - self._last_output_ns) / 1e9)
            alpha = 1.0 - math.exp(-dt_s / self.config.filter_time_constant_s)
        filtered_translation = tuple(
            previous + alpha * (current - previous)
            for previous, current in zip(self._filtered_translation or translation, translation)
        )
        filtered_orientation = _quaternion_slerp(
            self._filtered_orientation or orientation, orientation, alpha
        )
        self._filtered_translation = filtered_translation
        self._filtered_orientation = filtered_orientation
        self._last_output_ns = now_ns
        return filtered_translation, filtered_orientation

    def _transition(
        self, now_ns: int, current: OperatorInputState, reason: str
    ) -> None:
        previous = self.state
        if current is previous:
            return
        self.state = current
        self.transitions.append(OperatorStateTransition(now_ns, previous, current, reason))

    def _force_disengaged(self, now_ns: int, reason: str) -> None:
        self.reference_pose = None
        self.reference_host_sequence = None
        self._filtered_translation = None
        self._filtered_orientation = None
        self._last_output_ns = None
        self._transition(now_ns, OperatorInputState.DISENGAGED, reason)

    def _neutral(
        self, now_ns: int, current_sequence: int | None, reason: str
    ) -> OfflineOperatorTarget:
        return OfflineOperatorTarget(
            now_ns,
            self.state,
            False,
            True,
            CANONICAL_OPERATOR_FRAME,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            self.reference_host_sequence,
            current_sequence,
            reason,
        )

    def _target(
        self,
        now_ns: int,
        current_sequence: int | None,
        translation: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
        reason: str,
    ) -> OfflineOperatorTarget:
        return OfflineOperatorTarget(
            now_ns,
            self.state,
            True,
            False,
            CANONICAL_OPERATOR_FRAME,
            translation,
            orientation,
            self.reference_host_sequence,
            current_sequence,
            reason,
        )


def _triple(values: Any) -> tuple[float, float, float]:
    try:
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ProtocolValidationError("expected a three-value numeric sequence") from exc
    if len(converted) != 3:
        raise ProtocolValidationError("expected a three-value numeric sequence")
    return converted


def _distance(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))


def _quaternion_conjugate(
    quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return _normalized_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def _quaternion_distance(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> float:
    dot = min(1.0, abs(sum(a * b for a, b in zip(left, right))))
    return 2.0 * math.acos(dot)


def _quaternion_scaled(
    quaternion: tuple[float, float, float, float], scale: float
) -> tuple[float, float, float, float]:
    x, y, z, w = _normalized_quaternion(quaternion)
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    half_angle = math.acos(max(-1.0, min(1.0, w)))
    sine = math.sin(half_angle)
    if abs(sine) < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    new_half_angle = half_angle * scale
    factor = math.sin(new_half_angle) / sine
    return _normalized_quaternion(
        (x * factor, y * factor, z * factor, math.cos(new_half_angle))
    )


def _quaternion_slerp(
    start: tuple[float, float, float, float],
    end: tuple[float, float, float, float],
    fraction: float,
) -> tuple[float, float, float, float]:
    fraction = max(0.0, min(1.0, fraction))
    dot = sum(a * b for a, b in zip(start, end))
    if dot < 0:
        end = tuple(-value for value in end)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return _normalized_quaternion(
            tuple(a + fraction * (b - a) for a, b in zip(start, end))
        )
    angle = math.acos(dot)
    sine = math.sin(angle)
    left_weight = math.sin((1.0 - fraction) * angle) / sine
    right_weight = math.sin(fraction * angle) / sine
    return _normalized_quaternion(
        tuple(left_weight * a + right_weight * b for a, b in zip(start, end))
    )


def _normalized_quaternion(
    quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm == 0:
        raise ProtocolValidationError("cannot normalize a zero quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _inside_workspace(
    position: tuple[float, float, float],
    lower: tuple[float, float, float],
    upper: tuple[float, float, float],
) -> bool:
    return all(low <= value <= high for value, low, high in zip(position, lower, upper))
