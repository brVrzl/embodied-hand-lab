"""Thin accepted-joint-target adapter for the JAKA native transport."""

from __future__ import annotations

import time
import math
from typing import Protocol

from teleoperation.accepted_target import AcceptedArmTarget, ArmControlHeartbeat

from ..runtime.arm_only import ArmOnlyRuntime
from ..wire import (
    heartbeat_target_packet,
    hold_current_target_packet,
    joint_position_target_packet,
)


JAKA_JOINT_ORDER = tuple(f"jaka_joint_{index}" for index in range(1, 7))
JAKA_COMMAND_MODE = "edg_servo_j_absolute"
JAKA_JOINT_ANGLE_UNIT = "rad"

E2_MAXIMUM_FORWARD_DISPLACEMENT_M = 0.025
E2_MAXIMUM_CROSS_AXIS_DISPLACEMENT_M = 0.004
E2_MAXIMUM_NEUTRAL_OVERSHOOT_M = 0.003
E2_MAXIMUM_ORIENTATION_CHANGE_RAD = math.radians(3.0)
class _PacketRuntime(Protocol):
    def dispatch_packet(self, packet: object) -> bool: ...
    def dispatch_stop(self, *, sequence: int) -> bool: ...


class JakaAcceptedJointTargetAdapter:
    """Representation-only boundary: SI radians/J1..J6 go out unchanged."""

    def __init__(
        self,
        runtime: ArmOnlyRuntime | _PacketRuntime,
        *,
        allow_motion: bool,
        joint_order: tuple[str, ...] = JAKA_JOINT_ORDER,
        joint_angle_unit: str = JAKA_JOINT_ANGLE_UNIT,
        command_mode: str = JAKA_COMMAND_MODE,
    ) -> None:
        if tuple(joint_order) != JAKA_JOINT_ORDER:
            raise ValueError("JAKA adapter requires canonical J1..J6 ordering")
        if joint_angle_unit != JAKA_JOINT_ANGLE_UNIT:
            raise ValueError("JAKA EDG joint targets must be radians")
        if command_mode != JAKA_COMMAND_MODE:
            raise ValueError("JAKA adapter requires absolute EDG servo-j mode")
        self.runtime = runtime
        self.allow_motion = bool(allow_motion)
        self.applied_count = 0
        self.last_sequence = 0
        self.stopped = False

    def apply(self, target: AcceptedArmTarget) -> bool:
        if self.stopped:
            return False
        dispatch_ns = max(time.monotonic_ns(), target.generated_monotonic_ns)
        self.last_sequence += 1
        packet = joint_position_target_packet(
            sequence=self.last_sequence,
            joint_position_rad=target.joint_position_rad,
            local_receive_ns=target.input_receive_monotonic_ns,
            processing_ns=target.generated_monotonic_ns,
            dispatch_ns=dispatch_ns,
            allow_motion=self.allow_motion,
        )
        sent = self.runtime.dispatch_packet(packet)
        if sent:
            self.applied_count += 1
        return sent

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        if self.stopped:
            return False
        self.last_sequence += 1
        dispatch_ns = max(time.monotonic_ns(), heartbeat.generated_monotonic_ns)
        packet = heartbeat_target_packet(
            sequence=self.last_sequence,
            input_sequence=heartbeat.input_sequence_number,
            local_receive_ns=heartbeat.input_receive_monotonic_ns,
            processing_ns=heartbeat.generated_monotonic_ns,
            dispatch_ns=dispatch_ns,
            last_accepted_target_sequence=heartbeat.last_accepted_target_sequence,
            control_state_code=1,
            allow_motion=self.allow_motion,
        )
        return self.runtime.dispatch_packet(packet)

    def stop(self) -> bool:
        self.stopped = True
        self.last_sequence += 1
        return self.runtime.dispatch_stop(sequence=self.last_sequence)

    def pause(self) -> bool:
        """Request recoverable braking; unlike stop(), future targets remain allowed."""

        if self.stopped:
            return False
        self.last_sequence += 1
        return self.runtime.dispatch_packet(
            hold_current_target_packet(
                sequence=self.last_sequence,
                monotonic_ns=time.monotonic_ns(),
            )
        )


class E2IsolatedForwardTranslationGuard:
    """E2-only reject gate; forwards the immutable target without modification.

    The operator-aligned mapping sends operator-forward motion toward robot-base +X.
    This commissioning guard prevents E2 from becoming a general 6D session;
    it never scales, filters, interpolates, or rewrites an accepted target.
    """

    def __init__(
        self,
        output: JakaAcceptedJointTargetAdapter,
        *,
        startup_alignment_tolerance_rad: float,
    ) -> None:
        tolerance = float(startup_alignment_tolerance_rad)
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("E2 startup alignment tolerance must be finite and positive")
        self.output = output
        self.startup_alignment_tolerance_rad = tolerance
        self.abort_reason: str | None = None
        self.baseline: AcceptedArmTarget | None = None
        self.startup_joint_position_rad: tuple[float, ...] | None = None
        self.startup_alignment_difference_rad: tuple[float, ...] | None = None
        self.latest_measured_joint_position_rad: tuple[float, ...] | None = None
        self.maximum_observed_startup_difference_rad = [0.0] * 6
        self.maximum_requested_tcp_displacement_m = 0.0
        self.maximum_accepted_tcp_displacement_m = 0.0
        self.maximum_accepted_joint_displacement_rad = [0.0] * 6

    @property
    def stopped(self) -> bool:
        return self.output.stopped

    @property
    def applied_count(self) -> int:
        return self.output.applied_count

    def establish_startup_joint_position(self, joints_rad: tuple[float, ...]) -> None:
        values = tuple(float(value) for value in joints_rad)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("E2 startup position must contain six finite radians")
        if self.startup_joint_position_rad is not None:
            raise RuntimeError("E2 startup position is already established")
        self.startup_joint_position_rad = values
        self.latest_measured_joint_position_rad = values

    def observe_measured_joint_position(self, joints_rad: tuple[float, ...]) -> None:
        """Record live encoders without changing the armed-session command state."""

        values = tuple(float(value) for value in joints_rad)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("E2 measured position must contain six finite radians")
        if self.startup_joint_position_rad is None:
            raise RuntimeError("E2 startup position has not been established")
        self.latest_measured_joint_position_rad = values
        for joint, (measured, startup) in enumerate(
            zip(values, self.startup_joint_position_rad, strict=True)
        ):
            self.maximum_observed_startup_difference_rad[joint] = max(
                self.maximum_observed_startup_difference_rad[joint],
                abs(measured - startup),
            )

    def apply(self, target: AcceptedArmTarget) -> bool:
        if self.output.stopped or self.abort_reason is not None:
            return False
        if self.baseline is None:
            if self.startup_joint_position_rad is None:
                self.abort_reason = "e2_post_edg_startup_not_established"
                return False
            self.startup_alignment_difference_rad = tuple(
                target_value - startup_value
                for target_value, startup_value in zip(
                    target.joint_position_rad,
                    self.startup_joint_position_rad,
                    strict=True,
                )
            )
            if (
                max(map(abs, self.startup_alignment_difference_rad))
                > self.startup_alignment_tolerance_rad
            ):
                self.abort_reason = "e2_first_target_not_continuous_with_post_edg_state"
                return False
            self.baseline = target
        assert self.baseline is not None
        requested_delta = _position_delta(
            target.desired_tcp.position_m, self.baseline.desired_tcp.position_m
        )
        accepted_delta = _position_delta(
            target.filtered_tcp.position_m, self.baseline.filtered_tcp.position_m
        )
        self.maximum_requested_tcp_displacement_m = max(
            self.maximum_requested_tcp_displacement_m, _norm(requested_delta)
        )
        self.maximum_accepted_tcp_displacement_m = max(
            self.maximum_accepted_tcp_displacement_m, _norm(accepted_delta)
        )
        violation = _e2_violation(target, self.baseline, requested_delta, accepted_delta)
        if violation is not None:
            self.abort_reason = violation
            return False
        for joint, (current, start) in enumerate(
            zip(target.joint_position_rad, self.baseline.joint_position_rad, strict=True)
        ):
            self.maximum_accepted_joint_displacement_rad[joint] = max(
                self.maximum_accepted_joint_displacement_rad[joint], abs(current - start)
            )
        return self.output.apply(target)

    def stop(self) -> bool:
        return self.output.stop()

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        return self.output.heartbeat(heartbeat)


class ResearchThinBoundedMotionGuard:
    """Research gate envelope for separate forward or single-axis wrist motion.

    The guard only rejects; it never rewrites an immutable accepted target.
    Forward translation and wrist rotation may each be exercised relative to
    the first accepted endpoint, but not at the same time.
    """

    def __init__(self, output: JakaAcceptedJointTargetAdapter) -> None:
        self.output = output
        self.baseline: AcceptedArmTarget | None = None
        self.abort_reason: str | None = None

    @property
    def stopped(self) -> bool:
        return self.output.stopped

    @property
    def applied_count(self) -> int:
        return self.output.applied_count

    def apply(self, target: AcceptedArmTarget) -> bool:
        if self.stopped or self.abort_reason is not None:
            return False
        if self.baseline is None:
            self.baseline = target
            return self.output.apply(target)
        delta = _position_delta(
            target.filtered_tcp.position_m,
            self.baseline.filtered_tcp.position_m,
        )
        rotation_vector = _relative_rotation_vector(
            target.filtered_tcp.orientation_xyzw,
            self.baseline.filtered_tcp.orientation_xyzw,
        )
        translation_norm = _norm(delta)
        rotation_norm = _norm(rotation_vector)
        if translation_norm > 0.003 and rotation_norm > math.radians(2.0):
            self.abort_reason = "research_combined_translation_rotation_forbidden"
            return False
        if translation_norm > 0.003:
            if not (
                -0.003 <= delta[0] <= 0.025
                and abs(delta[1]) <= 0.004
                and abs(delta[2]) <= 0.004
                and rotation_norm <= math.radians(3.0)
            ):
                self.abort_reason = "research_forward_envelope_exceeded"
                return False
        if rotation_norm > math.radians(2.0):
            ordered = sorted(map(abs, rotation_vector), reverse=True)
            if not (
                translation_norm <= 0.003
                and rotation_norm <= math.radians(6.0)
                and ordered[1] <= math.radians(1.0)
            ):
                self.abort_reason = "research_single_axis_wrist_envelope_exceeded"
                return False
        return self.output.apply(target)

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        return self.output.heartbeat(heartbeat)

    def pause(self) -> bool:
        return self.output.pause()

    def stop(self) -> bool:
        return self.output.stop()


def _e2_violation(
    target: AcceptedArmTarget,
    baseline: AcceptedArmTarget,
    requested_delta: tuple[float, float, float],
    accepted_delta: tuple[float, float, float],
) -> str | None:
    for label, delta in (("requested", requested_delta), ("accepted", accepted_delta)):
        if delta[0] > E2_MAXIMUM_FORWARD_DISPLACEMENT_M:
            return f"e2_{label}_forward_displacement_exceeded"
        if -delta[0] > E2_MAXIMUM_NEUTRAL_OVERSHOOT_M:
            return f"e2_{label}_opposite_direction"
        if max(abs(delta[1]), abs(delta[2])) > E2_MAXIMUM_CROSS_AXIS_DISPLACEMENT_M:
            return f"e2_{label}_cross_axis_displacement"
    if max(
        _quaternion_angle(
            target.desired_tcp.orientation_xyzw,
            baseline.desired_tcp.orientation_xyzw,
        ),
        _quaternion_angle(
            target.filtered_tcp.orientation_xyzw,
            baseline.filtered_tcp.orientation_xyzw,
        ),
    ) > E2_MAXIMUM_ORIENTATION_CHANGE_RAD:
        return "e2_orientation_change"
    return None


def _position_delta(
    current: tuple[float, float, float], baseline: tuple[float, float, float]
) -> tuple[float, float, float]:
    return tuple(current_value - baseline_value for current_value, baseline_value in zip(current, baseline, strict=True))


def _norm(values: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _relative_rotation_vector(
    current: tuple[float, ...], baseline: tuple[float, ...]
) -> tuple[float, float, float]:
    x1, y1, z1, w1 = current
    x0, y0, z0, w0 = baseline
    # current * conjugate(baseline), xyzw convention.
    x = -w1 * x0 + x1 * w0 - y1 * z0 + z1 * y0
    y = -w1 * y0 + x1 * z0 + y1 * w0 - z1 * x0
    z = -w1 * z0 - x1 * y0 + y1 * x0 + z1 * w0
    w = w1 * w0 + x1 * x0 + y1 * y0 + z1 * z0
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return (math.inf, math.inf, math.inf)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    vector_norm = math.sqrt(x * x + y * y + z * z)
    if vector_norm <= 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, max(0.0, min(1.0, w)))
    scale = angle / vector_norm
    return (x * scale, y * scale, z * scale)


def _quaternion_angle(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    dot = abs(sum(a * b for a, b in zip(left, right, strict=True)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))
