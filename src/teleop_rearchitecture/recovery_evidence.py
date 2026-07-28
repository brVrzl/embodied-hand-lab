"""Rebuildable offline continuity evidence for recoverable engagement."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .cpp_shaping import CppReferenceShaper
from .engagement import EngagementCoordinator, EngagementResult, MeasuredJointState, SpatialPose
from .unified_evaluator import PalmModel


def build_recovery_evidence(library_path: Path, model_path: Path) -> dict[str, object]:
    capture_ns = 20_000_000_000
    q = (0.1, -0.2, 0.3, -0.1, 0.2, -0.3)
    measured = MeasuredJointState(2, capture_ns, q, (0.0,) * 6, (0.0,) * 6)
    coordinator = EngagementCoordinator()
    initial = MeasuredJointState(1, capture_ns - 1_000_000_000, (0.0,) * 6,
                                 (0.0,) * 6, (0.0,) * 6)
    assert coordinator.initialize_disengaged(initial) is EngagementResult.OK
    assert coordinator.observe_input(1, SpatialPose.checked((0, 0, 0), (1, 0, 0, 0))) is EngagementResult.OK
    assert coordinator.begin_engagement(initial, capture_ns - 1_000_000_000)[0] is EngagementResult.OK
    assert coordinator.complete_engagement() is EngagementResult.OK
    assert coordinator.note_target(1, 1, True) is EngagementResult.OK
    assert coordinator.request_release() is EngagementResult.OK
    assert coordinator.observe_input(
        2, SpatialPose.checked((0.75, -0.2, 0.1), (0.9238795325, 0, 0.3826834324, 0))
    ) is EngagementResult.OK
    assert coordinator.braking_complete() is EngagementResult.OK
    result, capture = coordinator.begin_engagement(measured, capture_ns)
    assert result is EngagementResult.OK and capture is not None
    assert coordinator.complete_engagement() is EngagementResult.OK
    relative = coordinator.relative_pose()
    assert relative is not None
    assert coordinator.note_target(99, capture.safety_epoch - 1, True) is EngagementResult.OLD_EPOCH

    with CppReferenceShaper(library_path) as shaper:
        shaper.initialize(
            position_rad=q, velocity_rad_s=(0.0,) * 6, acceleration_rad_s2=(0.0,) * 6,
            minimum_position_rad=(-3.0,) * 6, maximum_position_rad=(3.0,) * 6,
            maximum_velocity_rad_s=(np.pi,) * 6,
            maximum_acceleration_rad_s2=(4 * np.pi,) * 6,
            maximum_jerk_rad_s3=(50.0,) * 6, now_ns=capture_ns,
            safety_epoch=capture.safety_epoch,
        )
        shaper.replace_target(
            q, sequence=1, source_monotonic_ns=capture_ns,
            accepted_monotonic_ns=capture_ns, valid_until_monotonic_ns=capture_ns + 1_000_000_000,
        )
        first = shaper.tick(capture_ns)
    palm = PalmModel(model_path)
    reference_palm, _ = palm.pose(q)
    first_palm, _ = palm.pose(first.position_rad)
    first_jerk = tuple(value / 0.008 for value in first.acceleration_rad_s2)
    snapshot = coordinator.snapshot()
    return {
        "schema_version": "teleop_reengagement_continuity.v1",
        "period_s": 0.008,
        "safety_epoch": capture.safety_epoch,
        "capture_to_first_output_tick_count": 0,
        "first_tick_joint_position_delta_rad": max(
            abs(left - right) for left, right in zip(first.position_rad, q)
        ),
        "first_tick_palm_model_displacement_m": float(np.linalg.norm(first_palm - reference_palm)),
        "first_tick_peak_velocity_rad_s": max(abs(value) for value in first.velocity_rad_s),
        "first_tick_peak_acceleration_rad_s2": max(abs(value) for value in first.acceleration_rad_s2),
        "first_tick_peak_jerk_rad_s3": max(abs(value) for value in first_jerk),
        "first_relative_translation_m": list(relative.translation_m),
        "first_relative_rotation_wxyz": list(relative.rotation_wxyz),
        "old_target_rejection_count": snapshot.old_target_rejection_count,
        "input_mailbox_depth": 1,
        "queued_release_motion_count": 0,
        "limitations": [
            "Offline reference coordinator, C++ shaper, and MuJoCo palm-model FK only.",
            "No Quest receiver, SDK, controller, scheduler, network, or physical plant is present."
        ]
    }
