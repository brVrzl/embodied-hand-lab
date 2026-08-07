from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
import re

import numpy as np
import pytest

from jaka_driver_adapter.palm_target_ik import JAKA_MINI2_JOINT_LIMITS_RAD
from quest_jaka_sim import ReplayConfig, SharedJakaTargetGenerator
from quest_jaka_sim.se3 import bounded_pose_step, quaternion_angle_rad
from quest_jaka_sim.simulation import FeasibilityReason
from teleoperation.output_feasibility import (
    JointOutputFeasibilityTracker,
    JointOutputPrefilter,
)


CONFIG = "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
PERIOD_NS = 8_000_000
Q212 = (
    1.6108104942490866,
    -0.1634258244684054,
    -1.1203194182446221,
    -0.2764266537763923,
    -0.8181052330451166,
    0.49270800503151646,
)
Q213 = (
    1.6082960194916314,
    -0.1623994661613463,
    -1.1220051457136564,
    -0.2685048085831845,
    -0.8152173030695117,
    0.46753787793466656,
)
Q214 = (
    1.5989877336691674,
    -0.15630842145305937,
    -1.1341166309481496,
    -0.2418470947539033,
    -0.7982159224170279,
    0.3956532760306145,
)
T212 = 666_185_828_171_068
T213 = 666_185_844_837_051
T214 = 666_185_861_504_127


def test_python_joint_limits_match_shared_mjcf_contract() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    model_limits = [
        tuple(float(value) for value in generator.model.jnt_range[joint_id])
        for joint_id in generator.arm_joint_ids
    ]
    assert JAKA_MINI2_JOINT_LIMITS_RAD == tuple(model_limits)
    assert generator._contact_pairs(generator.ik.data) == set()


def test_native_final_joint_limits_match_python_contract() -> None:
    header = (
        Path(__file__).resolve().parents[1]
        / "native/jaka_servo_worker/joint_servo_resampler.hpp"
    ).read_text(encoding="utf-8")

    def values(name: str) -> tuple[float, ...]:
        match = re.search(rf"{name}\{{([^}}]+)\}}", header)
        assert match is not None
        return tuple(float(value.strip()) for value in match.group(1).split(","))

    assert values("kJointLower") == tuple(
        limit[0] for limit in JAKA_MINI2_JOINT_LIMITS_RAD
    )
    assert values("kJointUpper") == tuple(
        limit[1] for limit in JAKA_MINI2_JOINT_LIMITS_RAD
    )


def test_authoritative_measured_state_is_not_soft_clipped_at_startup() -> None:
    measured = (
        1.5624361730255556,
        -0.12716468713111112,
        -0.9298765030155555,
        -6.208851628425556,
        -1.05620343212,
        6.192044108015556,
    )
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))

    generator.synchronize_authoritative_arm_joints(list(measured))
    assert generator.last_safe_joint_target == pytest.approx(measured)
    assert generator.ik.arm_joints_rad == pytest.approx(measured)

    result = generator.evaluate(
        generator.capture_reference(),
        dt_s=1.0 / 60.0,
        generated_monotonic_ns=1_000_000_000,
        fresh_measured_joint_position_rad=measured,
    )
    assert result.accepted
    assert result.joint_target_rad == pytest.approx(measured)
    assert result.metrics.joint_limit_blockers == ()
    assert generator.ik.last_position_target_iterations_completed == 0

    invalid = list(measured)
    invalid[3] = -6.281
    with pytest.raises(ValueError, match="manufacturer limits"):
        generator.synchronize_authoritative_arm_joints(invalid)

    def evaluate_wrist_motion(delta_j4: float):
        trial = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
        trial.synchronize_authoritative_arm_joints(list(measured))
        trial.capture_reference()
        target_q = list(measured)
        target_q[3] += delta_j4
        trial.ik.set_authoritative_arm_joints_rad(target_q)
        target_pose = trial.current_tcp_pose
        trial.ik.set_authoritative_arm_joints_rad(list(measured))
        return trial.evaluate(
            target_pose,
            dt_s=1.0 / 60.0,
            generated_monotonic_ns=1_000_000_000,
            fresh_measured_joint_position_rad=measured,
        )

    retreat = evaluate_wrist_motion(0.001)
    assert retreat.accepted
    assert retreat.joint_target_rad[3] > measured[3]

    toward_hard_limit = evaluate_wrist_motion(-0.001)
    assert not toward_hard_limit.accepted
    assert "joint_4_below_safe_limit" in toward_hard_limit.metrics.joint_limit_blockers


def _tracker() -> JointOutputFeasibilityTracker:
    return JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=math.pi,
        servo_period_ns=PERIOD_NS,
    )


def _establish_zero_baseline(tracker: JointOutputFeasibilityTracker) -> None:
    tracker.reset((0.0,) * 6)
    tracker.commit(
        tracker.preview((0.0,) * 6, generated_monotonic_ns=1_000_000_000)
    )


def test_control_tick_uses_coarse_prefilter_without_native_segment_or_jerk() -> None:
    tracker = _tracker()
    tracker.reset((0.0,) * 6)
    baseline = tracker.prefilter(
        (0.0,) * 6,
        generated_monotonic_ns=1_000_000_000,
    )
    assert isinstance(baseline, JointOutputPrefilter)
    tracker.commit_prefilter(
        (0.0,) * 6,
        generated_monotonic_ns=1_000_000_000,
        prefilter=baseline,
    )
    candidate = tracker.prefilter(
        (math.pi * 0.016 * 1.01, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_016_000_000,
    )
    assert candidate.violating_joint_indices == (0,)
    assert not hasattr(candidate, "predicted_jerk_rad_s3")


def test_default_prefilter_uses_producer_interval_for_acceleration() -> None:
    tracker = JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=math.pi,
        maximum_acceleration_rad_s2=4.0 * math.pi,
        servo_period_ns=PERIOD_NS,
    )
    tracker.reset((0.0,) * 6)
    baseline = tracker.prefilter(
        (0.0,) * 6,
        generated_monotonic_ns=1_000_000_000,
    )
    assert baseline.feasible
    tracker.commit_prefilter(
        (0.0,) * 6,
        generated_monotonic_ns=1_000_000_000,
        prefilter=baseline,
    )

    baseline = tracker.prefilter(
        (0.02, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_050_000_000,
    )
    assert baseline.feasible
    tracker.commit_prefilter(
        (0.02, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_050_000_000,
        prefilter=baseline,
    )

    # A 50 ms producer replacement has a modest 0.2 rad/s velocity change.
    # Treating it as an 8 ms native transition would report 25 rad/s^2 and
    # incorrectly hold the target before native shaping gets to act.
    candidate = tracker.prefilter(
        (0.03, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_100_000_000,
    )
    assert candidate.feasible
    assert candidate.maximum_acceleration_rad_s2 == pytest.approx(4.0)


def test_hold_interval_does_not_turn_stale_velocity_into_acceleration_rejection() -> None:
    tracker = JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=math.pi,
        maximum_acceleration_rad_s2=4.0 * math.pi,
        servo_period_ns=PERIOD_NS,
        feasibility_acceleration_period_ns=16_666_667,
    )
    tracker.reset((0.0,) * 6)
    baseline = tracker.prefilter(
        (0.0,) * 6,
        generated_monotonic_ns=1_000_000_000,
    )
    tracker.commit_prefilter(
        (0.0,) * 6,
        generated_monotonic_ns=1_000_000_000,
        prefilter=baseline,
    )
    moving = tracker.prefilter(
        (0.02, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_050_000_000,
    )
    tracker.commit_prefilter(
        (0.02, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_050_000_000,
        prefilter=moving,
    )

    # The long hold interval must be used for the acceleration estimate; the
    # old fixed-period comparison would report roughly -22 rad/s^2 here.
    held = tracker.prefilter(
        (0.03, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_400_000_000,
    )
    assert held.feasible
    assert held.maximum_acceleration_rad_s2 == pytest.approx(
        (0.02 / 0.05 - 0.01 / 0.35) / 0.35
    )


def test_hold_resync_preserves_velocity_with_bounded_deceleration() -> None:
    tracker = JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=math.pi,
        maximum_acceleration_rad_s2=4.0,
        servo_period_ns=PERIOD_NS,
    )
    tracker.reset((0.0,) * 6)
    baseline = tracker.prefilter(
        (0.0,) * 6,
        generated_monotonic_ns=1_000_000_000,
    )
    tracker.commit_prefilter(
        (0.0,) * 6,
        generated_monotonic_ns=1_000_000_000,
        prefilter=baseline,
    )
    moving = tracker.prefilter(
        (0.04, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_100_000_000,
    )
    tracker.commit_prefilter(
        (0.04, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_100_000_000,
        prefilter=moving,
    )
    tracker.resync_hold(
        (0.04, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_150_000_000,
    )
    next_candidate = tracker.prefilter(
        (0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_200_000_000,
    )
    assert next_candidate.feasible
    # The accepted 0.04 rad/100 ms step established 0.4 rad/s.  A 50 ms hold
    # decelerates it by only 0.2 rad/s, so the next 0.01 rad/50 ms step keeps
    # the velocity and acceleration estimates continuous instead of starting
    # from an artificial zero-velocity state.
    assert next_candidate.maximum_acceleration_rad_s2 == pytest.approx(0.0)


def test_below_and_exact_output_velocity_boundary_are_accepted() -> None:
    tracker = _tracker()
    _establish_zero_baseline(tracker)
    dt_s = 0.016
    below = tracker.preview(
        (math.pi * dt_s * 0.99, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_016_000_000,
    )
    assert below.feasible

    tracker = _tracker()
    _establish_zero_baseline(tracker)
    exact = tracker.preview(
        (math.pi * dt_s, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_016_000_000,
    )
    assert exact.feasible
    assert exact.maximum_velocity_rad_s == pytest.approx(math.pi, abs=1e-12)


def test_above_output_velocity_boundary_is_rejected_without_state_commit() -> None:
    tracker = _tracker()
    _establish_zero_baseline(tracker)
    prediction = tracker.preview(
        (math.pi * 0.016 * 1.01, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_016_000_000,
    )
    assert not prediction.feasible
    assert prediction.violating_joint_indices == (0,)
    with pytest.raises(ValueError, match="infeasible"):
        tracker.commit(prediction)


def test_failed_p4_prediction_includes_active_segment_replacement_residual() -> None:
    tracker = _tracker()
    tracker.reset(Q212)
    tracker.commit(tracker.preview(Q212, generated_monotonic_ns=T212))
    tracker.commit(tracker.preview(Q213, generated_monotonic_ns=T213))
    prediction = tracker.preview(Q214, generated_monotonic_ns=T214)

    direct_j6_velocity = (Q214[5] - Q213[5]) / ((T214 - T213) / 1e9)
    assert direct_j6_velocity == pytest.approx(-4.3129701876953135)
    assert prediction.predicted_velocity_rad_s[5] < direct_j6_velocity
    assert prediction.predicted_velocity_rad_s[5] == pytest.approx(
        -4.37336, abs=2e-4
    )
    assert prediction.violating_joint_indices == (5,)


def test_failed_p4_candidate_backtracks_before_accepted_target_boundary() -> None:
    base = ReplayConfig.load(CONFIG)
    config = replace(
        base,
        output_contract=replace(
            base.output_contract,
            maximum_acceleration_rad_s2=math.inf,
            # Keep this continuation regression on the original scalar-hard
            # boundary; the production default is tested separately at 1.5.
            maximum_velocity_rad_s_per_joint=(math.pi,) * 6,
        ),
    )
    generator = SharedJakaTargetGenerator(config)
    generator.synchronize_authoritative_arm_joints(list(Q213))
    generator.output_feasibility.commit(
        generator.output_feasibility.preview(Q213, generated_monotonic_ns=T213)
    )
    generator.ik.set_arm_joints_rad(list(Q214))
    desired = generator.current_tcp_pose
    generator.ik.set_arm_joints_rad(list(Q213))
    dt_s = (T214 - T213) / 1e9

    full = generator.evaluate(
        desired,
        dt_s=dt_s,
        generated_monotonic_ns=T214,
    )
    assert not full.accepted
    assert full.reason is FeasibilityReason.OUTPUT_VELOCITY_INFEASIBLE
    assert full.metrics.output_velocity_violating_joint_indices == (5,)
    assert np.asarray(generator.last_safe_joint_target) == pytest.approx(Q213)

    half, fraction = bounded_pose_step(
        generator.last_safe_target,
        desired,
        maximum_translation_m=float(
            np.linalg.norm(
                np.asarray(desired.position_m)
                - np.asarray(generator.last_safe_target.position_m)
            )
            * 0.5
        ),
        maximum_rotation_rad=(
            quaternion_angle_rad(
                generator.last_safe_target.orientation_xyzw,
                desired.orientation_xyzw,
            )
            * 0.5
        ),
    )
    assert fraction == pytest.approx(0.5)
    recovered = generator.evaluate(
        half,
        dt_s=dt_s,
        generated_monotonic_ns=T214,
    )
    assert recovered.accepted
    assert recovered.reason is FeasibilityReason.ACCEPTED
    assert recovered.metrics.predicted_output_maximum_joint_velocity_rad_s < math.pi
    assert not recovered.metrics.branch_switch


def test_output_contract_is_the_single_joint_dynamic_policy() -> None:
    config = ReplayConfig.load(CONFIG)
    assert config.output_contract.maximum_velocity_rad_s == pytest.approx(math.pi)
    assert config.output_contract.maximum_acceleration_rad_s2 == pytest.approx(4.0 * math.pi)
    assert config.output_contract.servo_period_ns == PERIOD_NS


def test_per_joint_output_velocity_boundaries_preserve_scalar_hard_contract() -> None:
    tracker = JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=math.pi,
        maximum_velocity_rad_s_per_joint=(
            1.5,
            1.5,
            1.5,
            1.5,
            1.5,
            1.5,
        ),
        maximum_acceleration_rad_s2=math.inf,
        servo_period_ns=PERIOD_NS,
    )
    tracker.reset((0.0,) * 6)
    exact = tracker.preview(
        (0.012, 0.0, 0.0, 0.012, 0.0, 0.0),
        generated_monotonic_ns=1_000_000_000,
    )
    assert exact.feasible
    assert exact.predicted_velocity_rad_s == pytest.approx(
        (1.5, 0.0, 0.0, 1.5, 0.0, 0.0)
    )
    assert exact.boundary_rad_s == pytest.approx(math.pi)
    assert exact.boundary_rad_s_per_joint == pytest.approx(
        (1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
    )

    wrist_above = tracker.preview(
        (0.0, 0.0, 0.0, 0.012000_001, 0.0, 0.0),
        generated_monotonic_ns=1_000_000_000,
    )
    assert not wrist_above.feasible
    assert wrist_above.violating_joint_indices == (3,)

    shoulder_same_speed = tracker.preview(
        (0.009600_001, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_000_000_000,
    )
    assert shoulder_same_speed.feasible


def test_legacy_scalar_output_velocity_boundary_remains_compatible() -> None:
    tracker = JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=1.0,
        servo_period_ns=PERIOD_NS,
    )
    tracker.reset((0.0,) * 6)
    result = tracker.preview(
        (0.008,) * 6,
        generated_monotonic_ns=1_000_000_000,
    )
    assert result.feasible
    assert result.boundary_rad_s_per_joint == pytest.approx((1.0,) * 6)


def test_output_acceleration_boundary_is_checked_before_commit() -> None:
    boundary = 4.0 * math.pi
    tracker = JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=math.pi,
        maximum_acceleration_rad_s2=boundary,
        servo_period_ns=PERIOD_NS,
    )
    _establish_zero_baseline(tracker)
    interval_ns = 16_000_000
    interval_s = interval_ns / 1e9
    exact_velocity = boundary * (PERIOD_NS / 1e9)
    exact_delta = exact_velocity * interval_s
    exact = tracker.preview(
        (exact_delta, 0, 0, 0, 0, 0),
        generated_monotonic_ns=1_000_000_000 + interval_ns,
    )
    assert exact.feasible
    assert exact.maximum_acceleration_rad_s2 == pytest.approx(boundary)

    above = tracker.preview(
        (exact_delta * 1.01, 0, 0, 0, 0, 0),
        generated_monotonic_ns=1_000_000_000 + interval_ns,
    )
    assert not above.feasible
    assert above.acceleration_violating_joint_indices == (0,)
    with pytest.raises(ValueError, match="infeasible"):
        tracker.commit(above)


def test_acceleration_preview_uses_previous_emitted_8ms_velocity() -> None:
    tracker = JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=1.0,
        maximum_acceleration_rad_s2=4.0 * math.pi,
        servo_period_ns=PERIOD_NS,
    )
    _establish_zero_baseline(tracker)
    first = tracker.preview(
        (0.0016, 0, 0, 0, 0, 0), generated_monotonic_ns=1_016_000_000
    )
    assert first.feasible
    tracker.commit(first)
    smooth = tracker.preview(
        (0.0032, 0, 0, 0, 0, 0), generated_monotonic_ns=1_032_000_000
    )
    assert smooth.previous_emitted_velocity_rad_s[0] == pytest.approx(0.1)
    assert smooth.predicted_velocity_rad_s[0] == pytest.approx(0.1)
    assert smooth.predicted_acceleration_rad_s2[0] == pytest.approx(0.0, abs=1e-12)
    assert smooth.feasible

    abrupt = tracker.preview(
        (0.0036, 0, 0, 0, 0, 0), generated_monotonic_ns=1_032_000_000
    )
    assert abrupt.predicted_velocity_rad_s[0] == pytest.approx(0.125)
    assert abrupt.predicted_acceleration_rad_s2[0] == pytest.approx(3.125)
