from __future__ import annotations

import math

import numpy as np
import pytest

from embodiment_core.robot_limits import (
    PERIODIC_JOINT_INDICES,
    select_nearest_equivalent_joint_branch,
)
from jaka_driver_adapter.palm_target_ik import (
    directional_joint_limit_scale,
    safe_joint_limits_rad,
)
from quest_jaka_sim import ReplayConfig, SharedJakaTargetGenerator
from quest_jaka_sim.simulation import FeasibilityReason


CONFIG = "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"


def _joints(j4: float = 0.0, j6: float = 0.0) -> list[float]:
    result = [0.0] * 6
    result[3] = j4
    result[5] = j6
    return result


def test_ik_minus_five_degrees_uses_measured_355_degree_branch() -> None:
    candidate = _joints(j4=math.radians(-5.0))
    measured = _joints(j4=math.radians(355.0))
    selected, offsets = select_nearest_equivalent_joint_branch(candidate, measured)
    assert selected[3] == pytest.approx(measured[3])
    assert offsets[3] == 1


def test_ik_355_degrees_uses_measured_minus_five_degree_branch() -> None:
    candidate = _joints(j4=math.radians(355.0))
    measured = _joints(j4=math.radians(-5.0))
    selected, offsets = select_nearest_equivalent_joint_branch(candidate, measured)
    assert selected[3] == pytest.approx(measured[3])
    assert offsets[3] == -1


def test_small_periodic_steps_do_not_accumulate_extra_revolutions() -> None:
    previous = _joints(j4=math.radians(350.0), j6=math.radians(-350.0))
    total = np.zeros(6)
    for step in range(20):
        candidate = _joints(
            j4=math.radians(350.0 + step * 0.2),
            j6=math.radians(-350.0 - step * 0.2),
        )
        selected, _ = select_nearest_equivalent_joint_branch(candidate, previous)
        current = np.asarray(selected)
        total += np.abs(current - np.asarray(previous))
        previous = list(selected)
    assert total[3] < math.radians(5.0)
    assert total[5] < math.radians(5.0)


def test_fresh_measured_recapture_resets_branch_and_winding_reference() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    measured = np.asarray(generator.last_safe_joint_target, dtype=float)
    measured[3] = math.radians(354.5)
    measured[5] = math.radians(-354.5)
    generator.synchronize_authoritative_arm_joints(measured.tolist())
    assert generator.last_safe_joint_target == pytest.approx(measured)
    assert generator.episode_winding_rad == pytest.approx((0.0,) * 6)
    generator.observe_episode_winding(
        [*measured[:3], measured[3] + 0.05, measured[4], measured[5] - 0.05]
    )
    assert generator.episode_winding_rad[3] == pytest.approx(0.05)
    generator.capture_reference()
    assert generator.episode_winding_rad == pytest.approx((0.0,) * 6)


def test_total_periodic_travel_is_diagnostic_not_a_hard_stop() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    start = np.asarray(generator.last_safe_joint_target, dtype=float)
    generator.synchronize_authoritative_arm_joints(start.tolist())
    samples = []
    for _ in range(2):
        samples.extend(
            [
                [*start[:5], start[5] + math.radians(60.0)],
                start.tolist(),
                [*start[:5], start[5] - math.radians(60.0)],
                start.tolist(),
            ]
        )
    for sample in samples:
        generator.observe_episode_winding(sample)
    result = generator.evaluate(generator.current_tcp_pose, dt_s=1.0 / 60.0)
    assert result.accepted
    assert result.reason is FeasibilityReason.ACCEPTED
    assert result.metrics.episode_winding_rad[5] > 5.0
    lower, upper = safe_joint_limits_rad(generator.config.feasibility.joint_limit_margin_rad)[5]
    assert all(lower <= sample[5] <= upper for sample in samples)


def test_large_periodic_candidate_step_is_not_a_branch_hard_stop() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    seed = generator.last_safe_joint_target.copy()
    candidate = seed.copy()
    candidate[3] += 1.0

    def fake_ik(**_: object) -> bool:
        generator.ik.set_arm_joints_rad(candidate.tolist())
        return True

    generator.ik.apply_position_target = fake_ik  # type: ignore[method-assign]
    result = generator.evaluate(
        generator.current_tcp_pose,
        dt_s=1.0 / 60.0,
        fresh_measured_joint_position_rad=seed.tolist(),
    )
    assert not result.accepted
    assert result.reason is not FeasibilityReason.JOINT_BRANCH_DISCONTINUITY
    assert result.reason is not FeasibilityReason.EPISODE_WINDING_EXCEEDED
    assert not result.metrics.hard_stop_required


def test_wrist_sized_j6_step_is_not_a_branch_hard_stop() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    seed = generator.last_safe_joint_target.copy()
    candidate = seed.copy()
    candidate[5] += 0.24

    def fake_ik(**_: object) -> bool:
        generator.ik.set_arm_joints_rad(candidate.tolist())
        return True

    generator.ik.apply_position_target = fake_ik  # type: ignore[method-assign]
    result = generator.evaluate(
        generator.current_tcp_pose,
        dt_s=1.0 / 60.0,
        fresh_measured_joint_position_rad=seed.tolist(),
    )
    assert result.accepted
    assert result.reason is FeasibilityReason.ACCEPTED
    assert result.metrics.output_velocity_violating_joint_indices == (5,)
    assert result.metrics.branch_equivalent_offset == (0, 0, 0, 0, 0, 0)


def test_equivalent_branch_selection_does_not_make_wrist_motion_hard() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    seed = generator.last_safe_joint_target.copy()
    # The solver returned a representation one revolution away from the
    # measured branch.  The nearest legal representation is used; dynamic
    # output estimates remain diagnostic and do not create a branch fault.
    candidate = seed.copy()
    candidate[3] += 5.5

    def fake_ik(**_: object) -> bool:
        generator.ik.set_arm_joints_rad(candidate.tolist())
        return True

    generator.ik.apply_position_target = fake_ik  # type: ignore[method-assign]
    result = generator.evaluate(
        generator.current_tcp_pose,
        dt_s=1.0 / 60.0,
        fresh_measured_joint_position_rad=seed.tolist(),
    )
    assert not result.accepted
    assert result.reason is not FeasibilityReason.JOINT_BRANCH_DISCONTINUITY
    assert result.reason is not FeasibilityReason.EPISODE_WINDING_EXCEEDED
    assert not result.metrics.hard_stop_required
    assert result.metrics.branch_equivalent_offset[3] != 0


def test_no_legal_equivalent_branch_is_terminal() -> None:
    with pytest.raises(ValueError, match="no equivalent representation"):
        select_nearest_equivalent_joint_branch(
            _joints(j4=20.0),
            _joints(),
        )


def test_periodic_joint_scope_matches_jaka_full_range_axes() -> None:
    assert PERIODIC_JOINT_INDICES == (0, 3, 5)


def test_directional_joint_limit_scale_covers_all_six_joints() -> None:
    margin = math.radians(5.0)
    zone = math.radians(20.0)
    safe_limits = safe_joint_limits_rad(margin)
    for index, (lower, upper) in enumerate(safe_limits):
        far_upper = upper - zone - math.radians(1.0)
        near_upper = upper - zone * 0.5
        far_lower = lower + zone + math.radians(1.0)
        near_lower = lower + zone * 0.5

        joints = np.zeros(6)
        joints[index] = far_upper
        assert directional_joint_limit_scale(
            joints, np.eye(6)[index] * 0.01, margin_rad=margin, avoidance_zone_rad=zone
        ) == pytest.approx(1.0)

        joints[index] = near_upper
        outward = directional_joint_limit_scale(
            joints, np.eye(6)[index] * 0.01, margin_rad=margin, avoidance_zone_rad=zone
        )
        assert 0.0 < outward < 1.0
        assert directional_joint_limit_scale(
            joints, -np.eye(6)[index] * 0.01, margin_rad=margin, avoidance_zone_rad=zone
        ) == pytest.approx(1.0)

        joints[index] = upper
        assert directional_joint_limit_scale(
            joints, np.eye(6)[index] * 0.01, margin_rad=margin, avoidance_zone_rad=zone
        ) == pytest.approx(0.0)

        joints[index] = far_lower
        assert directional_joint_limit_scale(
            joints, -np.eye(6)[index] * 0.01, margin_rad=margin, avoidance_zone_rad=zone
        ) == pytest.approx(1.0)
        joints[index] = near_lower
        inward = directional_joint_limit_scale(
            joints, -np.eye(6)[index] * 0.01, margin_rad=margin, avoidance_zone_rad=zone
        )
        assert 0.0 < inward < 1.0
        assert directional_joint_limit_scale(
            joints, np.eye(6)[index] * 0.01, margin_rad=margin, avoidance_zone_rad=zone
        ) == pytest.approx(1.0)

        joints[index] = lower
        assert directional_joint_limit_scale(
            joints, -np.eye(6)[index] * 0.01, margin_rad=margin, avoidance_zone_rad=zone
        ) == pytest.approx(0.0)
