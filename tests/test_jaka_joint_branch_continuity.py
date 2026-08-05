from __future__ import annotations

import math

import numpy as np
import pytest

from embodiment_core.robot_limits import (
    PERIODIC_JOINT_INDICES,
    select_nearest_equivalent_joint_branch,
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


def test_winding_guard_rejects_and_holds_before_a_full_turn() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    start = np.asarray(generator.last_safe_joint_target, dtype=float)
    generator.synchronize_authoritative_arm_joints(start.tolist())
    for step in range(1, 6):
        generator.observe_episode_winding(
            [*start[:3], start[3] + step * 1.02, start[4], start[5] - 0.02]
        )
    result = generator.evaluate(generator.current_tcp_pose, dt_s=1.0 / 60.0)
    assert not result.accepted
    assert result.reason is FeasibilityReason.EPISODE_WINDING_EXCEEDED
    assert result.joint_target_rad is None
    assert result.metrics.episode_winding_rad[3] > 5.0


def test_branch_jump_guard_rejects_even_when_candidate_is_not_a_wrap() -> None:
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
    assert result.reason is FeasibilityReason.JOINT_BRANCH_DISCONTINUITY
    assert result.joint_target_rad is None


def test_periodic_joint_scope_matches_jaka_full_range_axes() -> None:
    assert PERIODIC_JOINT_INDICES == (0, 3, 5)
