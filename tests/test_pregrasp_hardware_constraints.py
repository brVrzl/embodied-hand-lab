from __future__ import annotations

from pregrasp.hardware_constraints import evaluate_rh56_hardware_constraints
from pregrasp.primitives import rh56_default_primitives


def test_default_primitives_are_hardware_feasible() -> None:
    for primitive in rh56_default_primitives():
        result = evaluate_rh56_hardware_constraints(primitive.hand_command)
        assert result.feasible, primitive.name


def test_thumb_index_blocking_rejects_deep_opposed_closure() -> None:
    result = evaluate_rh56_hardware_constraints(
        [0.82, 0.20, 0.20, 0.20, 0.78, 0.92],
    )

    assert not result.feasible
    assert result.thumb_index_blocking_risk >= 0.70
    assert "thumb_index_blocking_risk" in result.reasons
