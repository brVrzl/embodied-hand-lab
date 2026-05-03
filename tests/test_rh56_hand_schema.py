from __future__ import annotations

import numpy as np

from rh56_driver.hand_schema import (
    CANONICAL_HAND_ORDER,
    DEFAULT_RH56_CALIBRATION,
    RH56_INTERNAL_ORDER,
    RH56_PROTOCOL_ORDER,
    apply_delta,
    build_hand_state,
    canonical_to_raw,
    compute_delta,
    denormalize_canonical,
    moving_direction,
    normalize_raw,
    raw_to_canonical,
)


def test_canonical_raw_roundtrip() -> None:
    canonical = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    raw = canonical_to_raw(canonical, raw_order=RH56_INTERNAL_ORDER)
    restored = raw_to_canonical(raw, raw_order=RH56_INTERNAL_ORDER)

    assert np.allclose(raw, [0.5, 0.6, 0.1, 0.2, 0.3, 0.4])
    assert np.allclose(restored, canonical)


def test_official_rh56_protocol_order_roundtrip() -> None:
    canonical = [10, 20, 30, 40, 50, 60]

    protocol = canonical_to_raw(canonical, raw_order=RH56_PROTOCOL_ORDER)
    restored = raw_to_canonical(protocol, raw_order=RH56_PROTOCOL_ORDER)

    assert np.allclose(protocol, [40, 30, 20, 10, 50, 60])
    assert np.allclose(restored, canonical)


def test_normalize_and_denormalize_use_calibration() -> None:
    raw_open = canonical_to_raw([1000, 1000, 1000, 1000, 1000, 1000], raw_order=RH56_INTERNAL_ORDER)
    raw_close = canonical_to_raw([0, 0, 0, 0, 0, 0], raw_order=RH56_INTERNAL_ORDER)

    assert np.allclose(normalize_raw(raw_open, calibration=DEFAULT_RH56_CALIBRATION), [0, 0, 0, 0, 0, 0])
    assert np.allclose(normalize_raw(raw_close, calibration=DEFAULT_RH56_CALIBRATION), [1, 1, 1, 1, 1, 1])

    raw_mid = denormalize_canonical([0.25] * 6, calibration=DEFAULT_RH56_CALIBRATION)
    assert np.allclose(raw_to_canonical(raw_mid), [750] * 6)


def test_delta_is_limited_and_applied_with_clip() -> None:
    current = [0.0] * len(CANONICAL_HAND_ORDER)
    target = [1.0] * len(CANONICAL_HAND_ORDER)

    delta = compute_delta(current, target, limit=0.05)
    next_cmd = apply_delta(current, delta, limit=0.05)

    assert np.allclose(delta, [0.05] * 6)
    assert np.allclose(next_cmd, [0.05] * 6)
    assert apply_delta([0.98] * 6, [0.2] * 6, limit=0.05) == [1.0] * 6


def test_build_hand_state_keeps_raw_and_canonical_fields() -> None:
    raw_positions = [500, 250, 1000, 750, 500, 250]
    state = build_hand_state(
        raw_positions=raw_positions,
        raw_velocities=[1, 2, 3, 4, 5, 6],
        raw_currents=[10, 20, 30, 40, 50, 60],
        raw_forces=[100, 200, 300, 400, 500, 600],
        raw_contact_binary=[True, False, True, False, True, False],
        calibration=DEFAULT_RH56_CALIBRATION,
        last_command=[0.2] * 6,
        command=[0.3, 0.1, 0.2, 0.25, 0.5, 0.75],
    )

    assert state["finger_positions"] == raw_positions
    assert state["inspire6"]["positions"] == [1000, 750, 500, 250, 500, 250]
    assert np.allclose(state["inspire6"]["normalized_positions"], [0.0, 0.25, 0.5, 0.75, 0.5, 0.75])
    assert state["rh56_raw"]["calibration"]["index"]["raw_open"] == 1000.0
    assert moving_direction([0.2] * 6, [0.3, 0.1, 0.2, 0.25, 0.5, 0.75]) == [1, -1, 0, 1, 1, 1]
