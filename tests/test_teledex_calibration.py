from __future__ import annotations

import math

import numpy as np
import pytest

from teleop_tools.teledex_calibration import fit_phone_to_robot_rotation


def test_fit_phone_to_robot_rotation_recovers_arbitrary_yaw() -> None:
    yaw = math.radians(31.0)
    robot_from_phone = np.asarray(
        [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    )
    phone_from_robot = robot_from_phone.T
    displacements = {
        "x": phone_from_robot @ np.asarray([0.12, 0.0, 0.0]),
        "y": phone_from_robot @ np.asarray([0.0, 0.10, 0.0]),
        "z": phone_from_robot @ np.asarray([0.0, 0.0, 0.08]),
    }
    fitted, quality = fit_phone_to_robot_rotation(displacements)
    assert fitted == pytest.approx(robot_from_phone, abs=1e-9)
    assert quality["max_fit_angular_error_deg"] == pytest.approx(0.0, abs=1e-6)


def test_signed_permutation_snaps_noisy_gestures_to_exact_axis_map() -> None:
    displacements = {
        "x": [-0.01, -0.12, 0.015],
        "y": [0.10, -0.01, 0.004],
        "z": [-0.008, 0.001, 0.09],
    }
    fitted, quality = fit_phone_to_robot_rotation(
        displacements,
        mapping_mode="signed_permutation",
    )
    assert fitted == pytest.approx(
        np.asarray([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    )
    assert set(np.unique(fitted)).issubset({-1.0, 0.0, 1.0})
    assert quality["mapping_mode"] == "signed_permutation"
    assert min(quality["signed_axis_alignment_cosine"].values()) > 0.98


def test_fit_rejects_short_or_left_handed_capture() -> None:
    with pytest.raises(ValueError, match="only"):
        fit_phone_to_robot_rotation(
            {"x": [0.01, 0.0, 0.0], "y": [0.0, 0.1, 0.0], "z": [0.0, 0.0, 0.1]}
        )
    with pytest.raises(ValueError, match="left-handed"):
        fit_phone_to_robot_rotation(
            {"x": [0.1, 0.0, 0.0], "y": [0.0, 0.1, 0.0], "z": [0.0, 0.0, -0.1]}
        )
