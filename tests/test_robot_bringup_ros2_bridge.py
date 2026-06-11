from __future__ import annotations

import math

from embodiment_core.types import HandState, JointState
from robot_bringup.ros2_bridge import (
    SCHEMA_VERSION,
    build_hand_state_payload,
    build_joint_state_dict,
    evaluate_hand_safety,
    rpy_to_quaternion_xyzw,
)


def test_rpy_to_quaternion_identity_and_yaw() -> None:
    assert rpy_to_quaternion_xyzw(0.0, 0.0, 0.0) == [0.0, 0.0, 0.0, 1.0]

    qx, qy, qz, qw = rpy_to_quaternion_xyzw(0.0, 0.0, math.pi / 2.0)
    assert qx == 0.0
    assert qy == 0.0
    assert qz == math.sin(math.pi / 4.0)
    assert qw == math.cos(math.pi / 4.0)


def test_joint_state_dict_uses_ros_units_without_renaming_to_counts() -> None:
    payload = build_joint_state_dict(
        JointState(
            names=["joint_1", "joint_2"],
            positions=[0.1, -0.2],
            velocities=[0.0, 0.0],
            efforts=[],
        )
    )

    assert payload["frame_id"] == "jaka_base"
    assert payload["name"] == ["joint_1", "joint_2"]
    assert payload["position_rad"] == [0.1, -0.2]
    assert payload["velocity_rad_s"] == [0.0, 0.0]
    assert payload["effort"] == []


def test_hand_state_payload_keeps_vendor_counts_explicit() -> None:
    payload = build_hand_state_payload(
        HandState(
            mode="idle",
            finger_positions=[1000, 900, 800, 700, 600, 500],
            finger_currents=[0, 1, 2, 3, 4, 5],
            force_estimate=[-1, 0, 2, 3, -4, 5],
            contact_flags=[False, False, True, True, False, True],
        ),
        backend_mode="serial_protocol",
        timestamp_sec=123.0,
    )

    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["timestamp_sec"] == 123.0
    assert payload["backend_mode"] == "serial_protocol"
    assert payload["finger_state"]["angle_count_0_1000"] == [1000, 900, 800, 700, 600, 500]
    assert payload["finger_state"]["force_count"] == [-1, 0, 2, 3, -4, 5]
    assert payload["units"]["angle_count_0_1000"] == "vendor_count_open_1000_close_0"
    assert payload["units"]["force_count"] == "signed_vendor_raw_count"


def test_evaluate_hand_safety_flags_current_and_force_stop() -> None:
    status = evaluate_hand_safety(
        HandState(
            mode="idle",
            finger_positions=[1000] * 6,
            finger_currents=[100, 200, 901, 100, 100, 100],
            force_estimate=[10, -950, 10, 10, 10, 10],
            contact_flags=[False] * 6,
        ),
        {
            "estop_enabled": True,
            "current_warn_count": 700,
            "current_stop_count": 900,
            "force_stop_count": 900,
        },
    )

    assert status["current_warned"] is True
    assert status["stop"] is True
    assert status["reasons"] == ["hand_current_stop", "hand_force_stop"]
