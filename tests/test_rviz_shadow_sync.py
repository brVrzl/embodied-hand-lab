from __future__ import annotations

from types import SimpleNamespace

from teleop_tools.rviz_shadow_sync import extract_arm_joints_from_joint_state


def test_extract_arm_joints_accepts_real_and_rviz_joint_names() -> None:
    real = SimpleNamespace(
        name=[f"joint_{index}" for index in range(1, 7)],
        position=[float(index) for index in range(6)],
    )
    rviz = SimpleNamespace(
        name=[f"jaka_joint_{index}" for index in range(1, 7)],
        position=[float(index + 10) for index in range(6)],
    )

    assert extract_arm_joints_from_joint_state(real) == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert extract_arm_joints_from_joint_state(rviz) == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]


def test_extract_arm_joints_falls_back_to_first_six_positions() -> None:
    message = SimpleNamespace(name=[], position=[1, 2, 3, 4, 5, 6, 7])

    assert extract_arm_joints_from_joint_state(message) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert extract_arm_joints_from_joint_state(SimpleNamespace(name=[], position=[1, 2])) is None
