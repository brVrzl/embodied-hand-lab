from __future__ import annotations

import pytest

from teleop_tools.xbox_ros2 import XboxPalmTargetMapper, XboxSnapshot, apply_deadzone


def _mapper() -> XboxPalmTargetMapper:
    return XboxPalmTargetMapper(
        max_translation_velocity_m_s=0.08,
        max_wrist_roll_velocity_rad_s=0.6,
        precision_scale=0.25,
    )


def test_apply_deadzone_rescales_remaining_axis_range() -> None:
    assert apply_deadzone(0.1, 0.12) == 0.0
    assert apply_deadzone(1.0, 0.12) == 1.0
    assert apply_deadzone(-0.56, 0.12) == pytest.approx(-0.5)


def test_mapper_requires_deadman_and_supports_precision() -> None:
    mapper = _mapper()
    idle = mapper.map(XboxSnapshot(left_y=-1.0))
    assert idle.palm_velocity_m_s == [0.0] * 3
    assert idle.wrist_roll_velocity_rad_s == 0.0

    active = mapper.map(
        XboxSnapshot(
            left_y=-1.0,
            left_x=0.5,
            right_y=0.25,
            right_x=-0.5,
            buttons={"rb": True, "lb": True},
        )
    )
    assert active.palm_velocity_m_s == pytest.approx([0.02, -0.01, -0.005])
    assert active.wrist_roll_velocity_rad_s == pytest.approx(-0.075)


def test_mapper_accepts_configured_axis_direction_calibration() -> None:
    mapper = XboxPalmTargetMapper(
        max_translation_velocity_m_s=0.1,
        max_wrist_roll_velocity_rad_s=1.0,
        precision_scale=0.25,
        translation_axis_map={
            "x": {"source": "left_x", "sign": 1.0},
            "y": {"source": "left_y", "sign": 1.0},
            "z": {"source": "right_y", "sign": 1.0},
        },
        wrist_roll_axis_map={"source": "right_x", "sign": -1.0},
    )
    action = mapper.map(
        XboxSnapshot(
            left_x=0.5,
            left_y=-0.25,
            right_y=0.75,
            right_x=0.2,
            buttons={"rb": True},
        )
    )

    assert action.palm_velocity_m_s == pytest.approx([0.05, -0.025, 0.075])
    assert action.wrist_roll_velocity_rad_s == pytest.approx(-0.2)


def test_mapper_allows_open_without_deadman_but_gates_close_and_pinch() -> None:
    mapper = _mapper()
    assert mapper.map(XboxSnapshot(buttons={"a": True})).hand_command == "open"
    mapper.map(XboxSnapshot())
    assert mapper.map(XboxSnapshot(buttons={"b": True})).hand_command is None
    mapper.map(XboxSnapshot())
    assert mapper.map(XboxSnapshot(buttons={"rb": True, "b": True})).hand_command == "close"


def test_mapper_low_pass_filters_velocity_and_resets_on_deadman_release() -> None:
    mapper = XboxPalmTargetMapper(
        max_translation_velocity_m_s=0.1,
        max_wrist_roll_velocity_rad_s=1.0,
        precision_scale=0.25,
        velocity_filter_time_constant_sec=0.1,
    )
    first = mapper.map(XboxSnapshot(left_y=-1.0, buttons={"rb": True}), timestamp_sec=1.0)
    assert first.palm_velocity_m_s == pytest.approx([0.1, 0.0, 0.0])
    second = mapper.map(XboxSnapshot(left_y=0.0, buttons={"rb": True}), timestamp_sec=1.1)
    assert second.palm_velocity_m_s[0] == pytest.approx(0.05)
    mapper.map(XboxSnapshot(left_y=0.0), timestamp_sec=1.2)
    after_reset = mapper.map(XboxSnapshot(left_y=0.0, buttons={"rb": True}), timestamp_sec=2.0)
    assert after_reset.palm_velocity_m_s == pytest.approx([0.0, 0.0, 0.0])
