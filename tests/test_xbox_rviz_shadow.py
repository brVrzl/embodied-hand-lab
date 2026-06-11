from __future__ import annotations

import pytest

from teleop_tools.xbox_ros2 import XboxSnapshot
from teleop_tools.xbox_rviz_shadow import (
    HAND_PRESETS,
    TopicActionMirror,
    XboxPalmTargetAction,
    XboxPalmTargetMapper,
    XboxRvizPalmTargetState,
)


def test_palm_target_mapper_requires_deadman_and_maps_cartesian_axes() -> None:
    mapper = XboxPalmTargetMapper(
        max_translation_velocity_m_s=0.08,
        max_wrist_roll_velocity_rad_s=0.6,
        precision_scale=0.25,
    )
    inactive = mapper.map(XboxSnapshot(left_x=1.0, left_y=-1.0, right_x=0.5, right_y=-0.5))
    assert inactive.palm_velocity_m_s == [0.0, 0.0, 0.0]
    assert inactive.wrist_roll_velocity_rad_s == 0.0

    active = mapper.map(
        XboxSnapshot(
            left_x=1.0,
            left_y=-1.0,
            right_x=0.5,
            right_y=-0.5,
            buttons={"rb": True, "lb": True},
        )
    )
    assert active.palm_velocity_m_s == pytest.approx([0.02, -0.02, 0.01])
    assert active.wrist_roll_velocity_rad_s == pytest.approx(0.075)
    assert active.precision is True


def test_palm_target_state_tracks_target_and_updates_hand_without_backend() -> None:
    state = XboxRvizPalmTargetState([0.0] * 6)
    initial_palm = state.current_palm_position_m.copy()
    for _ in range(5):
        state.apply(
            action=XboxPalmTargetAction(
                palm_velocity_m_s=[0.04, 0.0, 0.0],
                wrist_roll_velocity_rad_s=0.0,
                deadman=True,
                hand_command="pinch",
            ),
            dt=0.1,
        )
    assert state.target_palm_position_m[0] - initial_palm[0] == pytest.approx(0.02)
    assert state.current_palm_position_m[0] - initial_palm[0] > 0.019
    assert state.target_error_m < 0.001
    assert state.hand_counts == HAND_PRESETS["pinch"]


def test_palm_target_state_applies_explicit_wrist_roll() -> None:
    state = XboxRvizPalmTargetState([0.0] * 6)
    state.apply(
        action=XboxPalmTargetAction(
            palm_velocity_m_s=[0.0, 0.0, 0.0],
            wrist_roll_velocity_rad_s=0.5,
            deadman=True,
        ),
        dt=0.1,
    )
    assert state.arm_joints_rad[5] == pytest.approx(0.05)


def test_palm_target_state_clips_target_workspace() -> None:
    state = XboxRvizPalmTargetState([0.0] * 6, target_workspace_radius_m=0.02)
    state.apply(
        action=XboxPalmTargetAction(
            palm_velocity_m_s=[100.0, 0.0, 0.0],
            wrist_roll_velocity_rad_s=0.0,
            deadman=True,
        ),
        dt=1.0,
    )
    target_offset = state.target_palm_position_m - state.initial_palm_position_m
    assert float((target_offset @ target_offset) ** 0.5) == pytest.approx(0.02)
    assert state.target_workspace_limited is True


def test_palm_target_state_resets_workspace_on_new_deadman_session() -> None:
    state = XboxRvizPalmTargetState([0.0] * 6, target_workspace_radius_m=0.02)
    move = XboxPalmTargetAction(
        palm_velocity_m_s=[1.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
        deadman=True,
    )
    state.apply(action=move, dt=0.1)
    first_target = state.target_palm_position_m.copy()
    state.apply(
        action=XboxPalmTargetAction(
            palm_velocity_m_s=[0.0, 0.0, 0.0],
            wrist_roll_velocity_rad_s=0.0,
            deadman=False,
        ),
        dt=0.1,
    )
    state.apply(action=move, dt=0.1)
    assert state.target_palm_position_m[0] > first_target[0] + 0.019


def test_palm_target_state_has_no_workspace_radius_by_default() -> None:
    state = XboxRvizPalmTargetState([0.0] * 6)
    initial_target = state.target_palm_position_m.copy()
    move = XboxPalmTargetAction(
        palm_velocity_m_s=[1.0, 0.0, 0.0],
        wrist_roll_velocity_rad_s=0.0,
        deadman=True,
    )
    for _ in range(2):
        state.apply(action=move, dt=0.1)
    assert state.target_palm_position_m[0] > initial_target[0] + 0.19
    assert state.target_workspace_limited is False


def test_palm_target_state_clips_to_documented_joint_margin() -> None:
    state = XboxRvizPalmTargetState([0.0, 0.0, 3.0, 0.0, 0.0, 0.0])
    assert state.arm_joints_rad[2] == pytest.approx(2.18166156)
    assert state.joint_limit_limited is True
    assert state.limited_joint_indices_1_based == [3]


def test_topic_action_mirror_converts_ros2_jog_and_times_out() -> None:
    mirror = TopicActionMirror(watchdog_sec=0.25)
    mirror.accept(
        '{"deadman": true, "palm_velocity_m_s": [0.01, 0.0, -0.02], '
        '"wrist_roll_velocity_rad_s": 0.1}',
        timestamp_sec=1.0,
    )

    active = mirror.action(timestamp_sec=1.2)
    assert active.deadman is True
    assert active.palm_velocity_m_s == pytest.approx([0.01, 0.0, -0.02])
    assert active.wrist_roll_velocity_rad_s == pytest.approx(0.1)

    stale = mirror.action(timestamp_sec=1.3)
    assert stale.deadman is False
    assert stale.palm_velocity_m_s == pytest.approx([0.0, 0.0, 0.0])
