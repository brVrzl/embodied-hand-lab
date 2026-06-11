from __future__ import annotations

import pytest

from robot_bringup.rviz_joint_state_bridge import (
    RH56_URDF_JOINTS,
    map_arm_joint_names,
    rh56_counts_to_urdf_joint_state,
)


def test_arm_joint_names_map_to_preview_urdf() -> None:
    assert map_arm_joint_names(["joint_1", "joint_6"]) == ["jaka_joint_1", "jaka_joint_6"]


def test_rh56_vendor_counts_map_open_to_zero_and_close_to_joint_limit() -> None:
    open_state = rh56_counts_to_urdf_joint_state([1000.0] * 6)
    assert set(open_state) == set(RH56_URDF_JOINTS.values())
    assert list(open_state.values()) == [0.0] * 6

    close_state = rh56_counts_to_urdf_joint_state([0.0] * 6)
    assert close_state["rh56_R_index_MCP_joint"] == pytest.approx(1.70)
    assert close_state["rh56_R_thumb_MCP_joint2"] == pytest.approx(0.50)
