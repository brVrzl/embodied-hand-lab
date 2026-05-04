from __future__ import annotations

import math

import pytest

from embodiment_core.config import load_yaml
from sim_maniskill.recorder import create_env_from_config


def test_pick_cube_jaka_rh56_reset_uses_custom_workspace() -> None:
    config = load_yaml("configs/sim/maniskill_jaka_rh56_pick_cube_state.yaml")

    env = create_env_from_config(config)
    try:
        env.reset(seed=0)
        summary = env.unwrapped.get_scene_summary()

        qpos = summary["qpos"][0]
        tcp_pose = summary["tcp_pose"][0]
        cube_pose = summary["cube_pose"][0]
        base_pose = summary["robot_base_pose"][0]
        table = summary["table"]
        base_to_cube_xy = math.dist(base_pose[:2], cube_pose[:2])

        assert len(qpos) == 18
        assert summary["start_pose"] == "pregrasp"
        assert qpos[:6] == pytest.approx([0.123, 0.429, 1.496, -1.447, -0.019, -2.164], abs=1e-6)
        assert qpos[6:] == pytest.approx([0.0] * 12, abs=1e-6)
        assert table["length_m"] == pytest.approx(1.20)
        assert table["width_m"] == pytest.approx(0.60)
        assert table["robot_mount_offset_from_right_edge_m"] == pytest.approx(0.25)
        assert table["robot_mount_offset_from_front_edge_m"] == pytest.approx(0.30)
        assert -0.18 < cube_pose[0] < -0.03
        assert 0.40 < base_to_cube_xy < 0.60
        assert 0.05 < abs(tcp_pose[0] - cube_pose[0]) < 0.12
        assert 0.15 < abs(tcp_pose[2] - cube_pose[2]) < 0.30
        assert base_pose[0] == pytest.approx(-0.615, abs=1e-5)
    finally:
        env.close()


def test_lift_cube_jaka_rh56_exposes_contact_only_success_terms() -> None:
    config = load_yaml("configs/sim/maniskill_jaka_rh56_lift_cube_state.yaml")

    env = create_env_from_config(config)
    try:
        env.reset(seed=1)
        info = env.unwrapped.evaluate()

        assert "success" in info
        assert "is_lifted" in info
        assert "is_grasped" in info
        assert "is_robot_static" in info
        assert "object_height" in info
        assert "lift_success_height" in info
        assert float(info["lift_success_height"].reshape(-1)[0]) == pytest.approx(0.075)
    finally:
        env.close()
