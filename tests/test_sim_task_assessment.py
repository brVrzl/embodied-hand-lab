from __future__ import annotations

from sim_maniskill.task_assessment import assess_pickcube_jaka_rh56_scene


def test_pickcube_jaka_rh56_task_assessment_accepts_expected_scene() -> None:
    assessment = assess_pickcube_jaka_rh56_scene(
        {
            "qpos": [[0.0] * 18],
            "tcp_pose": [[-0.12, 0.0, 0.22, 1.0, 0.0, 0.0, 0.0]],
            "cube_pose": [[-0.08, 0.0, 0.02, 1.0, 0.0, 0.0, 0.0]],
            "goal_pose": [[-0.10, 0.0, 0.10, 1.0, 0.0, 0.0, 0.0]],
            "table": {"length_m": 1.2, "width_m": 0.6},
            "hand_joint_names": [f"hand_{idx}" for idx in range(6)],
            "arm_joint_names": [f"arm_{idx}" for idx in range(6)],
            "start_pose": "pregrasp",
            "cube_spawn_half_size": 0.07,
        }
    )

    assert assessment["verdict"] == "suitable_for_offline_pipeline_and_action_representation"
    assert assessment["checks"]["robot_has_6_arm_dof"] is True
    assert assessment["checks"]["robot_has_6_controlled_hand_dof"] is True
    assert "kinematically carry" in assessment["limitations"][0]
