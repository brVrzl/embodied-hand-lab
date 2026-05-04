from __future__ import annotations

from typing import Any


ASSESSMENT_SCHEMA_VERSION = "jaka_rh56_sim_task_assessment_v0.1"


def assess_pickcube_jaka_rh56_scene(summary: dict[str, Any]) -> dict[str, Any]:
    qpos = summary.get("qpos", [[0.0] * 18])[0]
    tcp_pose = summary.get("tcp_pose", [[0.0] * 7])[0]
    cube_pose = summary.get("cube_pose", [[0.0] * 7])[0]
    goal_pose = summary.get("goal_pose", [[0.0] * 7])[0]
    table = summary.get("table", {})
    hand_joint_names = summary.get("hand_joint_names", [])
    arm_joint_names = summary.get("arm_joint_names", [])
    start_pose = summary.get("start_pose")
    cube_spawn_half_size = float(summary.get("cube_spawn_half_size", 0.0))

    tcp_cube_dx = abs(float(tcp_pose[0]) - float(cube_pose[0]))
    tcp_cube_dz = abs(float(tcp_pose[2]) - float(cube_pose[2]))
    goal_height = float(goal_pose[2]) if len(goal_pose) >= 3 else 0.0
    table_length = float(table.get("length_m", 0.0))
    table_width = float(table.get("width_m", 0.0))

    checks = {
        "robot_has_6_arm_dof": len(arm_joint_names) == 6,
        "robot_has_6_controlled_hand_dof": len(hand_joint_names) == 6,
        "full_model_has_expected_qpos": len(qpos) == 18,
        "starts_near_pregrasp": start_pose == "pregrasp" and 0.04 <= tcp_cube_dx <= 0.14 and 0.12 <= tcp_cube_dz <= 0.35,
        "workspace_is_desktop_scale": 0.8 <= table_length <= 1.5 and 0.4 <= table_width <= 0.9,
        "cube_spawn_is_small_enough_for_controlled_eval": 0.03 <= cube_spawn_half_size <= 0.12,
        "goal_requires_lift": goal_height > 0.03,
    }

    strengths = [
        "JAKA mini2 + RH56 combined MJCF is loaded as a single embodiment.",
        "State-only reset works without requiring an RGB-D renderer.",
        "Workspace, cube, and goal geometry are desktop-scale and suitable for schema/action tests.",
        "The pregrasp start pose makes short-horizon palm-frame demonstrations practical.",
    ]
    limitations = [
        "Current privileged oracle may kinematically carry the cube, so success is pipeline success rather than physical grasp success.",
        "RH56 contact and force simulation are not calibrated against the real hand.",
        "The controlled hand joints are a simplified 6-DOF interface over an 18-qpos model.",
        "The task is single-object PickCube; it is too narrow for final paper claims without later object and grasp diversity.",
    ]
    recommended_use = [
        "Validate episode schema, transition keys, and LeRobot-style export.",
        "Validate palm-frame action fields: ee_delta, hand_cmd, hand_code_id, close_strength.",
        "Train and evaluate a state-only BC or retrieval baseline before using real hardware.",
        "Use RGB-D only after renderer availability is confirmed; state-only is enough for the first baseline.",
    ]
    not_recommended_use = [
        "Do not report sim success as real RH56 grasping performance.",
        "Do not use this task alone to claim robust dexterous manipulation.",
        "Do not tune pseudo-tactile thresholds from simulated contact forces before real RH56 feedback is measured.",
    ]

    score = sum(1 for value in checks.values() if value) / max(len(checks), 1)
    if score >= 0.85:
        verdict = "suitable_for_offline_pipeline_and_action_representation"
    elif score >= 0.65:
        verdict = "usable_with_caveats"
    else:
        verdict = "not_ready"

    return {
        "schema_version": ASSESSMENT_SCHEMA_VERSION,
        "task": "PickCubeJakaRH56-v1",
        "verdict": verdict,
        "suitability_score": score,
        "checks": checks,
        "strengths": strengths,
        "limitations": limitations,
        "recommended_use": recommended_use,
        "not_recommended_use": not_recommended_use,
        "next_gate": {
            "name": "state_only_palm_frame_oracle_smoke",
            "pass_condition": "collect >=5 episodes, validate structured schema, and inspect action-field ranges",
        },
    }


def format_assessment_markdown(assessment: dict[str, Any]) -> str:
    lines = [
        "# JAKA RH56 Simulation Task Assessment",
        "",
        f"Schema version: `{assessment['schema_version']}`",
        "",
        f"Task: `{assessment['task']}`",
        "",
        f"Verdict: `{assessment['verdict']}`",
        "",
        f"Suitability score: `{assessment['suitability_score']:.2f}`",
        "",
        "## Checks",
        "",
    ]
    for key, value in assessment["checks"].items():
        mark = "pass" if value else "fail"
        lines.append(f"- `{key}`: {mark}")
    for section in ("strengths", "limitations", "recommended_use", "not_recommended_use"):
        title = section.replace("_", " ").title()
        lines.extend(["", f"## {title}", ""])
        for item in assessment[section]:
            lines.append(f"- {item}")
    lines.extend(
        [
            "",
            "## Next Gate",
            "",
            f"- `{assessment['next_gate']['name']}`",
            f"- {assessment['next_gate']['pass_condition']}",
            "",
            "# 中文版本",
            "",
            "结论：当前 `PickCubeJakaRH56-v1` 适合推进离线仿真闭环和 action representation 验证，但不适合直接作为真实 RH56 抓取物理性能证据。",
            "",
            "最推荐的用途是：先跑 state-only palm-frame oracle，验证 episode schema、structured export、训练字段和评测统计。等真实 RH56 feedback 接入后，再把 pseudo-tactile 阈值和 contact 相关结论迁移到实机。",
        ]
    )
    return "\n".join(lines)
