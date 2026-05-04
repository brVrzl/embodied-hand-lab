from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np

from rh56_driver.hand_schema import (
    DEFAULT_EE_ROTATION_DELTA_LIMIT_RAD,
    DEFAULT_EE_TRANSLATION_DELTA_LIMIT_M,
    RH56_INTERNAL_ORDER,
    canonical_to_raw,
    clip_ee_delta,
)
from sim_maniskill.teleop import _estimate_tcp_position_jacobian


def _arr(value: Any) -> np.ndarray:
    if hasattr(value, "detach") and hasattr(value, "cpu") and hasattr(value, "numpy"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float32).reshape(-1)


def _bool(value: Any) -> bool:
    return bool(np.asarray(_arr(value)).reshape(-1)[0])


def _ik_action(env: Any, target: np.ndarray, hand_cmd_canonical: np.ndarray) -> tuple[np.ndarray, list[float]]:
    agent = env.unwrapped.agent
    tcp = _arr(agent.tcp_pose.p)[:3]
    delta = target.astype(np.float32, copy=False) - tcp
    ee_delta = clip_ee_delta(
        [*delta.tolist(), 0.0, 0.0, 0.0],
        translation_limit_m=DEFAULT_EE_TRANSLATION_DELTA_LIMIT_M,
        rotation_limit_rad=DEFAULT_EE_ROTATION_DELTA_LIMIT_RAD,
    )
    jac = _estimate_tcp_position_jacobian(agent, eps=1e-3, arm_dof=6)
    lhs = jac @ jac.T + (0.05**2) * np.eye(3, dtype=np.float32)
    q_delta = jac.T @ np.linalg.solve(lhs, delta * 1.25)
    q_delta = np.clip(q_delta, -0.05, 0.05)

    hand_internal = np.asarray(canonical_to_raw(hand_cmd_canonical.tolist(), raw_order=RH56_INTERNAL_ORDER), dtype=np.float32)
    hand_action_internal = np.clip(hand_internal * 2.0 - 1.0, -1.0, 1.0)
    action = np.zeros(12, dtype=np.float32)
    action[:6] = np.clip(q_delta / 0.1, -1.0, 1.0)
    action[6:] = hand_action_internal
    return action, ee_delta


def _target_for_step(env: Any, step_idx: int) -> tuple[np.ndarray, np.ndarray, str]:
    obj = _arr(env.unwrapped.cube.pose.p)[:3]
    grasp_offset = np.array([-0.15, -0.07, 0.055], dtype=np.float32)
    if step_idx < 20:
        return obj + grasp_offset + np.array([0.0, 0.0, 0.11], dtype=np.float32), np.zeros(6, dtype=np.float32), "approach_high"
    if step_idx < 45:
        return obj + grasp_offset, np.zeros(6, dtype=np.float32), "approach_grasp"
    if step_idx < 65:
        return obj + grasp_offset, np.ones(6, dtype=np.float32), "close"
    return obj + grasp_offset + np.array([0.0, 0.0, 0.16], dtype=np.float32), np.ones(6, dtype=np.float32), "lift_hold"


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    import sim_maniskill.agents.jaka_rh56  # noqa: F401
    import sim_maniskill.tasks.pick_cube_jaka_rh56  # noqa: F401

    env = gym.make(
        "LiftCubeJakaRH56-v1",
        robot_uids="jaka_rh56",
        obs_mode="state_dict",
        control_mode="pd_joint_delta_pos",
        render_mode=None,
        render_backend="none",
        sim_backend="physx_cpu",
        num_envs=1,
        start_pose="pregrasp",
    )
    episode_summaries: list[dict[str, Any]] = []
    try:
        for episode_idx in range(args.episodes):
            env.reset(seed=args.seed + episode_idx)
            final_info: dict[str, Any] = {}
            max_height = 0.0
            first_success_step: int | None = None
            phase_counts: dict[str, int] = {}
            for step_idx in range(args.max_steps):
                target, hand_cmd, phase = _target_for_step(env, step_idx)
                phase_counts[phase] = phase_counts.get(phase, 0) + 1
                action, _ = _ik_action(env, target, hand_cmd)
                _, _, terminated, truncated, info = env.step(action)
                final_info = {
                    "success": _bool(info["success"]),
                    "is_lifted": _bool(info["is_lifted"]),
                    "is_grasped": _bool(info["is_grasped"]),
                    "is_robot_static": _bool(info["is_robot_static"]),
                    "object_height": float(_arr(info["object_height"])[0]),
                    "lift_success_height": float(_arr(info["lift_success_height"])[0]),
                }
                max_height = max(max_height, final_info["object_height"])
                if final_info["success"] and first_success_step is None:
                    first_success_step = step_idx
                done = _bool(terminated) or _bool(truncated)
                if done and final_info["success"]:
                    break
            episode_summaries.append(
                {
                    "episode_index": episode_idx,
                    "seed": args.seed + episode_idx,
                    "success": bool(final_info.get("success", False)),
                    "first_success_step": first_success_step,
                    "max_object_height": max_height,
                    "final_info": final_info,
                    "phase_counts": phase_counts,
                }
            )
    finally:
        env.close()

    success_count = sum(1 for item in episode_summaries if item["success"])
    summary = {
        "task": "LiftCubeJakaRH56-v1",
        "policy": "scripted_contact_only_lift_hold",
        "episodes": args.episodes,
        "success_count": success_count,
        "success_rate": success_count / max(args.episodes, 1),
        "episodes_detail": episode_summaries,
        "interpretation": (
            "This is a contact-only sim evaluation. Low success can indicate either policy weakness "
            "or uncalibrated RH56 contact simulation; do not tune real pseudo-tactile thresholds from it."
        ),
    }
    if args.out:
        out = Path(args.out).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate a contact-only JAKA+RH56 lift/hold task in ManiSkill.")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default="data/reports/jaka_rh56_lift_hold_eval/summary.json")
    args = parser.parse_args()
    print(json.dumps(evaluate(args), indent=2))


if __name__ == "__main__":
    main()
