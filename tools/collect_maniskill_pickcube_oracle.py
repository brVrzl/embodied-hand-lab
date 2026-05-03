from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np

from data_recorder.episode_recorder import EpisodeRecorder
from embodiment_core.config import load_yaml
from sim_maniskill.recorder import _squeeze_batch, _to_list, _to_numpy, extract_step_observation


def _first_numpy(value: Any) -> np.ndarray:
    value = _squeeze_batch(_to_numpy(value))
    return np.asarray(value, dtype=np.float32)


def _bool_info(info: dict[str, Any], key: str) -> bool:
    value = _to_numpy(info[key])
    return bool(np.asarray(value).reshape(-1)[0])


def _oracle_action(obs: dict[str, Any], step_idx: int) -> np.ndarray:
    extra = obs["extra"]
    tcp = _first_numpy(extra["tcp_pose"])[:3]
    obj = _first_numpy(extra["obj_pose"])[:3]
    goal = _first_numpy(extra["goal_pos"])

    if step_idx < 20:
        target = obj + np.array([0.0, 0.0, 0.08], dtype=np.float32)
        gripper = 1.0
    elif step_idx < 50:
        target = obj + np.array([0.0, 0.0, 0.005], dtype=np.float32)
        gripper = 1.0
    elif step_idx < 80:
        target = obj + np.array([0.0, 0.0, 0.005], dtype=np.float32)
        gripper = -1.0
    elif step_idx < 115:
        target = np.array([obj[0], obj[1], max(float(goal[2]) + 0.02, 0.22)], dtype=np.float32)
        gripper = -1.0
    else:
        target = goal
        gripper = -1.0

    action = np.zeros(7, dtype=np.float32)
    action[:3] = np.clip((target - tcp) * 8.0, -1.0, 1.0)
    action[6] = gripper
    return action


def _task_state(obs: dict[str, Any]) -> dict[str, Any]:
    extra = obs["extra"]
    return {
        "is_grasped": bool(np.asarray(_to_numpy(extra["is_grasped"])).reshape(-1)[0]),
        "tcp_pose": _first_numpy(extra["tcp_pose"]).tolist(),
        "goal_pos": _first_numpy(extra["goal_pos"]).tolist(),
        "obj_pose": _first_numpy(extra["obj_pose"]).tolist(),
        "tcp_to_obj_pos": _first_numpy(extra["tcp_to_obj_pos"]).tolist(),
        "obj_to_goal_pos": _first_numpy(extra["obj_to_goal_pos"]).tolist(),
    }


def collect(args: argparse.Namespace) -> dict[str, str]:
    import mani_skill.envs  # noqa: F401

    env = gym.make(
        "PickCube-v1",
        obs_mode="state_dict",
        control_mode="pd_ee_delta_pose",
        render_mode=None,
        render_backend="none",
        sim_backend="physx_cpu",
        num_envs=1,
    )
    recorder = EpisodeRecorder(
        load_yaml(args.logging_config),
        data_root=args.output_dir,
    )
    try:
        success_count = 0
        for episode_idx in range(args.episodes):
            seed = args.seed + episode_idx
            obs, reset_info = env.reset(seed=seed)
            recorder.start_episode(
                task_name="maniskill_pickcube_oracle",
                instruction="pick the cube and place it at the goal region",
                operator="scripted_pickcube_oracle",
                metadata={
                    "sim_env_id": "PickCube-v1",
                    "sim_obs_mode": "state_dict",
                    "sim_control_mode": "pd_ee_delta_pose",
                    "policy": "scripted_oracle",
                    "seed": seed,
                    "reset_info": _to_list(reset_info),
                },
            )

            final_success = False
            for step_idx in range(args.max_steps):
                action = _oracle_action(obs, step_idx)
                step_obs = extract_step_observation(obs, arm_joint_count=7, hand_joint_count=2)
                step_obs["extra_observation"] = _task_state(obs)
                next_obs, reward, terminated, truncated, info = env.step(action)
                final_success = _bool_info(info, "success")
                recorder.record_step(
                    observation=step_obs,
                    action={
                        "source": "maniskill",
                        "policy": "scripted_oracle",
                        "step_index": step_idx,
                        "action": action.tolist(),
                        "reward": float(np.asarray(_to_numpy(reward)).reshape(-1)[0]),
                        "terminated": _bool_info({"x": terminated}, "x"),
                        "truncated": _bool_info({"x": truncated}, "x"),
                        "success": final_success,
                        "info": _to_list(info),
                    },
                )
                obs = next_obs
                if final_success:
                    break

            success_count += int(final_success)
            recorder.mark_success(final_success, operator_notes="scripted_pickcube_oracle")
            recorder.stop_episode()

        export_dir = Path(args.export_dir).resolve()
        recorder.export_dataset(export_dir)
        return {
            "episodes_root": str(recorder.data_root),
            "export_dir": str(export_dir),
            "success_count": str(success_count),
        }
    finally:
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect successful scripted demos on official ManiSkill PickCube-v1.")
    parser.add_argument("--episodes", type=int, default=40)
    parser.add_argument("--max-steps", type=int, default=160)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output-dir", default="data/episodes/maniskill_pickcube_oracle")
    parser.add_argument("--export-dir", default="data/exports/structured/maniskill_pickcube_oracle")
    parser.add_argument("--logging-config", default="configs/logging/default.yaml")
    args = parser.parse_args()
    result = collect(args)
    print(f"Recorded episodes to: {result['episodes_root']}")
    print(f"Structured export written to: {result['export_dir']}")
    print(f"Success episodes: {result['success_count']}/{args.episodes}")


if __name__ == "__main__":
    main()
