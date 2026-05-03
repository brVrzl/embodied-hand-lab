from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
import torch
from torch import nn

from sim_maniskill.recorder import extract_step_observation


def _feature_from_export_sample(sample: dict[str, Any]) -> np.ndarray:
    obs = sample["observation"]
    arm = obs["arm_joint_states"]
    hand = obs["hand_states"]
    ee = obs["arm_ee_pose"] or {}
    extra = obs.get("extra_observation") or {}
    action = sample["action"]
    step_idx = float(action.get("step_index", 0))
    return np.asarray(
        [step_idx / 160.0]
        + list(arm.get("positions", []))
        + list(arm.get("velocities", []))
        + list((ee.get("position") or [0.0, 0.0, 0.0]))
        + list((ee.get("orientation_xyzw") or [0.0, 0.0, 0.0, 1.0]))
        + list(hand.get("finger_positions", []))
        + list(hand.get("finger_currents", []))
        + [1.0 if extra.get("is_grasped") else 0.0]
        + list(extra.get("tcp_pose") or [0.0] * 7)
        + list(extra.get("goal_pos") or [0.0] * 3)
        + list(extra.get("obj_pose") or [0.0] * 7)
        + list(extra.get("tcp_to_obj_pos") or [0.0] * 3)
        + list(extra.get("obj_to_goal_pos") or [0.0] * 3),
        dtype=np.float32,
    )


def _feature_from_env_obs(obs: dict[str, Any], step_idx: int) -> np.ndarray:
    step_obs = extract_step_observation(obs, arm_joint_count=7, hand_joint_count=2)
    extra = obs["extra"]
    step_obs["extra_observation"] = {
        "is_grasped": bool(np.asarray(extra["is_grasped"].detach().cpu()).reshape(-1)[0]),
        "tcp_pose": np.asarray(extra["tcp_pose"].detach().cpu()).reshape(-1).astype(np.float32).tolist(),
        "goal_pos": np.asarray(extra["goal_pos"].detach().cpu()).reshape(-1).astype(np.float32).tolist(),
        "obj_pose": np.asarray(extra["obj_pose"].detach().cpu()).reshape(-1).astype(np.float32).tolist(),
        "tcp_to_obj_pos": np.asarray(extra["tcp_to_obj_pos"].detach().cpu()).reshape(-1).astype(np.float32).tolist(),
        "obj_to_goal_pos": np.asarray(extra["obj_to_goal_pos"].detach().cpu()).reshape(-1).astype(np.float32).tolist(),
    }
    sample = {
        "observation": {
            "arm_joint_states": step_obs["arm_joint_states"],
            "arm_ee_pose": step_obs["arm_ee_pose"],
            "hand_states": step_obs["hand_states"],
            "extra_observation": step_obs["extra_observation"],
        },
        "action": {"step_index": step_idx},
    }
    return _feature_from_export_sample(sample)


def _load_dataset(samples_path: Path) -> tuple[np.ndarray, np.ndarray, list[str]]:
    features: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    episode_ids: list[str] = []
    with samples_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            sample = json.loads(line)
            action = sample["action"].get("action")
            if action is None:
                continue
            features.append(_feature_from_export_sample(sample))
            actions.append(np.asarray(action, dtype=np.float32))
            episode_ids.append(sample["episode_id"])
    return np.stack(features), np.stack(actions), episode_ids


class Policy(nn.Module):
    def __init__(self, input_dim: int, output_dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim),
            nn.Tanh(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def _split_by_episode(episode_ids: list[str], train_fraction: float) -> tuple[np.ndarray, np.ndarray]:
    unique = sorted(set(episode_ids))
    train_count = max(1, int(len(unique) * train_fraction))
    train_eps = set(unique[:train_count])
    train_mask = np.asarray([eid in train_eps for eid in episode_ids], dtype=bool)
    return train_mask, ~train_mask


def _rollout(model: Policy, mean: np.ndarray, std: np.ndarray, *, episodes: int, seed: int, max_steps: int) -> int:
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
    successes = 0
    model.eval()
    try:
        for episode_idx in range(episodes):
            obs, _ = env.reset(seed=seed + episode_idx)
            final_success = False
            for step_idx in range(max_steps):
                feat = (_feature_from_env_obs(obs, step_idx) - mean) / std
                with torch.no_grad():
                    action = model(torch.from_numpy(feat).float().unsqueeze(0)).squeeze(0).numpy()
                obs, _, _, _, info = env.step(np.clip(action, -1.0, 1.0).astype(np.float32))
                final_success = bool(np.asarray(info["success"].detach().cpu()).reshape(-1)[0])
                if final_success:
                    break
            successes += int(final_success)
    finally:
        env.close()
    return successes


def train(args: argparse.Namespace) -> dict[str, float | str]:
    x, y, episode_ids = _load_dataset(Path(args.samples))
    train_mask, val_mask = _split_by_episode(episode_ids, args.train_fraction)
    mean = x[train_mask].mean(axis=0)
    std = x[train_mask].std(axis=0) + 1e-6
    x_norm = (x - mean) / std

    model = Policy(x.shape[1], y.shape[1])
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    loss_fn = nn.MSELoss()
    x_train = torch.from_numpy(x_norm[train_mask]).float()
    y_train = torch.from_numpy(y[train_mask]).float()
    x_val = torch.from_numpy(x_norm[val_mask]).float() if val_mask.any() else x_train
    y_val = torch.from_numpy(y[val_mask]).float() if val_mask.any() else y_train

    for _ in range(args.epochs):
        permutation = torch.randperm(x_train.shape[0])
        for start in range(0, x_train.shape[0], args.batch_size):
            idx = permutation[start : start + args.batch_size]
            pred = model(x_train[idx])
            loss = loss_fn(pred, y_train[idx])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

    model.eval()
    with torch.no_grad():
        train_mse = float(loss_fn(model(x_train), y_train).item())
        val_mse = float(loss_fn(model(x_val), y_val).item())
    rollout_successes = _rollout(
        model,
        mean,
        std,
        episodes=args.rollout_episodes,
        seed=args.rollout_seed,
        max_steps=args.max_steps,
    )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "mean": mean,
            "std": std,
            "input_dim": x.shape[1],
            "output_dim": y.shape[1],
            "train_mse": train_mse,
            "val_mse": val_mse,
            "rollout_successes": rollout_successes,
            "rollout_episodes": args.rollout_episodes,
        },
        output,
    )
    return {
        "model": str(output),
        "samples": str(args.samples),
        "train_mse": train_mse,
        "val_mse": val_mse,
        "rollout_success_rate": rollout_successes / args.rollout_episodes,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a minimal state-only BC baseline for official ManiSkill PickCube-v1.")
    parser.add_argument("--samples", default="data/exports/structured/maniskill_pickcube_oracle/samples.jsonl")
    parser.add_argument("--output", default="data/baselines/pickcube_state_bc.pt")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--train-fraction", type=float, default=0.8)
    parser.add_argument("--rollout-episodes", type=int, default=10)
    parser.add_argument("--rollout-seed", type=int, default=1000)
    parser.add_argument("--max-steps", type=int, default=160)
    args = parser.parse_args()
    result = train(args)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
