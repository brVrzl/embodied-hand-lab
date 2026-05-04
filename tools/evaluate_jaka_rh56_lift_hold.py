from __future__ import annotations

import argparse
import json
import time
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


def _lazy_imageio() -> Any:
    import imageio.v3 as iio

    return iio


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


def _save_diagnostic_frame(
    path: Path,
    *,
    step_idx: int,
    phase: str,
    object_pos: np.ndarray,
    tcp_pos: np.ndarray,
    target_pos: np.ndarray,
    final_info: dict[str, Any],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(8, 4), dpi=120)
    fig.suptitle(f"LiftCubeJakaRH56 step={step_idx} phase={phase}")

    axes[0].set_title("top view x-y")
    axes[0].scatter([object_pos[0]], [object_pos[1]], c="red", label="cube")
    axes[0].scatter([tcp_pos[0]], [tcp_pos[1]], c="blue", label="tcp")
    axes[0].scatter([target_pos[0]], [target_pos[1]], c="green", label="target")
    axes[0].set_xlim(-0.35, 0.15)
    axes[0].set_ylim(-0.25, 0.25)
    axes[0].set_xlabel("x m")
    axes[0].set_ylabel("y m")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)

    axes[1].set_title("side view x-z")
    axes[1].scatter([object_pos[0]], [object_pos[2]], c="red", label="cube")
    axes[1].scatter([tcp_pos[0]], [tcp_pos[2]], c="blue", label="tcp")
    axes[1].scatter([target_pos[0]], [target_pos[2]], c="green", label="target")
    axes[1].axhline(float(final_info.get("lift_success_height", 0.075)), color="orange", linestyle="--", linewidth=1)
    axes[1].set_xlim(-0.35, 0.15)
    axes[1].set_ylim(0.0, 0.30)
    axes[1].set_xlabel("x m")
    axes[1].set_ylabel("z m")
    axes[1].grid(True, alpha=0.3)
    status = (
        f"h={final_info.get('object_height', 0.0):.3f} "
        f"grasp={final_info.get('is_grasped', False)} "
        f"lift={final_info.get('is_lifted', False)}"
    )
    axes[1].text(0.02, 0.96, status, transform=axes[1].transAxes, va="top", fontsize=8)

    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _write_mp4_from_frames(frame_paths: list[Path], video_path: Path, *, fps: float) -> None:
    if not frame_paths:
        return
    iio = _lazy_imageio()
    video_path.parent.mkdir(parents=True, exist_ok=True)
    frames = [iio.imread(path) for path in frame_paths]
    iio.imwrite(video_path, frames, fps=max(float(fps), 1.0))


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    import sim_maniskill.agents.jaka_rh56  # noqa: F401
    import sim_maniskill.tasks.pick_cube_jaka_rh56  # noqa: F401

    make_kwargs = {
        "robot_uids": "jaka_rh56",
        "obs_mode": "state_dict",
        "control_mode": "pd_joint_delta_pos",
        "render_mode": "human" if args.viewer else None,
        "sim_backend": "physx_cpu",
        "num_envs": 1,
        "start_pose": "pregrasp",
    }
    if not args.viewer:
        make_kwargs["render_backend"] = "none"
    env = gym.make("LiftCubeJakaRH56-v1", **make_kwargs)
    episode_summaries: list[dict[str, Any]] = []
    frame_root = Path(args.save_frames).resolve() if args.save_frames else None
    video_root = Path(args.save_video).resolve() if args.save_video else None
    try:
        for episode_idx in range(args.episodes):
            env.reset(seed=args.seed + episode_idx)
            viewer = env.unwrapped.render_human() if args.viewer else None
            final_info: dict[str, Any] = {}
            max_height = 0.0
            first_success_step: int | None = None
            phase_counts: dict[str, int] = {}
            frame_paths: list[Path] = []
            for step_idx in range(args.max_steps):
                if viewer is not None and viewer.closed:
                    break
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
                if viewer is not None:
                    env.unwrapped.render_human()
                    if args.fps > 0:
                        time.sleep(1.0 / args.fps)
                if frame_root is not None and step_idx % max(args.frame_stride, 1) == 0:
                    object_pos = _arr(env.unwrapped.cube.pose.p)[:3]
                    tcp_pos = _arr(env.unwrapped.agent.tcp_pose.p)[:3]
                    frame_path = frame_root / f"episode_{episode_idx:03d}" / f"frame_{step_idx:06d}.png"
                    _save_diagnostic_frame(
                        frame_path,
                        step_idx=step_idx,
                        phase=phase,
                        object_pos=object_pos,
                        tcp_pos=tcp_pos,
                        target_pos=target,
                        final_info=final_info,
                    )
                    frame_paths.append(frame_path)
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
                    "diagnostic_frame_dir": str(frame_paths[0].parent) if frame_paths else "",
                    "diagnostic_video": str(video_root / f"episode_{episode_idx:03d}.mp4") if video_root is not None and frame_paths else "",
                }
            )
            if video_root is not None and frame_paths:
                _write_mp4_from_frames(frame_paths, video_root / f"episode_{episode_idx:03d}.mp4", fps=args.fps)
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
    parser.add_argument("--viewer", action="store_true", help="Open ManiSkill human viewer and play the scripted rollout.")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--save-frames", default="", help="Save headless diagnostic PNG frames to this directory.")
    parser.add_argument("--save-video", default="", help="Save an MP4 built from diagnostic frames to this path or directory.")
    parser.add_argument("--frame-stride", type=int, default=1)
    args = parser.parse_args()
    print(json.dumps(evaluate(args), indent=2))


if __name__ == "__main__":
    main()
