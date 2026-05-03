from __future__ import annotations

import argparse
import signal
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

from data_recorder.episode_recorder import EpisodeRecorder
from embodiment_core.config import load_yaml

from .recorder import (
    _extract_success,
    _squeeze_batch,
    _to_list,
    _to_numpy,
    create_env_from_config,
    extract_step_observation,
    extract_step_observation_from_env,
)


ARM_KEYMAP = {
    "u": (0, 1.0),
    "j": (0, -1.0),
    "i": (1, 1.0),
    "k": (1, -1.0),
    "o": (2, 1.0),
    "l": (2, -1.0),
    "7": (3, 1.0),
    "4": (3, -1.0),
    "8": (4, 1.0),
    "5": (4, -1.0),
    "9": (5, 1.0),
    "6": (5, -1.0),
    "q": (5, 1.0),
    "e": (5, -1.0),
}

EE_TRANSLATION_KEYMAP = {
    "w": np.array([1.0, 0.0, 0.0], dtype=np.float32),
    "s": np.array([-1.0, 0.0, 0.0], dtype=np.float32),
    "a": np.array([0.0, 1.0, 0.0], dtype=np.float32),
    "d": np.array([0.0, -1.0, 0.0], dtype=np.float32),
    "r": np.array([0.0, 0.0, 1.0], dtype=np.float32),
    "v": np.array([0.0, 0.0, -1.0], dtype=np.float32),
}


def _zero_action(action_space: Any) -> np.ndarray:
    sample = _to_numpy(action_space.sample())
    if not isinstance(sample, np.ndarray):
        raise TypeError(f"Teleop currently expects a flat numpy action space, got {type(sample)!r}.")
    return np.zeros_like(sample, dtype=np.float32)


def _arm_qpos(agent: Any) -> np.ndarray:
    qpos = _squeeze_batch(_to_numpy(agent.robot.get_qpos()))
    return np.asarray(qpos, dtype=np.float32)


def _tcp_position(agent: Any) -> np.ndarray:
    tcp_pos = _squeeze_batch(_to_numpy(agent.tcp_pose.p))
    return np.asarray(tcp_pos, dtype=np.float32)


def _estimate_tcp_position_jacobian(agent: Any, *, eps: float = 1e-3, arm_dof: int = 6) -> np.ndarray:
    """Numerically estimate d(tcp_xyz)/d(arm_qpos) for the current simulator state."""
    robot = agent.robot
    qpos0 = _to_numpy(robot.get_qpos()).copy()
    qpos0_2d = np.asarray(qpos0, dtype=np.float32)
    if qpos0_2d.ndim == 1:
        qpos0_2d = qpos0_2d[None, :]

    jac = np.zeros((3, arm_dof), dtype=np.float32)
    try:
        for joint_idx in range(arm_dof):
            qpos_plus = qpos0_2d.copy()
            qpos_minus = qpos0_2d.copy()
            qpos_plus[:, joint_idx] += eps
            qpos_minus[:, joint_idx] -= eps

            robot.set_qpos(qpos_plus)
            pos_plus = _tcp_position(agent)
            robot.set_qpos(qpos_minus)
            pos_minus = _tcp_position(agent)
            jac[:, joint_idx] = (pos_plus - pos_minus) / (2.0 * eps)
    finally:
        robot.set_qpos(qpos0)
    return jac


def _translation_delta_from_keys(viewer: Any, *, ee_step: float) -> np.ndarray:
    delta = np.zeros(3, dtype=np.float32)
    for key, direction in EE_TRANSLATION_KEYMAP.items():
        if viewer.window.key_down(key):
            delta += direction * ee_step
    return delta


def _apply_hand_keys(action: np.ndarray, viewer: Any, *, hand_scale: float) -> None:
    if viewer.window.key_down("g"):
        action[6:] = hand_scale
    if viewer.window.key_down("f"):
        action[6:] = -hand_scale
    if viewer.window.key_down(" "):
        action[:] = 0.0


def _joint_action_from_keys(
    viewer: Any,
    action_space: Any,
    *,
    arm_scale: float,
    wrist_scale: float,
    hand_scale: float,
) -> np.ndarray:
    action = _zero_action(action_space)
    for key, (idx, direction) in ARM_KEYMAP.items():
        if viewer.window.key_down(key):
            scale = wrist_scale if idx == 5 else arm_scale
            action[idx] = direction * scale

    _apply_hand_keys(action, viewer, hand_scale=hand_scale)
    return action


def _ee_action_from_keys(
    viewer: Any,
    action_space: Any,
    agent: Any,
    *,
    ee_step: float,
    wrist_scale: float,
    hand_scale: float,
    ik_damping: float,
    max_joint_delta: float,
    joint_delta_action_scale: float,
) -> np.ndarray:
    action = _zero_action(action_space)
    xyz_delta = _translation_delta_from_keys(viewer, ee_step=ee_step)
    if np.linalg.norm(xyz_delta) > 0:
        jac = _estimate_tcp_position_jacobian(agent)
        lhs = jac @ jac.T + (ik_damping**2) * np.eye(3, dtype=np.float32)
        q_delta = jac.T @ np.linalg.solve(lhs, xyz_delta)
        q_delta = np.clip(q_delta, -max_joint_delta, max_joint_delta)
        action[:6] = np.clip(q_delta / joint_delta_action_scale, -1.0, 1.0)

    if viewer.window.key_down("q") or viewer.window.key_down("9"):
        action[5] = wrist_scale
    if viewer.window.key_down("e") or viewer.window.key_down("6"):
        action[5] = -wrist_scale

    _apply_hand_keys(action, viewer, hand_scale=hand_scale)
    return action


def _step_observation(env: Any, obs: Any, config: dict[str, Any]) -> dict[str, Any]:
    env_cfg = config.get("env", {})
    robot_cfg = config.get("robot", {})
    arm_joint_count = int(robot_cfg.get("arm_joint_count", 6))
    hand_joint_count = int(robot_cfg.get("hand_joint_count", 6))
    if isinstance(_to_numpy(obs), Mapping):
        return extract_step_observation(
            obs,
            camera_uid=env_cfg.get("camera_uid"),
            arm_joint_count=arm_joint_count,
            hand_joint_count=hand_joint_count,
        )
    return extract_step_observation_from_env(
        env,
        arm_joint_count=arm_joint_count,
        hand_joint_count=hand_joint_count,
    )


def run_teleop(
    config: dict[str, Any],
    *,
    control_space: str,
    arm_scale: float,
    wrist_scale: float,
    hand_scale: float,
    ee_step: float,
    ik_damping: float,
    max_joint_delta: float,
    joint_delta_action_scale: float,
    fps: float,
    max_steps_override: int | None = None,
) -> dict[str, str]:
    config = {
        "env": dict(config.get("env", {})),
        "task": dict(config.get("task", {})),
        "robot": dict(config.get("robot", {})),
        "recording": dict(config.get("recording", {})),
        "logging": dict(config.get("logging", {})),
    }
    config["env"]["render_mode"] = "human"

    env = create_env_from_config(config)
    recorder = EpisodeRecorder(
        load_yaml(config.get("logging", {}).get("logging_config", "configs/logging/default.yaml")),
        data_root=config["recording"].get("output_dir", "data/episodes/maniskill_teleop"),
    )
    try:
        obs, reset_info = env.reset(seed=int(config.get("recording", {}).get("seed", 0)))
        unwrapped = getattr(env, "unwrapped", env)
        viewer = unwrapped.render_human()
        configured_max_steps = int(config.get("recording", {}).get("max_steps", 0))
        max_steps = configured_max_steps if max_steps_override is None else max_steps_override
        sleep_s = 0.0 if fps <= 0 else 1.0 / fps
        final_success = False

        recorder.start_episode(
            task_name=config.get("task", {}).get("task_name", "pick_and_place"),
            instruction=config.get("task", {}).get("instruction", "pick the cube and place it at the goal region"),
            operator=config.get("task", {}).get("operator", "sim_maniskill_teleop"),
            metadata={
                "sim_env_id": config.get("env", {}).get("env_id"),
                "sim_obs_mode": config.get("env", {}).get("obs_mode"),
                "sim_control_mode": config.get("env", {}).get("control_mode"),
                "policy": "keyboard_teleop",
                "control_space": control_space,
                "reset_info": _to_list(reset_info),
                "controls": {
                    "ee": "w/s moves world x, a/d moves world y, r/v moves z",
                    "arm": "joint mode only: u/j i/k o/l 7/4 8/5 nudge joints 1-5",
                    "wrist": "q/e or 9/6 rolls joint 6",
                    "hand": "g closes all controlled fingers, f opens them",
                    "finish": "p marks success and exits, x marks failure and exits",
                    "max_steps": "0 means no automatic timeout",
                },
            },
        )

        print("Teleop controls:")
        print(f"  control space: {control_space}")
        if control_space == "ee":
            print("  tcp translation: w/s world x, a/d world y, r/v up/down")
        else:
            print("  arm joints 1-5: u/j i/k o/l 7/4 8/5")
        print("  wrist roll joint 6: q/e (also 9/6)")
        print("  hand: g close, f open")
        print("  finish: p success, x failure, close window/Ctrl+C failure")
        print("  timeout: disabled" if max_steps <= 0 else f"  timeout: {max_steps} steps")

        step_idx = 0
        while max_steps <= 0 or step_idx < max_steps:
            if viewer.closed:
                break
            if viewer.window.key_down("p"):
                final_success = True
                break
            if viewer.window.key_down("x"):
                final_success = False
                break

            if control_space == "ee":
                action = _ee_action_from_keys(
                    viewer,
                    env.action_space,
                    unwrapped.agent,
                    ee_step=ee_step,
                    wrist_scale=wrist_scale,
                    hand_scale=hand_scale,
                    ik_damping=ik_damping,
                    max_joint_delta=max_joint_delta,
                    joint_delta_action_scale=joint_delta_action_scale,
                )
            else:
                action = _joint_action_from_keys(
                    viewer,
                    env.action_space,
                    arm_scale=arm_scale,
                    wrist_scale=wrist_scale,
                    hand_scale=hand_scale,
                )
            step_obs = _step_observation(env, obs, config)
            next_obs, reward, terminated, truncated, info = env.step(action)
            step_success = _extract_success(info)
            if step_success is not None:
                final_success = step_success
            recorder.record_step(
                observation=step_obs,
                action={
                    "source": "keyboard_teleop",
                    "step_index": step_idx,
                    "action": _to_list(action),
                    "reward": float(np.asarray(_squeeze_batch(_to_numpy(reward))).reshape(-1)[0]),
                    "terminated": bool(np.asarray(_squeeze_batch(_to_numpy(terminated))).reshape(-1)[0]),
                    "truncated": bool(np.asarray(_squeeze_batch(_to_numpy(truncated))).reshape(-1)[0]),
                    "success": step_success,
                    "info": _to_list(info),
                },
            )
            obs = next_obs
            unwrapped.render_human()
            if sleep_s > 0:
                time.sleep(sleep_s)
            step_idx += 1

        recorder.mark_success(final_success, operator_notes="keyboard_teleop")
        episode_dir = recorder.stop_episode()
        export_dir = recorder.export_dataset(
            config.get("recording", {}).get("export_dir", "data/exports/structured/maniskill_teleop")
        )
        return {
            "episode_dir": str(episode_dir),
            "export_dir": str(export_dir),
            "success": str(final_success),
        }
    finally:
        env.close()


def main() -> None:
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    parser = argparse.ArgumentParser(description="Keyboard teleop for ManiSkill JAKA+RH56 tasks.")
    parser.add_argument("--config", default="configs/sim/maniskill_jaka_rh56_pick_cube.yaml")
    parser.add_argument(
        "--control-space",
        choices=("ee", "joint"),
        default="ee",
        help="Use ee for task-space TCP translation, or joint for raw joint nudging.",
    )
    parser.add_argument("--arm-scale", type=float, default=0.2)
    parser.add_argument("--wrist-scale", type=float, default=0.8)
    parser.add_argument("--hand-scale", type=float, default=0.45)
    parser.add_argument("--ee-step", type=float, default=0.008, help="TCP translation step per frame in meters.")
    parser.add_argument("--ik-damping", type=float, default=0.05, help="Damping for numerical Jacobian IK.")
    parser.add_argument("--max-joint-delta", type=float, default=0.04, help="Max IK joint delta per frame in radians.")
    parser.add_argument(
        "--joint-delta-action-scale",
        type=float,
        default=0.1,
        help="Controller joint delta represented by normalized action=1.",
    )
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Override recording.max_steps. Use 0 for no automatic timeout.",
    )
    args = parser.parse_args()

    result = run_teleop(
        load_yaml(Path(args.config)),
        control_space=args.control_space,
        arm_scale=args.arm_scale,
        wrist_scale=args.wrist_scale,
        hand_scale=args.hand_scale,
        ee_step=args.ee_step,
        ik_damping=args.ik_damping,
        max_joint_delta=args.max_joint_delta,
        joint_delta_action_scale=args.joint_delta_action_scale,
        fps=args.fps,
        max_steps_override=args.max_steps,
    )
    print(f"Episode written to: {result['episode_dir']}")
    print(f"Structured export written to: {result['export_dir']}")
    print(f"Success: {result['success']}")


if __name__ == "__main__":
    main()
