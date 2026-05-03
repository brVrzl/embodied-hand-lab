from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
import sapien

from data_recorder.episode_recorder import DATASET_SCHEMA_VERSION, FAILURE_MODES, EpisodeRecorder
from embodiment_core.config import load_yaml
from rh56_driver.hand_schema import (
    CANONICAL_HAND_ORDER,
    DEFAULT_EE_ROTATION_DELTA_LIMIT_RAD,
    DEFAULT_EE_TRANSLATION_DELTA_LIMIT_M,
    DEFAULT_HAND_DELTA_LIMIT,
    RH56_INTERNAL_ORDER,
    apply_delta,
    canonical_to_raw,
    clip_ee_delta,
    compute_delta,
    moving_direction,
)
from sim_maniskill.recorder import (
    _squeeze_batch,
    _to_list,
    _to_numpy,
    extract_step_observation,
    extract_step_observation_from_env,
)
from sim_maniskill.teleop import _estimate_tcp_position_jacobian
from export_episode_videos import export_episode_videos

STRONG_SUCCESS_OBJECT_HEIGHT_M = 0.08


def _arr(value: Any) -> np.ndarray:
    return np.asarray(_squeeze_batch(_to_numpy(value)), dtype=np.float32).reshape(-1)


def _bool(value: Any) -> bool:
    return bool(np.asarray(_to_numpy(value)).reshape(-1)[0])


def _as_rgb_uint8(frame: Any) -> np.ndarray:
    array = np.asarray(frame)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3 or array.shape[2] < 3:
        raise ValueError(f"RGB frame must have shape HxWx3 or HxWx4, got {array.shape}")
    array = array[:, :, :3]
    if array.dtype == np.uint8:
        return np.ascontiguousarray(array)
    array = array.astype(np.float32, copy=False)
    if np.nanmax(array) <= 1.0:
        array = array * 255.0
    return np.ascontiguousarray(np.clip(array, 0.0, 255.0).astype(np.uint8))


def _save_rgb_png(frame: Any, path: Path) -> None:
    import imageio.v3 as iio

    path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(path, _as_rgb_uint8(frame))


def _current_rgbd_observation(env: Any, info: dict[str, Any]) -> Any:
    get_obs = getattr(env.unwrapped, "get_obs", None)
    if get_obs is None:
        get_obs = getattr(env, "get_obs", None)
    if get_obs is None:
        raise RuntimeError("ManiSkill env does not expose get_obs for post-step RGB refresh.")
    return get_obs(info)


def _ik_action(
    env: Any,
    target: np.ndarray,
    hand_action_internal: np.ndarray,
) -> tuple[np.ndarray, list[float], list[float], list[float]]:
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
    current_qpos = _arr(agent.robot.get_qpos())[:6]

    action = np.zeros(12, dtype=np.float32)
    action[:6] = np.clip(q_delta / 0.1, -1.0, 1.0)
    action[6:] = hand_action_internal
    return action, ee_delta, current_qpos.astype(np.float32).tolist(), (current_qpos + q_delta).astype(np.float32).tolist()


def _phase_target(env: Any, step_idx: int, failure_mode: str) -> tuple[np.ndarray, float, bool]:
    obj = _arr(env.unwrapped.cube.pose.p)[:3]
    goal = _arr(env.unwrapped.goal_site.pose.p)[:3]
    grasp_offset = np.array([-0.15, -0.07, 0.055], dtype=np.float32)
    carry = False
    if failure_mode == "fail_lateral_offset":
        grasp_offset = grasp_offset + np.array([0.0, 0.16, 0.0], dtype=np.float32)
    if failure_mode == "fail_early_close":
        if step_idx < 50:
            return obj + np.array([-0.15, -0.07, 0.18], dtype=np.float32), 1.0, carry
        return goal + np.array([-0.15, -0.07, 0.10], dtype=np.float32), 1.0, carry

    if step_idx < 15:
        return obj + grasp_offset + np.array([0.0, 0.0, 0.105], dtype=np.float32), -1.0, carry
    if step_idx < 35:
        return obj + grasp_offset, -1.0, carry
    if step_idx < 50:
        hand = -1.0 if failure_mode == "fail_late_close" else 1.0
        if failure_mode == "fail_low_grip":
            hand = -0.6
        return obj + grasp_offset, hand, carry
    if step_idx < 70:
        if failure_mode not in {"none", "fail_object_slip"}:
            return obj + np.array([-0.15, -0.07, 0.20], dtype=np.float32), -0.6 if failure_mode == "fail_low_grip" else 1.0, carry
        carry = True
        return obj + np.array([-0.15, -0.07, 0.20], dtype=np.float32), 1.0, carry
    if step_idx < 95:
        if failure_mode not in {"none", "fail_object_slip"}:
            hand = 1.0 if failure_mode == "fail_late_close" else -0.6 if failure_mode == "fail_low_grip" else 1.0
            return goal + np.array([-0.15, -0.07, 0.06], dtype=np.float32), hand, carry
        carry = True
        return goal + np.array([-0.15, -0.07, 0.06], dtype=np.float32), 1.0, carry
    return goal + np.array([-0.15, -0.07, 0.06], dtype=np.float32), -1.0, carry


def _set_cube_pose(env: Any, position: np.ndarray) -> None:
    env.unwrapped.cube.set_pose(sapien.Pose(p=position.astype(np.float32, copy=False)))


def _success_goal_position(env: Any) -> np.ndarray:
    goal = _arr(env.unwrapped.goal_site.pose.p)[:3].copy()
    goal[2] = max(float(goal[2]), STRONG_SUCCESS_OBJECT_HEIGHT_M + 0.005)
    return goal


def _extra_state(env: Any, info: dict[str, Any]) -> dict[str, Any]:
    unwrapped = env.unwrapped
    return {
        "is_grasped": _bool(info["is_grasped"]),
        "tcp_pose": _arr(unwrapped.agent.tcp_pose.raw_pose).tolist(),
        "goal_pos": _arr(unwrapped.goal_site.pose.p)[:3].tolist(),
        "obj_pose": _arr(unwrapped.cube.pose.raw_pose).tolist(),
        "tcp_to_obj_pos": (_arr(unwrapped.cube.pose.p)[:3] - _arr(unwrapped.agent.tcp_pose.p)[:3]).tolist(),
        "obj_to_goal_pos": (_arr(unwrapped.goal_site.pose.p)[:3] - _arr(unwrapped.cube.pose.p)[:3]).tolist(),
    }


def _hand_state_from_observation(step_obs: dict[str, Any]) -> list[float]:
    hand_states = step_obs.get("hand_states") or {}
    inspire6 = hand_states.get("inspire6") if isinstance(hand_states.get("inspire6"), dict) else {}
    values = inspire6.get("normalized_positions") or inspire6.get("positions") or hand_states.get("finger_positions") or [0.0] * 6
    return np.clip(np.asarray(values, dtype=np.float32).reshape(-1)[:6], 0.0, 1.0).tolist()


def _quality_from_height(success: bool, failure_mode: str, final_height: float) -> str:
    if not success or failure_mode != "none":
        return "intended_failure"
    if final_height >= STRONG_SUCCESS_OBJECT_HEIGHT_M:
        return "strong_success"
    if final_height >= 0.04:
        return "weak_success"
    return "near_failure"


def collect(args: argparse.Namespace) -> dict[str, str]:
    import sim_maniskill.agents.jaka_rh56  # noqa: F401
    import sim_maniskill.tasks.pick_cube_jaka_rh56  # noqa: F401

    obs_mode = "rgbd" if args.rgbd else "state_dict"
    env = gym.make(
        "PickCubeJakaRH56-v1",
        robot_uids="jaka_rh56",
        obs_mode=obs_mode,
        control_mode="pd_joint_delta_pos",
        render_mode=None,
        render_backend=args.render_backend,
        sim_backend="physx_cpu",
        num_envs=1,
        start_pose="pregrasp",
        sensor_camera_preset=args.review_camera_preset,
        sensor_camera_fov=args.review_camera_fov,
        sensor_configs={"width": args.image_size, "height": args.image_size} if args.rgbd else None,
    )
    recorder = EpisodeRecorder(load_yaml(args.logging_config), data_root=args.output_dir)
    success_count = 0
    try:
        for episode_idx in range(args.episodes):
            seed = args.seed + episode_idx
            obs, info = env.reset(seed=seed)
            episode_dir = recorder.start_episode(
                task_name="jaka_rh56_pickcube_privileged_oracle",
                instruction="pick the cube and place it at the goal region with jaka mini2 and rh56",
                operator="sim_jaka_rh56_privileged_oracle",
                metadata={
                    "schema_version": DATASET_SCHEMA_VERSION,
                    "control_hz": args.control_hz,
                    "dt": 1.0 / args.control_hz,
                    "embodiment": "jaka_mini2_rh56_single_arm",
                    "arm_dof": 6,
                    "hand_dof": 6,
                    "hand_type": "inspire_rh56",
                    "canonical_hand_order": list(CANONICAL_HAND_ORDER),
                    "ee_delta_frame": "base",
                    "ee_translation_delta_limit_type": "per_axis",
                    "ee_translation_delta_limit_m": DEFAULT_EE_TRANSLATION_DELTA_LIMIT_M,
                    "rotation_delta_type": "euler_xyz",
                    "action_delta_base": "command",
                    "hand_delta_cmd_clipped": True,
                    "hand_delta_state_clipped": True,
                    "hand_delta_state_raw_available": True,
                    "calibration_version": "rh56_default_open1000_close0_v1",
                    "privileged_observation": {
                        "object_pose": True,
                        "fields": ["observation.extra_observation.obj_pose", "observation.state.object_pose"],
                    },
                    "sim_env_id": "PickCubeJakaRH56-v1",
                    "sim_obs_mode": obs_mode,
                    "sim_control_mode": "pd_joint_delta_pos",
                    "camera_name": args.camera_name if args.save_rgb else None,
                    "review_camera_preset": args.review_camera_preset if args.save_rgb else None,
                    "review_camera_fov": args.review_camera_fov if args.save_rgb else None,
                    "review_image_size": args.image_size if args.save_rgb else None,
                    "rgb_frame_semantics": "post_step_after_oracle_carry_review" if args.save_rgb else None,
                    "policy": "privileged_kinematic_oracle",
                    "requested_failure_mode": args.failure_mode,
                    "seed": seed,
                    "limitations": (
                        "Cube pose is kinematically carried after the scripted close phase. "
                        "Use this dataset only to validate schema, RGB-D capture, and training code; "
                        "do not report it as physical grasp performance."
                    ),
                    "reset_info": _to_list(info),
                },
            )

            final_success = False
            last_hand_cmd = np.zeros(len(CANONICAL_HAND_ORDER), dtype=np.float32)
            for step_idx in range(args.max_steps):
                target, hand, carry = _phase_target(env, step_idx, args.failure_mode)
                target_hand_cmd = np.full(len(CANONICAL_HAND_ORDER), (hand + 1.0) * 0.5, dtype=np.float32)
                hand_delta = compute_delta(last_hand_cmd, target_hand_cmd, limit=args.hand_delta_limit)
                hand_cmd = apply_delta(last_hand_cmd, hand_delta, limit=args.hand_delta_limit)
                hand_cmd_internal = np.asarray(canonical_to_raw(hand_cmd, raw_order=RH56_INTERNAL_ORDER), dtype=np.float32)
                hand_action_internal = np.clip(hand_cmd_internal * 2.0 - 1.0, -1.0, 1.0)
                action, ee_delta, robot_q_current, robot_q_desired = _ik_action(env, target, hand_action_internal)
                if obs_mode == "rgbd":
                    step_obs = extract_step_observation(obs, camera_uid="base_camera", arm_joint_count=6, hand_joint_count=6)
                else:
                    step_obs = extract_step_observation_from_env(env, arm_joint_count=6, hand_joint_count=6)
                step_obs["extra_observation"] = _extra_state(env, info)
                hand_state = _hand_state_from_observation(step_obs)
                hand_delta_state_raw = (np.asarray(hand_cmd, dtype=np.float32) - np.asarray(hand_state, dtype=np.float32)).tolist()
                hand_delta_state = np.clip(
                    np.asarray(hand_delta_state_raw, dtype=np.float32),
                    -args.hand_delta_limit,
                    args.hand_delta_limit,
                ).tolist()
                step_obs["state"] = {
                    "ee_state": (
                        list((step_obs.get("arm_ee_pose") or {}).get("position") or [0.0, 0.0, 0.0])
                        + list((step_obs.get("arm_ee_pose") or {}).get("orientation_xyzw") or [0.0, 0.0, 0.0, 1.0])
                    ),
                    "robot_q_current": robot_q_current,
                    "hand_state": hand_state,
                    "hand_cmd_last": np.asarray(last_hand_cmd, dtype=np.float32).tolist(),
                    "hand_error": (np.asarray(last_hand_cmd, dtype=np.float32) - np.asarray(hand_state, dtype=np.float32)).tolist(),
                    "canonical_hand_order": list(CANONICAL_HAND_ORDER),
                    "object_pose": step_obs["extra_observation"]["obj_pose"],
                    "object_pose_is_privileged": True,
                }

                next_obs, reward, terminated, truncated, info = env.step(action)
                if carry:
                    goal = _success_goal_position(env) if args.failure_mode == "none" else _arr(env.unwrapped.goal_site.pose.p)[:3]
                    progress = min(1.0, max(0.0, (step_idx - 50) / 45.0))
                    carried = (1.0 - progress) * _arr(env.unwrapped.cube.pose.p)[:3] + progress * goal
                    if args.failure_mode == "fail_object_slip" and step_idx >= 78:
                        carried = _arr(env.unwrapped.cube.pose.p)[:3]
                        carried[2] = 0.025
                    if step_idx >= 94 and args.failure_mode != "fail_object_slip":
                        carried = goal
                    _set_cube_pose(env, carried)
                    info = env.unwrapped.evaluate()

                env_success = _bool(info["success"])
                object_height = float(_arr(env.unwrapped.cube.pose.p)[:3][2])
                privileged_success = (
                    args.failure_mode == "none"
                    and step_idx >= 94
                    and object_height >= STRONG_SUCCESS_OBJECT_HEIGHT_M
                )
                final_success = privileged_success
                if obs_mode == "rgbd" and (args.save_rgb or carry):
                    next_obs = _current_rgbd_observation(env, info)
                if args.save_rgb:
                    review_obs = extract_step_observation(
                        next_obs,
                        camera_uid="base_camera",
                        arm_joint_count=6,
                        hand_joint_count=6,
                    )
                    rgb = review_obs.get("rgb")
                    if rgb is None:
                        raise RuntimeError(
                            "RGB saving requested, but refreshed ManiSkill observation did not contain an RGB frame. "
                            "Check --rgbd, render_backend, and local renderer support."
                        )
                    rgb_path = episode_dir / "rgb" / args.camera_name / f"frame_{step_idx:06d}.png"
                    _save_rgb_png(rgb, rgb_path)
                    step_obs["rgb_path"] = str(rgb_path)
                    step_obs["rgb_paths"] = {args.camera_name: str(rgb_path)}
                    step_obs["rgb"] = None
                recorder.record_step(
                    observation=step_obs,
                    action={
                        "source": "maniskill",
                        "policy": "privileged_kinematic_oracle",
                        "step_index": step_idx,
                        "action": action.tolist(),
                        "ee_delta": ee_delta,
                        "hand_delta_cmd": hand_delta,
                        "hand_delta_state_raw": hand_delta_state_raw,
                        "hand_delta_state": hand_delta_state,
                        "hand_cmd": hand_cmd,
                        "last_hand_cmd": np.asarray(last_hand_cmd, dtype=np.float32).tolist(),
                        "hand_order": list(CANONICAL_HAND_ORDER),
                        "moving_direction": moving_direction(last_hand_cmd, hand_cmd),
                        "robot_q_current": robot_q_current,
                        "robot_q_desired": robot_q_desired,
                        "reward": float(np.asarray(_to_numpy(reward)).reshape(-1)[0]),
                        "terminated": _bool(terminated),
                        "truncated": _bool(truncated),
                        "success": final_success,
                        "info": {
                            **_to_list(info),
                            "env_success": env_success,
                            "privileged_success": privileged_success,
                            "object_height": object_height,
                            "strong_success_height_threshold": STRONG_SUCCESS_OBJECT_HEIGHT_M,
                        },
                    },
                )
                last_hand_cmd = np.asarray(hand_cmd, dtype=np.float32)
                obs = next_obs
                if final_success:
                    break

            success_count += int(final_success)
            final_object_height = float(_arr(env.unwrapped.cube.pose.p)[:3][2])
            final_failure_mode = "none" if final_success else ("unknown" if args.failure_mode == "none" else args.failure_mode)
            observed_quality = _quality_from_height(
                success=final_success,
                failure_mode=final_failure_mode,
                final_height=final_object_height,
            )
            recorder.mark_success(
                final_success,
                failure_mode=final_failure_mode,
                failure_reason="" if final_success else observed_quality,
                operator_notes=(
                    "privileged_kinematic_oracle; success requires final_object_height >= "
                    f"{STRONG_SUCCESS_OBJECT_HEIGHT_M:.3f}; final_object_height={final_object_height:.6f}; "
                    f"observed_quality={observed_quality}; not physical grasp performance"
                ),
            )
            recorder.stop_episode()
            time.sleep(0.001)

        export_dir = Path(args.export_dir).resolve()
        recorder.export_dataset(export_dir)
        video_dir = None
        if args.save_video:
            video_dir = Path(args.video_dir).resolve() if args.video_dir else (Path("data/replays") / export_dir.name).resolve()
            manual_review_out = (
                Path(args.manual_review_out).resolve()
                if args.manual_review_out
                else (Path("data/reports") / export_dir.name / "manual_review.yaml").resolve()
            )
            export_episode_videos(recorder.data_root, video_dir, fps=args.video_fps, manual_review_out=manual_review_out)
        return {
            "episodes_root": str(recorder.data_root),
            "export_dir": str(export_dir),
            "video_dir": str(video_dir) if video_dir is not None else "",
            "success_count": str(success_count),
        }
    finally:
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect privileged JAKA+RH56 PickCube demos for pipeline validation.")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--max-steps", type=int, default=120)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--rgbd", action="store_true")
    parser.add_argument("--save-rgb", action="store_true", help="Save camera RGB frames as PNG files inside each episode.")
    parser.add_argument("--image-size", type=int, default=128)
    parser.add_argument("--render-backend", default="none")
    parser.add_argument("--camera-name", default="third_person")
    parser.add_argument("--review-camera-preset", choices=["default", "close"], default="default")
    parser.add_argument("--review-camera-fov", type=float, default=None)
    parser.add_argument("--save-video", action="store_true", help="Export original RGB frames to per-episode MP4 videos after collection.")
    parser.add_argument("--video-dir", default=None)
    parser.add_argument("--video-fps", type=int, default=10)
    parser.add_argument("--manual-review-out", default=None)
    parser.add_argument("--hand-delta-limit", type=float, default=DEFAULT_HAND_DELTA_LIMIT)
    parser.add_argument("--control-hz", type=float, default=10.0)
    parser.add_argument(
        "--failure-mode",
        choices=sorted(FAILURE_MODES),
        default="unknown",
        help="Failure label to use if an episode does not reach privileged success. Successful episodes are always exported with failure_mode=none.",
    )
    parser.add_argument("--output-dir", default="data/episodes/jaka_rh56_pickcube_privileged_oracle")
    parser.add_argument("--export-dir", default="data/exports/structured/jaka_rh56_pickcube_privileged_oracle")
    parser.add_argument("--logging-config", default="configs/logging/default.yaml")
    args = parser.parse_args()
    if args.save_video:
        args.save_rgb = True
    if args.save_rgb:
        args.rgbd = True
    result = collect(args)
    print(f"Recorded episodes to: {result['episodes_root']}")
    print(f"Structured export written to: {result['export_dir']}")
    if result.get("video_dir"):
        print(f"Episode videos written to: {result['video_dir']}")
    print(f"Success episodes: {result['success_count']}/{args.episodes}")


if __name__ == "__main__":
    main()
