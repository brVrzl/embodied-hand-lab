from __future__ import annotations

import importlib
import sys
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

from data_recorder.episode_recorder import EpisodeRecorder
from embodiment_core.config import load_yaml
from embodiment_core.logger import get_logger
from rh56_driver.hand_schema import RH56_INTERNAL_ORDER, build_hand_state


def _to_numpy(value: Any) -> Any:
    if hasattr(value, "detach") and hasattr(value, "cpu") and hasattr(value, "numpy"):
        return value.detach().cpu().numpy()
    if isinstance(value, np.ndarray):
        return value
    if isinstance(value, Mapping):
        return {key: _to_numpy(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_numpy(item) for item in value]
    return value


def _squeeze_batch(value: Any) -> Any:
    if isinstance(value, np.ndarray) and value.ndim > 0 and value.shape[0] == 1:
        return value[0]
    return value


def _to_list(value: Any) -> Any:
    value = _to_numpy(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Mapping):
        return {key: _to_list(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_to_list(item) for item in value]
    return value


def _extract_success(info: Any) -> bool | None:
    info = _to_numpy(info)
    if not isinstance(info, Mapping) or "success" not in info:
        return None
    value = info["success"]
    if isinstance(value, np.ndarray):
        flat = value.reshape(-1)
        if flat.size == 0:
            return None
        return bool(flat[0])
    if isinstance(value, list):
        return bool(value[0]) if value else None
    return bool(value)


def _select_camera(sensor_data: Mapping[str, Any], preferred_uid: str | None) -> Mapping[str, Any] | None:
    if preferred_uid and preferred_uid in sensor_data:
        return sensor_data[preferred_uid]
    for camera_obs in sensor_data.values():
        if isinstance(camera_obs, Mapping) and ("rgb" in camera_obs or "depth" in camera_obs):
            return camera_obs
    return None


def extract_step_observation(
    obs: Any,
    *,
    camera_uid: str | None = None,
    arm_joint_count: int = 7,
    hand_joint_count: int = 2,
) -> dict[str, Any]:
    obs = _to_numpy(obs)
    if not isinstance(obs, Mapping):
        raise TypeError("ManiSkill observation must be a mapping.")

    rgb = None
    depth = None
    if "rgb" in obs:
        rgb = _squeeze_batch(np.asarray(obs["rgb"]))
    if "depth" in obs:
        depth = _squeeze_batch(np.asarray(obs["depth"]))
    if rgb is None or depth is None:
        sensor_data = obs.get("sensor_data", {})
        if isinstance(sensor_data, Mapping):
            camera_obs = _select_camera(sensor_data, preferred_uid=camera_uid)
            if camera_obs:
                if rgb is None and "rgb" in camera_obs:
                    rgb = _squeeze_batch(np.asarray(camera_obs["rgb"]))
                if depth is None and "depth" in camera_obs:
                    depth = _squeeze_batch(np.asarray(camera_obs["depth"]))

    if isinstance(depth, np.ndarray) and depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]

    agent = obs.get("agent", {})
    if not isinstance(agent, Mapping):
        agent = {}
    qpos = _squeeze_batch(np.asarray(agent.get("qpos", np.array([], dtype=np.float32)))).astype(
        np.float32,
        copy=False,
    )
    qvel = _squeeze_batch(np.asarray(agent.get("qvel", np.array([], dtype=np.float32)))).astype(
        np.float32,
        copy=False,
    )
    qpos = np.atleast_1d(qpos)
    qvel = np.atleast_1d(qvel)

    arm_count = arm_joint_count
    hand_count = hand_joint_count
    if qpos.size < arm_count:
        arm_count = qpos.size
        hand_count = 0
    elif arm_count + hand_count > qpos.size:
        hand_count = max(0, qpos.size - arm_count)

    arm_qpos = qpos[:arm_count]
    arm_qvel = qvel[:arm_count] if qvel.size >= arm_count else np.array([], dtype=np.float32)
    hand_qpos = qpos[arm_count : arm_count + hand_count] if hand_count > 0 else np.array([], dtype=np.float32)
    hand_qvel = qvel[arm_count : arm_count + hand_count] if qvel.size >= arm_count + hand_count else np.array([], dtype=np.float32)

    extra = obs.get("extra", {})
    if not isinstance(extra, Mapping):
        extra = {}
    tcp_pose = _squeeze_batch(np.asarray(extra.get("tcp_pose", np.array([], dtype=np.float32))))
    arm_ee_pose = None
    if tcp_pose.size == 7:
        arm_ee_pose = {
            "position": tcp_pose[:3].astype(np.float32, copy=False).tolist(),
            "orientation_xyzw": [
                float(tcp_pose[4]),
                float(tcp_pose[5]),
                float(tcp_pose[6]),
                float(tcp_pose[3]),
            ],
            "frame_id": "maniskill_world",
        }

    hand_currents = hand_qvel if hand_qvel.size == hand_qpos.size else np.zeros_like(hand_qpos)
    return {
        "rgb": rgb,
        "depth": depth,
        "camera_timestamp": time.time(),
        "arm_joint_states": {
            "names": [f"joint_{idx + 1}" for idx in range(arm_qpos.size)],
            "positions": arm_qpos.tolist(),
            "velocities": arm_qvel.tolist(),
            "efforts": [],
        },
        "arm_ee_pose": arm_ee_pose,
        "hand_states": build_hand_state(
            raw_positions=hand_qpos,
            raw_velocities=hand_qvel if hand_qvel.size == hand_qpos.size else np.zeros_like(hand_qpos),
            raw_currents=hand_currents,
            raw_forces=np.zeros_like(hand_qpos),
            raw_contact_binary=[False] * hand_qpos.size,
            raw_order=RH56_INTERNAL_ORDER if hand_qpos.size == len(RH56_INTERNAL_ORDER) else tuple(f"finger_{i}" for i in range(hand_qpos.size)),
            calibration=None,
            mode="sim_gripper",
        )
        if hand_qpos.size == len(RH56_INTERNAL_ORDER)
        else {
            "mode": "sim_gripper",
            "finger_positions": hand_qpos.tolist(),
            "finger_currents": hand_currents.tolist(),
            "contact_flags": [False] * hand_qpos.size,
            "force_estimate": [0.0] * hand_qpos.size,
        },
        "dog_states": None,
    }


def extract_step_observation_from_env(
    env: Any,
    *,
    arm_joint_count: int = 7,
    hand_joint_count: int = 2,
) -> dict[str, Any]:
    unwrapped = getattr(env, "unwrapped", env)
    agent = getattr(unwrapped, "agent", None)
    if agent is None:
        raise RuntimeError("State-mode ManiSkill extraction requires env.unwrapped.agent.")

    qpos = _squeeze_batch(np.asarray(_to_numpy(agent.robot.get_qpos()))).astype(
        np.float32,
        copy=False,
    )
    qvel = _squeeze_batch(np.asarray(_to_numpy(agent.robot.get_qvel()))).astype(
        np.float32,
        copy=False,
    )
    qpos = np.atleast_1d(qpos)
    qvel = np.atleast_1d(qvel)

    arm_count = arm_joint_count
    hand_count = hand_joint_count
    if qpos.size < arm_count:
        arm_count = qpos.size
        hand_count = 0
    elif arm_count + hand_count > qpos.size:
        hand_count = max(0, qpos.size - arm_count)

    arm_qpos = qpos[:arm_count]
    arm_qvel = qvel[:arm_count] if qvel.size >= arm_count else np.array([], dtype=np.float32)
    hand_qpos = qpos[arm_count : arm_count + hand_count] if hand_count > 0 else np.array([], dtype=np.float32)
    hand_qvel = qvel[arm_count : arm_count + hand_count] if qvel.size >= arm_count + hand_count else np.array([], dtype=np.float32)

    tcp_pose = _squeeze_batch(np.asarray(_to_numpy(agent.tcp_pose.raw_pose)))
    arm_ee_pose = None
    if tcp_pose.size == 7:
        arm_ee_pose = {
            "position": tcp_pose[:3].astype(np.float32, copy=False).tolist(),
            "orientation_xyzw": [
                float(tcp_pose[4]),
                float(tcp_pose[5]),
                float(tcp_pose[6]),
                float(tcp_pose[3]),
            ],
            "frame_id": "maniskill_world",
        }

    hand_currents = hand_qvel if hand_qvel.size == hand_qpos.size else np.zeros_like(hand_qpos)
    return {
        "rgb": None,
        "depth": None,
        "camera_timestamp": time.time(),
        "arm_joint_states": {
            "names": [f"joint_{idx + 1}" for idx in range(arm_qpos.size)],
            "positions": arm_qpos.tolist(),
            "velocities": arm_qvel.tolist(),
            "efforts": [],
        },
        "arm_ee_pose": arm_ee_pose,
        "hand_states": build_hand_state(
            raw_positions=hand_qpos,
            raw_velocities=hand_qvel if hand_qvel.size == hand_qpos.size else np.zeros_like(hand_qpos),
            raw_currents=hand_currents,
            raw_forces=np.zeros_like(hand_qpos),
            raw_contact_binary=[False] * hand_qpos.size,
            raw_order=RH56_INTERNAL_ORDER,
            calibration=None,
            mode="sim_gripper",
        )
        if hand_qpos.size == len(RH56_INTERNAL_ORDER)
        else {
            "mode": "sim_gripper",
            "finger_positions": hand_qpos.tolist(),
            "finger_currents": hand_currents.tolist(),
            "contact_flags": [False] * hand_qpos.size,
            "force_estimate": [0.0] * hand_qpos.size,
        },
        "dog_states": None,
    }


class ManiSkillRecordingRunner:
    def __init__(
        self,
        env: Any,
        recorder: EpisodeRecorder,
        config: dict[str, Any],
    ) -> None:
        self.env = env
        self.recorder = recorder
        self.config = config
        self.logger = get_logger("ManiSkillRecordingRunner")

    def collect(self) -> dict[str, str]:
        env_cfg = self.config.get("env", {})
        task_cfg = self.config.get("task", {})
        robot_cfg = self.config.get("robot", {})
        recording_cfg = self.config.get("recording", {})

        episodes = int(recording_cfg.get("episodes", 1))
        max_steps = int(recording_cfg.get("max_steps", 50))
        base_seed = int(recording_cfg.get("seed", 0))
        policy = str(recording_cfg.get("policy", "random"))
        task_name = str(task_cfg.get("task_name", "pick_and_place"))
        instruction = str(task_cfg.get("instruction", "pick the cube and place it at the goal"))
        operator = str(task_cfg.get("operator", "sim_maniskill"))
        camera_uid = env_cfg.get("camera_uid")
        arm_joint_count = int(robot_cfg.get("arm_joint_count", 7))
        hand_joint_count = int(robot_cfg.get("hand_joint_count", 2))

        for episode_idx in range(episodes):
            episode_seed = base_seed + episode_idx
            obs, reset_info = self.env.reset(seed=episode_seed)
            metadata = {
                "sim_env_id": env_cfg.get("env_id"),
                "sim_obs_mode": env_cfg.get("obs_mode"),
                "sim_control_mode": env_cfg.get("control_mode"),
                "policy": policy,
                "seed": episode_seed,
                "camera_uid": camera_uid,
                "reset_info": _to_list(reset_info),
                **dict(recording_cfg.get("dataset_metadata", {})),
            }
            self.recorder.start_episode(
                task_name=task_name,
                instruction=instruction,
                operator=operator,
                metadata=metadata,
            )

            final_success = False
            for step_idx in range(max_steps):
                action = self._select_action(policy=policy)
                if isinstance(_to_numpy(obs), Mapping):
                    step_obs = extract_step_observation(
                        obs,
                        camera_uid=camera_uid,
                        arm_joint_count=arm_joint_count,
                        hand_joint_count=hand_joint_count,
                    )
                else:
                    step_obs = extract_step_observation_from_env(
                        self.env,
                        arm_joint_count=arm_joint_count,
                        hand_joint_count=hand_joint_count,
                    )
                next_obs, reward, terminated, truncated, info = self.env.step(action)
                step_success = _extract_success(info)
                if step_success is not None:
                    final_success = step_success
                self.recorder.record_step(
                    observation=step_obs,
                    action={
                        "source": "maniskill",
                        "policy": policy,
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
                done = bool(np.asarray(_squeeze_batch(_to_numpy(terminated))).reshape(-1)[0]) or bool(
                    np.asarray(_squeeze_batch(_to_numpy(truncated))).reshape(-1)[0]
                )
                if done:
                    break

            self.recorder.mark_success(final_success, operator_notes=f"policy={policy}")
            self.recorder.stop_episode()
            self.logger.info(
                "Recorded ManiSkill episode %s/%s seed=%s success=%s",
                episode_idx + 1,
                episodes,
                episode_seed,
                final_success,
            )

        export_dir = Path(recording_cfg.get("export_dir", "data/exports/structured/maniskill")).resolve()
        self.recorder.export_dataset(export_dir)
        return {
            "episodes_root": str(self.recorder.data_root),
            "export_dir": str(export_dir),
        }

    def _select_action(self, policy: str) -> Any:
        if policy == "zero":
            return self._zero_action(self.env.action_space.sample())
        if policy == "random":
            return self.env.action_space.sample()
        raise ValueError(f"Unsupported policy: {policy}")

    def _zero_action(self, sample: Any) -> Any:
        sample = _to_numpy(sample)
        if isinstance(sample, np.ndarray):
            return np.zeros_like(sample)
        if isinstance(sample, Mapping):
            return {key: self._zero_action(value) for key, value in sample.items()}
        if isinstance(sample, list):
            return [self._zero_action(value) for value in sample]
        return 0


def _maybe_wrap_env(env: Any, flatten_rgbd_obs: bool) -> Any:
    if not flatten_rgbd_obs:
        return env
    wrappers_mod = importlib.import_module("mani_skill.utils.wrappers")
    wrapper_cls = getattr(wrappers_mod, "FlattenRGBDObservationWrapper")
    return wrapper_cls(env)


def create_env_from_config(config: dict[str, Any]) -> Any:
    env_cfg = config.get("env", {})
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    if sys.version_info >= (3, 13):
        raise RuntimeError(
            "ManiSkill integration is running under Python "
            f"{python_version}, but this environment is typically only usable with Python 3.10/3.11. "
            "Recreate .venv with /usr/bin/python3.10 or python3.11, reinstall dependencies, then retry."
        )
    try:
        gym = importlib.import_module("gymnasium")
        importlib.import_module("mani_skill.envs")
    except ImportError as exc:
        raise RuntimeError(
            "ManiSkill integration requires gymnasium and mani_skill. "
            f"Current Python is {python_version}. "
            "Install them in a compatible Python 3.10/3.11 environment first."
        ) from exc

    agent_module = env_cfg.get("agent_module")
    if agent_module:
        importlib.import_module(agent_module)
    env_module = env_cfg.get("env_module")
    if env_module:
        importlib.import_module(env_module)

    make_kwargs = dict(
        obs_mode=env_cfg.get("obs_mode", "rgbd"),
        control_mode=env_cfg.get("control_mode", "pd_ee_delta_pose"),
        render_mode=env_cfg.get("render_mode"),
        num_envs=int(env_cfg.get("num_envs", 1)),
        sensor_configs=env_cfg.get("sensor_configs"),
    )
    make_kwargs.update(env_cfg.get("env_kwargs", {}))
    if env_cfg.get("sim_backend") is not None:
        make_kwargs["sim_backend"] = env_cfg.get("sim_backend")
    if env_cfg.get("render_backend") is not None:
        make_kwargs["render_backend"] = env_cfg.get("render_backend")
    if env_cfg.get("robot_uids") is not None:
        make_kwargs["robot_uids"] = env_cfg.get("robot_uids")

    env = gym.make(env_cfg.get("env_id", "PickCube-v1"), **make_kwargs)
    return _maybe_wrap_env(env, flatten_rgbd_obs=bool(env_cfg.get("flatten_rgbd_obs", False)))


def run_from_config(config: dict[str, Any], overrides: dict[str, Any] | None = None) -> dict[str, str]:
    overrides = overrides or {}
    config = {
        "env": dict(config.get("env", {})),
        "task": dict(config.get("task", {})),
        "robot": dict(config.get("robot", {})),
        "recording": dict(config.get("recording", {})),
        "logging": dict(config.get("logging", {})),
    }
    if overrides.get("episodes") is not None:
        config["recording"]["episodes"] = overrides["episodes"]
    if overrides.get("max_steps") is not None:
        config["recording"]["max_steps"] = overrides["max_steps"]
    if overrides.get("seed") is not None:
        config["recording"]["seed"] = overrides["seed"]
    if overrides.get("policy") is not None:
        config["recording"]["policy"] = overrides["policy"]
    if overrides.get("output_dir") is not None:
        config["recording"]["output_dir"] = overrides["output_dir"]
    if overrides.get("export_dir") is not None:
        config["recording"]["export_dir"] = overrides["export_dir"]
    if overrides.get("env_id") is not None:
        config["env"]["env_id"] = overrides["env_id"]
    if overrides.get("task_name") is not None:
        config["task"]["task_name"] = overrides["task_name"]
    if overrides.get("instruction") is not None:
        config["task"]["instruction"] = overrides["instruction"]

    recorder = EpisodeRecorder(
        load_yaml(config["logging"].get("logging_config", "configs/logging/default.yaml")),
        data_root=config["recording"].get("output_dir"),
    )
    env = create_env_from_config(config)
    try:
        runner = ManiSkillRecordingRunner(env=env, recorder=recorder, config=config)
        return runner.collect()
    finally:
        close = getattr(env, "close", None)
        if callable(close):
            close()
