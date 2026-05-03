from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from data_recorder.episode_recorder import EpisodeRecorder
from embodiment_core.config import load_yaml
from sim_maniskill.recorder import ManiSkillRecordingRunner, extract_step_observation


def test_extract_step_observation_nested_rgbd() -> None:
    obs = {
        "agent": {
            "qpos": np.array([[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.04, 0.05]], dtype=np.float32),
            "qvel": np.array([[0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.0, 0.0]], dtype=np.float32),
        },
        "extra": {
            "tcp_pose": np.array([[0.3, 0.1, 0.2, 1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
        },
        "sensor_data": {
            "base_camera": {
                "rgb": np.zeros((1, 8, 8, 3), dtype=np.uint8),
                "depth": np.ones((1, 8, 8, 1), dtype=np.uint16),
            }
        },
    }

    step_obs = extract_step_observation(obs, camera_uid="base_camera", arm_joint_count=7, hand_joint_count=2)

    assert step_obs["rgb"].shape == (8, 8, 3)
    assert step_obs["depth"].shape == (8, 8)
    assert np.allclose(step_obs["arm_joint_states"]["positions"], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    assert np.allclose(step_obs["hand_states"]["finger_positions"], [0.04, 0.05])
    assert np.allclose(step_obs["arm_ee_pose"]["orientation_xyzw"], [0.0, 0.0, 0.0, 1.0])


def test_extract_step_observation_flattened_rgbd() -> None:
    obs = {
        "rgb": np.zeros((1, 4, 5, 3), dtype=np.uint8),
        "depth": np.full((1, 4, 5, 1), 7, dtype=np.uint16),
        "agent": {
            "qpos": np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
            "qvel": np.array([[0.1, 0.2, 0.3]], dtype=np.float32),
        },
    }

    step_obs = extract_step_observation(obs, arm_joint_count=3, hand_joint_count=0)

    assert step_obs["rgb"].shape == (4, 5, 3)
    assert step_obs["depth"].shape == (4, 5)
    assert np.allclose(step_obs["arm_joint_states"]["positions"], [1.0, 2.0, 3.0])
    assert step_obs["hand_states"]["finger_positions"] == []


def test_extract_step_observation_rh56_hand_schema() -> None:
    obs = {
        "agent": {
            "qpos": np.array([[0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 10, 20, 30, 40, 50, 60]], dtype=np.float32),
            "qvel": np.array([[0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6]], dtype=np.float32),
        },
    }

    step_obs = extract_step_observation(obs, arm_joint_count=6, hand_joint_count=6)

    hand = step_obs["hand_states"]
    assert hand["schema_version"] == "inspire6_v1"
    assert hand["finger_positions"] == [10, 20, 30, 40, 50, 60]
    assert hand["inspire6"]["positions"] == [30, 40, 50, 60, 10, 20]
    assert hand["inspire6"]["normalized_positions"] is None


class _FakeActionSpace:
    def sample(self) -> np.ndarray:
        return np.array([0.25, -0.25, 0.1], dtype=np.float32)


class _FakeEnv:
    def __init__(self) -> None:
        self.action_space = _FakeActionSpace()
        self._step = 0

    def reset(self, seed: int | None = None) -> tuple[dict, dict]:
        self._step = 0
        return self._obs(offset=0.0), {"seed": seed}

    def step(self, action: np.ndarray) -> tuple[dict, float, bool, bool, dict]:
        self._step += 1
        terminated = self._step >= 2
        return self._obs(offset=0.1 * self._step), 1.0, terminated, False, {"success": terminated}

    def close(self) -> None:
        return None

    def _obs(self, offset: float) -> dict:
        return {
            "agent": {
                "qpos": np.array([[0.0 + offset, 0.1 + offset, 0.2 + offset]], dtype=np.float32),
                "qvel": np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
            },
            "extra": {
                "tcp_pose": np.array([[0.4, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            },
            "sensor_data": {
                "base_camera": {
                    "rgb": np.full((1, 6, 6, 3), fill_value=127, dtype=np.uint8),
                    "depth": np.full((1, 6, 6, 1), fill_value=100, dtype=np.uint16),
                }
            },
        }


def test_maniskill_runner_records_episode(tmp_path: Path) -> None:
    recorder = EpisodeRecorder(load_yaml("configs/logging/default.yaml"), data_root=tmp_path / "episodes")
    config = {
        "env": {
            "env_id": "FakePickCube-v1",
            "obs_mode": "rgbd",
            "control_mode": "pd_ee_delta_pose",
            "camera_uid": "base_camera",
        },
        "task": {
            "task_name": "pick_and_place",
            "instruction": "pick the cube and place it at the goal region",
            "operator": "sim_test",
        },
        "robot": {
            "arm_joint_count": 3,
            "hand_joint_count": 0,
        },
        "recording": {
            "episodes": 1,
            "max_steps": 5,
            "seed": 11,
            "policy": "zero",
            "export_dir": str(tmp_path / "exports" / "structured"),
        },
    }

    runner = ManiSkillRecordingRunner(env=_FakeEnv(), recorder=recorder, config=config)
    result = runner.collect()

    episodes_root = Path(result["episodes_root"])
    episode_dirs = sorted(episodes_root.glob("episode_*"))
    assert len(episode_dirs) == 1
    metadata = json.loads((episode_dirs[0] / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["success"] is True
    assert metadata["extra_metadata"]["sim_env_id"] == "FakePickCube-v1"

    samples_path = Path(result["export_dir"]) / "samples.jsonl"
    assert samples_path.exists()
    samples = samples_path.read_text(encoding="utf-8").splitlines()
    assert len(samples) == 2
