from __future__ import annotations

import json
import shutil
import time
from pathlib import Path
from typing import Any

import numpy as np

from embodiment_core.logger import get_logger
from rh56_driver.hand_schema import (
    CANONICAL_HAND_ORDER,
    DEFAULT_EE_ROTATION_DELTA_LIMIT_RAD,
    DEFAULT_EE_TRANSLATION_DELTA_LIMIT_M,
    DEFAULT_HAND_DELTA_LIMIT,
)

DATASET_SCHEMA_VERSION = "jaka_rh56_pickcube_v0.2"
SUPPORTED_SCHEMA_VERSIONS = {DATASET_SCHEMA_VERSION}

FAILURE_MODES = {
    "none",
    "fail_late_close",
    "fail_early_close",
    "fail_lateral_offset",
    "fail_low_grip",
    "fail_object_slip",
    "fail_collision",
    "fail_timeout",
    "unknown",
}

DEFAULT_SCHEMA_METADATA = {
    "schema_version": DATASET_SCHEMA_VERSION,
    "control_hz": 10.0,
    "dt": 0.1,
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
    "calibration": None,
    "privileged_observation": {
        "object_pose": True,
        "fields": ["observation.extra_observation.obj_pose", "observation.state.object_pose"],
    },
    "limits": {
        "hand_delta_cmd": DEFAULT_HAND_DELTA_LIMIT,
        "ee_rotation_delta_rad": DEFAULT_EE_ROTATION_DELTA_LIMIT_RAD,
    },
}


class EpisodeRecorder:
    def __init__(self, config: dict, data_root: str | Path | None = None) -> None:
        self.config = config
        self.data_root = Path(data_root or config.get("data_root", "data/episodes")).resolve()
        self.data_root.mkdir(parents=True, exist_ok=True)
        self.logger = get_logger("EpisodeRecorder")
        self._current_episode_dir: Path | None = None
        self._metadata: dict[str, Any] | None = None
        self._steps: list[dict[str, Any]] = []

    def start_episode(
        self,
        task_name: str,
        instruction: str,
        operator: str = "unknown",
        metadata: dict[str, Any] | None = None,
    ) -> Path:
        if self._current_episode_dir is not None:
            raise RuntimeError("An episode is already active.")
        episode_id = time.strftime("%Y%m%d_%H%M%S")
        episode_dir = self.data_root / f"episode_{episode_id}"
        suffix = 0
        while episode_dir.exists():
            suffix += 1
            episode_dir = self.data_root / f"episode_{episode_id}_{suffix}"
        (episode_dir / "rgb").mkdir(parents=True, exist_ok=False)
        (episode_dir / "depth").mkdir(parents=True, exist_ok=False)
        self._current_episode_dir = episode_dir
        self._steps = []
        schema_metadata = self._schema_metadata(metadata or {})
        self._metadata = {
            "episode_id": episode_dir.name,
            "task_name": task_name,
            "natural_language_instruction": instruction,
            "operator": operator,
            "start_time": time.time(),
            "end_time": None,
            "success": None,
            "failure_mode": "",
            "failure_reason": "",
            "operator_notes": "",
            "extra_metadata": metadata or {},
            **schema_metadata,
        }
        self._write_metadata()
        self.logger.info("Started episode at %s", episode_dir)
        return episode_dir

    def record_step(
        self,
        timestamp: float | None = None,
        observation: dict[str, Any] | None = None,
        action: dict[str, Any] | None = None,
        operator_notes: str = "",
    ) -> dict[str, Any]:
        if self._current_episode_dir is None or self._metadata is None:
            raise RuntimeError("No active episode.")
        observation = observation or {}
        timestamp = timestamp or time.time()
        rgb_path = observation.get("rgb_path") or self._save_array("rgb", timestamp, observation.get("rgb"))
        depth_path = observation.get("depth_path") or self._save_array("depth", timestamp, observation.get("depth"))
        step = {
            "timestamp": timestamp,
            "frame_index": len(self._steps),
            "task_name": self._metadata["task_name"],
            "natural_language_instruction": self._metadata["natural_language_instruction"],
            "rgb_path": rgb_path,
            "rgb_paths": observation.get("rgb_paths"),
            "depth_path": depth_path,
            "depth_paths": observation.get("depth_paths"),
            "camera_timestamp": observation.get("camera_timestamp"),
            "arm_joint_states": observation.get("arm_joint_states"),
            "arm_ee_pose": observation.get("arm_ee_pose"),
            "hand_states": observation.get("hand_states"),
            "state": observation.get("state"),
            "dog_states": observation.get("dog_states"),
            "extra_observation": observation.get("extra_observation"),
            "action": action or {},
            "success": None,
            "operator_notes": operator_notes,
        }
        self._steps.append(step)
        self._append_jsonl(self._current_episode_dir / "steps.jsonl", step)
        return step

    def mark_success(
        self,
        success: bool,
        failure_reason: str = "",
        operator_notes: str = "",
        failure_mode: str | None = None,
    ) -> None:
        if self._metadata is None:
            raise RuntimeError("No active episode.")
        self._metadata["success"] = success
        resolved_failure_mode = self._normalize_failure_mode(success=success, failure_mode=failure_mode, failure_reason=failure_reason)
        self._metadata["failure_mode"] = resolved_failure_mode
        self._metadata["failure_reason"] = failure_reason
        self._metadata["operator_notes"] = operator_notes
        self._write_metadata()

    def stop_episode(self, success: bool | None = None, operator_notes: str = "") -> Path:
        if self._current_episode_dir is None or self._metadata is None:
            raise RuntimeError("No active episode.")
        if success is not None:
            self._metadata["success"] = success
        self._metadata["operator_notes"] = operator_notes or self._metadata.get("operator_notes", "")
        self._metadata["end_time"] = time.time()
        self._metadata["duration_sec"] = self._metadata["end_time"] - self._metadata["start_time"]
        self._write_metadata()
        episode_dir = self._current_episode_dir
        self.logger.info("Stopped episode at %s", episode_dir)
        self._current_episode_dir = None
        self._metadata = None
        self._steps = []
        return episode_dir

    def export_dataset(self, export_dir: str | Path) -> Path:
        export_dir = Path(export_dir).resolve()
        export_dir.mkdir(parents=True, exist_ok=True)
        samples_path = export_dir / "samples.jsonl"
        manifest = {"episodes": []}
        if samples_path.exists():
            samples_path.unlink()
        for episode_index, episode_dir in enumerate(sorted(self.data_root.glob("episode_*"))):
            metadata = json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))
            export_metadata = self._schema_metadata(metadata)
            failure_mode = self._normalize_failure_mode(
                success=bool(metadata.get("success")),
                failure_mode=metadata.get("failure_mode"),
                failure_reason=metadata.get("failure_reason", ""),
            )
            steps_path = episode_dir / "steps.jsonl"
            if not steps_path.exists():
                continue
            step_count = 0
            for line in steps_path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                step = json.loads(line)
                observation_state = self._build_observation_state(step)
                action = self._build_action(step.get("action", {}), observation_state)
                sample = {
                    "episode_id": metadata["episode_id"],
                    "episode_index": episode_index,
                    "frame_index": step.get("frame_index", step_count),
                    "task_name": metadata["task_name"],
                    "instruction": metadata["natural_language_instruction"],
                    "timestamp": step["timestamp"],
                    "metadata": export_metadata,
                    "observation": {
                        "rgb_path": step["rgb_path"],
                        "rgb_paths": step.get("rgb_paths"),
                        "depth_path": step["depth_path"],
                        "depth_paths": step.get("depth_paths"),
                        "arm_joint_states": step["arm_joint_states"],
                        "arm_ee_pose": step["arm_ee_pose"],
                        "hand_states": step["hand_states"],
                        "state": observation_state,
                        "dog_states": step["dog_states"],
                        "extra_observation": step.get("extra_observation"),
                    },
                    "action": action,
                    "episode_success": metadata["success"],
                    "episode_failure_mode": failure_mode,
                    "operator_notes": step["operator_notes"] or metadata.get("operator_notes", ""),
                }
                self._append_jsonl(samples_path, sample)
                step_count += 1
            manifest["episodes"].append(
                {
                    "episode_id": metadata["episode_id"],
                    "task_name": metadata["task_name"],
                    "success": metadata["success"],
                    "failure_mode": failure_mode,
                    "step_count": step_count,
                    "source_dir": str(episode_dir),
                    "metadata": export_metadata,
                }
            )
        manifest["metadata"] = self._schema_metadata({})
        (export_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        return export_dir

    def clone_episode_tree(self, export_dir: str | Path) -> Path:
        export_dir = Path(export_dir).resolve()
        export_dir.mkdir(parents=True, exist_ok=True)
        for episode_dir in sorted(self.data_root.glob("episode_*")):
            target = export_dir / episode_dir.name
            if target.exists():
                shutil.rmtree(target)
            shutil.copytree(episode_dir, target)
        return export_dir

    def _write_metadata(self) -> None:
        if self._current_episode_dir is None or self._metadata is None:
            return
        metadata_path = self._current_episode_dir / "metadata.json"
        metadata_path.write_text(json.dumps(self._metadata, indent=2), encoding="utf-8")

    def _save_array(self, subdir: str, timestamp: float, array: Any) -> str | None:
        if self._current_episode_dir is None or array is None:
            return None
        np_array = np.asarray(array)
        path = self._current_episode_dir / subdir / f"{timestamp:.6f}.npy"
        np.save(path, np_array)
        return str(path)

    @staticmethod
    def _append_jsonl(path: Path, payload: dict[str, Any]) -> None:
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload) + "\n")

    @staticmethod
    def _schema_metadata(metadata: dict[str, Any]) -> dict[str, Any]:
        merged = dict(DEFAULT_SCHEMA_METADATA)
        extra = metadata.get("extra_metadata", {}) if isinstance(metadata.get("extra_metadata"), dict) else {}
        for source in (extra, metadata):
            for key in DEFAULT_SCHEMA_METADATA:
                if key in source and source[key] is not None:
                    if key == "schema_version" and source[key] not in SUPPORTED_SCHEMA_VERSIONS:
                        continue
                    merged[key] = source[key]
        return merged

    @staticmethod
    def _normalize_failure_mode(
        *,
        success: bool,
        failure_mode: str | None = None,
        failure_reason: str = "",
    ) -> str:
        if success:
            return "none"
        candidate = failure_mode or failure_reason or "unknown"
        legacy_map = {
            "success": "none",
            "fail_no_contact": "fail_low_grip",
            "fail_slip": "fail_object_slip",
            "fail_wrong_pose": "fail_lateral_offset",
            "fail_object_dropped": "fail_object_slip",
            "fail_hardware_error": "unknown",
        }
        candidate = legacy_map.get(candidate, candidate)
        return candidate if candidate in FAILURE_MODES and candidate != "none" else "unknown"

    @staticmethod
    def _list_or_empty(value: Any) -> list[float]:
        if value is None:
            return []
        array = np.asarray(value, dtype=np.float32).reshape(-1)
        return array.astype(np.float32).tolist()

    @staticmethod
    def _six(value: Any, *, clip01: bool = False) -> list[float]:
        values = EpisodeRecorder._list_or_empty(value)
        if len(values) < 6:
            values = values + [0.0] * (6 - len(values))
        values = values[:6]
        if clip01:
            values = np.clip(np.asarray(values, dtype=np.float32), 0.0, 1.0).tolist()
        return values

    @staticmethod
    def _hand_state_from_hand_states(hand_states: dict[str, Any] | None) -> list[float]:
        hand_states = hand_states or {}
        inspire6 = hand_states.get("inspire6") if isinstance(hand_states.get("inspire6"), dict) else {}
        normalized = inspire6.get("normalized_positions")
        if normalized is not None:
            return EpisodeRecorder._six(normalized, clip01=True)
        positions = inspire6.get("positions")
        if positions is not None:
            return EpisodeRecorder._six(positions, clip01=True)
        return EpisodeRecorder._six(hand_states.get("finger_positions"), clip01=True)

    @staticmethod
    def _build_observation_state(step: dict[str, Any]) -> dict[str, Any]:
        existing = dict(step.get("state") or {})
        action = step.get("action", {}) or {}
        hand_state = EpisodeRecorder._six(
            existing.get("hand_state") or EpisodeRecorder._hand_state_from_hand_states(step.get("hand_states")),
            clip01=True,
        )
        hand_delta_cmd = EpisodeRecorder._six(action.get("hand_delta_cmd") or action.get("hand_delta"))
        hand_cmd = EpisodeRecorder._six(action.get("hand_cmd"), clip01=True)
        inferred_last_cmd = (np.asarray(hand_cmd, dtype=np.float32) - np.asarray(hand_delta_cmd, dtype=np.float32)).tolist()
        hand_cmd_last = EpisodeRecorder._six(existing.get("hand_cmd_last") or action.get("last_hand_cmd") or inferred_last_cmd, clip01=True)
        hand_error = (
            np.asarray(existing.get("hand_error"), dtype=np.float32).reshape(-1).tolist()
            if existing.get("hand_error") is not None
            else (np.asarray(hand_cmd_last, dtype=np.float32) - np.asarray(hand_state, dtype=np.float32)).tolist()
        )

        arm_joint_states = step.get("arm_joint_states") or {}
        arm_positions = EpisodeRecorder._six(arm_joint_states.get("positions"))
        arm_ee_pose = step.get("arm_ee_pose") or {}
        extra = step.get("extra_observation") or {}
        state = {
            **existing,
            "ee_state": existing.get("ee_state")
            or list(arm_ee_pose.get("position") or [0.0, 0.0, 0.0])
            + list(arm_ee_pose.get("orientation_xyzw") or [0.0, 0.0, 0.0, 1.0]),
            "robot_q_current": EpisodeRecorder._six(existing.get("robot_q_current") or arm_positions),
            "hand_state": hand_state,
            "hand_cmd_last": hand_cmd_last,
            "hand_error": EpisodeRecorder._six(hand_error),
            "canonical_hand_order": list(CANONICAL_HAND_ORDER),
            "object_pose": existing.get("object_pose") or extra.get("obj_pose"),
            "object_pose_is_privileged": True,
        }
        return state

    @staticmethod
    def _build_action(action: dict[str, Any], observation_state: dict[str, Any]) -> dict[str, Any]:
        action = dict(action or {})
        hand_cmd = EpisodeRecorder._six(action.get("hand_cmd"), clip01=True)
        hand_delta_cmd = EpisodeRecorder._six(action.get("hand_delta_cmd") or action.get("hand_delta"))
        hand_delta_state_raw = EpisodeRecorder._six(
            action.get("hand_delta_state_raw")
            or action.get("hand_delta_state")
            or (np.asarray(hand_cmd, dtype=np.float32) - np.asarray(observation_state["hand_state"], dtype=np.float32)).tolist()
        )
        hand_delta_state = np.clip(
            np.asarray(hand_delta_state_raw, dtype=np.float32),
            -DEFAULT_HAND_DELTA_LIMIT,
            DEFAULT_HAND_DELTA_LIMIT,
        ).tolist()
        action["hand_cmd"] = hand_cmd
        action["hand_delta_cmd"] = hand_delta_cmd
        action["hand_delta_state_raw"] = hand_delta_state_raw
        action["hand_delta_state"] = hand_delta_state
        action.pop("hand_delta", None)
        action["hand_order"] = list(CANONICAL_HAND_ORDER)
        action["ee_delta"] = EpisodeRecorder._six(action.get("ee_delta"))
        action["robot_q_current"] = EpisodeRecorder._six(action.get("robot_q_current") or observation_state["robot_q_current"])
        action["robot_q_desired"] = EpisodeRecorder._six(action.get("robot_q_desired"))
        return action
