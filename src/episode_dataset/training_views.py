"""Small dependency-light ACT and ACT+Force views over the master dataset."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


def absolute_native_action(values: Sequence[float]) -> np.ndarray:
    """Return the stored absolute target without a delta conversion."""

    array = np.asarray(values, dtype=np.float32)
    if array.shape != (12,) or not np.isfinite(array).all():
        raise ValueError("absolute action must be twelve finite values")
    return array.copy()


def deployment_action(values: Sequence[float]) -> np.ndarray:
    """Inverse of the initial policy transform: absolute target identity."""

    return absolute_native_action(values)


@dataclass(frozen=True, slots=True)
class _RowRef:
    episode_index: int
    row: Mapping[str, Any]
    video_root: Path


class ActDatasetAdapter:
    """ACT-style examples with two RGB images, state, and action chunks.

    The repository has no maintained upstream ACT trainer.  This adapter uses
    a documented local convention: observations are uint8 CHW RGB arrays,
    actions are absolute/native `[horizon, 12]` targets, and the boolean mask
    marks real actions when the horizon reaches an episode/segment boundary.
    """

    def __init__(
        self,
        dataset_root: str | Path,
        *,
        split: str = "train",
        action_horizon: int = 16,
        force: bool = False,
    ) -> None:
        self.root = Path(dataset_root).resolve()
        self.master = self.root / "master"
        if action_horizon <= 0:
            raise ValueError("action_horizon must be positive")
        self.action_horizon = int(action_horizon)
        self.force_enabled = bool(force)
        splits = json.loads((self.root / "manifests" / "splits.json").read_text(encoding="utf-8"))["splits"]
        if split not in splits:
            raise ValueError(f"unknown split {split!r}")
        episode_ids = {int(value) for value in splits[split]}
        self._episodes: dict[int, list[dict[str, Any]]] = {}
        self._refs: list[_RowRef] = []
        for line in (self.master / "meta" / "episodes.jsonl").read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            episode = json.loads(line)
            episode_id = int(episode["episode_index"])
            if episode_id not in episode_ids:
                continue
            try:
                import pyarrow.parquet as parquet
            except ImportError as exc:
                raise RuntimeError("ACT adapter requires pyarrow") from exc
            rows = parquet.read_table(self.master / episode["data"]).to_pylist()
            # ACT+Force keeps the exact ACT sample set.  Invalid/stale force is
            # represented by the row's validity/age fields and is not a reason
            # to silently create a different dataset composition.
            self._episodes[episode_id] = rows
            video_root = self.master / "videos"
            for row in rows:
                self._refs.append(_RowRef(episode_id, row, video_root))

    def __len__(self) -> int:
        return len(self._refs)

    def __getitem__(self, index: int) -> dict[str, Any]:
        ref = self._refs[index]
        row = ref.row
        episode_rows = self._episodes[ref.episode_index]
        local_index = next(i for i, candidate in enumerate(episode_rows) if candidate["index"] == row["index"])
        segment = row.get("segment_id", 0)
        chunk_rows: list[Mapping[str, Any]] = []
        previous = row
        expected_period_ns = 1_000_000_000 / 30.0
        for candidate in episode_rows[local_index : local_index + self.action_horizon]:
            if candidate.get("segment_id", 0) != segment:
                break
            if chunk_rows:
                raw_gap = int(candidate.get("raw_frame_index", -1)) - int(previous.get("raw_frame_index", -1))
                time_gap = int(candidate["timestamp_ns"]) - int(previous["timestamp_ns"])
                if raw_gap != 1 or time_gap > expected_period_ns * 1.5:
                    break
            chunk_rows.append(candidate)
            previous = candidate
        actions = [absolute_native_action(candidate["action"]) for candidate in chunk_rows]
        mask = np.zeros(self.action_horizon, dtype=bool)
        chunk = np.zeros((self.action_horizon, 12), dtype=np.float32)
        if actions:
            for action_index, action in enumerate(actions):
                chunk[action_index] = action
                mask[action_index] = True
            if len(actions) < self.action_horizon:
                chunk[len(actions) :] = actions[-1]
        else:
            chunk[0] = absolute_native_action(row["action"])
            mask[0] = True
        observation: dict[str, Any] = {
            "images": {
                "workspace": self._read_image(ref.episode_index, "workspace", int(row["frame_index"])),
                "wrist": self._read_image(ref.episode_index, "wrist", int(row["frame_index"])),
            },
            "state": np.asarray(row["observation.state"], dtype=np.float32),
        }
        if self.force_enabled:
            observation["force"] = np.asarray(row["observation.force"], dtype=np.float32)
            observation["force_age_s"] = None if row["force_age_s"] is None else float(row["force_age_s"])
            observation["force_valid"] = bool(row["force_valid"])
            observation["force_timestamp_ns"] = row["force_timestamp_ns"]
        return {
            "observation": observation,
            "action": chunk,
            "action_mask": mask,
            "episode_index": ref.episode_index,
            "frame_index": int(row["frame_index"]),
            "timestamp_ns": int(row["timestamp_ns"]),
            "task": str(row["task"]),
            "action_semantics": "absolute_native_target",
        }

    def _read_image(self, episode_index: int, role: str, frame_index: int) -> np.ndarray:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("ACT image loading requires opencv") from exc
        path = self.master / "videos" / f"observation.images.{role}" / "chunk-000" / f"episode_{episode_index:06d}.mp4"
        capture = cv2.VideoCapture(str(path))
        if not capture.isOpened():
            raise RuntimeError(f"cannot open training video {path}")
        capture.set(cv2.CAP_PROP_POS_FRAMES, float(frame_index))
        ok, image = capture.read()
        capture.release()
        if not ok or image is None:
            raise RuntimeError(f"cannot read {role} frame {frame_index} from {path}")
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return np.transpose(rgb, (2, 0, 1)).copy()


class ActForceDatasetAdapter(ActDatasetAdapter):
    """The same ACT rows with causal native RH56 force metadata enabled."""

    def __init__(self, dataset_root: str | Path, *, split: str = "train", action_horizon: int = 16) -> None:
        super().__init__(dataset_root, split=split, action_horizon=action_horizon, force=True)


def smoke_act_dataset(dataset_root: str | Path, *, split: str = "train", action_horizon: int = 16, force: bool = False) -> dict[str, Any]:
    dataset = ActDatasetAdapter(dataset_root, split=split, action_horizon=action_horizon, force=force)
    if not len(dataset):
        raise ValueError(f"split {split!r} has no rows")
    indices = sorted({0, len(dataset) // 2, len(dataset) - 1})
    samples = [dataset[index] for index in indices]
    batch_indices = list(range(min(2, len(dataset))))
    batch = [dataset[index] for index in batch_indices]
    for sample in samples:
        if sample["observation"]["state"].shape != (12,):
            raise ValueError("ACT state shape is not (12,)")
        if sample["action"].shape != (action_horizon, 12):
            raise ValueError("ACT action chunk shape is incorrect")
        if sample["observation"]["images"]["workspace"].shape[0] != 3:
            raise ValueError("workspace image is not CHW RGB")
    return {
        "status": "passed",
        "view": "act_force" if force else "act",
        "split": split,
        "sample_count": len(dataset),
        "checked_indices": indices,
        "batch_size_checked": len(batch),
        "state_shape": list(samples[0]["observation"]["state"].shape),
        "action_chunk_shape": list(samples[0]["action"].shape),
        "workspace_image_shape": list(samples[0]["observation"]["images"]["workspace"].shape),
        "wrist_image_shape": list(samples[0]["observation"]["images"]["wrist"].shape),
        "action_semantics": "absolute_native_target",
        "force_shape": list(samples[0]["observation"]["force"].shape) if force else None,
    }
