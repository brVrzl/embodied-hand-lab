from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from episode_dataset.openpi_adapter import smoke_openpi_dataset
from episode_dataset.training_materialization import (
    materialize_training_dataset,
    validate_training_dataset,
)
from episode_dataset.training_views import ActDatasetAdapter, ActForceDatasetAdapter


def _write_video(path: Path, frame_count: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (640, 480)
    )
    assert writer.isOpened()
    for index in range(frame_count):
        writer.write(np.full((480, 640, 3), index, dtype=np.uint8))
    writer.release()


def _synthetic_staging(
    root: Path,
    *,
    episode_id: int = 65,
    future_row: bool = False,
) -> tuple[Path, Path]:
    data = root / "data/chunk-000"
    metadata_dir = root / "meta/episodes/chunk-000"
    episode_name = f"episode_{episode_id:06d}"
    workspace = root / f"videos/observation.images.workspace/chunk-000/{episode_name}.mp4"
    wrist = root / f"videos/observation.images.wrist/chunk-000/{episode_name}.mp4"
    data.mkdir(parents=True, exist_ok=True)
    metadata_dir.mkdir(parents=True, exist_ok=True)
    _write_video(workspace, 5)
    _write_video(wrist, 5)
    base = 10_000_000_000
    period = 1_000_000_000 // 30
    release = base + 2 * period
    rows: list[dict[str, object]] = []
    for index in range(5):
        timestamp = base + index * period
        source_base = timestamp - 1_000_000
        if future_row and index == 1:
            source_base = timestamp + 1_000_000
        timestamps = {
            "jaka_observation": source_base,
            "jaka_command": source_base,
            "rh56_angle_act": source_base,
            "rh56_force_act": source_base,
            "workspace": source_base,
            "wrist": source_base,
        }
        domains = {name: "host_monotonic_ns" for name in timestamps}
        validity = {
            "jaka_observation": True,
            "rh56_angle_act": True,
            "rh56_force_act": True,
            "workspace": True,
            "wrist": True,
        }
        timing = {
            "canonical_host_monotonic_ns": timestamp,
            "source_timestamps_ns": timestamps,
            "source_timestamp_domains": domains,
            "source_validity": validity,
            "source_age_ns": {name: timestamp - value for name, value in timestamps.items()},
            "rh56_force_act_valid": True,
            "synchronization_valid": True,
        }
        rows.append(
            {
                "frame_index": index,
                "timestamp_ns": timestamp,
                "timestamp": index / 30.0,
                "observation.state": [float(index)] * 12,
                "action": [float(index + 1)] * 12,
                "observation.force": [float(index)] * 6,
                "action_status": "accepted",
                "arm_trigger": True,
                "hand_grip": index > 0,
                "camera_frame_index": {"workspace": index, "wrist": index},
                "camera": {
                    role: {
                        "valid": True,
                        "rgb_timestamp_domain": "global_time",
                        "rgb_frame_number": index,
                    }
                    for role in ("workspace", "wrist")
                },
                "timing": timing,
            }
        )
    row_path = data / f"{episode_name}.jsonl"
    row_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    metadata_path = metadata_dir / f"{episode_name}.json"
    metadata_path.write_text(
        json.dumps(
            {
                "episode_index": episode_id,
                "episode_name": episode_name,
                "completion_status": "completed",
                "duration_s": 5 / 30,
                "num_frames": 5,
                "trigger_release_timestamp_ns": release,
            }
        ),
        encoding="utf-8",
    )
    return row_path, metadata_path


def _config(
    tmp_path: Path,
    source: Path,
    output: Path,
    *,
    episode_ids: tuple[int, ...] = (65,),
    future_row: bool = False,
) -> Path:
    config = tmp_path / ("future.yaml" if future_row else "config.yaml")
    config.write_text(
        yaml.safe_dump(
            {
                "schema_version": "embodied_lab.physical_training_config.v1",
                "source_root": str(source),
                "output_root": str(output),
                "task": {
                    "id": "place_bottle_on_cardboard_box",
                    "prompt": "Pick up the bottle and place it on the cardboard box.",
                },
                "fps": 30,
                "split_policy": {
                    "mode": "explicit",
                    "train": list(episode_ids),
                    "val": [],
                    "test": [],
                },
                "episodes": [
                    {
                        "id": episode_id,
                        "include": True,
                        "clean_demo": True,
                        "recovery_heavy": False,
                    }
                    for episode_id in episode_ids
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return config


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_materialization_is_cropped_causal_and_repeatable(tmp_path: Path) -> None:
    source = tmp_path / "raw"
    row_path, metadata_path = _synthetic_staging(source)
    source_hashes = {_path: _sha256(_path) for _path in (row_path, metadata_path)}
    for video in source.glob("videos/**/*.mp4"):
        source_hashes[video] = _sha256(video)
    config = _config(tmp_path, source, tmp_path / "training")

    result = materialize_training_dataset(config)
    assert not result.reused
    assert result.summary["master_row_count"] == 3
    assert result.summary["act_force_row_count"] == 3
    assert result.summary["episodes"][0]["crop"]["excluded_tail_rows"] == 2
    assert result.summary["episodes"][0]["video_modes"]["workspace"] == "materialized_cropped_video"
    for path, digest in source_hashes.items():
        assert _sha256(path) == digest

    validation = validate_training_dataset(result.output_root)
    assert validation["status"] == "passed"
    assert validation["row_count"] == 3

    import pyarrow.parquet as parquet

    table = parquet.read_table(result.output_root / "master/data/chunk-000/episode_000065.parquet")
    assert table.num_rows == 3
    assert table.column("observation.state").type.list_size == 12
    assert table.column("action").type.list_size == 12
    assert table.column("observation.force").type.list_size == 6
    assert table.column("segment_id").to_pylist() == [0, 0, 0]

    act = ActDatasetAdapter(result.output_root, action_horizon=4)
    assert len(act) == 3
    assert act[0]["action"].shape == (4, 12)
    assert act[-1]["action_mask"].tolist() == [True, False, False, False]
    assert act[-1]["observation"]["state"].shape == (12,)
    force = ActForceDatasetAdapter(result.output_root, action_horizon=4)
    assert len(force) == 3
    assert force[0]["observation"]["force"].shape == (6,)
    assert force[0]["observation"]["force_valid"] is True
    assert smoke_openpi_dataset(result.output_root)["status"] in {
        "skeleton_only_package_not_installed",
        "package_available_mapping_checked",
    }

    second = materialize_training_dataset(config)
    assert second.reused
    assert second.summary["fingerprint"] == result.summary["fingerprint"]


def test_future_source_row_is_excluded_without_repair(tmp_path: Path) -> None:
    source = tmp_path / "raw"
    _synthetic_staging(source, future_row=True)
    config = _config(tmp_path, source, tmp_path / "training", future_row=True)
    result = materialize_training_dataset(config)
    assert result.summary["master_row_count"] == 2
    excluded = (result.output_root / "reports/excluded_rows.jsonl").read_text(encoding="utf-8")
    assert "future_source:jaka_observation" in excluded
    assert "future_source:rh56_force_act" in excluded


def test_action_chunk_stops_at_episode_boundary(tmp_path: Path) -> None:
    source = tmp_path / "raw"
    _synthetic_staging(source, episode_id=65)
    _synthetic_staging(source, episode_id=66)
    config = _config(tmp_path, source, tmp_path / "training", episode_ids=(65, 66))
    result = materialize_training_dataset(config)
    dataset = ActDatasetAdapter(result.output_root, action_horizon=4)
    assert len(dataset) == 6
    last_first_episode = dataset[2]
    assert last_first_episode["episode_index"] == 65
    assert last_first_episode["action_mask"].tolist() == [True, False, False, False]
    assert dataset[3]["episode_index"] == 66
