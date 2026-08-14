"""Project-specific, read-only adapter for the audited RH56 demonstrations.

This module deliberately creates a derived LeRobot v2.0 view instead of
changing the ACT source view.  The source view contains the repository's
audited action contract, including the six active RH56 actuator channels.
Force observations remain in the source parquet files but are not exposed to
the first pi0.5 baseline.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path
from typing import Any

import numpy as np


TASK_PROMPT = "Pick up the bottle and place it on the cardboard box."
STATE_DIM = 12
ACTION_DIM = 12
FPS = 30
ACTION_HORIZON = 16
IMAGE_KEYS = ("observation.images.workspace", "observation.images.wrist")
STATE_ORDER = (
    "jaka_joint_1",
    "jaka_joint_2",
    "jaka_joint_3",
    "jaka_joint_4",
    "jaka_joint_5",
    "jaka_joint_6",
    "rh56_index",
    "rh56_middle",
    "rh56_ring",
    "rh56_pinky",
    "rh56_thumb_close",
    "rh56_thumb_lateral",
)
ACTION_ORDER = (
    "jaka_target_1",
    "jaka_target_2",
    "jaka_target_3",
    "jaka_target_4",
    "jaka_target_5",
    "jaka_target_6",
    "rh56_target_index",
    "rh56_target_middle",
    "rh56_target_ring",
    "rh56_target_pinky",
    "rh56_target_thumb_close",
    "rh56_target_thumb_lateral",
)


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text())


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as f:
        json.dump(value, f, indent=2, sort_keys=True)
        f.write("\n")
        temporary = Path(f.name)
    os.replace(temporary, path)


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as f:
        for row in rows:
            f.write(json.dumps(row, sort_keys=True) + "\n")
        temporary = Path(f.name)
    os.replace(temporary, path)


def _source_stats_dir(source_master: Path) -> Path:
    # .../physical_bottle_v4_nominal52/act/master -> .../physical_bottle_v4_nominal52/stats
    return source_master.parent.parent / "stats"


def _source_fingerprint(source_master: Path) -> str:
    """Fingerprint paths, sizes, and mtimes without rereading multi-GB video."""

    digest = hashlib.sha256()
    roots = (source_master / "meta", source_master / "data", source_master / "videos")
    entries: list[tuple[str, int, int]] = []
    for root in roots:
        if not root.exists():
            raise FileNotFoundError(root)
        for path in sorted(root.rglob("*")):
            if path.is_file() or path.is_symlink():
                resolved = path.resolve()
                stat = resolved.stat()
                entries.append((str(path.relative_to(source_master)), stat.st_size, stat.st_mtime_ns))
    for relative, size, mtime_ns in entries:
        digest.update(f"{relative}\0{size}\0{mtime_ns}\n".encode())
    return digest.hexdigest()


def _load_source(source_master: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    info = _read_json(source_master / "meta/info.json")
    episodes = _read_jsonl(source_master / "meta/episodes.jsonl")
    tasks = {row["task_index"]: row["task"] for row in _read_jsonl(source_master / "meta/tasks.jsonl")}
    if tasks != {0: TASK_PROMPT}:
        raise ValueError(f"Unexpected task mapping: {tasks}")
    if info["task_prompt"] != TASK_PROMPT or info["fps"] != FPS:
        raise ValueError("The source view is not the fixed ACT task at 30 Hz")
    if info["observation_state_order"] != list(STATE_ORDER) or info["action_order"] != list(ACTION_ORDER):
        raise ValueError("The source state/action order does not match the documented ACT contract")
    if info["total_episodes"] != len(episodes) or info["total_episodes"] != 52:
        raise ValueError(f"Expected exactly 52 source episodes, got {len(episodes)}")
    return info, episodes, tasks


def _load_audit_segments(source_master: Path) -> dict[str, dict[str, Any]]:
    path = source_master.parent.parent / "manifests/logical_segments.json"
    segments = _read_json(path)["segments"]
    # This is the materialized audit manifest, whose accepted entries use
    # status=included. The collection YAML has a different include field; do
    # not synthesize acceptance from either one.
    result = {segment["id"]: segment for segment in segments if segment.get("status") == "included"}
    if len(result) != 52:
        raise ValueError(f"The explicit included audit manifest has {len(result)} segments, expected 52")
    return result


def _load_locked_split(source_master: Path, split_name: str) -> dict[str, Any]:
    """Load the existing ACT logical-episode split; never invent a split here."""

    split_path = source_master.parent / "manifests/splits.json"
    split = _read_json(split_path)
    if split.get("unit") != "logical_segment":
        raise ValueError(f"Unexpected split unit in {split_path}: {split.get('unit')!r}")
    indices = split.get("splits", {}).get(split_name)
    if not isinstance(indices, list) or not all(isinstance(index, int) for index in indices):
        raise ValueError(f"Missing integer {split_name!r} split in {split_path}")
    all_indices = set(split["splits"].get("train", [])) | set(split["splits"].get("val", [])) | set(
        split["splits"].get("test", [])
    )
    if all_indices != set(range(52)) or len(set(indices)) != len(indices):
        raise ValueError(f"The locked ACT split is incomplete or duplicated: {split_path}")
    return {"path": str(split_path), "name": split_name, "episode_indices": indices}


def _read_episode_arrays(source_master: Path, episode: dict[str, Any]) -> dict[str, np.ndarray]:
    try:
        import pyarrow.parquet as pq
    except ImportError as exc:  # pragma: no cover - exercised in the Thor container
        raise RuntimeError("pyarrow is required for the adapter validation commands") from exc

    table = pq.read_table(source_master / episode["data"])
    columns = {name: np.asarray(table[name].to_pylist()) for name in table.column_names}
    for key in ("timestamp_ns", "source_frame_index"):
        if key not in columns:
            raise ValueError(f"{episode['logical_segment_id']} is missing {key}")
    state = np.asarray(columns["observation.state"], dtype=np.float32)
    action = np.asarray(columns["action"], dtype=np.float32)
    if state.shape != (episode["length"], STATE_DIM):
        raise ValueError(f"{episode['logical_segment_id']} state shape is {state.shape}")
    if action.shape != (episode["length"], ACTION_DIM):
        raise ValueError(f"{episode['logical_segment_id']} action shape is {action.shape}")
    if not np.isfinite(state).all() or not np.isfinite(action).all():
        raise ValueError(f"{episode['logical_segment_id']} contains non-finite state/action values")
    timestamps = np.asarray(columns["timestamp_ns"], dtype=np.int64)
    source_frames = np.asarray(columns["source_frame_index"], dtype=np.int64)
    relative_timestamps = np.asarray(columns.get("timestamp", np.arange(len(state)) / FPS), dtype=np.float64)
    if len(timestamps) != episode["length"] or len(source_frames) != episode["length"]:
        raise ValueError(f"{episode['logical_segment_id']} metadata length mismatch")
    if np.any(np.diff(timestamps) <= 0):
        raise ValueError(f"{episode['logical_segment_id']} timestamps are not strictly increasing")
    if np.any(np.diff(source_frames) != 1):
        raise ValueError(f"{episode['logical_segment_id']} source frame indices are not contiguous")
    timestamp_regular = bool(
        np.all(np.abs(np.diff(relative_timestamps) - (1.0 / FPS)) <= 2e-5)
    )
    return {
        "state": state,
        "action": action,
        "timestamp_ns": timestamps,
        "source_frame_index": source_frames,
        "relative_timestamp": relative_timestamps,
        "relative_timestamp_regular_30hz": np.asarray(timestamp_regular),
    }


def _video_metadata(path: Path) -> dict[str, Any]:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - exercised in the Thor container
        raise RuntimeError("opencv-python is required for camera validation") from exc
    if not path.exists():
        raise FileNotFoundError(path)
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError(f"Cannot open camera video {path}")
    result = {
        "path": str(path),
        "frame_count_header": int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT))),
        "width": int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH))),
        "height": int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))),
        "fps": float(capture.get(cv2.CAP_PROP_FPS)),
    }
    capture.release()
    if result["width"] != 640 or result["height"] != 480 or abs(result["fps"] - FPS) > 0.2:
        raise ValueError(f"Unexpected camera metadata for {path}: {result}")
    return result


def build_audited_manifest(source_master: Path, output_path: Path) -> dict[str, Any]:
    source_master = source_master.resolve()
    info, episodes, _ = _load_source(source_master)
    audit_segments = _load_audit_segments(source_master)
    locked_split_path = source_master.parent / "manifests/splits.json"
    locked_split = _read_json(locked_split_path)
    if locked_split.get("unit") != "logical_segment":
        raise ValueError(f"Unexpected split unit in {locked_split_path}")
    source_fingerprint = _source_fingerprint(source_master)

    entries: list[dict[str, Any]] = []
    total_frames = 0
    for episode in episodes:
        segment_id = episode["logical_segment_id"]
        audit = audit_segments.get(segment_id)
        if audit is None or audit.get("status") != "included":
            raise ValueError(f"Source episode {segment_id} is not explicitly included by the audit manifest")
        arrays = _read_episode_arrays(source_master, episode)
        cameras: dict[str, Any] = {}
        for camera in ("workspace", "wrist"):
            video_path = source_master / episode["videos"][camera]
            cameras[camera] = _video_metadata(video_path)
            cameras[camera]["availability"] = True
        duration = float((arrays["timestamp_ns"][-1] - arrays["timestamp_ns"][0]) / 1_000_000_000)
        entries.append(
            {
                "episode_index": episode["episode_index"],
                "episode_identifier": segment_id,
                "episode_path": episode["data"],
                "source_episode": episode["source_episode_index"],
                "audit_status": "human_audited_accepted",
                "audit_classification": audit["classification"],
                "audit_notes": audit.get("notes"),
                "audit_source": [
                    "configs/training/physical_bottle_v4_nominal52.yaml",
                    "research_log/physical_bottle_nominal52_audit.md",
                    "data/training/physical_bottle_v4_nominal52/manifests/logical_segments.json",
                ],
                "task": TASK_PROMPT,
                "frame_count": int(episode["length"]),
                "duration_s": duration,
                "camera_availability": cameras,
                "state_dim": STATE_DIM,
                "action_dim": ACTION_DIM,
                "state_order": list(STATE_ORDER),
                "action_order": list(ACTION_ORDER),
                "action_semantics": "absolute_native_target",
                "rh56_action_semantics": "six active underactuated actuator targets in index,middle,ring,pinky,thumb_close,thumb_lateral order",
                "force_preserved_in_source": True,
                "force_used_by_baseline": False,
                "source_timestamp_regular_30hz": bool(arrays["relative_timestamp_regular_30hz"]),
                "derived_timestamp_policy": "preserve timestamp" if bool(arrays["relative_timestamp_regular_30hz"]) else "frame_index / 30.0; original timestamp_ns preserved",
            }
        )
        total_frames += int(episode["length"])

    manifest = {
        "schema_version": "embodied_lab.pi05_rh56.audited_manifest.v1",
        "dataset_role": "project_specific_derived_openpi_training_input",
        "source_master": str(source_master),
        "source_master_fingerprint": source_fingerprint,
        "raw_source_immutable": True,
        "task": TASK_PROMPT,
        "audit_basis": "explicit human/operator audit and offline boundary review; no heuristic acceptance",
        "locked_split_manifest": str(locked_split_path),
        "split_unit": locked_split["unit"],
        "split_episode_indices": locked_split["splits"],
        "train_episode_count": len(locked_split["splits"]["train"]),
        "validation_episode_count": len(locked_split["splits"]["val"]),
        "accepted_episode_count": len(entries),
        "total_frame_count": total_frames,
        "fps": FPS,
        "state_dim": STATE_DIM,
        "action_dim": ACTION_DIM,
        "action_horizon": ACTION_HORIZON,
        "camera_keys": list(IMAGE_KEYS),
        "force_in_first_baseline": False,
        "episodes": entries,
        "source_info_summary": {
            "robot_type": info["robot_type"],
            "schema_version": info["schema_version"],
            "training_view": info["training_view"],
            "total_source_episodes": info["total_episodes"],
            "total_source_frames": info["total_frames"],
        },
    }
    _write_json(output_path, manifest)
    return manifest


def _stats_for_lerobot(source_master: Path) -> dict[str, dict[str, Any]]:
    stats_dir = _source_stats_dir(source_master)
    state = _read_json(stats_dir / "state_stats.json")
    action = _read_json(stats_dir / "action_stats.json")
    if state["names"] != list(STATE_ORDER) or action["names"] != list(ACTION_ORDER):
        raise ValueError("Source statistics order does not match the source info contract")
    # LeRobot v2 metadata only needs numeric aggregate statistics here. OpenPI
    # computes its own q01/q99 from the final training view below.
    return {
        "observation.state": {key: state[key] for key in ("mean", "std", "minimum", "maximum")},
        "action": {key: action[key] for key in ("mean", "std", "minimum", "maximum")},
    }


def build_openpi_view(
    source_master: Path, view_root: Path, manifest_path: Path, *, split_name: str = "train"
) -> dict[str, Any]:
    source_master = source_master.resolve()
    view_root = view_root.resolve()
    if view_root.exists():
        raise FileExistsError(f"Refusing to replace an existing derived view: {view_root}")
    manifest = _read_json(manifest_path)
    if manifest["source_master_fingerprint"] != _source_fingerprint(source_master):
        raise ValueError("The source master changed after the audited manifest was generated")
    info, episodes, _ = _load_source(source_master)
    if manifest["accepted_episode_count"] != len(episodes):
        raise ValueError("Manifest/source episode count mismatch")
    locked_split = _load_locked_split(source_master, split_name)
    selected_indices = set(locked_split["episode_indices"])
    selected_episodes = [episode for episode in episodes if episode["episode_index"] in selected_indices]
    if len(selected_episodes) != len(selected_indices):
        raise ValueError(f"Locked split {split_name!r} references unknown episode indices")
    derived_episodes: list[dict[str, Any]] = []
    for derived_index, source_episode in enumerate(selected_episodes):
        episode_chunk = derived_index // 1000
        derived_episodes.append(
            {
                **source_episode,
                "episode_index": derived_index,
                "data": f"data/chunk-{episode_chunk:03d}/episode_{derived_index:06d}.parquet",
                "videos": {
                    camera: f"videos/observation.images.{camera}/chunk-{episode_chunk:03d}/episode_{derived_index:06d}.mp4"
                    for camera in ("workspace", "wrist")
                },
            }
        )

    (view_root / "meta").mkdir(parents=True)
    (view_root / "data/chunk-000").mkdir(parents=True)
    (view_root / "videos/observation.images.workspace/chunk-000").mkdir(parents=True)
    (view_root / "videos/observation.images.wrist/chunk-000").mkdir(parents=True)

    derived_info = dict(info)
    selected_frame_count = sum(int(episode["length"]) for episode in selected_episodes)
    derived_info.update(
        {
            "codebase_version": "v2.0",
            "total_episodes": len(derived_episodes),
            "total_frames": selected_frame_count,
            "total_chunks": 1,
            "total_videos": 2 * len(derived_episodes),
            "chunks_size": 1000,
            "data_files_size_in_mb": 100,
            "video_files_size_in_mb": 200,
            "splits": {split_name: f"0:{len(derived_episodes)}"},
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/{video_key}/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.mp4",
        }
    )
    derived_info["features"] = dict(derived_info["features"])
    derived_info["features"]["timestamp"] = {"dtype": "float32", "shape": [1], "names": None}
    _write_json(view_root / "meta/info.json", derived_info)
    shutil.copy2(source_master / "meta/tasks.jsonl", view_root / "meta/tasks.jsonl")
    _write_jsonl(view_root / "meta/episodes.jsonl", derived_episodes)
    _write_json(view_root / "meta/stats.json", _stats_for_lerobot(source_master))

    timestamp_regularized: list[str] = []
    for source_episode, episode in zip(selected_episodes, derived_episodes, strict=True):
        data_source = (source_master / source_episode["data"]).resolve()
        data_target = view_root / episode["data"]
        data_target.parent.mkdir(parents=True, exist_ok=True)
        arrays = _read_episode_arrays(source_master, source_episode)
        try:
            import pyarrow as pa
            import pyarrow.parquet as pq
        except ImportError as exc:  # pragma: no cover - exercised in the Thor container
            raise RuntimeError("pyarrow is required to materialize the derived LeRobot view") from exc
        table = pq.read_table(data_source)
        episode_index_column = table.schema.get_field_index("episode_index")
        if episode_index_column < 0:
            raise ValueError(f"{source_episode['logical_segment_id']} has no episode_index column")
        derived_episode_indices = pa.array(
            np.full(source_episode["length"], episode["episode_index"], dtype=np.int64),
            type=table.schema.field(episode_index_column).type,
        )
        table = table.set_column(episode_index_column, "episode_index", derived_episode_indices)
        if not bool(arrays["relative_timestamp_regular_30hz"]):
            timestamp_index = table.schema.get_field_index("timestamp")
            if timestamp_index < 0:
                raise ValueError(f"{source_episode['logical_segment_id']} has no timestamp column to regularize")
            regular = np.arange(source_episode["length"], dtype=np.float32) / FPS
            table = table.set_column(timestamp_index, "timestamp", pa.array(regular, type=pa.float32()))
            timestamp_regularized.append(source_episode["logical_segment_id"])
        pq.write_table(table, data_target, compression="zstd")
        for camera in ("workspace", "wrist"):
            video_source = (source_master / source_episode["videos"][camera]).resolve()
            video_target = view_root / episode["videos"][camera]
            video_target.parent.mkdir(parents=True, exist_ok=True)
            os.link(video_source, video_target)

    provenance = {
        "schema_version": "embodied_lab.pi05_rh56.openpi_view_provenance.v1",
        "source_master": str(source_master),
        "source_master_fingerprint": manifest["source_master_fingerprint"],
        "manifest": str(manifest_path.resolve()),
        "audit_accepted_episode_count": len(episodes),
        "selected_episode_count": len(selected_episodes),
        "training_episode_count": len(selected_episodes) if split_name == "train" else 0,
        "split_name": split_name,
        "locked_split_manifest": locked_split["path"],
        "source_episode_indices": [episode["episode_index"] for episode in selected_episodes],
        "episode_index_mapping": [
            {
                "derived_episode_index": derived_episode["episode_index"],
                "source_episode_index": source_episode["episode_index"],
                "source_data": source_episode["data"],
                "source_videos": source_episode["videos"],
            }
            for source_episode, derived_episode in zip(selected_episodes, derived_episodes, strict=True)
        ],
        "held_out_episode_indices": [episode["episode_index"] for episode in episodes if episode["episode_index"] not in selected_indices],
        "task": TASK_PROMPT,
        "state_dim": STATE_DIM,
        "action_dim": ACTION_DIM,
        "action_horizon": ACTION_HORIZON,
        "camera_mapping": {
            "base_0_rgb": "observation.images.workspace",
            "left_wrist_0_rgb": "observation.images.wrist",
            "right_wrist_0_rgb": "synthetic zero image; image mask false",
        },
        "action_mapping": "identity absolute_native_target, arm six dimensions followed by six active RH56 actuator dimensions",
        "force_mapping": "observation.force retained in source parquet but not repacked into pi0.5 baseline",
        "derived_only": True,
        "hardlinks_to_source": False,
        "data_materialization": "copied parquet; derived episode_index is compacted for the selected split and timestamp is regularized where required, while source timestamp_ns/state/action/force columns are preserved",
        "timestamp_regularized_episodes": timestamp_regularized,
        "openpi_dataset_compatibility": "LeRobot v2.0 metadata for the pinned upstream OpenPI loader",
    }
    _write_json(view_root / "meta/openpi_provenance.json", provenance)
    if _source_fingerprint(source_master) != manifest["source_master_fingerprint"]:
        raise RuntimeError("Source fingerprint changed while building the derived view")
    return provenance


def validate_derived_view(view_root: Path, manifest_path: Path) -> dict[str, Any]:
    view_root = view_root.resolve()
    manifest = _read_json(manifest_path)
    if not view_root.exists():
        raise FileNotFoundError(view_root)
    provenance = _read_json(view_root / "meta/openpi_provenance.json")
    episodes = _read_jsonl(view_root / "meta/episodes.jsonl")
    expected_indices = provenance.get("source_episode_indices")
    episode_mapping = provenance.get("episode_index_mapping")
    if (
        not isinstance(expected_indices, list)
        or not isinstance(episode_mapping, list)
        or len(episodes) != len(expected_indices)
        or len(episode_mapping) != len(episodes)
    ):
        raise ValueError("Derived view episode count does not match its locked split")
    if provenance.get("audit_accepted_episode_count") != manifest["accepted_episode_count"]:
        raise ValueError("Derived view audit count does not match manifest")
    mapping_by_derived_index = {item["derived_episode_index"]: item for item in episode_mapping}
    if set(mapping_by_derived_index) != {episode["episode_index"] for episode in episodes}:
        raise ValueError("Derived episode index mapping is incomplete")
    for episode in episodes:
        mapping = mapping_by_derived_index[episode["episode_index"]]
        data_path = view_root / episode["data"]
        source_data_path = Path(manifest["source_master"]) / mapping["source_data"]
        if not data_path.exists():
            raise ValueError(f"Missing derived data: {data_path}")
        if data_path.stat().st_size <= 0 or source_data_path.stat().st_size <= 0:
            raise ValueError(f"Empty derived/source data: {data_path}")
        try:
            import pyarrow.parquet as pq
        except ImportError as exc:  # pragma: no cover - exercised in the Thor container
            raise RuntimeError("pyarrow is required to validate the derived view") from exc
        source_table = pq.read_table(source_data_path)
        derived_table = pq.read_table(data_path)
        for column in ("observation.state", "action", "timestamp_ns", "source_frame_index"):
            source_values = np.asarray(source_table[column].to_pylist())
            derived_values = np.asarray(derived_table[column].to_pylist())
            if not np.array_equal(source_values, derived_values):
                raise ValueError(f"Derived view changed source column {column} in {episode['logical_segment_id']}")
        timestamps = np.asarray(derived_table["timestamp"].to_pylist(), dtype=np.float64)
        expected_timestamps = np.arange(len(timestamps), dtype=np.float64) / FPS
        if not np.allclose(timestamps, expected_timestamps, atol=2e-5, rtol=0.0):
            raise ValueError(f"Derived timestamps are not regular at 30 Hz in {episode['logical_segment_id']}")
        for camera in ("workspace", "wrist"):
            target = view_root / episode["videos"][camera]
            source = Path(manifest["source_master"]) / mapping["source_videos"][camera]
            if not target.exists() or not target.samefile(source):
                raise ValueError(f"Derived video is not an immutable hardlink: {target}")
    return {
        "status": "validated",
        "accepted_episode_count": len(episodes),
        "audit_accepted_episode_count": manifest["accepted_episode_count"],
        "split_name": provenance["split_name"],
        "held_out_episode_count": len(provenance["held_out_episode_indices"]),
        "total_frame_count": sum(int(episode["length"]) for episode in episodes),
        "task": manifest["task"],
        "source_master_fingerprint": manifest["source_master_fingerprint"],
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    manifest = subparsers.add_parser("manifest")
    manifest.add_argument("--source-master", type=Path, required=True)
    manifest.add_argument("--output", type=Path, required=True)
    view = subparsers.add_parser("build-view")
    view.add_argument("--source-master", type=Path, required=True)
    view.add_argument("--view-root", type=Path, required=True)
    view.add_argument("--manifest", type=Path, required=True)
    view.add_argument("--split", default="train", choices=("train", "val"))
    validate = subparsers.add_parser("validate-view")
    validate.add_argument("--view-root", type=Path, required=True)
    validate.add_argument("--manifest", type=Path, required=True)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.command == "manifest":
        result = build_audited_manifest(args.source_master, args.output)
    elif args.command == "build-view":
        result = build_openpi_view(args.source_master, args.view_root, args.manifest, split_name=args.split)
    else:
        result = validate_derived_view(args.view_root, args.manifest)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
