#!/usr/bin/env python3
"""Build and validate pinned LeRobot views for the physical bottle dataset.

The rich master datasets under ``data/training/physical_bottle_v2`` remain the
source of truth.  This tool creates a disposable LeRobot v3 view for the
official LeRobot 0.6.2 trainer.  It is intended to run inside the pinned
training container; it never imports the teleoperation or hardware stack.

For ACT+Force, raw force is exposed as LeRobot's
``observation.environment_state`` feature.  ACT 0.6.2 has a native
environment-state token, so this keeps the stored 12-D robot state unchanged
while giving the policy a separate 6-D force input.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
from typing import Any

import cv2
import numpy as np
import pyarrow.parquet as parquet


EXPECTED_LEROBOT_VERSION = "0.6.2"
TASK_PROMPT = "Pick up the bottle and place it on the cardboard box."
STATE_KEY = "observation.state"
FORCE_KEY = "observation.environment_state"
ACTION_KEY = "action"
WORKSPACE_KEY = "observation.images.workspace"
WRIST_KEY = "observation.images.wrist"
STATE_NAMES = (
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
FORCE_NAMES = (
    "rh56_force_index",
    "rh56_force_middle",
    "rh56_force_ring",
    "rh56_force_pinky",
    "rh56_force_thumb_close",
    "rh56_force_thumb_lateral",
)
ACTION_NAMES = (
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


def _check_version() -> None:
    version = importlib.metadata.version("lerobot")
    if version != EXPECTED_LEROBOT_VERSION:
        raise RuntimeError(
            f"this repository integration requires LeRobot {EXPECTED_LEROBOT_VERSION}; "
            f"found {version}"
        )


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _tree_hash(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(path for path in root.rglob("*") if path.is_file()):
        digest.update(str(path.relative_to(root)).encode("utf-8"))
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def _records(master: Path) -> list[dict[str, Any]]:
    path = master / "meta/episodes.jsonl"
    if not path.is_file():
        raise ValueError(f"missing master episode index: {path}")
    records = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not records:
        raise ValueError("master contains no logical episodes")
    return records


def _rows(master: Path, record: dict[str, Any]) -> list[dict[str, Any]]:
    rows = parquet.read_table(master / record["data"]).to_pylist()
    episode_index = int(record["episode_index"])
    if len(rows) != int(record["length"]):
        raise ValueError(f"episode {episode_index}: parquet length mismatch")
    if [int(row["frame_index"]) for row in rows] != list(range(len(rows))):
        raise ValueError(f"episode {episode_index}: frame_index is not contiguous")
    source_frames = [int(row["source_frame_index"]) for row in rows]
    if any(right != left + 1 for left, right in zip(source_frames, source_frames[1:])):
        raise ValueError(f"episode {episode_index}: source frame indices are not contiguous")
    for row in rows:
        if np.asarray(row[STATE_KEY], dtype=np.float32).shape != (12,):
            raise ValueError(f"episode {episode_index}: state shape is not 12")
        if np.asarray(row[ACTION_KEY], dtype=np.float32).shape != (12,):
            raise ValueError(f"episode {episode_index}: action shape is not 12")
        if np.asarray(row["observation.force"], dtype=np.float32).shape != (6,):
            raise ValueError(f"episode {episode_index}: force shape is not 6")
        if not np.isfinite(np.asarray(row[STATE_KEY], dtype=np.float32)).all():
            raise ValueError(f"episode {episode_index}: state is non-finite")
        if not np.isfinite(np.asarray(row[ACTION_KEY], dtype=np.float32)).all():
            raise ValueError(f"episode {episode_index}: action is non-finite")
        if not np.isfinite(np.asarray(row["observation.force"], dtype=np.float32)).all():
            raise ValueError(f"episode {episode_index}: force is non-finite")
    return rows


def _video_path(master: Path, record: dict[str, Any], role: str) -> Path:
    return master / record["videos"][role]


def _validation_source_episodes(args: argparse.Namespace) -> set[int]:
    values: list[int] = list(args.validation_source_episodes or [])
    if args.split_config is not None:
        import yaml

        payload = yaml.safe_load(args.split_config.read_text(encoding="utf-8"))
        configured = payload.get("validation_source_episodes", [])
        if values and set(values) != {int(value) for value in configured}:
            raise ValueError("validation episodes were specified twice with different values")
        values = [int(value) for value in configured]
    result = {int(value) for value in values}
    if any(value < 0 for value in result):
        raise ValueError("validation source episode ids must be non-negative")
    return result


def _ordered_records(master: Path, validation_source_episodes: set[int]) -> list[dict[str, Any]]:
    records = _records(master)
    if not validation_source_episodes:
        return records
    present = {int(record["source_episode_index"]) for record in records}
    missing = validation_source_episodes - present
    if missing:
        raise ValueError(f"validation source episodes are absent from master: {sorted(missing)}")
    train = [
        record
        for record in records
        if int(record["source_episode_index"]) not in validation_source_episodes
    ]
    validation = [
        record
        for record in records
        if int(record["source_episode_index"]) in validation_source_episodes
    ]
    return train + validation


def _numeric_stats(values: list[np.ndarray]) -> dict[str, list[float] | list[int]]:
    data = np.concatenate(values, axis=0).astype(np.float64, copy=False)
    return {
        "min": data.min(axis=0).tolist(),
        "max": data.max(axis=0).tolist(),
        "mean": data.mean(axis=0).tolist(),
        "std": data.std(axis=0).tolist(),
        "count": [int(data.shape[0])],
        "q01": np.quantile(data, 0.01, axis=0).tolist(),
        "q10": np.quantile(data, 0.10, axis=0).tolist(),
        "q50": np.quantile(data, 0.50, axis=0).tolist(),
        "q90": np.quantile(data, 0.90, axis=0).tolist(),
        "q99": np.quantile(data, 0.99, axis=0).tolist(),
    }


def _features(height: int, width: int, *, include_force: bool) -> dict[str, dict[str, Any]]:
    image = {
        "dtype": "video",
        "shape": (height, width, 3),
        "names": ("height", "width", "channel"),
    }
    features: dict[str, dict[str, Any]] = {
        WORKSPACE_KEY: image,
        WRIST_KEY: image,
        STATE_KEY: {"dtype": "float32", "shape": (12,), "names": STATE_NAMES},
        ACTION_KEY: {"dtype": "float32", "shape": (12,), "names": ACTION_NAMES},
        "source_episode_index": {
            "dtype": "int64",
            "shape": (1,),
            "names": ("source_episode_index",),
        },
        "source_frame_index": {
            "dtype": "int64",
            "shape": (1,),
            "names": ("source_frame_index",),
        },
        "segment_id": {
            "dtype": "int64",
            "shape": (1,),
            "names": ("segment_id",),
        },
    }
    if include_force:
        features[FORCE_KEY] = {
            "dtype": "float32",
            "shape": (6,),
            "names": FORCE_NAMES,
        }
    return features


def build_view(args: argparse.Namespace) -> None:
    _check_version()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    master = args.master.resolve()
    view = args.view.resolve()
    if view.exists():
        if not args.replace:
            raise FileExistsError(
                f"derived LeRobot view already exists: {view}; use --replace only for this generated root"
            )
        shutil.rmtree(view)
    validation_source_episodes = _validation_source_episodes(args)
    records = _ordered_records(master, validation_source_episodes)
    source_hash_before = _tree_hash(master)
    include_force = args.kind == "act_force"
    train_values: dict[str, list[np.ndarray]] = {
        STATE_KEY: [],
        ACTION_KEY: [],
    }
    if include_force:
        train_values[FORCE_KEY] = []
    dataset = LeRobotDataset.create(
        repo_id=(
            "local/physical_bottle_v2_act_force"
            if include_force
            else "local/physical_bottle_v2_act"
        ),
        fps=30,
        features=_features(args.height, args.width, include_force=include_force),
        root=view,
        robot_type="jaka_mini2_rh56dfx",
        use_videos=True,
        video_backend="pyav",
        image_writer_threads=args.image_writer_threads,
        encoder_threads=args.encoder_threads,
    )
    episode_map: list[dict[str, Any]] = []
    for internal_episode, record in enumerate(records):
        rows = _rows(master, record)
        is_validation = int(record["source_episode_index"]) in validation_source_episodes
        if not is_validation:
            train_values[STATE_KEY].append(
                np.asarray([row[STATE_KEY] for row in rows], dtype=np.float64)
            )
            train_values[ACTION_KEY].append(
                np.asarray([row[ACTION_KEY] for row in rows], dtype=np.float64)
            )
            if include_force:
                train_values[FORCE_KEY].append(
                    np.asarray([row["observation.force"] for row in rows], dtype=np.float64)
                )
        captures = {
            role: cv2.VideoCapture(str(_video_path(master, record, role)))
            for role in ("workspace", "wrist")
        }
        try:
            for role, capture in captures.items():
                if not capture.isOpened():
                    raise RuntimeError(f"cannot open {role} video for episode {record['episode_index']}")
                frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
                if frame_count != len(rows):
                    raise ValueError(
                        f"episode {record['episode_index']} {role} frame count {frame_count} != {len(rows)}"
                    )
            for frame_index, row in enumerate(rows):
                images: dict[str, np.ndarray] = {}
                for role, capture in captures.items():
                    ok, bgr = capture.read()
                    if not ok or bgr is None:
                        raise RuntimeError(
                            f"cannot decode {role} episode {record['episode_index']} frame {frame_index}"
                        )
                    resized = cv2.resize(
                        bgr, (args.width, args.height), interpolation=cv2.INTER_AREA
                    )
                    images[role] = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                frame: dict[str, Any] = {
                    WORKSPACE_KEY: images["workspace"],
                    WRIST_KEY: images["wrist"],
                    STATE_KEY: np.asarray(row[STATE_KEY], dtype=np.float32),
                    ACTION_KEY: np.asarray(row[ACTION_KEY], dtype=np.float32),
                    "source_episode_index": np.asarray(
                        [int(row["source_episode_index"])], dtype=np.int64
                    ),
                    "source_frame_index": np.asarray(
                        [int(row["source_frame_index"])], dtype=np.int64
                    ),
                    "segment_id": np.asarray([0], dtype=np.int64),
                    "task": TASK_PROMPT,
                }
                if include_force:
                    frame[FORCE_KEY] = np.asarray(
                        row["observation.force"], dtype=np.float32
                    )
                dataset.add_frame(frame)
        finally:
            for capture in captures.values():
                capture.release()
        dataset.save_episode(parallel_encoding=True)
        episode_map.append(
            {
                "lerobot_episode_index": internal_episode,
                "logical_segment_id": record.get("logical_segment_id"),
                "source_episode_index": int(record["source_episode_index"]),
                "length": len(rows),
                "split": "val" if is_validation else "train",
            }
        )
    dataset.finalize()
    if validation_source_episodes:
        stats_path = view / "meta/stats.json"
        stats = json.loads(stats_path.read_text(encoding="utf-8"))
        stats[STATE_KEY] = _numeric_stats(train_values[STATE_KEY])
        stats[ACTION_KEY] = _numeric_stats(train_values[ACTION_KEY])
        if include_force:
            stats[FORCE_KEY] = _numeric_stats(train_values[FORCE_KEY])
        _write_json(stats_path, stats)
    source_hash_after = _tree_hash(master)
    if source_hash_before != source_hash_after:
        raise RuntimeError("source master changed while building the derived LeRobot view")
    provenance = {
        "schema_version": "embodied_lab.lerobot_act_view.v2",
        "source_master": str(master),
        "source_master_sha256_tree": source_hash_before,
        "derived_view": str(view),
        "lerobot_version": EXPECTED_LEROBOT_VERSION,
        "view_kind": args.kind,
        "source_features": {
            "observation.state": {"shape": [12], "names": list(STATE_NAMES)},
            "observation.force": {"shape": [6], "names": list(FORCE_NAMES)},
            "action": {"shape": [12], "names": list(ACTION_NAMES)},
        },
        "policy_features": {
            "observation.state": {
                "shape": [12],
                "names": list(STATE_NAMES),
            },
            "observation.environment_state": (
                {"shape": [6], "names": list(FORCE_NAMES)} if include_force else None
            ),
            "action": {"shape": [12], "names": list(ACTION_NAMES)},
        },
        "force_units": "rh56_force_act_raw_count",
        "force_model_mapping": (
            "observation.force -> observation.environment_state; ACT native environment-state token"
            if include_force
            else "not included in standard ACT view"
        ),
        "force_age_and_validity": "retained in source master; not policy inputs in this baseline",
        "image_resize": "OpenCV INTER_AREA from 640x480 to 320x240; BGR decoder output converted once to RGB",
        "action_semantics": "absolute_native_target",
        "episode_map": episode_map,
        "validation_source_episodes": sorted(validation_source_episodes),
        "normalization_stats_scope": (
            "train logical segments only" if validation_source_episodes else "all logical segments"
        ),
    }
    _write_json(view / "meta/embodied_lab_provenance.json", provenance)
    print(json.dumps(provenance, indent=2, ensure_ascii=False))


def validate_view(args: argparse.Namespace) -> None:
    _check_version()
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    view = args.view.resolve()
    provenance = json.loads(
        (view / "meta/embodied_lab_provenance.json").read_text(encoding="utf-8")
    )
    source_master = Path(provenance["source_master"])
    source_records = _records(source_master)
    source_by_segment = {
        str(record["logical_segment_id"]): record for record in source_records
    }
    episode_map = provenance.get("episode_map")
    if episode_map:
        records = [source_by_segment[str(item["logical_segment_id"])] for item in episode_map]
    else:
        records = source_records
    include_force = provenance["view_kind"] == "act_force"
    expected_state = 12
    expected_env = 6 if include_force else None
    dataset = LeRobotDataset(
        "local/physical_bottle_v2_act_force" if include_force else "local/physical_bottle_v2_act",
        root=view,
        delta_timestamps={ACTION_KEY: [i / 30 for i in range(args.chunk_size)]},
        video_backend="pyav",
        return_uint8=True,
    )
    expected_rows = sum(int(record["length"]) for record in records)
    if len(dataset) != expected_rows or dataset.num_episodes != len(records):
        raise ValueError(
            f"LeRobot loader got {len(dataset)} rows/{dataset.num_episodes} episodes; "
            f"expected {expected_rows}/{len(records)}"
        )
    representative = sorted({0, len(dataset) // 2, len(dataset) - 1})
    samples: list[dict[str, Any]] = []
    for index in representative:
        sample = dataset[index]
        if tuple(sample[STATE_KEY].shape) != (expected_state,):
            raise ValueError(f"state shape mismatch at dataset index {index}")
        if tuple(sample[ACTION_KEY].shape) != (args.chunk_size, 12):
            raise ValueError(f"action chunk shape mismatch at dataset index {index}")
        if include_force:
            if tuple(sample[FORCE_KEY].shape) != (expected_env,):
                raise ValueError(f"force shape mismatch at dataset index {index}")
        if sample[WORKSPACE_KEY].shape[0] != 3 or sample[WRIST_KEY].shape[0] != 3:
            raise ValueError(f"camera shape mismatch at dataset index {index}")
        if not torch.isfinite(sample[STATE_KEY]).all() or not torch.isfinite(sample[ACTION_KEY]).all():
            raise ValueError(f"non-finite model input/output at dataset index {index}")
        samples.append(
            {
                "dataset_index": index,
                "episode_index": int(sample["episode_index"]),
                "source_episode_index": int(sample["source_episode_index"]),
                "source_frame_index": int(sample["source_frame_index"]),
                "state_shape": list(sample[STATE_KEY].shape),
                "action_shape": list(sample[ACTION_KEY].shape),
                "force_shape": list(sample[FORCE_KEY].shape) if include_force else None,
                "valid_action_count": int((~sample[f"{ACTION_KEY}_is_pad"]).sum()),
            }
        )
    # Every logical episode end must be padded without crossing to the next one.
    offset = 0
    for record in records:
        last = dataset[offset + int(record["length"]) - 1]
        valid_count = int((~last[f"{ACTION_KEY}_is_pad"]).sum())
        if valid_count != 1:
            raise ValueError(f"episode {record['episode_index']} end has {valid_count} valid actions")
        if not torch.equal(last[ACTION_KEY], last[ACTION_KEY][0].expand_as(last[ACTION_KEY])):
            raise ValueError(f"episode {record['episode_index']} end action padding is not repeat-last")
        offset += int(record["length"])
    report = {
        "status": "PASS",
        "lerobot_version": EXPECTED_LEROBOT_VERSION,
        "view": str(view),
        "view_kind": provenance["view_kind"],
        "rows": len(dataset),
        "episodes": dataset.num_episodes,
        "state_shape": [12],
        "force_shape": [6] if include_force else None,
        "action_chunk_shape": [args.chunk_size, 12],
        "action_semantics": "absolute_native_target",
        "representative_samples": samples,
        "boundary_check": "PASS",
    }
    _write_json(view / "meta/embodied_lab_loader_validation.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))


def evaluate_checkpoint(args: argparse.Namespace) -> None:
    """Run deterministic teacher-forced inference on the episode-level val split."""

    _check_version()
    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    view = args.view.resolve()
    checkpoint = args.checkpoint.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    provenance = json.loads(
        (view / "meta/embodied_lab_provenance.json").read_text(encoding="utf-8")
    )
    episode_map = provenance.get("episode_map", [])
    validation = [record for record in episode_map if record.get("split") == "val"]
    if not validation:
        raise ValueError("derived view has no episode-level validation split")
    policy_config = PreTrainedConfig.from_pretrained(checkpoint)
    policy_config.device = "cuda" if torch.cuda.is_available() else "cpu"
    if int(policy_config.chunk_size) != args.chunk_size:
        raise ValueError(
            f"checkpoint chunk size {policy_config.chunk_size} != requested {args.chunk_size}"
        )
    policy = ACTPolicy.from_pretrained(checkpoint, config=policy_config).to(
        policy_config.device
    )
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_config,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={
            "device_processor": {"device": policy_config.device}
        },
    )
    dataset = LeRobotDataset(
        "local/physical_bottle_v2_act",
        root=view,
        delta_timestamps={ACTION_KEY: [index / 30 for index in range(args.chunk_size)]},
        video_backend="pyav",
        return_uint8=True,
    )
    offsets: list[tuple[int, int, dict[str, Any]]] = []
    offset = 0
    for record in episode_map:
        length = int(record["length"])
        if record.get("split") == "val":
            offsets.append((offset, offset + length, record))
        offset += length
    indices = [index for start, end, _ in offsets for index in range(start, end)]
    subset = torch.utils.data.Subset(dataset, indices)
    loader = torch.utils.data.DataLoader(
        subset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
    )
    predictions: list[np.ndarray] = []
    ground_truth: list[np.ndarray] = []
    valid: list[np.ndarray] = []
    source_episode: list[np.ndarray] = []
    source_frame: list[np.ndarray] = []
    policy.eval()
    torch.manual_seed(0)
    with torch.inference_mode():
        for batch in loader:
            ground_truth.append(batch[ACTION_KEY].numpy())
            valid.append((~batch[f"{ACTION_KEY}_is_pad"]).numpy())
            source_episode.append(batch["source_episode_index"].numpy().reshape(-1))
            source_frame.append(batch["source_frame_index"].numpy().reshape(-1))
            for key in (WORKSPACE_KEY, WRIST_KEY):
                batch[key] = batch[key].float() / 255.0
            processed = preprocessor(batch)
            native = postprocessor(policy.predict_action_chunk(processed))
            predictions.append(native.detach().cpu().numpy())
    prediction_array = np.concatenate(predictions)
    ground_truth_array = np.concatenate(ground_truth)
    valid_array = np.concatenate(valid)
    episode_array = np.concatenate(source_episode).astype(np.int64, copy=False)
    frame_array = np.concatenate(source_frame).astype(np.int64, copy=False)
    if prediction_array.shape != (len(indices), args.chunk_size, 12):
        raise ValueError(f"invalid checkpoint output shape {prediction_array.shape}")
    if not np.isfinite(prediction_array).all():
        raise ValueError("checkpoint produced non-finite actions")
    np.savez_compressed(
        output / "teacher_forced_arrays.npz",
        predictions=prediction_array,
        ground_truth=ground_truth_array,
        valid=valid_array,
        source_episode=episode_array,
        source_frame=frame_array,
    )
    error = np.abs(prediction_array - ground_truth_array)
    report = {
        "schema_version": "embodied_lab.act_teacher_forced_replay.v1",
        "checkpoint": str(checkpoint),
        "view": str(view),
        "device": policy_config.device,
        "validation_source_episodes": sorted(np.unique(episode_array).tolist()),
        "rows": len(indices),
        "output_shape": list(prediction_array.shape),
        "finite": True,
        "first_action_mae": {
            "jaka": float(np.mean(error[:, 0, :6])),
            "rh56": float(np.mean(error[:, 0, 6:])),
        },
        "arrays": str(output / "teacher_forced_arrays.npz"),
    }
    _write_json(output / "teacher_forced_replay.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build-view")
    build.add_argument("--master", type=Path, required=True)
    build.add_argument("--view", type=Path, required=True)
    build.add_argument("--kind", choices=("act", "act_force"), required=True)
    build.add_argument("--width", type=int, default=320)
    build.add_argument("--height", type=int, default=240)
    build.add_argument("--image-writer-threads", type=int, default=8)
    build.add_argument("--encoder-threads", type=int, default=4)
    build.add_argument("--split-config", type=Path)
    build.add_argument("--validation-source-episodes", type=int, nargs="+")
    build.add_argument("--replace", action="store_true")
    build.set_defaults(function=build_view)

    validate = subparsers.add_parser("validate-view")
    validate.add_argument("--view", type=Path, required=True)
    validate.add_argument("--chunk-size", type=int, default=16)
    validate.set_defaults(function=validate_view)

    evaluate = subparsers.add_parser("evaluate-checkpoint")
    evaluate.add_argument("--view", type=Path, required=True)
    evaluate.add_argument("--checkpoint", type=Path, required=True)
    evaluate.add_argument("--output", type=Path, required=True)
    evaluate.add_argument("--chunk-size", type=int, default=16)
    evaluate.add_argument("--batch-size", type=int, default=64)
    evaluate.add_argument("--num-workers", type=int, default=2)
    evaluate.set_defaults(function=evaluate_checkpoint)
    return parser


def main() -> None:
    args = make_parser().parse_args()
    args.function(args)


if __name__ == "__main__":
    main()
