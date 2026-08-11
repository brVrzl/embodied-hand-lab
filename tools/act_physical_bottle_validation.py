#!/usr/bin/env python3
"""Offline LeRobot ACT validation for the materialized physical bottle dataset.

This tool never imports or calls the robot/teleoperation stack.  It builds a
trainer-compatible LeRobot v3 view under an experiment directory and validates
or evaluates that derived view while treating the master dataset as immutable.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re
from typing import Any, Iterable

import cv2
import numpy as np
import pyarrow.parquet as pq


EXPECTED_SOURCE_EPISODES = (65, 66, 67, 69, 70)
EXCLUDED_SOURCE_EPISODES = (68, 71)
STATE_KEY = "observation.state"
ACTION_KEY = "action"
WORKSPACE_KEY = "observation.images.workspace"
WRIST_KEY = "observation.images.wrist"
TASK_PROMPT = "Pick up the bottle and place it on the cardboard box."
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


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _episode_records(master: Path) -> list[dict[str, Any]]:
    records = [
        json.loads(line)
        for line in (master / "meta" / "episodes.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    source_ids = tuple(int(record["episode_index"]) for record in records)
    if source_ids != EXPECTED_SOURCE_EPISODES:
        raise ValueError(f"expected source episodes {EXPECTED_SOURCE_EPISODES}, got {source_ids}")
    return records


def _read_rows(master: Path, record: dict[str, Any]) -> list[dict[str, Any]]:
    rows = pq.read_table(master / record["data"]).to_pylist()
    source_episode = int(record["episode_index"])
    if len(rows) != int(record["length"]):
        raise ValueError(f"episode {source_episode} row count mismatch")
    if {int(row["episode_index"]) for row in rows} != {source_episode}:
        raise ValueError(f"episode {source_episode} parquet contains another episode")
    if [int(row["frame_index"]) for row in rows] != list(range(len(rows))):
        raise ValueError(f"episode {source_episode} frame_index is not contiguous")
    if [int(row["raw_frame_index"]) for row in rows] != list(range(len(rows))):
        raise ValueError(f"episode {source_episode} raw_frame_index is not contiguous")
    if set(int(row.get("segment_id", 0)) for row in rows) != {0}:
        raise ValueError(f"episode {source_episode} has unexpected segment structure")
    return rows


def _master_files_fingerprint(master: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(p for p in master.rglob("*") if p.is_file()):
        digest.update(str(path.relative_to(master)).encode())
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest()


def _video_path(master: Path, source_episode: int, role: str) -> Path:
    return (
        master
        / "videos"
        / f"observation.images.{role}"
        / "chunk-000"
        / f"episode_{source_episode:06d}.mp4"
    )


def _read_rgb_frame(path: Path, frame_index: int, *, width: int, height: int) -> np.ndarray:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {path}")
    capture.set(cv2.CAP_PROP_POS_FRAMES, float(frame_index))
    ok, bgr = capture.read()
    capture.release()
    if not ok or bgr is None:
        raise RuntimeError(f"cannot read frame {frame_index} from {path}")
    if (bgr.shape[1], bgr.shape[0]) != (width, height):
        bgr = cv2.resize(bgr, (width, height), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def build_view(args: argparse.Namespace) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    master = args.master.resolve()
    view = args.view.resolve()
    if view.exists():
        raise FileExistsError(f"refusing to overwrite existing derived view: {view}")
    records = _episode_records(master)
    master_hash_before = _master_files_fingerprint(master)
    features = {
        WORKSPACE_KEY: {
            "dtype": "video",
            "shape": (args.height, args.width, 3),
            "names": ("height", "width", "channel"),
        },
        WRIST_KEY: {
            "dtype": "video",
            "shape": (args.height, args.width, 3),
            "names": ("height", "width", "channel"),
        },
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
        "segment_id": {"dtype": "int64", "shape": (1,), "names": ("segment_id",)},
    }
    dataset = LeRobotDataset.create(
        repo_id="local/physical_bottle_5demo_act_validation",
        fps=30,
        features=features,
        root=view,
        robot_type="jaka_mini2_rh56dfx",
        use_videos=True,
        video_backend="pyav",
        image_writer_threads=args.image_writer_threads,
        encoder_threads=args.encoder_threads,
    )
    source_map: list[dict[str, int]] = []
    for internal_episode, record in enumerate(records):
        source_episode = int(record["episode_index"])
        rows = _read_rows(master, record)
        captures = {
            role: cv2.VideoCapture(str(_video_path(master, source_episode, role)))
            for role in ("workspace", "wrist")
        }
        try:
            for role, capture in captures.items():
                if not capture.isOpened():
                    raise RuntimeError(f"cannot open source {role} video for episode {source_episode}")
                if int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) != len(rows):
                    raise ValueError(f"{role} frame count mismatch for episode {source_episode}")
            for source_frame, row in enumerate(rows):
                images: dict[str, np.ndarray] = {}
                for role, capture in captures.items():
                    ok, bgr = capture.read()
                    if not ok or bgr is None:
                        raise RuntimeError(
                            f"cannot read {role} episode {source_episode} frame {source_frame}"
                        )
                    resized = cv2.resize(
                        bgr, (args.width, args.height), interpolation=cv2.INTER_AREA
                    )
                    images[role] = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                state = np.asarray(row[STATE_KEY], dtype=np.float32)
                action = np.asarray(row[ACTION_KEY], dtype=np.float32)
                if state.shape != (12,) or action.shape != (12,):
                    raise ValueError(f"invalid state/action shape at episode {source_episode} frame {source_frame}")
                dataset.add_frame(
                    {
                        WORKSPACE_KEY: images["workspace"],
                        WRIST_KEY: images["wrist"],
                        STATE_KEY: state,
                        ACTION_KEY: action,
                        "source_episode_index": np.asarray([source_episode], dtype=np.int64),
                        "source_frame_index": np.asarray([source_frame], dtype=np.int64),
                        "segment_id": np.asarray([int(row.get("segment_id", 0))], dtype=np.int64),
                        "task": TASK_PROMPT,
                    }
                )
        finally:
            for capture in captures.values():
                capture.release()
        dataset.save_episode(parallel_encoding=True)
        source_map.append(
            {
                "lerobot_episode_index": internal_episode,
                "source_episode_index": source_episode,
                "length": len(rows),
            }
        )
    dataset.finalize()
    master_hash_after = _master_files_fingerprint(master)
    if master_hash_after != master_hash_before:
        raise RuntimeError("master dataset fingerprint changed while building derived view")
    provenance = {
        "schema_version": "embodied_lab.lerobot_act_view.v1",
        "source_master": str(master),
        "source_master_sha256_tree": master_hash_before,
        "source_episodes": list(EXPECTED_SOURCE_EPISODES),
        "excluded_source_episodes": list(EXCLUDED_SOURCE_EPISODES),
        "source_resolution_hwc": [480, 640, 3],
        "derived_resolution_hwc": [args.height, args.width, 3],
        "resize": "OpenCV INTER_AREA; BGR decoder output converted explicitly to RGB",
        "action_semantics": "absolute_native_target",
        "action_order": list(ACTION_NAMES),
        "state_order": list(STATE_NAMES),
        "standard_act_force_included": False,
        "episode_map": source_map,
    }
    _write_json(view / "meta" / "embodied_lab_provenance.json", provenance)
    print(json.dumps(provenance, indent=2))


def _source_offset_map(records: Iterable[dict[str, Any]]) -> tuple[list[int], dict[int, int]]:
    lengths = [int(record["length"]) for record in records]
    starts: dict[int, int] = {}
    offset = 0
    for source_episode, length in zip(EXPECTED_SOURCE_EPISODES, lengths, strict=True):
        starts[source_episode] = offset
        offset += length
    return lengths, starts


def _array_stats(values: np.ndarray) -> dict[str, list[float] | int]:
    return {
        "count": int(values.shape[0]),
        "mean": values.mean(axis=0).tolist(),
        "std": values.std(axis=0).tolist(),
        "min": values.min(axis=0).tolist(),
        "max": values.max(axis=0).tolist(),
    }


def inspect_loader(args: argparse.Namespace) -> None:
    import matplotlib.pyplot as plt
    import torch
    from lerobot.configs import FeatureType
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.processor_act import make_act_pre_post_processors
    from lerobot.utils.feature_utils import dataset_to_policy_features

    master = args.master.resolve()
    view = args.view.resolve()
    records = _episode_records(master)
    lengths, starts = _source_offset_map(records)
    delta_timestamps = {ACTION_KEY: [i / 30 for i in range(args.chunk_size)]}
    dataset = LeRobotDataset(
        "local/physical_bottle_5demo_act_validation",
        root=view,
        delta_timestamps=delta_timestamps,
        video_backend="pyav",
        return_uint8=True,
    )
    if len(dataset) != 3268 or dataset.num_episodes != 5:
        raise ValueError(f"real loader got {len(dataset)} frames/{dataset.num_episodes} episodes")
    loaded_sources = sorted(
        int(value) for value in dataset.hf_dataset.unique("source_episode_index")
    )
    if loaded_sources != list(EXPECTED_SOURCE_EPISODES):
        raise ValueError(f"real loader source episodes mismatch: {loaded_sources}")
    if any(source in loaded_sources for source in EXCLUDED_SOURCE_EPISODES):
        raise ValueError("excluded source episode reached real loader")

    representative = {
        "episode_65_first": starts[65],
        "dataset_middle": len(dataset) // 2,
        "episode_65_last": starts[65] + lengths[0] - 1,
        "episode_66_first_boundary": starts[66],
        "dataset_last": len(dataset) - 1,
    }
    sample_reports: dict[str, Any] = {}
    contact_rows: list[tuple[str, np.ndarray, np.ndarray]] = []
    for label, index in representative.items():
        sample = dataset[index]
        state = sample[STATE_KEY]
        action = sample[ACTION_KEY]
        pad = sample[f"{ACTION_KEY}_is_pad"]
        if tuple(state.shape) != (12,) or tuple(action.shape) != (args.chunk_size, 12):
            raise ValueError(f"shape mismatch for {label}")
        source_episode = int(sample["source_episode_index"].item())
        source_frame = int(sample["source_frame_index"].item())
        expected_real = min(args.chunk_size, lengths[EXPECTED_SOURCE_EPISODES.index(source_episode)] - source_frame)
        if int((~pad).sum().item()) != expected_real:
            raise ValueError(f"padding mismatch for {label}")
        if bool(pad.any()):
            final_real = action[expected_real - 1]
            if not torch.equal(action[expected_real:], final_real.expand_as(action[expected_real:])):
                raise ValueError(f"padding is not repeat-last for {label}")
        source_rows = _read_rows(master, records[EXPECTED_SOURCE_EPISODES.index(source_episode)])
        expected_actions = np.asarray(
            [
                source_rows[min(source_frame + horizon, len(source_rows) - 1)][ACTION_KEY]
                for horizon in range(args.chunk_size)
            ],
            dtype=np.float32,
        )
        if not np.array_equal(action.numpy(), expected_actions):
            raise ValueError(f"action sequence changed for {label}")

        image_metrics: dict[str, Any] = {}
        loaded_images: dict[str, np.ndarray] = {}
        refs: dict[str, np.ndarray] = {}
        for role, key in (("workspace", WORKSPACE_KEY), ("wrist", WRIST_KEY)):
            loaded = sample[key].permute(1, 2, 0).numpy()
            reference = _read_rgb_frame(
                _video_path(master, source_episode, role),
                source_frame,
                width=args.width,
                height=args.height,
            )
            loaded_images[role] = loaded
            refs[role] = reference
            rgb_mse = float(np.mean((loaded.astype(np.float32) - reference.astype(np.float32)) ** 2))
            bgr_mse = float(
                np.mean((loaded.astype(np.float32) - reference[..., ::-1].astype(np.float32)) ** 2)
            )
            image_metrics[role] = {"rgb_reference_mse": rgb_mse, "bgr_reference_mse": bgr_mse}
            if not rgb_mse < bgr_mse:
                raise ValueError(f"{role} appears BGR-swapped for {label}")
        workspace_swapped_mse = float(
            np.mean(
                (loaded_images["workspace"].astype(np.float32) - refs["wrist"].astype(np.float32)) ** 2
            )
        )
        wrist_swapped_mse = float(
            np.mean(
                (loaded_images["wrist"].astype(np.float32) - refs["workspace"].astype(np.float32)) ** 2
            )
        )
        if image_metrics["workspace"]["rgb_reference_mse"] >= workspace_swapped_mse:
            raise ValueError(f"workspace camera identity mismatch for {label}")
        if image_metrics["wrist"]["rgb_reference_mse"] >= wrist_swapped_mse:
            raise ValueError(f"wrist camera identity mismatch for {label}")
        image_metrics["workspace_swapped_reference_mse"] = workspace_swapped_mse
        image_metrics["wrist_swapped_reference_mse"] = wrist_swapped_mse
        sample_reports[label] = {
            "dataset_index": index,
            "source_episode_index": source_episode,
            "source_frame_index": source_frame,
            "state_shape": list(state.shape),
            "action_shape": list(action.shape),
            "valid_actions": expected_real,
            "padding": "repeat_last with action_is_pad=True",
            "image_metrics": image_metrics,
        }
        contact_rows.append((label, loaded_images["workspace"], loaded_images["wrist"]))

    loader = torch.utils.data.DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0)
    batch = next(iter(loader))
    if tuple(batch[STATE_KEY].shape) != (4, 12):
        raise ValueError("batch state shape mismatch")
    if tuple(batch[ACTION_KEY].shape) != (4, args.chunk_size, 12):
        raise ValueError("batch action shape mismatch")

    states: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    forces: list[np.ndarray] = []
    for record in records:
        for row in _read_rows(master, record):
            states.append(np.asarray(row[STATE_KEY], dtype=np.float64))
            actions.append(np.asarray(row[ACTION_KEY], dtype=np.float64))
            force = np.asarray(row["observation.force"], dtype=np.float64)
            if force.shape != (6,):
                raise ValueError("force readiness shape mismatch")
            forces.append(force)
    state_values = np.stack(states)
    action_values = np.stack(actions)

    features = dataset_to_policy_features(dataset.meta.features)
    config = ACTConfig(
        chunk_size=args.chunk_size,
        n_action_steps=args.chunk_size,
        device="cpu",
        pretrained_backbone_weights=None,
    )
    config.output_features = {
        key: feature for key, feature in features.items() if feature.type is FeatureType.ACTION
    }
    config.input_features = {
        key: feature for key, feature in features.items() if feature.type is not FeatureType.ACTION
    }
    preprocessor, postprocessor = make_act_pre_post_processors(config, dataset.meta.stats)
    raw = dataset[starts[65] + 100]
    trace_input = {
        key: value.clone() if isinstance(value, torch.Tensor) else value
        for key, value in raw.items()
    }
    for key in (WORKSPACE_KEY, WRIST_KEY):
        trace_input[key] = trace_input[key].float() / 255.0
    transformed = preprocessor(trace_input)
    normalized_action = transformed[ACTION_KEY]
    reconstructed = postprocessor(normalized_action)
    raw_action = raw[ACTION_KEY]
    roundtrip_error = torch.max(torch.abs(reconstructed - raw_action)).item()
    if roundtrip_error > 1e-5:
        raise ValueError(f"ACT action normalization round-trip error {roundtrip_error}")

    fig, axes = plt.subplots(len(contact_rows), 2, figsize=(8, 2.7 * len(contact_rows)))
    for row_index, (label, workspace, wrist) in enumerate(contact_rows):
        axes[row_index, 0].imshow(workspace)
        axes[row_index, 0].set_title(f"{label}: workspace RGB")
        axes[row_index, 1].imshow(wrist)
        axes[row_index, 1].set_title(f"{label}: wrist RGB")
        axes[row_index, 0].axis("off")
        axes[row_index, 1].axis("off")
    fig.tight_layout()
    plot_path = args.output / "loader_camera_contact_sheet.png"
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=140)
    plt.close(fig)

    report = {
        "status": "PASS",
        "real_loader": "lerobot.datasets.lerobot_dataset.LeRobotDataset",
        "dataset_frames": len(dataset),
        "dataset_episodes": dataset.num_episodes,
        "source_episodes": loaded_sources,
        "excluded_episodes_absent": True,
        "chunk_size": args.chunk_size,
        "delta_timestamps_s": delta_timestamps[ACTION_KEY],
        "image_shape_chw": list(batch[WORKSPACE_KEY].shape[1:]),
        "image_dtype": str(batch[WORKSPACE_KEY].dtype),
        "batch_size_checked": 4,
        "batch_state_shape": list(batch[STATE_KEY].shape),
        "batch_action_shape": list(batch[ACTION_KEY].shape),
        "episode_and_segment_boundary_check": "PASS (all source segments are 0; LeRobot clamps at episode end)",
        "representative_samples": sample_reports,
        "state_names": list(STATE_NAMES),
        "action_names": list(ACTION_NAMES),
        "normalization_mode": {"state": "MEAN_STD", "action": "MEAN_STD", "visual": "MEAN_STD"},
        "state_stats": _array_stats(state_values),
        "action_stats": _array_stats(action_values),
        "near_zero_std_dimensions": {
            "state": np.flatnonzero(state_values.std(0) < 1e-6).tolist(),
            "action": np.flatnonzero(action_values.std(0) < 1e-6).tolist(),
        },
        "action_semantics": "absolute_native_target; no delta conversion and no reordering",
        "normalization_trace": {
            "source_episode": 65,
            "source_frame": 100,
            "raw_action_first": raw_action[0].tolist(),
            "transformed_action_first": normalized_action[0].detach().cpu().tolist(),
            "reconstructed_action_first": reconstructed[0].detach().cpu().tolist(),
            "max_abs_roundtrip_error": roundtrip_error,
            "status": "PASS",
        },
        "force_readiness": {
            "status": "PASS",
            "shape": list(np.stack(forces).shape),
            "same_row_indices": True,
            "standard_act_view_contains_force": False,
        },
        "camera_contact_sheet": str(plot_path),
    }
    _write_json(args.output / "loader_and_normalization_validation.json", report)
    print(json.dumps(report, indent=2))


def _checkpoint_objects(checkpoint: Path):
    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    config = PreTrainedConfig.from_pretrained(checkpoint)
    config.device = "cuda" if torch.cuda.is_available() else "cpu"
    policy = ACTPolicy.from_pretrained(checkpoint, config=config).to(config.device)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": config.device}},
    )
    return config, policy, preprocessor, postprocessor


def evaluate(args: argparse.Namespace) -> None:
    import matplotlib.pyplot as plt
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    checkpoint = args.checkpoint.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config, policy, preprocessor, postprocessor = _checkpoint_objects(checkpoint)
    horizon = int(config.chunk_size)
    dataset = LeRobotDataset(
        "local/physical_bottle_5demo_act_validation",
        root=args.view.resolve(),
        delta_timestamps={ACTION_KEY: [i / 30 for i in range(horizon)]},
        video_backend="pyav",
        return_uint8=True,
    )
    loader = torch.utils.data.DataLoader(
        dataset, batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers
    )
    all_pred: list[np.ndarray] = []
    all_gt: list[np.ndarray] = []
    all_valid: list[np.ndarray] = []
    source_episode: list[np.ndarray] = []
    source_frame: list[np.ndarray] = []
    policy.eval()
    torch.manual_seed(0)
    with torch.inference_mode():
        for batch in loader:
            gt = batch[ACTION_KEY].clone()
            valid = ~batch[f"{ACTION_KEY}_is_pad"]
            source_episode.append(batch["source_episode_index"].numpy())
            source_frame.append(batch["source_frame_index"].numpy())
            for key in (WORKSPACE_KEY, WRIST_KEY):
                batch[key] = batch[key].float() / 255.0
            processed = preprocessor(batch)
            pred_normalized = policy.predict_action_chunk(processed)
            pred_native = postprocessor(pred_normalized)
            all_pred.append(pred_native.detach().cpu().numpy())
            all_gt.append(gt.numpy())
            all_valid.append(valid.numpy())
    predictions = np.concatenate(all_pred)
    ground_truth = np.concatenate(all_gt)
    valid = np.concatenate(all_valid)
    episodes = np.concatenate(source_episode).reshape(-1)
    frames = np.concatenate(source_frame).reshape(-1)
    if predictions.shape != (len(dataset), horizon, 12) or not np.isfinite(predictions).all():
        raise ValueError(f"invalid inference output {predictions.shape}")
    errors = predictions - ground_truth
    valid3 = valid[..., None]
    abs_error = np.abs(errors)
    squared_error = errors**2
    denom = int(valid.sum())
    per_dim_mae = (abs_error * valid3).sum(axis=(0, 1)) / denom
    per_dim_mse = (squared_error * valid3).sum(axis=(0, 1)) / denom
    horizon_count = valid.sum(axis=0)
    horizon_mae = (abs_error * valid3).sum(axis=(0, 2)) / (horizon_count * 12)
    horizon_mse = (squared_error * valid3).sum(axis=(0, 2)) / (horizon_count * 12)
    native_values = ground_truth[valid]
    predicted_values = predictions[valid]
    train_min = native_values.min(axis=0)
    train_max = native_values.max(axis=0)
    pred_min = predicted_values.min(axis=0)
    pred_max = predicted_values.max(axis=0)
    train_range = train_max - train_min
    significant_low = pred_min < train_min - 0.05 * train_range
    significant_high = pred_max > train_max + 0.05 * train_range
    per_source_episode: dict[str, dict[str, float]] = {}
    for source_id in EXPECTED_SOURCE_EPISODES:
        episode_mask = episodes == source_id
        episode_valid = valid[episode_mask]
        episode_abs = abs_error[episode_mask]
        episode_sq = squared_error[episode_mask]
        per_source_episode[str(source_id)] = {
            "mae": float(episode_abs[episode_valid].mean()),
            "mse": float(episode_sq[episode_valid].mean()),
            "jaka_mae": float(episode_abs[..., :6][episode_valid].mean()),
            "rh56_mae": float(episode_abs[..., 6:][episode_valid].mean()),
        }

    fixed = dataset[100]
    fixed_batch = torch.utils.data.default_collate([fixed])
    for key in (WORKSPACE_KEY, WRIST_KEY):
        fixed_batch[key] = fixed_batch[key].float() / 255.0
    with torch.inference_mode():
        fixed_processed = preprocessor(fixed_batch)
        first = postprocessor(policy.predict_action_chunk(fixed_processed)).detach().cpu()
        second = postprocessor(policy.predict_action_chunk(fixed_processed)).detach().cpu()
    deterministic_max_abs_diff = torch.max(torch.abs(first - second)).item()

    phase_fractions = {
        "approach": 0.10,
        "grasp": 0.30,
        "lift": 0.45,
        "transport": 0.65,
        "placement": 0.82,
        "release": 0.95,
    }
    episode65_indices = np.flatnonzero(episodes == 65)
    plot_files: list[str] = []
    for phase, fraction in phase_fractions.items():
        idx = int(episode65_indices[round(fraction * (len(episode65_indices) - 1))])
        for group, channel_slice, names in (
            ("jaka", slice(0, 6), ACTION_NAMES[:6]),
            ("rh56", slice(6, 12), ACTION_NAMES[6:]),
        ):
            fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
            x = np.arange(horizon)
            for channel, axis in enumerate(axes.flat):
                source_channel = channel + channel_slice.start
                axis.plot(x, ground_truth[idx, :, source_channel], label="ground truth", linewidth=2)
                axis.plot(x, predictions[idx, :, source_channel], label="ACT", linewidth=1.5)
                if valid[idx].sum() < horizon:
                    axis.axvspan(valid[idx].sum() - 0.5, horizon - 0.5, color="gray", alpha=0.2)
                axis.set_title(names[channel])
                axis.grid(alpha=0.25)
            axes.flat[0].legend()
            fig.suptitle(
                f"{phase} proxy — episode 65 frame {int(frames[idx])} — {group.upper()} native targets"
            )
            fig.tight_layout()
            path = output / f"{phase}_{group}_chunk.png"
            fig.savefig(path, dpi=140)
            plt.close(fig)
            plot_files.append(str(path))

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(np.arange(horizon), horizon_mae, marker="o")
    axes[0].set_ylabel("native MAE")
    axes[1].plot(np.arange(horizon), horizon_mse, marker="o")
    axes[1].set_ylabel("native MSE")
    axes[1].set_xlabel("chunk horizon")
    for axis in axes:
        axis.grid(alpha=0.3)
    fig.tight_layout()
    horizon_plot = output / "error_by_chunk_horizon.png"
    fig.savefig(horizon_plot, dpi=140)
    plt.close(fig)

    report = {
        "status": "PASS",
        "checkpoint": str(checkpoint),
        "frames_evaluated": len(dataset),
        "valid_action_targets_evaluated": denom,
        "output_shape": list(predictions.shape),
        "finite_outputs": True,
        "total_native_mae": float(abs_error[valid].mean()),
        "total_native_mse": float(squared_error[valid].mean()),
        "jaka_native_mae": float(abs_error[..., :6][valid].mean()),
        "jaka_native_mse": float(squared_error[..., :6][valid].mean()),
        "rh56_native_mae": float(abs_error[..., 6:][valid].mean()),
        "rh56_native_mse": float(squared_error[..., 6:][valid].mean()),
        "per_source_episode": per_source_episode,
        "per_dimension_mae": dict(zip(ACTION_NAMES, per_dim_mae.tolist(), strict=True)),
        "per_dimension_mse": dict(zip(ACTION_NAMES, per_dim_mse.tolist(), strict=True)),
        "horizon_mae": horizon_mae.tolist(),
        "horizon_mse": horizon_mse.tolist(),
        "horizon_valid_counts": horizon_count.tolist(),
        "training_action_min": train_min.tolist(),
        "training_action_max": train_max.tolist(),
        "prediction_min": pred_min.tolist(),
        "prediction_max": pred_max.tolist(),
        "significant_below_training_envelope": dict(
            zip(ACTION_NAMES, significant_low.tolist(), strict=True)
        ),
        "significant_above_training_envelope": dict(
            zip(ACTION_NAMES, significant_high.tolist(), strict=True)
        ),
        "deterministic_repeat_max_abs_diff": deterministic_max_abs_diff,
        "deterministic_repeat_identical": bool(deterministic_max_abs_diff == 0.0),
        "normalization_loaded_from_checkpoint": True,
        "native_action_semantics": "absolute_native_target; JAKA[0:6], RH56[6:12]",
        "phase_labels": "relative-progress proxies for qualitative inspection; dataset has no phase labels",
        "chunk_plots": plot_files,
        "horizon_plot": str(horizon_plot),
    }
    _write_json(output / "evaluation_metrics.json", report)
    np.savez_compressed(
        output / "evaluation_arrays.npz",
        predictions=predictions,
        ground_truth=ground_truth,
        valid=valid,
        source_episode=episodes,
        source_frame=frames,
    )
    print(json.dumps(report, indent=2))


def parse_loss(args: argparse.Namespace) -> None:
    import matplotlib.pyplot as plt

    text = args.log.read_text(encoding="utf-8", errors="replace")
    rows: list[dict[str, float | int]] = []
    for line in text.splitlines():
        if "step:" not in line or "loss:" not in line:
            continue
        values: dict[str, float | int] = {}
        for name in (
            "step",
            "loss",
            "l1_loss",
            "kld_loss",
            "lr",
            "updt_s",
            "data_s",
            "smp/s",
            "mem_gb",
        ):
            match = re.search(rf"(?:^|\s){re.escape(name)}:([0-9.eE+-]+)", line)
            if match:
                values[name] = int(match.group(1)) if name == "step" else float(match.group(1))
        if "step" in values and "loss" in values:
            rows.append(values)
    if not rows:
        raise ValueError(f"no LeRobot training metrics found in {args.log}")
    # LeRobot abbreviates displayed steps at 1K and above.  Log rows are emitted
    # at a fixed frequency, so restore exact optimizer steps from the unabridged
    # initial sequence instead of treating every displayed "1K" as step 1.
    initial_steps = [int(row["step"]) for row in rows]
    first_step = initial_steps[0]
    log_frequency = first_step
    for index, row in enumerate(rows):
        row["step"] = first_step + index * log_frequency
    _write_json(args.output / "loss_history.json", rows)
    fig, axis = plt.subplots(figsize=(9, 5))
    axis.plot([row["step"] for row in rows], [row["loss"] for row in rows], label="total loss")
    if all("l1_loss" in row for row in rows):
        axis.plot([row["step"] for row in rows], [row["l1_loss"] for row in rows], label="L1")
    axis.set_xlabel("optimizer step")
    axis.set_ylabel("training loss")
    axis.set_yscale("log")
    axis.grid(alpha=0.3)
    axis.legend()
    fig.tight_layout()
    path = args.output / "loss_history.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    summary = {
        "rows": len(rows),
        "first": rows[0],
        "last": rows[-1],
        "minimum_loss": min(float(row["loss"]) for row in rows),
        "loss_plot": str(path),
    }
    _write_json(args.output / "loss_summary.json", summary)
    print(json.dumps(summary, indent=2))


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build-view")
    build.add_argument("--master", type=Path, required=True)
    build.add_argument("--view", type=Path, required=True)
    build.add_argument("--width", type=int, default=320)
    build.add_argument("--height", type=int, default=240)
    build.add_argument("--image-writer-threads", type=int, default=8)
    build.add_argument("--encoder-threads", type=int, default=4)
    build.set_defaults(function=build_view)

    inspect = subparsers.add_parser("inspect-loader")
    inspect.add_argument("--master", type=Path, required=True)
    inspect.add_argument("--view", type=Path, required=True)
    inspect.add_argument("--output", type=Path, required=True)
    inspect.add_argument("--chunk-size", type=int, default=16)
    inspect.add_argument("--width", type=int, default=320)
    inspect.add_argument("--height", type=int, default=240)
    inspect.set_defaults(function=inspect_loader)

    evaluation = subparsers.add_parser("evaluate")
    evaluation.add_argument("--view", type=Path, required=True)
    evaluation.add_argument("--checkpoint", type=Path, required=True)
    evaluation.add_argument("--output", type=Path, required=True)
    evaluation.add_argument("--batch-size", type=int, default=16)
    evaluation.add_argument("--num-workers", type=int, default=2)
    evaluation.set_defaults(function=evaluate)

    loss = subparsers.add_parser("parse-loss")
    loss.add_argument("--log", type=Path, required=True)
    loss.add_argument("--output", type=Path, required=True)
    loss.set_defaults(function=parse_loss)
    return parser


def main() -> None:
    args = make_parser().parse_args()
    args.function(args)


if __name__ == "__main__":
    main()
