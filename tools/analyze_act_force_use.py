#!/usr/bin/env python3
"""Run a command-disabled, offline ACT ``FORCE_ACT`` intervention probe.

The normal entrypoint only validates paths and launches this same file inside
the repository's pinned, network-disabled LeRobot container.  The repository
is mounted read-only except for one new ignored output directory.  No hardware
or control module is imported, and this program has no command API.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from research.exploration.act_force_interventions import (  # noqa: E402
    ACTION_DIM,
    FORCE_FEATURE_ORDER,
    FORCE_ORDER,
    EpisodeSignals,
    InterventionSet,
    action_chunk,
    build_interventions,
    masked_action_metrics,
    masked_prediction_sensitivity,
    require_force_order,
)


SCHEMA_VERSION = "embodied_lab.icra2027.act_force_use.v1"
EXPECTED_LEROBOT_VERSION = "0.6.2"
CONTAINER_MARKER = "EMBODIED_LAB_ACT_FORCE_OFFLINE_CONTAINER"
CONTAINER_IMAGE_ID = "EMBODIED_LAB_ACT_FORCE_IMAGE_ID"
DEFAULT_CONFIG = ROOT / "configs/experiments/icra2027_act_force_use.yaml"
STATE_KEY = "observation.state"
FORCE_KEY = "observation.environment_state"
ACTION_KEY = "action"
WORKSPACE_KEY = "observation.images.workspace"
WRIST_KEY = "observation.images.wrist"
IDENTITY_COLUMNS = (
    "index",
    "episode_index",
    "frame_index",
    "source_episode_index",
    "source_frame_index",
    "segment_id",
    "timestamp",
    "task_index",
    STATE_KEY,
    ACTION_KEY,
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


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _tree_manifest(root: Path) -> dict[str, str]:
    if not root.is_dir():
        raise ValueError(f"missing directory: {root}")
    return {
        str(path.relative_to(root)): _sha256_file(path)
        for path in sorted(item for item in root.rglob("*") if item.is_file())
    }


def _tree_digest(files: Mapping[str, str]) -> str:
    payload = json.dumps(files, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _load_config(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("experiment config must be a mapping")
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported experiment schema: {payload.get('schema_version')!r}")
    if payload.get("offline_only") is not True:
        raise ValueError("experiment config must declare offline_only: true")
    return payload


def _inside(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _repo_path(value: str, *, must_exist: bool = True) -> Path:
    path = (ROOT / value).resolve() if not Path(value).is_absolute() else Path(value).resolve()
    if not _inside(path, ROOT):
        raise ValueError(f"repository input escapes the worktree: {value}")
    if must_exist and not path.exists():
        raise ValueError(f"repository input does not exist: {path}")
    return path


def _container_path(path: Path) -> str:
    relative = path.resolve().relative_to(ROOT)
    return str(Path("/workspace/embodied_lab") / relative)


def _host_launch(args: argparse.Namespace, config: Mapping[str, Any]) -> int:
    runtime = config.get("runtime")
    if not isinstance(runtime, Mapping):
        raise ValueError("config is missing runtime mapping")
    image = str(runtime.get("image", ""))
    expected_id = str(runtime.get("expected_image_id", ""))
    if not image or not expected_id.startswith("sha256:"):
        raise ValueError("runtime requires a pinned image and sha256 image id")
    config_path = args.config.resolve()
    if not _inside(config_path, ROOT):
        raise ValueError("config must be inside the current worktree")
    configured_output = str(config.get("output", ""))
    output = (args.output or _repo_path(configured_output, must_exist=False)).resolve()
    allowed_output_root = (ROOT / "outputs/research").resolve()
    if not _inside(output, allowed_output_root):
        raise ValueError(f"output must be below {allowed_output_root}")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite experiment output: {output}")

    try:
        inspected = subprocess.run(
            ["docker", "image", "inspect", image, "--format", "{{.Id}}"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except FileNotFoundError as exc:
        raise RuntimeError("docker is required for this pinned offline analysis") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"cannot inspect pinned LeRobot image {image}") from exc
    if inspected != expected_id:
        raise RuntimeError(
            f"LeRobot image identity mismatch: expected {expected_id}, got {inspected}"
        )

    output.mkdir(parents=True, exist_ok=False)
    command = [
        "docker",
        "run",
        "--rm",
        "--runtime=nvidia",
        "--ipc=host",
        "--network",
        "none",
        "-v",
        f"{ROOT}:/workspace/embodied_lab:ro",
        "-v",
        f"{output}:{_container_path(output)}:rw",
        "-w",
        "/workspace/embodied_lab",
        "-e",
        f"{CONTAINER_MARKER}=1",
        "-e",
        f"{CONTAINER_IMAGE_ID}={inspected}",
        image,
        "python",
        "tools/analyze_act_force_use.py",
        "--inside-container",
        "--config",
        _container_path(config_path),
        "--output",
        _container_path(output),
    ]
    subprocess.run(command, check=True)
    print(f"Offline force-use report: {output / 'force_use_report.json'}")
    return 0


def _parquet_files(view: Path) -> list[Path]:
    files = sorted((view / "data").glob("**/*.parquet"))
    if not files:
        raise ValueError(f"derived view contains no parquet data: {view}")
    return files


def _read_view_table(view: Path, columns: Sequence[str]):
    import pyarrow as pa
    import pyarrow.parquet as parquet

    tables = [parquet.read_table(path, columns=list(columns)) for path in _parquet_files(view)]
    return pa.concat_tables(tables) if len(tables) > 1 else tables[0]


def _table_digest(table: Any) -> str:
    payload = json.dumps(
        table.to_pydict(),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _video_manifest(view: Path) -> dict[str, str]:
    video_root = view / "videos"
    files = sorted(item for item in video_root.rglob("*") if item.is_file())
    if not files:
        raise ValueError(f"derived view contains no videos: {view}")
    return {str(path.relative_to(video_root)): _sha256_file(path) for path in files}


def _paired_view_identity(act_view: Path, force_view: Path) -> dict[str, Any]:
    act_provenance = _read_json(act_view / "meta/embodied_lab_provenance.json")
    force_provenance = _read_json(force_view / "meta/embodied_lab_provenance.json")
    act_info = _read_json(act_view / "meta/info.json")
    force_info = _read_json(force_view / "meta/info.json")
    require_force_order(
        force_info["features"][FORCE_KEY].get("names", ())
    )
    if tuple(act_info["features"][STATE_KEY].get("names", ())) != tuple(
        force_info["features"][STATE_KEY].get("names", ())
    ):
        raise ValueError("ACT and ACT+Force state channel orders differ")
    if tuple(act_info["features"][ACTION_KEY].get("names", ())) != ACTION_ORDER:
        raise ValueError("standard ACT action order does not match the repository contract")
    if tuple(force_info["features"][ACTION_KEY].get("names", ())) != ACTION_ORDER:
        raise ValueError("ACT+Force action order does not match the repository contract")
    if act_provenance.get("episode_map") != force_provenance.get("episode_map"):
        raise ValueError("ACT and ACT+Force episode maps differ")
    if act_provenance.get("validation_source_episodes") != force_provenance.get(
        "validation_source_episodes"
    ):
        raise ValueError("ACT and ACT+Force validation identities differ")
    act_table = _read_view_table(act_view, IDENTITY_COLUMNS)
    force_table = _read_view_table(force_view, IDENTITY_COLUMNS)
    if not act_table.equals(force_table):
        raise ValueError("ACT and ACT+Force non-force rows are not exactly identical")
    act_videos = _video_manifest(act_view)
    force_videos = _video_manifest(force_view)
    if act_videos != force_videos:
        raise ValueError("ACT and ACT+Force video bytes are not exactly identical")
    return {
        "status": "PASS",
        "row_count": int(act_table.num_rows),
        "episode_count": len(act_provenance["episode_map"]),
        "identity_columns": list(IDENTITY_COLUMNS),
        "identity_rows_sha256": _table_digest(act_table),
        "video_sha256": act_videos,
        "episode_map": act_provenance["episode_map"],
        "validation_source_episodes": act_provenance["validation_source_episodes"],
        "act_source_master_sha256_tree": act_provenance.get(
            "source_master_sha256_tree"
        ),
        "act_force_source_master_sha256_tree": force_provenance.get(
            "source_master_sha256_tree"
        ),
        "comparison_observation_source": str(force_view),
        "standard_act_force_field_removed_before_preprocessing": True,
    }


def _episode_records(master: Path) -> dict[str, dict[str, Any]]:
    path = master / "meta/episodes.jsonl"
    records = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    by_segment = {str(record["logical_segment_id"]): record for record in records}
    if len(by_segment) != len(records):
        raise ValueError("master logical segment ids are not unique")
    return by_segment


def _session_by_source(config: Mapping[str, Any], present: set[int]) -> dict[int, str]:
    collection = config.get("collection_sessions")
    if not isinstance(collection, Mapping):
        raise ValueError("config is missing collection_sessions")
    groups = collection.get("groups")
    if not isinstance(groups, Mapping):
        raise ValueError("collection_sessions.groups must be a mapping")
    result: dict[int, str] = {}
    for session, source_values in groups.items():
        if not isinstance(source_values, list) or not source_values:
            raise ValueError(f"collection session {session!r} must contain source episodes")
        for value in source_values:
            source = int(value)
            if source in result:
                raise ValueError(f"source episode {source} appears in two sessions")
            result[source] = str(session)
    if set(result) != present:
        raise ValueError(
            "collection session inventory differs from paired view sources: "
            f"missing={sorted(present - set(result))}, extra={sorted(set(result) - present)}"
        )
    return result


def _load_signals(
    master: Path,
    episode_map: Sequence[Mapping[str, Any]],
    session_by_source: Mapping[int, str],
) -> tuple[list[EpisodeSignals], dict[str, tuple[int, int]], np.ndarray]:
    import pyarrow.parquet as parquet

    records = _episode_records(master)
    episodes: list[EpisodeSignals] = []
    offsets: dict[str, tuple[int, int]] = {}
    force_values: list[np.ndarray] = []
    offset = 0
    for item in episode_map:
        key = str(item["logical_segment_id"])
        if key not in records:
            raise ValueError(f"paired view segment is absent from master: {key}")
        record = records[key]
        table = parquet.read_table(master / str(record["data"]))
        rows = table.to_pydict()
        count = int(table.num_rows)
        if count != int(item["length"]):
            raise ValueError(f"{key}: paired view/master length mismatch")
        source = int(item["source_episode_index"])
        if set(int(value) for value in rows["source_episode_index"]) != {source}:
            raise ValueError(f"{key}: source episode identity mismatch")
        frame = np.asarray(rows["frame_index"], dtype=np.int64)
        if not np.array_equal(frame, np.arange(count, dtype=np.int64)):
            raise ValueError(f"{key}: logical frame index is not contiguous")
        state = np.asarray(rows[STATE_KEY], dtype=np.float32)
        force = np.asarray(rows["observation.force"], dtype=np.float32)
        episode = EpisodeSignals(
            key=key,
            source_episode=source,
            session=session_by_source[source],
            canonical_timestamp_ns=np.asarray(rows["timestamp_ns"], dtype=np.int64),
            force_timestamp_ns=np.asarray(rows["force_timestamp_ns"], dtype=np.int64),
            hand_q=state[:, 6:12],
            force=force,
            action=np.asarray(rows[ACTION_KEY], dtype=np.float32),
        )
        episodes.append(episode)
        offsets[key] = (offset, offset + count)
        force_values.append(force)
        offset += count
    return episodes, offsets, np.concatenate(force_values, axis=0)


def _verify_force_materialization(force_view: Path, force: np.ndarray) -> str:
    table = _read_view_table(force_view, (FORCE_KEY,))
    derived = np.asarray(table.column(FORCE_KEY).to_pylist(), dtype=np.float32)
    if not np.array_equal(derived, force):
        raise ValueError("ACT+Force derived view changed or reordered raw FORCE_ACT")
    digest = hashlib.sha256(force.astype("<f4", copy=False).tobytes()).hexdigest()
    return digest


def _split_audit(
    episode_map: Sequence[Mapping[str, Any]], session_by_source: Mapping[int, str]
) -> dict[str, Any]:
    split_sessions: dict[str, set[str]] = {}
    validation: list[dict[str, Any]] = []
    for item in episode_map:
        split = str(item["split"])
        session = session_by_source[int(item["source_episode_index"])]
        split_sessions.setdefault(session, set()).add(split)
    leaky = sorted(session for session, splits in split_sessions.items() if len(splits) > 1)
    for item in episode_map:
        if item["split"] == "val":
            session = session_by_source[int(item["source_episode_index"])]
            validation.append(
                {
                    "logical_segment_id": item["logical_segment_id"],
                    "source_episode_index": int(item["source_episode_index"]),
                    "collection_session": session,
                    "session_seen_in_training": session in leaky,
                }
            )
    return {
        "label": "SESSION_LEAKY",
        "is_session_independent": False,
        "leaky_collection_sessions": leaky,
        "validation_episodes": validation,
        "interpretation": (
            "val4 is an episode-held-out diagnostic split, not a collection-session-held-out "
            "generalization benchmark and not a reliable model-selection benchmark"
        ),
    }


def _selected_indices(
    episode_map: Sequence[Mapping[str, Any]],
    offsets: Mapping[str, tuple[int, int]],
    *,
    samples_per_episode: int,
) -> tuple[list[int], list[str], list[int]]:
    if samples_per_episode <= 0:
        raise ValueError("samples_per_episode must be positive")
    indices: list[int] = []
    keys: list[str] = []
    local_rows: list[int] = []
    for item in episode_map:
        if item["split"] != "val":
            continue
        key = str(item["logical_segment_id"])
        start, stop = offsets[key]
        count = stop - start
        take = min(count, samples_per_episode)
        local = np.unique(
            np.rint(np.linspace(0, count - 1, num=take, dtype=np.float64)).astype(
                np.int64
            )
        )
        indices.extend((start + local).tolist())
        keys.extend([key] * len(local))
        local_rows.extend(local.tolist())
    if not indices:
        raise ValueError("paired view has no validation rows")
    return indices, keys, local_rows


def _checkpoint_objects(checkpoint: Path, *, require_force: bool, device: str):
    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    config = PreTrainedConfig.from_pretrained(checkpoint)
    has_force = FORCE_KEY in config.input_features
    if has_force != require_force:
        expected = "ACT+Force" if require_force else "standard ACT"
        raise ValueError(f"{checkpoint} is not the expected {expected} checkpoint")
    config.device = device
    policy = ACTPolicy.from_pretrained(checkpoint, config=config).to(device)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": device}},
    )
    policy.eval()
    torch.manual_seed(0)
    return config, policy, preprocessor, postprocessor


class _SelectedDataset:
    def __init__(self, dataset: Any, indices: Sequence[int]) -> None:
        self.dataset = dataset
        self.indices = list(indices)

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, index: int) -> dict[str, Any]:
        import torch

        global_index = self.indices[index]
        sample = dict(self.dataset[global_index])
        sample["probe_global_index"] = torch.tensor(global_index, dtype=torch.int64)
        sample["probe_selected_index"] = torch.tensor(index, dtype=torch.int64)
        return sample


def _model_batch(batch: Mapping[str, Any], *, force: np.ndarray | None) -> dict[str, Any]:
    import torch

    result = {
        WORKSPACE_KEY: batch[WORKSPACE_KEY].to(dtype=torch.float32).div(255.0),
        WRIST_KEY: batch[WRIST_KEY].to(dtype=torch.float32).div(255.0),
        STATE_KEY: batch[STATE_KEY].clone(),
    }
    if force is not None:
        value = torch.from_numpy(np.asarray(force, dtype=np.float32))
        result[FORCE_KEY] = value
    return result


def _expected_chunks(
    episodes: Mapping[str, EpisodeSignals],
    keys: Sequence[str],
    local_rows: Sequence[int],
    selected_positions: Sequence[int],
    *,
    horizon: int,
) -> tuple[np.ndarray, np.ndarray]:
    chunks: list[np.ndarray] = []
    pads: list[np.ndarray] = []
    for position in selected_positions:
        chunk, pad = action_chunk(
            episodes[keys[position]].action,
            start=int(local_rows[position]),
            horizon=horizon,
        )
        chunks.append(chunk)
        pads.append(pad)
    return np.stack(chunks), np.stack(pads)


def _infer_standard(
    *,
    dataset: Any,
    indices: Sequence[int],
    keys: Sequence[str],
    local_rows: Sequence[int],
    episodes: Mapping[str, EpisodeSignals],
    checkpoint: Path,
    device: str,
    batch_size: int,
    num_workers: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    import torch

    config, policy, preprocessor, postprocessor = _checkpoint_objects(
        checkpoint, require_force=False, device=device
    )
    horizon = int(config.chunk_size)
    loader = torch.utils.data.DataLoader(
        _SelectedDataset(dataset, indices),
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
    )
    prediction: list[np.ndarray] = []
    target: list[np.ndarray] = []
    valid: list[np.ndarray] = []
    with torch.inference_mode():
        for batch in loader:
            positions = batch["probe_selected_index"].numpy().astype(np.int64).tolist()
            expected, expected_pad = _expected_chunks(
                episodes, keys, local_rows, positions, horizon=horizon
            )
            actual = batch[ACTION_KEY].numpy()
            actual_pad = batch[f"{ACTION_KEY}_is_pad"].numpy()
            if not np.array_equal(actual, expected) or not np.array_equal(
                actual_pad, expected_pad
            ):
                raise ValueError("LeRobot action chunk crossed or changed an episode boundary")
            model_input = _model_batch(batch, force=None)
            native = postprocessor(policy.predict_action_chunk(preprocessor(model_input)))
            output = native.detach().cpu().numpy()
            if output.shape != expected.shape or not np.isfinite(output).all():
                raise ValueError(f"standard ACT produced invalid actions {output.shape}")
            prediction.append(output)
            target.append(actual)
            valid.append(~actual_pad)
    return np.concatenate(prediction), np.concatenate(target), np.concatenate(valid)


def _selected_force(
    interventions: InterventionSet,
    condition: str,
    keys: Sequence[str],
    local_rows: Sequence[int],
    positions: Sequence[int],
) -> np.ndarray:
    rows: list[np.ndarray] = []
    for position in positions:
        key = keys[position]
        name = condition if key in interventions.eligible[condition] else "factual"
        rows.append(interventions.values[name][key][int(local_rows[position])])
    result = np.asarray(rows, dtype=np.float32)
    if result.shape != (len(positions), len(FORCE_ORDER)) or not np.isfinite(result).all():
        raise ValueError(f"invalid raw force intervention {condition}: {result.shape}")
    return result


def _infer_force_conditions(
    *,
    dataset: Any,
    indices: Sequence[int],
    keys: Sequence[str],
    local_rows: Sequence[int],
    interventions: InterventionSet,
    checkpoint: Path,
    device: str,
    batch_size: int,
    num_workers: int,
) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray]:
    import torch

    config, policy, preprocessor, postprocessor = _checkpoint_objects(
        checkpoint, require_force=True, device=device
    )
    horizon = int(config.chunk_size)
    conditions = tuple(interventions.values)
    loader = torch.utils.data.DataLoader(
        _SelectedDataset(dataset, indices),
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
    )
    predictions: dict[str, list[np.ndarray]] = {name: [] for name in conditions}
    target: list[np.ndarray] = []
    valid: list[np.ndarray] = []
    with torch.inference_mode():
        for batch in loader:
            positions = batch["probe_selected_index"].numpy().astype(np.int64).tolist()
            expected = batch[ACTION_KEY].numpy()
            expected_pad = batch[f"{ACTION_KEY}_is_pad"].numpy()
            # The standard pass already checked every selected ground-truth chunk
            # against the immutable master.  This pass must see byte-identical
            # chunks and masks on the same selected indices.
            target.append(expected)
            valid.append(~expected_pad)
            factual_from_view = batch[FORCE_KEY].numpy()
            factual_from_master = _selected_force(
                interventions, "factual", keys, local_rows, positions
            )
            if not np.array_equal(factual_from_view, factual_from_master):
                raise ValueError("selected LeRobot FORCE_ACT differs from the causal master")
            for condition in conditions:
                raw_force = _selected_force(
                    interventions, condition, keys, local_rows, positions
                )
                model_input = _model_batch(batch, force=raw_force)
                processed = preprocessor(model_input)
                native = postprocessor(policy.predict_action_chunk(processed))
                output = native.detach().cpu().numpy()
                if output.shape != (len(positions), horizon, ACTION_DIM):
                    raise ValueError(
                        f"ACT+Force {condition} produced invalid shape {output.shape}"
                    )
                if not np.isfinite(output).all():
                    raise ValueError(f"ACT+Force {condition} produced non-finite actions")
                predictions[condition].append(output)
    return (
        {name: np.concatenate(parts) for name, parts in predictions.items()},
        np.concatenate(target),
        np.concatenate(valid),
    )


def _eligibility_mask(
    intervention: InterventionSet, condition: str, keys: Sequence[str]
) -> np.ndarray:
    return np.asarray([key in intervention.eligible[condition] for key in keys], dtype=bool)


def _changed_mask(
    intervention: InterventionSet,
    condition: str,
    keys: Sequence[str],
    local_rows: Sequence[int],
) -> np.ndarray:
    return np.asarray(
        [
            key in intervention.changed_rows[condition]
            and bool(intervention.changed_rows[condition][key][int(row)])
            for key, row in zip(keys, local_rows, strict=True)
        ],
        dtype=bool,
    )


def _runtime_versions() -> dict[str, str]:
    import torch

    return {
        "lerobot": importlib.metadata.version("lerobot"),
        "torch": torch.__version__,
        "numpy": np.__version__,
        "pyarrow": importlib.metadata.version("pyarrow"),
        "python": sys.version.split()[0],
        "cuda_device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none",
    }


def _git_identity() -> dict[str, Any]:
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, capture_output=True, text=True
    ).stdout.strip()
    status = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    return {"revision": revision, "dirty": bool(status), "status_entries": status}


def _inside_run(args: argparse.Namespace, config: Mapping[str, Any]) -> int:
    if os.environ.get(CONTAINER_MARKER) != "1":
        raise RuntimeError("internal analysis mode is available only through the pinned launcher")
    versions = _runtime_versions()
    if versions["lerobot"] != EXPECTED_LEROBOT_VERSION:
        raise RuntimeError(
            f"expected LeRobot {EXPECTED_LEROBOT_VERSION}, got {versions['lerobot']}"
        )
    runtime = config["runtime"]
    expected_image_id = str(runtime["expected_image_id"])
    if os.environ.get(CONTAINER_IMAGE_ID) != expected_image_id:
        raise RuntimeError("container image identity was not carried through the launcher")
    output = args.output.resolve()
    if not output.is_dir() or any(output.iterdir()):
        raise ValueError("container output must be an existing empty directory")
    paths = config.get("paths")
    if not isinstance(paths, Mapping):
        raise ValueError("config is missing paths")
    act_view = _repo_path(str(paths["act_view"]))
    force_view = _repo_path(str(paths["act_force_view"]))
    master = _repo_path(str(paths["act_force_master"]))
    act_checkpoint = _repo_path(str(paths["act_checkpoint"]))
    force_checkpoint = _repo_path(str(paths["act_force_checkpoint"]))
    paired_identity = _paired_view_identity(act_view, force_view)
    episode_map = paired_identity["episode_map"]
    present_sources = {int(item["source_episode_index"]) for item in episode_map}
    session_by_source = _session_by_source(config, present_sources)
    episodes, offsets, force = _load_signals(master, episode_map, session_by_source)
    paired_identity["raw_force_float32_sha256"] = _verify_force_materialization(
        force_view, force
    )
    split_audit = _split_audit(episode_map, session_by_source)
    experiment = config.get("experiment")
    if not isinstance(experiment, Mapping):
        raise ValueError("config is missing experiment mapping")
    training_keys = [
        str(item["logical_segment_id"]) for item in episode_map if item["split"] == "train"
    ]
    interventions = build_interventions(
        episodes,
        training_episode_keys=training_keys,
        lag_s=float(experiment["causal_lag_s"]),
        dropout_interval_s=float(experiment["dropout_interval_s"]),
        dropout_period_s=float(experiment["dropout_period_s"]),
        q_ridge=float(experiment["q_conditioned_ridge"]),
        seed=int(experiment["seed"]),
    )
    indices, keys, local_rows = _selected_indices(
        episode_map,
        offsets,
        samples_per_episode=int(experiment["samples_per_validation_episode"]),
    )

    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    device = str(experiment.get("device", "cuda"))
    if device != "cuda" or not torch.cuda.is_available():
        raise RuntimeError("this pinned probe requires the audited CUDA runtime")
    torch.manual_seed(int(experiment["seed"]))
    np.random.seed(int(experiment["seed"]))
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    torch.use_deterministic_algorithms(True, warn_only=True)
    chunk_size = int(experiment["chunk_size"])
    dataset = LeRobotDataset(
        "local/physical_bottle_v2_act_force",
        root=force_view,
        delta_timestamps={ACTION_KEY: [index / 30 for index in range(chunk_size)]},
        video_backend="pyav",
        return_uint8=True,
    )
    episode_by_key = {episode.key: episode for episode in episodes}
    batch_size = int(experiment["batch_size"])
    num_workers = int(experiment["num_workers"])
    standard_prediction, ground_truth, valid = _infer_standard(
        dataset=dataset,
        indices=indices,
        keys=keys,
        local_rows=local_rows,
        episodes=episode_by_key,
        checkpoint=act_checkpoint,
        device=device,
        batch_size=batch_size,
        num_workers=num_workers,
    )
    del dataset
    torch.cuda.empty_cache()
    dataset = LeRobotDataset(
        "local/physical_bottle_v2_act_force",
        root=force_view,
        delta_timestamps={ACTION_KEY: [index / 30 for index in range(chunk_size)]},
        video_backend="pyav",
        return_uint8=True,
    )
    force_predictions, force_target, force_valid = _infer_force_conditions(
        dataset=dataset,
        indices=indices,
        keys=keys,
        local_rows=local_rows,
        interventions=interventions,
        checkpoint=force_checkpoint,
        device=device,
        batch_size=batch_size,
        num_workers=num_workers,
    )
    if not np.array_equal(ground_truth, force_target) or not np.array_equal(
        valid, force_valid
    ):
        raise ValueError("standard ACT and ACT+Force did not receive identical action targets")

    factual = force_predictions["factual"]
    metrics: dict[str, Any] = {
        "standard_act": {
            "masked_action_error": masked_action_metrics(
                standard_prediction, ground_truth, valid
            ),
            "sensitivity_from_act_force_factual": masked_prediction_sensitivity(
                standard_prediction, factual, valid
            ),
        }
    }
    for condition, prediction in force_predictions.items():
        eligible = _eligibility_mask(interventions, condition, keys)
        changed = _changed_mask(interventions, condition, keys, local_rows)
        entry: dict[str, Any] = {
            "eligible_observation_count": int(eligible.sum()),
            "ineligible_observation_count": int((~eligible).sum()),
            "raw_force_changed_observation_count": int((changed & eligible).sum()),
            "masked_action_error": masked_action_metrics(
                prediction[eligible], ground_truth[eligible], valid[eligible]
            ),
            "sensitivity_from_factual": masked_prediction_sensitivity(
                prediction[eligible], factual[eligible], valid[eligible]
            ),
        }
        changed_eligible = changed & eligible
        if changed_eligible.any():
            entry["sensitivity_on_changed_observations"] = masked_prediction_sensitivity(
                prediction[changed_eligible], factual[changed_eligible], valid[changed_eligible]
            )
        metrics[condition] = entry

    selected_sources = [episode_by_key[key].source_episode for key in keys]
    arrays_path = output / "force_use_arrays.npz"
    np.savez_compressed(
        arrays_path,
        standard_act_prediction=standard_prediction.astype(np.float32),
        **{
            f"act_force_{name}_prediction": value.astype(np.float32)
            for name, value in force_predictions.items()
        },
        ground_truth=ground_truth.astype(np.float32),
        valid=valid,
        global_index=np.asarray(indices, dtype=np.int64),
        logical_segment_id=np.asarray(keys),
        source_episode_index=np.asarray(selected_sources, dtype=np.int64),
        local_row=np.asarray(local_rows, dtype=np.int64),
    )
    report = {
        "schema_version": SCHEMA_VERSION,
        "status": "OFFLINE_PROBE_COMPLETE",
        "experiment_status_before_run": config.get("status"),
        "scope": config.get("scope"),
        "physical_validation": "not performed; no physical claims",
        "force_semantics": (
            "native RH56 FORCE_ACT actuator/push-rod load counts; not fingertip force, "
            "not tactile-array sensing, and not contact localization"
        ),
        "command_api_present": False,
        "network_access": "disabled by launcher",
        "repository_mount": "read-only except this ignored output directory",
        "paired_dataset_identity": paired_identity,
        "split_audit": split_audit,
        "sample_selection": {
            "method": "deterministic evenly spaced rows within each val4 episode",
            "observations": len(indices),
            "samples_per_validation_episode_limit": int(
                experiment["samples_per_validation_episode"]
            ),
            "logical_segments": sorted(set(keys)),
            "source_episodes": sorted(set(selected_sources)),
            "temporally_adjacent_train_test_leakage": False,
        },
        "interventions": interventions.metadata,
        "metrics": metrics,
        "metric_units_warning": (
            "aggregate native-action values mix JAKA radians and normalized RH56 targets; "
            "use the separately reported jaka/rh56 and per-dimension values"
        ),
        "runtime": versions,
        "pinned_container": {
            "image": runtime["image"],
            "image_id": expected_image_id,
            "network": "none",
        },
        "arrays": str(arrays_path),
        "identity_manifest": str(output / "identity_manifest.json"),
    }
    report_path = output / "force_use_report.json"
    _write_json(report_path, report)
    checkpoint_files = {
        "standard_act": _tree_manifest(act_checkpoint),
        "act_force": _tree_manifest(force_checkpoint),
    }
    manifest = {
        "schema_version": "embodied_lab.icra2027.act_force_use_identity.v1",
        "git": _git_identity(),
        "config": {
            "path": str(args.config.resolve()),
            "sha256": _sha256_file(args.config.resolve()),
        },
        "implementation": {
            "tool_sha256": _sha256_file(Path(__file__).resolve()),
            "core_sha256": _sha256_file(
                ROOT / "research/exploration/act_force_interventions.py"
            ),
        },
        "checkpoints": {
            name: {"files": files, "tree_sha256": _tree_digest(files)}
            for name, files in checkpoint_files.items()
        },
        "paired_dataset": paired_identity,
        "outputs": {
            "force_use_report.json": _sha256_file(report_path),
            "force_use_arrays.npz": _sha256_file(arrays_path),
        },
        "container_image": {
            "name": runtime["image"],
            "id": expected_image_id,
        },
    }
    _write_json(output / "identity_manifest.json", manifest)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--inside-container", action="store_true", help=argparse.SUPPRESS)
    return parser


def main() -> int:
    args = make_parser().parse_args()
    args.config = args.config.resolve()
    config = _load_config(args.config)
    if args.inside_container:
        if args.output is None:
            raise ValueError("internal container run requires --output")
        return _inside_run(args, config)
    return _host_launch(args, config)


if __name__ == "__main__":
    raise SystemExit(main())
