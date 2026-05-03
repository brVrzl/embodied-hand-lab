from __future__ import annotations

import argparse
import json
import urllib.request
import re
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from huggingface_hub import hf_hub_download


HF_DATASET_API = "https://huggingface.co/api/datasets/{dataset_id}"


def _require_pandas() -> Any:
    try:
        import pandas as pd
    except ImportError as exc:
        raise SystemExit(
            "Missing data dependencies. Install them with: .venv/bin/pip install -e '.[data]'"
        ) from exc
    return pd


def _load_json_file(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _download_info(dataset_id: str, info_path: str | None = None, cache_dir: str | None = None) -> tuple[dict[str, Any], str]:
    if info_path is None:
        info_path = "meta/info.json"
    try:
        path = Path(hf_hub_download(repo_id=dataset_id, repo_type="dataset", filename=info_path, cache_dir=cache_dir))
    except Exception:
        with urllib.request.urlopen(HF_DATASET_API.format(dataset_id=dataset_id), timeout=30) as response:
            dataset = json.loads(response.read().decode("utf-8"))
        candidates = sorted(
            item["rfilename"]
            for item in dataset.get("siblings", [])
            if item.get("rfilename", "").endswith("meta/info.json")
        )
        if not candidates:
            raise
        info_path = candidates[0]
        path = Path(hf_hub_download(repo_id=dataset_id, repo_type="dataset", filename=info_path, cache_dir=cache_dir))
    return _load_json_file(path), info_path


def _download_data_files(dataset_id: str, info: dict[str, Any], info_path: str, cache_dir: str | None = None) -> list[Path]:
    data_path = str(info["data_path"])
    base_prefix = ""
    if info_path != "meta/info.json":
        base_prefix = info_path.removesuffix("meta/info.json")

    total_episodes = int(info.get("total_episodes") or 0)
    chunks_size = int(info.get("chunks_size") or 1000)
    file_count = max(1, (total_episodes + chunks_size - 1) // chunks_size)
    paths: list[Path] = []
    for file_index in range(file_count):
        relative = data_path.format(chunk_index=0, file_index=file_index)
        filename = f"{base_prefix}{relative}"
        paths.append(Path(hf_hub_download(repo_id=dataset_id, repo_type="dataset", filename=filename, cache_dir=cache_dir)))
    return paths


def _as_matrix(series: Any) -> np.ndarray:
    values = series.to_list()
    return np.asarray(values, dtype=np.float32)


def _active_unitree_hand(state12: np.ndarray, cmd12: np.ndarray, episode: np.ndarray) -> tuple[np.ndarray, np.ndarray, list[str]]:
    states: list[np.ndarray] = []
    cmds: list[np.ndarray] = []
    sides: list[str] = []
    for episode_id in np.unique(episode):
        mask = episode == episode_id
        left_state = state12[mask, :6]
        right_state = state12[mask, 6:12]
        left_cmd = cmd12[mask, :6]
        right_cmd = cmd12[mask, 6:12]
        left_energy = float(np.nanmean(np.var(left_cmd, axis=0)) + np.nanmean(np.var(left_state, axis=0)))
        right_energy = float(np.nanmean(np.var(right_cmd, axis=0)) + np.nanmean(np.var(right_state, axis=0)))
        if right_energy > left_energy:
            states.append(right_state)
            cmds.append(right_cmd)
            sides.extend(["right"] * int(mask.sum()))
        else:
            states.append(left_state)
            cmds.append(left_cmd)
            sides.extend(["left"] * int(mask.sum()))
    return np.concatenate(states, axis=0), np.concatenate(cmds, axis=0), sides


def _extract_dataset(entry: dict[str, Any], *, cache_dir: str | None = None) -> dict[str, Any]:
    pd = _require_pandas()
    dataset_id = entry["id"]
    configured_info_path = entry.get("info_path")
    info, info_path = _download_info(dataset_id, configured_info_path, cache_dir)
    data_files = _download_data_files(dataset_id, info, info_path, cache_dir)
    frames = pd.concat([pd.read_parquet(path) for path in data_files], ignore_index=True)

    episode = frames["episode_index"].to_numpy(dtype=np.int64)
    frame = frames["frame_index"].to_numpy(dtype=np.int64)
    fields = entry.get("fields") or {}
    hand_mapping = entry.get("hand_mapping") or {}
    role = entry.get("role")

    if "observation_state" in fields or role == "target_domain_grasp_sequences":
        state_col = fields.get("observation_state", "observation.state")
        action_col = fields.get("action", "action")
        state_slice = slice(*hand_mapping.get("state_slice", [15, 21]))
        action_slice = slice(*hand_mapping.get("action_slice", [9, 15]))
        hand_state = _as_matrix(frames[state_col])[:, state_slice]
        hand_cmd = _as_matrix(frames[action_col])[:, action_slice]
        side = ["right"] * len(frames)
    else:
        state_col = fields.get("observation_hand_state", "observation.state.hand_state")
        action_col = fields.get("action_hand_cmd", "action.hand_cmd")
        state12 = _as_matrix(frames[state_col])
        cmd12 = _as_matrix(frames[action_col])
        hand_state, hand_cmd, side = _active_unitree_hand(state12, cmd12, episode)

    if hand_state.shape[1] != 6 or hand_cmd.shape[1] != 6:
        raise RuntimeError(f"{dataset_id}: expected 6D hand data, got state={hand_state.shape}, cmd={hand_cmd.shape}")

    source_range = hand_mapping.get("source_range")
    if source_range == [0.0, 1.0] or source_range == [0, 1]:
        hand_state = np.clip(hand_state, 0.0, 1.0)
        hand_cmd = np.clip(hand_cmd, 0.0, 1.0)

    return {
        "dataset_id": dataset_id,
        "info_path": info_path,
        "robot_type": info.get("robot_type"),
        "fps": info.get("fps"),
        "episode_index": episode,
        "frame_index": frame,
        "side": np.asarray(side, dtype=object),
        "hand_state": hand_state.astype(np.float32),
        "hand_cmd": hand_cmd.astype(np.float32),
    }


def _safe_dataset_name(dataset_id: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "__", dataset_id)


def prepare(config_path: Path, output_dir: Path, dataset_ids: set[str] | None, cache_dir: str | None) -> dict[str, Any]:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, Any] = {
        "schema_version": "rh56_external_hand_sequences_v0.1",
        "config": str(config_path),
        "canonical_hand_order": config["canonical_hand_order"],
        "datasets": [],
    }
    for entry in config.get("datasets", []):
        dataset_id = entry["id"]
        if dataset_ids is not None and dataset_id not in dataset_ids:
            continue
        training_use = entry.get("training_use") or {}
        if not training_use.get("hand_vq") and entry.get("priority") != "smoke_test":
            continue
        extracted = _extract_dataset(entry, cache_dir=cache_dir)
        name = _safe_dataset_name(dataset_id)
        out_path = output_dir / f"{name}.npz"
        np.savez_compressed(
            out_path,
            hand_state=extracted["hand_state"],
            hand_cmd=extracted["hand_cmd"],
            episode_index=extracted["episode_index"],
            frame_index=extracted["frame_index"],
            side=extracted["side"],
        )
        manifest["datasets"].append(
            {
                "id": dataset_id,
                "path": str(out_path),
                "robot_type": extracted["robot_type"],
                "fps": extracted["fps"],
                "frames": int(extracted["hand_state"].shape[0]),
                "episodes": int(len(np.unique(extracted["episode_index"]))),
                "side_counts": {
                    side: int((extracted["side"] == side).sum())
                    for side in sorted(set(extracted["side"].tolist()))
                },
                "hand_state_min": extracted["hand_state"].min(axis=0).round(6).tolist(),
                "hand_state_max": extracted["hand_state"].max(axis=0).round(6).tolist(),
                "hand_cmd_min": extracted["hand_cmd"].min(axis=0).round(6).tolist(),
                "hand_cmd_max": extracted["hand_cmd"].max(axis=0).round(6).tolist(),
            }
        )
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract canonical 6D RH56 hand sequences from configured external LeRobot datasets.")
    parser.add_argument("--config", default="configs/datasets/rh56_external_pretrain.yaml")
    parser.add_argument("--output-dir", default="data/external/rh56_hand_sequences")
    parser.add_argument("--dataset", action="append", help="Dataset id to extract. Repeat to select multiple. Defaults to all hand_vq datasets.")
    parser.add_argument("--cache-dir", default=None)
    args = parser.parse_args()
    dataset_ids = set(args.dataset) if args.dataset else None
    manifest = prepare(Path(args.config), Path(args.output_dir), dataset_ids, args.cache_dir)
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
