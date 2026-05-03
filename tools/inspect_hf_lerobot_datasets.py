from __future__ import annotations

import argparse
import json
import urllib.request
from pathlib import Path
from typing import Any

import yaml


HF_DATASET_API = "https://huggingface.co/api/datasets/{dataset_id}"
HF_RAW_URL = "https://huggingface.co/datasets/{dataset_id}/resolve/main/{path}"


def _read_json_url(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=30) as response:
        return json.loads(response.read().decode("utf-8"))


def _read_text_url(url: str) -> str:
    with urllib.request.urlopen(url, timeout=30) as response:
        return response.read().decode("utf-8")


def _candidate_info_paths(dataset: dict[str, Any]) -> list[str]:
    siblings = dataset.get("siblings") or []
    paths = [item.get("rfilename") for item in siblings if item.get("rfilename")]
    return sorted(path for path in paths if path.endswith("meta/info.json"))


def _load_dataset_info(dataset_id: str) -> tuple[dict[str, Any], dict[str, Any], str]:
    dataset = _read_json_url(HF_DATASET_API.format(dataset_id=dataset_id))
    info_paths = _candidate_info_paths(dataset)
    if not info_paths:
        raise RuntimeError(f"{dataset_id}: no meta/info.json found")
    info_path = info_paths[0]
    info = _read_json_url(HF_RAW_URL.format(dataset_id=dataset_id, path=info_path))
    return dataset, info, info_path


def _summarize_features(info: dict[str, Any]) -> dict[str, Any]:
    features = info.get("features") or {}
    return {
        name: {
            "dtype": feature.get("dtype"),
            "shape": feature.get("shape"),
            "names": feature.get("names"),
        }
        for name, feature in features.items()
    }


def inspect(config_path: Path) -> dict[str, Any]:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    results: list[dict[str, Any]] = []
    for entry in config.get("datasets", []):
        if entry.get("source") != "huggingface":
            continue
        dataset_id = entry["id"]
        dataset, info, info_path = _load_dataset_info(dataset_id)
        card = dataset.get("cardData") or {}
        results.append(
            {
                "id": dataset_id,
                "role": entry.get("role"),
                "priority": entry.get("priority"),
                "license": card.get("license"),
                "gated": dataset.get("gated"),
                "private": dataset.get("private"),
                "last_modified": dataset.get("lastModified"),
                "downloads": dataset.get("downloads"),
                "info_path": info_path,
                "robot_type": info.get("robot_type"),
                "fps": info.get("fps"),
                "total_episodes": info.get("total_episodes"),
                "total_frames": info.get("total_frames"),
                "total_tasks": info.get("total_tasks"),
                "features": _summarize_features(info),
            }
        )
    return {
        "config": str(config_path),
        "schema_version": config.get("schema_version"),
        "datasets": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect configured Hugging Face LeRobot datasets without downloading data files.")
    parser.add_argument("--config", default="configs/datasets/rh56_external_pretrain.yaml")
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    result = inspect(Path(args.config))
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
