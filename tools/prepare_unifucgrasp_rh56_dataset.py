from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from rh56_driver.unifucgrasp_mapping import mapping_metadata, parse_unifuc_inspire_target


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def _load_unifuc_file(path: Path) -> tuple[dict[str, Any], np.ndarray]:
    data = np.load(path, allow_pickle=True)
    if len(data) < 2:
        raise ValueError(f"{path} does not look like a UniFucGrasp npy file.")
    metadata = dict(data[0])
    payload = data[1]
    if isinstance(payload, np.ndarray):
        payload = payload.item()
    if not isinstance(payload, dict) or "rtj" not in payload:
        raise ValueError(f"{path} missing data[1]['rtj'].")
    rtj = np.asarray(payload["rtj"], dtype=np.float32)
    if rtj.ndim != 2:
        raise ValueError(f"{path} has invalid rtj shape: {rtj.shape}.")
    return metadata, rtj


def prepare_dataset(
    dataset_root: Path,
    output_dir: Path,
    *,
    mesh_root: Path | None = None,
    hand_name: str = "inspire",
    max_files: int | None = None,
    max_samples: int | None = None,
) -> dict[str, Any]:
    dataset_root = dataset_root.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    samples_path = output_dir / "samples.jsonl"
    manifest_path = output_dir / "manifest.json"
    if samples_path.exists():
        samples_path.unlink()

    npy_files = sorted(dataset_root.rglob("*.npy"))
    if max_files is not None:
        npy_files = npy_files[: max(0, max_files)]

    stats: dict[str, Any] = {
        "dataset_root": str(dataset_root),
        "mesh_root": str(mesh_root.resolve()) if mesh_root else None,
        "hand_name": hand_name,
        "files_seen": 0,
        "files_used": 0,
        "samples_written": 0,
        "skipped": {},
        "object_categories": {},
        "mapping": mapping_metadata(),
    }

    def skip(reason: str) -> None:
        stats["skipped"][reason] = int(stats["skipped"].get(reason, 0)) + 1

    with samples_path.open("a", encoding="utf-8") as handle:
        for npy_path in npy_files:
            stats["files_seen"] += 1
            try:
                metadata, rtj = _load_unifuc_file(npy_path)
            except Exception:
                skip("load_error")
                continue
            if metadata.get("hand_name") != hand_name:
                skip("other_hand")
                continue
            obj_class = str(metadata.get("obj_class", "unknown"))
            obj_name = str(metadata.get("obj_name", npy_path.stem))
            rtj_length = int(metadata.get("rtj_length", rtj.shape[1]))
            if rtj_length < 19:
                skip("target_too_short")
                continue
            stats["files_used"] += 1
            stats["object_categories"][obj_class] = int(stats["object_categories"].get(obj_class, 0)) + 1
            mesh_rel = f"{obj_class}/{obj_name}.obj"
            mesh_path = str((mesh_root / mesh_rel).resolve()) if mesh_root else None
            for grasp_index, target_q in enumerate(rtj):
                if max_samples is not None and stats["samples_written"] >= max_samples:
                    break
                pose = parse_unifuc_inspire_target(target_q[:19])
                sample = {
                    "source": "unifucgrasp",
                    "source_file": str(npy_path),
                    "grasp_index": grasp_index,
                    "object": {
                        "class": obj_class,
                        "name": obj_name,
                        "mesh_rel": mesh_rel,
                        "mesh_path": mesh_path,
                    },
                    "hand_name": hand_name,
                    "unifuc": {
                        "metadata": _jsonable(metadata),
                        "target_q_19": np.asarray(target_q[:19], dtype=np.float32).tolist(),
                        "position_m": pose.position_m,
                        "quat_wxyz": pose.quat_wxyz,
                        "joints_12d": pose.joints_12d,
                    },
                    "rh56": {
                        "canonical_norm": pose.rh56_canonical_norm,
                        "raw_order": pose.rh56_raw_order,
                    },
                    "training_targets": {
                        "wrist_pos_m": pose.position_m,
                        "wrist_quat_wxyz": pose.quat_wxyz,
                        "rh56_canonical_norm": pose.rh56_canonical_norm,
                    },
                }
                handle.write(json.dumps(sample, ensure_ascii=False) + "\n")
                stats["samples_written"] += 1
            if max_samples is not None and stats["samples_written"] >= max_samples:
                break

    manifest_path.write_text(json.dumps(stats, indent=2, ensure_ascii=False), encoding="utf-8")
    return stats


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert UniFucGrasp Inspire annotations into project RH56 training samples.")
    parser.add_argument("--dataset-root", required=True, help="Path to UniFucGrasp Grasps_Dataset train/test directory.")
    parser.add_argument("--mesh-root", default=None, help="Optional path to UniFucGrasp Obj_Data root.")
    parser.add_argument("--output-dir", default="data/exports/structured/unifucgrasp_rh56")
    parser.add_argument("--hand-name", default="inspire")
    parser.add_argument("--max-files", type=int, default=None)
    parser.add_argument("--max-samples", type=int, default=None)
    args = parser.parse_args()
    stats = prepare_dataset(
        Path(args.dataset_root),
        Path(args.output_dir),
        mesh_root=Path(args.mesh_root) if args.mesh_root else None,
        hand_name=args.hand_name,
        max_files=args.max_files,
        max_samples=args.max_samples,
    )
    print(json.dumps(stats, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
