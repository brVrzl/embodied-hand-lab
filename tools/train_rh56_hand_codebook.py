from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


CANONICAL_HAND_ORDER = ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"]

RH56_THUMB_LATERAL_ANCHORS = np.asarray(
    [
        [0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
        [0.00, 0.00, 0.00, 0.00, 0.00, 1.00],
        [0.00, 0.00, 0.12, 0.15, 0.40, 1.00],
        [0.10, 0.10, 0.55, 0.60, 0.68, 1.00],
        [0.75, 0.75, 0.80, 0.80, 0.55, 0.65],
    ],
    dtype=np.float32,
)


def _iter_npz(paths: list[Path]) -> list[Path]:
    files: list[Path] = []
    for path in paths:
        if path.is_dir():
            files.extend(sorted(path.glob("*.npz")))
        else:
            files.append(path)
    return files


def _load_sequences(paths: list[Path], source: str, *, allow_out_of_range: bool = False) -> tuple[np.ndarray, list[dict[str, Any]]]:
    arrays: list[np.ndarray] = []
    sources: list[dict[str, Any]] = []
    for path in _iter_npz(paths):
        data = np.load(path, allow_pickle=True)
        if source not in data:
            continue
        arr = np.asarray(data[source], dtype=np.float32)
        if arr.ndim != 2 or arr.shape[1] != 6:
            raise RuntimeError(f"{path}: expected {source} shape [N,6], got {arr.shape}")
        mask = np.isfinite(arr).all(axis=1)
        arr = arr[mask]
        raw_min = arr.min(axis=0)
        raw_max = arr.max(axis=0)
        if not allow_out_of_range and (float(raw_min.min()) < -0.05 or float(raw_max.max()) > 1.05):
            sources.append(
                {
                    "path": str(path),
                    "source": source,
                    "frames": int(arr.shape[0]),
                    "skipped": True,
                    "skip_reason": "out_of_normalized_range",
                    "min": raw_min.round(6).tolist(),
                    "max": raw_max.round(6).tolist(),
                }
            )
            continue
        arr = np.clip(arr, 0.0, 1.0)
        arrays.append(arr)
        sources.append(
            {
                "path": str(path),
                "source": source,
                "frames": int(arr.shape[0]),
                "skipped": False,
                "min": arr.min(axis=0).round(6).tolist(),
                "max": arr.max(axis=0).round(6).tolist(),
            }
        )
    if not arrays:
        raise RuntimeError(f"No valid {source} arrays found in {paths}")
    return np.concatenate(arrays, axis=0), sources


def _sample_rows(x: np.ndarray, max_samples: int, seed: int) -> np.ndarray:
    if x.shape[0] <= max_samples:
        return x
    rng = np.random.default_rng(seed)
    idx = rng.choice(x.shape[0], size=max_samples, replace=False)
    return x[idx]


def _init_centroids_pp(x: np.ndarray, k: int, rng: np.random.Generator) -> np.ndarray:
    centroids = np.empty((k, x.shape[1]), dtype=np.float32)
    first = int(rng.integers(0, x.shape[0]))
    centroids[0] = x[first]
    closest = np.sum((x - centroids[0]) ** 2, axis=1)
    for idx in range(1, k):
        total = float(closest.sum())
        if total <= 1e-12:
            centroids[idx] = x[int(rng.integers(0, x.shape[0]))]
            continue
        next_idx = int(rng.choice(x.shape[0], p=closest / total))
        centroids[idx] = x[next_idx]
        closest = np.minimum(closest, np.sum((x - centroids[idx]) ** 2, axis=1))
    return centroids


def _kmeans(x: np.ndarray, k: int, *, seed: int, iterations: int, weights: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    rng = np.random.default_rng(seed)
    xw = x * weights
    centroids = _init_centroids_pp(xw, k, rng)
    labels = np.zeros(x.shape[0], dtype=np.int64)
    for _ in range(iterations):
        dist = np.sum((xw[:, None, :] - centroids[None, :, :]) ** 2, axis=2)
        labels = np.argmin(dist, axis=1)
        new_centroids = centroids.copy()
        for cluster in range(k):
            mask = labels == cluster
            if mask.any():
                new_centroids[cluster] = xw[mask].mean(axis=0)
            else:
                new_centroids[cluster] = xw[int(rng.integers(0, xw.shape[0]))]
        shift = float(np.max(np.linalg.norm(new_centroids - centroids, axis=1)))
        centroids = new_centroids
        if shift < 1e-5:
            break
    dist = np.sum((xw[:, None, :] - centroids[None, :, :]) ** 2, axis=2)
    labels = np.argmin(dist, axis=1)
    inertia = float(np.mean(np.min(dist, axis=1)))
    return centroids / weights, labels, inertia


def _sort_codebook(centroids: np.ndarray) -> np.ndarray:
    # Stable, interpretable order: open-to-close, then thumb lateral.
    score = centroids[:, :5].mean(axis=1)
    order = np.lexsort((centroids[:, 5], score))
    return centroids[order]


def train_codebook(args: argparse.Namespace) -> dict[str, Any]:
    x, sources = _load_sequences([Path(path) for path in args.input], args.source, allow_out_of_range=args.allow_out_of_range)
    raw_stats = {
        "frames": int(x.shape[0]),
        "min": x.min(axis=0).round(6).tolist(),
        "max": x.max(axis=0).round(6).tolist(),
        "std": x.std(axis=0).round(6).tolist(),
    }
    sampled = _sample_rows(x, args.max_samples, args.seed)

    anchors = np.empty((0, 6), dtype=np.float32)
    if args.thumb_lateral_anchors:
        anchors = RH56_THUMB_LATERAL_ANCHORS.copy()
    train_x = sampled
    reserved = min(args.reserve_anchor_codes, anchors.shape[0]) if anchors.size else 0
    if reserved:
        k_data = args.k - reserved
        if k_data <= 0:
            raise ValueError("--k must be larger than reserved anchor codes")
    else:
        k_data = args.k

    weights = np.asarray([1.0, 1.0, 1.0, 1.0, 1.0, args.thumb_lateral_weight], dtype=np.float32)
    data_centroids, labels, inertia = _kmeans(train_x, k_data, seed=args.seed, iterations=args.iterations, weights=weights)
    if reserved:
        centroids = np.concatenate([data_centroids, anchors[:reserved]], axis=0)
    else:
        centroids = data_centroids
    centroids = np.clip(_sort_codebook(centroids), 0.0, 1.0).astype(np.float32)
    full_dist = np.sum(((sampled * weights)[:, None, :] - (centroids * weights)[None, :, :]) ** 2, axis=2)
    full_labels = np.argmin(full_dist, axis=1)
    full_counts = np.bincount(full_labels, minlength=args.k)
    full_occupancy = (full_counts / max(1, full_counts.sum())).astype(np.float64)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        centroids=centroids,
        canonical_hand_order=np.asarray(CANONICAL_HAND_ORDER, dtype=object),
        source=args.source,
        weights=weights,
        thumb_lateral_anchors=anchors,
    )
    manifest = {
        "schema_version": "rh56_hand_codebook_v0.1",
        "method": "numpy_kmeans_with_reserved_thumb_lateral_anchors" if reserved else "numpy_kmeans",
        "output": str(output),
        "k": int(args.k),
        "source": args.source,
        "canonical_hand_order": CANONICAL_HAND_ORDER,
        "raw_stats": raw_stats,
        "sampled_frames": int(sampled.shape[0]),
        "k_data": int(k_data),
        "reserved_anchor_codes": int(reserved),
        "thumb_lateral_weight": float(args.thumb_lateral_weight),
        "inertia_weighted": inertia,
        "sources": sources,
        "sampled_code_counts": full_counts.astype(int).tolist(),
        "sampled_code_occupancy": full_occupancy.round(6).tolist(),
        "centroids": centroids.round(6).tolist(),
    }
    manifest_path = output.with_suffix(".json")
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a first-pass RH56 6D hand-state codebook.")
    parser.add_argument("--input", action="append", required=True, help="NPZ file or directory containing extracted hand sequences.")
    parser.add_argument("--source", choices=["hand_state", "hand_cmd"], default="hand_state")
    parser.add_argument("--output", default="data/models/rh56_hand_codebook_unitree_state_k16.npz")
    parser.add_argument("--k", type=int, default=16)
    parser.add_argument("--max-samples", type=int, default=200000)
    parser.add_argument("--iterations", type=int, default=80)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--thumb-lateral-weight", type=float, default=4.0)
    parser.add_argument("--thumb-lateral-anchors", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reserve-anchor-codes", type=int, default=5)
    parser.add_argument("--allow-out-of-range", action="store_true", help="Allow and clip input sequences outside normalized 0-1 range.")
    args = parser.parse_args()
    result = train_codebook(args)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
