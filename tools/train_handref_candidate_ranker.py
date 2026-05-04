from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn


GEOM_TYPES = ["box", "cylinder", "sphere"]
FAMILIES = [
    "box_precision_pinch",
    "box_power_envelope",
    "cylinder_power_envelope",
    "thin_cylinder_tripod",
    "sphere_containment",
]


@dataclass
class Row:
    object_name: str
    candidate_name: str
    features: np.ndarray
    label: float
    lift_m: float
    candidate_score: float
    xml: str


class Ranker(nn.Module):
    def __init__(self, in_dim: int, hidden: int = 128) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.ReLU(),
            nn.LayerNorm(hidden),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x).squeeze(-1)


def _load_json(path: Path) -> dict[str, Any] | list[Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _pad(values: list[float], n: int) -> list[float]:
    out = [float(v) for v in values[:n]]
    return out + [0.0] * (n - len(out))


def _one_hot(value: str, choices: list[str]) -> list[float]:
    return [1.0 if value == item else 0.0 for item in choices]


def _features(object_name: str, spec: dict[str, Any], candidate: dict[str, Any]) -> np.ndarray:
    family = str(candidate.get("family") or spec["family"])
    values: list[float] = []
    values.extend(_one_hot(str(spec["geom_type"]), GEOM_TYPES))
    values.extend(_one_hot(family, FAMILIES))
    values.extend(_pad(spec.get("visual_size", []), 3))
    values.extend(_pad(spec.get("collision_size", []), 3))
    values.extend(
        [
            float(spec["mass"]),
            float(spec["planar_width_m"]),
            float(spec["collision_padding_m"]),
        ]
    )
    values.extend([float(v) for v in candidate["physical_rotate_norm"]])
    values.extend([float(v) for v in candidate["physical_close_norm"]])
    values.extend(_pad(candidate.get("object_offset", []), 3))
    values.append(float(candidate.get("z_drop", 0.0)))
    values.extend(_pad(candidate.get("wrist_delta", []), 3))
    values.extend(_pad(candidate.get("wrist_rpy", []), 3))
    values.extend(
        [
            float(candidate.get("target_proxy_width_m", 0.0)),
            float(candidate["ik_error_m"]),
            float(candidate["ik_rot_error"]),
        ]
    )
    return np.asarray(values, dtype=np.float32)


def load_rows(benchmark_dir: Path) -> list[Row]:
    summary = _load_json(benchmark_dir / "benchmark_summary.json")
    assert isinstance(summary, dict)
    rows: list[Row] = []
    for object_name, object_summary in summary["objects"].items():
        spec = object_summary["spec"]
        candidates = _load_json(benchmark_dir / object_name / "candidates.json")
        assert isinstance(candidates, list)
        for candidate in candidates:
            result = candidate["result"]
            rows.append(
                Row(
                    object_name=object_name,
                    candidate_name=candidate["name"],
                    features=_features(object_name, spec, candidate),
                    label=1.0 if result["success"] else 0.0,
                    lift_m=float(result["lift_m"]),
                    candidate_score=float(candidate["candidate_score"]),
                    xml=candidate["xml"],
                )
            )
    return rows


def _split_rows(rows: list[Row], val_objects: set[str]) -> tuple[list[Row], list[Row]]:
    train = [row for row in rows if row.object_name not in val_objects]
    val = [row for row in rows if row.object_name in val_objects]
    if not train or not val:
        objects = sorted({row.object_name for row in rows})
        fallback = {objects[-1]}
        train = [row for row in rows if row.object_name not in fallback]
        val = [row for row in rows if row.object_name in fallback]
    return train, val


def _standardize(train_x: np.ndarray, x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mean = train_x.mean(axis=0)
    std = train_x.std(axis=0) + 1e-6
    return (x - mean) / std, mean, std


def _auc(labels: np.ndarray, scores: np.ndarray) -> float | None:
    pos = scores[labels > 0.5]
    neg = scores[labels <= 0.5]
    if len(pos) == 0 or len(neg) == 0:
        return None
    wins = 0.0
    total = 0
    for p in pos:
        wins += float(np.sum(p > neg)) + 0.5 * float(np.sum(p == neg))
        total += len(neg)
    return float(wins / max(1, total))


def _ranking_metrics(rows: list[Row], scores: np.ndarray) -> dict[str, Any]:
    by_object: dict[str, list[tuple[Row, float]]] = {}
    for row, score in zip(rows, scores):
        by_object.setdefault(row.object_name, []).append((row, float(score)))
    object_metrics: dict[str, Any] = {}
    top1_successes = 0
    top5_success_objects = 0
    for object_name, items in by_object.items():
        ranked = sorted(items, key=lambda item: item[1], reverse=True)
        top1 = ranked[0][0]
        top5 = ranked[:5]
        top1_successes += int(top1.label > 0.5)
        top5_success_objects += int(any(row.label > 0.5 for row, _ in top5))
        object_metrics[object_name] = {
            "num_candidates": len(items),
            "num_success": int(sum(row.label > 0.5 for row, _ in items)),
            "top1_success": bool(top1.label > 0.5),
            "top1_candidate": top1.candidate_name,
            "top1_lift_m": top1.lift_m,
            "top5_has_success": bool(any(row.label > 0.5 for row, _ in top5)),
        }
    n = max(1, len(by_object))
    return {
        "top1_success_rate_by_object": top1_successes / n,
        "top5_success_rate_by_object": top5_success_objects / n,
        "objects": object_metrics,
    }


def train(args: argparse.Namespace) -> dict[str, Any]:
    rows = load_rows(Path(args.benchmark_dir))
    val_objects = set(args.val_objects)
    train_rows, val_rows = _split_rows(rows, val_objects)
    train_x = np.stack([row.features for row in train_rows], axis=0)
    val_x = np.stack([row.features for row in val_rows], axis=0)
    train_y = np.asarray([row.label for row in train_rows], dtype=np.float32)
    val_y = np.asarray([row.label for row in val_rows], dtype=np.float32)
    train_x_norm, mean, std = _standardize(train_x, train_x)
    val_x_norm = (val_x - mean) / std

    device = torch.device(args.device if args.device else ("cuda" if torch.cuda.is_available() else "cpu"))
    model = Ranker(train_x.shape[1], hidden=args.hidden).to(device)
    x_train = torch.from_numpy(train_x_norm).to(device)
    y_train = torch.from_numpy(train_y).to(device)
    x_val = torch.from_numpy(val_x_norm).to(device)
    y_val = torch.from_numpy(val_y).to(device)
    positives = float(train_y.sum())
    negatives = float(len(train_y) - positives)
    pos_weight = torch.tensor([negatives / max(1.0, positives)], dtype=torch.float32, device=device)
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=pos_weight)
    optim = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)

    best_state: dict[str, torch.Tensor] | None = None
    best_val_loss = float("inf")
    history: list[dict[str, float]] = []
    batch_size = min(args.batch_size, len(train_rows))
    for epoch in range(args.epochs):
        model.train()
        perm = torch.randperm(x_train.shape[0], device=device)
        for start in range(0, x_train.shape[0], batch_size):
            idx = perm[start : start + batch_size]
            loss = loss_fn(model(x_train[idx]), y_train[idx])
            optim.zero_grad()
            loss.backward()
            optim.step()
        if epoch % args.eval_every == 0 or epoch == args.epochs - 1:
            model.eval()
            with torch.no_grad():
                train_loss = float(loss_fn(model(x_train), y_train).item())
                val_loss = float(loss_fn(model(x_val), y_val).item())
            history.append({"epoch": float(epoch), "train_loss": train_loss, "val_loss": val_loss})
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
    if best_state is not None:
        model.load_state_dict(best_state)
    model.eval()
    with torch.no_grad():
        train_scores = torch.sigmoid(model(x_train)).detach().cpu().numpy()
        val_scores = torch.sigmoid(model(x_val)).detach().cpu().numpy()
    train_auc = _auc(train_y, train_scores)
    val_auc = _auc(val_y, val_scores)
    train_rank = _ranking_metrics(train_rows, train_scores)
    val_rank = _ranking_metrics(val_rows, val_scores)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "mean": mean,
            "std": std,
            "feature_dim": int(train_x.shape[1]),
            "geom_types": GEOM_TYPES,
            "families": FAMILIES,
        },
        out_dir / "model.pt",
    )
    prediction_rows = []
    for split, split_rows, scores in [("train", train_rows, train_scores), ("val", val_rows, val_scores)]:
        for row, score in zip(split_rows, scores):
            prediction_rows.append(
                {
                    "split": split,
                    "object": row.object_name,
                    "candidate": row.candidate_name,
                    "pred_success": float(score),
                    "label_success": bool(row.label > 0.5),
                    "lift_m": row.lift_m,
                    "candidate_score": row.candidate_score,
                    "xml": row.xml,
                }
            )
    (out_dir / "predictions.jsonl").write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in prediction_rows) + "\n",
        encoding="utf-8",
    )
    metrics = {
        "schema": "rh56_handref_ranker_baseline_v0.1",
        "benchmark_dir": args.benchmark_dir,
        "device": str(device),
        "train_objects": sorted({row.object_name for row in train_rows}),
        "val_objects": sorted({row.object_name for row in val_rows}),
        "num_train": len(train_rows),
        "num_val": len(val_rows),
        "train_positive_rate": float(train_y.mean()),
        "val_positive_rate": float(val_y.mean()),
        "best_val_loss": best_val_loss,
        "train_auc": train_auc,
        "val_auc": val_auc,
        "train_ranking": train_rank,
        "val_ranking": val_rank,
        "history": history,
    }
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a GPU-capable success ranker baseline for RH56 hand-ref candidates.")
    parser.add_argument("--benchmark-dir", default="data/mujoco_handref_grasps")
    parser.add_argument("--out-dir", default="data/baselines/rh56_handref_candidate_ranker")
    parser.add_argument("--val-objects", nargs="+", default=["light_can_50mm"])
    parser.add_argument("--device", default="")
    parser.add_argument("--hidden", type=int, default=128)
    parser.add_argument("--epochs", type=int, default=600)
    parser.add_argument("--eval-every", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    args = parser.parse_args()
    metrics = train(args)
    print(json.dumps({k: metrics[k] for k in ["device", "num_train", "num_val", "train_auc", "val_auc", "val_ranking"]}, indent=2))


if __name__ == "__main__":
    main()
