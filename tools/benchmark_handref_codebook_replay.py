from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mujoco_rh56_grasp_benchmark import _physical_norm_to_mujoco_ctrl  # noqa: E402
from rh56_handref_grasp_planner import _run_candidate  # noqa: E402


DEFAULT_OBJECTS = ["foam_block_40mm", "light_cylinder_36mm", "light_can_50mm", "062_dice"]
DEFAULT_CODEBOOK = "data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16_ordered.npz"
DEFAULT_BENCHMARK_DIR = "data/mujoco_handref_grasps"
PLANNER_PHYSICAL_ORDER = ["pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"]


def _load_codebook(path: Path, *, active_only: bool) -> tuple[np.ndarray, list[int], dict[str, Any]]:
    data = np.load(path, allow_pickle=True)
    centroids = np.asarray(data["centroids"], dtype=np.float64)
    if "canonical_hand_order" in data:
        source_order = [str(item) for item in np.asarray(data["canonical_hand_order"], dtype=object).tolist()]
        if source_order != PLANNER_PHYSICAL_ORDER:
            reorder = [source_order.index(name) for name in PLANNER_PHYSICAL_ORDER]
            centroids = centroids[:, reorder]
    if active_only:
        active_indices = np.asarray(data["active_indices"], dtype=np.int64).tolist()
        code_centroids = centroids[active_indices]
        code_indices = active_indices
    else:
        code_centroids = centroids
        code_indices = list(range(len(centroids)))
    manifest_path = path.with_suffix(".json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    return code_centroids, code_indices, manifest


def _nearest_code(target: np.ndarray, centroids: np.ndarray, code_indices: list[int], weights: np.ndarray) -> dict[str, Any]:
    dist = np.sum(((centroids - target[None, :]) * weights[None, :]) ** 2, axis=1)
    local_idx = int(np.argmin(dist))
    return {
        "code_index": int(code_indices[local_idx]),
        "local_index": local_idx,
        "distance": float(dist[local_idx]),
        "physical_norm": centroids[local_idx].round(6).tolist(),
        "mujoco_ctrl": _physical_norm_to_mujoco_ctrl(centroids[local_idx]).round(6).tolist(),
    }


def _candidate_rows(summary_path: Path, max_candidates: int, successful_only: bool) -> list[dict[str, Any]]:
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    rows = []
    for rank, candidate in enumerate(summary.get("top_candidates", [])):
        if len(rows) >= max_candidates:
            break
        if successful_only and not bool(candidate.get("result", {}).get("success")):
            continue
        required = ["xml", "approach_q", "grasp_q", "lift_q", "rotate_ctrl_mujoco", "close_ctrl_mujoco", "physical_close_norm"]
        if not all(key in candidate for key in required):
            continue
        rows.append({"rank": rank, "candidate": candidate})
    return rows


def benchmark(args: argparse.Namespace) -> dict[str, Any]:
    codebook_path = Path(args.codebook)
    centroids, code_indices, codebook_manifest = _load_codebook(codebook_path, active_only=not args.all_codes)
    weights = np.asarray(args.weights, dtype=np.float64)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    objects: dict[str, Any] = {}
    all_rows: list[dict[str, Any]] = []
    for object_name in args.objects:
        summary_path = Path(args.benchmark_dir) / object_name / "summary.json"
        if not summary_path.exists():
            objects[object_name] = {"error": f"missing {summary_path}"}
            continue
        rows = _candidate_rows(summary_path, args.max_candidates, args.successful_only)
        object_results: list[dict[str, Any]] = []
        for row in rows:
            rank = row["rank"]
            candidate = row["candidate"]
            target = np.asarray(candidate["physical_close_norm"], dtype=np.float64)
            nearest = _nearest_code(target, centroids, code_indices, weights)
            result = _run_candidate(
                Path(candidate["xml"]),
                np.asarray(candidate["approach_q"], dtype=np.float64),
                np.asarray(candidate["grasp_q"], dtype=np.float64),
                np.asarray(candidate["lift_q"], dtype=np.float64),
                np.asarray(candidate["rotate_ctrl_mujoco"], dtype=np.float64),
                np.asarray(nearest["mujoco_ctrl"], dtype=np.float64),
                candidate["family"],
                args.duration,
                float(candidate.get("result", {}).get("success_lift_m", args.success_lift)),
            )
            baseline = candidate.get("result", {})
            record = {
                "object": object_name,
                "rank": rank,
                "candidate_name": candidate["name"],
                "family": candidate["family"],
                "baseline_success": bool(baseline.get("success", False)),
                "baseline_lift_m": float(baseline.get("lift_m", 0.0)),
                "baseline_max_lift_m": float(baseline.get("max_lift_m", baseline.get("lift_m", 0.0))),
                "target_physical_close_norm": np.asarray(target).round(6).tolist(),
                "nearest_code": nearest,
                "code_success": bool(result["success"]),
                "code_lift_m": float(result["lift_m"]),
                "code_max_lift_m": float(result["max_lift_m"]),
                "code_failure_mode": result["failure_mode"],
                "code_final_contacts": result["final_contacts"],
                "code_initial_contacts": result["initial_contacts"],
            }
            object_results.append(record)
            all_rows.append(record)
        objects[object_name] = {
            "summary_path": str(summary_path),
            "evaluated": len(object_results),
            "baseline_successes": int(sum(row["baseline_success"] for row in object_results)),
            "code_successes": int(sum(row["code_success"] for row in object_results)),
            "mean_baseline_lift_m": float(np.mean([row["baseline_lift_m"] for row in object_results])) if object_results else 0.0,
            "mean_code_lift_m": float(np.mean([row["code_lift_m"] for row in object_results])) if object_results else 0.0,
            "records": object_results,
        }
    result = {
        "schema_version": "rh56_handref_codebook_replay_v0.1",
        "codebook": str(codebook_path),
        "active_only": not args.all_codes,
        "active_indices": codebook_manifest.get("active_indices") if not args.all_codes else None,
        "weights": weights.tolist(),
        "duration": args.duration,
        "successful_only": args.successful_only,
        "max_candidates": args.max_candidates,
        "objects": objects,
        "overall": {
            "evaluated": len(all_rows),
            "baseline_successes": int(sum(row["baseline_success"] for row in all_rows)),
            "code_successes": int(sum(row["code_success"] for row in all_rows)),
            "mean_baseline_lift_m": float(np.mean([row["baseline_lift_m"] for row in all_rows])) if all_rows else 0.0,
            "mean_code_lift_m": float(np.mean([row["code_lift_m"] for row in all_rows])) if all_rows else 0.0,
        },
    }
    output = out_dir / "summary.json"
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    (out_dir / "records.jsonl").write_text("\n".join(json.dumps(row, sort_keys=True) for row in all_rows) + ("\n" if all_rows else ""), encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay handref grasp candidates with nearest RH56 hand codebook close state.")
    parser.add_argument("--codebook", default=DEFAULT_CODEBOOK)
    parser.add_argument("--benchmark-dir", default=DEFAULT_BENCHMARK_DIR)
    parser.add_argument("--objects", nargs="+", default=DEFAULT_OBJECTS)
    parser.add_argument("--max-candidates", type=int, default=5)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--success-lift", type=float, default=0.05)
    parser.add_argument("--out-dir", default="data/codebook_replay/rvqvae_ordered_active_top5")
    parser.add_argument("--all-codes", action="store_true", help="Use all ordered codes instead of active subset.")
    parser.add_argument("--successful-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--weights",
        nargs=6,
        type=float,
        default=[1.0, 1.0, 1.0, 1.0, 1.0, 1.8],
        metavar=("INDEX", "MIDDLE", "RING", "PINKY", "THUMB_CLOSE", "THUMB_LAT"),
    )
    args = parser.parse_args()
    result = benchmark(args)
    print(json.dumps(result["overall"], indent=2))
    for name, item in result["objects"].items():
        if "error" in item:
            print(f"{name}: {item['error']}")
            continue
        print(
            f"{name}: evaluated={item['evaluated']} baseline={item['baseline_successes']} "
            f"code={item['code_successes']} mean_lift={item['mean_code_lift_m']:.4f}"
        )


if __name__ == "__main__":
    main()
