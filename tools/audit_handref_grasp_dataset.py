from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


STRICT_IK_ERROR_M = 0.005
STRICT_IK_ROT_ERROR = 0.08
STRICT_MAX_XY_DISPLACEMENT_M = 0.055
STRICT_MAX_CONTACTS = 32


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _strict_reject_reasons(candidate: dict[str, Any], *, success_lift_m: float) -> list[str]:
    result = candidate["result"]
    contacts = result["final_contacts"]
    reasons: list[str] = []
    if not result["success"]:
        reasons.append(f"not_success:{result.get('failure_mode', 'unknown')}")
    if result.get("initial_penetration", False):
        reasons.append("initial_penetration")
    if float(result["lift_m"]) < success_lift_m:
        reasons.append("final_lift_below_threshold")
    if float(result["max_lift_m"]) < success_lift_m:
        reasons.append("max_lift_below_threshold")
    if contacts["object_table"] != 0:
        reasons.append("object_table_contact")
    if contacts["hand_table"] != 0:
        reasons.append("hand_table_contact")
    if contacts["hand_self"] != 0:
        reasons.append("hand_self_contact")
    if not result.get("family_contact_ok", False):
        reasons.append("bad_family_contact")
    if float(result["max_xy_displacement_m"]) > STRICT_MAX_XY_DISPLACEMENT_M:
        reasons.append("large_xy_displacement")
    if float(candidate["ik_error_m"]) > STRICT_IK_ERROR_M:
        reasons.append("large_ik_position_error")
    if float(candidate["ik_rot_error"]) > STRICT_IK_ROT_ERROR:
        reasons.append("large_ik_rotation_error")
    if int(contacts["total"]) > STRICT_MAX_CONTACTS:
        reasons.append("excessive_contact_count")
    if candidate["family"] == "box_precision_pinch" and contacts["object_ring_pinky"] > 0:
        reasons.append("precision_grasp_ring_pinky_overwrap")
    return reasons


def audit(args: argparse.Namespace) -> dict[str, Any]:
    benchmark_dir = Path(args.benchmark_dir)
    summary = _load_json(benchmark_dir / "benchmark_summary.json")
    success_lift_m = float(summary["success_lift_m"])
    report: dict[str, Any] = {
        "schema": "rh56_handref_dataset_audit_v0.1",
        "benchmark_dir": str(benchmark_dir),
        "success_lift_m": success_lift_m,
        "strict_criteria": {
            "ik_error_m_max": STRICT_IK_ERROR_M,
            "ik_rot_error_max": STRICT_IK_ROT_ERROR,
            "max_xy_displacement_m_max": STRICT_MAX_XY_DISPLACEMENT_M,
            "max_contact_count": STRICT_MAX_CONTACTS,
            "required": [
                "success=true",
                "failure_mode=success",
                "no initial penetration",
                "no object-table, hand-table, or hand-self contact at final state",
                "family_contact_ok=true",
                "final lift and max lift above success_lift_m",
            ],
        },
        "objects": {},
        "totals": {
            "objects": 0,
            "candidates": 0,
            "sim_success": 0,
            "strict_pass": 0,
            "strict_pass_objects": 0,
        },
    }
    all_reason_counts: Counter[str] = Counter()
    for object_name in sorted(summary["objects"]):
        object_dir = benchmark_dir / object_name
        candidates = _load_json(object_dir / "candidates.json")
        reason_counts: Counter[str] = Counter()
        strict_rows: list[dict[str, Any]] = []
        best_strict: dict[str, Any] | None = None
        for rank, candidate in enumerate(candidates):
            reasons = _strict_reject_reasons(candidate, success_lift_m=success_lift_m)
            if reasons:
                reason_counts.update(reasons)
                all_reason_counts.update(reasons)
            else:
                row = {
                    "rank": rank,
                    "name": candidate["name"],
                    "xml": candidate["xml"],
                    "lift_m": candidate["result"]["lift_m"],
                    "max_lift_m": candidate["result"]["max_lift_m"],
                    "candidate_score": candidate["candidate_score"],
                    "wrist_pose_name": candidate["wrist_pose_name"],
                    "physical_close_raw": candidate["physical_close_raw"],
                    "final_contacts": candidate["result"]["final_contacts"],
                }
                strict_rows.append(row)
                if best_strict is None:
                    best_strict = row
        sim_success = sum(1 for item in candidates if item["result"]["success"])
        report["objects"][object_name] = {
            "num_candidates": len(candidates),
            "sim_success": sim_success,
            "strict_pass": len(strict_rows),
            "strict_pass_rate": len(strict_rows) / max(1, len(candidates)),
            "best_strict": best_strict,
            "top_strict": strict_rows[: args.top_k],
            "reject_reasons": dict(reason_counts.most_common()),
        }
        report["totals"]["objects"] += 1
        report["totals"]["candidates"] += len(candidates)
        report["totals"]["sim_success"] += sim_success
        report["totals"]["strict_pass"] += len(strict_rows)
        if strict_rows:
            report["totals"]["strict_pass_objects"] += 1
    report["totals"]["strict_pass_rate"] = report["totals"]["strict_pass"] / max(1, report["totals"]["candidates"])
    report["reject_reasons"] = dict(all_reason_counts.most_common())
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Strictly audit RH56 hand-ref MuJoCo grasp candidates.")
    parser.add_argument("--benchmark-dir", default="data/mujoco_handref_grasps")
    parser.add_argument("--out", default="data/reports/rh56_handref_dataset_audit/report.json")
    parser.add_argument("--top-k", type=int, default=5)
    args = parser.parse_args()
    report = audit(args)
    compact = {
        "out": args.out,
        "totals": report["totals"],
        "objects": {
            name: {
                "sim_success": row["sim_success"],
                "strict_pass": row["strict_pass"],
                "best_strict": None if row["best_strict"] is None else row["best_strict"]["name"],
            }
            for name, row in report["objects"].items()
        },
        "top_reject_reasons": dict(list(report["reject_reasons"].items())[:8]),
    }
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
