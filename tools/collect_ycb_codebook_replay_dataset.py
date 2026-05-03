from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rh56_handref_grasp_planner import (  # noqa: E402
    BASE_XML,
    OBJECT_CENTER_XY,
    ROBOT_CONFIG,
    TABLE_TOP_Z,
    _candidate_families,
    _hand_base_pose,
    _load_yaml,
    _object_center_from_plan,
    _physical_norm_to_mujoco_ctrl,
    _physical_norm_to_raw,
    _prepare_robot_xml,
    _replay_record,
    _rpy_matrix,
    _run_candidate,
    _shift_robot_base,
    _solve_hand_pose_target_q,
    _solve_lift_q,
    _solve_radial_approach_q,
)


DEFAULT_YCB_MANIFEST = Path("data/external/maniskill_ycb_mujoco_assets.json")
DEFAULT_CODEBOOK = Path("data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16_ordered.npz")
DEFAULT_OUT_DIR = Path("data/ycb_codebook_replay/rvqvae_ordered")
PLANNER_PHYSICAL_ORDER = ["pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"]


@dataclass(frozen=True)
class YCBMeshSpec:
    name: str
    dataset: str
    dataset_id: str
    display_name: str
    category: str
    family: str
    collision_obj: Path
    density: float
    bbox_min_m: tuple[float, float, float]
    bbox_max_m: tuple[float, float, float]
    bbox_size_m: tuple[float, float, float]
    mesh_contact_padding_m: float = 0.0
    collision_padding: float = 0.003
    pose_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    friction: str = "1.8 0.08 0.004"

    @property
    def half_height(self) -> float:
        return 0.5 * float(self.effective_bbox_size_m[2])

    @property
    def table_body_z_offset_m(self) -> float:
        return -float(self.effective_bbox_min_m[2])

    @property
    def planar_width(self) -> float:
        return float(min(self.effective_bbox_size_m[0], self.effective_bbox_size_m[1]))

    @property
    def mass(self) -> float:
        # Only for replay metadata. MuJoCo computes the actual mesh mass from density.
        volume_proxy = float(np.prod(np.asarray(self.effective_bbox_size_m, dtype=np.float64)))
        return max(0.001, volume_proxy * float(self.density) * 0.35)

    @property
    def pose_quat_string(self) -> str:
        return " ".join(f"{value:.8f}" for value in self.pose_quat)

    @property
    def mesh_scale(self) -> tuple[float, float, float]:
        if self.mesh_contact_padding_m <= 0:
            return (1.0, 1.0, 1.0)
        size = np.asarray(self.bbox_size_m, dtype=np.float64)
        scale = (size + 2.0 * float(self.mesh_contact_padding_m)) / np.maximum(size, 1e-6)
        return tuple(float(value) for value in scale)

    @property
    def effective_bbox_min_m(self) -> tuple[float, float, float]:
        scale = np.asarray(self.mesh_scale, dtype=np.float64)
        return tuple(float(value) for value in np.asarray(self.bbox_min_m, dtype=np.float64) * scale)

    @property
    def effective_bbox_max_m(self) -> tuple[float, float, float]:
        scale = np.asarray(self.mesh_scale, dtype=np.float64)
        return tuple(float(value) for value in np.asarray(self.bbox_max_m, dtype=np.float64) * scale)

    @property
    def effective_bbox_size_m(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in np.asarray(self.effective_bbox_max_m) - np.asarray(self.effective_bbox_min_m))


def _load_ycb_specs(path: Path, split: str, mesh_contact_padding_m: float) -> dict[str, YCBMeshSpec]:
    if not path.exists():
        raise FileNotFoundError(f"Missing {path}. Run tools/prepare_maniskill_ycb_mujoco_assets.py first.")
    manifest = json.loads(path.read_text(encoding="utf-8"))
    specs: dict[str, YCBMeshSpec] = {}
    for item in manifest["objects"]:
        if split != "all" and item["split"] != split:
            continue
        family = _infer_grasp_family(item["category"], np.asarray(item["bbox_size_m"], dtype=np.float64))
        specs[item["id"]] = YCBMeshSpec(
            name=item["id"],
            dataset="ManiSkill YCB mesh",
            dataset_id=item["id"],
            display_name=f"YCB {item['id']}",
            category=item["category"],
            family=family,
            collision_obj=Path(item["collision_obj"]),
            density=float(item["density"]),
            bbox_min_m=tuple(float(v) for v in item["bbox_min_m"]),
            bbox_max_m=tuple(float(v) for v in item["bbox_max_m"]),
            bbox_size_m=tuple(float(v) for v in item["bbox_size_m"]),
            mesh_contact_padding_m=float(mesh_contact_padding_m),
            collision_padding=_family_collision_padding(family),
        )
    return specs


def _infer_grasp_family(category: str, extents: np.ndarray) -> str:
    smallest = float(np.min(extents))
    largest = float(np.max(extents))
    elongation = largest / max(1e-6, smallest)
    flatness = smallest / max(1e-6, largest)
    if category in {"can", "bottle"}:
        return "cylinder_power_envelope"
    if category in {"ball"}:
        return "sphere_containment"
    if category == "fruit":
        return "thin_cylinder_tripod" if elongation > 2.4 else "sphere_containment"
    if category == "tool" or (elongation > 3.0 and smallest < 0.040):
        return "thin_cylinder_tripod"
    if category in {"box", "cup", "dish"}:
        return "box_power_envelope" if largest > 0.075 or flatness < 0.45 else "box_precision_pinch"
    return "box_precision_pinch" if largest < 0.070 else "box_power_envelope"


def _family_collision_padding(family: str) -> float:
    if family == "thin_cylinder_tripod":
        return 0.0025
    if family == "sphere_containment":
        return 0.003
    if family == "box_power_envelope":
        return 0.004
    return 0.003


def _load_codebook(path: Path, active_only: bool) -> tuple[np.ndarray, list[int], dict[str, Any]]:
    data = np.load(path, allow_pickle=True)
    centroids = np.asarray(data["centroids"], dtype=np.float64)
    if "canonical_hand_order" in data:
        source_order = [str(item) for item in np.asarray(data["canonical_hand_order"], dtype=object).tolist()]
        if source_order != PLANNER_PHYSICAL_ORDER:
            reorder = [source_order.index(name) for name in PLANNER_PHYSICAL_ORDER]
            centroids = centroids[:, reorder]
    code_indices = list(range(len(centroids)))
    if active_only and "active_indices" in data:
        code_indices = np.asarray(data["active_indices"], dtype=np.int64).tolist()
        centroids = centroids[code_indices]
    metadata_path = path.with_suffix(".json")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else {}
    return centroids, code_indices, metadata


def _nearest_code(target: np.ndarray, centroids: np.ndarray, code_indices: list[int], weights: np.ndarray) -> dict[str, Any]:
    dist = np.sum(((centroids - target[None, :]) * weights[None, :]) ** 2, axis=1)
    local_idx = int(np.argmin(dist))
    physical = np.asarray(centroids[local_idx], dtype=np.float64)
    return {
        "code_index": int(code_indices[local_idx]),
        "local_index": local_idx,
        "distance": float(dist[local_idx]),
        "physical_norm": physical.round(6).tolist(),
        "mujoco_ctrl": _physical_norm_to_mujoco_ctrl(physical).round(6).tolist(),
    }


def _ensure_asset(root: ET.Element) -> ET.Element:
    asset = root.find("asset")
    if asset is not None:
        return asset
    asset = ET.Element("asset")
    root.insert(0, asset)
    return asset


def _add_ycb_mesh_scene(root: ET.Element, spec: YCBMeshSpec, object_pos: np.ndarray, table_top_z: float) -> None:
    asset = _ensure_asset(root)
    mesh_name = f"bench_{spec.name}_collision"
    if not any(mesh.get("name") == mesh_name for mesh in asset.iter("mesh")):
        mesh_attrs = {"name": mesh_name, "file": str(spec.collision_obj.resolve())}
        if spec.mesh_contact_padding_m > 0:
            mesh_attrs["scale"] = " ".join(f"{value:.8f}" for value in spec.mesh_scale)
        ET.SubElement(asset, "mesh", mesh_attrs)

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Missing worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "bench_table",
            "type": "box",
            "pos": f"{object_pos[0]:.6f} {object_pos[1]:.6f} {table_top_z - 0.020:.6f}",
            "size": "0.42 0.32 0.020",
            "rgba": "0.72 0.66 0.56 1",
            "friction": "1.4 0.05 0.003",
            "condim": "4",
            "contype": "1",
            "conaffinity": "7",
        },
    )
    body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "bench_object_body",
            "pos": f"{object_pos[0]:.6f} {object_pos[1]:.6f} {object_pos[2]:.6f}",
            "quat": spec.pose_quat_string,
        },
    )
    ET.SubElement(body, "freejoint", {"name": "bench_object_freejoint"})
    ET.SubElement(
        body,
        "geom",
        {
            "name": "bench_object",
            "type": "mesh",
            "mesh": mesh_name,
            "density": f"{spec.density:.6f}",
            "rgba": "0.95 0.46 0.16 1",
            "friction": spec.friction,
            "condim": "4",
            "priority": "2",
            "contype": "1",
            "conaffinity": "6",
            "solref": "0.004 1",
            "solimp": "0.92 0.98 0.002",
            "group": "3",
        },
    )
    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": "bench_close_camera",
            "mode": "fixed",
            "pos": "-0.30 -0.76 1.10",
            "xyaxes": "0.96 -0.29 0 0.18 0.60 0.78",
            "fovy": "38",
        },
    )


def _write_ycb_scene_xml(
    base_xml: Path,
    out_xml: Path,
    spec: YCBMeshSpec,
    object_pos: np.ndarray,
    table_top_z: float,
    scene_z_offset: float,
) -> None:
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    root = _prepare_robot_xml(base_xml)
    _shift_robot_base(root, scene_z_offset)
    _add_ycb_mesh_scene(root, spec, object_pos, table_top_z)
    root.set("model", f"rh56_ycb_codebook_{spec.name}")
    ET.indent(root, space="  ")
    out_xml.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")


def _candidate_code_variants(
    candidate: dict[str, Any],
    centroids: np.ndarray,
    code_indices: list[int],
    *,
    mode: str,
    weights: np.ndarray,
) -> list[dict[str, Any]]:
    if mode == "target":
        physical = np.asarray(candidate["physical_close_norm"], dtype=np.float64)
        return [
            {
                "code_index": -1,
                "local_index": -1,
                "distance": 0.0,
                "physical_norm": physical.round(6).tolist(),
                "mujoco_ctrl": _physical_norm_to_mujoco_ctrl(physical).round(6).tolist(),
            }
        ]
    if mode == "nearest":
        return [_nearest_code(np.asarray(candidate["physical_close_norm"], dtype=np.float64), centroids, code_indices, weights)]
    return [
        {
            "code_index": int(code_indices[idx]),
            "local_index": idx,
            "distance": None,
            "physical_norm": np.asarray(centroid, dtype=np.float64).round(6).tolist(),
            "mujoco_ctrl": _physical_norm_to_mujoco_ctrl(centroid).round(6).tolist(),
        }
        for idx, centroid in enumerate(centroids)
    ]


def _record_for_candidate(object_name: str, spec: YCBMeshSpec, candidate: dict[str, Any], rank: int) -> dict[str, Any]:
    row = _replay_record(object_name, spec, candidate, rank)
    row["schema"] = "rh56_ycb_mesh_codebook_replay_v0"
    row["object_category"] = spec.category
    row["bbox_size_m"] = list(spec.bbox_size_m)
    row["collision_obj"] = str(spec.collision_obj)
    row["nearest_or_sampled_code"] = candidate["nearest_or_sampled_code"]
    row["target_physical_close_norm"] = candidate["target_physical_close_norm"]
    return row


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text("\n".join(json.dumps(row, sort_keys=True) for row in rows) + ("\n" if rows else ""), encoding="utf-8")


def collect(args: argparse.Namespace) -> dict[str, Any]:
    specs = _load_ycb_specs(Path(args.ycb_manifest), args.split, args.mesh_contact_padding)
    object_names = sorted(specs) if args.objects == ["all"] else args.objects
    missing = [name for name in object_names if name not in specs]
    if missing:
        raise ValueError(f"Unknown YCB object(s) for split={args.split}: {missing}; choices={sorted(specs)}")

    centroids, code_indices, codebook_metadata = _load_codebook(Path(args.codebook), active_only=not args.all_codes)
    weights = np.asarray(args.weights, dtype=np.float64)
    robot_cfg = _load_yaml(Path(args.robot_config))
    seed_grasp_q = np.asarray(robot_cfg["joint_presets"][args.arm_preset], dtype=np.float64)
    seed_hand_base_pos, seed_hand_base_rot = _hand_base_pose(Path(args.base_xml), seed_grasp_q)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    summary: dict[str, Any] = {
        "schema_version": "rh56_ycb_mesh_codebook_replay_v0.1",
        "method": "YCB mesh object-conditioned wrist sampling + RH56 DQ-RISE-style codebook close + MuJoCo validation",
        "ycb_manifest": str(args.ycb_manifest),
        "split": args.split,
        "codebook": str(args.codebook),
        "codebook_active_only": not args.all_codes,
        "active_indices": codebook_metadata.get("active_indices") if not args.all_codes else None,
        "hand_code_mode": args.hand_code_mode,
        "weights": weights.tolist(),
        "base_xml": str(args.base_xml),
        "seed_arm_preset": args.arm_preset,
        "object_center_xy_m": [args.object_x, args.object_y],
        "table_top_z_m": args.table_height,
        "lift_dz_m": args.lift_dz,
        "success_lift_m": args.success_lift,
        "objects": {},
    }
    replay_records: list[dict[str, Any]] = []

    for object_name in object_names:
        spec = specs[object_name]
        object_dir = out_dir / object_name
        object_dir.mkdir(parents=True, exist_ok=True)
        object_pos_unshifted = np.asarray(
            [
                float(args.object_x),
                float(args.object_y),
                spec.table_body_z_offset_m + float(args.table_clearance),
            ],
            dtype=np.float64,
        )
        planned_table_top_z = float(object_pos_unshifted[2] - spec.table_body_z_offset_m - args.table_clearance)
        scene_z_offset = float(args.table_height - planned_table_top_z)
        object_pos = object_pos_unshifted + np.asarray([0.0, 0.0, scene_z_offset], dtype=np.float64)
        table_top_z = float(args.table_height)

        results: list[dict[str, Any]] = []
        base_candidates = _candidate_families(spec)
        if args.candidate_name_contains:
            base_candidates = [item for item in base_candidates if args.candidate_name_contains in item["name"]]
        base_candidates = base_candidates[: args.max_base_candidates]
        for base_idx, candidate in enumerate(base_candidates):
            reference_object_center = _object_center_from_plan(Path(args.base_xml), seed_grasp_q, spec, candidate)
            target_hand_pos = (
                object_pos_unshifted
                + seed_hand_base_pos
                - reference_object_center
                + np.asarray(candidate["wrist_delta"], dtype=np.float64)
            )
            target_hand_rot = seed_hand_base_rot @ _rpy_matrix(candidate["wrist_rpy"])
            grasp_q, ik_error, ik_rot_error = _solve_hand_pose_target_q(
                Path(args.base_xml),
                seed_grasp_q,
                target_hand_pos,
                target_hand_rot,
            )
            approach_q = _solve_radial_approach_q(
                Path(args.base_xml),
                grasp_q,
                target_hand_pos,
                object_pos_unshifted,
                args.approach_distance,
                args.approach_dy,
            )
            lift_q = _solve_lift_q(Path(args.base_xml), grasp_q, args.lift_dz)
            rotate_ctrl = _physical_norm_to_mujoco_ctrl(candidate["physical_rotate_norm"])

            for code in _candidate_code_variants(candidate, centroids, code_indices, mode=args.hand_code_mode, weights=weights):
                if len(results) >= args.max_evals_per_object:
                    break
                eval_name = f"{candidate['name']}_c{code['code_index']:02d}"
                xml_path = object_dir / f"{eval_name}.xml"
                _write_ycb_scene_xml(Path(args.base_xml), xml_path, spec, object_pos, table_top_z, scene_z_offset)
                close_ctrl = np.asarray(code["mujoco_ctrl"], dtype=np.float64)
                result = _run_candidate(
                    xml_path,
                    approach_q,
                    grasp_q,
                    lift_q,
                    rotate_ctrl,
                    close_ctrl,
                    candidate["family"],
                    duration=args.duration,
                    success_lift_m=args.success_lift,
                )
                candidate_score = float(result["score"] - 60.0 * ik_error - 6.0 * ik_rot_error)
                results.append(
                    {
                        **candidate,
                        "name": eval_name,
                        "base_candidate_name": candidate["name"],
                        "base_candidate_index": base_idx,
                        "xml": str(xml_path),
                        "object_pos": object_pos.round(6).tolist(),
                        "object_pos_unshifted": object_pos_unshifted.round(6).tolist(),
                        "reference_object_center": reference_object_center.round(6).tolist(),
                        "target_hand_base_pos": target_hand_pos.round(6).tolist(),
                        "target_hand_base_rot": target_hand_rot.round(6).tolist(),
                        "ik_error_m": ik_error,
                        "ik_rot_error": ik_rot_error,
                        "candidate_score": candidate_score,
                        "table_top_z": table_top_z,
                        "planned_table_top_z": planned_table_top_z,
                        "scene_z_offset": scene_z_offset,
                        "physical_rotate_raw": _physical_norm_to_raw(candidate["physical_rotate_norm"]),
                        "physical_close_raw": _physical_norm_to_raw(code["physical_norm"]),
                        "target_physical_close_norm": candidate["physical_close_norm"],
                        "physical_close_norm": code["physical_norm"],
                        "grasp_q": grasp_q.round(6).tolist(),
                        "approach_q": approach_q.round(6).tolist(),
                        "lift_q": lift_q.round(6).tolist(),
                        "rotate_ctrl_mujoco": rotate_ctrl.round(6).tolist(),
                        "close_ctrl_mujoco": close_ctrl.round(6).tolist(),
                        "nearest_or_sampled_code": code,
                        "result": result,
                    }
                )
            if len(results) >= args.max_evals_per_object:
                break

        results.sort(key=lambda item: (item["result"]["success"], item["candidate_score"]), reverse=True)
        replay_records.extend(_record_for_candidate(object_name, spec, item, rank) for rank, item in enumerate(results))
        object_summary = {
            "spec": {
                "dataset": spec.dataset,
                "dataset_id": spec.dataset_id,
                "display_name": spec.display_name,
                "category": spec.category,
                "family": spec.family,
                "bbox_size_m": list(spec.bbox_size_m),
                "effective_bbox_size_m": list(spec.effective_bbox_size_m),
                "bbox_min_m": list(spec.bbox_min_m),
                "bbox_max_m": list(spec.bbox_max_m),
                "mesh_contact_padding_m": spec.mesh_contact_padding_m,
                "mesh_scale": list(spec.mesh_scale),
                "planar_width_m": spec.planar_width,
                "table_body_z_offset_m": spec.table_body_z_offset_m,
                "density": spec.density,
                "collision_obj": str(spec.collision_obj),
            },
            "num_candidates": len(results),
            "num_success": sum(1 for item in results if item["result"]["success"]),
            "top_candidates": results[: min(10, len(results))],
        }
        (object_dir / "candidates.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
        (object_dir / "summary.json").write_text(json.dumps(object_summary, indent=2), encoding="utf-8")
        summary["objects"][object_name] = object_summary

    summary["overall"] = {
        "objects": len(summary["objects"]),
        "evaluated": len(replay_records),
        "successes": int(sum(row["success"] for row in replay_records)),
        "mean_lift_m": float(np.mean([row["lift_m"] for row in replay_records])) if replay_records else 0.0,
        "mean_max_lift_m": float(np.mean([row["max_lift_m"] for row in replay_records])) if replay_records else 0.0,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    _write_jsonl(out_dir / "replay_dataset.jsonl", replay_records)
    best_presets = {
        "metadata": {
            name: {
                "success": item["top_candidates"][0]["result"]["success"] if item["top_candidates"] else False,
                "family": item["spec"]["family"],
                "category": item["spec"]["category"],
                "bbox_size_m": item["spec"]["bbox_size_m"],
                "best_code": item["top_candidates"][0]["nearest_or_sampled_code"] if item["top_candidates"] else None,
                "best_xml": item["top_candidates"][0]["xml"] if item["top_candidates"] else None,
            }
            for name, item in summary["objects"].items()
        },
        "hand_raw": {
            name: item["top_candidates"][0]["physical_close_raw"]
            for name, item in summary["objects"].items()
            if item["top_candidates"]
        },
        "arm_q": {
            name: {
                "approach": item["top_candidates"][0]["approach_q"],
                "grasp": item["top_candidates"][0]["grasp_q"],
                "lift": item["top_candidates"][0]["lift_q"],
            }
            for name, item in summary["objects"].items()
            if item["top_candidates"]
        },
    }
    (out_dir / "best_presets.yaml").write_text(yaml.safe_dump(best_presets, sort_keys=True), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect RH56 codebook replay labels on converted ManiSkill YCB MuJoCo meshes.")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--robot-config", default=str(ROBOT_CONFIG))
    parser.add_argument("--ycb-manifest", default=str(DEFAULT_YCB_MANIFEST))
    parser.add_argument("--codebook", default=str(DEFAULT_CODEBOOK))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    parser.add_argument("--split", default="train", choices=["train", "heldout", "all"])
    parser.add_argument("--objects", nargs="+", default=["002_master_chef_can", "008_pudding_box", "013_apple"])
    parser.add_argument("--arm-preset", default="pinch_grasp_box_v2")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--approach-dy", type=float, default=0.0)
    parser.add_argument("--approach-distance", type=float, default=0.0)
    parser.add_argument("--lift-dz", type=float, default=0.120)
    parser.add_argument("--success-lift", type=float, default=0.050)
    parser.add_argument("--object-x", type=float, default=OBJECT_CENTER_XY[0])
    parser.add_argument("--object-y", type=float, default=OBJECT_CENTER_XY[1])
    parser.add_argument("--table-height", type=float, default=TABLE_TOP_Z)
    parser.add_argument("--table-clearance", type=float, default=0.004)
    parser.add_argument("--mesh-contact-padding", type=float, default=0.0)
    parser.add_argument("--max-base-candidates", type=int, default=16)
    parser.add_argument("--max-evals-per-object", type=int, default=16)
    parser.add_argument("--candidate-name-contains", default="")
    parser.add_argument("--hand-code-mode", choices=["target", "nearest", "active"], default="nearest")
    parser.add_argument("--all-codes", action="store_true", help="Use all ordered codebook entries instead of active subset.")
    parser.add_argument(
        "--weights",
        nargs=6,
        type=float,
        default=[1.0, 1.0, 1.0, 1.0, 1.0, 1.8],
        metavar=("PINKY", "RING", "MIDDLE", "INDEX", "THUMB_CLOSE", "THUMB_LAT"),
    )
    args = parser.parse_args()
    summary = collect(args)
    compact = {
        "out_dir": args.out_dir,
        "overall": summary["overall"],
        "objects": {
            name: {
                "category": item["spec"]["category"],
                "family": item["spec"]["family"],
                "num_candidates": item["num_candidates"],
                "num_success": item["num_success"],
                "best_success": item["top_candidates"][0]["result"]["success"] if item["top_candidates"] else None,
                "best_lift_m": item["top_candidates"][0]["result"]["lift_m"] if item["top_candidates"] else None,
                "best_code": item["top_candidates"][0]["nearest_or_sampled_code"]["code_index"] if item["top_candidates"] else None,
                "best_failure_mode": item["top_candidates"][0]["result"]["failure_mode"] if item["top_candidates"] else None,
            }
            for name, item in summary["objects"].items()
        },
    }
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
