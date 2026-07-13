from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import coacd
import trimesh


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_DIR = PROJECT_ROOT / "data" / "sim_assets" / "meshes" / "rh56"
DEFAULT_OUT_DIR = PROJECT_ROOT / "data" / "sim_assets" / "meshes" / "rh56_collision_visual_coacd"

SOURCE_FILES: dict[str, str] = {
    "rh56_R_hand_base_link": "R_hand_base_link.STL",
    "rh56_R_thumb_proximal_base": "R_thumb_proximal_base.STL",
    "rh56_R_thumb_proximal": "R_thumb_proximal.STL",
    "rh56_R_thumb_intermediate": "R_thumb_intermediate.STL",
    "rh56_R_thumb_distal": "R_thumb_distal.STL",
    "rh56_R_index_proximal": "R_index_proximal.STL",
    "rh56_R_index_distal": "R_index_distal.STL",
    "rh56_R_middle_proximal": "R_middle_proximal.STL",
    "rh56_R_middle_distal": "R_middle_distal.STL",
    "rh56_R_ring_proximal": "R_ring_proximal.STL",
    "rh56_R_ring_distal": "R_ring_distal.STL",
    "rh56_R_pinky_proximal": "R_pinky_proximal.STL",
    "rh56_R_pinky_distal": "R_pinky_distal.STL",
}

MAX_HULLS_BY_BODY: dict[str, int] = {
    "rh56_R_hand_base_link": 24,
    "rh56_R_thumb_proximal_base": 4,
    "rh56_R_thumb_proximal": 14,
    "rh56_R_thumb_intermediate": 12,
    "rh56_R_thumb_distal": 8,
    "rh56_R_index_proximal": 12,
    "rh56_R_index_distal": 10,
    "rh56_R_middle_proximal": 12,
    "rh56_R_middle_distal": 10,
    "rh56_R_ring_proximal": 12,
    "rh56_R_ring_distal": 10,
    "rh56_R_pinky_proximal": 12,
    "rh56_R_pinky_distal": 10,
}


def _load_mesh(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected a single mesh from {path}")
    mesh.remove_unreferenced_vertices()
    return mesh


def _decompose(mesh: trimesh.Trimesh, *, threshold_m: float, max_hulls: int, seed: int) -> list[trimesh.Trimesh]:
    parts = coacd.run_coacd(
        coacd.Mesh(mesh.vertices, mesh.faces),
        threshold=threshold_m,
        real_metric=True,
        max_convex_hull=max_hulls,
        preprocess_mode="auto",
        preprocess_resolution=50,
        resolution=2000,
        mcts_nodes=10,
        mcts_iterations=80,
        mcts_max_depth=3,
        merge=True,
        decimate=False,
        max_ch_vertex=64,
        seed=seed,
    )
    result: list[trimesh.Trimesh] = []
    for vertices, faces in parts:
        part = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        part.remove_unreferenced_vertices()
        result.append(part)
    return result


def generate_collision_meshes(
    *,
    source_dir: Path,
    out_dir: Path,
    threshold_m: float,
    overwrite: bool,
    only_body: str | None,
    seed: int,
) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, Any] = {
        "generator": "tools/generate_rh56_visual_coacd_collision.py",
        "method": "coacd",
        "source_dir": str(source_dir.relative_to(PROJECT_ROOT)),
        "out_dir": str(out_dir.relative_to(PROJECT_ROOT)),
        "threshold_m": threshold_m,
        "seed": seed,
        "bodies": {},
    }

    for body_name, filename in SOURCE_FILES.items():
        if only_body is not None and body_name != only_body:
            continue
        source_path = source_dir / filename
        if not source_path.exists():
            raise FileNotFoundError(source_path)
        stem = source_path.stem
        existing = sorted(out_dir.glob(f"{stem}_part*.stl"))
        if existing and not overwrite:
            files = existing
            part_count = len(files)
            source_mesh = _load_mesh(source_path)
            input_faces = len(source_mesh.faces)
        else:
            for path in existing:
                path.unlink()
            source_mesh = _load_mesh(source_path)
            parts = _decompose(
                source_mesh,
                threshold_m=threshold_m,
                max_hulls=MAX_HULLS_BY_BODY[body_name],
                seed=seed,
            )
            files = []
            for index, part in enumerate(parts):
                out_path = out_dir / f"{stem}_part{index:03d}.stl"
                part.export(out_path)
                files.append(out_path)
            input_faces = len(source_mesh.faces)
            part_count = len(parts)

        manifest["bodies"][body_name] = {
            "source_file": str(source_path.relative_to(PROJECT_ROOT)),
            "input_faces": input_faces,
            "max_hulls": MAX_HULLS_BY_BODY[body_name],
            "part_count": part_count,
            "collision_files": [str(path.relative_to(PROJECT_ROOT)) for path in files],
        }

    manifest_path = out_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate RH56 collision meshes from visual STL files using CoACD convex decomposition."
    )
    parser.add_argument("--source-dir", type=Path, default=DEFAULT_SOURCE_DIR)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--threshold-m", type=float, default=0.002)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--only-body", choices=sorted(SOURCE_FILES), default=None)
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()

    manifest = generate_collision_meshes(
        source_dir=args.source_dir,
        out_dir=args.out_dir,
        threshold_m=args.threshold_m,
        overwrite=args.overwrite,
        only_body=args.only_body,
        seed=args.seed,
    )
    total_parts = sum(body["part_count"] for body in manifest["bodies"].values())
    print(f"wrote {manifest['out_dir']}/manifest.json")
    print(f"generated bodies={len(manifest['bodies'])} convex_parts={total_parts}")


if __name__ == "__main__":
    main()
