from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import trimesh


DEFAULT_SPLITS = Path("data/external/maniskill_ycb_grasp_splits.json")
DEFAULT_OUTPUT_ROOT = Path("data/sim_assets/maniskill_ycb_mujoco")
DEFAULT_MANIFEST = Path("data/external/maniskill_ycb_mujoco_assets.json")


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Missing {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _as_float_triplet(values: np.ndarray) -> list[float]:
    return [round(float(value), 8) for value in values]


def _scaled_mesh(source: Path, scale: float) -> trimesh.Trimesh:
    mesh = trimesh.load(source, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"{source} did not load as a trimesh.Trimesh")
    mesh = mesh.copy()
    mesh.vertices = np.asarray(mesh.vertices, dtype=np.float64) * float(scale)
    return mesh


def _export_obj(mesh: trimesh.Trimesh, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(path)


def _compile_check(collision_obj: Path, density: float, bounds: np.ndarray) -> dict[str, Any]:
    collision_obj = collision_obj.resolve()
    body_z = -float(bounds[0, 2]) + 0.01
    xml = f"""
<mujoco model="ycb_mesh_compile_check">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81"/>
  <asset>
    <mesh name="object_collision_mesh" file="{collision_obj}"/>
  </asset>
  <worldbody>
    <geom name="table" type="box" pos="0 0 -0.01" size="0.25 0.25 0.01"/>
    <body name="object_body" pos="0 0 {body_z:.8f}">
      <freejoint/>
      <geom
        name="object_collision"
        type="mesh"
        mesh="object_collision_mesh"
        density="{density:.8f}"
        friction="1.8 0.08 0.004"
        condim="4"
      />
    </body>
  </worldbody>
</mujoco>
""".strip()
    with tempfile.TemporaryDirectory(prefix="mujoco_ycb_check_") as tmpdir:
        xml_path = Path(tmpdir) / "scene.xml"
        xml_path.write_text(xml, encoding="utf-8")
        model = mujoco.MjModel.from_xml_path(str(xml_path))
    return {
        "ok": True,
        "nmesh": int(model.nmesh),
        "ngeom": int(model.ngeom),
        "nbody": int(model.nbody),
        "body_z_for_table_top_0": round(body_z, 8),
    }


def _objects_from_splits(splits: dict[str, Any], split_names: list[str]) -> list[tuple[str, dict[str, Any]]]:
    rows: list[tuple[str, dict[str, Any]]] = []
    seen: set[str] = set()
    for split_name in split_names:
        for item in splits.get(split_name, []):
            object_id = item["id"]
            if object_id in seen:
                continue
            seen.add(object_id)
            rows.append((split_name, item))
    return rows


def prepare_assets(
    splits_path: Path,
    output_root: Path,
    split_names: list[str],
    *,
    compile_check: bool,
) -> dict[str, Any]:
    splits = _load_json(splits_path)
    source_root = Path(splits["source_root"])
    rows = _objects_from_splits(splits, split_names)
    prepared: list[dict[str, Any]] = []
    failures: list[dict[str, str]] = []

    for split_name, item in rows:
        object_id = item["id"]
        scale = float(item.get("scale", 1.0))
        density = float(item.get("density", 1000.0))
        source_dir = source_root / "models" / object_id
        collision_ply = source_dir / "collision.ply"
        visual_obj = source_dir / "textured.obj"
        out_dir = output_root / object_id
        collision_obj = out_dir / "collision.obj"
        metadata_path = out_dir / "metadata.json"

        try:
            if not collision_ply.exists():
                raise FileNotFoundError(collision_ply)
            mesh = _scaled_mesh(collision_ply, scale)
            _export_obj(mesh, collision_obj)
            bounds = np.asarray(mesh.bounds, dtype=np.float64)
            extents = np.asarray(mesh.extents, dtype=np.float64)
            check = _compile_check(collision_obj, density, bounds) if compile_check else {"ok": None}
            record = {
                "id": object_id,
                "split": split_name,
                "category": item.get("category", "other"),
                "source_collision": str(collision_ply),
                "source_visual": str(visual_obj) if visual_obj.exists() else None,
                "collision_obj": str(collision_obj),
                "scale_applied": scale,
                "density": density,
                "bbox_min_m": _as_float_triplet(bounds[0]),
                "bbox_max_m": _as_float_triplet(bounds[1]),
                "bbox_size_m": _as_float_triplet(extents),
                "table_body_z_offset_m": round(-float(bounds[0, 2]), 8),
                "mesh_vertices": int(len(mesh.vertices)),
                "mesh_faces": int(len(mesh.faces)),
                "mujoco_compile_check": check,
            }
            metadata_path.write_text(json.dumps(record, indent=2), encoding="utf-8")
            prepared.append(record)
        except Exception as exc:  # noqa: BLE001 - record per-object import failures.
            failures.append({"id": object_id, "split": split_name, "error": f"{type(exc).__name__}: {exc}"})

    return {
        "schema_version": "maniskill_ycb_mujoco_assets_v0.1",
        "source_splits": str(splits_path),
        "source_root": str(source_root),
        "output_root": str(output_root),
        "split_names": split_names,
        "counts": {
            "prepared": len(prepared),
            "failed": len(failures),
        },
        "objects": prepared,
        "failures": failures,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert ManiSkill YCB collision PLY assets into MuJoCo-loadable OBJ meshes.")
    parser.add_argument("--splits", default=str(DEFAULT_SPLITS))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--splits-to-prepare", nargs="+", default=["train", "heldout"], choices=["train", "heldout", "reserve"])
    parser.add_argument("--skip-compile-check", action="store_true")
    args = parser.parse_args()

    result = prepare_assets(
        Path(args.splits),
        Path(args.output_root),
        args.splits_to_prepare,
        compile_check=not args.skip_compile_check,
    )
    manifest = Path(args.manifest)
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "manifest": str(manifest),
                "output_root": result["output_root"],
                "split_names": result["split_names"],
                "counts": result["counts"],
                "failures": result["failures"][:5],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
