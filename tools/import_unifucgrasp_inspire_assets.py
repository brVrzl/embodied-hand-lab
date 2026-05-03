from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


UNIFUC_INSPIRE_ACTIVE_JOINT_ORDER = [
    "right_thumb_1_joint",
    "right_thumb_2_joint",
    "right_thumb_3_joint",
    "right_thumb_4_joint",
    "right_index_1_joint",
    "right_index_2_joint",
    "right_middle_1_joint",
    "right_middle_2_joint",
    "right_ring_1_joint",
    "right_ring_2_joint",
    "right_little_1_joint",
    "right_little_2_joint",
]

PROJECT_RH56_CANONICAL_ORDER = [
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
]

UNIFUC_TO_RH56_NOTES = {
    "index": ["right_index_1_joint", "right_index_2_joint"],
    "middle": ["right_middle_1_joint", "right_middle_2_joint"],
    "ring": ["right_ring_1_joint", "right_ring_2_joint"],
    "pinky": ["right_little_1_joint", "right_little_2_joint"],
    "thumb_close": ["right_thumb_2_joint", "right_thumb_3_joint", "right_thumb_4_joint"],
    "thumb_lateral": ["right_thumb_1_joint"],
}


def _copytree_contents(src: Path, dst: Path, *, overwrite: bool) -> int:
    if not src.exists():
        raise FileNotFoundError(src)
    dst.mkdir(parents=True, exist_ok=True)
    copied = 0
    for path in sorted(src.iterdir()):
        if not path.is_file():
            continue
        target = dst / path.name
        if target.exists() and not overwrite:
            continue
        shutil.copy2(path, target)
        copied += 1
    return copied


def _patch_urdf_mesh_paths(text: str) -> str:
    replacements = {
        "urdf_right_with_force_sensor/meshes/": "../meshes/",
        "package://urdf_right_with_force_sensor/meshes/": "../meshes/",
        "package://inspire/meshes/": "../meshes/",
    }
    patched = text
    for old, new in replacements.items():
        patched = patched.replace(old, new)
    return patched


def import_assets(unifuc_root: Path, output_dir: Path, *, overwrite: bool = False) -> dict[str, object]:
    unifuc_root = unifuc_root.resolve()
    output_dir = output_dir.resolve()
    if not unifuc_root.exists():
        raise FileNotFoundError(f"UniFucGrasp root not found: {unifuc_root}")

    source_urdf = unifuc_root / "assets" / "yinshi_meshes" / "urdf_right_with_force_sensor_no_xml.urdf"
    source_mesh_dir = unifuc_root / "data" / "data_hand" / "inspire" / "meshes"
    if not source_mesh_dir.exists():
        source_mesh_dir = unifuc_root / "assets" / "yinshi_meshes"
    source_pointcloud = unifuc_root / "data" / "Pointhand_down" / "inspire.pth"
    source_license = unifuc_root / "LICENSE"

    if not source_urdf.exists():
        raise FileNotFoundError(f"UniFucGrasp Inspire URDF not found: {source_urdf}")
    if not source_mesh_dir.exists():
        raise FileNotFoundError(f"UniFucGrasp Inspire meshes not found: {source_mesh_dir}")

    urdf_dir = output_dir / "urdf"
    mesh_dir = output_dir / "meshes"
    pc_dir = output_dir / "pointcloud"
    urdf_dir.mkdir(parents=True, exist_ok=True)
    pc_dir.mkdir(parents=True, exist_ok=True)

    copied_meshes = _copytree_contents(source_mesh_dir, mesh_dir, overwrite=overwrite)

    target_urdf = urdf_dir / "inspire_right_force_sensor.urdf"
    if overwrite or not target_urdf.exists():
        patched_urdf = _patch_urdf_mesh_paths(source_urdf.read_text(encoding="utf-8"))
        target_urdf.write_text(patched_urdf, encoding="utf-8")

    copied_pointcloud = False
    target_pointcloud = pc_dir / "inspire.pth"
    if source_pointcloud.exists() and (overwrite or not target_pointcloud.exists()):
        shutil.copy2(source_pointcloud, target_pointcloud)
        copied_pointcloud = True

    copied_license = False
    target_license = output_dir / "UNIFUCGRASP_LICENSE"
    if source_license.exists() and (overwrite or not target_license.exists()):
        shutil.copy2(source_license, target_license)
        copied_license = True

    manifest = {
        "source": {
            "repo": "https://github.com/cxcAxxy/UniFucGrasp",
            "root": str(unifuc_root),
            "urdf": str(source_urdf),
            "mesh_dir": str(source_mesh_dir),
            "pointcloud": str(source_pointcloud) if source_pointcloud.exists() else None,
        },
        "imported": {
            "output_dir": str(output_dir),
            "urdf": str(target_urdf),
            "mesh_dir": str(mesh_dir),
            "pointcloud": str(target_pointcloud) if target_pointcloud.exists() else None,
            "copied_mesh_count": copied_meshes,
            "copied_pointcloud": copied_pointcloud,
            "copied_license": copied_license,
        },
        "license_note": "UniFucGrasp repository is MIT-licensed at time of inspection; retain the copied license when using these assets.",
        "active_joint_order_12d": UNIFUC_INSPIRE_ACTIVE_JOINT_ORDER,
        "project_rh56_canonical_order_6d": PROJECT_RH56_CANONICAL_ORDER,
        "unifuc_to_rh56_mapping_notes": UNIFUC_TO_RH56_NOTES,
        "collision_use": {
            "recommended": "Use as collision/mesh reference and fingertip pad geometry source; do not drop in as the main JAKA+RH56 model without mount and joint-order validation.",
            "why": [
                "UniFucGrasp asset includes tactile/force sensor meshes and richer fingertip surfaces.",
                "Project model already has a calibrated JAKA mount and RH56 actuator ordering.",
                "A/B testing against current proxy collision is safer than replacing the full hand model.",
            ],
        },
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Import UniFucGrasp InspireHand assets for local collision/model comparison.")
    parser.add_argument("--unifuc-root", default="/tmp/UniFucGrasp", help="Path to a cloned cxcAxxy/UniFucGrasp repository.")
    parser.add_argument("--output-dir", default="data/external/unifucgrasp_inspire", help="Destination inside this project.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite already imported files.")
    args = parser.parse_args()

    manifest = import_assets(Path(args.unifuc_root), Path(args.output_dir), overwrite=args.overwrite)
    print(json.dumps(manifest["imported"], indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
