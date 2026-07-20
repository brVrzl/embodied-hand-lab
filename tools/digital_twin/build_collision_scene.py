#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json
from digital_twin.registration.transforms import quaternion_xyzw_to_matrix


SHAPES = {"plane", "box", "cylinder", "capsule", "convex_mesh"}


def validate_collision_config(data: dict, base_dir: Path | None = None) -> list[dict]:
    frame = data.get("frame")
    if data.get("units") != "meter" or frame not in {"B", "P"}:
        raise ValueError("Collision scene must declare units: meter and frame: B or P.")
    pose_key = f"pose_in_{frame}"
    names = set()
    output = []
    for index, item in enumerate(data.get("objects", [])):
        missing = [key for key in ("name", "shape_type", "dimensions", pose_key, "source", "status", "uncertainty_m", "collision", "provisional") if key not in item]
        if missing:
            raise ValueError(f"Object {index} is missing: {', '.join(missing)}")
        if item["name"] in names or item["shape_type"] not in SHAPES:
            raise ValueError(f"Object {index} has duplicate name or unsupported shape.")
        names.add(item["name"])
        pose = item[pose_key]
        translation = np.asarray(pose.get("translation_m"), dtype=float)
        quaternion = np.asarray(pose.get("quaternion_xyzw"), dtype=float)
        dimensions = np.asarray(item["dimensions"], dtype=float)
        if translation.shape != (3,) or quaternion.shape != (4,) or dimensions.ndim != 1 or np.any(dimensions <= 0):
            raise ValueError(f"Object {item['name']} has invalid pose or dimensions.")
        quaternion_xyzw_to_matrix(quaternion)
        if item["shape_type"] == "convex_mesh" and not item.get("mesh_path"):
            raise ValueError(f"Convex mesh {item['name']} needs mesh_path.")
        if item["shape_type"] == "convex_mesh":
            validation = item.get("mesh_validation", {})
            required_flags = ("deliberately_simplified", "normals_verified", "watertight_verified")
            if not all(validation.get(flag) is True for flag in required_flags):
                raise ValueError(
                    f"Convex mesh {item['name']} requires true mesh_validation flags: {', '.join(required_flags)}."
                )
            mesh_path = Path(item["mesh_path"])
            resolved = mesh_path if mesh_path.is_absolute() or base_dir is None else base_dir / mesh_path
            if not resolved.is_file():
                raise ValueError(f"Convex mesh file does not exist: {resolved}")
            mesh = trimesh.load_mesh(resolved, process=False)
            if not isinstance(mesh, trimesh.Trimesh) or not mesh.is_watertight or not mesh.is_convex:
                raise ValueError(f"Collision mesh {item['name']} must be one watertight convex mesh.")
            if len(mesh.faces) > int(validation.get("maximum_faces", 5000)):
                raise ValueError(f"Collision mesh {item['name']} exceeds its simplified face-count limit.")
        output.append(dict(item))
    return output


def write_mjcf(path: Path, objects: list[dict], frame: str = "B") -> None:
    root = ET.Element("mujoco", {"model": "digital_twin_static_collision"})
    asset = ET.SubElement(root, "asset")
    worldbody = ET.SubElement(root, "worldbody")
    for item in objects:
        if not item["collision"]:
            continue
        pose = item[f"pose_in_{frame}"]
        xyzw = pose["quaternion_xyzw"]
        attributes = {
            "name": item["name"],
            "type": "mesh" if item["shape_type"] == "convex_mesh" else item["shape_type"],
            "pos": " ".join(map(str, pose["translation_m"])),
            "quat": " ".join(map(str, [xyzw[3], xyzw[0], xyzw[1], xyzw[2]])),
            "rgba": "0.3 0.7 0.9 0.35" if item["provisional"] else "0.5 0.5 0.5 1",
            "contype": "1", "conaffinity": "1",
        }
        if item["shape_type"] == "box":
            attributes["size"] = " ".join(str(float(value) / 2) for value in item["dimensions"])
        elif item["shape_type"] == "plane":
            attributes["size"] = " ".join(map(str, item["dimensions"]))
        elif item["shape_type"] in {"cylinder", "capsule"}:
            if len(item["dimensions"]) != 2:
                raise ValueError(f"{item['shape_type']} dimensions must be [radius, full_length].")
            attributes["size"] = f"{item['dimensions'][0]} {float(item['dimensions'][1]) / 2}"
        else:
            mesh_name = f"mesh_{item['name']}"
            ET.SubElement(asset, "mesh", {"name": mesh_name, "file": item["mesh_path"]})
            attributes["mesh"] = mesh_name
        ET.SubElement(worldbody, "geom", attributes)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate primitive/convex static collision objects and export JSON plus optional MJCF.")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mjcf-output", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        data = load_structured(args.config)
        objects = validate_collision_config(data, base_dir=args.config.parent)
        report = {"schema_version": 1, "units": "meter", "frame": data["frame"], "objects": objects, "dense_reconstruction_used_directly": False}
        if args.dry_run:
            print(json.dumps(report, indent=2)); return
        write_json(args.output, report)
        if args.mjcf_output:
            write_mjcf(args.mjcf_output, objects, data["frame"])
        print(f"Collision scene written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
