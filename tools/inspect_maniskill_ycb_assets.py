from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


MANISKILL_YCB_ROOT = Path("/home/w/.maniskill/data/assets/mani_skill2_ycb")
MANISKILL_PICK_SINGLE_EXCLUDES = {
    "022_windex_bottle",
    "028_skillet_lid",
    "029_plate",
    "059_chain",
}


def _rough_category(model_id: str) -> str:
    name = model_id.lower()
    if any(token in name for token in ["box", "cracker", "sugar", "gelatin", "pudding"]):
        return "box"
    if any(token in name for token in ["can", "chips"]):
        return "can"
    if any(token in name for token in ["bottle", "cleanser", "windex"]):
        return "bottle"
    if any(token in name for token in ["mug", "cup"]):
        return "cup"
    if any(token in name for token in ["bowl", "plate", "skillet"]):
        return "dish"
    if any(token in name for token in ["banana", "apple", "orange", "lemon", "pear", "strawberry", "peach", "plum"]):
        return "fruit"
    if any(token in name for token in ["marker", "screwdriver", "hammer", "scissors", "spoon", "fork", "knife"]):
        return "tool"
    if any(token in name for token in ["ball"]):
        return "ball"
    return "other"


def inspect_ycb(root: Path) -> dict[str, Any]:
    info_path = root / "info_pick_v0.json"
    if not info_path.exists():
        return {
            "available": False,
            "root": str(root),
            "missing": str(info_path),
            "download_command": ".venv/bin/python -m mani_skill.utils.download_asset ycb",
        }
    info = json.loads(info_path.read_text(encoding="utf-8"))
    objects: list[dict[str, Any]] = []
    for model_id, metadata in sorted(info.items()):
        model_dir = root / "models" / model_id
        bbox = metadata.get("bbox") or {}
        bbox_min = bbox.get("min") or [0, 0, 0]
        bbox_max = bbox.get("max") or [0, 0, 0]
        scale = (metadata.get("scales") or [1.0])[0]
        size = [round((float(hi) - float(lo)) * float(scale), 6) for lo, hi in zip(bbox_min, bbox_max)]
        objects.append(
            {
                "id": model_id,
                "category": _rough_category(model_id),
                "excluded_by_maniskill_pick_single": model_id in MANISKILL_PICK_SINGLE_EXCLUDES,
                "density": metadata.get("density", 1000),
                "scale": scale,
                "bbox_size_m": size,
                "has_collision": (model_dir / "collision.ply").exists(),
                "has_visual": (model_dir / "textured.obj").exists(),
            }
        )
    by_category: dict[str, int] = {}
    for item in objects:
        by_category[item["category"]] = by_category.get(item["category"], 0) + 1
    return {
        "available": True,
        "root": str(root),
        "total_objects": len(objects),
        "usable_pick_single": sum(not item["excluded_by_maniskill_pick_single"] for item in objects),
        "by_category": dict(sorted(by_category.items())),
        "objects": objects,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect local ManiSkill YCB object assets.")
    parser.add_argument("--root", default=str(MANISKILL_YCB_ROOT))
    parser.add_argument("--output", default="data/external/maniskill_ycb_manifest.json")
    args = parser.parse_args()
    result = inspect_ycb(Path(args.root))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "objects"}, indent=2))
    if result.get("available"):
        print(f"wrote {output}")


if __name__ == "__main__":
    main()
