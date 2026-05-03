from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


DEFAULT_MANIFEST = Path("data/external/maniskill_ycb_manifest.json")

HELDOUT_IDS = {
    # Instance-level heldout with categories present in train.
    "009_gelatin_box",
    "010_potted_meat_can",
    "014_lemon",
    "025_mug",
    "040_large_marker",
    "055_baseball",
    "062_dice",
    "077_rubiks_cube",
    # Shape/category stress heldout.
    "011_banana",
    "019_pitcher_base",
    "030_fork",
    "037_scissors",
}

TRAIN_PREFERRED = {
    "002_master_chef_can",
    "003_cracker_box",
    "004_sugar_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
    "007_tuna_fish_can",
    "008_pudding_box",
    "012_strawberry",
    "013_apple",
    "015_peach",
    "016_pear",
    "017_orange",
    "018_plum",
    "021_bleach_cleanser",
    "024_bowl",
    "026_sponge",
    "036_wood_block",
    "043_phillips_screwdriver",
    "044_flat_screwdriver",
    "048_hammer",
    "053_mini_soccer_ball",
    "056_tennis_ball",
    "057_racquetball",
    "061_foam_brick",
    "065-c_cups",
    "065-e_cups",
    "065-g_cups",
    "070-a_colored_wood_blocks",
}


def _load_manifest(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Missing {path}. Run tools/inspect_maniskill_ycb_assets.py first.")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not data.get("available"):
        raise RuntimeError(f"YCB assets unavailable in {path}. Download with: .venv/bin/python -m mani_skill.utils.download_asset ycb")
    return data


def create_splits(manifest_path: Path) -> dict[str, Any]:
    manifest = _load_manifest(manifest_path)
    objects = [
        item
        for item in manifest["objects"]
        if not item["excluded_by_maniskill_pick_single"] and item["has_collision"] and item["has_visual"]
    ]
    by_id = {item["id"]: item for item in objects}
    train_ids = [model_id for model_id in sorted(TRAIN_PREFERRED) if model_id in by_id and model_id not in HELDOUT_IDS]
    heldout_ids = [model_id for model_id in sorted(HELDOUT_IDS) if model_id in by_id]
    reserve_ids = [
        item["id"]
        for item in objects
        if item["id"] not in set(train_ids) and item["id"] not in set(heldout_ids)
    ]

    def rows(ids: list[str]) -> list[dict[str, Any]]:
        return [by_id[model_id] for model_id in ids]

    def category_counts(items: list[dict[str, Any]]) -> dict[str, int]:
        counts: dict[str, int] = {}
        for item in items:
            counts[item["category"]] = counts.get(item["category"], 0) + 1
        return dict(sorted(counts.items()))

    train = rows(train_ids)
    heldout = rows(heldout_ids)
    reserve = rows(reserve_ids)
    return {
        "schema_version": "maniskill_ycb_grasp_splits_v0.1",
        "source_manifest": str(manifest_path),
        "source_root": manifest["root"],
        "policy": {
            "train": "common tabletop objects and categories likely available for real validation",
            "heldout": "mix of held-out instances and harder shape/category stress objects",
            "reserve": "remaining YCB assets for later expansion",
        },
        "counts": {
            "train": len(train),
            "heldout": len(heldout),
            "reserve": len(reserve),
        },
        "category_counts": {
            "train": category_counts(train),
            "heldout": category_counts(heldout),
            "reserve": category_counts(reserve),
        },
        "train": train,
        "heldout": heldout,
        "reserve": reserve,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Create initial YCB object train/heldout splits for RH56 grasp simulation.")
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--output", default="data/external/maniskill_ycb_grasp_splits.json")
    args = parser.parse_args()
    result = create_splits(Path(args.manifest))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(output), "counts": result["counts"], "category_counts": result["category_counts"]}, indent=2))


if __name__ == "__main__":
    main()
