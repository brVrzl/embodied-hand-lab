from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


CANONICAL_HAND_ORDER = ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"]


def _load_json(path: Path) -> dict[str, Any]:
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {}


def _label_code(code: np.ndarray, *, contact_total: int = 0, occupancy: float = 0.0) -> tuple[str, str]:
    finger_mean = float(code[:4].mean())
    close_mean = float(code[:5].mean())
    thumb_close = float(code[4])
    thumb_lateral = float(code[5])
    risk = "risky_but_allowed" if contact_total > 0 else "clean"

    if close_mean < 0.08 and thumb_lateral < 0.3:
        return "open", risk
    if close_mean < 0.18 and thumb_lateral >= 0.7:
        return "thumb_opposition_open", risk
    if thumb_lateral >= 0.7 and close_mean < 0.35:
        return "lateral_preshape", risk
    if thumb_lateral >= 0.55 and close_mean < 0.65:
        return "lateral_pinch", risk
    if thumb_lateral >= 0.55:
        return "lateral_power", risk
    if close_mean < 0.28:
        return "light_close", risk
    if close_mean < 0.48:
        return "soft_close", risk
    if close_mean < 0.68:
        return "mid_close", risk
    if thumb_close > 0.75 or occupancy > 0.2:
        return "hold_close", risk
    return "strong_close", risk


def _sort_indices(centroids: np.ndarray, occupancy: np.ndarray, contacts: dict[int, int]) -> list[int]:
    def group(idx: int) -> tuple[int, float, float, float]:
        code = centroids[idx]
        label, _ = _label_code(code, contact_total=contacts.get(idx, 0), occupancy=float(occupancy[idx]))
        order = {
            "open": 0,
            "thumb_opposition_open": 1,
            "lateral_preshape": 2,
            "light_close": 3,
            "soft_close": 4,
            "mid_close": 5,
            "lateral_pinch": 6,
            "strong_close": 7,
            "hold_close": 8,
            "lateral_power": 9,
        }.get(label, 99)
        return (order, float(code[:5].mean()), -float(occupancy[idx]), float(code[5]))

    return sorted(range(len(centroids)), key=group)


def _load_contacts(path: Path) -> dict[int, int]:
    data = _load_json(path)
    contacts: dict[int, int] = {}
    for item in data.get("results", []):
        contacts[int(item["code"])] = int((item.get("counts") or {}).get("total", 0))
    return contacts


def order_codebook(args: argparse.Namespace) -> dict[str, Any]:
    codebook_path = Path(args.codebook)
    data = np.load(codebook_path, allow_pickle=True)
    centroids = np.asarray(data["centroids"], dtype=np.float32)
    pairs = np.asarray(data["code_pairs"], dtype=np.int64) if "code_pairs" in data else np.asarray([[idx] for idx in range(len(centroids))], dtype=np.int64)
    occupancy = np.asarray(data["sampled_code_occupancy"], dtype=np.float32) if "sampled_code_occupancy" in data else np.zeros(len(centroids), dtype=np.float32)
    contacts = _load_contacts(Path(args.contacts)) if args.contacts else {}

    order = _sort_indices(centroids, occupancy, contacts)
    ordered = centroids[order]
    ordered_pairs = pairs[order]
    ordered_occupancy = occupancy[order]
    old_to_new = {old: new for new, old in enumerate(order)}
    labels: list[dict[str, Any]] = []
    active_new_indices: list[int] = []
    for new_idx, old_idx in enumerate(order):
        code = ordered[new_idx]
        contact_total = contacts.get(old_idx, 0)
        label, risk = _label_code(code, contact_total=contact_total, occupancy=float(ordered_occupancy[new_idx]))
        use = bool(risk == "clean" or label in {"lateral_preshape", "lateral_pinch", "lateral_power"})
        if label in {"strong_close"} and float(ordered_occupancy[new_idx]) == 0.0:
            use = False
        if label == "hold_close" and new_idx > 0:
            # Keep one high-close hold state; avoid duplicate saturated closes.
            prior_holds = [item for item in labels if item["label"] == "hold_close" and item["active"]]
            if prior_holds:
                use = False
        if use:
            active_new_indices.append(new_idx)
        labels.append(
            {
                "new_index": new_idx,
                "old_index": int(old_idx),
                "label": label,
                "risk": risk,
                "active": use,
                "occupancy": float(ordered_occupancy[new_idx]),
                "contact_total": int(contact_total),
                "code_pair": ordered_pairs[new_idx].astype(int).tolist(),
                "physical_norm": code.round(6).tolist(),
            }
        )

    active_centroids = ordered[active_new_indices]
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        centroids=ordered,
        active_centroids=active_centroids,
        active_indices=np.asarray(active_new_indices, dtype=np.int64),
        old_to_new=np.asarray([old_to_new[idx] for idx in range(len(centroids))], dtype=np.int64),
        code_pairs=ordered_pairs,
        sampled_code_occupancy=ordered_occupancy,
        canonical_hand_order=np.asarray(CANONICAL_HAND_ORDER, dtype=object),
    )
    manifest = {
        "schema_version": "rh56_ordered_hand_codebook_v0.1",
        "source_codebook": str(codebook_path),
        "source_contacts": args.contacts,
        "output": str(output),
        "canonical_hand_order": CANONICAL_HAND_ORDER,
        "active_count": len(active_new_indices),
        "active_indices": active_new_indices,
        "old_to_new": {str(k): int(v) for k, v in old_to_new.items()},
        "codes": labels,
    }
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Reorder and label RH56 hand codebook for policy use.")
    parser.add_argument("--codebook", default="data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz")
    parser.add_argument("--contacts", default="data/collision_diagnostics/rh56_codebook_dqrise_rvqvae_contacts_proxy.json")
    parser.add_argument("--output", default="data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16_ordered.npz")
    args = parser.parse_args()
    result = order_codebook(args)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
