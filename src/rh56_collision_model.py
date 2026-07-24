from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


VISUAL_ONLY_ATTRS = {
    "contype": "0",
    "conaffinity": "0",
    "group": "1",
}

VISUAL_COACD_COLLISION_ATTRS = {
    "type": "mesh",
    "condim": "4",
    "priority": "2",
    "friction": "1.8 0.08 0.004",
    "solref": "0.004 1",
    "solimp": "0.92 0.98 0.002",
    "rgba": "0.05 0.85 0.80 0.38",
    "group": "3",
}

VISUAL_COACD_ASSET_DIR = Path("meshes/rh56_collision_visual_coacd")
DEFAULT_RH56_COLLISION_MODE = "visual_coacd"

VISUAL_COACD_SOURCE_STEMS: dict[str, str] = {
    "rh56_R_hand_base_link": "R_hand_base_link",
    "rh56_R_thumb_proximal_base": "R_thumb_proximal_base",
    "rh56_R_thumb_proximal": "R_thumb_proximal",
    "rh56_R_thumb_intermediate": "R_thumb_intermediate",
    "rh56_R_thumb_distal": "R_thumb_distal",
    "rh56_R_index_proximal": "R_index_proximal",
    "rh56_R_index_distal": "R_index_distal",
    "rh56_R_middle_proximal": "R_middle_proximal",
    "rh56_R_middle_distal": "R_middle_distal",
    "rh56_R_ring_proximal": "R_ring_proximal",
    "rh56_R_ring_distal": "R_ring_distal",
    "rh56_R_pinky_proximal": "R_pinky_proximal",
    "rh56_R_pinky_distal": "R_pinky_distal",
}

VISUAL_COACD_THUMB_INDEX_BODIES = {
    "rh56_R_thumb_proximal_base",
    "rh56_R_thumb_proximal",
    "rh56_R_thumb_intermediate",
    "rh56_R_thumb_distal",
    "rh56_R_index_proximal",
    "rh56_R_index_distal",
}

REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS: tuple[tuple[str, str], ...] = (
    ("rh56_R_thumb_proximal_base", "rh56_R_thumb_proximal"),
    ("rh56_R_thumb_proximal", "rh56_R_thumb_intermediate"),
    ("rh56_R_thumb_intermediate", "rh56_R_thumb_distal"),
    ("rh56_R_index_proximal", "rh56_R_index_distal"),
    ("rh56_R_middle_proximal", "rh56_R_middle_distal"),
    ("rh56_R_ring_proximal", "rh56_R_ring_distal"),
    ("rh56_R_pinky_proximal", "rh56_R_pinky_distal"),
)


def _find_body(root: ET.Element, name: str) -> ET.Element | None:
    return next((body for body in root.iter("body") if body.get("name") == name), None)


def _insert_body_geom(body: ET.Element, attrs: dict[str, str]) -> None:
    children = list(body)
    insert_at = next(
        (index for index, child in enumerate(children) if child.tag == "body"),
        len(children),
    )
    body.insert(insert_at, ET.Element("geom", attrs))


def _ensure_asset(root: ET.Element) -> ET.Element:
    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        compiler = root.find("compiler")
        insert_at = 1 if compiler is not None and list(root).index(compiler) == 0 else 0
        root.insert(insert_at, asset)
    return asset


def _ensure_contact(root: ET.Element) -> ET.Element:
    contact = root.find("contact")
    if contact is None:
        contact = ET.Element("contact")
        equality = root.find("equality")
        insert_at = list(root).index(equality) if equality is not None else len(list(root))
        root.insert(insert_at, contact)
    return contact


def _visual_geom_local_transform(body: ET.Element, mesh_name: str) -> dict[str, str]:
    for geom in body.findall("geom"):
        if geom.get("mesh") != mesh_name or "visual_coacd" in geom.get("name", ""):
            continue
        return {
            key: str(geom.get(key))
            for key in ("pos", "quat")
            if geom.get(key) is not None
        }
    return {}


def _is_vendor_visual_geom(body_name: str, geom: ET.Element) -> bool:
    return (
        geom.get("name") == f"{body_name}_geom_0"
        and geom.get("type") == "mesh"
        and geom.get("mesh") == body_name
    )


def _remove_nonruntime_hand_geoms(root: ET.Element) -> None:
    """Keep only vendor visuals and reviewed CoACD parts on RH56 bodies."""

    for body in root.iter("body"):
        body_name = body.get("name", "")
        if body_name not in VISUAL_COACD_SOURCE_STEMS:
            continue
        for geom in list(body.findall("geom")):
            name = geom.get("name", "")
            if _is_vendor_visual_geom(body_name, geom):
                geom.attrib.update(VISUAL_ONLY_ATTRS)
            elif not name.startswith(f"{body_name}_visual_coacd_collision_"):
                body.remove(geom)


def _prune_unreferenced_reference_meshes(root: ET.Element) -> None:
    asset = root.find("asset")
    if asset is None:
        return
    referenced = {geom.get("mesh") for geom in root.iter("geom") if geom.get("mesh")}
    for mesh in list(asset.findall("mesh")):
        name = mesh.get("name", "")
        if name.startswith("rh56_correll_") and name not in referenced:
            asset.remove(mesh)


def _add_reviewed_internal_exclusions(root: ET.Element) -> None:
    contact = _ensure_contact(root)
    existing = {
        (exclude.get("body1"), exclude.get("body2"))
        for exclude in contact.findall("exclude")
    }
    existing |= {(body2, body1) for body1, body2 in existing}
    for body1, body2 in REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS:
        if (body1, body2) not in existing:
            contact.append(ET.Element("exclude", {"body1": body1, "body2": body2}))


def patch_rh56_visual_coacd_collision_model(
    root: ET.Element,
    *,
    asset_root: str | Path = "data/sim_assets",
) -> None:
    """Derive the supported RH56 runtime model from committed CoACD parts."""

    asset_root = Path(asset_root)
    collision_dir = asset_root / VISUAL_COACD_ASSET_DIR
    if not collision_dir.exists():
        raise FileNotFoundError(f"Missing committed RH56 CoACD assets: {collision_dir}")

    asset = _ensure_asset(root)
    existing_meshes = {mesh.get("name") for mesh in asset.findall("mesh")}
    _remove_nonruntime_hand_geoms(root)

    for body_name, source_stem in VISUAL_COACD_SOURCE_STEMS.items():
        body = _find_body(root, body_name)
        if body is None:
            continue
        part_paths = sorted(collision_dir.glob(f"{source_stem}_part*.stl"))
        if not part_paths:
            raise FileNotFoundError(f"Missing CoACD parts for {body_name} in {collision_dir}")
        visual_transform = _visual_geom_local_transform(body, body_name)

        for index, part_path in enumerate(part_paths):
            mesh_name = f"rh56_visual_coacd_{source_stem}_part{index:03d}"
            if mesh_name not in existing_meshes:
                mesh_file = VISUAL_COACD_ASSET_DIR / part_path.name
                asset.append(ET.Element("mesh", {"name": mesh_name, "file": mesh_file.as_posix()}))
                existing_meshes.add(mesh_name)

            geom_name = f"{body_name}_visual_coacd_collision_{index:03d}"
            attrs = {
                **VISUAL_COACD_COLLISION_ATTRS,
                **visual_transform,
                "name": geom_name,
                "mesh": mesh_name,
                "contype": "2" if body_name in VISUAL_COACD_THUMB_INDEX_BODIES else "4",
                "conaffinity": "3" if body_name in VISUAL_COACD_THUMB_INDEX_BODIES else "1",
            }
            existing_geom = next(
                (geom for geom in body.findall("geom") if geom.get("name") == geom_name),
                None,
            )
            if existing_geom is not None:
                existing_geom.attrib.update(attrs)
            else:
                _insert_body_geom(body, attrs)

    _add_reviewed_internal_exclusions(root)
    _prune_unreferenced_reference_meshes(root)


def patch_rh56_collision_model(root: ET.Element) -> None:
    """Apply the sole supported RH56 collision representation."""

    patch_rh56_visual_coacd_collision_model(root)


def patch_rh56_collision_text(xml_text: str) -> str:
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError:
        return xml_text
    patch_rh56_collision_model(root)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")
