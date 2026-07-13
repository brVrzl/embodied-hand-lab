from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


HAND_COLLISION_ATTRS = {
    "contype": "4",
    "conaffinity": "1",
    "condim": "4",
    "priority": "2",
    "friction": "1.8 0.08 0.004",
    "solref": "0.004 1",
    "solimp": "0.92 0.98 0.002",
    "rgba": "0.05 0.85 0.80 0.28",
    "group": "3",
}

VISUAL_ONLY_ATTRS = {
    "contype": "0",
    "conaffinity": "0",
    "group": "1",
}

DISABLED_COLLISION_ATTRS = {
    "contype": "0",
    "conaffinity": "0",
    "group": "1",
}

THUMB_INDEX_SELF_COLLISION_GEOMS = {
    "rh56_R_thumb_proximal_collision",
    "rh56_R_thumb_intermediate_collision",
    "rh56_R_thumb_distal_collision",
    "rh56_R_index_proximal_collision",
    "rh56_R_index_distal_collision",
}

PROXY_GEOMS: dict[str, list[dict[str, str]]] = {
    "rh56_R_hand_base_link": [
        {
            "name": "rh56_R_palm_collision",
            "type": "box",
            "pos": "-0.002 0 -0.068",
            "size": "0.038 0.024 0.060",
        },
    ],
    "rh56_R_thumb_proximal_base": [
        {
            "name": "rh56_R_thumb_base_collision",
            "type": "capsule",
            "fromto": "-0.004 -0.014 0.001 -0.004 -0.003 0.001",
            "size": "0.006",
        },
    ],
    "rh56_R_thumb_proximal": [
        {
            "name": "rh56_R_thumb_proximal_collision",
            "type": "capsule",
            "fromto": "0 0.006 -0.001 0 0.055 -0.001",
            "size": "0.008",
        },
    ],
    "rh56_R_thumb_intermediate": [
        {
            "name": "rh56_R_thumb_intermediate_collision",
            "type": "capsule",
            "fromto": "0 -0.006 -0.005 0 0.020 -0.005",
            "size": "0.007",
        },
    ],
    "rh56_R_thumb_distal": [
        {
            "name": "rh56_R_thumb_distal_collision",
            "type": "capsule",
            "fromto": "0 0.002 -0.001 0 0.022 -0.001",
            "size": "0.0065",
        },
    ],
    "rh56_R_index_proximal": [
        {
            "name": "rh56_R_index_proximal_collision",
            "type": "capsule",
            "fromto": "0.0099 0.002 0.0015 0.0099 0.029 0.0090",
            "size": "0.007",
        },
    ],
    "rh56_R_index_distal": [
        {
            "name": "rh56_R_index_distal_collision",
            "type": "capsule",
            "fromto": "0.0083 0.002 0.0015 0.0083 0.040 0.0015",
            "size": "0.0065",
        },
    ],
    "rh56_R_middle_proximal": [
        {
            "name": "rh56_R_middle_proximal_collision",
            "type": "capsule",
            "fromto": "0.0081 0.002 0.0015 0.0081 0.029 0.0090",
            "size": "0.007",
        },
    ],
    "rh56_R_middle_distal": [
        {
            "name": "rh56_R_middle_distal_collision",
            "type": "capsule",
            "fromto": "0.0064 0.002 0.0015 0.0064 0.044 0.0015",
            "size": "0.0065",
        },
    ],
    "rh56_R_ring_proximal": [
        {
            "name": "rh56_R_ring_proximal_collision",
            "type": "capsule",
            "fromto": "0.0080 0.002 0.0015 0.0080 0.029 0.0090",
            "size": "0.007",
        },
    ],
    "rh56_R_ring_distal": [
        {
            "name": "rh56_R_ring_distal_collision",
            "type": "capsule",
            "fromto": "0.0080 0.002 0.0015 0.0080 0.040 0.0015",
            "size": "0.0065",
        },
    ],
    "rh56_R_pinky_proximal": [
        {
            "name": "rh56_R_pinky_proximal_collision",
            "type": "capsule",
            "fromto": "0.0079 0.002 0.0015 0.0079 0.029 0.0090",
            "size": "0.0068",
        },
    ],
    "rh56_R_pinky_distal": [
        {
            "name": "rh56_R_pinky_distal_collision",
            "type": "capsule",
            "fromto": "0.0079 0.002 0.0016 0.0079 0.034 0.0016",
            "size": "0.0062",
        },
    ],
}

CORRELL_COLLISION_MESHES: dict[str, tuple[str, str]] = {
    "rh56_R_thumb_proximal_base": (
        "rh56_correll_right_thumb_proximal_base",
        "correll_rh56dfx/assets/collision/right_thumb_proximal_base.stl",
    ),
    "rh56_R_thumb_proximal": (
        "rh56_correll_right_thumb_proximal",
        "correll_rh56dfx/assets/collision/right_thumb_proximal.stl",
    ),
    "rh56_R_thumb_intermediate": (
        "rh56_correll_right_thumb_intermediate",
        "correll_rh56dfx/assets/collision/right_thumb_intermediate.stl",
    ),
    "rh56_R_thumb_distal": (
        "rh56_correll_right_thumb_distal",
        "correll_rh56dfx/assets/collision/right_thumb_distal.stl",
    ),
    "rh56_R_index_proximal": (
        "rh56_correll_right_index_proximal",
        "correll_rh56dfx/assets/collision/right_index_proximal.stl",
    ),
    "rh56_R_index_distal": (
        "rh56_correll_right_index_intermediate",
        "correll_rh56dfx/assets/collision/right_index_intermediate.stl",
    ),
    "rh56_R_middle_proximal": (
        "rh56_correll_right_middle_proximal",
        "correll_rh56dfx/assets/collision/right_index_proximal.stl",
    ),
    "rh56_R_middle_distal": (
        "rh56_correll_right_middle_intermediate",
        "correll_rh56dfx/assets/collision/right_middle_intermediate.stl",
    ),
    "rh56_R_ring_proximal": (
        "rh56_correll_right_ring_proximal",
        "correll_rh56dfx/assets/collision/right_index_proximal.stl",
    ),
    "rh56_R_ring_distal": (
        "rh56_correll_right_ring_intermediate",
        "correll_rh56dfx/assets/collision/right_index_intermediate.stl",
    ),
    "rh56_R_pinky_proximal": (
        "rh56_correll_right_pinky_proximal",
        "correll_rh56dfx/assets/collision/right_index_proximal.stl",
    ),
    "rh56_R_pinky_distal": (
        "rh56_correll_right_pinky_intermediate",
        "correll_rh56dfx/assets/collision/right_pinky_intermediate.stl",
    ),
}

CORRELL_MESH_COLLISION_ATTRS = {
    "type": "mesh",
    "condim": "4",
    "priority": "2",
    "friction": "1.8 0.08 0.004",
    "solref": "0.004 1",
    "solimp": "0.92 0.98 0.002",
    "rgba": "0.05 0.85 0.80 0.38",
    "group": "3",
}

CORRELL_THUMB_INDEX_MESH_GEOMS = {
    "rh56_R_thumb_proximal_base_correll_collision",
    "rh56_R_thumb_proximal_correll_collision",
    "rh56_R_thumb_intermediate_correll_collision",
    "rh56_R_thumb_distal_correll_collision",
    "rh56_R_index_proximal_correll_collision",
    "rh56_R_index_distal_correll_collision",
}

VISUAL_COACD_ASSET_DIR = Path("meshes/rh56_collision_visual_coacd")

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
    # These are adjacent bodies inside the same articulated finger where contact
    # is dominated by hidden joint structure, not useful object interaction.
    # Palm-to-finger, thumb-to-finger, finger-to-finger, and finger-to-palm
    # contacts are intentionally left active for grasp and blocking checks.
    ("rh56_R_thumb_proximal_base", "rh56_R_thumb_proximal"),
    ("rh56_R_thumb_proximal", "rh56_R_thumb_intermediate"),
    ("rh56_R_thumb_intermediate", "rh56_R_thumb_distal"),
    ("rh56_R_index_proximal", "rh56_R_index_distal"),
    ("rh56_R_middle_proximal", "rh56_R_middle_distal"),
    ("rh56_R_ring_proximal", "rh56_R_ring_distal"),
    ("rh56_R_pinky_proximal", "rh56_R_pinky_distal"),
)


def _find_body(root: ET.Element, name: str) -> ET.Element | None:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    return None


def _insert_body_geom(body: ET.Element, attrs: dict[str, str]) -> None:
    children = list(body)
    insert_at = len(children)
    for index, child in enumerate(children):
        if child.tag == "body":
            insert_at = index
            break
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
        if geom.get("mesh") != mesh_name:
            continue
        name = geom.get("name", "")
        if "visual_coacd" in name or name.endswith("_correll_collision"):
            continue
        transform: dict[str, str] = {}
        if geom.get("pos") is not None:
            transform["pos"] = str(geom.get("pos"))
        if geom.get("quat") is not None:
            transform["quat"] = str(geom.get("quat"))
        return transform
    return {}


def _disable_existing_hand_collision_geoms(root: ET.Element, *, keep_correll: bool = True) -> None:
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if keep_correll and name.endswith("_correll_collision"):
            continue
        if name.startswith("rh56_R_") and (
            name.endswith("_collision")
            or geom.get("type") in {"mesh", "capsule", "box", "sphere"}
        ):
            geom.attrib.update(DISABLED_COLLISION_ATTRS)


def _add_reviewed_internal_exclusions(root: ET.Element) -> None:
    contact = _ensure_contact(root)
    existing = {
        (exclude.get("body1"), exclude.get("body2"))
        for exclude in contact.findall("exclude")
    }
    existing |= {(body2, body1) for body1, body2 in existing}
    for body1, body2 in REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS:
        if (body1, body2) in existing:
            continue
        contact.append(ET.Element("exclude", {"body1": body1, "body2": body2}))


def patch_rh56_correll_collision_model(root: ET.Element) -> None:
    """Use Correll RH56DFX collision meshes for the mounted RH56 hand.

    The current mounted model keeps its JAKA mount and `rh56_R_*` body names.
    Correll meshes are added as body-local collision geoms on the corresponding
    RH56 links, while the older analytic proxies are disabled.
    """

    asset = _ensure_asset(root)
    existing_meshes = {mesh.get("name") for mesh in asset.findall("mesh")}
    for mesh_name, mesh_file in set(CORRELL_COLLISION_MESHES.values()):
        if mesh_name in existing_meshes:
            continue
        asset.append(ET.Element("mesh", {"name": mesh_name, "file": mesh_file}))

    _disable_existing_hand_collision_geoms(root)

    for body_name, (mesh_name, _) in CORRELL_COLLISION_MESHES.items():
        body = _find_body(root, body_name)
        if body is None:
            continue
        geom_name = f"{body_name}_correll_collision"
        attrs = {
            **CORRELL_MESH_COLLISION_ATTRS,
            "name": geom_name,
            "mesh": mesh_name,
        }
        if geom_name in CORRELL_THUMB_INDEX_MESH_GEOMS:
            attrs["contype"] = "2"
            attrs["conaffinity"] = "3"
        else:
            attrs["contype"] = "4"
            attrs["conaffinity"] = "1"
        existing_geom = next((geom for geom in body.findall("geom") if geom.get("name") == geom_name), None)
        if existing_geom is not None:
            existing_geom.attrib.update(attrs)
            continue
        _insert_body_geom(body, attrs)


def patch_rh56_visual_coacd_collision_model(root: ET.Element, *, asset_root: str | Path = "data/sim_assets") -> None:
    """Use CoACD convex parts generated from the mounted RH56 visual STL files."""

    asset_root = Path(asset_root)
    collision_dir = asset_root / VISUAL_COACD_ASSET_DIR
    if not collision_dir.exists():
        raise FileNotFoundError(
            f"Missing {collision_dir}. Run tools/generate_rh56_visual_coacd_collision.py first."
        )

    asset = _ensure_asset(root)
    existing_meshes = {mesh.get("name") for mesh in asset.findall("mesh")}

    _disable_existing_hand_collision_geoms(root, keep_correll=False)

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
                **CORRELL_MESH_COLLISION_ATTRS,
                **visual_transform,
                "name": geom_name,
                "mesh": mesh_name,
            }
            if body_name in VISUAL_COACD_THUMB_INDEX_BODIES:
                attrs["contype"] = "2"
                attrs["conaffinity"] = "3"
            else:
                attrs["contype"] = "4"
                attrs["conaffinity"] = "1"
            existing_geom = next((geom for geom in body.findall("geom") if geom.get("name") == geom_name), None)
            if existing_geom is not None:
                existing_geom.attrib.update(attrs)
                continue
            _insert_body_geom(body, attrs)
    _add_reviewed_internal_exclusions(root)


def patch_rh56_collision_model(root: ET.Element) -> None:
    """Use Correll RH56DFX collision meshes and keep mounted visual meshes visual-only."""

    patch_rh56_correll_collision_model(root)


def patch_rh56_proxy_collision_model(root: ET.Element) -> None:
    """Use analytic convex RH56 contact proxies and keep STL meshes visual-only."""

    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if name.startswith("rh56_R_") and geom.get("type") == "mesh":
            geom.attrib.update(VISUAL_ONLY_ATTRS)

    for body_name, geom_defs in PROXY_GEOMS.items():
        body = _find_body(root, body_name)
        if body is None:
            continue
        existing = {geom.get("name") for geom in body.findall("geom")}
        for geom_def in geom_defs:
            if geom_def["name"] in existing:
                continue
            attrs = {**HAND_COLLISION_ATTRS, **geom_def}
            if geom_def["name"] in THUMB_INDEX_SELF_COLLISION_GEOMS:
                attrs["contype"] = "2"
                attrs["conaffinity"] = "3"
            _insert_body_geom(body, attrs)


def patch_rh56_collision_text(xml_text: str) -> str:
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError:
        return xml_text
    patch_rh56_collision_model(root)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")
