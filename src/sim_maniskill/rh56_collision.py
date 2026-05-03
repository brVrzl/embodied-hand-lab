from __future__ import annotations

import xml.etree.ElementTree as ET


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


def patch_rh56_collision_model(root: ET.Element) -> None:
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
