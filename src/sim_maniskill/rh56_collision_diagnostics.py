from __future__ import annotations

import heapq
import json
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import mujoco
import numpy as np

from sim_maniskill.rh56_collision_validation import classify_body_geom_pair

COACD_GEOM_RE = re.compile(r"^(?P<body>rh56_R_.+)_visual_coacd_collision_(?P<index>\d{3})$")


def identify_coacd_part(
    geom_name: str,
    manifest_path: str | Path = "data/sim_assets/meshes/rh56_collision_visual_coacd/manifest.json",
) -> dict[str, Any] | None:
    match = COACD_GEOM_RE.fullmatch(geom_name)
    if match is None:
        return None
    body = match.group("body")
    part_index = int(match.group("index"))
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    body_entry = manifest.get("bodies", {}).get(body)
    if body_entry is None:
        raise KeyError(f"CoACD geom {geom_name!r} names body {body!r}, absent from {manifest_path}.")
    collision_files = body_entry["collision_files"]
    if part_index >= len(collision_files):
        raise IndexError(
            f"CoACD geom {geom_name!r} has part {part_index}, but manifest lists {len(collision_files)} parts."
        )
    return {
        "manifest_id": f"{body}:{part_index:03d}",
        "body": body,
        "geom": geom_name,
        "part_index": part_index,
        "collision_file": collision_files[part_index],
        "source_file": body_entry["source_file"],
    }


def _name(model: mujoco.MjModel, object_type: mujoco.mjtObj, object_id: int) -> str:
    return mujoco.mj_id2name(model, object_type, int(object_id)) or ""


def _static_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        geom1 = _name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1))
        geom2 = _name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2))
        body1_id = int(model.geom_bodyid[int(contact.geom1)])
        body2_id = int(model.geom_bodyid[int(contact.geom2)])
        body1 = _name(model, mujoco.mjtObj.mjOBJ_BODY, body1_id)
        body2 = _name(model, mujoco.mjtObj.mjOBJ_BODY, body2_id)
        if not body1.startswith("rh56_R_") or not body2.startswith("rh56_R_"):
            continue
        classification = classify_body_geom_pair(geom1, body1, geom2, body2)
        rows.append(
            {
                "geom1": geom1,
                "geom2": geom2,
                "body1": body1,
                "body2": body2,
                "category": classification.category,
                "severity": classification.severity,
                "distance_m": float(contact.dist),
                "penetration_m": max(0.0, -float(contact.dist)),
                "position_m": np.asarray(contact.pos, dtype=np.float64).round(10).tolist(),
                "normal": np.asarray(contact.frame[:3], dtype=np.float64).round(10).tolist(),
                "thumb_index": {
                    body1.split("_")[2] if body1.startswith("rh56_R_") else "",
                    body2.split("_")[2] if body2.startswith("rh56_R_") else "",
                }
                == {"thumb", "index"},
                "coacd_part1": identify_coacd_part(geom1),
                "coacd_part2": identify_coacd_part(geom2),
            }
        )
    return rows


def static_cross_evaluate_qpos(
    xml_by_mode: Mapping[str, str | Path],
    qpos: Sequence[float],
    *,
    ctrl: Sequence[float] | None = None,
) -> dict[str, Any]:
    qpos_array = np.asarray(qpos, dtype=np.float64)
    ctrl_array = None if ctrl is None else np.asarray(ctrl, dtype=np.float64)
    modes: dict[str, Any] = {}
    for mode, xml_path in xml_by_mode.items():
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        if qpos_array.shape != (model.nq,):
            raise ValueError(f"qpos shape {qpos_array.shape} does not match {mode} model nq={model.nq}.")
        if ctrl_array is not None and ctrl_array.shape != (model.nu,):
            raise ValueError(f"ctrl shape {ctrl_array.shape} does not match {mode} model nu={model.nu}.")
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        data.qpos[:] = qpos_array
        if ctrl_array is not None:
            data.ctrl[:] = ctrl_array
        mujoco.mj_forward(model, data)
        contacts = _static_contacts(model, data)
        thumb_index_contacts = [row for row in contacts if row["thumb_index"]]
        modes[mode] = {
            "xml": str(xml_path),
            "rh56_contacts": contacts,
            "thumb_index_contacts": thumb_index_contacts,
            "max_rh56_penetration_m": max((row["penetration_m"] for row in contacts), default=0.0),
            "max_thumb_index_penetration_m": max(
                (row["penetration_m"] for row in thumb_index_contacts), default=0.0
            ),
        }
    return {
        "diagnostic_type": "static_same_qpos_collision_representation_cross_evaluation",
        "dynamic_reachability_claim": False,
        "method": "Assign exact saved qpos independently per mode, then call mujoco.mj_forward once.",
        "qpos": qpos_array.round(10).tolist(),
        "ctrl": None if ctrl_array is None else ctrl_array.round(10).tolist(),
        "modes": modes,
    }


def _quat_matrix(quat_wxyz: Sequence[float]) -> np.ndarray:
    quat = np.asarray(quat_wxyz, dtype=np.float64)
    if quat.shape != (4,):
        raise ValueError("Quaternion must be [w, x, y, z].")
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        raise ValueError("Quaternion cannot be zero.")
    w, x, y, z = quat / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def visual_mesh_transform_matrix(
    *,
    body_position: Sequence[float],
    body_quaternion: Sequence[float],
    geom_position: Sequence[float] = (0.0, 0.0, 0.0),
    geom_quaternion: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    mesh_scale: Sequence[float] = (1.0, 1.0, 1.0),
) -> np.ndarray:
    body = np.eye(4, dtype=np.float64)
    body[:3, :3] = _quat_matrix(body_quaternion)
    body[:3, 3] = np.asarray(body_position, dtype=np.float64)
    geom = np.eye(4, dtype=np.float64)
    geom[:3, :3] = _quat_matrix(geom_quaternion)
    geom[:3, 3] = np.asarray(geom_position, dtype=np.float64)
    scale = np.eye(4, dtype=np.float64)
    scale[:3, :3] = np.diag(np.asarray(mesh_scale, dtype=np.float64))
    return body @ geom @ scale


def _float_vector(value: str | None, size: int, default: Sequence[float]) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=np.float64)
    values = np.fromstring(value, sep=" ", dtype=np.float64)
    if values.shape == (1,) and size == 3:
        values = np.repeat(values, 3)
    if values.shape != (size,):
        raise ValueError(f"Expected {size} values, got {value!r}.")
    return values


def _visual_spec(base_xml: str | Path, body_name: str) -> dict[str, Any]:
    base_xml = Path(base_xml)
    root = ET.parse(base_xml).getroot()
    body = next((item for item in root.iter("body") if item.get("name") == body_name), None)
    if body is None:
        raise KeyError(f"Body {body_name!r} not found in {base_xml}.")
    candidates = [geom for geom in body.findall("geom") if geom.get("type") == "mesh" and geom.get("mesh")]
    visual = next(
        (
            geom
            for geom in candidates
            if geom.get("contype", "0") == "0" and geom.get("conaffinity", "0") == "0"
        ),
        None,
    )
    if visual is None:
        raise KeyError(f"No disabled vendor visual mesh geom found on {body_name!r}.")
    mesh_name = visual.get("mesh")
    mesh_asset = next((mesh for mesh in root.findall("./asset/mesh") if mesh.get("name") == mesh_name), None)
    if mesh_asset is None:
        raise KeyError(f"Mesh asset {mesh_name!r} for {body_name!r} not found.")
    mesh_file = base_xml.parent / str(mesh_asset.get("file"))
    return {
        "body": body_name,
        "geom": visual.get("name", ""),
        "mesh_name": mesh_name,
        "mesh_file": str(mesh_file),
        "mesh_scale": _float_vector(mesh_asset.get("scale"), 3, (1.0, 1.0, 1.0)),
        "geom_position": _float_vector(visual.get("pos"), 3, (0.0, 0.0, 0.0)),
        "geom_quaternion": _float_vector(visual.get("quat"), 4, (1.0, 0.0, 0.0, 0.0)),
    }


def _load_world_visual_mesh(
    base_xml: str | Path,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
) -> tuple[Any, dict[str, Any]]:
    import trimesh

    spec = _visual_spec(base_xml, body_name)
    loaded = trimesh.load_mesh(spec["mesh_file"], process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.to_geometry()
    mesh = loaded.copy()
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise KeyError(f"Body {body_name!r} not found in kinematic model.")
    body_position = np.asarray(data.xpos[body_id], dtype=np.float64)
    body_quaternion = np.asarray(data.xquat[body_id], dtype=np.float64)
    transform = visual_mesh_transform_matrix(
        body_position=body_position,
        body_quaternion=body_quaternion,
        geom_position=spec["geom_position"],
        geom_quaternion=spec["geom_quaternion"],
        mesh_scale=spec["mesh_scale"],
    )
    mesh.apply_transform(transform)
    mesh_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, str(spec["mesh_name"]))
    alignment: dict[str, Any] | None = None
    if mesh_id >= 0:
        from scipy.spatial import cKDTree

        vertex_address = int(model.mesh_vertadr[mesh_id])
        vertex_count = int(model.mesh_vertnum[mesh_id])
        compiled = np.asarray(
            model.mesh_vert[vertex_address : vertex_address + vertex_count], dtype=np.float64
        )
        compiled_raw_local = compiled @ _quat_matrix(model.mesh_quat[mesh_id]).T
        compiled_raw_local += np.asarray(model.mesh_pos[mesh_id], dtype=np.float64)
        body_geom_transform = visual_mesh_transform_matrix(
            body_position=body_position,
            body_quaternion=body_quaternion,
            geom_position=spec["geom_position"],
            geom_quaternion=spec["geom_quaternion"],
        )
        compiled_world = np.column_stack(
            (compiled_raw_local, np.ones(len(compiled_raw_local), dtype=np.float64))
        ) @ body_geom_transform.T
        nearest, _ = cKDTree(np.asarray(mesh.vertices, dtype=np.float64)).query(compiled_world[:, :3])
        alignment = {
            "method": "nearest compiled MuJoCo mesh vertex to transformed raw STL vertex",
            "max_error_m": float(nearest.max(initial=0.0)),
            "mean_error_m": float(nearest.mean() if len(nearest) else 0.0),
            "compiled_vertex_count": vertex_count,
        }
    metadata = {
        "body": body_name,
        "visual_geom": spec["geom"],
        "source_file": spec["mesh_file"],
        "mesh_scale": spec["mesh_scale"].tolist(),
        "geom_position": spec["geom_position"].tolist(),
        "geom_quaternion_wxyz": spec["geom_quaternion"].tolist(),
        "body_world_position": body_position.tolist(),
        "body_world_quaternion_wxyz": body_quaternion.tolist(),
        "world_transform": transform.round(12).tolist(),
        "mujoco_compiled_mesh_alignment": alignment,
        "vertex_count": int(len(mesh.vertices)),
        "face_count": int(len(mesh.faces)),
    }
    return mesh, metadata


@dataclass(slots=True)
class _AabbNode:
    minimum: np.ndarray
    maximum: np.ndarray
    indices: np.ndarray
    left: _AabbNode | None = None
    right: _AabbNode | None = None


def _build_aabb_tree(triangles: np.ndarray, indices: np.ndarray, leaf_size: int = 12) -> _AabbNode:
    selected = triangles[indices]
    minimum = selected.min(axis=(0, 1))
    maximum = selected.max(axis=(0, 1))
    node = _AabbNode(minimum, maximum, indices)
    if len(indices) <= leaf_size:
        return node
    centers = selected.mean(axis=1)
    axis = int(np.argmax(maximum - minimum))
    order = indices[np.argsort(centers[:, axis])]
    midpoint = len(order) // 2
    node.left = _build_aabb_tree(triangles, order[:midpoint], leaf_size)
    node.right = _build_aabb_tree(triangles, order[midpoint:], leaf_size)
    return node


def _aabb_distance(a: _AabbNode, b: _AabbNode) -> float:
    delta = np.maximum(np.maximum(a.minimum - b.maximum, b.minimum - a.maximum), 0.0)
    return float(np.linalg.norm(delta))


def _triangle_sat_intersects(a: np.ndarray, b: np.ndarray, tolerance: float) -> bool:
    edges_a = (a[1] - a[0], a[2] - a[1], a[0] - a[2])
    edges_b = (b[1] - b[0], b[2] - b[1], b[0] - b[2])
    normal_a = np.cross(edges_a[0], edges_a[1])
    normal_b = np.cross(edges_b[0], edges_b[1])
    axes = [normal_a, normal_b]
    axes.extend(np.cross(edge_a, edge_b) for edge_a in edges_a for edge_b in edges_b)
    axes.extend(np.cross(normal_a, edge) for edge in edges_a)
    axes.extend(np.cross(normal_b, edge) for edge in edges_b)
    for axis in axes:
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-15:
            continue
        unit = axis / norm
        projection_a = a @ unit
        projection_b = b @ unit
        if projection_a.max() < projection_b.min() - tolerance:
            return False
        if projection_b.max() < projection_a.min() - tolerance:
            return False
    return True


def _point_triangle_closest(point: np.ndarray, triangle: np.ndarray) -> tuple[float, np.ndarray]:
    import trimesh

    closest = trimesh.triangles.closest_point(triangle.reshape(1, 3, 3), point.reshape(1, 3))[0]
    return float(np.linalg.norm(point - closest)), closest


def _segment_closest(
    p1: np.ndarray,
    q1: np.ndarray,
    p2: np.ndarray,
    q2: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray]:
    direction1 = q1 - p1
    direction2 = q2 - p2
    offset = p1 - p2
    a = float(direction1 @ direction1)
    e = float(direction2 @ direction2)
    f = float(direction2 @ offset)
    epsilon = 1e-18
    if a <= epsilon and e <= epsilon:
        return float(np.linalg.norm(p1 - p2)), p1, p2
    if a <= epsilon:
        s = 0.0
        t = float(np.clip(f / e, 0.0, 1.0))
    else:
        c = float(direction1 @ offset)
        if e <= epsilon:
            t = 0.0
            s = float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = float(direction1 @ direction2)
            denominator = a * e - b * b
            s = 0.0 if abs(denominator) <= epsilon else float(np.clip((b * f - c * e) / denominator, 0.0, 1.0))
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t = 1.0
                s = float(np.clip((b - c) / a, 0.0, 1.0))
    point1 = p1 + direction1 * s
    point2 = p2 + direction2 * t
    return float(np.linalg.norm(point1 - point2)), point1, point2


def _triangle_distance(a: np.ndarray, b: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    best_distance = math.inf
    best_a = a[0]
    best_b = b[0]
    for vertex in a:
        distance, closest = _point_triangle_closest(vertex, b)
        if distance < best_distance:
            best_distance, best_a, best_b = distance, vertex, closest
    for vertex in b:
        distance, closest = _point_triangle_closest(vertex, a)
        if distance < best_distance:
            best_distance, best_a, best_b = distance, closest, vertex
    edges = ((0, 1), (1, 2), (2, 0))
    for start_a, end_a in edges:
        for start_b, end_b in edges:
            distance, point_a, point_b = _segment_closest(a[start_a], a[end_a], b[start_b], b[end_b])
            if distance < best_distance:
                best_distance, best_a, best_b = distance, point_a, point_b
    return best_distance, best_a, best_b


def mesh_pair_proximity(mesh1: Any, mesh2: Any, *, tolerance_m: float = 1e-8) -> dict[str, Any]:
    triangles1 = np.asarray(mesh1.triangles, dtype=np.float64)
    triangles2 = np.asarray(mesh2.triangles, dtype=np.float64)
    tree1 = _build_aabb_tree(triangles1, np.arange(len(triangles1), dtype=np.int32))
    tree2 = _build_aabb_tree(triangles2, np.arange(len(triangles2), dtype=np.int32))
    queue: list[tuple[float, int, _AabbNode, _AabbNode]] = []
    serial = 0
    heapq.heappush(queue, (_aabb_distance(tree1, tree2), serial, tree1, tree2))
    best_distance = math.inf
    best_point1: np.ndarray | None = None
    best_point2: np.ndarray | None = None
    tested_pairs = 0
    while queue:
        lower_bound, _, node1, node2 = heapq.heappop(queue)
        if lower_bound > best_distance:
            continue
        leaf1 = node1.left is None and node1.right is None
        leaf2 = node2.left is None and node2.right is None
        if leaf1 and leaf2:
            for index1 in node1.indices:
                triangle1 = triangles1[index1]
                minimum1 = triangle1.min(axis=0)
                maximum1 = triangle1.max(axis=0)
                for index2 in node2.indices:
                    triangle2 = triangles2[index2]
                    minimum2 = triangle2.min(axis=0)
                    maximum2 = triangle2.max(axis=0)
                    box_delta = np.maximum(
                        np.maximum(minimum1 - maximum2, minimum2 - maximum1), 0.0
                    )
                    box_distance = float(np.linalg.norm(box_delta))
                    if box_distance > best_distance:
                        continue
                    tested_pairs += 1
                    if box_distance <= tolerance_m and _triangle_sat_intersects(
                        triangle1, triangle2, tolerance_m
                    ):
                        point = (triangle1.mean(axis=0) + triangle2.mean(axis=0)) * 0.5
                        return {
                            "intersects": True,
                            "minimum_surface_distance_m": 0.0,
                            "minimum_surface_distance_mm": 0.0,
                            "closest_point1_m": point.tolist(),
                            "closest_point2_m": point.tolist(),
                            "intersecting_triangle1_id": int(index1),
                            "intersecting_triangle2_id": int(index2),
                            "method": "deterministic_bvh_triangle_sat_fallback",
                            "optional_exact_backend_available": False,
                            "tolerance_m": tolerance_m,
                            "tested_triangle_pairs": tested_pairs,
                            "limitations": (
                                "python-fcl and rtree are unavailable. Triangle SAT plus exact primitive "
                                "distances are deterministic, but intersection classification is "
                                "tolerance-sensitive for degenerate or nearly coplanar triangles."
                            ),
                        }
                    distance, point1, point2 = _triangle_distance(triangle1, triangle2)
                    if distance < best_distance:
                        best_distance = distance
                        best_point1 = point1
                        best_point2 = point2
            continue
        split_first = not leaf1 and (leaf2 or len(node1.indices) >= len(node2.indices))
        children = (node1.left, node1.right) if split_first else (node2.left, node2.right)
        for child in children:
            if child is None:
                continue
            serial += 1
            child1, child2 = (child, node2) if split_first else (node1, child)
            distance = _aabb_distance(child1, child2)
            if distance <= best_distance:
                heapq.heappush(queue, (distance, serial, child1, child2))
    if best_point1 is None or best_point2 is None or not np.isfinite(best_distance):
        raise RuntimeError("Mesh proximity traversal did not evaluate any triangle pair.")
    return {
        "intersects": False,
        "minimum_surface_distance_m": best_distance,
        "minimum_surface_distance_mm": best_distance * 1000.0,
        "closest_point1_m": best_point1.tolist(),
        "closest_point2_m": best_point2.tolist(),
        "method": "deterministic_bvh_triangle_distance_fallback",
        "optional_exact_backend_available": False,
        "tolerance_m": tolerance_m,
        "tested_triangle_pairs": tested_pairs,
        "limitations": (
            "python-fcl and rtree are unavailable. Distance covers triangle vertex-face and edge-edge "
            "features exactly for nondegenerate triangles; near-coplanar intersection remains "
            "tolerance-sensitive."
        ),
    }


def _contact_point_distance(mesh: Any, point: Sequence[float]) -> dict[str, Any]:
    import trimesh

    query = np.asarray(point, dtype=np.float64).reshape(1, 3)
    closest, distance, triangle_id = trimesh.proximity.closest_point_naive(mesh, query)
    return {
        "distance_m": float(distance[0]),
        "distance_mm": float(distance[0] * 1000.0),
        "closest_point_m": closest[0].tolist(),
        "triangle_id": int(triangle_id[0]),
        "method": "trimesh.closest_point_naive",
    }


def diagnose_vendor_visual_mesh_pair(
    *,
    base_xml: str | Path,
    kinematic_xml: str | Path,
    qpos: Sequence[float],
    body1: str,
    body2: str,
    contact_points: Sequence[Sequence[float]] = (),
    tolerance_m: float = 1e-8,
) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(kinematic_xml))
    qpos_array = np.asarray(qpos, dtype=np.float64)
    if qpos_array.shape != (model.nq,):
        raise ValueError(f"qpos shape {qpos_array.shape} does not match model nq={model.nq}.")
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = qpos_array
    mujoco.mj_forward(model, data)
    mesh1, metadata1 = _load_world_visual_mesh(base_xml, model, data, body1)
    mesh2, metadata2 = _load_world_visual_mesh(base_xml, model, data, body2)
    proximity = mesh_pair_proximity(mesh1, mesh2, tolerance_m=tolerance_m)
    point_diagnostics = []
    for point in contact_points:
        point_diagnostics.append(
            {
                "contact_point_m": list(point),
                "body1_surface": _contact_point_distance(mesh1, point),
                "body2_surface": _contact_point_distance(mesh2, point),
            }
        )
    return {
        "diagnostic_type": "static_original_vendor_visual_mesh_proximity",
        "dynamic_reachability_claim": False,
        "qpos": qpos_array.round(10).tolist(),
        "body1": metadata1,
        "body2": metadata2,
        "proximity": proximity,
        "contact_point_surface_distances": point_diagnostics,
    }


def diagnose_vendor_visual_mesh_object_pair(
    *,
    base_xml: str | Path,
    kinematic_xml: str | Path,
    qpos: Sequence[float],
    hand_body: str,
    object_geom: str = "stage2_object",
    tolerance_m: float = 1e-8,
) -> dict[str, Any]:
    import trimesh

    model = mujoco.MjModel.from_xml_path(str(kinematic_xml))
    qpos_array = np.asarray(qpos, dtype=np.float64)
    if qpos_array.shape != (model.nq,):
        raise ValueError(f"qpos shape {qpos_array.shape} does not match model nq={model.nq}.")
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, object_geom)
    if geom_id < 0:
        raise KeyError(f"Object geom {object_geom!r} not found in {kinematic_xml}.")
    if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_BOX):
        raise ValueError("Vendor visual/object diagnostic currently supports box objects only.")
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = qpos_array
    mujoco.mj_forward(model, data)
    hand_mesh, hand_metadata = _load_world_visual_mesh(base_xml, model, data, hand_body)
    object_transform = np.eye(4, dtype=np.float64)
    object_transform[:3, :3] = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    object_transform[:3, 3] = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    half_size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    object_mesh = trimesh.creation.box(extents=2.0 * half_size, transform=object_transform)
    return {
        "diagnostic_type": "static_original_vendor_visual_mesh_to_exact_box_proximity",
        "dynamic_reachability_claim": False,
        "qpos": qpos_array.round(10).tolist(),
        "hand": hand_metadata,
        "object": {
            "geom": object_geom,
            "half_size_m": half_size.tolist(),
            "world_position_m": np.asarray(data.geom_xpos[geom_id], dtype=np.float64).tolist(),
            "world_rotation": object_transform[:3, :3].tolist(),
        },
        "proximity": mesh_pair_proximity(hand_mesh, object_mesh, tolerance_m=tolerance_m),
    }


def classify_focused_root_cause(
    *,
    dynamic_rows: Sequence[Mapping[str, Any]],
    same_qpos_evaluations: Sequence[Mapping[str, Any]],
    visual_mesh_diagnostics: Sequence[Mapping[str, Any]],
    meaningful_gap_m: float = 5e-4,
    near_touch_tolerance_m: float = 2e-4,
) -> dict[str, Any]:
    reasons: list[str] = []
    if any(bool(row.get("numerical_instability")) for row in dynamic_rows):
        return {
            "classification": "inconclusive",
            "reasons": ["At least one dynamic trajectory reported numerical instability."],
        }

    proximity_rows = [row.get("proximity", {}) for row in visual_mesh_diagnostics]
    if any(row.get("intersects") is True for row in proximity_rows):
        return {
            "classification": "shared_visual_or_kinematic_intersection",
            "reasons": ["Original transformed vendor visual meshes intersect at a relevant saved state."],
        }

    gaps = [
        float(row["minimum_surface_distance_m"])
        for row in proximity_rows
        if row.get("intersects") is False and row.get("minimum_surface_distance_m") is not None
    ]
    if gaps and min(gaps) <= near_touch_tolerance_m:
        return {
            "classification": "contact_timing_difference_near_visual_touching",
            "reasons": [
                f"Minimum original visual-mesh gap {min(gaps):.9f} m is within the "
                f"{near_touch_tolerance_m:.9f} m near-touch tolerance."
            ],
        }

    by_profile: dict[str, dict[str, Mapping[str, Any]]] = {}
    for row in dynamic_rows:
        by_profile.setdefault(str(row["profile"]), {})[str(row["collision_mode"])] = row
    dynamics_decisive = bool(by_profile)
    for profile, modes in by_profile.items():
        visual = modes.get("visual_coacd")
        references = [modes.get("correll_mesh"), modes.get("unifuc_pad_proxy")]
        if visual is None or any(reference is None for reference in references):
            dynamics_decisive = False
            reasons.append(f"Profile {profile} does not contain all three collision modes.")
            continue
        if not bool(visual.get("blocked")):
            dynamics_decisive = False
            reasons.append(f"visual_coacd did not show persistent blockage under {profile}.")
        if bool(visual.get("timeout")) or bool(visual.get("slow_progress")):
            dynamics_decisive = False
            reasons.append(f"visual_coacd outcome under {profile} is explainable by timeout or slow progress.")
        if not all(bool(reference.get("reached")) for reference in references if reference is not None):
            dynamics_decisive = False
            reasons.append(f"Both reference modes did not reach under {profile}.")

    static_decisive = bool(same_qpos_evaluations)
    for evaluation in same_qpos_evaluations:
        modes = evaluation.get("modes", {})
        visual = modes.get("visual_coacd", {})
        references = [modes.get("correll_mesh", {}), modes.get("unifuc_pad_proxy", {})]
        visual_penetration = float(visual.get("max_thumb_index_penetration_m", 0.0))
        reference_penetration = max(
            (float(reference.get("max_thumb_index_penetration_m", 0.0)) for reference in references),
            default=0.0,
        )
        if not visual.get("thumb_index_contacts"):
            static_decisive = False
            reasons.append("A saved visual_coacd state did not reproduce thumb/index contact at the same qpos.")
        if any(reference.get("thumb_index_contacts") for reference in references) and (
            visual_penetration <= reference_penetration + near_touch_tolerance_m
        ):
            static_decisive = False
            reasons.append("Reference collision modes were not materially less restrictive at the same qpos.")

    visual_gap_decisive = bool(gaps) and min(gaps) >= meaningful_gap_m
    if not gaps:
        reasons.append("No original visual-mesh separation result is available.")
    elif not visual_gap_decisive:
        reasons.append(
            f"Original visual-mesh separation is below the meaningful-gap threshold {meaningful_gap_m:.9f} m."
        )

    if dynamics_decisive and static_decisive and visual_gap_decisive:
        return {
            "classification": "confirmed_coacd_outward_approximation",
            "reasons": [
                "visual_coacd blocks across reviewed speed profiles while both reference modes reach.",
                "At identical saved qpos, visual_coacd is materially more restrictive than both references.",
                f"Original vendor visual meshes remain separated by at least {min(gaps):.9f} m.",
                "No result is explained by timeout, slow progress, or numerical instability.",
            ],
        }
    if not reasons:
        reasons.append("Available dynamic, same-qpos, and visual-mesh evidence is not jointly decisive.")
    return {"classification": "inconclusive", "reasons": reasons}
