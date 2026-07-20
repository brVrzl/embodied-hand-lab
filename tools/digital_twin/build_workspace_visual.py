#!/usr/bin/env python3
"""Transform, segment, clean, and package the registered sparse workspace in P."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from scipy.spatial import cKDTree

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json, write_yaml
from digital_twin.registration.transforms import apply_similarity


def load_colmap_points(path: Path, max_error_px: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    points, colors, errors, track_lengths = [], [], [], []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw or raw.startswith("#"):
            continue
        fields = raw.split()
        error = float(fields[7])
        if error <= max_error_px:
            points.append([float(value) for value in fields[1:4]])
            colors.append([int(value) for value in fields[4:7]])
            errors.append(error)
            track_lengths.append((len(fields) - 8) // 2)
    if not points:
        raise ValueError("No COLMAP points pass the reprojection-error threshold.")
    return np.asarray(points, float), np.asarray(colors, np.uint8), np.asarray(errors, float), np.asarray(track_lengths, int)


def inside_box(points: np.ndarray, minimum: np.ndarray, maximum: np.ndarray) -> np.ndarray:
    return np.all((points >= minimum) & (points <= maximum), axis=1)


def voxel_downsample(points: np.ndarray, colors: np.ndarray, size: float) -> tuple[np.ndarray, np.ndarray]:
    keys = np.floor(points / size).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    output_points = np.column_stack([np.bincount(inverse, weights=points[:, dim]) / counts for dim in range(3)])
    output_colors = np.column_stack([np.bincount(inverse, weights=colors[:, dim]) / counts for dim in range(3)])
    return output_points, np.clip(output_colors, 0, 255).astype(np.uint8)


def remove_outliers(points: np.ndarray, colors: np.ndarray, neighbors: int, std: float) -> tuple[np.ndarray, np.ndarray]:
    if len(points) <= neighbors:
        return points, colors
    distances, _ = cKDTree(points).query(points, k=neighbors + 1)
    score = distances[:, 1:].mean(axis=1)
    keep = score <= score.mean() + std * score.std()
    return points[keep], colors[keep]


def remove_radius_outliers(points: np.ndarray, colors: np.ndarray, radius: float, minimum_neighbors: int) -> tuple[np.ndarray, np.ndarray]:
    if len(points) == 0:
        return points, colors
    neighborhoods = cKDTree(points).query_ball_point(points, radius)
    keep = np.asarray([len(indices) - 1 >= minimum_neighbors for indices in neighborhoods], bool)
    return points[keep], colors[keep]


def remove_small_components(points: np.ndarray, colors: np.ndarray, radius: float, minimum_points: int) -> tuple[np.ndarray, np.ndarray]:
    if len(points) == 0 or minimum_points <= 1:
        return points, colors
    neighborhoods = cKDTree(points).query_ball_point(points, radius)
    visited = np.zeros(len(points), bool)
    keep = np.zeros(len(points), bool)
    for seed in range(len(points)):
        if visited[seed]:
            continue
        stack, component = [seed], []
        visited[seed] = True
        while stack:
            index = stack.pop(); component.append(index)
            for neighbor in neighborhoods[index]:
                if not visited[neighbor]:
                    visited[neighbor] = True; stack.append(neighbor)
        if len(component) >= minimum_points:
            keep[component] = True
    return points[keep], colors[keep]


def colored_box(name: str, dimensions: list[float], center: list[float], color: tuple[int, int, int, int]) -> trimesh.Trimesh:
    transform = np.eye(4); transform[:3, 3] = center
    mesh = trimesh.creation.box(extents=dimensions, transform=transform)
    mesh.visual.vertex_colors = np.tile(np.asarray(color, np.uint8), (len(mesh.vertices), 1))
    mesh.metadata["name"] = name
    return mesh


def voxel_surface(points: np.ndarray, colors: np.ndarray, size: float, maximum_voxels: int = 4500) -> trimesh.Trimesh:
    if len(points) > maximum_voxels:
        indices = np.unique(np.rint(np.linspace(0, len(points) - 1, maximum_voxels)).astype(int))
        points, colors = points[indices], colors[indices]
    offsets = size * np.asarray([
        [-.5, -.5, -.5], [.5, -.5, -.5], [.5, .5, -.5], [-.5, .5, -.5],
        [-.5, -.5, .5], [.5, -.5, .5], [.5, .5, .5], [-.5, .5, .5],
    ])
    faces_template = np.asarray([
        [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
        [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
    ], dtype=np.int64)
    vertices = (points[:, None, :] + offsets[None, :, :]).reshape(-1, 3)
    faces = np.concatenate([faces_template + 8 * index for index in range(len(points))], axis=0)
    vertex_colors = np.repeat(np.column_stack((colors, np.full(len(colors), 255, np.uint8))), 8, axis=0)
    return trimesh.Trimesh(vertices=vertices, faces=faces, vertex_colors=vertex_colors, process=False)


def export_mesh(mesh: trimesh.Trimesh, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exported = mesh.export(file_type=path.suffix.lstrip("."))
    if isinstance(exported, str):
        path.write_text(exported, encoding="utf-8")
    else:
        path.write_bytes(exported)


def export_scene_glb(geometries: dict[str, trimesh.Trimesh], path: Path) -> None:
    path.write_bytes(trimesh.Scene(geometries).export(file_type="glb"))


def draw_box_edges(axis, center: np.ndarray, size: np.ndarray, color: str, linewidth: float = 1.0) -> None:
    signs = np.asarray([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])
    vertices = center + signs * size / 2
    for first, second in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
        axis.plot(*zip(vertices[first], vertices[second]), color=color, linewidth=linewidth)


def save_views(output_dir: Path, before: tuple[np.ndarray, np.ndarray], after: tuple[np.ndarray, np.ndarray], boxes: list[dict]) -> None:
    before_points, before_colors = before; points, colors = after
    sample_before = np.arange(0, len(before_points), max(1, len(before_points) // 18000))
    sample_after = np.arange(0, len(points), max(1, len(points) // 12000))
    for name, values, rgb, title in [
        ("segmentation_before.png", before_points[sample_before], before_colors[sample_before], "Registered sparse reconstruction before semantic cleanup"),
        ("segmentation_after.png", points[sample_after], colors[sample_after], "Rendering layer after robot/board/clutter removal"),
    ]:
        fig = plt.figure(figsize=(11, 8)); axis = fig.add_subplot(111, projection="3d")
        axis.scatter(values[:,0], values[:,1], values[:,2], c=rgb/255.0, s=1)
        axis.set_xlabel("P x"); axis.set_ylabel("P y"); axis.set_zlabel("P z"); axis.set_title(title)
        axis.view_init(elev=32, azim=-55); fig.tight_layout(); fig.savefig(output_dir / name, dpi=170); plt.close(fig)

    def scene_view(filename: str, elevation: float, azimuth: float, wireframe: bool = False, axes: bool = False):
        fig = plt.figure(figsize=(12, 8)); ax = fig.add_subplot(111, projection="3d")
        if not wireframe:
            ax.scatter(points[sample_after,0], points[sample_after,1], points[sample_after,2], c=colors[sample_after]/255.0, s=2, alpha=.8)
        for box in boxes:
            draw_box_edges(ax, np.asarray(box["center"]), np.asarray(box["dimensions"]), box["color"], 1.6 if wireframe else .8)
        if axes:
            origin = np.zeros(3)
            for vector, color, label in [(np.array([.25,0,0]),"r","P +x"),(np.array([0,.25,0]),"g","P +y"),(np.array([0,0,.25]),"b","P +z")]:
                ax.quiver(*origin, *vector, color=color, linewidth=3, label=label)
            ax.legend()
        ax.set_xlim(-.55,.55); ax.set_ylim(-1.3,.3); ax.set_zlim(-.15,.75)
        ax.set_xlabel("P x (m)"); ax.set_ylabel("P y (m)"); ax.set_zlabel("P z (m)")
        ax.view_init(elev=elevation, azim=azimuth); ax.set_title("P-frame integrated workspace visual layer")
        fig.tight_layout(); fig.savefig(output_dir / filename, dpi=180); plt.close(fig)

    scene_view("workspace_scene_preview.png", 28, -58)
    scene_view("workspace_scene_oblique.png", 34, -42)
    scene_view("workspace_scene_top.png", 89.9, -90)
    scene_view("workspace_scene_wireframe.png", 30, -52, wireframe=True)
    scene_view("workspace_scene_axes.png", 28, -58, axes=True)
    for source, alias in [("workspace_scene_preview.png","scene_preview.png"),("workspace_scene_oblique.png","scene_oblique.png"),("workspace_scene_top.png","scene_top.png"),("workspace_scene_wireframe.png","scene_wireframe.png"),("workspace_scene_axes.png","scene_axes.png")]:
        (output_dir / alias).write_bytes((output_dir / source).read_bytes())


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the registered P-frame static visual workspace and semantic segmentation package.")
    parser.add_argument("--points3d", type=Path, required=True, help="Selected COLMAP points3D.txt.")
    parser.add_argument("--registration", type=Path, required=True, help="Provisional T_P_R JSON.")
    parser.add_argument("--config", type=Path, required=True, help="Static-environment YAML/JSON.")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-reprojection-error-px", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        for path in (args.points3d, args.registration, args.config):
            if not path.is_file():
                raise FileNotFoundError(path)
        config, registration = load_structured(args.config), load_structured(args.registration)
        source_points, colors, errors, track_lengths = load_colmap_points(args.points3d, args.max_reprojection_error_px)
        minimum_track = int(config["segmentation"]["permanent_background"].get("minimum_colmap_track_length", 2))
        track_mask = track_lengths >= minimum_track
        source_points, colors, errors, track_lengths = source_points[track_mask], colors[track_mask], errors[track_mask], track_lengths[track_mask]
        points = apply_similarity(source_points, registration["scale"], np.asarray(registration["rotation_matrix"]), np.asarray(registration["translation_m"]))
        crop = config["crop_P"]
        crop_mask = inside_box(points, np.asarray(crop["minimum_m"]), np.asarray(crop["maximum_m"]))
        points, colors, errors, track_lengths = points[crop_mask], colors[crop_mask], errors[crop_mask], track_lengths[crop_mask]

        table = config["segmentation"]["tabletop"]
        table_center, table_size = np.asarray(table["center_P_m"]), np.asarray(table["dimensions_m"])
        table_xy = np.all(np.abs(points[:,:2] - table_center[:2]) <= table_size[:2] / 2 + .02, axis=1)
        table_plane = table_center[2] + table_size[2] / 2
        gray_span = np.ptp(colors.astype(float), axis=1)
        brightness = colors.mean(axis=1)

        aluminum = np.zeros(len(points), bool)
        for member in config["aluminium_frame"]["members"]:
            center, size = np.asarray(member["center_P_m"]), np.asarray(member["dimensions_P_xyz_m"])
            aluminum |= inside_box(points, center - size / 2 - .015, center + size / 2 + .015)
        robot_cfg = config["segmentation"]["reconstructed_robot_removal"]
        robot = inside_box(points, np.asarray(robot_cfg["minimum_P_m"]), np.asarray(robot_cfg["maximum_P_m"]))
        aluminum &= ~robot
        board = table_xy & (np.abs(points[:,2] - table_plane) <= .012) & (gray_span < 35) & ((brightness < 85) | (brightness > 175)) & ~aluminum & ~robot
        cables = table_xy & (points[:,2] > table_plane + .008) & (points[:,2] < table_plane + .20) & (brightness < 65) & ~aluminum & ~robot & ~board
        clutter = table_xy & (points[:,2] > table_plane + .012) & ~aluminum & ~robot & ~board & ~cables
        for volume in config["segmentation"]["removable_clutter"].get("reviewed_removal_volumes", []):
            reviewed = inside_box(points, np.asarray(volume["minimum_P_m"]), np.asarray(volume["maximum_P_m"]))
            clutter |= reviewed & ~aluminum & ~robot & ~board & ~cables
        table_points = table_xy & (np.abs(points[:,2] - table_plane) <= .018) & ~aluminum & ~robot & ~board & ~cables & ~clutter
        assigned = robot | board | clutter | table_points | aluminum | cables
        background = ~assigned

        categories = {"table": table_points, "aluminium_frame": aluminum, "permanent_background": background, "removable_clutter": clutter, "robot": robot, "calibration_boards": board, "cables": cables}
        output_dir = args.output_dir
        if args.dry_run:
            print(json.dumps({name: int(mask.sum()) for name, mask in categories.items()}, indent=2)); return
        output_dir.mkdir(parents=True, exist_ok=True)
        for name, mask in categories.items():
            trimesh.PointCloud(points[mask], colors=colors[mask]).export(output_dir / f"segment_{name}.ply")

        background_points, background_colors = voxel_downsample(points[background], colors[background], float(config["segmentation"]["permanent_background"]["voxel_size_m"]))
        background_points, background_colors = remove_outliers(background_points, background_colors, int(config["segmentation"]["permanent_background"]["outlier_neighbors"]), float(config["segmentation"]["permanent_background"]["outlier_std"]))
        background_points, background_colors = remove_radius_outliers(background_points, background_colors, float(config["segmentation"]["permanent_background"]["radius_outlier_radius_m"]), int(config["segmentation"]["permanent_background"]["radius_outlier_min_neighbors"]))
        background_points, background_colors = remove_small_components(background_points, background_colors, float(config["segmentation"]["permanent_background"]["radius_outlier_radius_m"]), int(config["segmentation"]["permanent_background"]["minimum_component_points"]))
        retain_cables = bool(config["segmentation"]["cables"].get("include_in_active_visual", False))
        cable_points, cable_colors = voxel_downsample(points[cables], colors[cables], .018) if cables.any() and retain_cables else (np.empty((0,3)), np.empty((0,3),np.uint8))
        visual_points = np.vstack((background_points, cable_points))
        visual_colors = np.vstack((background_colors, cable_colors))
        debug_cloud = trimesh.PointCloud(visual_points, colors=visual_colors)
        debug_cloud.export(output_dir / "sparse_debug.ply")
        debug_cloud.export(output_dir / "static_environment.ply")

        marker_size = float(config["visual_mesh"].get("marker_size_m", .006))
        background_mesh = voxel_surface(background_points, background_colors, marker_size, 3500)
        background_mesh.metadata["name"] = "colmap_sparse_debug"
        cable_mesh = voxel_surface(cable_points, cable_colors, marker_size, 500) if len(cable_points) else trimesh.Trimesh()
        table_mesh = colored_box("tabletop", table_size.tolist(), table_center.tolist(), (174, 126, 72, 255))
        member_meshes = [colored_box(member["name"], member["dimensions_P_xyz_m"], member["center_P_m"], (185, 195, 205, 255)) for member in config["aluminium_frame"]["members"]]
        sparse_debug_mesh = trimesh.util.concatenate([background_mesh, cable_mesh])
        clean_mesh = trimesh.util.concatenate([table_mesh, *member_meshes])
        export_mesh(sparse_debug_mesh, output_dir / "sparse_debug.obj")
        export_mesh(sparse_debug_mesh, output_dir / "sparse_debug.glb")
        export_mesh(sparse_debug_mesh, output_dir / "static_environment.obj")
        export_mesh(sparse_debug_mesh, output_dir / "static_environment.glb")
        export_mesh(sparse_debug_mesh, output_dir / "static_environment_render.obj")
        for basename in ("workspace_scene", "scene"):
            export_mesh(clean_mesh, output_dir / f"{basename}.ply")
            export_mesh(clean_mesh, output_dir / f"{basename}.obj")
            export_scene_glb({"tabletop": table_mesh, **{mesh.metadata["name"]: mesh for mesh in member_meshes}}, output_dir / f"{basename}.glb")

        boxes = [{"center": table_center.tolist(), "dimensions": table_size.tolist(), "color": "saddlebrown"}] + [
            {"center": member["center_P_m"], "dimensions": member["dimensions_P_xyz_m"], "color": "slategray"} for member in config["aluminium_frame"]["members"]
        ]
        save_views(output_dir, (points, colors), (visual_points, visual_colors), boxes)
        manifest = {
            "schema_version": 1, "frame": "P", "units": "meter",
            "source_point_count_after_crop": len(points), "source_reprojection_error_limit_px": args.max_reprojection_error_px,
            "segment_counts": {name: int(mask.sum()) for name, mask in categories.items()},
            "sparse_debug_point_count": len(visual_points), "sparse_debug_mesh_vertices": len(sparse_debug_mesh.vertices), "sparse_debug_mesh_faces": len(sparse_debug_mesh.faces),
            "robot_reconstruction_removed": True, "calibration_boards_included": False,
            "cables_visual_only": True, "cables_in_active_visual": retain_cables, "dense_reconstruction_used_for_collision": False,
            "permanent_background_default_enabled": False, "sparse_debug_default_enabled": False,
            "sparse_debug_collision": False, "static_background_collision": False, "table_collision": True, "aluminium_collision": True,
            "limitations": [
                "Semantic masks are conservative geometry/color rules over sparse points, not learned instance segmentation.",
                "Compact marker mesh is an optional sparse-debug overlay and is intentionally disconnected/non-watertight.",
                "Operational table/frame poses are provisional and collision validation is required before manipulation.",
            ],
            "assets": {name: str(output_dir / name) for name in ["workspace_scene.ply","workspace_scene.obj","workspace_scene.glb","sparse_debug.ply","sparse_debug.obj","sparse_debug.glb","static_environment.ply","static_environment.obj","static_environment.glb","static_environment_render.obj"]},
        }
        write_yaml(output_dir / "segmentation_manifest.yaml", manifest)
        write_json(output_dir / "build_report.json", manifest)
        print(f"Static workspace package written to: {output_dir}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
