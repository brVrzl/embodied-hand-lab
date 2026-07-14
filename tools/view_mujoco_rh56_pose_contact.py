from __future__ import annotations

import argparse
import copy
import importlib
import json
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mujoco_rh56_grasp_benchmark import (
    ARM_ACTUATOR_NAMES,
    BASE_XML,
    COLLISION_MODES,
    HAND_ACTUATOR_NAMES,
    OUT_DIR,
    _configure_collision_model,
    _ids,
    _load_yaml,
    _physical_norm_to_mujoco_ctrl,
    _set_compiler_meshdir,
    _set_thumb_mimic_coupling,
    _solve_hand_base_lift_q,
    _tune_actuators_for_grasp_benchmark,
)


POSE_XML = Path("data/mujoco_debug/rh56_pose_contact_calibration.xml")
HANDREF_OUT_DIR = Path("data/mujoco_handref_grasps")
DEFAULT_CODEBOOK = Path("data/models/rh56_hand_codebook_unitree_state_k16.npz")
DEFAULT_ORDERED_CODEBOOK = Path("data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16_ordered.npz")
PLANNER_PHYSICAL_ORDER = ["pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"]
HANDREF_OBJECT_ALIASES = {
    "foam_cube": "foam_block_40mm",
    "paper_box": "061_foam_brick",
    "light_cylinder": "light_cylinder_36mm",
    "can": "light_can_50mm",
    "light_can": "light_can_50mm",
    "round_ball": "056_tennis_ball",
}

PHYSICAL_POSES: dict[str, list[float]] = {
    "open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "thumb_rotate": [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "real_pinch_v4": [0.0, 0.0, 0.12, 0.15, 0.40, 1.0],
    "sim_best_pinch": [0.10, 0.10, 0.55, 0.60, 0.68, 1.0],
    "power_close": [0.75, 0.75, 0.80, 0.80, 0.55, 0.65],
}

THUMB_COUPLINGS: dict[str, tuple[float, float]] = {
    "urdf": (0.6, 0.8),
    "xacro": (0.8, 1.2),
    "gazebo_plugin": (1.0, 1.0),
}

REVIEW_GEOMETRY_CHOICES = ("visual_coacd", "coacd_only")
DISABLED_REFERENCE_CHOICES = ("none", "legacy", "correll", "all")


def _capture_disabled_reference_geometry(
    root: ET.Element,
    selection: str,
) -> tuple[dict[str, list[ET.Element]], dict[str, ET.Element]]:
    if selection not in DISABLED_REFERENCE_CHOICES:
        raise ValueError(f"Unknown disabled reference selection: {selection}")
    if selection == "none":
        return {}, {}

    references: dict[str, list[ET.Element]] = {}
    mesh_names: set[str] = set()
    for body in root.iter("body"):
        body_name = body.get("name", "")
        if not body_name.startswith("rh56_R_"):
            continue
        for geom in body.findall("geom"):
            name = geom.get("name", "")
            if name == f"{body_name}_geom_0" or "visual_coacd_collision" in name:
                continue
            kind = "correll" if name.endswith("_correll_collision") else "legacy"
            if selection not in {kind, "all"}:
                continue
            reference = copy.deepcopy(geom)
            reference.set("name", f"review_disabled_{kind}__{name}")
            reference.set("contype", "0")
            reference.set("conaffinity", "0")
            reference.set("group", "4")
            references.setdefault(body_name, []).append(reference)
            if reference.get("mesh"):
                mesh_names.add(str(reference.get("mesh")))

    asset = root.find("asset")
    meshes = {
        str(mesh.get("name")): copy.deepcopy(mesh)
        for mesh in ([] if asset is None else asset.findall("mesh"))
        if mesh.get("name") in mesh_names
    }
    return references, meshes


def _append_disabled_reference_geometry(
    root: ET.Element,
    references: dict[str, list[ET.Element]],
    meshes: dict[str, ET.Element],
) -> None:
    if not references:
        return
    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(0, asset)
    existing_meshes = {mesh.get("name") for mesh in asset.findall("mesh")}
    for mesh_name, mesh in meshes.items():
        if mesh_name not in existing_meshes:
            asset.append(mesh)

    bodies = {body.get("name"): body for body in root.iter("body")}
    for body_name, geoms in references.items():
        body = bodies.get(body_name)
        if body is None:
            continue
        children = list(body)
        insert_at = next(
            (index for index, child in enumerate(children) if child.tag == "body"),
            len(children),
        )
        for geom in geoms:
            body.insert(insert_at, geom)
            insert_at += 1


def _build_pose_xml(
    base_xml: Path,
    out_xml: Path,
    *,
    thumb_coupling: str = "urdf",
    collision_mode: str = "visual_coacd",
    disabled_reference_geometry: str = "none",
) -> None:
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(base_xml)
    root = tree.getroot()
    references: dict[str, list[ET.Element]] = {}
    reference_meshes: dict[str, ET.Element] = {}
    if disabled_reference_geometry != "none":
        if collision_mode != "visual_coacd":
            raise ValueError("Disabled reference overlays are only supported with visual_coacd")
        references, reference_meshes = _capture_disabled_reference_geometry(
            root,
            disabled_reference_geometry,
        )
    _set_compiler_meshdir(root, base_xml)
    pip_multiplier, dip_multiplier = THUMB_COUPLINGS[thumb_coupling]
    _set_thumb_mimic_coupling(root, pip_multiplier=pip_multiplier, dip_multiplier=dip_multiplier)
    _tune_actuators_for_grasp_benchmark(root)
    _configure_collision_model(root, collision_mode=collision_mode, include_calibration_markers=True)
    _append_disabled_reference_geometry(root, references, reference_meshes)
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Missing worldbody.")
    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": "hand_check_camera",
            "mode": "fixed",
            "pos": "-0.28 -0.82 0.44",
            "xyaxes": "0.96 -0.28 0 0.15 0.51 0.85",
            "fovy": "38",
        },
    )
    root.set("model", "rh56_pose_contact_calibration")
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)


def _set_hand_qpos_from_ctrl(data: mujoco.MjData, ctrl: np.ndarray, *, thumb_coupling: str = "urdf") -> None:
    thumb_rotate, thumb_bend, index, middle, ring, pinky = ctrl
    pip_multiplier, dip_multiplier = THUMB_COUPLINGS[thumb_coupling]
    data.qpos[6:18] = [
        thumb_rotate,
        thumb_bend,
        pip_multiplier * thumb_bend,
        dip_multiplier * thumb_bend,
        index,
        index,
        middle,
        middle,
        ring,
        ring,
        pinky,
        pinky,
    ]


def _configure_viewer(
    handle: Any,
    *,
    show_contacts: bool = False,
    lookat_z: float = 0.09,
    review_geometry: str | None = None,
    show_disabled_references: bool = False,
) -> None:
    handle.cam.azimuth = -120
    handle.cam.elevation = -18
    handle.cam.distance = 0.42
    handle.cam.lookat[:] = [-0.04, -0.57, lookat_z]
    if review_geometry is not None:
        handle.opt.geomgroup[1] = 1 if review_geometry == "visual_coacd" else 0
        handle.opt.geomgroup[3] = 1
        handle.opt.geomgroup[4] = 1 if show_disabled_references else 0
    try:
        value = 1 if show_contacts else 0
        handle.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = value
        handle.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = value
    except Exception:
        pass


def _print_contact_summary(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    names: list[str] = []
    for idx in range(data.ncon):
        contact = data.contact[idx]
        geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or ""
        geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or ""
        joined = f"{geom1} {geom2}"
        if "bench_object" in joined or "bench_table" in joined or "pad_proxy" in joined or "rh56_R_" in joined:
            names.append(f"{geom1}<->{geom2}")
    if names:
        print("contacts:", "; ".join(names), flush=True)


def _load_codebook(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    data = np.load(path, allow_pickle=True)
    if "centroids" not in data:
        raise RuntimeError(f"{path} missing centroids")
    centroids = np.asarray(data["centroids"], dtype=np.float64)
    if "canonical_hand_order" in data:
        source_order = [str(item) for item in np.asarray(data["canonical_hand_order"], dtype=object).tolist()]
        if source_order != PLANNER_PHYSICAL_ORDER:
            reorder = [source_order.index(name) for name in PLANNER_PHYSICAL_ORDER]
            centroids = centroids[:, reorder]
    if centroids.ndim != 2 or centroids.shape[1] != 6:
        raise RuntimeError(f"{path} expected centroids shape [K,6], got {centroids.shape}")
    metadata_path = path.with_suffix(".json")
    metadata: dict[str, Any] = {}
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    return centroids, metadata


def _nearest_codebook_ctrl(
    physical_norm: np.ndarray,
    codebook_path: Path,
    *,
    active_only: bool = True,
    weights: np.ndarray | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    data = np.load(codebook_path, allow_pickle=True)
    centroids = np.asarray(data["centroids"], dtype=np.float64)
    if "canonical_hand_order" in data:
        source_order = [str(item) for item in np.asarray(data["canonical_hand_order"], dtype=object).tolist()]
        if source_order != PLANNER_PHYSICAL_ORDER:
            reorder = [source_order.index(name) for name in PLANNER_PHYSICAL_ORDER]
            centroids = centroids[:, reorder]
    code_indices = list(range(len(centroids)))
    if active_only and "active_indices" in data:
        active_indices = np.asarray(data["active_indices"], dtype=np.int64).tolist()
        centroids = centroids[active_indices]
        code_indices = active_indices
    if weights is None:
        weights = np.asarray([1.0, 1.0, 1.0, 1.0, 1.0, 1.8], dtype=np.float64)
    dist = np.sum(((centroids - physical_norm[None, :]) * weights[None, :]) ** 2, axis=1)
    local_idx = int(np.argmin(dist))
    code = centroids[local_idx]
    ctrl = _physical_norm_to_mujoco_ctrl(code)
    return ctrl, {
        "code_index": int(code_indices[local_idx]),
        "distance": float(dist[local_idx]),
        "physical_norm": code.round(6).tolist(),
        "mujoco_ctrl": ctrl.round(6).tolist(),
    }


def run_pose_view(args: argparse.Namespace) -> None:
    _build_pose_xml(
        Path(args.base_xml),
        Path(args.out_xml),
        thumb_coupling=args.thumb_coupling,
        collision_mode=args.collision_mode,
        disabled_reference_geometry=args.disabled_reference_geometry,
    )
    model = mujoco.MjModel.from_xml_path(str(args.out_xml))
    data = mujoco.MjData(model)
    robot_cfg = _load_yaml(args.robot_config)
    arm_q = np.asarray(robot_cfg["joint_presets"][args.arm_preset], dtype=np.float64)
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)

    pose_items = list(PHYSICAL_POSES.items())
    viewer = importlib.import_module("mujoco.viewer")
    print("Pose viewer:")
    if args.collision_mode == "correll_mesh":
        print("  当前使用 Correll RH56DFX collision mesh 作为 mounted hand 碰撞体。")
        print("  原项目 analytic proxy 已禁用；视觉 STL 只作为外形参考。")
    elif args.collision_mode == "mesh":
        print("  当前基础模型仍保留 RH56 analytic collision proxy；STL mesh 只用于视觉。")
    elif args.collision_mode == "unifuc_pad_proxy":
        print("  观察已有 UniFuc-style cyan rectangular pad proxy 是否落在真实 distal 指腹附近。")
        print("  橙色小球是每块已有矩形 pad 的中心，只可视化、不参与碰撞。")
    elif args.collision_mode == "visual_coacd":
        print("  Runtime RH56 geometry contains vendor visuals and 148 active visual_coacd hulls only.")
        if args.disabled_reference_geometry != "none":
            print(
                f"  review_disabled_* group-4 references={args.disabled_reference_geometry}; "
                "collision is disabled."
            )
    else:
        print("  观察 cyan capsule/box collision proxy 是否落在真实指腹/指尖附近。")
        print("  黄/橙/红/紫小球是沿 distal link 的候选校准点，只可视化、不参与碰撞。")
    print("  注意: poses 模式直接写 qpos，只用于看几何贴合；穿插不会被动力学接触约束自动推开。")
    print(f"  thumb_coupling={args.thumb_coupling} PIP/DIP={THUMB_COUPLINGS[args.thumb_coupling]}")
    print(f"  collision_mode={args.collision_mode}")
    print("  自动循环: open -> thumb_rotate -> real_pinch_v4 -> sim_best_pinch -> power_close")
    print("  关闭 viewer 即退出。", flush=True)
    with viewer.launch_passive(model, data) as handle:
        _configure_viewer(
            handle,
            show_contacts=args.show_contacts,
            review_geometry=args.review_geometry if args.collision_mode == "visual_coacd" else None,
            show_disabled_references=args.disabled_reference_geometry != "none",
        )
        start = time.time()
        last_pose_name = ""
        while handle.is_running():
            elapsed = time.time() - start
            pose_name, physical_norm = pose_items[int(elapsed / args.pose_period) % len(pose_items)]
            hand_ctrl = _physical_norm_to_mujoco_ctrl(physical_norm)
            data.qpos[:6] = arm_q
            _set_hand_qpos_from_ctrl(data, hand_ctrl, thumb_coupling=args.thumb_coupling)
            data.ctrl[arm_ids] = arm_q
            data.ctrl[hand_ids] = hand_ctrl
            mujoco.mj_forward(model, data)
            if pose_name != last_pose_name:
                print(f"pose={pose_name} physical_norm={physical_norm} mujoco_ctrl={hand_ctrl.round(4).tolist()}", flush=True)
                last_pose_name = pose_name
            handle.sync()
            time.sleep(0.02)


def run_codebook_view(args: argparse.Namespace) -> None:
    _build_pose_xml(
        Path(args.base_xml),
        Path(args.out_xml),
        thumb_coupling=args.thumb_coupling,
        collision_mode=args.collision_mode,
        disabled_reference_geometry=args.disabled_reference_geometry,
    )
    centroids, metadata = _load_codebook(Path(args.codebook))
    if args.codebook_index is not None and not (0 <= args.codebook_index < len(centroids)):
        raise IndexError(f"--codebook-index={args.codebook_index} out of range 0..{len(centroids) - 1}")

    model = mujoco.MjModel.from_xml_path(str(args.out_xml))
    data = mujoco.MjData(model)
    robot_cfg = _load_yaml(args.robot_config)
    arm_q = np.asarray(robot_cfg["joint_presets"][args.arm_preset], dtype=np.float64)
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)
    code_indices = [args.codebook_index] if args.codebook_index is not None else list(range(len(centroids)))
    occupancy = metadata.get("sampled_code_occupancy") or []

    viewer = importlib.import_module("mujoco.viewer")
    print("Codebook viewer:")
    print(f"  codebook={args.codebook}")
    print(f"  codes={len(centroids)} order=[index, middle, ring, pinky, thumb_close, thumb_lateral]")
    print(f"  thumb_coupling={args.thumb_coupling} PIP/DIP={THUMB_COUPLINGS[args.thumb_coupling]}")
    print(f"  collision_mode={args.collision_mode}")
    print("  注: physical_norm 会映射到 MuJoCo ctrl=[thumb_lateral, thumb_close, index, middle, ring, pinky]")
    print("  关闭 viewer 即退出。", flush=True)
    with viewer.launch_passive(model, data) as handle:
        _configure_viewer(
            handle,
            show_contacts=args.show_contacts,
            review_geometry=args.review_geometry if args.collision_mode == "visual_coacd" else None,
            show_disabled_references=args.disabled_reference_geometry != "none",
        )
        start = time.time()
        last_code_idx = -1
        while handle.is_running():
            elapsed = time.time() - start
            code_idx = code_indices[int(elapsed / args.pose_period) % len(code_indices)]
            physical_norm = centroids[code_idx]
            hand_ctrl = _physical_norm_to_mujoco_ctrl(physical_norm)
            data.qpos[:6] = arm_q
            _set_hand_qpos_from_ctrl(data, hand_ctrl, thumb_coupling=args.thumb_coupling)
            data.ctrl[arm_ids] = arm_q
            data.ctrl[hand_ids] = hand_ctrl
            mujoco.mj_forward(model, data)
            if code_idx != last_code_idx:
                occ = float(occupancy[code_idx]) if code_idx < len(occupancy) else 0.0
                marker = "anchor/rare" if occ == 0.0 or physical_norm[5] > 0.3 else "data"
                print(
                    f"code={code_idx:02d} marker={marker} occupancy={occ:.6f} "
                    f"physical_norm={np.round(physical_norm, 4).tolist()} "
                    f"mujoco_ctrl={hand_ctrl.round(4).tolist()} ncon={data.ncon}",
                    flush=True,
                )
                _print_contact_summary(model, data)
                last_code_idx = code_idx
            handle.sync()
            time.sleep(0.02)


def _load_best_candidate(object_name: str, benchmark_dir: Path, rank: int = 0) -> dict[str, Any]:
    canonical_name = HANDREF_OBJECT_ALIASES.get(object_name, object_name)
    summary_path = benchmark_dir / canonical_name / "summary.json"
    if not summary_path.exists():
        raise FileNotFoundError(
            f"Missing {summary_path}. Run scripts/run_rh56_handref_grasps.sh first."
        )
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if not summary.get("top_candidates"):
        raise RuntimeError(f"No candidates in {summary_path}")
    top_candidates = summary["top_candidates"]
    if rank < 0 or rank >= len(top_candidates):
        raise IndexError(f"rank={rank} out of range for {summary_path}; available=0..{len(top_candidates) - 1}")
    return top_candidates[rank]


def run_grasp_view(args: argparse.Namespace) -> None:
    candidate = _load_best_candidate(args.object, Path(args.benchmark_dir), rank=args.rank)
    xml_path = Path(candidate["xml"])
    if not xml_path.exists():
        raise FileNotFoundError(f"Missing candidate XML: {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    robot_cfg = _load_yaml(args.robot_config)
    grasp_q = np.asarray(
        candidate.get("grasp_q", robot_cfg["joint_presets"][args.arm_preset]),
        dtype=np.float64,
    )
    approach_q = np.asarray(candidate.get("approach_q", grasp_q), dtype=np.float64)
    lift_q = np.asarray(
        candidate.get("lift_q", _solve_hand_base_lift_q(Path(args.base_xml), grasp_q=grasp_q, lift_dz=args.lift_dz)),
        dtype=np.float64,
    )
    rotate_ctrl = np.asarray(candidate["rotate_ctrl_mujoco"], dtype=np.float64)
    close_ctrl = np.asarray(candidate["close_ctrl_mujoco"], dtype=np.float64)
    codebook_close: dict[str, Any] | None = None
    if args.use_codebook_close:
        close_ctrl, codebook_close = _nearest_codebook_ctrl(
            np.asarray(candidate["physical_close_norm"], dtype=np.float64),
            Path(args.codebook),
            active_only=not args.all_codebook_codes,
        )
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)
    object_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bench_object_body")

    viewer = importlib.import_module("mujoco.viewer")
    print("Grasp viewer:")
    print(f"  object={args.object} canonical={HANDREF_OBJECT_ALIASES.get(args.object, args.object)}")
    print(f"  rank={args.rank}")
    print(f"  candidate={candidate['name']} success={candidate['result']['success']} lift={candidate['result']['lift_m']:.4f}m")
    print(f"  wrist_pose={candidate.get('wrist_pose_name', 'legacy')} ik_error={candidate.get('ik_error_m', 0.0):.4f}m")
    print(f"  physical_close_norm={candidate['physical_close_norm']}")
    if codebook_close is not None:
        print(
            f"  codebook_close code={codebook_close['code_index']} distance={codebook_close['distance']:.4f} "
            f"physical_norm={codebook_close['physical_norm']}"
        )
    print(f"  contact point/force visualization={'enabled' if args.show_contacts else 'disabled'}", flush=True)

    def reset_cycle() -> None:
        mujoco.mj_resetData(model, data)
        data.qpos[:6] = approach_q
        data.ctrl[arm_ids] = approach_q
        data.ctrl[hand_ids] = np.zeros(6)
        mujoco.mj_forward(model, data)

    with viewer.launch_passive(model, data) as handle:
        _configure_viewer(handle, show_contacts=args.show_contacts, lookat_z=0.88)
        last_print_bucket = -1
        last_cycle_t = -1.0
        reset_cycle()
        while handle.is_running():
            cycle_t = data.time % args.duration
            if last_cycle_t >= 0.0 and cycle_t < last_cycle_t:
                reset_cycle()
                cycle_t = 0.0
            if cycle_t < 0.50:
                arm_q = approach_q
                hand = np.zeros(6)
            elif cycle_t < 1.00:
                alpha = (cycle_t - 0.50) / 0.50
                arm_q = (1.0 - alpha) * approach_q + alpha * grasp_q
                hand = rotate_ctrl
            elif cycle_t < 1.70:
                arm_q = grasp_q
                alpha = (cycle_t - 1.00) / 0.70
                hand = (1.0 - alpha) * rotate_ctrl + alpha * (0.72 * close_ctrl + 0.28 * rotate_ctrl)
            elif cycle_t < 2.70:
                arm_q = grasp_q
                alpha = (cycle_t - 1.70) / 1.00
                hand = (1.0 - alpha) * (0.72 * close_ctrl + 0.28 * rotate_ctrl) + alpha * close_ctrl
            else:
                arm_alpha = min(1.0, (cycle_t - 2.70) / max(0.50, args.duration - 2.70))
                hand = close_ctrl
                arm_q = (1.0 - arm_alpha) * grasp_q + arm_alpha * lift_q
            data.ctrl[arm_ids] = arm_q
            data.ctrl[hand_ids] = hand
            mujoco.mj_step(model, data)
            last_cycle_t = cycle_t

            bucket = int(data.time / 0.5)
            if bucket != last_print_bucket:
                obj = data.xpos[object_body].round(4).tolist() if object_body >= 0 else None
                print(f"t={data.time:.2f} phase_t={cycle_t:.2f} object={obj} ncon={data.ncon}", flush=True)
                _print_contact_summary(model, data)
                last_print_bucket = bucket
            handle.sync()
            time.sleep(model.opt.timestep)


def main() -> None:
    parser = argparse.ArgumentParser(description="View RH56 MuJoCo pose/contact calibration.")
    parser.add_argument("--mode", choices=["poses", "grasp", "codebook"], default="poses")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--arm-preset", default="pinch_grasp_box_v2")
    parser.add_argument("--out-xml", default=str(POSE_XML))
    parser.add_argument("--pose-period", type=float, default=2.5)
    parser.add_argument("--thumb-coupling", choices=sorted(THUMB_COUPLINGS), default="urdf")
    parser.add_argument("--collision-mode", choices=COLLISION_MODES, default="visual_coacd")
    parser.add_argument(
        "--review-geometry",
        choices=REVIEW_GEOMETRY_CHOICES,
        default="visual_coacd",
        help="visual_coacd review display: vendor visual overlay or CoACD hulls only.",
    )
    parser.add_argument(
        "--disabled-reference-geometry",
        choices=DISABLED_REFERENCE_CHOICES,
        default="none",
        help="Explicitly add collision-disabled legacy/Correll group-4 review references.",
    )
    object_choices = sorted(
        [
            "004_sugar_box",
            "005_tomato_soup_can",
            "009_gelatin_box",
            "040_large_marker",
            "056_tennis_ball",
            "061_foam_brick",
            "062_dice",
            "foam_block_40mm",
            "light_can_50mm",
            "light_cylinder_36mm",
            *HANDREF_OBJECT_ALIASES,
        ]
    )
    parser.add_argument("--object", choices=object_choices, default="foam_block_40mm")
    parser.add_argument("--rank", type=int, default=0, help="Index into summary top_candidates.")
    parser.add_argument("--benchmark-dir", default=str(HANDREF_OUT_DIR))
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--lift-dz", type=float, default=0.120)
    parser.add_argument("--codebook", default=str(DEFAULT_ORDERED_CODEBOOK))
    parser.add_argument("--codebook-index", type=int, default=None, help="Show one codebook index instead of cycling all codes.")
    parser.add_argument("--use-codebook-close", action="store_true", help="In grasp mode, replace continuous close hand target with nearest active codebook state.")
    parser.add_argument("--all-codebook-codes", action="store_true", help="Use all codebook codes instead of active subset for --use-codebook-close.")
    parser.add_argument("--show-contacts", action="store_true", help="Show MuJoCo contact points and force lines.")
    args = parser.parse_args()

    if args.mode == "poses":
        run_pose_view(args)
    elif args.mode == "codebook":
        run_codebook_view(args)
    else:
        run_grasp_view(args)


if __name__ == "__main__":
    main()
