from __future__ import annotations

import argparse
import importlib
import json
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from data_recorder.episode_recorder import EpisodeRecorder
from embodiment_core.config import load_yaml
from preview_mujoco_tennis_ball_lift import (
    ARM_ACTUATOR_NAMES,
    ARM_PREGRASP_QPOS,
    CameraPanelWindow,
    HAND_ACTUATOR_NAMES,
    HAND_OPEN_CTRL,
    _as_vec3,
    _camera_panel_rgb,
    _camera_xyaxes,
    build_workspace_xml,
)
from rh56_driver.hand_schema import RH56_INTERNAL_ORDER, build_hand_state


BALL_BODY = "tennis_ball_body"
BALL_GEOM = "tennis_ball"
BALL_JOINT = "tennis_ball_freejoint"
SCRIPTED_CLOSE_CTRL = np.asarray([0.75, 0.45, 1.45, 1.45, 1.45, 1.45], dtype=np.float64)


def _ids(model: mujoco.MjModel, obj_type: mujoco.mjtObj, names: list[str]) -> np.ndarray:
    ids: list[int] = []
    for name in names:
        idx = mujoco.mj_name2id(model, obj_type, name)
        if idx < 0:
            raise KeyError(f"Missing {obj_type} named {name}")
        ids.append(idx)
    return np.asarray(ids, dtype=np.int32)


def _camera_intrinsics(width: int, height: int, fovy_deg: float) -> dict[str, float]:
    fy = 0.5 * height / np.tan(np.deg2rad(float(fovy_deg)) / 2.0)
    fx = fy
    return {
        "fx": float(fx),
        "fy": float(fy),
        "cx": float((width - 1) / 2.0),
        "cy": float((height - 1) / 2.0),
    }


def _save_array(path: Path, array: np.ndarray) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.save(path, array)
    return str(path)


def _set_initial_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    ball_xyz: np.ndarray,
    arm_actuator_ids: np.ndarray,
    hand_actuator_ids: np.ndarray,
) -> None:
    data.qpos[:6] = ARM_PREGRASP_QPOS
    data.ctrl[arm_actuator_ids] = ARM_PREGRASP_QPOS
    data.ctrl[hand_actuator_ids] = HAND_OPEN_CTRL
    ball_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, BALL_JOINT)
    if ball_joint < 0:
        raise KeyError(f"Missing joint {BALL_JOINT}")
    qpos_addr = int(model.jnt_qposadr[ball_joint])
    data.qpos[qpos_addr : qpos_addr + 7] = [
        float(ball_xyz[0]),
        float(ball_xyz[1]),
        float(ball_xyz[2]),
        1.0,
        0.0,
        0.0,
        0.0,
    ]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def _solve_hand_base_target_q(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    start_q: np.ndarray,
    target_xyz: np.ndarray,
    iterations: int = 180,
    damping: float = 0.05,
    max_step: float = 0.03,
) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    if body_id < 0:
        raise KeyError("Missing rh56_R_hand_base_link")
    arm_dof_ids = np.asarray(
        [
            model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{idx}")]
            for idx in range(1, 7)
        ],
        dtype=np.int32,
    )
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{idx}")
        for idx in range(1, 7)
    ]
    q = start_q.astype(np.float64).copy()
    for _ in range(iterations):
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        err = target_xyz - data.xpos[body_id]
        if np.linalg.norm(err) < 5e-4:
            break
        jacp = np.zeros((3, model.nv), dtype=np.float64)
        jacr = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        j_arm = jacp[:, arm_dof_ids]
        lhs = j_arm @ j_arm.T + (damping**2) * np.eye(3)
        dq = j_arm.T @ np.linalg.solve(lhs, err)
        q += np.clip(dq, -max_step, max_step)
        for idx, joint_id in enumerate(joint_ids):
            if bool(model.jnt_limited[joint_id]):
                low, high = model.jnt_range[joint_id]
                q[idx] = np.clip(q[idx], low, high)
    return q


def _scripted_grasp_plan(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    ball_xyz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    hand_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    ball_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BALL_BODY)
    data.qpos[:6] = ARM_PREGRASP_QPOS
    mujoco.mj_forward(model, data)
    nominal_ball = data.xpos[ball_body].copy()
    nominal_hand = data.xpos[hand_body].copy()
    # Lower the current pregrasp so fingertips reach the tennis-ball surface,
    # then translate laterally with the sampled ball position.
    grasp_target = nominal_hand + (ball_xyz - nominal_ball)
    grasp_target[2] -= 0.08
    grasp_q = _solve_hand_base_target_q(model, data, start_q=ARM_PREGRASP_QPOS, target_xyz=grasp_target)
    lift_q = _solve_hand_base_target_q(
        model,
        data,
        start_q=grasp_q,
        target_xyz=grasp_target + np.asarray([0.0, 0.0, 0.10], dtype=np.float64),
    )
    return grasp_q, lift_q, SCRIPTED_CLOSE_CTRL.copy()


def _scripted_controls(
    t: float,
    *,
    approach_q: np.ndarray,
    grasp_q: np.ndarray,
    lift_q: np.ndarray,
    close_ctrl: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, str]:
    if t < 1.0:
        alpha = t / 1.0
        return (1.0 - alpha) * approach_q + alpha * grasp_q, HAND_OPEN_CTRL, "approach"
    if t < 2.2:
        alpha = (t - 1.0) / 1.2
        return grasp_q, (1.0 - alpha) * HAND_OPEN_CTRL + alpha * close_ctrl, "close"
    if t < 3.6:
        alpha = (t - 2.2) / 1.4
        return (1.0 - alpha) * grasp_q + alpha * lift_q, close_ctrl, "lift"
    return lift_q, close_ctrl, "hold"


def _estimate_ball_from_camera(
    *,
    depth: np.ndarray,
    segmentation: np.ndarray,
    geom_id: int,
    eye: np.ndarray,
    target: np.ndarray,
    fovy_deg: float,
    ball_radius_m: float,
) -> dict[str, Any]:
    geom_type = int(mujoco.mjtObj.mjOBJ_GEOM)
    mask = (segmentation[:, :, 0] == geom_id) & (segmentation[:, :, 1] == geom_type)
    pixels = np.argwhere(mask)
    if pixels.size == 0:
        return {"detected": False, "pixel_count": 0, "center_xyz_m": None, "surface_xyz_m": None}

    height, width = depth.shape
    intr = _camera_intrinsics(width, height, fovy_deg)
    center_vu = np.median(pixels, axis=0)
    nearest_idx = int(np.argmin(np.sum((pixels - center_vu) ** 2, axis=1)))
    v, u = pixels[nearest_idx]
    z = float(depth[v, u])
    x_axis, y_axis = _camera_xyaxes(eye, target)
    forward = target - eye
    forward = forward / max(np.linalg.norm(forward), 1e-9)
    x = (float(u) - intr["cx"]) * z / intr["fx"]
    y = -(float(v) - intr["cy"]) * z / intr["fy"]
    surface = eye + x * x_axis + y * y_axis + z * forward
    ray = surface - eye
    ray = ray / max(np.linalg.norm(ray), 1e-9)
    center = surface + ball_radius_m * ray
    return {
        "detected": True,
        "pixel_count": int(pixels.shape[0]),
        "center_pixel_uv": [float(u), float(v)],
        "center_xyz_m": center.round(6).tolist(),
        "surface_xyz_m": surface.round(6).tolist(),
        "intrinsics": intr,
    }


def _render_camera(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    camera_name: str,
    camera_cfg: dict[str, Any],
    width: int,
    height: int,
    ball_geom_id: int,
    ball_radius_m: float,
) -> dict[str, Any]:
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if camera_id < 0:
        raise KeyError(f"Missing camera {camera_name}")
    renderer = mujoco.Renderer(model, height=height, width=width)
    renderer._scene_option.geomgroup[5] = 0

    renderer.update_scene(data, camera=camera_id)
    rgb = renderer.render()

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=camera_id)
    depth = renderer.render().astype(np.float32, copy=False)
    renderer.disable_depth_rendering()

    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera_id)
    segmentation = renderer.render()
    renderer.disable_segmentation_rendering()
    renderer.close()

    eye = _as_vec3(camera_cfg["eye_xyz_m"], field=f"cameras.{camera_name}.eye_xyz_m")
    target = _as_vec3(camera_cfg["target_xyz_m"], field=f"cameras.{camera_name}.target_xyz_m")
    estimate = _estimate_ball_from_camera(
        depth=depth,
        segmentation=segmentation,
        geom_id=ball_geom_id,
        eye=eye,
        target=target,
        fovy_deg=float(camera_cfg.get("fovy_deg", 45.0)),
        ball_radius_m=ball_radius_m,
    )
    return {"rgb": rgb, "depth": depth, "segmentation": segmentation, "ball_estimate": estimate}


def _observation_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    ball_xyz: np.ndarray,
    camera_estimates: dict[str, Any],
) -> dict[str, Any]:
    ball_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BALL_BODY)
    ball_pose = {
        "position": np.asarray(data.xpos[ball_body], dtype=np.float64).round(6).tolist(),
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "frame_id": "mujoco_world",
    }
    detected_centers = [
        value["center_xyz_m"]
        for value in camera_estimates.values()
        if value.get("detected") and value.get("center_xyz_m") is not None
    ]
    fused_center = None
    if detected_centers:
        fused_center = np.mean(np.asarray(detected_centers, dtype=np.float64), axis=0).round(6).tolist()
    return {
        "robot_q_current": data.qpos[:6].round(6).tolist(),
        "hand_state": [0.0] * 6,
        "hand_cmd_last": [0.0] * 6,
        "hand_error": [0.0] * 6,
        "object_pose": ball_pose,
        "object_pose_is_privileged": True,
        "sampled_object_position_xyz_m": ball_xyz.round(6).tolist(),
        "camera_estimated_object_position_xyz_m": fused_center,
    }


def collect_dataset(args: argparse.Namespace) -> dict[str, Any]:
    config = load_yaml(args.config)
    summary = build_workspace_xml(config)
    model = mujoco.MjModel.from_xml_path(str(summary["out_xml"]))
    data = mujoco.MjData(model)
    arm_actuator_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
    hand_actuator_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)
    ball_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, BALL_GEOM)
    if ball_geom_id < 0:
        raise KeyError(f"Missing geom {BALL_GEOM}")

    scene_cfg = config.get("scene", {})
    table_cfg = scene_cfg.get("table", {})
    ball_cfg = scene_cfg.get("tennis_ball", {})
    table_center = _as_vec3(table_cfg.get("center_xyz_m", [-0.265, 0.0, -0.05]), field="table.center_xyz_m")
    table_size = _as_vec3(table_cfg.get("size_xyz_m", [0.60, 0.30, 0.02]), field="table.size_xyz_m")
    ball_center = _as_vec3(ball_cfg.get("center_xyz_m", [-0.54, -0.11, 0.0035]), field="tennis_ball.center_xyz_m")
    ball_radius = float(ball_cfg.get("radius_m", 0.0335))
    half_size = float(args.random_half_size_m if args.random_half_size_m is not None else ball_cfg.get("spawn_region_half_size_m", 0.04))
    table_top_z = float(table_center[2] + table_size[2])
    ball_z = table_top_z + ball_radius

    cameras = config.get("cameras", {}) or {}
    camera_names = list(cameras.keys())
    recorder = EpisodeRecorder(load_yaml(args.logging_config), data_root=args.data_root)
    rng = np.random.default_rng(args.seed)
    episode_dirs: list[str] = []
    errors: list[float] = []
    viewer_handle: Any | None = None
    panel_window: CameraPanelWindow | None = None
    panel_renderers: list[mujoco.Renderer] = []
    panel_camera_ids: list[int] = []
    if args.viewer_preview:
        mujoco_viewer = importlib.import_module("mujoco.viewer")
        viewer_handle = mujoco_viewer.launch_passive(model, data)
        viewer_handle.opt.geomgroup[5] = 1
        viewer_handle.cam.azimuth = -130
        viewer_handle.cam.elevation = -25
        viewer_handle.cam.distance = 0.85
        viewer_handle.cam.lookat[:] = [-0.35, -0.03, 0.13]
        if args.camera_panel:
            panel_renderers = [
                mujoco.Renderer(model, height=args.panel_height, width=args.panel_width)
                for _ in camera_names
            ]
            panel_camera_ids = [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
                for name in camera_names
            ]
            panel_window = CameraPanelWindow(
                title="MuJoCo collection cameras",
                geometry=f"{args.panel_width}x{args.panel_height * max(1, len(camera_names))}+20+40",
            )

    def update_preview() -> None:
        if viewer_handle is not None and viewer_handle.is_running():
            viewer_handle.sync()
        if panel_window is not None and not panel_window.closed:
            panel = _camera_panel_rgb(
                model,
                data,
                renderers=panel_renderers,
                camera_ids=panel_camera_ids,
                camera_names=camera_names,
            )
            panel_window.update(panel)

    try:
        for episode_idx in range(args.episodes):
            offset = rng.uniform(-half_size, half_size, size=2)
            ball_xyz = np.asarray([ball_center[0] + offset[0], ball_center[1] + offset[1], ball_z], dtype=np.float64)
            _set_initial_state(
                model,
                data,
                ball_xyz=ball_xyz,
                arm_actuator_ids=arm_actuator_ids,
                hand_actuator_ids=hand_actuator_ids,
            )
            if viewer_handle is not None:
                update_preview()
                time.sleep(max(0.0, args.preview_hold_sec))
            episode_dir = recorder.start_episode(
                task_name="mujoco_tennis_ball_randomized_perception",
                instruction="observe the randomized tennis ball position before grasping",
                operator="mujoco",
                metadata={
                    "workspace_xml": str(summary["out_xml"]),
                    "camera_names": camera_names,
                    "random_half_size_m": half_size,
                    "seed": args.seed,
                    "episode_index_in_run": episode_idx,
                },
            )
            if args.scripted_grasp:
                grasp_q, lift_q, close_ctrl = _scripted_grasp_plan(model, data, ball_xyz=ball_xyz)
                total_steps = max(1, int(args.grasp_duration_sec / model.opt.timestep))
                record_interval_steps = max(1, int(1.0 / (args.record_hz * model.opt.timestep)))
            else:
                grasp_q = ARM_PREGRASP_QPOS.copy()
                lift_q = ARM_PREGRASP_QPOS.copy()
                close_ctrl = HAND_OPEN_CTRL.copy()
                total_steps = max(1, args.frames_per_episode)
                record_interval_steps = 1
            preview_interval_steps = max(1, int(1.0 / (max(args.preview_hz, 1.0) * model.opt.timestep)))

            recorded = 0
            for step_idx in range(total_steps):
                sim_t = step_idx * float(model.opt.timestep)
                if args.scripted_grasp:
                    arm_q, hand_ctrl, phase = _scripted_controls(
                        sim_t,
                        approach_q=ARM_PREGRASP_QPOS,
                        grasp_q=grasp_q,
                        lift_q=lift_q,
                        close_ctrl=close_ctrl,
                    )
                    data.ctrl[arm_actuator_ids] = arm_q
                    data.ctrl[hand_actuator_ids] = hand_ctrl
                    mujoco.mj_step(model, data)
                else:
                    phase = "observe"
                    mujoco.mj_forward(model, data)
                if viewer_handle is not None and step_idx % preview_interval_steps == 0:
                    update_preview()
                should_record = step_idx % record_interval_steps == 0
                if not should_record:
                    continue
                if not args.scripted_grasp and recorded >= args.frames_per_episode:
                    break
                timestamp = time.time()
                rgb_paths: dict[str, str] = {}
                depth_paths: dict[str, str] = {}
                camera_estimates: dict[str, Any] = {}
                for camera_name in camera_names:
                    rendered = _render_camera(
                        model,
                        data,
                        camera_name=camera_name,
                        camera_cfg=cameras[camera_name],
                        width=args.width,
                        height=args.height,
                        ball_geom_id=ball_geom_id,
                        ball_radius_m=ball_radius,
                    )
                    rgb_paths[camera_name] = _save_array(
                        episode_dir / "rgb" / f"{timestamp:.6f}_{camera_name}.npy",
                        rendered["rgb"],
                    )
                    depth_paths[camera_name] = _save_array(
                        episode_dir / "depth" / f"{timestamp:.6f}_{camera_name}.npy",
                        rendered["depth"],
                    )
                    camera_estimates[camera_name] = rendered["ball_estimate"]
                state = _observation_state(model, data, ball_xyz=ball_xyz, camera_estimates=camera_estimates)
                state["scripted_phase"] = phase
                state["sim_time"] = round(float(data.time), 6)
                estimate = state.get("camera_estimated_object_position_xyz_m")
                if estimate is not None:
                    errors.append(float(np.linalg.norm(np.asarray(estimate, dtype=np.float64) - ball_xyz)))
                recorder.record_step(
                    timestamp=timestamp,
                    observation={
                        "rgb_paths": rgb_paths,
                        "depth_paths": depth_paths,
                        "camera_timestamp": {name: timestamp for name in camera_names},
                        "arm_joint_states": {
                            "names": [f"jaka_joint_{idx}" for idx in range(1, 7)],
                        "positions": data.qpos[:6].round(6).tolist(),
                        "velocities": data.qvel[:6].round(6).tolist(),
                            "efforts": [],
                        },
                        "arm_ee_pose": None,
                        "hand_states": build_hand_state(
                            raw_positions=np.zeros(6),
                            raw_velocities=np.zeros(6),
                            raw_currents=np.zeros(6),
                            raw_forces=np.zeros(6),
                            raw_contact_binary=[False] * 6,
                            raw_order=RH56_INTERNAL_ORDER,
                            calibration=None,
                            mode="mujoco",
                        ),
                        "state": state,
                        "extra_observation": {
                            "obj_pose": state["object_pose"],
                            "camera_ball_estimates": camera_estimates,
                        },
                    },
                    action={
                    "type": "mujoco_static_observation",
                    "scripted_phase": phase,
                    "sim_time": round(float(data.time), 6),
                    "ee_delta": [0.0] * 6,
                    "hand_cmd": np.clip(data.ctrl[hand_actuator_ids], 0.0, 1.7).round(6).tolist(),
                    "hand_delta_cmd": [0.0] * 6,
                    "robot_q_current": data.qpos[:6].round(6).tolist(),
                    "robot_q_desired": data.ctrl[arm_actuator_ids].round(6).tolist(),
                },
            )
                recorded += 1
                for _ in range(max(0, args.sim_steps_between_frames)):
                    mujoco.mj_step(model, data)
                    if viewer_handle is not None:
                        update_preview()
            recorder.mark_success(True, operator_notes="sim perception data collected")
            episode_dirs.append(str(recorder.stop_episode(success=True)))
    finally:
        for renderer in panel_renderers:
            renderer.close()
        if panel_window is not None:
            panel_window.close()
        if viewer_handle is not None:
            viewer_handle.close()

    run_summary = {
        "episodes": args.episodes,
        "frames_per_episode": args.frames_per_episode,
        "data_root": str(Path(args.data_root).resolve()),
        "episode_dirs": episode_dirs,
        "camera_names": camera_names,
        "random_half_size_m": half_size,
        "mean_camera_position_error_m": float(np.mean(errors)) if errors else None,
        "max_camera_position_error_m": float(np.max(errors)) if errors else None,
    }
    summary_path = Path(args.data_root).resolve() / "last_collection_summary.json"
    summary_path.write_text(json.dumps(run_summary, indent=2), encoding="utf-8")
    return run_summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect randomized MuJoCo tennis-ball perception episodes.")
    parser.add_argument("--config", default="configs/sim/mujoco_jaka_rh56_tennis_ball_lift.yaml")
    parser.add_argument("--logging-config", default="configs/logging/default.yaml")
    parser.add_argument("--data-root", default="data/episodes_mujoco_tennis_ball")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--frames-per-episode", type=int, default=3)
    parser.add_argument("--sim-steps-between-frames", type=int, default=0)
    parser.add_argument("--random-half-size-m", type=float, default=None)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--viewer-preview", action="store_true")
    parser.add_argument("--preview-hold-sec", type=float, default=1.5)
    parser.add_argument("--scripted-grasp", action="store_true")
    parser.add_argument("--grasp-duration-sec", type=float, default=4.0)
    parser.add_argument("--record-hz", type=float, default=10.0)
    parser.add_argument("--preview-hz", type=float, default=30.0)
    parser.add_argument("--camera-panel", action="store_true")
    parser.add_argument("--panel-width", type=int, default=560)
    parser.add_argument("--panel-height", type=int, default=315)
    args = parser.parse_args()
    summary = collect_dataset(args)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
