from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import imageio.v3 as iio
import mujoco
import numpy as np

from rh56_handref_grasp_planner import ARM_ACTUATORS, HAND_ACTUATORS, _ids


def _format_vec(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.6f}" for value in values)


def _audit_camera_axes(pos: np.ndarray, target: np.ndarray) -> str:
    forward = target - pos
    forward /= np.linalg.norm(forward)
    world_up = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-6:
        right = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    up /= np.linalg.norm(up)
    return f"{_format_vec(right)} {_format_vec(up)}"


def _build_audit_xml(candidate: dict[str, Any], out_xml: Path) -> Path:
    tree = ET.parse(candidate["xml"])
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError(f"{candidate['xml']} missing worldbody")
    object_pos = np.asarray(candidate["object_pos"], dtype=np.float64)
    target = object_pos + np.asarray([0.0, 0.0, 0.075], dtype=np.float64)
    camera_pos = object_pos + np.asarray([-0.34, -0.48, 0.28], dtype=np.float64)
    existing = {camera.get("name") for camera in worldbody.findall("camera")}
    if "audit_wide_camera" not in existing:
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": "audit_wide_camera",
                "mode": "fixed",
                "pos": _format_vec(camera_pos),
                "xyaxes": _audit_camera_axes(camera_pos, target),
                "fovy": "34",
            },
        )
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)
    return out_xml


def _load_json(path: Path) -> dict[str, Any] | list[Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _camera_id(model: mujoco.MjModel, preferred: str) -> int | None:
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, preferred)
    return int(idx) if idx >= 0 else None


def _set_initial(model: mujoco.MjModel, data: mujoco.MjData, candidate: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    approach_q = np.asarray(candidate["approach_q"], dtype=np.float64)
    grasp_q = np.asarray(candidate["grasp_q"], dtype=np.float64)
    lift_q = np.asarray(candidate["lift_q"], dtype=np.float64)
    rotate_ctrl = np.asarray(candidate["rotate_ctrl_mujoco"], dtype=np.float64)
    close_ctrl = np.asarray(candidate["close_ctrl_mujoco"], dtype=np.float64)
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATORS)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATORS)
    data.qpos[:6] = approach_q
    data.ctrl[arm_ids] = approach_q
    data.ctrl[hand_ids] = np.zeros(6)
    mujoco.mj_forward(model, data)
    return approach_q, grasp_q, lift_q, rotate_ctrl, close_ctrl


def _phase_action(
    t: float,
    duration: float,
    approach_q: np.ndarray,
    grasp_q: np.ndarray,
    lift_q: np.ndarray,
    rotate_ctrl: np.ndarray,
    close_ctrl: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if t < 0.45:
        return approach_q, np.zeros(6)
    if t < 0.95:
        alpha = (t - 0.45) / 0.50
        return (1.0 - alpha) * approach_q + alpha * grasp_q, rotate_ctrl
    if t < 1.70:
        alpha = (t - 0.95) / 0.75
        hand = (1.0 - alpha) * rotate_ctrl + alpha * (0.72 * close_ctrl + 0.28 * rotate_ctrl)
        return grasp_q, hand
    if t < 2.70:
        alpha = (t - 1.70) / 1.00
        hand = (1.0 - alpha) * (0.72 * close_ctrl + 0.28 * rotate_ctrl) + alpha * close_ctrl
        return grasp_q, hand
    alpha = min(1.0, (t - 2.70) / max(0.50, duration - 2.70))
    return (1.0 - alpha) * grasp_q + alpha * lift_q, close_ctrl


def render_candidate(
    candidate: dict[str, Any],
    out_dir: Path,
    *,
    duration: float,
    fps: int,
    width: int,
    height: int,
    camera: str,
) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    audit_xml = _build_audit_xml(candidate, out_dir / "audit_scene.xml")
    model = mujoco.MjModel.from_xml_path(str(audit_xml))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)
    camera_id = _camera_id(model, camera)
    approach_q, grasp_q, lift_q, rotate_ctrl, close_ctrl = _set_initial(model, data, candidate)
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATORS)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATORS)
    object_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bench_object_body")

    frames: list[np.ndarray] = []
    key_times = {
        "00_approach": 0.20,
        "01_preclose": 1.05,
        "02_closed": 2.70,
        "03_lift": min(duration - 0.05, 4.90),
    }
    keyframes: dict[str, str] = {}
    next_frame_time = 0.0
    saved_keys: set[str] = set()
    object_trace: list[dict[str, Any]] = []
    while data.time < duration:
        arm, hand = _phase_action(float(data.time), duration, approach_q, grasp_q, lift_q, rotate_ctrl, close_ctrl)
        data.ctrl[arm_ids] = arm
        data.ctrl[hand_ids] = hand
        mujoco.mj_step(model, data)
        if data.time >= next_frame_time:
            renderer.update_scene(data, camera=camera_id)
            frame = renderer.render().copy()
            frames.append(frame)
            next_frame_time += 1.0 / max(1, fps)
            for key, key_time in key_times.items():
                if key not in saved_keys and data.time >= key_time:
                    frame_path = out_dir / f"{key}.png"
                    iio.imwrite(frame_path, frame)
                    keyframes[key] = str(frame_path)
                    saved_keys.add(key)
            if object_body >= 0:
                object_trace.append(
                    {
                        "time": round(float(data.time), 3),
                        "object_pos": data.xpos[object_body].round(5).tolist(),
                    }
                )
    video_path = out_dir / "rollout.mp4"
    iio.imwrite(video_path, frames, fps=fps)
    renderer.close()
    manifest = {
        "candidate": candidate["name"],
        "object": candidate.get("object"),
        "xml": candidate["xml"],
        "audit_xml": str(audit_xml),
        "success": candidate["result"]["success"],
        "failure_mode": candidate["result"]["failure_mode"],
        "lift_m": candidate["result"]["lift_m"],
        "max_lift_m": candidate["result"]["max_lift_m"],
        "final_contacts": candidate["result"]["final_contacts"],
        "video": str(video_path),
        "keyframes": keyframes,
        "object_trace": object_trace,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def _select_candidates(benchmark_dir: Path, objects: list[str], ranks: list[int]) -> list[tuple[str, int, dict[str, Any]]]:
    selected: list[tuple[str, int, dict[str, Any]]] = []
    for object_name in objects:
        candidates_path = benchmark_dir / object_name / "candidates.json"
        candidates = _load_json(candidates_path)
        assert isinstance(candidates, list)
        for rank in ranks:
            if 0 <= rank < len(candidates):
                candidate = dict(candidates[rank])
                candidate["object"] = object_name
                selected.append((object_name, rank, candidate))
    return selected


def export_media(args: argparse.Namespace) -> dict[str, Any]:
    benchmark_dir = Path(args.benchmark_dir)
    summary = _load_json(benchmark_dir / "benchmark_summary.json")
    assert isinstance(summary, dict)
    objects = args.objects if args.objects else sorted(summary["objects"])
    selected = _select_candidates(benchmark_dir, objects, args.ranks)
    out_root = Path(args.out_dir)
    manifests = []
    for object_name, rank, candidate in selected:
        candidate_dir = out_root / object_name / f"rank_{rank:02d}_{candidate['name']}"
        manifest = render_candidate(
            candidate,
            candidate_dir,
            duration=args.duration,
            fps=args.fps,
            width=args.width,
            height=args.height,
            camera=args.camera,
        )
        manifests.append(manifest)
    report = {"benchmark_dir": str(benchmark_dir), "out_dir": str(out_root), "count": len(manifests), "items": manifests}
    out_root.mkdir(parents=True, exist_ok=True)
    (out_root / "media_manifest.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Export MP4 videos and keyframes for RH56 hand-ref candidates.")
    parser.add_argument("--benchmark-dir", default="data/mujoco_handref_grasps")
    parser.add_argument("--out-dir", default="data/replays/rh56_handref_candidates")
    parser.add_argument("--objects", nargs="*", default=None)
    parser.add_argument("--ranks", nargs="+", type=int, default=[0])
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--camera", default="audit_wide_camera")
    args = parser.parse_args()
    report = export_media(args)
    print(json.dumps({"out_dir": report["out_dir"], "count": report["count"]}, indent=2))


if __name__ == "__main__":
    main()
