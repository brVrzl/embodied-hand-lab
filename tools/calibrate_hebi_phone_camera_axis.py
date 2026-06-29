from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from embodiment_core.config import load_yaml
from teleop_tools.hebi_mobile_io import HebiMobileIOClient, quat_conjugate_wxyz, rotate_vector_wxyz


AXES = {
    "+X": np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
    "-X": np.asarray([-1.0, 0.0, 0.0], dtype=np.float64),
    "+Y": np.asarray([0.0, 1.0, 0.0], dtype=np.float64),
    "-Y": np.asarray([0.0, -1.0, 0.0], dtype=np.float64),
    "+Z": np.asarray([0.0, 0.0, 1.0], dtype=np.float64),
    "-Z": np.asarray([0.0, 0.0, -1.0], dtype=np.float64),
}


POSES = [
    {
        "name": "screen_up",
        "prompt": "1) Put iPhone screen up, back camera toward the table/ground. Hold still, then press Enter.",
        "expected_z": -1.0,
    },
    {
        "name": "upright_back_camera_forward",
        "prompt": "2) Put iPhone vertical, back camera facing the workspace/front. Hold still, then press Enter.",
        "expected_abs_z": 0.0,
    },
    {
        "name": "screen_down",
        "prompt": "3) Put iPhone screen down, back camera toward the sky/ceiling. Hold still, then press Enter.",
        "expected_z": 1.0,
    },
]


def _phone_to_world_quat(quaternion_wxyz: list[float], convention: str) -> np.ndarray:
    if convention == "world-to-phone":
        return quat_conjugate_wxyz(quaternion_wxyz)
    return np.asarray(quaternion_wxyz, dtype=np.float64)


def _axis_direction_z(
    quaternion_wxyz: list[float],
    axis: np.ndarray,
    *,
    convention: str,
) -> float:
    quat = _phone_to_world_quat(quaternion_wxyz, convention)
    direction = rotate_vector_wxyz(quat, axis)
    return float(direction[2])


def _capture_pose(
    client: HebiMobileIOClient,
    *,
    duration_sec: float,
    hz: float,
    read_timeout_ms: float,
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    start = time.time()
    period = 1.0 / max(float(hz), 1e-6)
    while time.time() - start < duration_sec:
        snapshot = client.read(timeout_ms=read_timeout_ms)
        if snapshot.valid:
            samples.append(snapshot.to_dict(elapsed_sec=time.time() - start))
        time.sleep(period)
    return samples


def _score_axis(samples_by_pose: dict[str, list[dict[str, Any]]], axis: np.ndarray, convention: str) -> dict[str, float]:
    score = 0.0
    details: dict[str, float] = {}
    for pose in POSES:
        pose_samples = samples_by_pose[pose["name"]]
        z_values = np.asarray(
            [
                _axis_direction_z(sample["quaternion_wxyz"], axis, convention=convention)
                for sample in pose_samples
            ],
            dtype=np.float64,
        )
        z_mean = float(np.mean(z_values)) if len(z_values) else 0.0
        details[f"{pose['name']}_z_mean"] = z_mean
        if "expected_z" in pose:
            term = 1.0 - abs(z_mean - float(pose["expected_z"])) / 2.0
        else:
            term = 1.0 - abs(z_mean)
        score += term
    details["score"] = score
    details["passes_screen_up"] = details["screen_up_z_mean"] < -0.75
    details["passes_upright"] = abs(details["upright_back_camera_forward_z_mean"]) < 0.35
    details["passes_screen_down"] = details["screen_down_z_mean"] > 0.75
    details["passes_all"] = bool(
        details["passes_screen_up"]
        and details["passes_upright"]
        and details["passes_screen_down"]
    )
    return details


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate which HEBI phone axis matches the iPhone back-camera direction.")
    parser.add_argument("--config", default="configs/teleop/hebi_mobile_io_jaka_rh56.yaml")
    parser.add_argument("--sample-sec", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--output", default="data/checks/hebi_phone_axis_calibration.json")
    args = parser.parse_args()

    config = load_yaml(args.config)
    hebi_cfg = config.get("hebi", {})
    client = HebiMobileIOClient(
        family=str(hebi_cfg.get("family", "HEBI")),
        name=str(hebi_cfg.get("name", "mobileIO")),
        lookup_wait_sec=float(hebi_cfg.get("lookup_wait_sec", 2.0)),
        setup_ui=bool(hebi_cfg.get("setup_ui", True)),
        max_stale_feedback_sec=float(hebi_cfg.get("max_stale_feedback_sec", 0.25)),
    )
    client.connect()
    samples_by_pose: dict[str, list[dict[str, Any]]] = {}
    for pose in POSES:
        input(pose["prompt"])
        samples = _capture_pose(
            client,
            duration_sec=float(args.sample_sec),
            hz=float(args.hz),
            read_timeout_ms=float(hebi_cfg.get("read_timeout_ms", 10.0)),
        )
        if not samples:
            raise RuntimeError(f"No valid HEBI samples captured for {pose['name']}.")
        samples_by_pose[pose["name"]] = samples
        print(json.dumps({"pose": pose["name"], "samples": len(samples)}, ensure_ascii=False))

    results: list[dict[str, Any]] = []
    for convention in ("body-to-world", "world-to-phone"):
        for axis_name, axis in AXES.items():
            details = _score_axis(samples_by_pose, axis, convention)
            results.append(
                {
                    "axis": axis_name,
                    "convention": convention,
                    **details,
                }
            )
    results.sort(key=lambda item: float(item["score"]), reverse=True)
    valid_results = [item for item in results if item["passes_all"]]
    report = {
        "best": results[0],
        "best_valid": valid_results[0] if valid_results else None,
        "results": results,
        "sample_counts": {name: len(samples) for name, samples in samples_by_pose.items()},
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
