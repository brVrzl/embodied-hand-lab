from __future__ import annotations

import argparse
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

from embodiment_core.config import load_yaml

from .recorder import _to_list, _to_numpy, create_env_from_config


def _squeeze_frame(frame: Any) -> np.ndarray:
    array = np.asarray(_to_numpy(frame))
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3:
        raise ValueError(f"Expected an HxWxC render frame, got shape {array.shape}.")
    if array.shape[-1] == 4:
        array = array[..., :3]
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8, copy=False)
    return array


def _save_ppm(path: Path, rgb: np.ndarray) -> None:
    height, width, channels = rgb.shape
    if channels != 3:
        raise ValueError(f"PPM export requires 3 channels, got {channels}.")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        handle.write(rgb.tobytes())


def _extract_sensor_rgb(obs: Any) -> np.ndarray | None:
    obs = _to_numpy(obs)
    if not isinstance(obs, Mapping):
        return None
    if "rgb" in obs:
        return _squeeze_frame(obs["rgb"])
    sensor_data = obs.get("sensor_data", {})
    if not isinstance(sensor_data, Mapping):
        return None
    for camera_obs in sensor_data.values():
        if isinstance(camera_obs, Mapping) and "rgb" in camera_obs:
            return _squeeze_frame(camera_obs["rgb"])
    return None


def export_scene_preview(
    config: dict[str, Any],
    *,
    output_dir: Path,
    seed: int = 0,
) -> dict[str, Any]:
    env = create_env_from_config(config)
    try:
        obs, reset_info = env.reset(seed=seed)
        unwrapped = getattr(env, "unwrapped", env)
        summary = {
            "seed": seed,
            "env_id": config.get("env", {}).get("env_id"),
            "reset_info": _to_list(reset_info),
            "scene": (
                _to_list(unwrapped.get_scene_summary())
                if hasattr(unwrapped, "get_scene_summary")
                else {}
            ),
        }

        sensor_rgb = _extract_sensor_rgb(obs)
        preview_files: dict[str, str] = {}
        if sensor_rgb is not None:
            sensor_path = output_dir / "sensor_rgb.ppm"
            _save_ppm(sensor_path, sensor_rgb)
            preview_files["sensor_rgb"] = str(sensor_path)

        if getattr(unwrapped.scene, "can_render", lambda: False)():
            try:
                render_rgb = _squeeze_frame(env.render())
                render_path = output_dir / "human_render.ppm"
                _save_ppm(render_path, render_rgb)
                preview_files["human_render"] = str(render_path)
            except Exception as exc:  # pragma: no cover - depends on renderer availability
                summary["render_error"] = str(exc)
        else:
            summary["render_error"] = "Scene backend does not support rendering. Check render_backend / local GPU driver."

        summary["preview_files"] = preview_files
        output_dir.mkdir(parents=True, exist_ok=True)
        summary_path = output_dir / "scene_summary.json"
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return {
            "output_dir": str(output_dir),
            "summary_path": str(summary_path),
            "preview_files": preview_files,
        }
    finally:
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a ManiSkill scene preview and summary.")
    parser.add_argument("--config", default="configs/sim/maniskill_jaka_rh56_scene_preview.yaml")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()

    config = load_yaml(Path(args.config))
    default_output_dir = Path("data/previews") / Path(args.config).stem
    result = export_scene_preview(
        config,
        output_dir=Path(args.output_dir) if args.output_dir else default_output_dir,
        seed=args.seed,
    )
    print(f"Scene summary written to: {result['summary_path']}")
    if result["preview_files"]:
        for name, path in sorted(result["preview_files"].items()):
            print(f"{name}: {path}")


if __name__ == "__main__":
    main()
