from __future__ import annotations

import argparse
import json
import re
import subprocess
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def _filename_token(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", value).strip("-")


def _height_token(height: float | None) -> str:
    if height is None:
        return "none"
    return f"{height:.3f}".replace(".", "p")


def _as_rgb_uint8(frame: Any) -> np.ndarray:
    array = np.asarray(frame)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3 or array.shape[2] < 3:
        raise ValueError(f"RGB frame must have shape HxWx3 or HxWx4, got {array.shape}")
    array = array[:, :, :3]
    if array.dtype == np.uint8:
        return np.ascontiguousarray(array)
    array = array.astype(np.float32, copy=False)
    if np.nanmax(array) <= 1.0:
        array = array * 255.0
    return np.ascontiguousarray(np.clip(array, 0.0, 255.0).astype(np.uint8))


def _object_height_from_step(step: dict[str, Any]) -> float | None:
    state = step.get("state") or {}
    pose = state.get("object_pose")
    if pose is None:
        pose = (step.get("extra_observation") or {}).get("obj_pose")
    if pose is None:
        return None
    array = np.asarray(pose, dtype=np.float32).reshape(-1)
    if array.size < 3:
        return None
    return float(array[2])


def _episode_quality(*, success: bool, failure_mode: str, final_height: float | None) -> str:
    if not success or failure_mode != "none":
        return "intended_failure"
    if final_height is None:
        return "near_failure"
    if final_height >= 0.08:
        return "strong_success"
    if final_height >= 0.04:
        return "weak_success"
    return "near_failure"


def _load_steps(episode_dir: Path) -> list[dict[str, Any]]:
    steps_path = episode_dir / "steps.jsonl"
    if not steps_path.exists():
        raise FileNotFoundError(f"Missing {steps_path}")
    steps: list[dict[str, Any]] = []
    with steps_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                steps.append(json.loads(line))
    return steps


def _video_filename(episode_dir: Path, metadata: dict[str, Any], final_height: float | None) -> str:
    success = bool(metadata.get("success"))
    failure_mode = str(metadata.get("failure_mode") or ("none" if success else "unknown"))
    quality = _episode_quality(success=success, failure_mode=failure_mode, final_height=final_height)
    return "__".join(
        [
            _filename_token(episode_dir.name),
            f"auto_success-{str(success).lower()}",
            f"failure_mode-{_filename_token(failure_mode)}",
            f"episode_quality-{_filename_token(quality)}",
            f"final_object_height-{_height_token(final_height)}",
        ]
    ) + ".mp4"


def _write_mp4_from_png_sequence(rgb_paths: list[Path], output_path: Path, *, fps: int) -> dict[str, Any]:
    if not rgb_paths:
        raise ValueError("No RGB frames available.")
    first = rgb_paths[0]
    if not first.name.startswith("frame_") or first.suffix.lower() != ".png":
        raise ValueError("PNG sequence must use frame_*.png names.")
    command = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(first.parent / "frame_%06d.png"),
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_path),
    ]
    process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
    if process.returncode != 0:
        raise RuntimeError(f"ffmpeg failed for {output_path}:\n{process.stderr}")
    return {"frame_count": len(rgb_paths), "fps": fps}


def _write_mp4_from_rgb_npy(rgb_paths: list[Path], output_path: Path, *, fps: int) -> dict[str, Any]:
    if not rgb_paths:
        raise ValueError("No RGB frames available.")
    first = _as_rgb_uint8(np.load(rgb_paths[0]))
    height, width = first.shape[:2]
    command = [
        "ffmpeg",
        "-y",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert process.stdin is not None
    try:
        process.stdin.write(first.tobytes())
        for rgb_path in rgb_paths[1:]:
            frame = _as_rgb_uint8(np.load(rgb_path))
            if frame.shape[:2] != (height, width):
                raise ValueError(f"Frame size changed in {rgb_path}: {frame.shape[:2]} != {(height, width)}")
            process.stdin.write(frame.tobytes())
        process.stdin.close()
        stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
        returncode = process.wait()
    except Exception:
        process.kill()
        raise
    if returncode != 0:
        raise RuntimeError(f"ffmpeg failed for {output_path}:\n{stderr}")
    return {"frame_count": len(rgb_paths), "width": width, "height": height, "fps": fps}


def _rgb_path_from_step(step: dict[str, Any], camera_name: str) -> str | None:
    rgb_paths = step.get("rgb_paths")
    if isinstance(rgb_paths, dict) and rgb_paths.get(camera_name):
        return str(rgb_paths[camera_name])
    return step.get("rgb_path")


def export_episode_videos(
    episodes_root: str | Path,
    out_dir: str | Path,
    *,
    fps: int = 10,
    camera_name: str = "third_person",
    manual_review_out: str | Path | None = None,
) -> dict[str, Any]:
    episodes_root = Path(episodes_root).resolve()
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    videos: list[dict[str, Any]] = []
    skipped: list[dict[str, str]] = []
    review_entries: dict[str, dict[str, Any]] = {}
    quality_counts: Counter[str] = Counter()
    for episode_dir in sorted(episodes_root.glob("episode_*")):
        metadata_path = episode_dir / "metadata.json"
        if not metadata_path.exists():
            skipped.append({"episode_id": episode_dir.name, "reason": "missing metadata.json"})
            continue
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        steps = _load_steps(episode_dir)
        rgb_paths = [Path(path) for step in steps if (path := _rgb_path_from_step(step, camera_name))]
        if not rgb_paths:
            skipped.append({"episode_id": episode_dir.name, "reason": "no rgb_path frames"})
            continue
        final_height = _object_height_from_step(steps[-1]) if steps else None
        output_path = out_dir / _video_filename(episode_dir, metadata, final_height)
        if all(path.suffix.lower() == ".png" for path in rgb_paths):
            stats = _write_mp4_from_png_sequence(rgb_paths, output_path, fps=fps)
        else:
            stats = _write_mp4_from_rgb_npy(rgb_paths, output_path, fps=fps)
        success = bool(metadata.get("success"))
        failure_mode = str(metadata.get("failure_mode") or ("none" if success else "unknown"))
        quality = _episode_quality(success=success, failure_mode=failure_mode, final_height=final_height)
        quality_counts[quality] += 1
        videos.append(
            {
                "episode_id": episode_dir.name,
                "video_path": str(output_path),
                "auto_success": success,
                "failure_mode": failure_mode,
                "episode_quality": quality,
                "final_object_height": final_height,
                **stats,
            }
        )
        review_entries[episode_dir.name] = {
            "auto_success": success,
            "failure_mode": failure_mode,
            "episode_quality": quality,
            "final_object_height": final_height,
            "video_path": str(output_path),
            "manual_quality": "unreviewed",
            "use_for_bc": False,
            "note": "",
        }

    index_lines = [
        "<!doctype html>",
        '<html lang="en"><head><meta charset="utf-8"><title>Episode Videos</title>',
        "<style>body{font:14px system-ui;margin:24px;color:#17202a} video{max-width:720px;width:100%;display:block;margin:8px 0 24px}</style>",
        "</head><body><h1>Episode Videos</h1>",
    ]
    for video in videos:
        filename = Path(video["video_path"]).name
        index_lines.extend(
            [
                f"<section><h2>{video['episode_id']}</h2>",
                f"<p>auto_success={str(video['auto_success']).lower()}, failure_mode={video['failure_mode']}, "
                f"episode_quality={video['episode_quality']}, final_object_height={video['final_object_height']}</p>",
                f'<video src="{filename}" controls preload="metadata"></video>',
                f'<p><a href="{filename}">{filename}</a></p></section>',
            ]
        )
    if skipped:
        index_lines.extend(["<h2>Skipped</h2>", "<ul>"])
        for item in skipped:
            index_lines.append(f"<li>{item['episode_id']}: {item['reason']}</li>")
        index_lines.append("</ul>")
    index_lines.append("</body></html>")
    (out_dir / "index.html").write_text("\n".join(index_lines) + "\n", encoding="utf-8")

    manual_review_path = None
    if manual_review_out is not None:
        manual_review_path = Path(manual_review_out).resolve()
        manual_review_path.parent.mkdir(parents=True, exist_ok=True)
        with manual_review_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump({"episodes": review_entries}, handle, sort_keys=False, allow_unicode=True)

    summary = {
        "episodes_root": str(episodes_root),
        "out_dir": str(out_dir),
        "video_count": len(videos),
        "skipped_count": len(skipped),
        "camera_name": camera_name,
        "manual_review_path": str(manual_review_path) if manual_review_path is not None else None,
        "replay_index": str(out_dir / "index.html"),
        "episode_quality_counts": dict(quality_counts),
        "videos": videos,
        "skipped": skipped,
    }
    (out_dir / "video_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Export original RGB frame videos from recorded episode directories.")
    parser.add_argument("--episodes-root", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--camera-name", default="third_person")
    parser.add_argument("--manual-review-out", default=None)
    args = parser.parse_args()
    summary = export_episode_videos(
        args.episodes_root,
        args.out_dir,
        fps=args.fps,
        camera_name=args.camera_name,
        manual_review_out=args.manual_review_out,
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
