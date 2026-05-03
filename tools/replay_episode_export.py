from __future__ import annotations

import argparse
import html
import json
import re
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def _array(value: Any, *, length: int | None = None) -> np.ndarray:
    array = np.asarray(value if value is not None else [], dtype=np.float32).reshape(-1)
    if length is not None and array.size < length:
        array = np.pad(array, (0, length - array.size))
    if length is not None:
        array = array[:length]
    return array


def _object_pose(sample: dict[str, Any]) -> list[float] | None:
    state = (sample.get("observation") or {}).get("state") or {}
    object_pose = state.get("object_pose")
    if object_pose is None:
        object_pose = ((sample.get("observation") or {}).get("extra_observation") or {}).get("obj_pose")
    if object_pose is None:
        return None
    array = _array(object_pose)
    if array.size < 3:
        return None
    return array[:3].astype(float).tolist()


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


def _load_samples(export_root: Path) -> list[dict[str, Any]]:
    samples_path = export_root / "samples.jsonl"
    if not samples_path.exists():
        raise FileNotFoundError(f"Missing {samples_path}")
    samples: list[dict[str, Any]] = []
    with samples_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                samples.append(json.loads(line))
    if not samples:
        raise RuntimeError(f"No samples found in {samples_path}")
    return samples


def _filename_token(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", value).strip("-")


def _height_token(height: float | None) -> str:
    if height is None:
        return "none"
    return f"{height:.3f}".replace(".", "p")


def _frame_payload(sample: dict[str, Any]) -> dict[str, Any]:
    state = sample["observation"]["state"]
    ee_state = _array(state.get("ee_state"), length=7)
    hand_cmd = _array(sample["action"].get("hand_cmd"), length=6)
    hand_state = _array(state.get("hand_state"), length=6)
    return {
        "frame_index": int(sample["frame_index"]),
        "object": _object_pose(sample),
        "ee": ee_state[:3].astype(float).tolist(),
        "hand_cmd_mean": float(np.mean(hand_cmd)),
        "hand_state_mean": float(np.mean(hand_state)),
    }


def _write_episode_html(
    *,
    out_path: Path,
    episode_id: str,
    auto_success: bool,
    failure_mode: str,
    quality: str,
    final_height: float | None,
    frames: list[dict[str, Any]],
) -> None:
    title = (
        f"{episode_id} | auto_success={auto_success} | failure_mode={failure_mode} | "
        f"episode_quality={quality} | final_object_height={final_height}"
    )
    payload = json.dumps(frames)
    body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{html.escape(title)}</title>
  <style>
    body {{ margin: 24px; font: 14px/1.45 system-ui, -apple-system, Segoe UI, sans-serif; color: #17202a; }}
    h1 {{ font-size: 18px; margin: 0 0 12px; }}
    .meta {{ display: grid; grid-template-columns: repeat(2, minmax(240px, 1fr)); gap: 6px 18px; margin-bottom: 14px; }}
    canvas {{ border: 1px solid #ccd3dc; display: block; width: 980px; max-width: 100%; height: 560px; }}
    .controls {{ margin-top: 12px; display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }}
    input[type="range"] {{ width: min(760px, 80vw); }}
    code {{ background: #f2f4f7; padding: 1px 4px; border-radius: 4px; }}
  </style>
</head>
<body>
  <h1>{html.escape(episode_id)}</h1>
  <div class="meta">
    <div>auto_success: <code>{str(auto_success).lower()}</code></div>
    <div>failure_mode: <code>{html.escape(failure_mode)}</code></div>
    <div>episode_quality: <code>{html.escape(quality)}</code></div>
    <div>final_object_height: <code>{final_height}</code></div>
  </div>
  <canvas id="replay" width="980" height="560"></canvas>
  <div class="controls">
    <button id="play">Play</button>
    <input id="slider" type="range" min="0" max="{max(len(frames) - 1, 0)}" value="0">
    <span id="label"></span>
  </div>
  <script>
    const frames = {payload};
    const canvas = document.getElementById("replay");
    const ctx = canvas.getContext("2d");
    const slider = document.getElementById("slider");
    const label = document.getElementById("label");
    const playButton = document.getElementById("play");
    let timer = null;

    function finiteValues(getter) {{
      return frames.map(getter).filter((v) => Number.isFinite(v));
    }}

    const xs = finiteValues((f) => f.object ? f.object[0] : f.ee[0]).concat(finiteValues((f) => f.ee[0]));
    const ys = finiteValues((f) => f.object ? f.object[1] : f.ee[1]).concat(finiteValues((f) => f.ee[1]));
    const zs = finiteValues((f) => f.object ? f.object[2] : f.ee[2]).concat(finiteValues((f) => f.ee[2]));
    const bounds = {{
      x0: Math.min(...xs) - 0.04, x1: Math.max(...xs) + 0.04,
      y0: Math.min(...ys) - 0.04, y1: Math.max(...ys) + 0.04,
      z0: Math.min(...zs, 0.0) - 0.01, z1: Math.max(...zs, 0.10) + 0.02,
    }};

    function map(v, a, b, c, d) {{
      if (Math.abs(b - a) < 1e-6) return (c + d) * 0.5;
      return c + (v - a) * (d - c) / (b - a);
    }}

    function drawPanelTitle(text, x, y) {{
      ctx.fillStyle = "#17202a";
      ctx.font = "16px system-ui";
      ctx.fillText(text, x, y);
    }}

    function drawFrame(idx) {{
      const f = frames[idx] || frames[0];
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#ffffff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      drawPanelTitle("Top view: object/EE XY", 30, 32);
      drawPanelTitle("Side view: object/EE XZ", 520, 32);

      ctx.strokeStyle = "#d8dee8";
      ctx.lineWidth = 1;
      ctx.strokeRect(30, 50, 420, 340);
      ctx.strokeRect(520, 50, 420, 340);

      function xy(point) {{
        return [map(point[0], bounds.x0, bounds.x1, 50, 430), map(point[1], bounds.y0, bounds.y1, 370, 70)];
      }}
      function xz(point) {{
        return [map(point[0], bounds.x0, bounds.x1, 540, 920), map(point[2], bounds.z0, bounds.z1, 370, 70)];
      }}

      for (let i = 0; i <= idx; i++) {{
        const p = frames[i].object;
        if (!p) continue;
        const a = xy(p);
        const b = xz(p);
        ctx.fillStyle = "rgba(46, 134, 193, 0.20)";
        ctx.fillRect(a[0] - 2, a[1] - 2, 4, 4);
        ctx.fillRect(b[0] - 2, b[1] - 2, 4, 4);
      }}

      if (f.object) {{
        const a = xy(f.object);
        const b = xz(f.object);
        ctx.fillStyle = "#2e86c1";
        ctx.fillRect(a[0] - 7, a[1] - 7, 14, 14);
        ctx.fillRect(b[0] - 7, b[1] - 7, 14, 14);
      }}
      const eeA = xy(f.ee);
      const eeB = xz(f.ee);
      ctx.fillStyle = "#d35400";
      ctx.beginPath(); ctx.arc(eeA[0], eeA[1], 6, 0, Math.PI * 2); ctx.fill();
      ctx.beginPath(); ctx.arc(eeB[0], eeB[1], 6, 0, Math.PI * 2); ctx.fill();

      ctx.fillStyle = "#17202a";
      ctx.font = "13px system-ui";
      ctx.fillText("object: blue square, EE: orange dot", 30, 416);
      ctx.fillText(`frame=${{f.frame_index}} object_z=${{f.object ? f.object[2].toFixed(4) : "n/a"}}`, 30, 438);
      ctx.fillText(`hand_cmd_mean=${{f.hand_cmd_mean.toFixed(3)}} hand_state_mean=${{f.hand_state_mean.toFixed(3)}}`, 30, 460);

      ctx.fillStyle = "#eef2f7";
      ctx.fillRect(520, 430, 360, 16);
      ctx.fillRect(520, 470, 360, 16);
      ctx.fillStyle = "#1f77b4";
      ctx.fillRect(520, 430, 360 * f.hand_cmd_mean, 16);
      ctx.fillStyle = "#2ca02c";
      ctx.fillRect(520, 470, 360 * f.hand_state_mean, 16);
      ctx.fillStyle = "#17202a";
      ctx.fillText("hand_cmd_mean", 520, 424);
      ctx.fillText("hand_state_mean", 520, 464);
      label.textContent = `${{idx + 1}} / ${{frames.length}}`;
    }}

    slider.addEventListener("input", () => drawFrame(Number(slider.value)));
    playButton.addEventListener("click", () => {{
      if (timer) {{
        clearInterval(timer);
        timer = null;
        playButton.textContent = "Play";
        return;
      }}
      playButton.textContent = "Pause";
      timer = setInterval(() => {{
        slider.value = (Number(slider.value) + 1) % frames.length;
        drawFrame(Number(slider.value));
      }}, 100);
    }});
    drawFrame(0);
  </script>
</body>
</html>
"""
    out_path.write_text(body, encoding="utf-8")


def _bounds(frames: list[dict[str, Any]]) -> dict[str, float]:
    xs = [frame["ee"][0] for frame in frames]
    ys = [frame["ee"][1] for frame in frames]
    zs = [frame["ee"][2] for frame in frames]
    for frame in frames:
        if frame["object"] is not None:
            xs.append(frame["object"][0])
            ys.append(frame["object"][1])
            zs.append(frame["object"][2])
    return {
        "x0": float(min(xs) - 0.04),
        "x1": float(max(xs) + 0.04),
        "y0": float(min(ys) - 0.04),
        "y1": float(max(ys) + 0.04),
        "z0": float(min(min(zs), 0.0) - 0.01),
        "z1": float(max(max(zs), 0.10) + 0.02),
    }


def _write_episode_mp4(
    *,
    out_path: Path,
    episode_id: str,
    auto_success: bool,
    failure_mode: str,
    quality: str,
    final_height: float | None,
    frames: list[dict[str, Any]],
    fps: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter

    bounds = _bounds(frames)
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), gridspec_kw={"height_ratios": [3, 1]})
    ax_xy = axes[0, 0]
    ax_xz = axes[0, 1]
    ax_cmd = axes[1, 0]
    ax_state = axes[1, 1]
    fig.suptitle(
        f"{episode_id} | auto_success={auto_success} | {failure_mode} | {quality} | final_z={final_height}",
        fontsize=10,
    )
    metadata = {"title": episode_id, "artist": "embodied_lab replay_episode_export.py"}
    writer = FFMpegWriter(fps=fps, metadata=metadata, bitrate=1800)

    with writer.saving(fig, str(out_path), dpi=130):
        for index, frame in enumerate(frames):
            for ax in axes.reshape(-1):
                ax.clear()

            trail = frames[: index + 1]
            obj_trail = [item["object"] for item in trail if item["object"] is not None]
            ee_trail = [item["ee"] for item in trail]
            if obj_trail:
                obj = np.asarray(obj_trail, dtype=np.float32)
                ax_xy.plot(obj[:, 0], obj[:, 1], color="#2e86c1", alpha=0.35, linewidth=1.0)
                ax_xz.plot(obj[:, 0], obj[:, 2], color="#2e86c1", alpha=0.35, linewidth=1.0)
                ax_xy.scatter(obj[-1, 0], obj[-1, 1], color="#2e86c1", marker="s", s=70, label="object")
                ax_xz.scatter(obj[-1, 0], obj[-1, 2], color="#2e86c1", marker="s", s=70, label="object")
            ee = np.asarray(ee_trail, dtype=np.float32)
            ax_xy.plot(ee[:, 0], ee[:, 1], color="#d35400", alpha=0.25, linewidth=1.0)
            ax_xz.plot(ee[:, 0], ee[:, 2], color="#d35400", alpha=0.25, linewidth=1.0)
            ax_xy.scatter(ee[-1, 0], ee[-1, 1], color="#d35400", s=50, label="ee")
            ax_xz.scatter(ee[-1, 0], ee[-1, 2], color="#d35400", s=50, label="ee")

            ax_xy.set_title("Top view XY")
            ax_xy.set_xlim(bounds["x0"], bounds["x1"])
            ax_xy.set_ylim(bounds["y0"], bounds["y1"])
            ax_xy.set_xlabel("x")
            ax_xy.set_ylabel("y")
            ax_xy.grid(True, alpha=0.25)
            ax_xy.legend(loc="best", fontsize=7)

            ax_xz.set_title("Side view XZ")
            ax_xz.set_xlim(bounds["x0"], bounds["x1"])
            ax_xz.set_ylim(bounds["z0"], bounds["z1"])
            ax_xz.set_xlabel("x")
            ax_xz.set_ylabel("z")
            ax_xz.grid(True, alpha=0.25)
            ax_xz.legend(loc="best", fontsize=7)

            ax_cmd.bar(["cmd"], [frame["hand_cmd_mean"]], color="#1f77b4")
            ax_state.bar(["state"], [frame["hand_state_mean"]], color="#2ca02c")
            ax_cmd.set_ylim(0.0, 1.0)
            ax_state.set_ylim(0.0, 1.0)
            ax_cmd.set_title("hand_cmd_mean")
            ax_state.set_title("hand_state_mean")
            ax_cmd.grid(True, axis="y", alpha=0.25)
            ax_state.grid(True, axis="y", alpha=0.25)

            object_z = frame["object"][2] if frame["object"] is not None else None
            fig.text(
                0.02,
                0.02,
                f"frame={frame['frame_index']} object_z={object_z} hand_cmd_mean={frame['hand_cmd_mean']:.3f} "
                f"hand_state_mean={frame['hand_state_mean']:.3f}",
                fontsize=8,
            )
            fig.tight_layout(rect=[0, 0.04, 1, 0.94])
            writer.grab_frame()
            fig.texts.clear()
    plt.close(fig)


def replay_export(
    export_root: str | Path,
    out_dir: str | Path,
    manual_review_out: str | Path | None = None,
    *,
    media: str = "html",
    fps: int = 10,
) -> dict[str, Any]:
    export_root = Path(export_root).resolve()
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    samples = _load_samples(export_root)
    by_episode: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for sample in samples:
        by_episode[str(sample["episode_id"])].append(sample)

    review_entries: dict[str, dict[str, Any]] = {}
    replay_files: list[str] = []
    mp4_files: list[str] = []
    index_lines = [
        "<!doctype html>",
        '<html lang="en"><head><meta charset="utf-8"><title>Episode Replays</title></head><body>',
        "<h1>Episode Replays</h1>",
        "<ul>",
    ]

    for episode_id, episode_samples in sorted(by_episode.items()):
        episode_samples.sort(key=lambda item: int(item["frame_index"]))
        final_sample = episode_samples[-1]
        auto_success = bool(final_sample["episode_success"])
        failure_mode = str(final_sample["episode_failure_mode"])
        final_pose = _object_pose(final_sample)
        final_height = final_pose[2] if final_pose is not None else None
        quality = _episode_quality(success=auto_success, failure_mode=failure_mode, final_height=final_height)
        frames = [_frame_payload(sample) for sample in episode_samples]
        filename_base = "__".join(
            [
                _filename_token(episode_id),
                f"auto_success-{str(auto_success).lower()}",
                f"failure_mode-{_filename_token(failure_mode)}",
                f"episode_quality-{_filename_token(quality)}",
                f"final_object_height-{_height_token(final_height)}",
            ]
        )
        links: list[str] = []
        if media in {"html", "both"}:
            filename = filename_base + ".html"
            out_path = out_dir / filename
            _write_episode_html(
                out_path=out_path,
                episode_id=episode_id,
                auto_success=auto_success,
                failure_mode=failure_mode,
                quality=quality,
                final_height=final_height,
                frames=frames,
            )
            replay_files.append(str(out_path))
            links.append(f'<a href="{html.escape(filename)}">html</a>')
        if media in {"mp4", "both"}:
            filename = filename_base + ".mp4"
            out_path = out_dir / filename
            _write_episode_mp4(
                out_path=out_path,
                episode_id=episode_id,
                auto_success=auto_success,
                failure_mode=failure_mode,
                quality=quality,
                final_height=final_height,
                frames=frames,
                fps=fps,
            )
            mp4_files.append(str(out_path))
            links.append(f'<a href="{html.escape(filename)}">mp4</a>')
        index_lines.append(f"<li>{html.escape(filename_base)}: {' | '.join(links)}</li>")
        review_entries[episode_id] = {
            "auto_success": auto_success,
            "failure_mode": failure_mode,
            "episode_quality": quality,
            "final_object_height": final_height,
            "manual_quality": "unreviewed",
            "use_for_bc": False,
            "note": "",
        }

    index_lines.extend(["</ul>", "</body></html>"])
    (out_dir / "index.html").write_text("\n".join(index_lines) + "\n", encoding="utf-8")

    manual_review_path = None
    if manual_review_out is not None:
        manual_review_path = Path(manual_review_out).resolve()
        manual_review_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"episodes": review_entries}
        with manual_review_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)

    summary = {
        "export_root": str(export_root),
        "out_dir": str(out_dir),
        "episode_count": len(by_episode),
        "replay_file_count": len(replay_files),
        "mp4_file_count": len(mp4_files),
        "manual_review_path": str(manual_review_path) if manual_review_path is not None else None,
        "replay_index": str(out_dir / "index.html"),
        "replay_files": replay_files,
        "mp4_files": mp4_files,
    }
    (out_dir / "replay_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate lightweight HTML replays for a structured episode export.")
    parser.add_argument("--export-root", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--manual-review-out", default=None)
    parser.add_argument("--media", choices=["html", "mp4", "both"], default="html")
    parser.add_argument("--fps", type=int, default=10)
    args = parser.parse_args()
    summary = replay_export(args.export_root, args.out_dir, args.manual_review_out, media=args.media, fps=args.fps)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
