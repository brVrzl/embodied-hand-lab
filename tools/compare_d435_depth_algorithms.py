from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from embodiment_core.types import CameraIntrinsics
from vision_interface.depth_processing import (
    PointCloud,
    depth_to_point_cloud,
    guided_depth_completion,
    remove_radius_outliers,
    remove_statistical_outliers,
    voxel_downsample,
)


def capture_comparison(
    *,
    output_dir: str | Path,
    serial: str | None,
    width: int = 848,
    height: int = 480,
    fps: int = 30,
    frames: int = 45,
    warmup_frames: int = 15,
    depth_min_m: float = 0.3,
    depth_max_m: float = 1.5,
) -> dict[str, Any]:
    import pyrealsense2 as rs

    if frames < 3:
        raise ValueError("frames must be at least 3 for temporal metrics.")
    if depth_max_m <= depth_min_m:
        raise ValueError("depth_max_m must be greater than depth_min_m.")

    pipeline = rs.pipeline()
    config = rs.config()
    if serial:
        config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    profile = pipeline.start(config)
    device = profile.get_device()
    sensor = device.first_depth_sensor()
    if sensor.supports(rs.option.visual_preset):
        sensor.set_option(rs.option.visual_preset, 1.0)
    depth_scale = float(sensor.get_depth_scale())
    align = rs.align(rs.stream.color)
    spatial_chain = _realsense_filter_chain(rs, temporal=False)
    temporal_chain = _realsense_filter_chain(rs, temporal=True)
    sequences: dict[str, list[np.ndarray]] = {"raw": [], "spatial": [], "temporal": []}
    rgb_sequence: list[np.ndarray] = []
    last_rgb: np.ndarray | None = None
    last_intrinsics: CameraIntrinsics | None = None
    frame_numbers: list[int] = []

    started = time.perf_counter()
    try:
        for index in range(warmup_frames + frames):
            frameset = align.process(pipeline.wait_for_frames(5000))
            color_frame = frameset.get_color_frame()
            depth_frame = frameset.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            spatial_frame = _process_filter_chain(depth_frame, spatial_chain)
            temporal_frame = _process_filter_chain(depth_frame, temporal_chain)
            if index < warmup_frames:
                continue
            raw = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
            spatial = np.asanyarray(spatial_frame.get_data()).astype(np.float32) * depth_scale
            temporal = np.asanyarray(temporal_frame.get_data()).astype(np.float32) * depth_scale
            sequences["raw"].append(raw.copy())
            sequences["spatial"].append(spatial.copy())
            sequences["temporal"].append(temporal.copy())
            last_rgb = np.asanyarray(color_frame.get_data()).copy()
            rgb_sequence.append(last_rgb)
            last_intrinsics = _intrinsics_from_frame(depth_frame)
            frame_numbers.append(int(depth_frame.get_frame_number()))
    finally:
        pipeline.stop()
    capture_seconds = time.perf_counter() - started

    if last_rgb is None or last_intrinsics is None or len(sequences["raw"]) < 3:
        raise RuntimeError("D435 did not produce enough aligned RGB-D frames.")

    # Guided completion deliberately starts from the spatial result and leaves
    # every valid metric measurement unchanged.
    guided_sequence = [
        guided_depth_completion(
            depth,
            rgb,
            radius_px=6,
            epsilon=1e-3,
            min_support=0.10,
            min_depth_m=depth_min_m,
            max_depth_m=depth_max_m,
        )
        for depth, rgb in zip(sequences["spatial"], rgb_sequence, strict=True)
    ]
    sequences["guided"] = guided_sequence
    arrays = {name: np.stack(items) for name, items in sequences.items()}
    last_depth = {name: values[-1] for name, values in arrays.items()}

    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output / "rgb.png"), cv2.cvtColor(last_rgb, cv2.COLOR_RGB2BGR))
    for name, depth in last_depth.items():
        cv2.imwrite(
            str(output / f"depth_{name}.png"),
            _depth_colormap(depth, depth_min_m=depth_min_m, depth_max_m=depth_max_m),
        )

    clouds = _build_cloud_variants(
        last_depth,
        last_rgb,
        last_intrinsics,
        depth_min_m=depth_min_m,
        depth_max_m=depth_max_m,
    )
    metrics = {
        name: _sequence_metrics(values, depth_min_m=depth_min_m, depth_max_m=depth_max_m)
        for name, values in arrays.items()
    }
    raw_last_valid = _valid_depth(last_depth["raw"], depth_min_m, depth_max_m)
    guided_last_valid = _valid_depth(last_depth["guided"], depth_min_m, depth_max_m)
    metrics["guided"]["new_valid_pixels_vs_raw"] = int(
        np.count_nonzero(guided_last_valid & ~raw_last_valid)
    )
    cloud_metrics = {name: {"point_count": len(cloud)} for name, cloud in clouds.items()}
    report = {
        "schema_version": "d435_algorithm_comparison_v1",
        "created_at_unix_s": time.time(),
        "device": {
            "name": _device_info(device, rs.camera_info.name),
            "serial": _device_info(device, rs.camera_info.serial_number),
            "firmware": _device_info(device, rs.camera_info.firmware_version),
            "sdk": _sdk_version(rs),
        },
        "capture": {
            "width": width,
            "height": height,
            "fps": fps,
            "frames": len(sequences["raw"]),
            "capture_seconds": capture_seconds,
            "observed_fps": (warmup_frames + len(sequences["raw"]))
            / max(capture_seconds, 1e-6),
            "first_frame_number": frame_numbers[0],
            "last_frame_number": frame_numbers[-1],
            "depth_range_m": [depth_min_m, depth_max_m],
        },
        "intrinsics": last_intrinsics.to_dict(),
        "depth_metrics": metrics,
        "cloud_metrics": cloud_metrics,
        "assessment": {
            "dynamic_runtime": "spatial",
            "static_capture_only": "temporal",
            "geometry_rejected": ["guided"],
            "point_cloud_runtime": "cleaned",
            "reason": (
                "Guided completion raised temporal P95 and visually expanded unobserved regions; "
                "temporal filtering reduced median noise but can retain moving geometry."
            ),
        },
    }
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    web_clouds = {name: _cloud_for_browser(cloud) for name, cloud in clouds.items()}
    (output / "index.html").write_text(
        _comparison_html(report, web_clouds), encoding="utf-8"
    )
    report["output_dir"] = str(output)
    report["browser_page"] = str(output / "index.html")
    return report


def _realsense_filter_chain(rs: Any, *, temporal: bool) -> list[Any]:
    chain = [
        rs.disparity_transform(True),
        rs.spatial_filter(0.5, 20.0, 2, 0),
    ]
    if temporal:
        chain.append(rs.temporal_filter(0.4, 20.0, 3))
    chain.append(rs.disparity_transform(False))
    return chain


def _process_filter_chain(frame: Any, chain: list[Any]) -> Any:
    output = frame
    for depth_filter in chain:
        output = depth_filter.process(output)
    return output


def _intrinsics_from_frame(frame: Any) -> CameraIntrinsics:
    values = frame.get_profile().as_video_stream_profile().get_intrinsics()
    model = str(values.model).rsplit(".", 1)[-1]
    return CameraIntrinsics(
        width=int(values.width),
        height=int(values.height),
        fx=float(values.fx),
        fy=float(values.fy),
        cx=float(values.ppx),
        cy=float(values.ppy),
        frame_id="camera_color_optical_frame",
        distortion_model=model,
        distortion_coefficients=[float(value) for value in values.coeffs],
    )


def _build_cloud_variants(
    depth: dict[str, np.ndarray],
    rgb: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    depth_min_m: float,
    depth_max_m: float,
) -> dict[str, PointCloud]:
    clouds = {}
    for name in ("raw", "spatial", "guided"):
        cloud = depth_to_point_cloud(
            depth[name],
            intrinsics,
            rgb=rgb,
            min_depth_m=depth_min_m,
            max_depth_m=depth_max_m,
        )
        clouds[name] = voxel_downsample(cloud, 0.005)
    cleaned = remove_statistical_outliers(clouds["spatial"], mean_k=16, std_ratio=1.5)
    cleaned = remove_radius_outliers(cleaned, radius_m=0.012, min_neighbors=3)
    clouds["cleaned"] = cleaned
    return clouds


def _sequence_metrics(
    depths: np.ndarray,
    *,
    depth_min_m: float,
    depth_max_m: float,
) -> dict[str, float | int]:
    valid = np.isfinite(depths) & (depths >= depth_min_m) & (depths <= depth_max_m)
    valid_ratio = float(np.mean(valid))
    minimum_samples = max(3, int(np.ceil(0.8 * len(depths))))
    stable = np.count_nonzero(valid, axis=0) >= minimum_samples
    values = np.where(valid, depths, np.nan)
    count = np.count_nonzero(valid, axis=0)
    total = np.nansum(values, axis=0)
    squared_total = np.nansum(values * values, axis=0)
    mean = np.divide(total, count, out=np.zeros_like(total), where=count > 0)
    variance = np.divide(
        squared_total,
        count,
        out=np.zeros_like(squared_total),
        where=count > 0,
    ) - mean * mean
    temporal_std = np.sqrt(np.maximum(variance, 0.0))
    noise_mm = temporal_std[stable] * 1000.0
    return {
        "valid_ratio": valid_ratio,
        "last_valid_pixels": int(np.count_nonzero(valid[-1])),
        "temporal_std_median_mm": float(np.median(noise_mm)) if noise_mm.size else float("nan"),
        "temporal_std_p95_mm": float(np.percentile(noise_mm, 95)) if noise_mm.size else float("nan"),
    }


def _valid_depth(depth: np.ndarray, minimum: float, maximum: float) -> np.ndarray:
    return np.isfinite(depth) & (depth >= minimum) & (depth <= maximum)


def _depth_colormap(
    depth: np.ndarray,
    *,
    depth_min_m: float,
    depth_max_m: float,
) -> np.ndarray:
    valid = _valid_depth(depth, depth_min_m, depth_max_m)
    normalized = np.zeros(depth.shape, dtype=np.float32)
    normalized[valid] = np.clip(
        (depth[valid] - depth_min_m) / (depth_max_m - depth_min_m), 0.0, 1.0
    )
    colored = cv2.applyColorMap(
        np.rint((1.0 - normalized) * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO
    )
    colored[~valid] = 0
    return colored


def _cloud_for_browser(cloud: PointCloud, maximum_points: int = 14000) -> dict[str, Any]:
    if len(cloud) > maximum_points:
        indices = np.linspace(0, len(cloud) - 1, maximum_points, dtype=np.int64)
    else:
        indices = np.arange(len(cloud))
    points = np.round(cloud.points_m[indices], 4)
    colors = (
        np.full((len(indices), 3), 210, dtype=np.uint8)
        if cloud.colors_rgb is None
        else cloud.colors_rgb[indices].astype(np.uint8)
    )
    return {"points": points.tolist(), "colors": colors.tolist(), "total": len(cloud)}


def _comparison_html(report: dict[str, Any], clouds: dict[str, Any]) -> str:
    metrics = report["depth_metrics"]
    rows = "".join(
        f"<tr><th>{name.title()}</th><td>{value['valid_ratio'] * 100:.1f}%</td>"
        f"<td>{value['temporal_std_median_mm']:.2f}</td>"
        f"<td>{value['temporal_std_p95_mm']:.2f}</td></tr>"
        for name, value in metrics.items()
    )
    cloud_json = json.dumps(clouds, separators=(",", ":"))
    device = report["device"]
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>D435 Depth Algorithm Comparison</title>
  <style>
    :root {{ color-scheme: dark; --bg:#101214; --panel:#191c1f; --line:#343a40; --text:#f2f3f4; --muted:#aab1b8; --accent:#39c58a; }}
    * {{ box-sizing:border-box; }}
    body {{ margin:0; background:var(--bg); color:var(--text); font-family:system-ui,sans-serif; letter-spacing:0; }}
    header {{ height:58px; display:flex; align-items:center; gap:18px; padding:0 20px; border-bottom:1px solid var(--line); background:#151719; }}
    header strong {{ font-size:18px; }} .meta {{ color:var(--muted); font-size:13px; }}
    main {{ width:min(1500px,100%); margin:auto; padding:18px; }}
    h2 {{ font-size:16px; margin:4px 0 12px; }}
    .overview {{ display:grid; grid-template-columns:minmax(420px,1fr) minmax(440px,1.25fr); gap:16px; align-items:start; }}
    .rgb img {{ width:100%; display:block; border:1px solid var(--line); border-radius:4px; }}
    table {{ width:100%; border-collapse:collapse; background:var(--panel); font-size:14px; }}
    th,td {{ padding:11px 12px; border-bottom:1px solid var(--line); text-align:right; }}
    th:first-child {{ text-align:left; }} thead th {{ color:var(--muted); font-weight:600; }}
    .depths {{ display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:12px; }}
    figure {{ margin:0; background:var(--panel); border:1px solid var(--line); border-radius:4px; overflow:hidden; }}
    figure img {{ display:block; width:100%; }} figcaption {{ padding:9px 11px; font-size:13px; color:var(--muted); }}
    .cloud-toolbar {{ display:flex; align-items:center; gap:8px; margin-bottom:10px; }}
    button {{ width:40px; height:34px; border:1px solid var(--line); background:#202428; color:var(--text); cursor:pointer; }}
    button[data-active="true"] {{ border-color:var(--accent); color:var(--accent); }}
    button.text {{ width:auto; padding:0 12px; }}
    canvas {{ width:100%; height:560px; display:block; background:#090b0c; border:1px solid var(--line); border-radius:4px; cursor:grab; }}
    section {{ margin-top:22px; }}
    @media (max-width:900px) {{ .overview,.depths {{ grid-template-columns:1fr; }} canvas {{ height:430px; }} header .meta {{ display:none; }} }}
  </style>
</head>
<body>
<header><strong>D435 深度算法对比</strong><span class="meta">{device['name']} · {device['serial']} · FW {device['firmware']} · SDK {device['sdk']}</span></header>
<main>
  <div class="overview">
    <div class="rgb"><h2>同帧 RGB</h2><img src="rgb.png" alt="RGB reference frame"></div>
    <div><h2>量化结果（0.3–1.5 m）</h2><table><thead><tr><th>方法</th><th>有效率</th><th>时序噪声中位数 mm</th><th>P95 mm</th></tr></thead><tbody>{rows}</tbody></table></div>
  </div>
  <section><h2>固定量程深度图</h2><div class="depths">
    <figure><img src="depth_raw.png"><figcaption>Raw：D435 ASIC 原始深度</figcaption></figure>
    <figure><img src="depth_spatial.png"><figcaption>Spatial：动态操作默认</figcaption></figure>
    <figure><img src="depth_temporal.png"><figcaption>Temporal：仅静态采集</figcaption></figure>
    <figure><img src="depth_guided.png"><figcaption>Guided：拒绝用于几何</figcaption></figure>
  </div></section>
  <section><h2>点云效果</h2><div class="cloud-toolbar">
    <button class="text" data-cloud="raw" data-active="true">Raw</button><button class="text" data-cloud="spatial">Spatial</button><button class="text" data-cloud="guided">Guided</button><button class="text" data-cloud="cleaned">Cleaned</button>
    <span class="meta" id="cloudMeta"></span>
  </div><canvas id="cloud"></canvas></section>
</main>
<script>
const clouds={cloud_json}; const canvas=document.getElementById('cloud'); const ctx=canvas.getContext('2d');
let active='raw', yaw=0, pitch=-0.15, zoom=620, dragging=false, px=0, py=0;
function resize() {{ const d=devicePixelRatio||1; canvas.width=Math.round(canvas.clientWidth*d); canvas.height=Math.round(canvas.clientHeight*d); draw(); }}
function draw() {{
  const d=devicePixelRatio||1,w=canvas.width,h=canvas.height; ctx.fillStyle='#090b0c';ctx.fillRect(0,0,w,h);
  const cloud=clouds[active], pts=cloud.points, cols=cloud.colors; if(!pts.length)return;
  let sx=0,sy=0,sz=0; for(const p of pts){{sx+=p[0];sy+=p[1];sz+=p[2];}} sx/=pts.length;sy/=pts.length;sz/=pts.length;
  const cy=Math.cos(yaw),syaw=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch), projected=[];
  for(let i=0;i<pts.length;i++){{let x=pts[i][0]-sx,y=pts[i][1]-sy,z=pts[i][2]-sz;let x1=cy*x+syaw*z,z1=-syaw*x+cy*z;let y1=cp*y-sp*z1,z2=sp*y+cp*z1;projected.push([z2,w/2+x1*zoom*d,h/2+y1*zoom*d,i]);}}
  projected.sort((a,b)=>b[0]-a[0]); const size=Math.max(1.2*d,1);
  for(const p of projected){{const c=cols[p[3]];ctx.fillStyle=`rgb(${{c[0]}},${{c[1]}},${{c[2]}})`;ctx.fillRect(p[1],p[2],size,size);}}
  document.getElementById('cloudMeta').textContent=`${{cloud.total.toLocaleString()}} points`;
}}
document.querySelectorAll('[data-cloud]').forEach(b=>b.onclick=()=>{{active=b.dataset.cloud;document.querySelectorAll('[data-cloud]').forEach(x=>x.dataset.active='false');b.dataset.active='true';draw();}});
canvas.onpointerdown=e=>{{dragging=true;px=e.clientX;py=e.clientY;canvas.setPointerCapture(e.pointerId);canvas.style.cursor='grabbing';}};
canvas.onpointermove=e=>{{if(!dragging)return;yaw+=(e.clientX-px)*0.008;pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-py)*0.008));px=e.clientX;py=e.clientY;draw();}};
canvas.onpointerup=()=>{{dragging=false;canvas.style.cursor='grab';}}; canvas.onwheel=e=>{{e.preventDefault();zoom=Math.max(120,Math.min(1800,zoom*Math.exp(-e.deltaY*0.001)));draw();}};
addEventListener('resize',resize);resize();
</script>
</body></html>"""


def _device_info(device: Any, key: Any) -> str:
    try:
        return str(device.get_info(key))
    except Exception:
        return "unknown"


def _sdk_version(rs: Any) -> str:
    try:
        from importlib.metadata import version

        return version("pyrealsense2")
    except Exception:
        return str(getattr(rs, "__version__", "unknown"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare lightweight D435 depth and point-cloud filters.")
    parser.add_argument("--output-dir", default="data/reports/d435_algorithm_compare")
    parser.add_argument("--serial", default=None)
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frames", type=int, default=45)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--depth-min-m", type=float, default=0.3)
    parser.add_argument("--depth-max-m", type=float, default=1.5)
    args = parser.parse_args()
    result = capture_comparison(
        output_dir=args.output_dir,
        serial=args.serial,
        width=args.width,
        height=args.height,
        fps=args.fps,
        frames=args.frames,
        warmup_frames=args.warmup_frames,
        depth_min_m=args.depth_min_m,
        depth_max_m=args.depth_max_m,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
