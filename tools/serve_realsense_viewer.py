from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import cv2
import numpy as np

from vision_interface.realsense_adapter import RealSenseCamera, list_realsense_devices


class RealSenseFrameHub:
    def __init__(self, config: dict[str, object]) -> None:
        self.config = config
        self.camera: RealSenseCamera | None = None
        self.latest_jpeg: bytes | None = None
        self.latest_error = ""
        self.frames = 0
        self.started_at = time.time()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self.camera is not None:
            self.camera.close()

    def snapshot(self) -> tuple[bytes | None, str, int, float]:
        with self._lock:
            elapsed = max(time.time() - self.started_at, 1e-6)
            return self.latest_jpeg, self.latest_error, self.frames, self.frames / elapsed

    def _run(self) -> None:
        try:
            with RealSenseCamera(self.config) as camera:
                self.camera = camera
                while not self._stop.is_set():
                    rgb = camera.get_rgb()
                    depth = camera.get_depth()
                    panel = _make_preview_panel(rgb, depth)
                    ok, encoded = cv2.imencode(".jpg", panel, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                    if not ok:
                        continue
                    with self._lock:
                        self.latest_jpeg = encoded.tobytes()
                        self.latest_error = ""
                        self.frames += 1
        except Exception as exc:
            with self._lock:
                self.latest_error = f"{type(exc).__name__}: {exc}"


def _make_preview_panel(rgb: np.ndarray, depth_m: np.ndarray) -> np.ndarray:
    rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    finite_depth = depth_m[np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < 10.0)]
    if finite_depth.size:
        max_depth = max(float(np.percentile(finite_depth, 95)), 0.5)
    else:
        max_depth = 2.0
    depth_u8 = np.clip(depth_m / max_depth * 255.0, 0, 255).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
    panel = np.hstack([rgb_bgr, depth_color])
    cv2.putText(panel, "RGB", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(
        panel,
        "Depth",
        (rgb_bgr.shape[1] + 12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return panel


def _handler_class(hub: RealSenseFrameHub, devices: list[dict[str, str]]) -> type[BaseHTTPRequestHandler]:
    class RealSenseViewerHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path in {"/", "/index.html"}:
                self._send_html(devices)
            elif self.path == "/stream.mjpg":
                self._send_stream()
            elif self.path == "/status.json":
                self._send_status()
            else:
                self.send_error(404)

        def log_message(self, format: str, *args: Any) -> None:
            return

        def _send_html(self, devices_info: list[dict[str, str]]) -> None:
            device_text = ", ".join(
                f"{device.get('name', '')} serial={device.get('serial', '')} usb={device.get('usb_type', '')}"
                for device in devices_info
            )
            html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RealSense Viewer</title>
  <style>
    body {{ margin: 0; background: #111; color: #eee; font-family: system-ui, sans-serif; }}
    header {{ padding: 12px 16px; background: #1d1d1d; display: flex; gap: 16px; align-items: baseline; }}
    main {{ display: grid; place-items: center; min-height: calc(100vh - 52px); }}
    img {{ max-width: 100vw; max-height: calc(100vh - 52px); object-fit: contain; }}
    .muted {{ color: #bbb; font-size: 13px; }}
  </style>
</head>
<body>
  <header>
    <strong>RealSense RGB-D</strong>
    <span class="muted">{device_text}</span>
  </header>
  <main><img src="/stream.mjpg" alt="RealSense RGB and depth stream"></main>
</body>
</html>"""
            body = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_status(self) -> None:
            _, error, frames, fps = hub.snapshot()
            body = json.dumps({"frames": frames, "fps": fps, "error": error}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_stream(self) -> None:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                jpeg, error, _, _ = hub.snapshot()
                if error:
                    break
                if jpeg is None:
                    time.sleep(0.03)
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
                time.sleep(0.01)

    return RealSenseViewerHandler


def _pick_port(host: str, preferred_port: int) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind((host, preferred_port))
            return preferred_port
        except OSError:
            probe.bind((host, 0))
            return int(probe.getsockname()[1])


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve a browser-based RealSense RGB-D live viewer.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--serial", default=None)
    args = parser.parse_args()

    devices = list_realsense_devices()
    config: dict[str, object] = {"width": args.width, "height": args.height, "fps": args.fps}
    if args.serial:
        config["serial"] = args.serial

    hub = RealSenseFrameHub(config)
    hub.start()
    port = _pick_port(args.host, args.port)
    server = ThreadingHTTPServer((args.host, port), _handler_class(hub, devices))
    print(f"RealSense viewer: http://{args.host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        hub.stop()


if __name__ == "__main__":
    main()
