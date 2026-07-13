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

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER
from teleop_tools.iphone_hand import apply_retarget_safety, retarget_mediapipe_landmarks_to_rh56
from vision_interface.realsense_adapter import RealSenseCamera, list_realsense_devices


class RealSenseFrameHub:
    def __init__(self, config: dict[str, object]) -> None:
        self.config = config
        self.camera: RealSenseCamera | None = None
        self.latest_jpeg: bytes | None = None
        self.latest_error = ""
        self.latest_hand_status: dict[str, Any] = {}
        self.frames = 0
        self._hand_frame_index = 0
        self._last_hand_landmarks: list[dict[str, float]] | None = None
        self.started_at = time.time()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._previous_hand_target: list[float] | None = None

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

    def hand_status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self.latest_hand_status)

    def _run(self) -> None:
        hands = None
        try:
            if self.config.get("hand_overlay"):
                import mediapipe as mp  # type: ignore

                hands = mp.solutions.hands.Hands(
                    max_num_hands=1,
                    min_detection_confidence=float(self.config.get("min_detection_confidence", 0.6)),
                    min_tracking_confidence=float(self.config.get("min_tracking_confidence", 0.5)),
                )
            with RealSenseCamera(self.config) as camera:
                self.camera = camera
                while not self._stop.is_set():
                    rgb = camera.get_rgb()
                    depth = camera.get_depth() if self.config.get("view_mode") == "rgbd" else None
                    hand_status: dict[str, Any] = {"enabled": bool(hands is not None), "hand_detected": False}
                    if hands is not None:
                        rgb, hand_status = self._process_hand_overlay(rgb, hands)
                    panel = _make_preview_panel(rgb, depth, hand_status=hand_status)
                    ok, encoded = cv2.imencode(
                        ".jpg",
                        panel,
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.get("jpeg_quality", 75))],
                    )
                    if not ok:
                        continue
                    with self._lock:
                        self.latest_jpeg = encoded.tobytes()
                        self.latest_error = ""
                        self.latest_hand_status = hand_status
                        self.frames += 1
        except Exception as exc:
            with self._lock:
                self.latest_error = f"{type(exc).__name__}: {exc}"
        finally:
            if hands is not None:
                hands.close()

    def _process_hand_overlay(self, rgb: np.ndarray, hands: Any) -> tuple[np.ndarray, dict[str, Any]]:
        self._hand_frame_index += 1
        process_every = max(int(self.config.get("hand_process_every", 1)), 1)
        should_process = self._hand_frame_index % process_every == 1 or self._last_hand_landmarks is None
        hands_landmarks = None
        result = None
        if should_process:
            inference_width = int(self.config.get("hand_inference_width", 0))
            if inference_width > 0 and inference_width < rgb.shape[1]:
                scale = inference_width / float(rgb.shape[1])
                inference_height = max(int(round(rgb.shape[0] * scale)), 1)
                inference_rgb = cv2.resize(rgb, (inference_width, inference_height), interpolation=cv2.INTER_AREA)
            else:
                inference_rgb = rgb
            result = hands.process(inference_rgb)
            hands_landmarks = getattr(result, "multi_hand_landmarks", None)
        status: dict[str, Any] = {
            "enabled": True,
            "hand_detected": False,
            "target_norm": self._previous_hand_target,
            "canonical_order": list(CANONICAL_HAND_ORDER),
        }
        if should_process and not hands_landmarks:
            self._last_hand_landmarks = None
            self._previous_hand_target = apply_retarget_safety(
                self._previous_hand_target,
                [0.0] * len(CANONICAL_HAND_ORDER),
                delta_limit=float(self.config.get("delta_limit", 0.05)),
                max_close=float(self.config.get("max_close", 0.85)),
            )
            status["target_norm"] = self._previous_hand_target
            return rgb, status
        if not should_process and self._last_hand_landmarks is None:
            return rgb, status

        handedness = "unknown"
        score = 0.0
        multi_handedness = getattr(result, "multi_handedness", None) if result is not None else None
        if multi_handedness:
            classification = multi_handedness[0].classification[0]
            handedness = str(classification.label)
            score = float(classification.score)

        if should_process and hands_landmarks:
            landmarks = [
                {"x": float(item.x), "y": float(item.y), "z": float(item.z)}
                for item in hands_landmarks[0].landmark
            ]
            self._last_hand_landmarks = landmarks
            retarget = retarget_mediapipe_landmarks_to_rh56(
                landmarks,
                max_close=float(self.config.get("max_close", 0.85)),
                thumb_mode=str(self.config.get("thumb_mode", "rh56_task")),
            )
            self._previous_hand_target = apply_retarget_safety(
                self._previous_hand_target,
                retarget.target_norm,
                delta_limit=float(self.config.get("delta_limit", 0.05)),
                max_close=float(self.config.get("max_close", 0.85)),
            )
            raw_count = retarget.target_raw_count
            features = retarget.features
        else:
            landmarks = self._last_hand_landmarks or []
            raw_count = None
            features = {}

        annotated = _draw_hand_landmarks(rgb, landmarks)
        status.update(
            {
                "hand_detected": True,
                "handedness": handedness,
                "score": score,
                "target_norm": self._previous_hand_target,
                "raw_count": raw_count,
                "features": features,
            }
        )
        return annotated, status


HAND_CONNECTIONS = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (13, 17),
    (17, 18),
    (18, 19),
    (19, 20),
    (0, 17),
)


def _draw_hand_landmarks(rgb: np.ndarray, landmarks: list[dict[str, float]]) -> np.ndarray:
    output = rgb.copy()
    height, width = output.shape[:2]
    points = [
        (
            int(np.clip(item["x"], 0.0, 1.0) * (width - 1)),
            int(np.clip(item["y"], 0.0, 1.0) * (height - 1)),
        )
        for item in landmarks
    ]
    for start, end in HAND_CONNECTIONS:
        cv2.line(output, points[start], points[end], (0, 220, 255), 2, cv2.LINE_AA)
    for index, point in enumerate(points):
        color = (255, 80, 80) if index in {4, 8, 12, 16, 20} else (80, 255, 120)
        cv2.circle(output, point, 4, color, -1, cv2.LINE_AA)
    return output


def _make_preview_panel(rgb: np.ndarray, depth_m: np.ndarray | None, *, hand_status: dict[str, Any] | None = None) -> np.ndarray:
    rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if hand_status:
        _draw_hand_status(rgb_bgr, hand_status)
    if depth_m is None:
        cv2.putText(rgb_bgr, "RGB", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        return rgb_bgr
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


def _draw_hand_status(frame_bgr: np.ndarray, status: dict[str, Any]) -> None:
    height, width = frame_bgr.shape[:2]
    detected = bool(status.get("hand_detected"))
    label = "HAND OK" if detected else "NO HAND"
    color = (40, 220, 80) if detected else (40, 80, 255)
    cv2.rectangle(frame_bgr, (8, height - 190), (min(width - 8, 368), height - 8), (15, 15, 15), -1)
    cv2.rectangle(frame_bgr, (8, height - 190), (min(width - 8, 368), height - 8), color, 2)
    cv2.putText(frame_bgr, label, (22, height - 158), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
    if detected:
        handedness = str(status.get("handedness", "unknown"))
        score = float(status.get("score", 0.0))
        cv2.putText(
            frame_bgr,
            f"{handedness} {score:.2f}",
            (170, height - 158),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )

    targets = status.get("target_norm") or [0.0] * len(CANONICAL_HAND_ORDER)
    for i, (name, value) in enumerate(zip(CANONICAL_HAND_ORDER, targets, strict=False)):
        y = height - 128 + i * 20
        value = float(np.clip(value, 0.0, 1.0))
        cv2.putText(frame_bgr, name[:10], (22, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1, cv2.LINE_AA)
        x0, x1 = 130, 330
        cv2.rectangle(frame_bgr, (x0, y - 6), (x1, y + 6), (70, 70, 70), 1)
        cv2.rectangle(frame_bgr, (x0, y - 6), (int(x0 + (x1 - x0) * value), y + 6), (80, 180, 255), -1)
        cv2.putText(frame_bgr, f"{value:.2f}", (x1 + 8, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1, cv2.LINE_AA)


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
    #status {{ margin-left: auto; color: #bbb; font-size: 13px; }}
  </style>
</head>
<body>
  <header>
    <strong>RealSense RGB-D</strong>
    <span class="muted">{device_text}</span>
    <span id="status"></span>
  </header>
  <main><img src="/stream.mjpg" alt="RealSense RGB and depth stream"></main>
  <script>
    async function tick() {{
      try {{
        const res = await fetch('/status.json', {{cache: 'no-store'}});
        const data = await res.json();
        const hand = data.hand || {{}};
        const handText = hand.enabled ? (hand.hand_detected ? `hand ${{hand.score?.toFixed?.(2) ?? ''}}` : 'no hand') : 'hand off';
        document.getElementById('status').textContent = `${{data.fps.toFixed(1)}} fps | ${{handText}}`;
      }} catch (e) {{}}
    }}
    setInterval(tick, 500);
    tick();
  </script>
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
            body = json.dumps({"frames": frames, "fps": fps, "error": error, "hand": hub.hand_status()}).encode("utf-8")
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
    parser.add_argument("--view-mode", choices=["rgb", "rgbd"], default="rgbd")
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--hand-overlay", action="store_true", help="Overlay MediaPipe hand landmarks and RH56 retarget bars.")
    parser.add_argument("--hand-process-every", type=int, default=1)
    parser.add_argument("--hand-inference-width", type=int, default=0)
    parser.add_argument("--max-close", type=float, default=0.85)
    parser.add_argument("--delta-limit", type=float, default=0.05)
    parser.add_argument("--thumb-mode", default="rh56_task")
    parser.add_argument("--min-detection-confidence", type=float, default=0.6)
    parser.add_argument("--min-tracking-confidence", type=float, default=0.5)
    args = parser.parse_args()

    devices = list_realsense_devices()
    config: dict[str, object] = {
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "hand_overlay": args.hand_overlay,
        "view_mode": args.view_mode,
        "jpeg_quality": args.jpeg_quality,
        "hand_process_every": args.hand_process_every,
        "hand_inference_width": args.hand_inference_width,
        "max_close": args.max_close,
        "delta_limit": args.delta_limit,
        "thumb_mode": args.thumb_mode,
        "min_detection_confidence": args.min_detection_confidence,
        "min_tracking_confidence": args.min_tracking_confidence,
    }
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
