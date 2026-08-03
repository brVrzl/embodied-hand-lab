from __future__ import annotations

from dataclasses import dataclass, replace
import threading
import time
from types import ModuleType

import numpy as np

from .camera import AsyncRGBDCamera
from .collector import CaptureState
from .episode import CameraSample


_CV2: ModuleType | None = None


def require_preview_dependencies() -> ModuleType:
    """Load the optional GUI dependency only when preview is requested."""

    global _CV2
    if _CV2 is not None:
        return _CV2
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "episode preview requires OpenCV; install embodied-lab[vision-teleop]"
        ) from exc
    _CV2 = cv2
    return cv2


@dataclass(frozen=True, slots=True)
class PreviewStatus:
    state: CaptureState
    temporary_id: str
    episode_start_ns: int | None
    arm_trigger: bool
    hand_grip: bool
    recording_frame_count: int


class DualCameraPreview:
    """Best-effort display; capture never waits for preview rendering."""

    def __init__(self, window_name: str = "JAKA/RH56 episode capture") -> None:
        self.cv2 = require_preview_dependencies()
        self.window_name = window_name
        self._opened = False

    def render(
        self,
        workspace: AsyncRGBDCamera,
        wrist: AsyncRGBDCamera,
        status: PreviewStatus,
    ) -> bool:
        workspace_frame = workspace.latest()
        wrist_frame = wrist.latest()
        if workspace_frame is None or wrist_frame is None:
            return True
        panel = np.vstack(
            [
                np.hstack(
                    [
                        _rgb_panel(workspace_frame, "workspace RGB"),
                        _depth_panel(workspace_frame, "workspace depth"),
                    ]
                ),
                np.hstack(
                    [
                        _rgb_panel(wrist_frame, "wrist RGB"),
                        _depth_panel(wrist_frame, "wrist depth"),
                    ]
                ),
            ]
        )
        elapsed = (
            0.0
            if status.episode_start_ns is None
            else max(0.0, (time.monotonic_ns() - status.episode_start_ns) / 1e9)
        )
        lines = [
            f"{status.state.value}  episode={status.temporary_id}  elapsed={elapsed:.2f}s",
            f"workspace={workspace.actual_fps:.1f}fps drop={workspace.dropped_frames}  "
            f"wrist={wrist.actual_fps:.1f}fps drop={wrist.dropped_frames}",
            f"RGB-depth age: workspace={workspace_frame.device_depth_timestamp_ms - workspace_frame.device_rgb_timestamp_ms:+.2f}ms  "
            f"wrist={wrist_frame.device_depth_timestamp_ms - wrist_frame.device_rgb_timestamp_ms:+.2f}ms",
            f"arm_trigger={int(status.arm_trigger)} hand_grip={int(status.hand_grip)} "
            f"recording_frames={status.recording_frame_count}",
        ]
        for index, line in enumerate(lines):
            self.cv2.putText(
                panel,
                line,
                (12, 24 + index * 23),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                self.cv2.LINE_AA,
            )
        self.cv2.imshow(self.window_name, panel)
        self._opened = True
        key = self.cv2.waitKey(1) & 0xFF
        if key == 27:
            return False
        try:
            return self.cv2.getWindowProperty(
                self.window_name, self.cv2.WND_PROP_VISIBLE
            ) >= 1
        except self.cv2.error:
            return False

    def close(self) -> None:
        if self._opened:
            self.cv2.destroyWindow(self.window_name)
            self._opened = False


class AsyncDualCameraPreview:
    """Own all GUI work on a best-effort thread outside control/capture."""

    def __init__(
        self,
        workspace: AsyncRGBDCamera,
        wrist: AsyncRGBDCamera,
        initial_status: PreviewStatus,
        *,
        refresh_hz: float = 30.0,
    ) -> None:
        self.workspace = workspace
        self.wrist = wrist
        self.refresh_hz = float(refresh_hz)
        self._status = initial_status
        self._status_lock = threading.Lock()
        self._stop = threading.Event()
        self._closed = threading.Event()
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, name="episode-preview", daemon=True)

    def start(self) -> None:
        require_preview_dependencies()
        self._thread.start()

    def update(self, status: PreviewStatus) -> None:
        with self._status_lock:
            self._status = status

    def set_capture_state(self, state: CaptureState) -> None:
        with self._status_lock:
            self._status = replace(self._status, state=state)

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    @property
    def error(self) -> BaseException | None:
        return self._error

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        renderer = DualCameraPreview()
        try:
            while not self._stop.is_set():
                with self._status_lock:
                    status = self._status
                if not renderer.render(self.workspace, self.wrist, status):
                    self._closed.set()
                    return
                self._stop.wait(max(1.0 / self.refresh_hz, 0.001))
        except BaseException as exc:
            self._error = exc
            self._closed.set()
        finally:
            renderer.close()


def _rgb_panel(frame: CameraSample, label: str) -> np.ndarray:
    cv2 = require_preview_dependencies()
    panel = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
    return _label(panel, label)


def _depth_panel(frame: CameraSample, label: str) -> np.ndarray:
    cv2 = require_preview_dependencies()
    depth = (
        frame.depth_aligned_to_rgb
        if frame.depth_aligned_to_rgb is not None
        else frame.depth_raw
    )
    if frame.depth_scale_m is not None:
        depth_m = depth.astype(np.float32) * frame.depth_scale_m
        valid = np.isfinite(depth_m) & (depth_m >= 0.15) & (depth_m <= 1.5)
        normalized = np.zeros(depth.shape, dtype=np.float32)
        normalized[valid] = (1.5 - depth_m[valid]) / (1.5 - 0.15)
        scaled = np.rint(normalized * 255.0).astype(np.uint8)
        panel = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        panel[~valid] = 0
    else:
        valid_depth = depth[depth > 0]
        if valid_depth.size:
            high = max(float(np.percentile(valid_depth, 95)), 1.0)
            scaled = np.clip(
                depth.astype(np.float32) * (255.0 / high), 0, 255
            ).astype(np.uint8)
        else:
            scaled = np.zeros(depth.shape, dtype=np.uint8)
        panel = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        panel[depth == 0] = 0
    return _label(panel, label)


def _label(panel: np.ndarray, label: str) -> np.ndarray:
    cv2 = require_preview_dependencies()
    result = panel.copy()
    cv2.putText(
        result,
        label,
        (10, result.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return result
