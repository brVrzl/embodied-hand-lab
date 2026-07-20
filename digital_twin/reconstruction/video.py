from __future__ import annotations

import json
import math
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import cv2
import numpy as np


VIDEO_SUFFIXES = {".mov", ".mp4"}
SENSITIVE_TAG_TERMS = ("location", "gps", "latitude", "longitude", "iso6709")


def discover_root_videos(root: str | Path) -> list[Path]:
    directory = Path(root)
    return sorted(
        path for path in directory.iterdir() if path.is_file() and path.suffix.lower() in VIDEO_SUFFIXES
    )


def _fraction(value: str | None) -> float | None:
    if not value or value in {"0/0", "N/A"}:
        return None
    if "/" in value:
        numerator, denominator = value.split("/", 1)
        return float(numerator) / float(denominator) if float(denominator) else None
    return float(value)


def _redact_tags(tags: dict[str, Any]) -> tuple[dict[str, Any], bool]:
    sensitive = any(any(term in key.lower() for term in SENSITIVE_TAG_TERMS) for key in tags)
    clean = {
        key: value
        for key, value in tags.items()
        if not any(term in key.lower() for term in SENSITIVE_TAG_TERMS)
    }
    return clean, sensitive


def probe_video(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"Video does not exist: {source}")
    ffprobe = shutil.which("ffprobe")
    if ffprobe:
        command = [
            ffprobe,
            "-v", "error",
            "-show_format", "-show_streams",
            "-of", "json",
            str(source),
        ]
        raw = json.loads(subprocess.run(command, check=True, capture_output=True, text=True).stdout)
        streams = raw.get("streams", [])
        video_streams = [stream for stream in streams if stream.get("codec_type") == "video"]
        audio_streams = [stream for stream in streams if stream.get("codec_type") == "audio"]
        if not video_streams:
            raise RuntimeError(f"No video stream found in: {source}")
        stream = video_streams[0]
        format_tags, format_sensitive = _redact_tags(raw.get("format", {}).get("tags", {}))
        stream_tags, stream_sensitive = _redact_tags(stream.get("tags", {}))
        side_rotation = next(
            (
                item.get("rotation")
                for side in stream.get("side_data_list", [])
                for item in [side]
                if "rotation" in item
            ),
            None,
        )
        nominal = _fraction(stream.get("r_frame_rate"))
        average = _fraction(stream.get("avg_frame_rate"))
        return {
            "filename": source.name,
            "file_size_bytes": source.stat().st_size,
            "backend": "ffprobe",
            "duration_sec": float(raw.get("format", {}).get("duration") or stream.get("duration")),
            "width": int(stream["width"]),
            "height": int(stream["height"]),
            "nominal_frame_rate_fps": nominal,
            "average_frame_rate_fps": average,
            "codec": stream.get("codec_name"),
            "codec_long_name": stream.get("codec_long_name"),
            "pixel_format": stream.get("pix_fmt"),
            "color_metadata": {key: stream.get(key) for key in ("color_range", "color_space", "color_transfer", "color_primaries")},
            "rotation_degrees": side_rotation if side_rotation is not None else stream_tags.get("rotate"),
            "variable_frame_rate_indicator": (
                None if nominal is None or average is None else abs(nominal - average) > max(0.01, nominal * 0.001)
            ),
            "audio_present": bool(audio_streams),
            "timestamp_metadata": {
                "creation_time": format_tags.get("creation_time") or stream_tags.get("creation_time"),
                "start_time": raw.get("format", {}).get("start_time"),
                "time_base": stream.get("time_base"),
                "frame_count": int(stream["nb_frames"]) if str(stream.get("nb_frames", "")).isdigit() else None,
            },
            "device_metadata": {
                key: value for key, value in format_tags.items()
                if "make" in key.lower() or "model" in key.lower() or "software" in key.lower()
            },
            "location_metadata_present": bool(format_sensitive or stream_sensitive),
            "metadata_limitations": [],
        }
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"OpenCV could not open video: {source}")
    fps = float(capture.get(cv2.CAP_PROP_FPS)) or None
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) or None
    fourcc_value = int(capture.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fourcc_value >> (8 * index)) & 0xFF) for index in range(4)).strip("\x00")
    rotation = (
        float(capture.get(cv2.CAP_PROP_ORIENTATION_META))
        if hasattr(cv2, "CAP_PROP_ORIENTATION_META")
        else None
    )
    result = {
        "filename": source.name,
        "file_size_bytes": source.stat().st_size,
        "backend": "opencv_ffmpeg_fallback",
        "duration_sec": frame_count / fps if frame_count and fps else None,
        "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "nominal_frame_rate_fps": fps,
        "average_frame_rate_fps": None,
        "codec": fourcc or None,
        "codec_long_name": None,
        "pixel_format": None,
        "color_metadata": None,
        "rotation_degrees": rotation,
        "variable_frame_rate_indicator": None,
        "audio_present": None,
        "timestamp_metadata": {"creation_time": None, "start_time": None, "time_base": None, "frame_count": frame_count},
        "device_metadata": {},
        "location_metadata_present": None,
        "metadata_limitations": [
            "ffprobe unavailable: average frame rate, VFR, audio, pixel/color, device, timestamp, and location metadata could not be fully audited"
        ],
    }
    capture.release()
    return result


def oriented_frame(frame: np.ndarray, rotation_degrees: float | int | None) -> np.ndarray:
    rotation = int(round(float(rotation_degrees or 0))) % 360
    if rotation == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if rotation == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if rotation == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def resize_max_dimension(frame: np.ndarray, maximum: int) -> np.ndarray:
    if maximum <= 0 or max(frame.shape[:2]) <= maximum:
        return frame
    scale = maximum / max(frame.shape[:2])
    return cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


def iter_sampled_frames(
    path: str | Path,
    *,
    sample_fps: float,
    max_dimension: int,
    start_sec: float = 0.0,
    end_sec: float | None = None,
) -> Iterator[tuple[int, float, np.ndarray]]:
    if sample_fps <= 0:
        raise ValueError("sample_fps must be positive.")
    metadata = probe_video(path)
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"OpenCV could not open video: {path}")
    if hasattr(cv2, "CAP_PROP_ORIENTATION_AUTO"):
        capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    source_fps = float(capture.get(cv2.CAP_PROP_FPS))
    if source_fps <= 0:
        raise RuntimeError("Video reports a non-positive frame rate.")
    start_frame = max(0, int(math.floor(start_sec * source_fps)))
    last_frame = int(math.ceil(end_sec * source_fps)) if end_sec is not None else None
    step = source_fps / sample_fps
    next_sample_frame = float(start_frame)
    capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    frame_index = start_frame
    try:
        while True:
            if last_frame is not None and frame_index > last_frame:
                break
            ok, frame = capture.read()
            if not ok:
                break
            if frame_index + 0.5 >= next_sample_frame:
                timestamp = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
                frame = oriented_frame(frame, metadata.get("rotation_degrees"))
                yield frame_index, timestamp, resize_max_dimension(frame, max_dimension)
                next_sample_frame += step
            frame_index += 1
    finally:
        capture.release()


@dataclass
class FrameMetrics:
    sharpness: float
    brightness: float
    underexposed_ratio: float
    overexposed_ratio: float
    motion: float | None
    duplicate_score: float | None
    rotation_deg: float | None
    feature_count: int


def calculate_metrics(frame: np.ndarray, previous: np.ndarray | None) -> FrameMetrics:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small = resize_max_dimension(gray, 640)
    sharpness = float(cv2.Laplacian(small, cv2.CV_64F).var())
    brightness = float(np.mean(small))
    under = float(np.mean(small <= 8))
    over = float(np.mean(small >= 247))
    detected_features = cv2.goodFeaturesToTrack(small, 500, 0.01, 8)
    feature_count = 0 if detected_features is None else len(detected_features)
    motion = duplicate = rotation = None
    if previous is not None:
        previous_gray = cv2.cvtColor(previous, cv2.COLOR_BGR2GRAY)
        previous_small = cv2.resize(previous_gray, (small.shape[1], small.shape[0]), interpolation=cv2.INTER_AREA)
        difference = cv2.absdiff(previous_small, small)
        motion = float(np.mean(difference) / 255.0)
        duplicate = float(1.0 - motion)
        points = cv2.goodFeaturesToTrack(previous_small, 300, 0.01, 8)
        if points is not None and len(points) >= 8:
            tracked, status, _ = cv2.calcOpticalFlowPyrLK(previous_small, small, points, None)
            valid = status.ravel().astype(bool)
            if valid.sum() >= 8:
                affine, _ = cv2.estimateAffinePartial2D(points[valid], tracked[valid], method=cv2.RANSAC)
                if affine is not None:
                    rotation = float(np.degrees(np.arctan2(affine[1, 0], affine[0, 0])))
    return FrameMetrics(sharpness, brightness, under, over, motion, duplicate, rotation, feature_count)
