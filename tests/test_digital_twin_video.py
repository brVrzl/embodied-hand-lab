from __future__ import annotations

import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from digital_twin.reconstruction.video import iter_sampled_frames, probe_video
from tools.digital_twin import prepare_video_frames


def make_video(path: Path, fps: int = 5, count: int = 10) -> None:
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (96, 64))
    assert writer.isOpened()
    for index in range(count):
        frame = np.zeros((64, 96, 3), dtype=np.uint8)
        cv2.circle(frame, (10 + 6 * index, 32), 8, (255, 255, 255), -1)
        cv2.putText(frame, str(index), (2, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 50), 1)
        writer.write(frame)
    writer.release()


def test_video_metadata_parsing_and_sampling(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    video = tmp_path / "fixture.avi"
    make_video(video)
    monkeypatch.setattr("digital_twin.reconstruction.video.shutil.which", lambda name: None)
    metadata = probe_video(video)
    assert metadata["width"] == 96
    assert metadata["height"] == 64
    assert metadata["nominal_frame_rate_fps"] == pytest.approx(5)
    sampled = list(iter_sampled_frames(video, sample_fps=2.5, max_dimension=96))
    assert 4 <= len(sampled) <= 6


def test_frame_manifest_generation(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    video = tmp_path / "fixture.avi"
    output = tmp_path / "frames"
    make_video(video, count=8)
    monkeypatch.setattr("digital_twin.reconstruction.video.shutil.which", lambda name: None)
    monkeypatch.setattr(sys, "argv", ["prepare_video_frames.py", "--input", str(video), "--output-dir", str(output), "--sample-fps", "2.5", "--min-sharpness", "0", "--rapid-motion-threshold", "1"])
    prepare_video_frames.main()
    with (output / "frame_manifest.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert {"frame_filename", "source_timestamp_sec", "source_frame_index", "sharpness_metric", "brightness_metric", "motion_metric", "accepted", "rejection_reason"} <= set(rows[0])
    assert (output / "frame_manifest.json").is_file()


def test_invalid_video_input() -> None:
    with pytest.raises(FileNotFoundError):
        probe_video("does-not-exist.MOV")
